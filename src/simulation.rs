use rand::SeedableRng;
use rand::rngs::StdRng;

use crate::config::Config;
use crate::components::robot::RobotState;
use crate::debug;
use crate::metrics::{self, TickMetrics};
use crate::setup;
use crate::systems::{charging, energy, movement, order_generator, pathfinding, scheduler};
use crate::world::World;

/// Owns everything that spans the full simulation run.
pub struct Simulation {
    world:  World,
    config: Config,
    rng:    StdRng,
    metrics_log:  Vec<TickMetrics>,
    /// Tracks whether we have already fired the one-shot stuck dump so we
    /// don't spam it every tick once a robot becomes stuck.
    stuck_dumped: bool,
}

impl Simulation {
    pub fn new(config: Config) -> Self {
        let mut rng = match config.sim.random_seed {
            Some(seed) => StdRng::seed_from_u64(seed),
            None       => StdRng::from_entropy(),
        };
        let world = setup::build_world(&config, &mut rng);
        Simulation {
            world,
            config,
            rng,
            metrics_log:  Vec::new(),
            stuck_dumped: false, }
    }

    /// Runs the simulation for max_ticks and prints a final summary.
    pub fn run(&mut self) {
        println!("ShipSim - {} robots, {}×{} grid, {} ticks",
                 self.config.world.num_robots,
                 self.config.world.width,
                 self.config.world.height,
                 self.config.sim.max_ticks,
        );
        println!("{:-<60}", "");

        for _ in 0..self.config.sim.max_ticks {
            self.run_tick();
        }

        println!("{:-<60}", "");
        println!("Simulation complete at tick {}.", self.world.tick);
        println!("  Orders completed : {}", self.world.completed_orders.len());
        println!("  Orders active    : {}", self.world.active_orders.len());
        println!("  Orders pending   : {}", self.world.pending_orders.len());

        if !self.world.completed_orders.is_empty() {
            let avg_fulfillment: f64 = self.world.completed_orders
                .iter()
                .filter_map(|o| o.fulfillment_time())
                .map(|t| t as f64)
                .sum::<f64>()
                / self.world.completed_orders.len() as f64;
            println!("  Avg fulfillment  : {:.1} ticks", avg_fulfillment);
        }

        let csv_path = self.config.sim.output_csv.clone();
        match metrics::csv_writer::write_csv(&csv_path, &self.metrics_log) {
            Ok(_)  => println!("Metrics written to {}", csv_path),
            Err(e) => eprintln!("Failed to write metrics CSV: {}", e),
        }
    }

    // The Tick Loop
    // Systems run in this fixed order every tick:
    //
    // order_generator - inject new orders
    // scheduler       - assign pending orders to idle robots
    // pathfinding     - (re)compute routes around current obstacles
    // movement        - advance each robot one step; collect arrival events
    // handle_arrivals - state transitions triggered by arrivals
    // energy          - drains battery for robots that moved
    // charging        - charges robots at stations, advances station queues
    // metrics         - records this tick's snapshot
    // debug           - optional debug output

    fn run_tick(&mut self) {
        order_generator::run(&mut self.world, &self.config, &mut self.rng);
        scheduler::run(&mut self.world, &self.config);
        pathfinding::run(&mut self.world);

        let arrivals = movement::run(&mut self.world);
        self.handle_arrivals(arrivals);

        energy::run(&mut self.world, &self.config);
        charging::run(&mut self.world, &self.config);

        if self.world.tick % self.config.sim.print_every == 0 {
            self.print_status();
        }

        let tick_metrics = metrics::collect(&self.world);
        self.metrics_log.push(tick_metrics);

        self.maybe_print_debug();

        self.world.advance_tick();
    }

    /// Handles state transitions when a robot finishes a step of its journey.
    ///
    /// This is where state transitions happen: a robot arriving at its pickup
    /// location transitions to RoutingToDropoff; a robot arriving at the dispatch
    /// zone becomes Idle; a robot arriving at a station joins the charge queue.
    ///
    /// Keeping transitions here (not inside movement.rs) means the movement
    /// system stays focused on a single responsibility: moving robots one step.
    fn handle_arrivals(&mut self, arrivals: Vec<movement::ArrivalEvent>) {
        for event in arrivals {
            // Clone the state so we release the immutable borrow before mutating
            let robot_state = self.world.robots[&event.robot_id].state.clone();

            match robot_state {
                RobotState::RoutingToPickup { order_id } => {
                    // Robot reached the shelf;  pick up item and head to dispatch
                    let robot = self.world.robots.get_mut(&event.robot_id).unwrap();
                    robot.is_carrying_payload = true;
                    robot.state       = RobotState::RoutingToDropoff { order_id };
                    robot.destination = Some(self.world.dispatch_position);
                }

                RobotState::RoutingToDropoff { order_id } => {
                    // Robot reached dispatch — deliver item, become idle
                    self.world.complete_order(order_id);
                    let robot = self.world.robots.get_mut(&event.robot_id).unwrap();
                    robot.is_carrying_payload = false;
                    robot.state       = RobotState::Idle;
                    robot.destination = None;
                }

                RobotState::RoutingToCharge { station_id } => {
                    // Robot has physically arrived at the station.
                    // Change state to Waiting and add it to the queue.
                    // The charging system will pull it into the slot when one is free.
                    let robot = self.world.robots.get_mut(&event.robot_id).unwrap();
                    robot.state        = RobotState::WaitingToCharge { station_id };
                    robot.destination  = None;
                    robot.planned_path = Vec::new();

                    // Enqueue after physical arrival. The charging system must
                    // never try to charge a robot that hasn't reached the station yet
                    self.world.stations
                        .get_mut(&station_id)
                        .unwrap()
                        .enqueue(event.robot_id);
                }

                // No transition needed for these states
                RobotState::Idle
                | RobotState::Charging { .. }
                | RobotState::WaitingToCharge { .. } => {}
            }
        }
    }

    fn print_status(&self) {
        let idle    = self.world.robots.values().filter(|r| r.is_idle()).count();
        let routing = self.world.robots.values().filter(|r| r.is_routing()).count();
        println!(
            "[Tick {:>6}]  completed: {:>4}  active: {:>2}  pending: {:>2}  \
             robots — idle: {}  routing: {}",
            self.world.tick,
            self.world.completed_orders.len(),
            self.world.active_orders.len(),
            self.world.pending_orders.len(),
            idle,
            routing,
        );
    }

    fn maybe_print_debug(&mut self) {
        let dc = &self.config.debug;
        if !dc.enabled {
            return;
        }

        let tick = self.world.tick;

        // One-shot stuck dump: fires the first time any robot has no path
        if dc.dump_on_stuck && !self.stuck_dumped && debug::has_stuck_robots(&self.world) {
            println!("\n\x1b[1;31m!!! STUCK ROBOT DETECTED AT TICK {} !!!\x1b[0m\n", tick);
            debug::print_grid(&self.world);
            println!();
            debug::print_status(&self.world);
            println!();
            self.stuck_dumped = true;
        }

        // Periodic grid print
        if dc.grid_every > 0 && tick % dc.grid_every == 0 {
            println!("\n─── Grid at tick {} ───────────────────────────", tick);
            debug::print_grid(&self.world);
        }

        // Periodic status print
        if dc.status_every > 0 && tick % dc.status_every == 0 {
            println!("\n─── Status at tick {} ──────────────────────────", tick);
            debug::print_status(&self.world);
        }
    }
}
