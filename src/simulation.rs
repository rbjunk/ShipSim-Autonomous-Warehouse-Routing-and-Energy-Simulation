use rand::SeedableRng;
use rand::rngs::StdRng;

use crate::config::Config;
use crate::components::robot::RobotState;
use crate::setup;
use crate::systems::{movement, order_generator, pathfinding, scheduler};
use crate::world::World;

/// Owns everything that spans the full simulation run.
pub struct Simulation {
    world:  World,
    config: Config,
    rng:    StdRng,
}

impl Simulation {
    pub fn new(config: Config) -> Self {
        let mut rng = match config.sim.random_seed {
            Some(seed) => StdRng::seed_from_u64(seed),
            None       => StdRng::from_entropy(),
        };
        let world = setup::build_world(&config, &mut rng);
        Simulation { world, config, rng }
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
    }

    // The Tick Loop
    // Systems run in this fixed order every tick:
    //
    // order_generator - inject new orders
    // scheduler       - assign pending orders to idle robots
    // pathfinding     - (re)compute routes around current obstacles
    // movement        - advance each robot one step; collect arrival events
    // handle_arrivals - state transitions triggered by arrivals

    fn run_tick(&mut self) {
        order_generator::run(&mut self.world, &self.config, &mut self.rng);
        scheduler::run(&mut self.world);
        pathfinding::run(&mut self.world);

        let arrivals = movement::run(&mut self.world);
        self.handle_arrivals(arrivals);

        if self.world.tick % self.config.sim.print_every == 0 {
            self.print_status();
        }

        self.world.advance_tick();
    }

    /// Handles state transitions when a robot finishes a step of its journey.
    ///
    /// Keeping transitions here (not inside movement.rs) means the movement
    /// system stays focused on a single responsibility: moving robots one step.
    fn handle_arrivals(&mut self, arrivals: Vec<movement::ArrivalEvent>) {
        for event in arrivals {
            // Clone the state so we release the immutable borrow before mutating
            let state = self.world.robots[&event.robot_id].state.clone();

            match state {
                RobotState::RoutingToPickup { order_id } => {
                    // Robot reached the shelf — pick up item, head to dispatch
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

                // Idle robots don't move, so this branch shouldn't be reached
                RobotState::Idle => {}
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
}
