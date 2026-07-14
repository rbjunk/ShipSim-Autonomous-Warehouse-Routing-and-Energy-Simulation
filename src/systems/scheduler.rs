use crate::components::order::OrderStatus;
use crate::components::robot::{RobotId, RobotState};
use crate::components::station::StationId;
use crate::config::Config;
use crate::world::World;

/// Runs two scheduling decisions each tick:
///   1. Triage: Send any robot below the critical battery threshold to the
///      nearest charging station, abandoning its current task if needed.
///   2. Assign: Pair each idle robot with the nearest pending order
///      (greedy nearest-neighbor heuristic).
///
/// Triage runs before assignment so that a newly-idle robot (just finished
/// an order) is caught by triage if its battery is low, rather than being
/// immediately assigned another task it can't complete.
pub fn run(world: &mut World, config: &Config) {
    run_battery_triage(world, config);
    run_order_assignment(world);
}

fn run_battery_triage(world: &mut World, config: &Config) {
    let threshold = config.robot.battery_critical_threshold;

    // Collect robots that need to recharge but aren't already heading there
    let mut robots_needing_charge: Vec<RobotId> = world.robots
        .values()
        .filter(|r| {
            r.needs_recharge(threshold)
                && !matches!(
                    r.state,
                    RobotState::Charging { .. }
                        | RobotState::WaitingToCharge { .. }
                        | RobotState::RoutingToCharge { .. }
                )
        })
        .map(|r| r.id)
        .collect();
    robots_needing_charge.sort_by_key(|id| id.0);
    for robot_id in robots_needing_charge {
        let robot_pos = world.robots[&robot_id].position;

        // If the robot was mid-task, find out which order it was on so we
        // can return that order to the pending queue.
        // We extract the order_id first so the immutable borrow of world.robots ends
        // before we mutably borrow world.active_orders below.
        let abandoned_order_id = match &world.robots[&robot_id].state {
            RobotState::RoutingToPickup  { order_id }
            | RobotState::RoutingToDropoff { order_id } => Some(*order_id),
            _ => None,
        };

        // Return the abandoned order to the pending queue
        if let Some(order_id) = abandoned_order_id {
            if let Some(pos) = world.active_orders.iter().position(|o| o.id == order_id) {
                let mut order   = world.active_orders.remove(pos);
                order.status    = OrderStatus::Pending;
                order.assigned_to = None;
                world.pending_orders.push(order);
            }
        }

        // Find the nearest station, weighted by queue length to spread load
        let best_station: Option<(StationId, usize)> = world.stations
            .values()
            .map(|s| (s.id, s.position.manhattan_distance(&robot_pos) + s.total_load() * 2))
            .min_by_key(|&(_, cost)| cost);

        if let Some((station_id, _)) = best_station {
            let station_pos = world.stations[&station_id].position;
            let robot       = world.robots.get_mut(&robot_id).unwrap();

            robot.is_carrying_payload = false;
            robot.state               = RobotState::RoutingToCharge { station_id };
            robot.destination         = Some(station_pos);

            // NOTE: We do NOT enqueue the robot here. The robot is only added
            // to station.queue when it physically arrives (in simulation.rs
            // handle_arrivals). This prevents the charging system from trying
            // to charge a robot that hasn't arrived yet.
        }
    }
}

fn run_order_assignment(world: &mut World) {
    // Collect idle robots and pending orders (we need owned IDs to avoid
    // borrow conflicts when we mutate world below)
    let mut idle_robots: Vec<RobotId> = world.robots
        .values()
        .filter(|r| r.is_idle())
        .map(|r| r.id)
        .collect();
    idle_robots.sort_by_key(|id| id.0);
    if idle_robots.is_empty() || world.pending_orders.is_empty() {
        return;
    }

    // Greedily assign: for each idle robot, find the closest pending order.
    // Marking orders as we go prevents two robots from being assigned the same order.
    let mut assigned_order_indices: Vec<usize> = Vec::new();

    for robot_id in idle_robots {
        let robot_pos = world.robots[&robot_id].position;

        // Find the pending order with the shortest Manhattan distance to this robot,
        // excluding orders already assigned in this tick's loop
        let best_order = world.pending_orders
            .iter()
            .enumerate()
            .filter(|(idx, _)| !assigned_order_indices.contains(idx))
            .min_by_key(|(_, order)| order.pickup_location.manhattan_distance(&robot_pos));

        if let Some((order_idx, order)) = best_order {
            let order_id       = order.id;
            let pickup_location = order.pickup_location;
            assigned_order_indices.push(order_idx);

            // Update the robot: send it to the pickup location
            let robot = world.robots.get_mut(&robot_id).unwrap();
            robot.state       = RobotState::RoutingToPickup { order_id };
            robot.destination = Some(pickup_location);

            // Move the order from pending to active
            world.assign_order(order_id, robot_id);
        }
    }
}