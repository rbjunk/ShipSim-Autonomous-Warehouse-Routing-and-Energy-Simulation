use crate::components::robot::{RobotId, RobotState};
use crate::world::World;

/// Assigns pending orders to idle robots using a greedy nearest-neighbor heuristic.
///
/// Each idle robot is paired with the closest unassigned pending order
/// (measured by Manhattan distance). This runs every tick, so robots that
/// finish their current task become eligible for the next order immediately.
pub fn run(world: &mut World) {
    let idle_robots: Vec<RobotId> = world.robots
        .values()
        .filter(|r| r.is_idle())
        .map(|r| r.id)
        .collect();

    if idle_robots.is_empty() || world.pending_orders.is_empty() {
        return;
    }

    // Track which pending order indices have been claimed this tick so two
    // robots can't be assigned the same order in the same scheduler pass.
    let mut claimed_indices: Vec<usize> = Vec::new();

    for robot_id in idle_robots {
        let robot_pos = world.robots[&robot_id].position;

        // Find the nearest unclaimed pending order
        let best = world.pending_orders
            .iter()
            .enumerate()
            .filter(|(idx, _)| !claimed_indices.contains(idx))
            .min_by_key(|(_, order)| order.pickup_location.manhattan_distance(&robot_pos));

        if let Some((order_idx, order)) = best {
            let order_id        = order.id;
            let pickup_location = order.pickup_location;
            claimed_indices.push(order_idx);

            // Give the robot its destination and flip its state
            let robot       = world.robots.get_mut(&robot_id).unwrap();
            robot.state       = RobotState::RoutingToPickup { order_id };
            robot.destination = Some(pickup_location);

            // Move the order from pending to active
            world.assign_order(order_id, robot_id);
        }
    }
}