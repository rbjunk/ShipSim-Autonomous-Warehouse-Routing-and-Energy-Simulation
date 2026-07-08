pub mod csv_writer;

use crate::world::World;
use crate::components::robot::RobotState;

/// A snapshot of the simulation's key performance indicators for one tick.
#[derive(Debug, Clone)]
pub struct TickMetrics {
    pub tick: u64,

    /// Total orders completed so far (cumulative)
    pub total_orders_completed: usize,

    /// Orders completed in the last 100 ticks (throughput window)
    pub throughput_last_100_ticks: usize,

    /// Fraction of robots actively routing or picking up (not idle/charging)
    pub robot_utilization_pct: f64,

    /// Length of the longest charging station queue this tick
    pub max_charging_queue_length: usize,

    /// Number of robots currently idle
    pub idle_robot_count: usize,

    /// Number of robots currently routing to a charger or waiting to charge
    pub robots_in_charge_cycle: usize,

    /// Number of pending orders not yet assigned
    pub pending_order_count: usize,
}

/// Collects metrics from the current world state for this tick.
pub fn collect(world: &World) -> TickMetrics {
    let total_orders_completed = world.completed_orders.len();

    let throughput_last_100_ticks = world.completed_orders
        .iter()
        .filter(|o| {
            o.completed_at_tick
                .map(|t| world.tick.saturating_sub(100) <= t && t <= world.tick)
                .unwrap_or(false)
        })
        .count();

    let total_robots = world.robots.len() as f64;
    let active_robots = world.robots
        .values()
        .filter(|r| {
            matches!(
                r.state,
                RobotState::RoutingToPickup { .. } | RobotState::RoutingToDropoff { .. }
            )
        })
        .count() as f64;

    let robot_utilization_pct = if total_robots > 0.0 {
        (active_robots / total_robots) * 100.0
    } else {
        0.0
    };

    let max_charging_queue_length = world.stations
        .values()
        .map(|s| s.queue.len())
        .max()
        .unwrap_or(0);

    let idle_robot_count = world.robots
        .values()
        .filter(|r| r.is_idle())
        .count();

    let robots_in_charge_cycle = world.robots
        .values()
        .filter(|r| {
            matches!(
                r.state,
                RobotState::RoutingToCharge { .. }
                    | RobotState::WaitingToCharge { .. }
                    | RobotState::Charging { .. }
            )
        })
        .count();

    TickMetrics {
        tick: world.tick,
        total_orders_completed,
        throughput_last_100_ticks,
        robot_utilization_pct,
        max_charging_queue_length,
        idle_robot_count,
        robots_in_charge_cycle,
        pending_order_count: world.pending_orders.len(),
    }
}