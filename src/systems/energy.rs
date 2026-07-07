use crate::config::Config;
use crate::world::World;


/// Drains the battery of every robot that moved this tick.
///
/// Implements the linear energy model from the paper:
///   E = (m_robot + m_payload) * d * µ
///
/// Where:
///   m_robot = robot body mass (kg)
///   m_payload = mass of the item being carried (0 if not carrying anything)
///   d = distance traveled this tick (always 1 tile = 1 unit of distance)
///   µ = mechanical efficiency coefficient from config
///
/// Also flags any robot whose battery has dropped below the critical threshold,
/// which the scheduler will pick up next tick to send them to a charger.
pub fn run(world: &mut World, config: &Config) {
    let robot_config = &config.robot;
    let distance_per_tick: f64 = 1.0;

    for robot in world.robots.values_mut() {
        if !robot.is_routing() {
            continue; // Idle or charging robots don't consume energy
        }

        let payload_mass = if robot.is_carrying_payload {
            robot_config.payload_mass_kg
        }
        else {
            0.0
        };

        let energy_consumed = (robot_config.robot_mass_kg + payload_mass) * distance_per_tick * robot_config.efficiency_coefficient;
        robot.battery_level = (robot.battery_level - energy_consumed).max(0.0);
    }
}

/// Calculates the energy cost for a single move, given payload status.
/// Exposed as a utility so the scheduler can estimate remaining range.
pub fn energy_per_step(config: &Config, carrying_payload: bool) -> f64 {
    let payload_mass = if carrying_payload { config.robot.payload_mass_kg } else { 0.0 };
    (config.robot.robot_mass_kg + payload_mass) * config.robot.efficiency_coefficient
}

/// Estimates how many more steps a robot can take before hitting the
/// critical battery threshold, given its current battery and payload state.
pub fn steps_remaining(config: &Config, current_battery: f64, carrying_payload: bool) -> f64 {
    let available = (current_battery - config.robot.battery_critical_threshold).max(0.0);
    let cost_per_step = energy_per_step(config, carrying_payload);
    if cost_per_step == 0.0 { return f64::INFINITY; }
    available / cost_per_step
}