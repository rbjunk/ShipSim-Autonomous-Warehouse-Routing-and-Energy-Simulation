use crate::components::robot::RobotState;
use crate::config::Config;
use crate::world::World;

/// Runs two operations on every charging station each tick:
///   1. Charge the robot currently in the charging slot.
///   2. If the slot just opened (or was already free), pull the next robot
///      from the queue into the slot.
///
/// A robot is considered "done charging" when its battery hits full capacity.
/// At that point it leaves the station and returns to Idle.

pub fn run(world: &mut World, config: &Config) {
    let station_ids: Vec<_> = world.stations.keys().copied().collect();

    for station_id in station_ids {
        // Step 1: Charge the current robot, if any
        let maybe_charging_robot = world.stations[&station_id].charging_robot;

        if let Some(robot_id) = maybe_charging_robot {
            let robot = world.robots.get_mut(&robot_id).unwrap();

            robot.battery_level = (robot.battery_level + config.robot.charge_rate_per_tick)
                .min(config.robot.battery_capacity);

            let is_full = robot.battery_level >= config.robot.battery_capacity;

            if is_full {
                //free up the charging slot and send it idle
                robot.state       = RobotState::Idle;
                robot.destination = None;

                world.stations.get_mut(&station_id).unwrap().finish_charging();
            }
        }

        // Step 2: Advance the queue if the slot is now free
        if let Some(next_robot_id) = world.stations
            .get_mut(&station_id)
            .unwrap()
            .try_advance_queue()
        {
            let robot = world.robots.get_mut(&next_robot_id).unwrap();
            robot.state = RobotState::Charging { station_id };
            robot.destination   = None;
            robot.planned_path  = Vec::new();
        }
    }
}