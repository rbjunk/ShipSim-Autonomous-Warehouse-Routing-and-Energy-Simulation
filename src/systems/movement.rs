use crate::components::robot::RobotId;
use crate::world::{grid::Position, World};

/// Emitted when a robot takes its final step and reaches its destination.
/// Processed by simulation.rs::handle_arrivals() to trigger state transitions.
#[derive(Debug)]
pub struct ArrivalEvent {
    pub robot_id:   RobotId,
    pub arrived_at: Position,
}

/// Advances every routing robot exactly one step along its planned path.
///
/// Returns arrival events for any robot that reached its destination this tick.
/// State transitions are handled by the caller so this system stays focused
/// on movement only.
pub fn run(world: &mut World) -> Vec<ArrivalEvent> {
    let mut arrivals: Vec<ArrivalEvent> = Vec::new();
    let robot_ids: Vec<RobotId> = world.robots.keys().copied().collect();

    for id in robot_ids {
        let robot = world.robots.get_mut(&id).unwrap();

        if !robot.is_routing() {
            continue;
        }

        match robot.planned_path.first().copied() {
            Some(next_pos) => {
                robot.position = next_pos;
                robot.planned_path.remove(0);

                if robot.planned_path.is_empty() {
                    arrivals.push(ArrivalEvent { robot_id: id, arrived_at: next_pos });
                }
            }
            None => {
                // Path is empty but robot is still routing — treat as arrival so
                // the state machine can recover (e.g. if A* found no path last tick).
                arrivals.push(ArrivalEvent { robot_id: id, arrived_at: robot.position });
            }
        }
    }

    arrivals
}
