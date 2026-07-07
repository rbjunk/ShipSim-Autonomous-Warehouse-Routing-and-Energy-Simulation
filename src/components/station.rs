use std::collections::VecDeque;
use crate::world::grid::Position;
use crate::components::robot::RobotId;

/// Type-safe charging station identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct StationId(pub u32);

/// A charging station in the warehouse.
/// One robot can charge at a time; all others wait in a FIFO queue.
#[derive(Debug, Clone)]
pub struct ChargingStation {
    pub id:       StationId,
    pub position: Position,

    /// The robot currently occupying the charging slot (None = station is free)
    pub charging_robot: Option<RobotId>,

    /// Robots that have arrived and are waiting for a free slot, in arrival order
    pub queue: VecDeque<RobotId>
}

impl ChargingStation {
    pub fn new(id: StationId, position: Position) -> Self {
        ChargingStation {
            id,
            position,
            charging_robot: None,
            queue: VecDeque::new(),
        }
    }

    /// True if no robot is currently using the charging slot.
    pub fn is_free(&self) -> bool {
        self.charging_robot.is_none()
    }

    /// Total number of robots waiting or charging here (for metrics and scheduler).
    pub fn total_load(&self) -> usize {
        self.queue.len() + if self.charging_robot.is_some() { 1 } else { 0 }
    }

    /// Adds a robot to the back of the waiting queue.
    pub fn enqueue(&mut self, robot_id: RobotId) {
        self.queue.push_back(robot_id);
    }

    /// If the station is free, pulls the next robot from the queue into the
    /// charging slot. Returns that robot's ID if one was started.
    pub fn try_advance_queue(&mut self) -> Option<RobotId> {
        if self.charging_robot.is_none() {
            if let Some(next) = self.queue.pop_front() {
                self.charging_robot = Some(next);
                return Some(next);
            }
        }
        None
    }

    /// Called when the robot currently charging has finished. Clears the slot.
    pub fn finish_charging(&mut self) {
        self.charging_robot = None;
    }
}
