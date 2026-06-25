use crate::world::grid::Position;
use crate::components::order::OrderId;

/// Type-safe robot identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RobotId(pub u32);

/// The three states a robot can be in
/// Charging states are out of scope and will be added later.
#[derive(Debug, Clone, PartialEq)]
pub enum RobotState {
    /// Waiting for the scheduler to assign a task
    Idle,
    /// Navigating to the shelf to pick up an order item
    RoutingToPickup { order_id: OrderId },
    /// Item collected; navigating to the dispatch zone to deliver it
    RoutingToDropoff { order_id: OrderId },
}

/// All data for a single autonomous robot.
#[derive(Debug, Clone)]
pub struct Robot {
    pub id:       RobotId,
    pub position: Position,
    pub state:    RobotState,

    /// Where the robot is ultimately trying to reach.
    /// The pathfinding system reads this every tick to compute planned_path.
    pub destination: Option<Position>,

    /// Step-by-step route to destination, computed by A* each tick.
    /// planned_path[0] is the very next tile to step onto.
    /// When this is empty the robot has arrived at destination.
    pub planned_path: Vec<Position>,

    /// True while the robot is carrying an order item (picked up, not yet delivered).
    pub is_carrying_payload: bool,
}

impl Robot {
    pub fn new(id: RobotId, position: Position) -> Self {
        Robot {
            id,
            position,
            state:               RobotState::Idle,
            destination:         None,
            planned_path:        Vec::new(),
            is_carrying_payload: false,
        }
    }

    /// True if the robot can accept a new task from the scheduler.
    pub fn is_idle(&self) -> bool {
        self.state == RobotState::Idle
    }

    /// True if the robot is actively navigating somewhere.
    pub fn is_routing(&self) -> bool {
        matches!(
            self.state,
            RobotState::RoutingToPickup { .. } | RobotState::RoutingToDropoff { .. }
        )
    }
}

