pub mod grid;

use std::collections::HashMap;
use crate::components::{
    robot::{Robot, RobotId},
    station::{ChargingStation, StationId},
    order::{Order, OrderId, OrderStatus},
};
use grid::{Grid, Position};

/// The World is the single source of truth for all simulation state.
/// Every system function takes `&mut World` and reads or writes to this struct.
#[derive(Debug)]
pub struct World {
    pub grid:     Grid,
    pub robots:   HashMap<RobotId, Robot>,
    /// Charging stations exist as static entities; their charging logic is out of scope for M2.
    pub stations: HashMap<StationId, ChargingStation>,

    /// Orders waiting to be assigned to a robot
    pub pending_orders: Vec<Order>,
    /// Orders a robot is currently carrying out
    pub active_orders: Vec<Order>,
    /// Orders that have been fully delivered
    pub completed_orders: Vec<Order>,

    /// The single drop-off tile where robots deliver completed orders
    pub dispatch_position: Position,

    /// Auto-incrementing counter for unique OrderIds
    next_order_id: u32,

    pub tick: u64,
}

impl World {
    pub fn new(
        grid:              Grid,
        robots:            HashMap<RobotId, Robot>,
        stations:          HashMap<StationId, ChargingStation>,
        dispatch_position: Position,
    ) -> Self {
        World {
            grid,
            robots,
            stations,
            pending_orders:   Vec::new(),
            active_orders:    Vec::new(),
            completed_orders: Vec::new(),
            dispatch_position,
            next_order_id: 0,
            tick: 0,
        }
    }

    /// Creates a new order and places it in the pending queue.
    pub fn inject_order(&mut self, pickup_location: Position) {
        let id    = OrderId(self.next_order_id);
        self.next_order_id += 1;
        self.pending_orders.push(Order::new(id, pickup_location, self.tick));
    }

    /// Moves an order from pending to active and records which robot owns it.
    pub fn assign_order(&mut self, order_id: OrderId, robot_id: RobotId) {
        if let Some(pos) = self.pending_orders.iter().position(|o| o.id == order_id) {
            let mut order     = self.pending_orders.remove(pos);
            order.status      = OrderStatus::InProgress;
            order.assigned_to = Some(robot_id);
            self.active_orders.push(order);
        }
    }

    /// Moves an order from active to completed and stamps the completion tick.
    pub fn complete_order(&mut self, order_id: OrderId) {
        if let Some(pos) = self.active_orders.iter().position(|o| o.id == order_id) {
            let mut order          = self.active_orders.remove(pos);
            order.status           = OrderStatus::Completed;
            order.completed_at_tick = Some(self.tick);
            self.completed_orders.push(order);
        }
    }

    /// All shelf tile positions — used by the order generator to pick a destination.
    pub fn shelf_positions(&self) -> Vec<Position> {
        self.grid.positions_of_kind(grid::TileKind::ShelfLocation)
    }

    pub fn advance_tick(&mut self) {
        self.tick += 1;
    }
}
