use crate::world::grid::Position;

/// Type-safe charging station identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct StationId(pub u32);

/// A charging station in the warehouse.
///
/// For Milestone 2 this is a static entity only. it occupies a tile on the
/// grid and robots are aware of its position, but the charging queue and
/// charge-rate logic are out of scope until a later milestone.
#[derive(Debug, Clone)]
pub struct ChargingStation {
    pub id:       StationId,
    pub position: Position,
}

impl ChargingStation {
    pub fn new(id: StationId, position: Position) -> Self {
        ChargingStation { id, position }
    }
}
