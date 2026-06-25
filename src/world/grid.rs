/// A 2D coordinate on the warehouse grid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Position {
    pub x: usize,
    pub y: usize,
}

impl Position {
    pub fn new(x: usize, y: usize) -> Self {
        Position { x, y }
    }

    /// Manhattan distance to another position.
    /// Used as the A* heuristic — admissible because diagonal movement is not allowed.
    pub fn manhattan_distance(&self, other: &Position) -> usize {
        self.x.abs_diff(other.x) + self.y.abs_diff(other.y)
    }

    /// All orthogonally adjacent tiles (N, S, E, W) that are within grid bounds.
    pub fn neighbors(&self, grid_width: usize, grid_height: usize) -> Vec<Position> {
        let mut result = Vec::with_capacity(4);
        if self.x > 0               { result.push(Position::new(self.x - 1, self.y)); }
        if self.x + 1 < grid_width  { result.push(Position::new(self.x + 1, self.y)); }
        if self.y > 0               { result.push(Position::new(self.x, self.y - 1)); }
        if self.y + 1 < grid_height { result.push(Position::new(self.x, self.y + 1)); }
        result
    }
}

/// What a single grid tile represents in the warehouse.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TileKind {
    /// Open floor — robots can enter freely
    Floor,
    /// Permanent wall or obstacle — always impassable
    Wall,
    /// A shelf location; robots navigate here to pick up an order item
    ShelfLocation,
    /// A charging station tile (static entity; charging logic is out of scope for M2)
    ChargingStation,
    /// The drop-off zone where completed orders are delivered
    DispatchZone,
}

/// The 2D warehouse grid. Holds only the static layout of the warehouse.
/// Robot positions are NOT stored here — they live on the Robot component.
#[derive(Debug)]
pub struct Grid {
    pub width:  usize,
    pub height: usize,
    tiles: Vec<TileKind>,
}

impl Grid {
    /// Creates a grid where every tile is Floor.
    pub fn new_empty(width: usize, height: usize) -> Self {
        Grid {
            width,
            height,
            tiles: vec![TileKind::Floor; width * height],
        }
    }

    pub fn get(&self, pos: Position) -> TileKind {
        self.tiles[pos.y * self.width + pos.x]
    }

    pub fn set(&mut self, pos: Position, kind: TileKind) {
        self.tiles[pos.y * self.width + pos.x] = kind;
    }

    /// Returns true if the tile type allows robot entry.
    /// Does NOT check for other robots — that is the pathfinding system's job.
    pub fn is_passable(&self, pos: Position) -> bool {
        !matches!(self.get(pos), TileKind::Wall)
    }

    /// Collects all positions of a given tile type.
    pub fn positions_of_kind(&self, kind: TileKind) -> Vec<Position> {
        (0..self.height)
            .flat_map(|y| (0..self.width).map(move |x| Position::new(x, y)))
            .filter(|&pos| self.get(pos) == kind)
            .collect()
    }
}