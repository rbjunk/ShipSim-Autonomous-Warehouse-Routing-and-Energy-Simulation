use std::collections::HashMap;
use rand::Rng;

use crate::config::Config;
use crate::components::{
    robot::{Robot, RobotId},
    station::{ChargingStation, StationId},
};
use crate::world::{
    World,
    grid::{Grid, Position, TileKind},
};

/// Builds the initial World from configuration.
///
/// Placement order matters — each step skips tiles already claimed:
///   1. Mark the dispatch zone
///   2. Place charging stations (evenly spaced, top row)
///   3. Scatter random wall/obstacle tiles
///   4. Scatter random shelf locations
///   5. Spawn robots near the dispatch zone
///
/// To swap in a different layout strategy (e.g. load from a file), only
/// this function needs to change — no simulation logic is touched.
pub fn build_world(config: &Config, rng: &mut impl Rng) -> World {
    let wc = &config.world;
    let mut grid = Grid::new_empty(wc.width, wc.height);

    // --- 1. Dispatch zone ---
    let dispatch_pos = Position::new(wc.dispatch_x, wc.dispatch_y);
    grid.set(dispatch_pos, TileKind::DispatchZone);

    // --- 2. Charging stations (evenly spaced along the top row) ---
    let mut stations: HashMap<StationId, ChargingStation> = HashMap::new();
    for (i, pos) in distribute_evenly(wc.num_charging_stations, wc.width, 0).into_iter().enumerate() {
        let id = StationId(i as u32);
        grid.set(pos, TileKind::ChargingStation);
        stations.insert(id, ChargingStation::new(id, pos));
    }

    // --- 3. Obstacle/wall tiles (randomly placed on remaining floor tiles) ---
    place_randomly(&mut grid, TileKind::Wall, wc.num_obstacles, rng);

    // --- 4. Shelf locations (randomly placed on remaining floor tiles) ---
    place_randomly(&mut grid, TileKind::ShelfLocation, wc.num_shelf_locations, rng);

    // --- 5. Robots (spread out from the dispatch zone along the bottom row) ---
    let mut robots: HashMap<RobotId, Robot> = HashMap::new();
    for i in 0..wc.num_robots {
        let id  = RobotId(i as u32);
        let pos = Position::new(
            (wc.dispatch_x + i).min(wc.width - 1),
            wc.dispatch_y,
        );
        robots.insert(id, Robot::new(id, pos));
    }

    World::new(grid, robots, stations, dispatch_pos)
}

/// Randomly scatters `count` tiles of a given type onto Floor tiles.
/// Retries up to 20× per tile to find a free spot; stops early if the
/// grid is too full to place all requested tiles.
fn place_randomly(grid: &mut Grid, kind: TileKind, count: usize, rng: &mut impl Rng) {
    let mut placed   = 0;
    let mut attempts = 0;
    let max_attempts = count * 20;

    while placed < count && attempts < max_attempts {
        let pos = Position::new(
            rng.gen_range(0..grid.width),
            rng.gen_range(1..grid.height), // row 0 reserved for stations
        );
        if grid.get(pos) == TileKind::Floor {
            grid.set(pos, kind);
            placed += 1;
        }
        attempts += 1;
    }
}

/// Returns `count` evenly-spaced positions along a given row.
fn distribute_evenly(count: usize, grid_width: usize, row: usize) -> Vec<Position> {
    if count == 0 { return Vec::new(); }
    let spacing = grid_width / (count + 1);
    (1..=count)
        .map(|i| Position::new((i * spacing).min(grid_width - 1), row))
        .collect()
}
