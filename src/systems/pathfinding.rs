use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

use crate::components::robot::RobotId;
use crate::world::{grid::{Grid, Position}, World};


/// Cost penalty for a tile currently occupied by another robot.
/// Large-but-finite so that the algorithm routes around occupied tiles
/// when any alternative exists, but can still find a path through them
/// as a last resort rather than returning None.
const OCCUPIED_TILE_PENALTY: usize = 10_000;

/// A node in the A* open set, ordered by f_cost ascending (min-heap).
#[derive(Eq, PartialEq)]
struct AStarNode {
    f_cost:   usize,
    g_cost:   usize,
    position: Position,
}

// BinaryHeap in Rust is a max-heap; reversing the comparison makes it a min-heap.
impl Ord for AStarNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other.f_cost.cmp(&self.f_cost)
            .then_with(|| other.g_cost.cmp(&self.g_cost))
    }
}
impl PartialOrd for AStarNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Finds a path from `start` to `goal`.
///
/// Tiles in `occupied_positions` are penalized heavily, routing around them
/// without making them absolutely impassable
/// (adaptation of A* for dynamic obstacle avoidance)
///
/// Returns `Some(path)` where `path[0]` is the first step to take,
/// or `None` if no path exists at all (e.g. goal is fully surrounded by walls).
pub fn find_path(
    grid:                &Grid,
    occupied_positions:  &HashSet<Position>,
    start:               Position,
    goal:                Position,
) -> Option<Vec<Position>> {
    if start == goal {
        return Some(Vec::new());
    }

    let mut open_set:  BinaryHeap<AStarNode> = BinaryHeap::new();
    let mut came_from: HashMap<Position, Position> = HashMap::new();
    let mut g_costs:   HashMap<Position, usize> = HashMap::new();

    g_costs.insert(start, 0);
    open_set.push(AStarNode {
        f_cost:   start.manhattan_distance(&goal),
        g_cost:   0,
        position: start,
    });

    while let Some(current_node) = open_set.pop() {
        let current = current_node.position;

        if current == goal {
            return Some(reconstruct_path(&came_from, current));
        }

        let current_g = *g_costs.get(&current).unwrap_or(&usize::MAX);

        for neighbor in current.neighbors(grid.width, grid.height) {
            if !grid.is_passable(neighbor) {
                continue; // Hard wall — skip entirely
            }

            // Base cost: 1 per tile. Heavy penalty if another robot is there.
            let step_cost = if occupied_positions.contains(&neighbor) {
                1 + OCCUPIED_TILE_PENALTY
            } else {
                1
            };

            let tentative_g = current_g.saturating_add(step_cost);
            let known_g     = *g_costs.get(&neighbor).unwrap_or(&usize::MAX);

            if tentative_g < known_g {
                came_from.insert(neighbor, current);
                g_costs.insert(neighbor, tentative_g);
                let h = neighbor.manhattan_distance(&goal);
                open_set.push(AStarNode {
                    f_cost:   tentative_g + h,
                    g_cost:   tentative_g,
                    position: neighbor,
                });
            }
        }
    }

    None // No path exists
}

/// Walks `came_from` backward from the goal to produce a start→goal path,
/// with the start position excluded (it's where the robot already is).
fn reconstruct_path(came_from: &HashMap<Position, Position>, goal: Position) -> Vec<Position> {
    let mut path    = vec![goal];
    let mut current = goal;
    while let Some(&prev) = came_from.get(&current) {
        path.push(prev);
        current = prev;
    }
    path.pop(); // remove start position
    path.reverse();
    path
}

/// Recalculates planned_path for every routing robot each tick.
///
/// Runs every tick so robots continuously reroute around each other as they
/// move, implementing the dynamic obstacle avoidance.
///
/// Uses a three-phase pattern to satisfy Rust's borrow checker:
///   Phase 1 — read all robot positions and routing tasks (immutable borrow)
///   Phase 2 — compute new paths (no borrow of world needed)
///   Phase 3 — write new paths back to robots (mutable borrow)
pub fn run(world: &mut World) {
    // Phase 1: snapshot all robot positions as obstacles, and collect tasks
    let all_robot_positions: HashSet<Position> = world.robots
        .values()
        .map(|r| r.position)
        .collect();

    struct RoutingTask {
        robot_id:   RobotId,
        start:      Position,
        goal:       Position,
        own_pos:    Position,
    }

    let mut tasks: Vec<RoutingTask> = world.robots
        .values()
        .filter(|r| r.destination.is_some() && r.is_routing())
        .map(|r| RoutingTask {
            robot_id: r.id,
            start:    r.position,
            goal:     r.destination.unwrap(),
            own_pos:  r.position,
        })
        .collect();
    tasks.sort_by_key(|t| t.robot_id.0);

    // Phase 2: compute paths — each robot excludes its own tile from obstacles
    let new_paths: Vec<(RobotId, Vec<Position>)> = tasks
        .into_iter()
        .filter_map(|task| {
            // The robot should not treat its own current tile as an obstacle.
            let mut obstacles = all_robot_positions.clone();
            obstacles.remove(&task.own_pos);

            find_path(&world.grid, &obstacles, task.start, task.goal)
                .map(|path| (task.robot_id, path))
        })
        .collect();

    // Phase 3: apply
    for (id, path) in new_paths {
        world.robots.get_mut(&id).unwrap().planned_path = path;
    }
}
