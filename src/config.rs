use serde::{Deserialize, Serialize};
use std::fs;

/// Top-level simulation configuration loaded from config.toml.
/// Battery and energy parameters are out of scope for Milestone 2.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    pub sim:    SimConfig,
    pub world:  WorldConfig,
    pub orders: OrderConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimConfig {
    pub max_ticks:   u64,
    /// Fixed seed → fully reproducible run. Remove from config for random behavior.
    pub random_seed: Option<u64>,
    /// Print a one-line status summary to stdout every this many ticks.
    pub print_every: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorldConfig {
    pub width:                 usize,
    pub height:                usize,
    pub num_robots:            usize,
    pub num_charging_stations: usize,
    pub num_shelf_locations:   usize,
    /// Impassable wall tiles scattered across the grid
    pub num_obstacles:         usize,
    /// Drop-off point where robots deliver completed orders
    pub dispatch_x:            usize,
    pub dispatch_y:            usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OrderConfig {
    /// λ for the Poisson arrival process — average orders generated per tick
    pub arrival_rate_lambda: f64,
}

impl Config {
    pub fn from_file(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let contents = fs::read_to_string(path)?;
        let config: Config = toml::from_str(&contents)?;
        Ok(config)
    }
}

impl Default for Config {
    fn default() -> Self {
        Config {
            sim: SimConfig {
                max_ticks:   2_000,
                random_seed: Some(42),
                print_every: 100,
            },
            world: WorldConfig {
                width:                 20,
                height:                20,
                num_robots:            5,
                num_charging_stations: 2,
                num_shelf_locations:   20,
                num_obstacles:         15,
                dispatch_x:            0,
                dispatch_y:            0,
            },
            orders: OrderConfig {
                arrival_rate_lambda: 0.5,
            },
        }
    }
}