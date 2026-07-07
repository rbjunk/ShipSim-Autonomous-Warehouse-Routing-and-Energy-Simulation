use serde::{Deserialize, Serialize};
use std::fs;

/// Top-level simulation configuration loaded from config.toml.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    pub sim:    SimConfig,
    pub world:  WorldConfig,
    pub orders: OrderConfig,
    pub robot:  RobotConfig,
    pub debug:  DebugConfig,
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotConfig {
    pub battery_capacity:           f64,
    /// Below this level the robot must abandon its task and seek a charger
    pub battery_critical_threshold: f64,
    pub robot_mass_kg:              f64,
    pub payload_mass_kg:            f64,
    /// µ - mechanical efficiency coefficient in E = (m_robot + m_payload) * d * µ
    pub efficiency_coefficient:     f64,
    pub charge_rate_per_tick:       f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DebugConfig {
    /// Master switch:  set to true to enable any debug output
    pub enabled: bool,

    /// Print the ASCII grid every N ticks (0 = never)
    pub grid_every: u64,

    /// Print the full robot/station status every N ticks (0 = never)
    pub status_every: u64,

    /// Automatically dump both views the first time a stuck robot is detected
    /// (a robot that is routing but A* found no path). Useful for catching the
    /// freeze mid-run without knowing which tick it will happen.
    pub dump_on_stuck: bool,
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
            robot: RobotConfig {
                battery_capacity:           100.0,
                battery_critical_threshold: 20.0,
                robot_mass_kg:              50.0,
                payload_mass_kg:            10.0,
                efficiency_coefficient:     0.01,
                charge_rate_per_tick:       5.0,
            },
            debug: DebugConfig {
                enabled:       false,
                grid_every:    0,
                status_every:  0,
                dump_on_stuck: true,
            },
        }
    }
}