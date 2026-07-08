mod components;
mod config;
mod setup;
mod simulation;
mod systems;
mod world;
mod debug;
mod metrics;

use config::Config;
use simulation::Simulation;

fn main() {
    let config = Config::from_file("config.toml").unwrap_or_else(|e| {
        eprintln!("Could not load config.toml ({}), using built-in defaults.", e);
        Config::default()
    });

    let mut sim = Simulation::new(config);
    sim.run();
}