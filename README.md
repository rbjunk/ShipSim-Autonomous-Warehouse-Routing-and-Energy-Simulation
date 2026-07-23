# ShipSim-Autonomous-Warehouse-Routing-and-Energy-Simulation

Before you begin, ensure you have the following installed on your system:
*   **Rust** (latest stable via [rustup](https://rustup.rs/))

## Getting the Source

Clone the repository and install the dependencies:
```bash
git clone https://github.com/rbjunk/ShipSim-Autonomous-Warehouse-Routing-and-Energy-Simulation.git
cd ShipSim-Autonomous-Warehouse-Routing-and-Energy-Simulation
```

## Running the Simulation
```bash
cargo run
```

## Expected Output
Text printed to the console detailing the current state of the simulation every 100 ticks.
<br>
[Tick x] completed: y active :z pending: a robots - idle: b routing: c
<br> Example: [Tick    100]  completed:   12  active:  5  pending: 36  robots — idle: 0  routing: 5

If Debug mode is enabled in ```config.toml``` then a grid showing the current state of all agents will also be displayed.  
Two files ```summary.txt``` and ```metrics.csv``` will be generated in the root project folder under output\
