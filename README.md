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

## Project Status
Currently, all of the basic functionality of the simulation is implemented including:  
- main discrete-event simulation loop  
- 2D grid environment
- Spawning of static entities (Charging stations, layout obstacles (walls) and dynamic entities (Robots))
- Basic A* pathfinding and collision free movement
- Poisson process order generation
- Greedy task scheduler for assigning orders to the nearest idle robot

## Architecture Overview

### Components:
- order: Has a unique id and a state (Pending, in progress, completed). When created it is assigned
a pickup location and documents the tick that the order was created and when it was completed (if it is successfully fulfilled).
- robot: Has a unique id and a state (Idle, Routing to Pickup, Routing to drop-off). Sits idle until assigned an order  
- station: Static charging station entity. Currently does not perform any functions but will eventually perform queueing and charging of robots.
### Systems:  
- movement: Responsible for moving each robot. Moves them one step each tick and if a robot completes and order
on the same tick, then it will assign a new task on the same tick.
- order_generator: Generates the orders for the simulation using a Poisson arrival process. Each newly generated order is given a randomly selected shelf tile where it needs to be picked up.
- pathfinding: Implements the modified A* pathfinding algorithm to find a robots shortest path from its start to its goal.
Returns the optimal path in reverse for the robot to walk it. Also recalculates (reroutes) the robots every tick so they reroute around each other, implementing dynamic object avoidance.
- scheduler: Pairs idle robots with the closest unassigned pending order (greedy nearest-neighbor heuristic). Runs every tick.
### World:
- grid: contains the definitions for positioning, tile kinds (Floor, Wall, ShelfLocation, ChargingStation, DispatchZone).
Also hold functions for retrieving whether a certain tile is passable or not, what tile is located at what position etc.
### Simulation:
- main: entry point and starts the simulation.
- simulation: Owns all objects that span the entire simulation (World, Config, RNG).
Loads a config to create the world and handles a loop of the simulation (state transitions, handling arrivals, running all systems) for the specified number of ticks.
Also prints all data to the console, but this will be removed in the next milestone in favor of a CSV output.
### Config and Setup:
Config defines the data to be loaded from a configuration file (or default if one is not provided) and setup actually implements this config file.

## Architecture Changes
Some of the architecture was changed as I was building/writing the code for this milestone. Smalls details like velocity of robots was no longer found to be needed.  
Generally the architecture has stayed the same though. Each component holds its own state and receives state changes from the systems, which are directed by the simulation world itself.
The main simulation loop that was proposed in milestone 1 has stayed the same.