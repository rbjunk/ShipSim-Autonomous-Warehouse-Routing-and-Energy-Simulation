use rand::Rng;
use rand_distr::{Distribution, Poisson};

use crate::config::Config;
use crate::world::World;

/// Injects new orders into the pending queue using a Poisson arrival process.
///
/// The number of orders generated each tick is drawn from Poisson(λ), where λ
/// is `config.orders.arrival_rate_lambda`. This produces the bursty, irregular
/// arrival pattern of real fulfillment centers rather than a constant stream.
///
/// Each generated order targets a randomly selected shelf tile.
pub fn run(world: &mut World, config: &Config, rng: &mut impl Rng) {
    let lambda = config.orders.arrival_rate_lambda;
    if lambda <= 0.0 {
        return;
    }

    let shelf_positions = world.shelf_positions();
    if shelf_positions.is_empty() {
        return;
    }

    let num_new_orders = sample_poisson(lambda, rng);

    for _ in 0..num_new_orders {
        let idx             = rng.gen_range(0..shelf_positions.len());
        let pickup_location = shelf_positions[idx];
        world.inject_order(pickup_location);
    }
}

fn sample_poisson(lambda: f64, rng: &mut impl Rng) -> u64 {
    match Poisson::new(lambda) {
        Ok(dist) => dist.sample(rng) as u64,
        Err(_)   => 0,
    }
}
