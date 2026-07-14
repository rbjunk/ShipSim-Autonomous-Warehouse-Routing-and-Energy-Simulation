use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::Path;

use crate::world::World;
use super::TickMetrics;

/// Writes a summary of the full run.
/// Called once at the end of simulation, after the CSV is written.
pub fn write_summary(
    path:        &str,
    world:       &World,
    metrics_log: &[TickMetrics],
) -> Result<(), Box<dyn std::error::Error>> {
    if let Some(parent) = Path::new(path).parent() {
        fs::create_dir_all(parent)?;
    }

    let file = File::create(path)?;
    let mut w = BufWriter::new(file);

    //Derived values
    let total_ticks       = world.tick;
    let orders_completed  = world.completed_orders.len();
    let orders_generated  = orders_completed
        + world.active_orders.len()
        + world.pending_orders.len();

    let throughput = if total_ticks == 0 { 0.0 }
    else { orders_completed as f64 / total_ticks as f64 };

    let avg_utilization = if metrics_log.is_empty() { 0.0 }
    else {
        metrics_log.iter().map(|m| m.robot_utilization_pct).sum::<f64>()
            / metrics_log.len() as f64
    };

    let longest_queue = metrics_log.iter()
        .map(|m| m.max_charging_queue_length)
        .max()
        .unwrap_or(0);

    let avg_battery = if metrics_log.is_empty() { 0.0 }
    else {
        metrics_log.iter().map(|m| m.avg_battery_level).sum::<f64>()
            / metrics_log.len() as f64
    };

    let avg_fulfillment: f64 = world.completed_orders
        .iter()
        .filter_map(|o| o.fulfillment_time())
        .map(|t| t as f64)
        .sum::<f64>()
        / world.completed_orders.len() as f64;

    // Count deadlock episodes: each time deadlocked_robot_count transitions
    // from 0 to >0 is one new deadlock event.
    let mut deadlock_episodes: u64 = 0;
    let mut prev_was_deadlocked = false;
    for m in metrics_log {
        let is_deadlocked = m.deadlocked_robot_count > 0;
        if is_deadlocked && !prev_was_deadlocked {
            deadlock_episodes += 1;
        }
        prev_was_deadlocked = is_deadlocked;
    }

    // ── Output ────────────────────────────────────────────────────────────────
    writeln!(w, "ShipSim - Run Summary")?;
    writeln!(w, "─────────────────────────────────────────")?;
    writeln!(w, "total_ticks_elapsed:     {}", total_ticks)?;
    writeln!(w, "total_orders_generated:  {}", orders_generated)?;
    writeln!(w, "total_orders_completed:  {}", orders_completed)?;
    writeln!(w, "total_order_throughput:  {:.4} orders/tick", throughput)?;
    writeln!(w, "avg_fulfillment:         {:.1}", avg_fulfillment)?;
    writeln!(w, "avg_robot_utilization:   {:.2}%", avg_utilization)?;
    writeln!(w, "longest_charging_queue:  {}", longest_queue)?;
    writeln!(w, "avg_battery_level:       {:.2}%", avg_battery)?;
    writeln!(w, "deadlock_episodes:       {}", deadlock_episodes)?;

    w.flush()?;
    Ok(())
}