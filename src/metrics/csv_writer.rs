use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::Path;

use super::TickMetrics;

/// Writes the full metrics log to a CSV file at the given path.
/// Creates the parent directory if it doesn't exist.
pub fn write_csv(path: &str, metrics: &[TickMetrics]) -> Result<(), Box<dyn std::error::Error>> {
    // Ensure the output directory exists
    if let Some(parent) = Path::new(path).parent() {
        fs::create_dir_all(parent)?;
    }

    let file   = File::create(path)?;
    let mut w  = BufWriter::new(file);

    // Header row
    writeln!(
        w,
        "tick,\
         total_orders_completed,\
         throughput_last_100_ticks,\
         robot_utilization_pct,\
         max_charging_queue_length,\
         idle_robot_count,\
         robots_in_charge_cycle,\
         pending_order_count"
    )?;

    // Data rows
    for m in metrics {
        writeln!(
            w,
            "{},{},{},{:.2},{},{},{},{}",
            m.tick,
            m.total_orders_completed,
            m.throughput_last_100_ticks,
            m.robot_utilization_pct,
            m.max_charging_queue_length,
            m.idle_robot_count,
            m.robots_in_charge_cycle,
            m.pending_order_count,
        )?;
    }

    w.flush()?;
    Ok(())
}
