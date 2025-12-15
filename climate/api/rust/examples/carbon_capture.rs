//! Carbon Capture example using the simulator adapter
//!
//! This example demonstrates how to use the CarbonCaptureSimulator
//! to generate realistic carbon capture data.

use wia_climate::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    println!("=== WIA Climate Standard - Carbon Capture Simulator ===\n");

    // Create a simulator for the Climeworks Orca plant in Iceland
    let simulator = CarbonCaptureSimulator::new(
        Location::new(64.0, -21.0).with_altitude(100.0),
        Device::new("Climeworks", "Orca DAC Simulator")
            .with_serial("SIM-ORCA-001"),
    )
    .with_capture_rate(125.0)  // Base capture rate in kg/hour
    .with_variation(0.15);      // 15% variation

    println!("Simulator: {}", simulator.name());
    println!("Connected: {}", simulator.is_connected());
    println!();

    // Generate 5 data points
    println!("Generating 5 carbon capture data points...\n");

    for i in 1..=5 {
        let message = simulator.read().await?;

        println!("--- Data Point {} ---", i);
        println!("Timestamp: {:?}", message.timestamp.iso8601);
        println!("Location: ({:.2}, {:.2})",
            message.location.latitude,
            message.location.longitude
        );

        if let ClimateData::CarbonCapture(ref data) = message.data {
            println!("Technology: {:?}", data.technology);
            println!("Capture Rate: {:.2} kg/hour", data.capture_rate_kg_per_hour);

            if let Some(purity) = data.co2_purity_percentage {
                println!("CO2 Purity: {:.1}%", purity);
            }

            if let Some(energy) = data.energy_consumption_kwh {
                println!("Energy: {:.1} kWh", energy);
            }

            if let Some(ref sorbent) = data.sorbent_status {
                if let Some(eff) = sorbent.efficiency_percentage {
                    println!("Sorbent Efficiency: {:.1}%", eff);
                }
                if let Some(cycles) = sorbent.cycles_completed {
                    println!("Cycles Completed: {}", cycles);
                }
            }

            if let Some(ref storage) = data.storage {
                println!("Storage Method: {:?}", storage.method);
                if let Some(depth) = storage.depth_m {
                    println!("Storage Depth: {:.0}m", depth);
                }
            }

            if let Some(cumulative) = data.cumulative_captured_tonnes {
                println!("Cumulative Captured: {:.2} tonnes", cumulative);
            }
        }

        if let Some(ref meta) = message.meta {
            if let Some(quality) = meta.quality_score {
                println!("Quality Score: {:.2}", quality);
            }
        }

        println!();

        // Small delay for timestamp variation
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
    }

    // Export one message to JSON
    println!("--- JSON Export ---");
    let message = simulator.read().await?;
    let json = message.to_json_pretty()?;
    println!("{}", json);

    println!("\n弘益人間 - Benefit All Humanity 🌍");

    Ok(())
}
