//! Integration Pipeline Example
//!
//! This example demonstrates how to set up an output pipeline that processes
//! climate data through multiple adapters (console, webhook, storage).
//!
//! Run with: cargo run --example integration_pipeline

use wia_climate::prelude::*;
use wia_climate::integration::adapters::*;

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging
    tracing_subscriber::fmt::init();

    println!("=== WIA Climate - Integration Pipeline Example ===\n");

    // Create output manager with fanout strategy (parallel processing)
    let config = OutputConfig::fanout()
        .with_buffer_size(100)
        .with_circuit_breaker(true);

    let mut manager = OutputManager::new(config);

    // Add console adapter for debugging
    let console = ConsoleAdapter::new("debug-console").verbose();
    manager.add_adapter(console);

    // Add webhook adapter (simulated)
    let webhook_config = WebhookConfig::new("https://example.com/wia-climate/webhook")
        .with_secret("demo-secret")
        .with_header("X-Source", "wia-climate-demo");
    let webhook = WebhookAdapter::new("alert-webhook", webhook_config);
    manager.add_adapter(webhook);

    // Add InfluxDB adapter (simulated)
    let influx_config = InfluxDBConfig::new(
        "http://localhost:8086",
        "wia-climate",
        "climate-data",
    )
    .with_token("demo-token")
    .with_batch_size(10);
    let influx = InfluxDBAdapter::new("timeseries-db", influx_config);
    manager.add_adapter(influx);

    println!("Configured adapters: {:?}\n", manager.adapter_names());

    // Initialize all adapters
    // Note: In real usage, init would validate connections
    // For demo, we skip init to avoid connection errors

    // Create sample climate messages
    let messages = vec![
        // Carbon capture facility in Iceland
        ClimateMessage::builder()
            .location(Location::new(64.0, -21.0).with_altitude(100.0))
            .device(
                Device::new("Climeworks", "Orca DAC")
                    .with_serial("ORCA-2024-001"),
            )
            .carbon_capture_data(CarbonCaptureData {
                technology: CarbonCaptureTechnology::Dac,
                capture_rate_kg_per_hour: 125.5,
                co2_purity_percentage: Some(99.2),
                energy_consumption_kwh: Some(2500.0),
                ..Default::default()
            })
            .build()?,

        // Vertical farm in Singapore
        ClimateMessage::builder()
            .location(Location::new(1.35, 103.82))
            .device(Device::new("SkyGreens", "A-Go-Gro Tower"))
            .vertical_farming_data(VerticalFarmingData {
                facility_type: FacilityType::VerticalTower,
                crop_type: "Leafy Greens".to_string(),
                growing_area_sqm: 500.0,
                environmental_conditions: Some(EnvironmentalConditions {
                    temperature_celsius: 25.0,
                    humidity_percentage: 70.0,
                    co2_ppm: Some(800.0),
                    light_intensity_lux: Some(15000.0),
                    ..Default::default()
                }),
                ..Default::default()
            })
            .build()?,

        // Ocean cleanup vessel
        ClimateMessage::builder()
            .location(Location::new(35.0, -140.0))
            .device(Device::new("Ocean Cleanup", "System 002"))
            .ocean_cleanup_data(OceanCleanupData {
                operation_type: OceanOperationType::PlasticCollection,
                debris_collected_kg: Some(1500.0),
                area_covered_sqkm: Some(25.0),
                ..Default::default()
            })
            .build()?,
    ];

    // Process messages through the pipeline
    println!("--- Processing {} messages ---\n", messages.len());

    for (i, message) in messages.iter().enumerate() {
        println!("Processing message {} of {}...", i + 1, messages.len());

        let result = manager.process(message).await?;

        println!("  Success: {:?}", result.success);
        if !result.failed.is_empty() {
            println!("  Failed: {:?}", result.failed);
        }
        println!("  Duration: {}ms\n", result.duration_ms);
    }

    // Check adapter health
    println!("--- Adapter Health Status ---\n");
    let health = manager.health().await;
    for (name, status) in health {
        println!("  {}: {:?} (processed: {} messages)",
            name,
            status.status,
            status.messages_processed
        );
    }

    // Flush and shutdown
    println!("\n--- Shutting down ---");
    manager.shutdown().await?;

    println!("\n=== Summary ===");
    println!("Total messages processed: {}", manager.messages_processed());

    println!("\n弘益人間 - Benefit All Humanity 🌍");
    Ok(())
}
