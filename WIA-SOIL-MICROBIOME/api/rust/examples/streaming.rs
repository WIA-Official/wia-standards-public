//! Streaming Example for WIA-SOIL-MICROBIOME SDK
//!
//! 弘益人間 (홍익인간) - Benefit All Humanity
//!
//! This example demonstrates real-time streaming of soil sensor data.

use wia_soil_microbiome_sdk::{
    streaming::{StreamClient, StreamEvent},
    Environment,
};
use futures_util::StreamExt;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("===========================================");
    println!("  WIA-SOIL-MICROBIOME - Streaming Example");
    println!("  弘益人間 (홍익인간) - Benefit All Humanity");
    println!("===========================================\n");

    let api_key = std::env::var("WIA_SOIL_API_KEY")
        .unwrap_or_else(|_| {
            println!("⚠️  WIA_SOIL_API_KEY not set, using demo key");
            "demo_key".to_string()
        });

    println!("🔗 Connecting to soil sensor stream...\n");

    let stream_client = StreamClient::new(&api_key, Environment::Sandbox)?;

    let field_id = "FIELD-001";
    println!("📡 Subscribing to field: {}", field_id);

    let mut stream = stream_client.subscribe_field(field_id).await?;

    println!("✅ Connected! Waiting for sensor data...\n");
    println!("(Press Ctrl+C to stop)\n");

    let mut event_count = 0;
    while let Some(event) = stream.next().await {
        match event {
            Ok(StreamEvent::SensorReading { sensor_id, reading, .. }) => {
                event_count += 1;
                println!(
                    "[Event #{}] 📊 Sensor {}: {:.2} {}",
                    event_count,
                    sensor_id,
                    reading.value,
                    reading.unit
                );
            }
            Ok(StreamEvent::MoistureAlert { zone, level, .. }) => {
                event_count += 1;
                let icon = if level < 20.0 { "🔴" } else if level > 80.0 { "🟡" } else { "🟢" };
                println!(
                    "[Event #{}] {} Moisture Alert - Zone {}: {:.1}%",
                    event_count, icon, zone, level
                );
            }
            Ok(StreamEvent::AnalysisComplete { sample_id, .. }) => {
                event_count += 1;
                println!(
                    "[Event #{}] ✅ Analysis Complete: {}",
                    event_count, sample_id
                );
            }
            Ok(StreamEvent::Heartbeat { timestamp }) => {
                println!("[Heartbeat] 💓 {}", timestamp);
            }
            Err(e) => {
                eprintln!("❌ Stream error: {}", e);
                break;
            }
        }

        if event_count >= 10 {
            println!("\n📍 Demo limit reached (10 events)");
            break;
        }
    }

    println!("\n✅ Streaming example completed!");
    println!("Events processed: {}", event_count);
    println!("弘益人間 (홍익인간) - For sustainable agriculture 🌾");

    Ok(())
}
