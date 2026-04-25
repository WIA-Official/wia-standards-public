//! Streaming Example for WIA-AGING SDK
//!
//! 弘益人間 (홍익인간) - Benefit All Humanity
//!
//! This example demonstrates real-time streaming of aging data
//! using WebSocket connections.

use wia_aging_sdk::{
    streaming::{StreamClient, StreamEvent},
    Environment,
};
use futures_util::StreamExt;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("===========================================");
    println!("  WIA-AGING SDK - Streaming Example");
    println!("  弘益人間 (홍익인간) - Benefit All Humanity");
    println!("===========================================\n");

    // Get API key from environment
    let api_key = std::env::var("WIA_AGING_API_KEY").unwrap_or_else(|_| {
        println!("⚠️  WIA_AGING_API_KEY not set, using demo key");
        "demo_key".to_string()
    });

    println!("🔗 Connecting to WebSocket stream...\n");

    // Create streaming client
    let stream_client = StreamClient::new(&api_key, Environment::Sandbox)?;

    // Subscribe to a profile's updates
    let profile_id = "demo-profile-001";
    println!("📡 Subscribing to profile: {}", profile_id);

    let mut stream = stream_client.subscribe_profile(profile_id).await?;

    println!("✅ Connected! Waiting for events...\n");
    println!("(Press Ctrl+C to stop)\n");

    // Process streaming events
    let mut event_count = 0;
    while let Some(event) = stream.next().await {
        match event {
            Ok(StreamEvent::BiomarkerUpdate { biomarker, .. }) => {
                event_count += 1;
                println!(
                    "[Event #{}] 📊 Biomarker Update: {} = {:.2} {}",
                    event_count,
                    biomarker.name.as_deref().unwrap_or(&biomarker.code),
                    biomarker.value,
                    biomarker.unit
                );
            }
            Ok(StreamEvent::AssessmentComplete { assessment_id, biological_age, .. }) => {
                event_count += 1;
                println!(
                    "[Event #{}] ✅ Assessment Complete: {} - Bio Age: {:.1}",
                    event_count, assessment_id, biological_age.value
                );
            }
            Ok(StreamEvent::Alert { message, severity, .. }) => {
                event_count += 1;
                println!(
                    "[Event #{}] ⚠️  Alert ({}): {}",
                    event_count, severity, message
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

        // Demo: stop after 10 events
        if event_count >= 10 {
            println!("\n📍 Demo limit reached (10 events)");
            break;
        }
    }

    println!("\n✅ Streaming example completed!");
    println!("Total events processed: {}", event_count);
    println!("弘益人間 (홍익인간) - For the benefit of all humanity 🌍");

    Ok(())
}
