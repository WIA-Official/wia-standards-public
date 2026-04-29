//! Basic usage example for WIA-CONTACT-009 Cosmic Communication SDK
//!
//! 홍익인간 (弘益人間) - Benefit All Humanity through Cosmic Communication

use wia_contact_009::{CosmicClient, UniversalMessage, MessageType, EncodingFormat};
use chrono::Utc;
use uuid::Uuid;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🌌 WIA-CONTACT-009 Cosmic Communication Demo");
    println!("============================================\n");

    // Initialize client
    let api_key = std::env::var("WIA_API_KEY")
        .unwrap_or_else(|_| "demo-key".to_string());
    let client = CosmicClient::new(api_key);

    // Example 1: Receive cosmic signal
    println!("📡 Receiving cosmic signal...");
    match client.receive_signal().await {
        Ok(signal) => {
            println!("✅ Signal received!");
            println!("   ID: {}", signal.signal_id);
            println!("   Type: {:?}", signal.signal_type);
            println!("   Frequency: {} Hz", signal.frequency);
            println!("   Confidence: {:.2}%", signal.confidence * 100.0);

            // Analyze the signal
            println!("\n🔬 Analyzing signal...");
            match client.analyze_signal(&signal.signal_id).await {
                Ok(analysis) => {
                    println!("✅ Analysis complete!");
                    println!("   Artificial probability: {:.2}%",
                        analysis.artificial_probability * 100.0);
                    println!("   Patterns found: {}", analysis.patterns.len());
                }
                Err(e) => println!("⚠️  Analysis failed: {}", e),
            }
        }
        Err(e) => println!("⚠️  Failed to receive signal: {}", e),
    }

    // Example 2: Send universal message
    println!("\n📤 Sending universal message...");
    let message = UniversalMessage {
        message_id: Uuid::new_v4().to_string(),
        message_type: MessageType::Greeting,
        content: "Greetings from Earth! We seek peaceful communication. 홍익인간".to_string(),
        encoding: EncodingFormat::Binary,
        protocol_version: "1.0".to_string(),
        created_at: Utc::now(),
    };

    match client.send_message(&message).await {
        Ok(message_id) => {
            println!("✅ Message sent!");
            println!("   Message ID: {}", message_id);
        }
        Err(e) => println!("⚠️  Failed to send message: {}", e),
    }

    // Example 3: List recent signals
    println!("\n📋 Listing recent signals...");
    match client.list_signals(5).await {
        Ok(signals) => {
            println!("✅ Found {} signals:", signals.len());
            for (i, signal) in signals.iter().enumerate() {
                println!("   {}. {} - {:?} ({:.2} Hz)",
                    i + 1,
                    signal.signal_id,
                    signal.signal_type,
                    signal.frequency
                );
            }
        }
        Err(e) => println!("⚠️  Failed to list signals: {}", e),
    }

    println!("\n🌟 Demo complete!");
    println!("   Remember: 홍익인간 (弘益人間) - Benefit All Humanity");

    Ok(())
}
