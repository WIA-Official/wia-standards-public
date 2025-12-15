//! WIA-AI Hub Integration Demo
//!
//! This example demonstrates how to use the WIA-AI Hub to integrate
//! with other WIA standards (AAC, BCI, Voice, TTS, Braille).
//!
//! Run with: cargo run --example wia_hub_demo

use wia_ai::integration::*;
use wia_ai::integration::adapters::*;

#[tokio::main]
async fn main() {
    println!("=== WIA-AI Hub Integration Demo ===\n");

    // 1. Create the WIA-AI Hub
    let hub = WiaAiHub::new();
    println!("Created WIA-AI Hub");

    // 2. Register input adapters
    hub.register_input_adapter(Box::new(AacInputAdapter::new())).await;
    hub.register_input_adapter(Box::new(BciInputAdapter::new())).await;
    hub.register_input_adapter(Box::new(VoiceInputAdapter::new())).await;
    println!("Registered input adapters: AAC, BCI, Voice");

    // 3. Register output adapters
    hub.register_output_adapter(Box::new(TtsOutputAdapter::new())).await;
    hub.register_output_adapter(Box::new(BrailleOutputAdapter::new())).await;
    println!("Registered output adapters: TTS, Braille\n");

    // 4. Set up AI response handler
    hub.set_response_handler(|input| {
        let user_text = input.text.unwrap_or_else(|| "[no input]".to_string());
        AiOutput::text(format!(
            "AI Assistant: I received your message: '{}'. How can I help you today?",
            user_text
        ))
    }).await;
    println!("Set AI response handler\n");

    // 5. Simulate AAC input
    println!("--- Simulating AAC Input ---");
    let aac_message = WiaMessage::text(
        WiaStandardType::Aac,
        WiaStandardType::Ai,
        "I need water",
    ).with_metadata("sensor_type", serde_json::json!("eye_tracker"));

    println!("AAC Input: 'I need water' (from eye tracker)");

    // Process and route to TTS
    match hub.conversation(aac_message, vec![WiaStandardType::Tts]).await {
        Ok(responses) => {
            for response in responses {
                if let WiaPayload::Json(json) = &response.payload {
                    println!("TTS Output: {}", json["text"]);
                }
            }
        }
        Err(e) => eprintln!("Error: {}", e),
    }
    println!();

    // 6. Simulate BCI input
    println!("--- Simulating BCI Input ---");
    let bci_message = WiaMessage::text(
        WiaStandardType::Bci,
        WiaStandardType::Ai,
        "yes",
    ).with_metadata("confidence", serde_json::json!(0.92))
     .with_metadata("paradigm", serde_json::json!("p300"));

    println!("BCI Input: 'yes' (P300 paradigm, 92% confidence)");

    match hub.conversation(bci_message, vec![WiaStandardType::Braille]).await {
        Ok(responses) => {
            for response in responses {
                if let WiaPayload::Json(json) = &response.payload {
                    println!("Braille Output: {}", json["braille"]);
                    println!("Original: {}", json["original_text"]);
                }
            }
        }
        Err(e) => eprintln!("Error: {}", e),
    }
    println!();

    // 7. Simulate Voice input with multi-output
    println!("--- Simulating Voice Input with Multi-Output ---");
    let voice_message = WiaMessage::text(
        WiaStandardType::Voice,
        WiaStandardType::Ai,
        "What time is it?",
    ).with_metadata("language", serde_json::json!("en-US"));

    println!("Voice Input: 'What time is it?'");

    match hub.conversation(
        voice_message,
        vec![WiaStandardType::Tts, WiaStandardType::Braille],
    ).await {
        Ok(responses) => {
            for response in &responses {
                match &response.target {
                    WiaStandardType::Tts => {
                        if let WiaPayload::Json(json) = &response.payload {
                            println!("TTS Output: {}", json["text"]);
                        }
                    }
                    WiaStandardType::Braille => {
                        if let WiaPayload::Json(json) = &response.payload {
                            println!("Braille Output: {}", json["braille"]);
                        }
                    }
                    _ => {}
                }
            }
        }
        Err(e) => eprintln!("Error: {}", e),
    }

    println!("\n=== Demo Complete ===");
    println!("\nThe WIA-AI Hub successfully:");
    println!("  - Processed AAC, BCI, and Voice inputs");
    println!("  - Converted inputs to AI-processable format");
    println!("  - Generated AI responses");
    println!("  - Routed outputs to TTS and Braille");
}
