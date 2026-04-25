//! Streaming Client Example
//!
//! This example demonstrates how to use the WIA AI streaming protocol
//! to receive real-time responses from an AI agent.

use std::sync::Arc;
use wia_ai::prelude::*;
use wia_ai::streaming::{StreamAccumulator, PrintStreamHandler, StreamProcessor, SseParser};

#[tokio::main]
async fn main() -> ProtocolResult<()> {
    println!("=== WIA AI Streaming Client Example ===\n");

    // Create a mock transport with streaming responses
    let transport = MockScenarioBuilder::new()
        .with_streaming_response("Hello! I'm an AI assistant powered by the WIA AI protocol. How can I help you today?")
        .await
        .build();

    // Create a stream processor with a print handler
    let mut processor = StreamProcessor::new();
    processor.add_handler(Arc::new(PrintStreamHandler::new()));

    println!("Sending message and receiving streamed response:\n");
    println!("---");

    // Send a message and process the streaming response
    transport
        .send_streaming(
            ProtocolMessageBuilder::new()
                .message_type(MessageType::Message)
                .source(EntityReference::client("client-001"))
                .payload(serde_json::json!({
                    "role": "user",
                    "content": [{"type": "text", "text": "Hello!"}]
                }))
                .build()?,
            Box::new(move |msg| {
                // In a real application, you would process each message
                // through the stream processor
                if let Ok(payload) = serde_json::to_string_pretty(&msg.payload) {
                    println!("Received: {}", payload);
                }
            }),
        )
        .await?;

    println!("---\n");

    // Demonstrate SSE parsing
    println!("=== SSE Event Parsing Example ===\n");

    let sse_data = r#"event: stream_start
data: {"stream_id": "stream-001", "model": "claude-opus-4-5"}

event: stream_delta
data: {"delta": {"type": "text_delta", "text": "Hello"}}

event: stream_delta
data: {"delta": {"type": "text_delta", "text": " World"}}

event: stream_end
data: {"stop_reason": "end_turn", "usage": {"output_tokens": 2}}

"#;

    let mut parser = SseParser::new();
    let events = parser.feed(sse_data);

    for event in events {
        println!("Event Type: {:?}", event.event_type);
        println!("Data: {:?}", event.data);
        println!("---");
    }

    println!("\n=== Stream Accumulator Example ===\n");

    // Create a stream accumulator
    let mut accumulator = StreamAccumulator::new();

    // Simulate stream events
    accumulator.on_start(&StreamStartPayload {
        stream_id: "demo-stream".to_string(),
        model: Some("claude-opus-4-5".to_string()),
        input_tokens: Some(100),
    });

    accumulator.on_delta(
        &Delta::text("The WIA AI protocol "),
        0,
    );

    accumulator.on_delta(
        &Delta::text("enables standardized "),
        1,
    );

    accumulator.on_delta(
        &Delta::text("AI communication."),
        2,
    );

    accumulator.on_end(&StreamEndPayload {
        stream_id: "demo-stream".to_string(),
        stop_reason: Some(StopReason::EndTurn),
        usage: Some(Usage {
            input_tokens: Some(100),
            output_tokens: Some(10),
            total_tokens: Some(110),
            ..Default::default()
        }),
    });

    println!("Stream ID: {:?}", accumulator.stream_id);
    println!("Model: {:?}", accumulator.model);
    println!("Delta Count: {}", accumulator.delta_count);
    println!("Accumulated Text: {}", accumulator.text());
    println!("Stop Reason: {:?}", accumulator.stop_reason);
    println!("Completed: {}", accumulator.is_completed());

    Ok(())
}
