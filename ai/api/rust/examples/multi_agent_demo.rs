//! Multi-Agent Demo
//!
//! This example demonstrates how to use the WIA AI protocol
//! for multi-agent communication and coordination.

use std::sync::Arc;
use wia_ai::prelude::*;

#[tokio::main]
async fn main() -> ProtocolResult<()> {
    println!("=== WIA AI Multi-Agent Demo ===\n");

    // Define our agents
    let researcher = EntityReference::agent("agent-researcher-001", "Researcher")
        .with_version("1.0.0");
    let coder = EntityReference::agent("agent-coder-001", "Coder")
        .with_version("1.0.0");
    let reviewer = EntityReference::agent("agent-reviewer-001", "Reviewer")
        .with_version("1.0.0");
    let orchestrator = EntityReference::orchestrator("agent-orchestrator-001", "Orchestrator")
        .with_version("1.0.0");

    println!("Agents registered:");
    println!("  - Researcher: {}", researcher.id);
    println!("  - Coder: {}", coder.id);
    println!("  - Reviewer: {}", reviewer.id);
    println!("  - Orchestrator: {}", orchestrator.id);
    println!();

    // Create a protocol router
    let mut router = ProtocolRouter::new();
    router.register(Arc::new(PingPongHandler));

    // Demonstrate ping/pong
    println!("=== Ping/Pong Example ===\n");
    let ping = ProtocolMessage::ping();
    println!("Sending ping: {}", ping.message_id);

    let responses = router.route(ping).await?;
    for response in &responses {
        println!("Received pong: {:?}", response.message_type);
    }
    println!();

    // Demonstrate handoff from Researcher to Coder
    println!("=== Handoff Example ===\n");

    let handoff = HandoffBuilder::new(
        researcher.clone(),
        coder.clone(),
        "Code implementation required",
    )
    .priority(HandoffPriority::High)
    .task("Implement a search algorithm based on research findings")
    .requirement("Python 3.10+")
    .requirement("Use async/await pattern")
    .requirement("Include unit tests")
    .deadline_ms(300000)
    .build()?;

    println!("Handoff Message:");
    println!("{}", serde_json::to_string_pretty(&handoff).unwrap_or_default());
    println!();

    // Simulate handoff acknowledgment
    let handoff_payload: HandoffPayload =
        serde_json::from_value(handoff.payload.clone().unwrap())
            .map_err(|e| ProtocolError::validation_error(e.to_string()))?;

    let handoff_ack = ProtocolMessageBuilder::new()
        .message_type(MessageType::HandoffAck)
        .source(coder.clone())
        .target(researcher.clone())
        .typed_payload(&HandoffAckPayload {
            handoff_id: handoff_payload.handoff_id.clone(),
            accepted: true,
            reason: None,
            estimated_duration_ms: Some(60000),
        })?
        .build()?;

    println!("Handoff Acknowledgment:");
    println!("{}", serde_json::to_string_pretty(&handoff_ack).unwrap_or_default());
    println!();

    // Demonstrate broadcast from Orchestrator
    println!("=== Broadcast Example ===\n");

    let broadcast = ProtocolMessageBuilder::new()
        .message_type(MessageType::Broadcast)
        .source(orchestrator.clone())
        .payload(serde_json::json!({
            "broadcast_type": "status_update",
            "message": "Project deadline extended by 2 days",
            "recipients": [
                researcher.id,
                coder.id,
                reviewer.id
            ]
        }))
        .build()?;

    println!("Broadcast Message:");
    println!("{}", serde_json::to_string_pretty(&broadcast).unwrap_or_default());
    println!();

    // Demonstrate protocol session
    println!("=== Protocol Session Example ===\n");

    let session = ProtocolSession::new("session-001");
    println!("Session ID: {}", session.session_id);
    println!("Initial state: {:?}", session.state().await);

    session.set_state(ConnectionState::Connecting).await;
    println!("Connecting...");

    session.set_state(ConnectionState::Connected).await;
    println!("Connected: {}", session.is_connected().await);

    session.set_metadata("agent_id", serde_json::json!("agent-researcher-001")).await;
    session.set_metadata("capabilities", serde_json::json!(["streaming", "tools"])).await;

    println!("Session metadata set");
    println!();

    // Demonstrate stream manager
    println!("=== Stream Manager Example ===\n");

    let stream_manager = StreamManager::new();

    // Start a stream
    let stream_id = stream_manager
        .start_stream("stream-multi-agent", Some("claude-opus-4-5".to_string()))
        .await;
    println!("Started stream: {}", stream_id);

    // Apply some deltas
    stream_manager
        .apply_delta(&stream_id, &Delta::text("Analyzing request..."))
        .await?;
    stream_manager
        .apply_delta(&stream_id, &Delta::text(" Delegating to coder."))
        .await?;

    // Get stream state
    let state = stream_manager.get_stream(&stream_id).await.unwrap();
    println!("Stream text: {}", state.accumulated_text);
    println!("Delta count: {}", state.delta_index);

    // End the stream
    let final_state = stream_manager
        .end_stream(&stream_id, Some(Usage {
            input_tokens: Some(50),
            output_tokens: Some(10),
            total_tokens: Some(60),
            ..Default::default()
        }))
        .await?;
    println!("Stream completed: {}", !final_state.active);
    println!();

    // Demonstrate error handling
    println!("=== Error Handling Example ===\n");

    let error = ProtocolError::agent_busy("agent-coder-001");
    println!("Error: {}", error);
    println!("Retryable: {}", error.is_retryable());
    println!("Retry after: {:?}ms", error.retry_after_ms);

    let retry_policy = wia_ai::protocol::RetryPolicy::new(3)
        .with_backoff(wia_ai::protocol::BackoffStrategy::Exponential)
        .with_initial_delay(1000);

    println!("\nRetry delays:");
    for attempt in 0..3 {
        println!("  Attempt {}: {}ms", attempt + 1, retry_policy.calculate_delay(attempt));
    }

    println!("\n=== Demo Complete ===");

    Ok(())
}
