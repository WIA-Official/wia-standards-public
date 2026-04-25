//! Tool Invocation Example
//!
//! This example demonstrates how to use the WIA AI protocol
//! for tool calls and result handling.

use std::sync::Arc;
use wia_ai::prelude::*;
use wia_ai::protocol::{ToolError, ToolResultStatus};

#[tokio::main]
async fn main() -> ProtocolResult<()> {
    println!("=== WIA AI Tool Invocation Demo ===\n");

    // Define agents
    let assistant = EntityReference::agent("agent-assistant-001", "Assistant")
        .with_version("1.0.0");
    let tool_server = EntityReference::tool("tool-server-001")
        .with_version("1.0.0");

    // Create a tool call tracker
    let tracker = ToolCallTracker::new();

    // Example 1: Simple tool call
    println!("=== Simple Tool Call ===\n");

    let search_call = ToolCallBuilder::new("web_search")
        .arguments(serde_json::json!({
            "query": "WIA AI protocol specification",
            "max_results": 5
        }))
        .source(assistant.clone())
        .build()?;

    println!("Tool Call Message:");
    println!("{}", serde_json::to_string_pretty(&search_call).unwrap());
    println!();

    // Track the tool call
    let call_payload: ToolCallPayload =
        serde_json::from_value(search_call.payload.clone().unwrap())
            .map_err(|e| ProtocolError::validation_error(e.to_string()))?;
    tracker.track_call(call_payload.clone()).await;
    println!("Tracking call: {}", call_payload.call_id);
    println!("Pending calls: {:?}", tracker.pending_calls().await.len());

    // Simulate tool result
    let search_result = ProtocolMessageBuilder::new()
        .message_type(MessageType::ToolResult)
        .source(tool_server.clone())
        .target(assistant.clone())
        .typed_payload(&ToolResultPayload {
            call_id: call_payload.call_id.clone(),
            status: ToolResultStatus::Success,
            result: Some(serde_json::json!({
                "results": [
                    {
                        "title": "WIA AI Communication Protocol",
                        "url": "https://wia.live/protocol",
                        "snippet": "The WIA AI protocol enables seamless agent communication..."
                    },
                    {
                        "title": "Getting Started with WIA",
                        "url": "https://wia.live/docs",
                        "snippet": "Learn how to implement the WIA AI protocol..."
                    }
                ]
            })),
            error: None,
            execution_time_ms: Some(245),
        })?
        .build()?;

    println!("Tool Result Message:");
    println!("{}", serde_json::to_string_pretty(&search_result).unwrap());
    println!();

    // Complete the tracked call
    tracker.complete_call(&call_payload.call_id).await;
    println!("Call completed. Pending calls: {:?}", tracker.pending_calls().await.len());
    println!();

    // Example 2: Tool call with error
    println!("=== Tool Call with Error ===\n");

    let db_call = ToolCallBuilder::new("database_query")
        .arguments(serde_json::json!({
            "query": "SELECT * FROM non_existent_table",
            "database": "main"
        }))
        .source(assistant.clone())
        .build()?;

    let db_call_payload: ToolCallPayload =
        serde_json::from_value(db_call.payload.clone().unwrap())
            .map_err(|e| ProtocolError::validation_error(e.to_string()))?;

    println!("Database Query Call ID: {}", db_call_payload.call_id);

    // Simulate error result
    let error_result = ProtocolMessageBuilder::new()
        .message_type(MessageType::ToolResult)
        .source(tool_server.clone())
        .target(assistant.clone())
        .typed_payload(&ToolResultPayload {
            call_id: db_call_payload.call_id.clone(),
            status: ToolResultStatus::Error,
            result: None,
            error: Some(ToolError {
                code: "TABLE_NOT_FOUND".to_string(),
                message: "Table 'non_existent_table' does not exist".to_string(),
            }),
            execution_time_ms: Some(15),
        })?
        .build()?;

    println!("Error Result:");
    println!("{}", serde_json::to_string_pretty(&error_result).unwrap());
    println!();

    // Example 3: Multiple parallel tool calls
    println!("=== Parallel Tool Calls ===\n");

    let calls = vec![
        ("get_weather", serde_json::json!({"city": "Seoul"})),
        ("get_time", serde_json::json!({"timezone": "Asia/Seoul"})),
        ("get_news", serde_json::json!({"category": "technology", "limit": 3})),
    ];

    let mut call_ids = Vec::new();

    for (tool_name, args) in &calls {
        let call = ToolCallBuilder::new(*tool_name)
            .arguments(args.clone())
            .source(assistant.clone())
            .build()?;

        let payload: ToolCallPayload =
            serde_json::from_value(call.payload.clone().unwrap())
                .map_err(|e| ProtocolError::validation_error(e.to_string()))?;

        println!("  {} -> {}", tool_name, payload.call_id);
        call_ids.push(payload.call_id.clone());
        tracker.track_call(payload).await;
    }

    println!("\nTotal pending calls: {}", tracker.pending_calls().await.len());

    // Simulate receiving results in different order
    for (i, call_id) in call_ids.iter().rev().enumerate() {
        tracker.complete_call(call_id).await;
        println!(
            "Completed call {}: {} remaining",
            i + 1,
            tracker.pending_calls().await.len()
        );
    }
    println!();

    // Example 4: Using protocol router for tool handling
    println!("=== Protocol Router for Tools ===\n");

    let mut router = ProtocolRouter::new();
    router.register(Arc::new(SimpleToolHandler));

    // Create a tool call message
    let code_exec_call = ToolCallBuilder::new("code_execute")
        .arguments(serde_json::json!({
            "language": "python",
            "code": "print('Hello from WIA!')"
        }))
        .source(assistant.clone())
        .build()?;

    println!("Routing tool call through protocol router...");
    let responses = router.route(code_exec_call).await?;

    for response in &responses {
        println!("Response type: {:?}", response.message_type);
        if let Some(ref payload) = response.payload {
            if let Ok(result) = serde_json::from_value::<ToolResultPayload>(payload.clone()) {
                println!("Execution time: {:?}ms", result.execution_time_ms);
                if let Some(output) = result.result {
                    println!("Output: {}", output);
                }
            }
        }
    }
    println!();

    // Example 5: Error handling with retry
    println!("=== Error Handling with Retry ===\n");

    let error = ProtocolError::rate_limited(5000);
    println!("Error: {}", error);
    println!("Retryable: {}", error.is_retryable());
    println!("Retry after: {:?}ms", error.retry_after_ms);

    let retry_policy = wia_ai::protocol::RetryPolicy::new(3)
        .with_backoff(wia_ai::protocol::BackoffStrategy::Exponential)
        .with_initial_delay(1000)
        .with_max_delay(30000);

    println!("\nRetry policy:");
    for attempt in 0..4 {
        if retry_policy.should_retry(attempt) {
            println!(
                "  Attempt {}: retry after {}ms",
                attempt + 1,
                retry_policy.calculate_delay(attempt)
            );
        } else {
            println!("  Attempt {}: max retries exceeded", attempt + 1);
        }
    }

    println!("\n=== Demo Complete ===");

    Ok(())
}

/// Simple handler for tool calls
struct SimpleToolHandler;

#[async_trait::async_trait]
impl MessageHandler for SimpleToolHandler {
    async fn handle(&self, message: ProtocolMessage) -> ProtocolResult<Option<ProtocolMessage>> {
        if message.message_type != MessageType::ToolCall {
            return Ok(None);
        }

        let payload: ToolCallPayload = message
            .payload
            .as_ref()
            .map(|p| serde_json::from_value(p.clone()))
            .transpose()
            .map_err(|e| ProtocolError::validation_error(e.to_string()))?
            .ok_or_else(|| ProtocolError::validation_error("Missing payload"))?;

        // Simulate tool execution
        let result = match payload.tool_name.as_str() {
            "code_execute" => {
                serde_json::json!({
                    "stdout": "Hello from WIA!",
                    "stderr": "",
                    "exit_code": 0
                })
            }
            _ => {
                serde_json::json!({
                    "status": "executed",
                    "tool": payload.tool_name
                })
            }
        };

        // Build response
        let mut builder = ProtocolMessageBuilder::new()
            .message_type(MessageType::ToolResult);

        if let Some(ref target) = message.target {
            builder = builder.source(target.clone());
        }
        if let Some(ref source) = message.source {
            builder = builder.target(source.clone());
        }

        let response = builder
            .typed_payload(&ToolResultPayload {
                call_id: payload.call_id,
                status: ToolResultStatus::Success,
                result: Some(result),
                error: None,
                execution_time_ms: Some(50),
            })?
            .build()?;

        Ok(Some(response))
    }

    fn supported_types(&self) -> Vec<MessageType> {
        vec![MessageType::ToolCall]
    }
}
