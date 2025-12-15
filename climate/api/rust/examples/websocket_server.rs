//! WebSocket Server Example
//!
//! This example demonstrates how to create a simple WIA Climate WebSocket server
//! that handles incoming connections and messages.
//!
//! Note: This is a simplified example. A production server would need
//! additional features like authentication, rate limiting, and persistence.

use std::sync::Arc;
use tokio::sync::RwLock;
use wia_climate::prelude::*;
use wia_climate::protocol::*;
use wia_climate::transport::*;

/// Simple in-memory state for tracking connections
struct ServerState {
    sessions: std::collections::HashMap<String, SessionInfo>,
    message_count: u64,
}

struct SessionInfo {
    client_id: String,
    connected_at: chrono::DateTime<chrono::Utc>,
    capabilities: Vec<String>,
}

impl ServerState {
    fn new() -> Self {
        Self {
            sessions: std::collections::HashMap::new(),
            message_count: 0,
        }
    }
}

/// Custom message handler for the server
struct ClimateServerHandler {
    state: Arc<RwLock<ServerState>>,
}

impl ClimateServerHandler {
    fn new(state: Arc<RwLock<ServerState>>) -> Self {
        Self { state }
    }
}

#[async_trait::async_trait]
impl MessageHandler for ClimateServerHandler {
    async fn on_connect(&self, payload: &ConnectPayload) -> Result<ConnectAckPayload> {
        println!("  → Client connecting: {}", payload.client_id);

        let session_id = uuid::Uuid::new_v4().to_string();

        // Store session info
        let mut state = self.state.write().await;
        state.sessions.insert(
            session_id.clone(),
            SessionInfo {
                client_id: payload.client_id.clone(),
                connected_at: chrono::Utc::now(),
                capabilities: payload.capabilities.clone().unwrap_or_default(),
            },
        );

        println!("  ✓ Client connected: {} (session: {})", payload.client_id, &session_id[..8]);

        Ok(ConnectAckPayload {
            success: true,
            session_id: Some(session_id),
            server_info: Some(ServerInfo {
                name: "WIA Climate Demo Server".to_string(),
                version: "1.0.0".to_string(),
            }),
            keep_alive_interval: Some(30000),
            error: None,
        })
    }

    async fn on_disconnect(&self, payload: Option<&DisconnectPayload>) -> Result<()> {
        let reason = payload.and_then(|p| p.reason);
        println!("  → Client disconnected: {:?}", reason);
        Ok(())
    }

    async fn on_data(&self, message: &ClimateMessage) -> Result<()> {
        let mut state = self.state.write().await;
        state.message_count += 1;

        println!("  → Data received: {:?}", message.data_type);
        println!("    Location: ({:.2}, {:.2})",
            message.location.latitude,
            message.location.longitude
        );
        println!("    Device: {} {}",
            message.device.manufacturer,
            message.device.model
        );
        println!("    Total messages: {}", state.message_count);

        Ok(())
    }

    async fn on_command(&self, payload: &CommandPayload) -> Result<CommandAckPayload> {
        println!("  → Command received: {} → {}", payload.target_id, payload.action);

        // Simulate command execution
        let result = match payload.action.as_str() {
            "set_capture_rate" => {
                let rate = payload.parameters
                    .as_ref()
                    .and_then(|p| p.get("rate_kg_per_hour"))
                    .and_then(|v| v.as_f64())
                    .unwrap_or(0.0);

                println!("    Setting capture rate to {} kg/hour", rate);

                let mut result = std::collections::HashMap::new();
                result.insert(
                    "new_rate_kg_per_hour".to_string(),
                    serde_json::json!(rate),
                );
                Some(result)
            }
            "get_status" => {
                let mut result = std::collections::HashMap::new();
                result.insert("status".to_string(), serde_json::json!("operational"));
                result.insert("uptime_hours".to_string(), serde_json::json!(42));
                Some(result)
            }
            _ => {
                println!("    Unknown action");
                None
            }
        };

        Ok(CommandAckPayload {
            command_id: payload.target_id.clone(),
            success: true,
            result,
            error: None,
            execution_time: Some(10),
        })
    }

    async fn on_subscribe(&self, payload: &SubscribePayload) -> Result<SubscribeAckPayload> {
        let topics: Vec<String> = payload.topics.iter().map(|t| t.pattern.clone()).collect();
        println!("  → Subscribe: {:?} (QoS: {:?})", topics, payload.qos);

        let subscription_id = uuid::Uuid::new_v4().to_string();

        Ok(SubscribeAckPayload::success(subscription_id, topics))
    }

    async fn on_unsubscribe(&self, payload: &UnsubscribePayload) -> Result<()> {
        println!("  → Unsubscribe: {}", payload.subscription_id);
        Ok(())
    }

    async fn on_error(&self, payload: &ErrorPayload) -> Result<()> {
        println!("  → Error from client: {} - {}", payload.code, payload.message);
        Ok(())
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    println!("=== WIA Climate - WebSocket Server Example ===\n");
    println!("Note: This example demonstrates server-side message handling");
    println!("using a mock transport for simulation.\n");

    // Create server state
    let state = Arc::new(RwLock::new(ServerState::new()));

    // Create handler and dispatcher
    let handler = ClimateServerHandler::new(state.clone());
    let handler = LoggingHandler::new(handler);
    let dispatcher = MessageDispatcher::new(handler);

    // Simulate incoming messages using mock transport
    let mut transport = MockTransport::new();
    transport.set_connected(true);

    // Queue simulated client messages
    println!("--- Simulating Client Connection ---\n");

    // 1. Connect message
    let connect_msg = ConnectBuilder::new()
        .client_id("carbon-sensor-001")
        .client_type(ClientType::Sensor)
        .capability("carbon_capture")
        .build()
        .map_err(|e| ClimateError::Validation(e.to_string()))?;

    transport.queue_receive(connect_msg);

    // 2. Data message
    let climate_data = ClimateMessage::builder()
        .location(Location::new(64.0, -21.0))
        .device(Device::new("Climeworks", "Orca DAC"))
        .carbon_capture_data(CarbonCaptureData {
            technology: CarbonCaptureTechnology::Dac,
            capture_rate_kg_per_hour: 125.5,
            co2_purity_percentage: Some(99.2),
            ..Default::default()
        })
        .build()?;

    let data_msg = ProtocolMessage::data(&climate_data)
        .map_err(|e| ClimateError::SerializationError(e.to_string()))?;
    transport.queue_receive(data_msg);

    // 3. Command message
    let command_msg = CommandBuilder::new()
        .target("device-001")
        .action("set_capture_rate")
        .param("rate_kg_per_hour", 150.0)
        .build()
        .map_err(|e| ClimateError::Validation(e.to_string()))?;
    transport.queue_receive(command_msg);

    // 4. Subscribe message
    let subscribe_msg = SubscribeBuilder::new()
        .topic("carbon_capture/*")
        .qos(QoS::AtLeastOnce)
        .build()
        .map_err(|e| ClimateError::Validation(e.to_string()))?;
    transport.queue_receive(subscribe_msg);

    // 5. Ping message
    transport.queue_receive(ProtocolMessage::ping());

    // 6. Disconnect message
    let disconnect_msg = ProtocolMessage::disconnect(Some(DisconnectPayload::normal()))
        .map_err(|e| ClimateError::SerializationError(e.to_string()))?;
    transport.queue_receive(disconnect_msg);

    // Process all messages
    println!("--- Processing Messages ---\n");

    let mut responses = Vec::new();

    loop {
        match transport.receive().await {
            Ok(msg) => {
                println!("Processing: {}", msg.message_type);

                if let Ok(response) = dispatcher.dispatch(&msg).await {
                    if let Some(resp) = response {
                        println!("  ← Response: {}", resp.message_type);
                        responses.push(resp);
                    }
                }
                println!();
            }
            Err(_) => break,
        }
    }

    // Show summary
    println!("--- Server Summary ---\n");

    let state = state.read().await;
    println!("Active sessions: {}", state.sessions.len());
    println!("Messages processed: {}", state.message_count);
    println!("Responses generated: {}", responses.len());

    for resp in responses {
        println!("  - {} (ID: {})", resp.message_type, &resp.message_id[..8]);
    }

    println!("\n弘益人間 - Benefit All Humanity 🌍");
    Ok(())
}
