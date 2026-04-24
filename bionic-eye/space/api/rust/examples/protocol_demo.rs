//! WIA Space Protocol (WSP) Demo
//!
//! This example demonstrates the WIA Space Protocol for mission communication.
//!
//! Run with: cargo run --example protocol_demo

use wia_space::protocol::*;
use wia_space::transport::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== WIA Space Protocol Demo ===\n");

    // 1. Create message builders for different endpoints
    println!("1. Creating endpoints...");
    let ground_station = MessageBuilder::ground_station("goldstone-dsn");
    let spacecraft = MessageBuilder::spacecraft("mars-orbiter-01");

    println!("   Ground Station: goldstone-dsn");
    println!("   Spacecraft: mars-orbiter-01\n");

    // 2. Connection handshake
    println!("2. Connection Handshake");
    println!("   ─────────────────────");

    let dest_mission_ctrl = Endpoint::ground_station("mission-control");

    let connect_msg = ground_station.connect(
        &dest_mission_ctrl,
        ConnectPayload {
            client_name: "Goldstone Deep Space Network".to_string(),
            capabilities: vec![
                "telemetry".to_string(),
                "command".to_string(),
                "tracking".to_string(),
            ],
            supported_technologies: Some(vec![
                "dyson_sphere".to_string(),
                "mars_terraforming".to_string(),
                "asteroid_mining".to_string(),
            ]),
            options: Some(ConnectOptions {
                data_rate: Some(1_000_000),
                compression: Some(true),
                encryption: Some(true),
            }),
        },
    );

    println!("   → CONNECT message:");
    println!("     Type: {:?}", connect_msg.message_type);
    println!("     From: {} ({:?})", connect_msg.source.id, connect_msg.source.endpoint_type);
    println!("     To: {} ({:?})", connect_msg.destination.id, connect_msg.destination.endpoint_type);
    println!();

    // 3. Ping/Pong
    println!("3. Heartbeat (Ping/Pong)");
    println!("   ─────────────────────");

    let dest_spacecraft = Endpoint::spacecraft("mars-orbiter-01");
    let ping_msg = ground_station.ping(&dest_spacecraft);
    println!("   → PING sent at: {}ms", ping_msg.timestamp);

    let pong_msg = spacecraft.pong(
        &Endpoint::ground_station("goldstone-dsn"),
        &ping_msg.message_id,
        ping_msg.timestamp,
    );
    println!("   ← PONG received");
    println!("     Latency: {}ms\n", pong_msg.timestamp - ping_msg.timestamp);

    // 4. Telemetry data
    println!("4. Telemetry Data");
    println!("   ───────────────");

    let telemetry_msg = spacecraft.telemetry(
        &Endpoint::ground_station("mission-control"),
        TelemetryPayload {
            mission_id: "mars-terraform-2035".to_string(),
            subsystem: "atmospheric_processor".to_string(),
            readings: serde_json::json!({
                "co2_concentration": 0.953,
                "temperature_k": 210.5,
                "pressure_pa": 636,
                "processing_rate_kg_per_day": 1500
            }),
            status: "nominal".to_string(),
            timestamp: Some(TelemetryTimestamp {
                mission_elapsed_time_s: Some(86400.0),
                utc: Some("2035-03-15T12:00:00Z".to_string()),
            }),
        },
    );

    println!("   → TELEMETRY from mars-orbiter-01:");
    println!("     Mission: mars-terraform-2035");
    println!("     Subsystem: atmospheric_processor");
    println!("     Status: nominal");
    println!();

    // 5. Command
    println!("5. Sending Command");
    println!("   ─────────────────");

    let command_msg = ground_station.command(
        &dest_spacecraft,
        CommandPayload {
            command_id: "cmd-2035-001".to_string(),
            command_type: "adjust_parameters".to_string(),
            target_subsystem: "atmospheric_processor".to_string(),
            parameters: serde_json::json!({
                "processing_rate_target": 2000,
                "priority_gas": "O2"
            }),
            execution_time: Some("2035-03-15T14:00:00Z".to_string()),
            requires_ack: Some(true),
        },
    );

    println!("   → COMMAND: {}", command_msg.payload["commandId"]);
    println!("     Target: atmospheric_processor");
    println!("     Action: adjust_parameters");
    println!();

    // Command acknowledgment
    let cmd_ack_msg = spacecraft.command_ack(
        &Endpoint::ground_station("mission-control"),
        CommandAckPayload {
            command_id: "cmd-2035-001".to_string(),
            status: CommandStatus::Queued,
            executed_at: None,
            result: None,
            error: None,
        },
    );

    println!("   ← COMMAND_ACK: {:?}", cmd_ack_msg.payload["status"]);
    println!();

    // 6. Error handling
    println!("6. Error Handling");
    println!("   ───────────────");

    let error_msg = ground_station.error(
        &Endpoint::ground_station("mission-control"),
        2001,
        "SPACECRAFT_UNREACHABLE",
        "Unable to establish contact with mars-orbiter-01",
    );

    let error_code = ErrorCode::SpacecraftUnreachable;
    println!("   ⚠ ERROR: {}", error_code.name());
    println!("     Code: {}", error_code.code());
    println!("     Category: {:?}", error_code.category());
    println!("     Recoverable: {}", error_code.is_recoverable());
    println!("     Retry after: {:?}s", error_code.retry_delay_s());
    println!();

    // 7. Latency simulation
    println!("7. Deep Space Latency Simulation");
    println!("   ──────────────────────────────");

    let targets = [
        DeepSpaceTarget::Leo,
        DeepSpaceTarget::Moon,
        DeepSpaceTarget::MarsOpposition,
        DeepSpaceTarget::MarsConjunction,
        DeepSpaceTarget::Jupiter,
        DeepSpaceTarget::Saturn,
    ];

    println!("   {:15} {:>12} {:>12}", "Target", "One-way", "Round-trip");
    println!("   {:15} {:>12} {:>12}", "──────", "───────", "──────────");

    for target in targets {
        println!(
            "   {:15} {:>12} {:>12}",
            format!("{:?}", target),
            target.delay_string(),
            format!("{}x2", target.delay_string())
        );
    }
    println!();

    // 8. Mock transport demo
    println!("8. Mock Transport Demo");
    println!("   ────────────────────");

    let (mut transport_a, mut transport_b) = MockTransport::create_pair();

    // Send a message
    let test_msg = WspMessage::new(
        MessageType::Ping,
        Endpoint::ground_station("station-a"),
        Endpoint::ground_station("station-b"),
        serde_json::json!({ "test": true }),
    );

    transport_a.send(&test_msg).await?;
    println!("   → Sent message via MockTransport");

    let received = transport_b.receive().await?;
    println!("   ← Received message: {:?}", received.message_type);
    println!();

    // 9. Protocol handler demo
    println!("9. Protocol Handler Demo");
    println!("   ──────────────────────");

    let handler = ProtocolHandler::new();

    // Create session
    let session_id = handler
        .create_session(
            Endpoint::ground_station("gs-01"),
            Endpoint::ground_station("mission-control"),
        )
        .await;
    println!("   Created session: {}", &session_id[..8]);

    // Process a ping message
    let ping = WspMessage::new(
        MessageType::Ping,
        Endpoint::ground_station("gs-01"),
        Endpoint::ground_station("mission-control"),
        serde_json::json!({ "sentAt": chrono::Utc::now().timestamp_millis() }),
    );

    if let Some(response) = handler.process(ping).await? {
        println!("   Auto-responded with: {:?}", response.message_type);
    }
    println!();

    // Summary
    println!("=== Demo Complete ===");
    println!();
    println!("WIA Space Protocol provides:");
    println!("  ✓ Standardized message format");
    println!("  ✓ Connection management");
    println!("  ✓ Telemetry & command handling");
    println!("  ✓ Error codes & recovery");
    println!("  ✓ Deep space latency simulation");
    println!("  ✓ Transport abstraction layer");
    println!();
    println!("弘益人間 - Benefit All Humanity");

    Ok(())
}
