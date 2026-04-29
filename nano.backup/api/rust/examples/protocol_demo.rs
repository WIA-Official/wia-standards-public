//! WIA Nano Protocol Demo
//!
//! Demonstrates the use of WNP (WIA Nano Protocol) for nanoscale communication.
//!
//! Run with: cargo run --example protocol_demo

use wia_nano::prelude::*;
use wia_nano::protocol::*;
use wia_nano::transport::*;
use wia_nano::types::Position3D;

#[tokio::main]
async fn main() -> NanoResult<()> {
    println!("=== WIA Nano Protocol Demo ===\n");

    // Demo 1: Creating and sending protocol messages
    demo_message_creation().await?;

    // Demo 2: Transport layer demonstration
    demo_transport_layer().await?;

    // Demo 3: Swarm coordination
    demo_swarm_coordination().await?;

    println!("\n=== Demo Complete ===");
    Ok(())
}

async fn demo_message_creation() -> NanoResult<()> {
    println!("--- Demo 1: Protocol Messages ---\n");

    // Create a status message
    let status_msg = ProtocolMessage::status(
        "nanobot-001",
        StatusPayload {
            system_type: NanoSystemType::Nanorobot,
            operational_state: "active".to_string(),
            position: Some(Position3D::new(100.0, 200.0, 50.0)),
            energy_level: Some(0.85),
            error_code: None,
            metrics: serde_json::json!({
                "temperature_k": 310.0,
                "ph": 7.4,
                "cargo_remaining": 850
            }),
        },
    );

    println!("Status Message:");
    println!("  Source: {}", status_msg.header.source_id);
    println!("  Type: {:?}", status_msg.header.msg_type);
    println!("  Timestamp: {}", status_msg.header.timestamp_ns);

    // Create a command message
    let command_msg = ProtocolMessage::command(
        "controller-001",
        "nanobot-001",
        CommandPayload {
            command_id: "cmd-001".to_string(),
            command_type: CommandType::Move,
            parameters: serde_json::json!({
                "target_position": {"x": 500.0, "y": 300.0, "z": 100.0},
                "speed_nm_per_s": 10.0
            }),
            timeout_ms: Some(60000),
        },
    );

    println!("\nCommand Message:");
    println!("  From: {}", command_msg.header.source_id);
    println!("  To: {:?}", command_msg.header.dest_id);
    println!("  Command: {:?}", MessageType::Command);

    // Create an emergency message
    let emergency_header = ProtocolHeader::new("nanobot-001", MessageType::Emergency);
    let emergency_msg = ProtocolMessage::new(
        emergency_header,
        MessagePayload::Emergency(EmergencyPayload {
            emergency_type: EmergencyType::EnvironmentHazard,
            severity: 8,
            description: "Toxic molecule detected in vicinity".to_string(),
            position: Some(Position3D::new(150.0, 180.0, 45.0)),
            recommended_action: Some("evacuate_area".to_string()),
        }),
    ).with_checksum();

    println!("\nEmergency Message:");
    println!("  Type: {:?}", emergency_msg.header.msg_type);
    println!("  Urgent: {}", emergency_msg.header.flags.urgent);
    println!("  Checksum: {:?}", emergency_msg.checksum);
    println!("  Checksum valid: {}", emergency_msg.verify_checksum());

    Ok(())
}

async fn demo_transport_layer() -> NanoResult<()> {
    println!("\n--- Demo 2: Transport Layer ---\n");

    let source = Position3D::new(0.0, 0.0, 0.0);
    let destination = Position3D::new(500.0, 300.0, 100.0);
    let distance = source.distance_to(&destination);

    println!("Source: ({:.1}, {:.1}, {:.1}) nm", source.x, source.y, source.z);
    println!("Destination: ({:.1}, {:.1}, {:.1}) nm", destination.x, destination.y, destination.z);
    println!("Distance: {:.2} nm\n", distance);

    // Diffusion transport
    println!("1. Diffusion Transport (Molecular Communication):");
    let diffusion_config = DiffusionConfig::default();
    let diffusion = DiffusionTransport::new(diffusion_config);

    let result = diffusion.send(&source, &destination, 100).await?;
    println!("   Delivery time: {:.4} s", result.delivery_time_s.unwrap_or(0.0));
    println!("   Signal strength: {:.4}", result.signal_strength.unwrap_or(0.0));
    println!("   Success: {}", result.success);

    // Guided transport
    println!("\n2. Guided Transport (Magnetic Field):");
    let guided_config = GuidedConfig::magnetic_propulsion();
    let guided = GuidedTransport::new(guided_config);

    let result = guided.send(&source, &destination, 100).await?;
    println!("   Delivery time: {:.4} s", result.delivery_time_s.unwrap_or(0.0));
    println!("   Signal strength: {:.4}", result.signal_strength.unwrap_or(0.0));
    println!("   Success: {}", result.success);

    // Compare transport methods
    println!("\n3. Transport Comparison:");
    let diff_time = diffusion.estimate_delivery_time(&source, &destination)?;
    let guided_time = guided.estimate_delivery_time(&source, &destination)?;

    println!("   Diffusion delivery time: {:.4} s", diff_time);
    println!("   Guided delivery time: {:.4} s", guided_time);
    println!("   Speedup with guidance: {:.1}x", diff_time / guided_time);

    Ok(())
}

async fn demo_swarm_coordination() -> NanoResult<()> {
    println!("\n--- Demo 3: Swarm Coordination ---\n");

    // Create swarm coordinator
    let config = SwarmConfig {
        min_members: 3,
        max_members: 100,
        heartbeat_interval_ms: 1000,
        election_timeout_ms: 5000,
        consensus_threshold: 0.66,
    };

    let mut coordinator = SwarmCoordinator::new("swarm-alpha", "leader-001", config);

    // Join the swarm
    let join_msg = coordinator.join().await?;
    println!("Join Message Created:");
    println!("  Swarm ID: {}", coordinator.swarm_id());
    println!("  Node ID: {}", coordinator.node_id());
    println!("  Role: {:?}", coordinator.role());

    // Simulate other nodes joining
    println!("\nSimulating swarm formation...");
    for i in 2..=5 {
        let node_id = format!("node-{:03}", i);
        // In real scenario, we'd receive join messages from other nodes
        println!("  Node {} joined", node_id);
    }

    // Leader election
    let election_msg = coordinator.start_election().await?;
    println!("\nLeader Election:");
    if let MessagePayload::Swarm(payload) = &election_msg.payload {
        println!("  Action: {:?}", payload.swarm_action);
        println!("  Swarm: {}", payload.swarm_id);
    }

    // Set leader (simulating election result)
    coordinator.set_leader("leader-001".to_string()).await;
    println!("  Elected Leader: {:?}", coordinator.get_leader().await);
    println!("  Current Role: {:?}", coordinator.role());

    // Create a formation
    let formation = Formation::circle(
        Position3D::new(1000.0, 1000.0, 500.0),
        200.0,
        5,
    );

    let formation_msg = coordinator.update_formation(formation).await?;
    println!("\nFormation Update:");
    if let MessagePayload::Swarm(payload) = &formation_msg.payload {
        println!("  Action: {:?}", payload.swarm_action);
    }

    // Swarm statistics
    println!("\nSwarm Status:");
    println!("  Members: {}", coordinator.member_count().await);

    Ok(())
}
