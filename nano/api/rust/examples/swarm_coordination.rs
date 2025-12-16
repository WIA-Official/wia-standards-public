//! Swarm Coordination Example
//!
//! Demonstrates multi-nanorobot swarm coordination using WIA Nano Protocol.
//! Features: formation control, leader election, task assignment.
//!
//! Run with: cargo run --example swarm_coordination

use wia_nano::prelude::*;
use wia_nano::protocol::*;
use wia_nano::transport::*;
use wia_nano::types::Position3D;
use std::collections::HashMap;

#[tokio::main]
async fn main() -> NanoResult<()> {
    println!("=== Swarm Coordination Demo ===\n");

    // Create swarm configuration
    let config = SwarmConfig {
        min_members: 3,
        max_members: 50,
        heartbeat_interval_ms: 500,
        election_timeout_ms: 3000,
        consensus_threshold: 0.66,
    };

    // Demo 1: Formation patterns
    println!("--- Demo 1: Formation Patterns ---\n");
    demo_formations().await?;

    // Demo 2: Leader election and task assignment
    println!("\n--- Demo 2: Leader Election & Tasks ---\n");
    demo_leader_election(config.clone()).await?;

    // Demo 3: Coordinated movement
    println!("\n--- Demo 3: Coordinated Movement ---\n");
    demo_coordinated_movement().await?;

    // Demo 4: Distributed sensing
    println!("\n--- Demo 4: Distributed Sensing ---\n");
    demo_distributed_sensing().await?;

    println!("\n=== Demo Complete ===");
    Ok(())
}

async fn demo_formations() -> NanoResult<()> {
    // Line formation
    println!("1. Line Formation:");
    let line = Formation::line(
        Position3D::new(0.0, 0.0, 0.0),
        100.0,  // 100nm spacing
        5,      // 5 positions
    );
    println!("   Type: {:?}", line.formation_type);
    println!("   Spacing: {:.0} nm", line.spacing_nm);
    println!("   Positions:");
    for (id, pos) in line.positions.iter() {
        println!("     {}: ({:.0}, {:.0}, {:.0})", id, pos.x, pos.y, pos.z);
    }

    // Circle formation
    println!("\n2. Circle Formation:");
    let circle = Formation::circle(
        Position3D::new(500.0, 500.0, 0.0),
        200.0,  // 200nm radius
        8,      // 8 positions
    );
    println!("   Type: {:?}", circle.formation_type);
    println!("   Radius: {:.0} nm", circle.spacing_nm);
    println!("   Center: ({:.0}, {:.0}, {:.0})",
        circle.center.as_ref().unwrap().x,
        circle.center.as_ref().unwrap().y,
        circle.center.as_ref().unwrap().z);
    println!("   Positions: {} nodes arranged in circle", circle.positions.len());

    // Custom sphere formation
    println!("\n3. Custom 3D Sphere Formation:");
    let sphere = create_sphere_formation(
        Position3D::new(1000.0, 1000.0, 1000.0),
        300.0,
        20,
    );
    println!("   Positions: {} nodes", sphere.len());
    println!("   Coverage: 3D sphere with radius 300nm");

    Ok(())
}

fn create_sphere_formation(center: Position3D, radius: f64, count: usize) -> HashMap<String, Position3D> {
    use std::f64::consts::PI;

    let mut positions = HashMap::new();
    let golden_ratio = (1.0 + 5.0_f64.sqrt()) / 2.0;

    for i in 0..count {
        let theta = 2.0 * PI * (i as f64) / golden_ratio;
        let phi = (1.0 - 2.0 * (i as f64 + 0.5) / count as f64).acos();

        let x = center.x + radius * phi.sin() * theta.cos();
        let y = center.y + radius * phi.sin() * theta.sin();
        let z = center.z + radius * phi.cos();

        positions.insert(format!("sphere_{}", i), Position3D::new(x, y, z));
    }

    positions
}

async fn demo_leader_election(config: SwarmConfig) -> NanoResult<()> {
    // Create multiple swarm coordinators (simulating different nodes)
    let mut nodes: Vec<SwarmCoordinator> = Vec::new();

    // Create 5 nodes
    for i in 1..=5 {
        let node_id = format!("nanobot-{:03}", i);
        let coordinator = SwarmCoordinator::new("drug-delivery-swarm", &node_id, config.clone());
        nodes.push(coordinator);
    }

    println!("Created {} swarm nodes", nodes.len());

    // Each node joins the swarm
    println!("\nNodes joining swarm:");
    for node in &mut nodes {
        let join_msg = node.join().await?;
        println!("  {} joined (members: {})", node.node_id(), node.member_count().await);
    }

    // First node initiates election
    println!("\nInitiating leader election...");
    let election_msg = nodes[0].start_election().await?;

    if let MessagePayload::Swarm(payload) = &election_msg.payload {
        println!("  Election initiated by: {}", nodes[0].node_id());
        println!("  Action: {:?}", payload.swarm_action);
    }

    // Simulate election result (highest ID wins in bully algorithm)
    let leader_id = "nanobot-005".to_string();
    println!("\nElection result:");
    println!("  Elected leader: {}", leader_id);

    for node in &mut nodes {
        node.set_leader(leader_id.clone()).await;
    }

    // Leader assigns tasks
    println!("\nLeader assigning tasks:");
    if let Some(leader_node) = nodes.iter().find(|n| n.role() == SwarmRole::Leader) {
        let task = SwarmTask {
            id: "task-001".to_string(),
            task_type: SwarmTaskType::Deliver,
            assignees: vec![
                "nanobot-001".to_string(),
                "nanobot-002".to_string(),
                "nanobot-003".to_string(),
            ],
            parameters: serde_json::json!({
                "target": "tumor_site",
                "cargo": "doxorubicin",
                "quantity": 1000
            }),
        };

        let task_msg = leader_node.assign_task(task).await?;
        if let MessagePayload::Swarm(payload) = &task_msg.payload {
            println!("  Task assigned: {:?}", payload.swarm_action);
            println!("  Task data: {}", payload.data);
        }
    }

    Ok(())
}

async fn demo_coordinated_movement() -> NanoResult<()> {
    // Simulate coordinated swarm movement using guided transport
    let guided_config = GuidedConfig::magnetic_propulsion();
    let transport = GuidedTransport::new(guided_config);

    // Initial positions (line formation)
    let mut positions: Vec<Position3D> = (0..5)
        .map(|i| Position3D::new(i as f64 * 100.0, 0.0, 0.0))
        .collect();

    // Target positions (converge to a point)
    let target = Position3D::new(500.0, 500.0, 500.0);

    println!("Initial positions:");
    for (i, pos) in positions.iter().enumerate() {
        println!("  Node {}: ({:.0}, {:.0}, {:.0})", i, pos.x, pos.y, pos.z);
    }

    println!("\nTarget: ({:.0}, {:.0}, {:.0})", target.x, target.y, target.z);

    // Calculate movement times
    println!("\nMovement plan:");
    let mut max_time = 0.0f64;

    for (i, pos) in positions.iter().enumerate() {
        let delivery_time = transport.estimate_delivery_time(pos, &target)?;
        max_time = max_time.max(delivery_time);
        println!("  Node {}: {:.3}s to reach target", i, delivery_time);
    }

    println!("\nSynchronized arrival time: {:.3}s", max_time);
    println!("(All nodes will adjust velocity to arrive simultaneously)");

    // Simulate trajectory
    println!("\nSimulated trajectory:");
    let planner = TrajectoryPlanner::new(GuidedConfig::magnetic_propulsion());

    let sample_node = &positions[2];  // Middle node
    let distance = sample_node.distance_to(&target);
    println!("  Sample node (2) distance to target: {:.1} nm", distance);
    println!("  Estimated velocity: {:.1} nm/s", transport.calculate_velocity());

    Ok(())
}

async fn demo_distributed_sensing() -> NanoResult<()> {
    // Simulate distributed environmental sensing with data aggregation

    println!("Setting up distributed sensor network...\n");

    // Sensor readings from different positions
    let sensor_data: Vec<(Position3D, SensorReading)> = vec![
        (Position3D::new(0.0, 0.0, 0.0), SensorReading {
            ph: 7.4,
            temperature_k: 310.0,
            glucose_mm: 5.5,
            oxygen_percent: 95.0,
        }),
        (Position3D::new(1000.0, 0.0, 0.0), SensorReading {
            ph: 6.8,  // Lower pH - possible tumor
            temperature_k: 311.0,
            glucose_mm: 3.2,  // Lower glucose
            oxygen_percent: 40.0,  // Hypoxic
        }),
        (Position3D::new(500.0, 500.0, 0.0), SensorReading {
            ph: 7.2,
            temperature_k: 310.5,
            glucose_mm: 4.5,
            oxygen_percent: 70.0,
        }),
        (Position3D::new(500.0, -500.0, 0.0), SensorReading {
            ph: 7.3,
            temperature_k: 310.0,
            glucose_mm: 5.0,
            oxygen_percent: 88.0,
        }),
    ];

    println!("Sensor readings:");
    for (i, (pos, reading)) in sensor_data.iter().enumerate() {
        println!("  Sensor {} at ({:.0}, {:.0}, {:.0}):",
            i, pos.x, pos.y, pos.z);
        println!("    pH: {:.1}, Temp: {:.1}K, Glucose: {:.1}mM, O2: {:.0}%",
            reading.ph, reading.temperature_k, reading.glucose_mm, reading.oxygen_percent);
    }

    // Aggregate and analyze
    println!("\nAggregated analysis:");

    let avg_ph: f64 = sensor_data.iter().map(|(_, r)| r.ph).sum::<f64>() / sensor_data.len() as f64;
    let min_oxygen = sensor_data.iter().map(|(_, r)| r.oxygen_percent).fold(f64::MAX, f64::min);

    println!("  Average pH: {:.2}", avg_ph);
    println!("  Minimum O2: {:.0}%", min_oxygen);

    // Detect anomaly
    if let Some((pos, reading)) = sensor_data.iter().find(|(_, r)| r.ph < 7.0 && r.oxygen_percent < 50.0) {
        println!("\n  ANOMALY DETECTED at ({:.0}, {:.0}, {:.0}):", pos.x, pos.y, pos.z);
        println!("    Low pH ({:.1}) and hypoxic ({:.0}%) - possible tumor microenvironment", reading.ph, reading.oxygen_percent);
        println!("    Recommended action: Converge swarm for targeted delivery");
    }

    Ok(())
}

#[derive(Debug, Clone)]
struct SensorReading {
    ph: f64,
    temperature_k: f64,
    glucose_mm: f64,
    oxygen_percent: f64,
}
