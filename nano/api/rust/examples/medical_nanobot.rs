//! Example: Medical Nanorobot Navigation
//!
//! This example demonstrates a medical nanorobot navigating
//! through a simulated blood vessel to reach a target site.

use wia_nano::prelude::*;
use wia_nano::simulator::{NanoSimulator, SimulationWorld};

#[tokio::main]
async fn main() -> NanoResult<()> {
    println!("=== WIA Nano SDK: Medical Nanorobot Example ===\n");

    // Create simulator with bloodstream environment
    let world = SimulationWorld::bloodstream();
    let mut sim = NanoSimulator::with_world(world);

    // Create a medical nanorobot
    let robot = sim.create_nanorobot("med-bot-001")
        .with_position(Position3D::new(0.0, 0.0, 0.0))
        .with_propulsion(PropulsionType::Magnetic)
        .with_energy(50_000_000.0) // 50 pJ
        .build();

    // Add to simulation
    let robot_id = sim.add_nanorobot(robot).await?;
    println!("✓ Nanorobot deployed: {}", robot_id);

    // Get robot reference
    let robot_ref = sim.get_nanorobot(&robot_id).unwrap();

    // Define mission waypoints (navigation through vessel)
    let waypoints = vec![
        Waypoint {
            position: Position3D::new(1000.0, 0.0, 0.0),
            action: None,
            tolerance_nm: 100.0,
            dwell_time_ms: None,
        },
        Waypoint {
            position: Position3D::new(5000.0, 500.0, 200.0),
            action: Some(WaypointAction::Scan),
            tolerance_nm: 100.0,
            dwell_time_ms: Some(500),
        },
        Waypoint {
            position: Position3D::new(10000.0, 1000.0, 0.0),
            action: Some(WaypointAction::Sample),
            tolerance_nm: 50.0,
            dwell_time_ms: Some(1000),
        },
    ];

    // Create mission
    let mission = Mission {
        id: format!("mission-{}", uuid::Uuid::new_v4()),
        name: "Tumor Site Navigation".to_string(),
        mission_type: MissionType::Navigation,
        waypoints,
        parameters: MissionParameters {
            max_velocity: Some(50.0), // 50 nm/s
            collision_avoidance: true,
            energy_conservation: true,
            logging_enabled: true,
        },
        timeout_ms: Some(60000),
    };

    // Start mission
    {
        let mut robot = robot_ref.write().await;
        let mission_id = robot.start_mission(mission).await?;
        println!("✓ Mission started: {}", mission_id);
    }

    // Simulate navigation
    println!("\nNavigating to target site...");

    for i in 0..5 {
        // Move robot
        {
            let mut robot = robot_ref.write().await;
            let target = Position3D::new((i as f64 + 1.0) * 2000.0, 0.0, 0.0);
            robot.move_to(target).await?;

            let pos = robot.position();
            println!("  Step {}: Position ({:.0}, {:.0}, {:.0}) nm",
                     i + 1, pos.x, pos.y, pos.z);
        }

        // Advance simulation
        sim.run_for(1_000_000.0).await?; // 1 ms
    }

    // Execute action at target
    {
        let mut robot = robot_ref.write().await;

        // Scan surroundings
        let scan_result = robot.execute_action(RobotAction::Scan { range_nm: 500.0 }).await?;
        println!("\n✓ Scan completed: {} fJ consumed", scan_result.energy_consumed_fj);

        // Collect sample
        let sample_result = robot.execute_action(RobotAction::Sample { volume_fl: 1.0 }).await?;
        println!("✓ Sample collected: {} fJ consumed", sample_result.energy_consumed_fj);
    }

    // Get final status
    {
        let robot = robot_ref.read().await;
        let message = robot.to_message()?;
        println!("\nFinal Robot Status:");
        println!("{}", serde_json::to_string_pretty(&message)?);

        if let Some(energy) = robot.energy_level() {
            println!("\nRemaining energy: {:.2} pJ ({:.1}%)",
                     energy / 1000.0,
                     energy / 500_000.0);
        }
    }

    // Print simulation stats
    let stats = sim.stats();
    println!("\nSimulation Stats:");
    println!("  Time elapsed: {:.3} ms", stats.time_ns / 1_000_000.0);
    println!("  Active robots: {}", stats.nanorobot_count);

    println!("\n✓ Mission complete!");

    Ok(())
}
