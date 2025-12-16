//! Quorum Sensing Simulation
//!
//! Demonstrates bacterial-inspired quorum sensing for nanoscale swarm coordination.
//! When enough nanorobots are in proximity, they collectively trigger a behavior.
//!
//! Run with: cargo run --example quorum_sensing_sim

use wia_nano::prelude::*;
use wia_nano::protocol::quorum_sensing::*;
use wia_nano::types::Position3D;

#[tokio::main]
async fn main() -> NanoResult<()> {
    println!("=== Quorum Sensing Simulation ===\n");

    // Simulation 1: Basic quorum sensing with clustered nodes
    println!("--- Simulation 1: Clustered Nodes ---");
    run_clustered_simulation().await?;

    // Simulation 2: Grid-based node arrangement
    println!("\n--- Simulation 2: Grid Arrangement ---");
    run_grid_simulation().await?;

    // Simulation 3: Sparse nodes (quorum may not be reached)
    println!("\n--- Simulation 3: Sparse Nodes ---");
    run_sparse_simulation().await?;

    println!("\n=== Simulation Complete ===");
    Ok(())
}

async fn run_clustered_simulation() -> NanoResult<()> {
    println!("Creating clustered nanorobot swarm...\n");

    // Create a network with clustered nodes
    let mut network = QuorumSensingBuilder::new()
        .molecule(SignalMolecule::AHL)
        .threshold(50.0)          // 50 nM activation threshold
        .production_rate(5000.0)  // High production for quick activation
        .behavior(CollectiveBehavior::CargoRelease)
        .cluster_nodes(20, Position3D::new(0.0, 0.0, 0.0), 500.0)  // 20 nodes in 500nm radius
        .build()
        .await;

    println!("Network configuration:");
    println!("  Nodes: {}", network.node_count().await);
    println!("  Molecule: AHL");
    println!("  Threshold: 50 nM");
    println!("  Behavior: Cargo Release\n");

    // Run simulation
    println!("Running simulation (time steps of 0.1s):");
    let mut step = 0;

    loop {
        let result = network.update(0.1).await?;
        step += 1;

        // Print status every 10 steps
        if step % 10 == 0 || result.quorum_reached {
            let status = network.get_status().await;
            println!(
                "  t={:.1}s: activated={}/{}, avg_conc={:.1}nM, max_conc={:.1}nM",
                result.current_time_s,
                result.activated_nodes,
                result.total_nodes,
                status.avg_concentration_nm,
                status.max_concentration_nm,
            );

            if !result.newly_activated.is_empty() {
                println!("    Newly activated: {:?}", result.newly_activated);
            }
        }

        if result.quorum_reached {
            println!("\n  QUORUM REACHED at t={:.1}s!", result.current_time_s);
            if let Some(behavior) = network.trigger_behavior().await? {
                println!("  Triggering behavior: {:?}", behavior);
            }
            break;
        }

        // Safety limit
        if step > 500 {
            println!("\n  Simulation timeout (50s)");
            break;
        }
    }

    Ok(())
}

async fn run_grid_simulation() -> NanoResult<()> {
    println!("Creating grid-arranged nanorobot swarm...\n");

    // Create a network with grid nodes
    let mut network = QuorumSensingBuilder::new()
        .molecule(SignalMolecule::AI2)  // Universal autoinducer
        .threshold(30.0)
        .production_rate(3000.0)
        .behavior(CollectiveBehavior::BiofilmFormation)
        .grid_nodes(5, 5, 200.0)  // 5x5 grid with 200nm spacing
        .build()
        .await;

    println!("Network configuration:");
    println!("  Nodes: {}", network.node_count().await);
    println!("  Molecule: AI-2 (universal)");
    println!("  Threshold: 30 nM");
    println!("  Grid: 5x5, 200nm spacing\n");

    // Run simulation with progress tracking
    let max_time = 30.0;  // 30 seconds
    let dt = 0.1;
    let mut t = 0.0;

    println!("Running simulation for up to {:.0}s:", max_time);

    while t < max_time {
        let result = network.update(dt).await?;
        t += dt;

        if result.quorum_reached {
            println!("\n  QUORUM REACHED at t={:.1}s!", t);
            println!("  Activated: {}/{}", result.activated_nodes, result.total_nodes);

            let status = network.get_status().await;
            println!("  Final concentration: avg={:.1}nM, max={:.1}nM",
                status.avg_concentration_nm, status.max_concentration_nm);

            if let Some(behavior) = network.trigger_behavior().await? {
                println!("  Triggering behavior: {:?}", behavior);
            }
            return Ok(());
        }
    }

    let status = network.get_status().await;
    println!("\n  Quorum not reached after {:.0}s", max_time);
    println!("  Activated: {}/{}", status.activated_nodes, status.total_nodes);
    println!("  Final concentration: avg={:.1}nM, max={:.1}nM",
        status.avg_concentration_nm, status.max_concentration_nm);

    Ok(())
}

async fn run_sparse_simulation() -> NanoResult<()> {
    println!("Creating sparsely distributed nanorobots...\n");

    // Create a network with sparse nodes (may not reach quorum)
    let config = QuorumSensingConfig {
        molecule: SignalMolecule::AHL,
        production_rate: 1000.0,
        threshold_nm: 100.0,       // Higher threshold
        sensing_range_nm: 2000.0,  // Limited sensing range
        degradation_rate: 0.05,    // Higher degradation
        behavior: CollectiveBehavior::TargetAttack,
        activation_delay_s: 0.5,
    };

    let mut network = QuorumSensingNetwork::new(config);

    // Add sparse nodes (far apart)
    let positions = vec![
        Position3D::new(0.0, 0.0, 0.0),
        Position3D::new(5000.0, 0.0, 0.0),      // 5 μm apart
        Position3D::new(0.0, 5000.0, 0.0),
        Position3D::new(5000.0, 5000.0, 0.0),
        Position3D::new(2500.0, 2500.0, 0.0),   // Center node
    ];

    for (i, pos) in positions.into_iter().enumerate() {
        network.add_node(QuorumNode::new(format!("sparse_{}", i), pos)).await;
    }

    println!("Network configuration:");
    println!("  Nodes: {}", network.node_count().await);
    println!("  Spacing: ~5000 nm (5 μm)");
    println!("  Sensing range: 2000 nm");
    println!("  Threshold: 100 nM");
    println!("  Degradation: 5%/s\n");

    // Run simulation
    let max_time = 20.0;
    let dt = 0.2;
    let mut t = 0.0;

    println!("Running simulation (sparse configuration):");

    while t < max_time {
        let result = network.update(dt).await?;
        t += dt;

        if t as i32 % 5 == 0 && (t * 10.0) as i32 % 10 == 0 {
            let status = network.get_status().await;
            println!(
                "  t={:.0}s: activated={}/{}, avg_conc={:.2}nM",
                t, status.activated_nodes, status.total_nodes, status.avg_concentration_nm
            );
        }

        if result.quorum_reached {
            println!("\n  QUORUM REACHED (unexpected with sparse config)!");
            return Ok(());
        }
    }

    let status = network.get_status().await;
    println!("\n  Result: Quorum NOT reached (as expected with sparse nodes)");
    println!("  Activated: {}/{}", status.activated_nodes, status.total_nodes);
    println!("  Max concentration: {:.2}nM (below threshold of {:.0}nM)",
        status.max_concentration_nm, status.threshold_nm);
    println!("  This demonstrates that quorum sensing requires sufficient node density.");

    Ok(())
}
