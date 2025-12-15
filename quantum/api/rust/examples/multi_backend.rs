//! Multi-Backend Execution Example
//!
//! Demonstrates executing quantum circuits on multiple backends simultaneously.

use wia_quantum::prelude::*;
use wia_quantum::integration::{JobRequest, JobMonitor};
use std::sync::Arc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("=== WIA Quantum: Multi-Backend Execution ===\n");

    // =========================================
    // Part 1: Create Provider Manager
    // =========================================
    println!("--- Setting Up Provider Manager ---\n");

    let manager = Arc::new(ProviderManager::new());

    // Register providers
    manager.register(Box::new(LocalSimulatorProvider::new())).await?;
    manager.register(Box::new(IBMProvider::new())).await?;
    manager.register(Box::new(AmazonBraketProvider::new())).await?;

    println!("Registered providers:");
    for provider in manager.list().await {
        println!("  - {}", provider);
    }

    // =========================================
    // Part 2: Connect to Local Simulator
    // =========================================
    println!("\n--- Connecting to Local Simulator ---\n");

    // Connect only local (others need API keys)
    let mut configs = std::collections::HashMap::new();
    configs.insert("local".to_string(), ProviderConfig::default());

    // For demo, we only connect to local
    // In real usage, you'd provide API keys for cloud providers

    // =========================================
    // Part 3: List Available Backends
    // =========================================
    println!("--- Available Backends ---\n");

    // Get backends from local provider
    let local_backends = manager.get("local").await;
    if local_backends.is_some() {
        println!("Local Simulator Backends:");
        println!("  - statevector_simulator (30 qubits)");
    }

    // =========================================
    // Part 4: Create Quantum Circuit
    // =========================================
    println!("\n--- Creating Quantum Circuit ---\n");

    let mut circuit = QuantumCircuit::new(3, 3)
        .with_name("GHZ State")
        .with_description("Create 3-qubit GHZ state");

    // Create GHZ state: |000⟩ + |111⟩) / √2
    circuit.h(0)?;
    circuit.cx(0, 1)?;
    circuit.cx(1, 2)?;
    circuit.measure_all()?;

    println!("Circuit: {} qubits, {} gates", circuit.num_qubits(), circuit.gate_count());
    println!("OpenQASM:\n{}", circuit.to_qasm());

    // =========================================
    // Part 5: Submit to Multiple Backends
    // =========================================
    println!("\n--- Submitting Jobs ---\n");

    // Create job request
    let job = JobRequest::from_circuit(&circuit)
        .with_shots(2048);

    // Submit to local simulator
    let handle = manager.submit(job).await?;
    println!("Submitted job: {}", handle.job_id);
    println!("Provider: {}", handle.provider_id);
    println!("Backend: {}", handle.backend_id);

    // =========================================
    // Part 6: Monitor Job
    // =========================================
    println!("\n--- Monitoring Job ---\n");

    let monitor = JobMonitor::new(Arc::clone(&manager));
    monitor.track(handle.clone()).await;

    println!("Tracked jobs: {:?}", monitor.tracked().await);

    // Get status
    let status = monitor.status(&handle.job_id).await?;
    println!("Job status: {:?}", status);

    // =========================================
    // Part 7: Analyze Results
    // =========================================
    println!("\n--- Analyzing Results ---\n");

    // Create mock result for demonstration
    let mut counts = std::collections::HashMap::new();
    counts.insert("000".to_string(), 1024);
    counts.insert("111".to_string(), 1024);

    let result = ExecutionResult {
        counts,
        ..Default::default()
    };

    let mut analyzer = ResultAnalyzer::new();
    analyzer.add(result);

    let summary = analyzer.summary();
    println!("Total shots: {}", summary.total_shots);
    println!("Unique states: {}", summary.unique_states);
    println!("Entropy: {:.4} bits", summary.entropy);

    if let Some((state, prob)) = summary.most_probable {
        println!("Most probable: {} ({:.1}%)", state, prob * 100.0);
    }

    // =========================================
    // Part 8: Visualize Results
    // =========================================
    println!("\n--- Visualization ---\n");

    let viz = Visualizer::new();

    let histogram = viz.histogram(&analyzer.counts());
    println!("{}", histogram);

    println!("\n--- Summary ---\n");
    println!("{}", viz.summary_text(&summary));

    println!("\n=== Example Complete ===");

    Ok(())
}
