//! VQE (Variational Quantum Eigensolver) Example
//!
//! Demonstrates running a VQE algorithm to find ground state energy.

use wia_quantum::prelude::*;
use wia_quantum::integration::AnsatzType;
use std::sync::Arc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("=== WIA Quantum: VQE Example ===\n");

    // =========================================
    // Part 1: Setup Provider
    // =========================================
    println!("--- Setting Up Provider ---\n");

    let manager = Arc::new(ProviderManager::new());
    manager.register(Box::new(LocalSimulatorProvider::new())).await?;

    println!("Provider registered: local");

    // =========================================
    // Part 2: Create Hybrid Executor
    // =========================================
    println!("\n--- Creating Hybrid Executor ---\n");

    let executor = HybridExecutor::new(Arc::clone(&manager));
    println!("Hybrid executor ready");

    // =========================================
    // Part 3: Configure VQE
    // =========================================
    println!("\n--- Configuring VQE ---\n");

    // Simple 2-qubit Hamiltonian: H = ZZ + XX
    let config = VQEConfig {
        num_qubits: 2,
        hamiltonian: vec![
            ("ZZ".to_string(), 0.5),
            ("XX".to_string(), 0.5),
        ],
        ansatz: AnsatzType::RealAmplitudes,
        layers: 2,
        optimizer: wia_quantum::integration::OptimizerType::COBYLA,
        shots: 1024,
        max_iterations: 10, // Reduced for demo
        tolerance: 1e-4,
        backend_id: None,
    };

    println!("Hamiltonian: 0.5 * ZZ + 0.5 * XX");
    println!("Ansatz: RealAmplitudes with {} layers", config.layers);
    println!("Optimizer: COBYLA");
    println!("Max iterations: {}", config.max_iterations);

    // =========================================
    // Part 4: Run VQE
    // =========================================
    println!("\n--- Running VQE ---\n");

    println!("Starting optimization...");

    let result = executor.run_vqe(config).await?;

    // =========================================
    // Part 5: Display Results
    // =========================================
    println!("\n--- VQE Results ---\n");

    println!("Optimal energy: {:.6}", result.optimal_value);
    println!("Converged: {}", result.converged);
    println!("Iterations: {}", result.iterations);

    println!("\nOptimal parameters:");
    for (i, param) in result.optimal_params.iter().enumerate() {
        println!("  θ_{}: {:.4}", i, param);
    }

    println!("\nEnergy history:");
    for (i, energy) in result.energy_history.iter().enumerate() {
        let bar_len = ((energy.abs() * 20.0).min(20.0)) as usize;
        let bar = "█".repeat(bar_len);
        println!("  Iter {:>2}: {:+.4} {}", i + 1, energy, bar);
    }

    println!("\nFinal state distribution:");
    let mut sorted: Vec<_> = result.final_counts.iter().collect();
    sorted.sort_by(|a, b| b.1.cmp(a.1));
    for (state, count) in sorted.iter().take(4) {
        println!("  |{}⟩: {} counts", state, count);
    }

    // =========================================
    // Part 6: Theoretical Comparison
    // =========================================
    println!("\n--- Theoretical Comparison ---\n");

    // For H = 0.5 * ZZ + 0.5 * XX, ground state energy is approximately -0.5
    let theoretical = -0.5;
    let error = (result.optimal_value - theoretical).abs();

    println!("Theoretical ground state: {:.4}", theoretical);
    println!("VQE result:              {:.4}", result.optimal_value);
    println!("Absolute error:          {:.4}", error);

    println!("\n=== VQE Example Complete ===");

    Ok(())
}
