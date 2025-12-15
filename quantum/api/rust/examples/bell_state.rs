//! Bell State Example
//!
//! Creates a Bell state |Φ+⟩ = (|00⟩ + |11⟩) / √2 and measures it.

use wia_quantum::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    println!("=== WIA Quantum: Bell State Example ===\n");

    // Create a 2-qubit circuit with 2 classical bits
    let mut circuit = QuantumCircuit::new(2, 2)
        .with_name("Bell State")
        .with_description("Create Bell state |Φ+⟩");

    // Apply Hadamard to qubit 0
    circuit.h(0)?;

    // Apply CNOT with control=0, target=1
    circuit.cx(0, 1)?;

    // Measure all qubits
    circuit.measure_all()?;

    // Print OpenQASM representation
    println!("OpenQASM 3.0:");
    println!("{}", circuit.to_qasm());

    // Create simulator backend
    let backend = SimulatorBackend::new();
    println!("Backend: {} ({})", backend.name(), backend.provider());
    println!("Max qubits: {}\n", backend.max_qubits());

    // Run with 1024 shots
    let config = ExecutionConfig {
        shots: 1024,
        seed: Some(42),
        ..Default::default()
    };

    let result = backend.run(&circuit, &config).await?;

    // Print results
    println!("Execution Results:");
    println!("  Status: {:?}", result.status);
    println!("  Shots: {}", result.metadata.shots);
    println!("  Time: {:.2} ms", result.metadata.execution_time_ms);
    println!("\nMeasurement Counts:");
    for (bitstring, count) in &result.counts {
        let percentage = (*count as f64 / config.shots as f64) * 100.0;
        println!("  |{}⟩: {} ({:.1}%)", bitstring, count, percentage);
    }

    // Expected: ~50% |00⟩, ~50% |11⟩
    println!("\nExpected: ~50% |00⟩, ~50% |11⟩ (Bell state correlation)");

    // Print state vector (last shot)
    if let Some(sv) = &result.statevector {
        println!("\nFinal State Vector (last shot):");
        for i in 0..sv.len() {
            let amp = sv.amplitude(i);
            if amp.norm() > 1e-6 {
                println!("  |{:02b}⟩: {:.4} + {:.4}i", i, amp.re, amp.im);
            }
        }
    }

    // Export to JSON
    println!("\nCircuit JSON:");
    println!("{}", circuit.to_json()?);

    Ok(())
}
