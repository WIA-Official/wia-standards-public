//! Quantum backend module

mod simulator;

pub use simulator::SimulatorBackend;

use async_trait::async_trait;
use crate::circuit::QuantumCircuit;
use crate::error::Result;
use crate::types::{ExecutionResult, ExecutionConfig};

/// Quantum backend trait
#[async_trait]
pub trait QuantumBackend: Send + Sync {
    /// Get backend name
    fn name(&self) -> &str;

    /// Get provider name
    fn provider(&self) -> &str;

    /// Get maximum supported qubits
    fn max_qubits(&self) -> usize;

    /// Check if backend is a simulator
    fn is_simulator(&self) -> bool;

    /// Run a quantum circuit
    async fn run(&self, circuit: &QuantumCircuit, config: &ExecutionConfig) -> Result<ExecutionResult>;

    /// Run with default config
    async fn run_default(&self, circuit: &QuantumCircuit) -> Result<ExecutionResult> {
        self.run(circuit, &ExecutionConfig::default()).await
    }
}
