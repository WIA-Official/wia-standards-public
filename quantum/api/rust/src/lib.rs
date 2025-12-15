//! # WIA Quantum SDK
//!
//! Rust SDK for WIA Quantum Standard - Quantum Computing
//!
//! ## Features
//!
//! - **Quantum Circuits**: Build and manipulate quantum circuits (OpenQASM 3 compatible)
//! - **Backends**: Run circuits on simulators or cloud backends
//! - **PQC**: Post-Quantum Cryptography (ML-KEM, ML-DSA, SLH-DSA)
//! - **QKD**: Quantum Key Distribution session management
//! - **States**: Quantum state manipulation and analysis
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use wia_quantum::{QuantumCircuit, SimulatorBackend, QuantumBackend};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     // Create Bell state circuit
//!     let mut circuit = QuantumCircuit::new(2, 2);
//!     circuit.h(0)?;
//!     circuit.cx(0, 1)?;
//!     circuit.measure_all()?;
//!
//!     // Run on simulator
//!     let backend = SimulatorBackend::new();
//!     let result = backend.run_default(&circuit).await?;
//!
//!     println!("Counts: {:?}", result.counts);
//!     Ok(())
//! }
//! ```
//!
//! ## PQC Example
//!
//! ```rust,no_run
//! use wia_quantum::crypto::{PqcAlgorithm, PqcKeyPair};
//!
//! fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     // Generate ML-KEM-768 key pair
//!     let keypair = PqcKeyPair::generate(PqcAlgorithm::MlKem768)?;
//!     println!("Key ID: {}", keypair.metadata.key_id);
//!     Ok(())
//! }
//! ```

pub mod error;
pub mod types;
pub mod circuit;
pub mod backend;
pub mod crypto;
pub mod state;
pub mod protocol;
pub mod integration;

// Re-exports
pub use error::{QuantumError, Result};
pub use types::{
    WIA_QUANTUM_VERSION, JobStatus, BackendType, BackendInfo,
    ExecutionConfig, Job, ExecutionResult, StateVector, ResultMetadata,
};
pub use circuit::{Gate, QuantumCircuit, CircuitData};
pub use backend::{QuantumBackend, SimulatorBackend};
pub use state::QuantumState;

/// Prelude module for common imports
pub mod prelude {
    pub use crate::error::{QuantumError, Result};
    pub use crate::circuit::{Gate, QuantumCircuit};
    pub use crate::backend::{QuantumBackend, SimulatorBackend};
    pub use crate::types::{ExecutionConfig, ExecutionResult};
    pub use crate::state::QuantumState;
    pub use crate::crypto::{PqcAlgorithm, PqcKeyPair, QkdSession};
    pub use crate::protocol::{Client, ClientConfig, Message, MessageType};
    pub use crate::integration::{
        ProviderManager, ProviderConfig, QuantumProvider,
        LocalSimulatorProvider, IBMProvider, AmazonBraketProvider,
        HybridExecutor, VQEConfig, QAOAConfig,
        ResultAnalyzer, Visualizer,
    };
}
