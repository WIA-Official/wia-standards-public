//! Error types for WIA Quantum SDK

use thiserror::Error;

/// Quantum SDK error types
#[derive(Error, Debug, Clone)]
pub enum QuantumError {
    /// Circuit construction error
    #[error("Circuit error: {0}")]
    CircuitError(String),

    /// Backend execution error
    #[error("Backend error: {0}")]
    BackendError(String),

    /// Job execution error
    #[error("Execution error: {0}")]
    ExecutionError(String),

    /// Invalid qubit index
    #[error("Invalid qubit index: {0} (max: {1})")]
    InvalidQubit(usize, usize),

    /// Invalid classical bit index
    #[error("Invalid clbit index: {0} (max: {1})")]
    InvalidClbit(usize, usize),

    /// Cryptography error
    #[error("Crypto error: {0}")]
    CryptoError(String),

    /// Serialization error
    #[error("Serialization error: {0}")]
    SerializationError(String),

    /// Invalid configuration
    #[error("Invalid configuration: {0}")]
    InvalidConfig(String),

    /// State vector error
    #[error("State error: {0}")]
    StateError(String),

    /// Connection error
    #[error("Connection error: {0}")]
    ConnectionError(String),

    /// Protocol error
    #[error("Protocol error: {0}")]
    ProtocolError(String),

    /// Job error
    #[error("Job error: {0}")]
    JobError(String),

    /// Authentication error
    #[error("Authentication error: {0}")]
    AuthError(String),
}

/// Result type alias for QuantumError
pub type Result<T> = std::result::Result<T, QuantumError>;

impl From<serde_json::Error> for QuantumError {
    fn from(err: serde_json::Error) -> Self {
        QuantumError::SerializationError(err.to_string())
    }
}
