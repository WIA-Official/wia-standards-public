//! Error types for nanotechnology

use thiserror::Error;

pub type Result<T> = std::result::Result<T, NanotechnologyError>;

#[derive(Error, Debug)]
pub enum NanotechnologyError {
    #[error("Network error: {0}")]
    NetworkError(String),

    #[error("Parse error: {0}")]
    ParseError(String),

    #[error("Validation error: {0}")]
    ValidationError(String),

    #[error("Synthesis error: {0}")]
    SynthesisError(String),

    #[error("Simulation error: {0}")]
    SimulationError(String),

    #[error("Configuration error: {0}")]
    ConfigError(String),

    #[error("Unknown error: {0}")]
    Unknown(String),
}
