//! Error types for deep sea exploration

use thiserror::Error;

pub type Result<T> = std::result::Result<T, DeepSeaExplorationError>;

#[derive(Error, Debug)]
pub enum DeepSeaExplorationError {
    #[error("Network error: {0}")]
    NetworkError(String),

    #[error("Parse error: {0}")]
    ParseError(String),

    #[error("Validation error: {0}")]
    ValidationError(String),

    #[error("Mission error: {0}")]
    MissionError(String),

    #[error("Depth limit exceeded: {0}m")]
    DepthLimitExceeded(f64),

    #[error("Equipment error: {0}")]
    EquipmentError(String),

    #[error("Unknown error: {0}")]
    Unknown(String),
}
