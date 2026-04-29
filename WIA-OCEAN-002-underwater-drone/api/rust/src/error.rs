//! Error types for underwater drone

use thiserror::Error;

pub type Result<T> = std::result::Result<T, UnderwaterDroneError>;

#[derive(Error, Debug)]
pub enum UnderwaterDroneError {
    #[error("Network error: {0}")]
    NetworkError(String),

    #[error("Parse error: {0}")]
    ParseError(String),

    #[error("Validation error: {0}")]
    ValidationError(String),

    #[error("Navigation error: {0}")]
    NavigationError(String),

    #[error("Battery low: {0}%")]
    BatteryLow(f64),

    #[error("Depth limit exceeded: {0}m")]
    DepthLimitExceeded(f64),

    #[error("Communication lost")]
    CommunicationLost,

    #[error("Unknown error: {0}")]
    Unknown(String),
}
