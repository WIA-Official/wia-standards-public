//! Error types for marine sensor

use thiserror::Error;

pub type Result<T> = std::result::Result<T, MarineSensorError>;

#[derive(Error, Debug)]
pub enum MarineSensorError {
    #[error("Network error: {0}")]
    NetworkError(String),

    #[error("Parse error: {0}")]
    ParseError(String),

    #[error("Validation error: {0}")]
    ValidationError(String),

    #[error("Unknown error: {0}")]
    Unknown(String),
}
