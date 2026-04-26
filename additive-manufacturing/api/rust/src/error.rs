//! Error types for additive-manufacturing

use thiserror::Error;

pub type Result<T> = std::result::Result<T, AdditiveManufacturingError>;

#[derive(Error, Debug)]
pub enum AdditiveManufacturingError {
    #[error("Network error: {0}")]
    NetworkError(String),

    #[error("Parse error: {0}")]
    ParseError(String),

    #[error("Validation error: {0}")]
    ValidationError(String),

    #[error("Unknown error: {0}")]
    Unknown(String),
}
