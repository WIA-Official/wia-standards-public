//! Error types for air-quality

use thiserror::Error;

pub type Result<T> = std::result::Result<T, AirQualityError>;

#[derive(Error, Debug)]
pub enum AirQualityError {
    #[error("Network error: {0}")]
    NetworkError(String),

    #[error("Parse error: {0}")]
    ParseError(String),

    #[error("Validation error: {0}")]
    ValidationError(String),

    #[error("Unknown error: {0}")]
    Unknown(String),
}
