//! Error types for microplastic detection

use thiserror::Error;

pub type Result<T> = std::result::Result<T, MicroplasticDetectionError>;

#[derive(Error, Debug)]
pub enum MicroplasticDetectionError {
    #[error("Network error: {0}")]
    NetworkError(String),

    #[error("Parse error: {0}")]
    ParseError(String),

    #[error("Validation error: {0}")]
    ValidationError(String),

    #[error("Detection error: {0}")]
    DetectionError(String),

    #[error("Configuration error: {0}")]
    ConfigError(String),

    #[error("Unknown error: {0}")]
    Unknown(String),
}
