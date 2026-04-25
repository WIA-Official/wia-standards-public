//! Error types for adas

use thiserror::Error;

pub type Result<T> = std::result::Result<T, AdasError>;

#[derive(Error, Debug)]
pub enum AdasError {
    #[error("Network error: {0}")]
    NetworkError(String),

    #[error("Parse error: {0}")]
    ParseError(String),

    #[error("Validation error: {0}")]
    ValidationError(String),

    #[error("Unknown error: {0}")]
    Unknown(String),
}
