//! Error types for senior mobility

use thiserror::Error;

pub type Result<T> = std::result::Result<T, SeniorMobilityError>;

#[derive(Error, Debug)]
pub enum SeniorMobilityError {
    #[error("Network error: {0}")]
    NetworkError(String),

    #[error("Parse error: {0}")]
    ParseError(String),

    #[error("Validation error: {0}")]
    ValidationError(String),

    #[error("Unknown error: {0}")]
    Unknown(String),
}
