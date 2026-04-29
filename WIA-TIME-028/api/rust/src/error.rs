//! Error types for time management 028

use thiserror::Error;

pub type Result<T> = std::result::Result<T, TimeManagementError>;

#[derive(Error, Debug)]
pub enum TimeManagementError {
    #[error("Network error: {0}")]
    NetworkError(String),

    #[error("Parse error: {0}")]
    ParseError(String),

    #[error("Validation error: {0}")]
    ValidationError(String),

    #[error("Unknown error: {0}")]
    Unknown(String),
}
