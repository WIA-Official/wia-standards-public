//! Error types for underwater communication

use thiserror::Error;

pub type Result<T> = std::result::Result<T, UnderwaterCommunicationError>;

#[derive(Error, Debug)]
pub enum UnderwaterCommunicationError {
    #[error("Network error: {0}")]
    NetworkError(String),

    #[error("Parse error: {0}")]
    ParseError(String),

    #[error("Validation error: {0}")]
    ValidationError(String),

    #[error("Transmission error: {0}")]
    TransmissionError(String),

    #[error("Unknown error: {0}")]
    Unknown(String),
}
