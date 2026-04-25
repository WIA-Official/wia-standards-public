//! Error types for amr

use thiserror::Error;

pub type Result<T> = std::result::Result<T, AmrError>;

#[derive(Error, Debug)]
pub enum AmrError {
    #[error("Network error: {0}")]
    NetworkError(String),

    #[error("Parse error: {0}")]
    ParseError(String),

    #[error("Validation error: {0}")]
    ValidationError(String),

    #[error("Unknown error: {0}")]
    Unknown(String),
}
