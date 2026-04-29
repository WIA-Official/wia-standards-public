//! Error types for intergenerational

use thiserror::Error;

pub type Result<T> = std::result::Result<T, IntergenerationalError>;

#[derive(Error, Debug)]
pub enum IntergenerationalError {
    #[error("Network error: {0}")]
    NetworkError(String),

    #[error("Parse error: {0}")]
    ParseError(String),

    #[error("Validation error: {0}")]
    ValidationError(String),

    #[error("Unknown error: {0}")]
    Unknown(String),
}
