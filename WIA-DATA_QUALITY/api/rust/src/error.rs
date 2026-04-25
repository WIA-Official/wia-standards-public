//! Error types for data quality

use thiserror::Error;

#[derive(Error, Debug)]
pub enum Error {
    #[error("Network error: {0}")]
    NetworkError(String),
    #[error("API error: {0}")]
    ApiError(String),
    #[error("Parse error: {0}")]
    ParseError(String),
    #[error("Validation error: {0}")]
    ValidationError(String),
    #[error("{0}")]
    Other(String),
}

pub type Result<T> = std::result::Result<T, Error>;
