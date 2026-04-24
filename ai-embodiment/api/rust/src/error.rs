//! Error types for ai-embodiment

use thiserror::Error;

pub type Result<T> = std::result::Result<T, AiEmbodimentError>;

#[derive(Error, Debug)]
pub enum AiEmbodimentError {
    #[error("Network error: {0}")]
    NetworkError(String),

    #[error("Parse error: {0}")]
    ParseError(String),

    #[error("Validation error: {0}")]
    ValidationError(String),

    #[error("Unknown error: {0}")]
    Unknown(String),
}
