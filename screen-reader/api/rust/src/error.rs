//! Error types for WIA Screen Reader

use thiserror::Error;

/// WIA Screen Reader errors
#[derive(Debug, Error)]
pub enum WIAError {
    /// Serialization error
    #[error("Serialization error: {0}")]
    Serialization(String),

    /// Invalid input
    #[error("Invalid input: {0}")]
    InvalidInput(String),

    /// Unsupported language
    #[error("Unsupported language: {0}")]
    UnsupportedLanguage(String),

    /// Configuration error
    #[error("Configuration error: {0}")]
    Configuration(String),
}

/// Result type for WIA operations
pub type WIAResult<T> = Result<T, WIAError>;
