//! Error types for cosmic communication

use thiserror::Error;

/// Error type for cosmic communication operations
#[derive(Error, Debug)]
pub enum Error {
    /// Network-related errors
    #[error("Network error: {0}")]
    NetworkError(String),

    /// API errors
    #[error("API error: {0}")]
    ApiError(String),

    /// Parse/deserialization errors
    #[error("Parse error: {0}")]
    ParseError(String),

    /// Validation errors
    #[error("Validation error: {0}")]
    ValidationError(String),

    /// Signal processing errors
    #[error("Signal processing error: {0}")]
    SignalError(String),

    /// Message encoding errors
    #[error("Encoding error: {0}")]
    EncodingError(String),

    /// Configuration errors
    #[error("Configuration error: {0}")]
    ConfigError(String),

    /// Authentication errors
    #[error("Authentication error: {0}")]
    AuthError(String),

    /// Generic errors
    #[error("{0}")]
    Other(String),
}

/// Result type alias
pub type Result<T> = std::result::Result<T, Error>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = Error::NetworkError("connection failed".to_string());
        assert_eq!(err.to_string(), "Network error: connection failed");
    }

    #[test]
    fn test_validation_error() {
        let err = Error::ValidationError("invalid signal".to_string());
        assert!(err.to_string().contains("Validation error"));
    }
}
