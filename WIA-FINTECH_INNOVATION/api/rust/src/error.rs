//! Error types for the WIA-FINTECH_INNOVATION SDK
//!
//! This module defines all error types that can occur when using the SDK.

use std::fmt;

/// Result type alias for WIA-FINTECH_INNOVATION SDK operations
pub type Result<T> = std::result::Result<T, Error>;

/// SDK Error type
#[derive(Debug)]
pub enum Error {
    /// HTTP request failed
    Http(reqwest::Error),

    /// JSON serialization/deserialization failed
    Json(serde_json::Error),

    /// Invalid configuration
    Config(String),

    /// Authentication failed
    Authentication(String),

    /// Rate limited
    RateLimited {
        /// Retry after (seconds)
        retry_after: Option<u64>,
    },

    /// Resource not found
    NotFound(String),

    /// Validation error
    Validation(String),

    /// Timeout
    Timeout,

    /// Connection error
    Connection(String),

    /// Unknown error
    Unknown(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Http(e) => write!(f, "HTTP error: {}", e),
            Error::Json(e) => write!(f, "JSON error: {}", e),
            Error::Config(msg) => write!(f, "Configuration error: {}", msg),
            Error::Authentication(msg) => write!(f, "Authentication error: {}", msg),
            Error::RateLimited { retry_after } => {
                if let Some(secs) = retry_after {
                    write!(f, "Rate limited. Retry after {} seconds", secs)
                } else {
                    write!(f, "Rate limited")
                }
            }
            Error::NotFound(resource) => write!(f, "Not found: {}", resource),
            Error::Validation(msg) => write!(f, "Validation error: {}", msg),
            Error::Timeout => write!(f, "Request timed out"),
            Error::Connection(msg) => write!(f, "Connection error: {}", msg),
            Error::Unknown(msg) => write!(f, "Unknown error: {}", msg),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Http(e) => Some(e),
            Error::Json(e) => Some(e),
            _ => None,
        }
    }
}

impl From<reqwest::Error> for Error {
    fn from(err: reqwest::Error) -> Self {
        if err.is_timeout() {
            Error::Timeout
        } else if err.is_connect() {
            Error::Connection(err.to_string())
        } else {
            Error::Http(err)
        }
    }
}

impl From<serde_json::Error> for Error {
    fn from(err: serde_json::Error) -> Self {
        Error::Json(err)
    }
}
