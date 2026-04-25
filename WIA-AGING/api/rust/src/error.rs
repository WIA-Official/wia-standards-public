//! Error types for the WIA-AGING SDK
//!
//! This module defines all error types that can occur when using the SDK.

use serde::{Deserialize, Serialize};
use std::fmt;

/// Result type alias for WIA-AGING SDK operations
pub type Result<T> = std::result::Result<T, Error>;

/// API error details
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApiErrorDetails {
    /// Field that caused the error
    #[serde(skip_serializing_if = "Option::is_none")]
    pub field: Option<String>,
    /// Invalid value
    #[serde(skip_serializing_if = "Option::is_none")]
    pub value: Option<serde_json::Value>,
    /// Constraint that was violated
    #[serde(skip_serializing_if = "Option::is_none")]
    pub constraint: Option<String>,
}

/// API error response structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApiError {
    /// Error code
    pub code: String,
    /// Error message
    pub message: String,
    /// Error details
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<ApiErrorDetails>,
    /// Request ID for tracking
    pub request_id: String,
}

/// SDK Error type
#[derive(Debug)]
pub enum Error {
    /// HTTP request failed
    Http(reqwest::Error),

    /// API returned an error
    Api(ApiError),

    /// JSON serialization/deserialization failed
    Json(serde_json::Error),

    /// WebSocket error
    WebSocket(String),

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
            Error::Api(e) => write!(f, "API error [{}]: {}", e.code, e.message),
            Error::Json(e) => write!(f, "JSON error: {}", e),
            Error::WebSocket(msg) => write!(f, "WebSocket error: {}", msg),
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

impl From<tokio_tungstenite::tungstenite::Error> for Error {
    fn from(err: tokio_tungstenite::tungstenite::Error) -> Self {
        Error::WebSocket(err.to_string())
    }
}

/// Error codes returned by the API
pub mod codes {
    /// Invalid request body
    pub const INVALID_REQUEST: &str = "INVALID_REQUEST";
    /// Validation failed
    pub const VALIDATION_ERROR: &str = "VALIDATION_ERROR";
    /// Not authenticated
    pub const UNAUTHORIZED: &str = "UNAUTHORIZED";
    /// No permission
    pub const FORBIDDEN: &str = "FORBIDDEN";
    /// Resource not found
    pub const NOT_FOUND: &str = "NOT_FOUND";
    /// Resource conflict
    pub const CONFLICT: &str = "CONFLICT";
    /// Rate limited
    pub const RATE_LIMITED: &str = "RATE_LIMITED";
    /// Internal server error
    pub const INTERNAL_ERROR: &str = "INTERNAL_ERROR";
}
