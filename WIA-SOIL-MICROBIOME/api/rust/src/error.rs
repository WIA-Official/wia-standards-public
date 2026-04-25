//! Error types for the WIA-SOIL-MICROBIOME SDK
//!
//! This module contains all error types used throughout the SDK.

use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Result type for SDK operations
pub type Result<T> = std::result::Result<T, Error>;

/// Main error type for the SDK
#[derive(Debug, Error)]
pub enum Error {
    /// HTTP/Network error
    #[error("Network error: {0}")]
    Network(#[from] reqwest::Error),

    /// JSON serialization/deserialization error
    #[error("JSON error: {0}")]
    Json(#[from] serde_json::Error),

    /// Authentication error
    #[error("Authentication error: {0}")]
    Authentication(String),

    /// Resource not found
    #[error("Resource not found: {0}")]
    NotFound(String),

    /// Rate limit exceeded
    #[error("Rate limit exceeded")]
    RateLimited {
        /// Seconds to wait before retrying
        retry_after: Option<u64>,
    },

    /// API error from the server
    #[error("API error: {0}")]
    Api(#[from] ApiError),

    /// WebSocket error
    #[error("WebSocket error: {0}")]
    WebSocket(String),

    /// Invalid configuration
    #[error("Invalid configuration: {0}")]
    InvalidConfig(String),

    /// Unknown error
    #[error("Unknown error: {0}")]
    Unknown(String),
}

/// API error response from the server
#[derive(Debug, Clone, Serialize, Deserialize, Error)]
#[error("API Error {code}: {message}")]
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

/// Detailed error information
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

impl From<tokio_tungstenite::tungstenite::Error> for Error {
    fn from(err: tokio_tungstenite::tungstenite::Error) -> Self {
        Error::WebSocket(err.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let error = Error::NotFound("sample-123".to_string());
        assert_eq!(error.to_string(), "Resource not found: sample-123");
    }

    #[test]
    fn test_api_error() {
        let api_error = ApiError {
            code: "INVALID_REQUEST".to_string(),
            message: "Invalid sample data".to_string(),
            details: None,
            request_id: "req-123".to_string(),
        };
        assert_eq!(api_error.code, "INVALID_REQUEST");
    }
}
