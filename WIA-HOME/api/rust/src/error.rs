//! Error types for WIA-HOME SDK
//!
//! 弘益人間 - Clear error handling for better user experience

use thiserror::Error;

/// Result type alias for WIA-HOME operations
pub type Result<T> = std::result::Result<T, HomeError>;

/// WIA-HOME error types
#[derive(Error, Debug)]
pub enum HomeError {
    /// Invalid URL format
    #[error("Invalid URL: {0}")]
    InvalidUrl(String),

    /// Invalid API key
    #[error("Invalid API key: {0}")]
    InvalidApiKey(String),

    /// HTTP request error
    #[error("HTTP error: {0}")]
    HttpError(reqwest::StatusCode),

    /// Network error
    #[error("Network error: {0}")]
    NetworkError(#[from] reqwest::Error),

    /// JSON parsing error
    #[error("JSON error: {0}")]
    JsonError(#[from] serde_json::Error),

    /// Device not found
    #[error("Device not found: {0}")]
    DeviceNotFound(String),

    /// Scene not found
    #[error("Scene not found: {0}")]
    SceneNotFound(String),

    /// Validation error
    #[error("Validation error: {0}")]
    ValidationError(String),

    /// API error
    #[error("API error: {0}")]
    ApiError(String),

    /// Authentication error
    #[error("Authentication failed")]
    AuthenticationError,

    /// Permission denied
    #[error("Permission denied: {0}")]
    PermissionDenied(String),

    /// Device offline
    #[error("Device is offline: {0}")]
    DeviceOffline(String),

    /// Timeout error
    #[error("Operation timeout")]
    Timeout,

    /// Generic error
    #[error("Error: {0}")]
    Other(String),
}

impl HomeError {
    /// Check if error is retryable
    pub fn is_retryable(&self) -> bool {
        matches!(
            self,
            HomeError::NetworkError(_) | HomeError::Timeout | HomeError::DeviceOffline(_)
        )
    }

    /// Get error code for logging/monitoring
    pub fn error_code(&self) -> &str {
        match self {
            HomeError::InvalidUrl(_) => "INVALID_URL",
            HomeError::InvalidApiKey(_) => "INVALID_API_KEY",
            HomeError::HttpError(_) => "HTTP_ERROR",
            HomeError::NetworkError(_) => "NETWORK_ERROR",
            HomeError::JsonError(_) => "JSON_ERROR",
            HomeError::DeviceNotFound(_) => "DEVICE_NOT_FOUND",
            HomeError::SceneNotFound(_) => "SCENE_NOT_FOUND",
            HomeError::ValidationError(_) => "VALIDATION_ERROR",
            HomeError::ApiError(_) => "API_ERROR",
            HomeError::AuthenticationError => "AUTH_ERROR",
            HomeError::PermissionDenied(_) => "PERMISSION_DENIED",
            HomeError::DeviceOffline(_) => "DEVICE_OFFLINE",
            HomeError::Timeout => "TIMEOUT",
            HomeError::Other(_) => "OTHER",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let error = HomeError::DeviceNotFound("test-device".to_string());
        assert!(error.to_string().contains("test-device"));
    }

    #[test]
    fn test_retryable() {
        assert!(HomeError::Timeout.is_retryable());
        assert!(!HomeError::InvalidApiKey("test".to_string()).is_retryable());
    }

    #[test]
    fn test_error_code() {
        assert_eq!(HomeError::Timeout.error_code(), "TIMEOUT");
    }
}
