//! Error types for 3D Touch SDK
//!
//! 弘益人間 - Clear errors help all developers

use thiserror::Error;

/// Result type alias for SDK operations
pub type Result<T> = std::result::Result<T, Error>;

/// Error types for 3D touch operations
#[derive(Error, Debug)]
pub enum Error {
    /// Network communication error
    #[error("Network error: {0}")]
    NetworkError(String),

    /// API returned an error
    #[error("API error: {0}")]
    ApiError(String),

    /// Failed to parse response
    #[error("Parse error: {0}")]
    ParseError(String),

    /// Invalid input provided
    #[error("Invalid input: {0}")]
    InvalidInput(String),

    /// Touch event validation error
    #[error("Touch event validation error: {0}")]
    TouchEventError(String),

    /// Haptic feedback error
    #[error("Haptic error: {0}")]
    HapticError(String),

    /// Gesture recognition error
    #[error("Gesture recognition error: {0}")]
    GestureError(String),

    /// Calibration error
    #[error("Calibration error: {0}")]
    CalibrationError(String),

    /// Device capability error
    #[error("Device capability error: {0}")]
    DeviceError(String),

    /// Pressure sensing error
    #[error("Pressure error: {0}")]
    PressureError(String),

    /// Authentication error
    #[error("Authentication error: {0}")]
    AuthenticationError(String),

    /// Resource not found
    #[error("Resource not found: {0}")]
    NotFound(String),

    /// Unauthorized access
    #[error("Unauthorized: {0}")]
    Unauthorized(String),

    /// Internal error
    #[error("Internal error: {0}")]
    InternalError(String),
}

impl Error {
    /// Check if error is retryable
    pub fn is_retryable(&self) -> bool {
        matches!(self, Error::NetworkError(_) | Error::ApiError(_))
    }

    /// Get error code for logging
    pub fn error_code(&self) -> &str {
        match self {
            Error::NetworkError(_) => "NETWORK_ERROR",
            Error::ApiError(_) => "API_ERROR",
            Error::ParseError(_) => "PARSE_ERROR",
            Error::InvalidInput(_) => "INVALID_INPUT",
            Error::TouchEventError(_) => "TOUCH_EVENT_ERROR",
            Error::HapticError(_) => "HAPTIC_ERROR",
            Error::GestureError(_) => "GESTURE_ERROR",
            Error::CalibrationError(_) => "CALIBRATION_ERROR",
            Error::DeviceError(_) => "DEVICE_ERROR",
            Error::PressureError(_) => "PRESSURE_ERROR",
            Error::AuthenticationError(_) => "AUTHENTICATION_ERROR",
            Error::NotFound(_) => "NOT_FOUND",
            Error::Unauthorized(_) => "UNAUTHORIZED",
            Error::InternalError(_) => "INTERNAL_ERROR",
        }
    }
}
