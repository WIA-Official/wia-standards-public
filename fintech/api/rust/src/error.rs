//! WIA Fintech Error Types
//!
//! Error definitions for the WIA Fintech SDK.

use thiserror::Error;

/// Fintech SDK error types
#[derive(Error, Debug)]
pub enum FintechError {
    /// Profile not found
    #[error("Profile not found: {0}")]
    ProfileNotFound(String),

    /// ATM not found
    #[error("ATM not found: {0}")]
    ATMNotFound(String),

    /// Invalid profile data
    #[error("Invalid profile data: {0}")]
    InvalidProfile(String),

    /// Validation error
    #[error("Validation error: {0}")]
    ValidationError(String),

    /// Compatibility check failed
    #[error("Compatibility check failed: {0}")]
    CompatibilityError(String),

    /// Storage error
    #[error("Storage error: {0}")]
    StorageError(String),

    /// Serialization error
    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),

    /// WIA integration error
    #[error("WIA integration error: {0}")]
    WIAIntegrationError(String),

    /// Notification delivery error
    #[error("Notification delivery error: {0}")]
    NotificationError(String),

    /// Authentication error
    #[error("Authentication error: {0}")]
    AuthenticationError(String),

    /// Configuration error
    #[error("Configuration error: {0}")]
    ConfigurationError(String),

    /// Network error
    #[error("Network error: {0}")]
    NetworkError(String),

    /// Internal error
    #[error("Internal error: {0}")]
    InternalError(String),
}

/// Result type alias for Fintech operations
pub type FintechResult<T> = Result<T, FintechError>;

/// Error codes for API responses
#[derive(Debug, Clone, Copy)]
pub enum ErrorCode {
    // Profile errors (1xxx)
    ProfileNotFound = 1001,
    ProfileInvalid = 1002,
    ProfileValidation = 1003,

    // ATM errors (2xxx)
    ATMNotFound = 2001,
    ATMOffline = 2002,
    ATMIncompatible = 2003,

    // Compatibility errors (3xxx)
    IncompatibleVisual = 3001,
    IncompatibleAuditory = 3002,
    IncompatibleMotor = 3003,
    IncompatibleCognitive = 3004,

    // WIA errors (4xxx)
    WIADeviceNotConnected = 4001,
    WIASyncFailed = 4002,
    WIAProfileMismatch = 4003,

    // Notification errors (5xxx)
    NotificationFailed = 5001,
    DeliveryFailed = 5002,
    ChannelUnavailable = 5003,

    // Auth errors (6xxx)
    AuthFailed = 6001,
    SessionExpired = 6002,
    InsufficientPermissions = 6003,

    // General errors (9xxx)
    InternalError = 9001,
    NetworkError = 9002,
    ConfigurationError = 9003,
}

impl ErrorCode {
    /// Get the numeric code
    pub fn code(&self) -> u32 {
        *self as u32
    }

    /// Get a human-readable message for the error code
    pub fn message(&self) -> &'static str {
        match self {
            ErrorCode::ProfileNotFound => "User profile not found",
            ErrorCode::ProfileInvalid => "Invalid profile data",
            ErrorCode::ProfileValidation => "Profile validation failed",
            ErrorCode::ATMNotFound => "ATM not found",
            ErrorCode::ATMOffline => "ATM is currently offline",
            ErrorCode::ATMIncompatible => "ATM does not meet accessibility requirements",
            ErrorCode::IncompatibleVisual => "Visual accessibility requirements not met",
            ErrorCode::IncompatibleAuditory => "Auditory accessibility requirements not met",
            ErrorCode::IncompatibleMotor => "Motor accessibility requirements not met",
            ErrorCode::IncompatibleCognitive => "Cognitive accessibility requirements not met",
            ErrorCode::WIADeviceNotConnected => "WIA device is not connected",
            ErrorCode::WIASyncFailed => "WIA profile sync failed",
            ErrorCode::WIAProfileMismatch => "WIA profile mismatch detected",
            ErrorCode::NotificationFailed => "Failed to send notification",
            ErrorCode::DeliveryFailed => "Notification delivery failed",
            ErrorCode::ChannelUnavailable => "Notification channel unavailable",
            ErrorCode::AuthFailed => "Authentication failed",
            ErrorCode::SessionExpired => "Session has expired",
            ErrorCode::InsufficientPermissions => "Insufficient permissions",
            ErrorCode::InternalError => "Internal server error",
            ErrorCode::NetworkError => "Network error occurred",
            ErrorCode::ConfigurationError => "Configuration error",
        }
    }
}

/// Error response structure for API
#[derive(Debug, serde::Serialize)]
pub struct ErrorResponse {
    pub code: u32,
    pub message: String,
    pub details: Option<String>,
    pub recovery_hint: Option<String>,
}

impl ErrorResponse {
    /// Create a new error response
    pub fn new(code: ErrorCode, details: Option<String>) -> Self {
        Self {
            code: code.code(),
            message: code.message().to_string(),
            details,
            recovery_hint: Self::get_recovery_hint(code),
        }
    }

    fn get_recovery_hint(code: ErrorCode) -> Option<String> {
        match code {
            ErrorCode::ProfileNotFound => Some("Please create a new profile or check the profile ID".to_string()),
            ErrorCode::ATMNotFound => Some("Try searching for nearby ATMs".to_string()),
            ErrorCode::ATMOffline => Some("Please try another ATM nearby".to_string()),
            ErrorCode::ATMIncompatible => Some("Search for ATMs with your required accessibility features".to_string()),
            ErrorCode::WIADeviceNotConnected => Some("Check your WIA device connection and try again".to_string()),
            ErrorCode::SessionExpired => Some("Please log in again".to_string()),
            ErrorCode::NetworkError => Some("Check your internet connection and try again".to_string()),
            _ => None,
        }
    }
}
