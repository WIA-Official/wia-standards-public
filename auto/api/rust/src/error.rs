//! Error types for WIA Auto API

use thiserror::Error;

/// Result type alias for WIA Auto operations
pub type Result<T> = std::result::Result<T, Error>;

/// Error types for WIA Auto API
#[derive(Debug, Error)]
pub enum Error {
    /// Validation error
    #[error("Validation error: {0}")]
    Validation(String),

    /// Serialization error
    #[error("Serialization error: {0}")]
    Serialization(#[from] serde_json::Error),

    /// Vehicle not found
    #[error("Vehicle not found: {0}")]
    VehicleNotFound(String),

    /// Profile not found
    #[error("Profile not found: {0}")]
    ProfileNotFound(String),

    /// Trip not found
    #[error("Trip not found: {0}")]
    TripNotFound(String),

    /// No matching vehicle available
    #[error("No accessible vehicle available matching requirements")]
    NoVehicleAvailable,

    /// Securement error
    #[error("Securement error: {0}")]
    SecurementError(String),

    /// Emergency handling error
    #[error("Emergency handling error: {0}")]
    EmergencyError(String),

    /// Communication error
    #[error("Communication error: {0}")]
    CommunicationError(String),

    /// Configuration error
    #[error("Configuration error: {0}")]
    ConfigError(String),

    /// Timeout error
    #[error("Operation timed out: {0}")]
    Timeout(String),

    /// Not supported
    #[error("Feature not supported: {0}")]
    NotSupported(String),

    /// Internal error
    #[error("Internal error: {0}")]
    Internal(String),

    /// WebSocket error
    #[error("WebSocket error: {0}")]
    WebSocket(String),

    /// Authentication error
    #[error("Authentication error: {0}")]
    Auth(String),

    /// Cryptographic error
    #[error("Cryptographic error: {0}")]
    Crypto(String),

    /// HTTP error
    #[error("HTTP error: {status} - {message}")]
    Http { status: u16, message: String },

    /// Rate limit exceeded
    #[error("Rate limit exceeded: retry after {retry_after_secs} seconds")]
    RateLimited { retry_after_secs: u64 },
}

impl Error {
    /// Create a validation error
    pub fn validation(msg: impl Into<String>) -> Self {
        Error::Validation(msg.into())
    }

    /// Create a vehicle not found error
    pub fn vehicle_not_found(id: impl Into<String>) -> Self {
        Error::VehicleNotFound(id.into())
    }

    /// Create a profile not found error
    pub fn profile_not_found(id: impl Into<String>) -> Self {
        Error::ProfileNotFound(id.into())
    }

    /// Create a trip not found error
    pub fn trip_not_found(id: impl Into<String>) -> Self {
        Error::TripNotFound(id.into())
    }

    /// Create a securement error
    pub fn securement(msg: impl Into<String>) -> Self {
        Error::SecurementError(msg.into())
    }

    /// Create an emergency error
    pub fn emergency(msg: impl Into<String>) -> Self {
        Error::EmergencyError(msg.into())
    }

    /// Create a communication error
    pub fn communication(msg: impl Into<String>) -> Self {
        Error::CommunicationError(msg.into())
    }

    /// Create a config error
    pub fn config(msg: impl Into<String>) -> Self {
        Error::ConfigError(msg.into())
    }

    /// Create a timeout error
    pub fn timeout(msg: impl Into<String>) -> Self {
        Error::Timeout(msg.into())
    }

    /// Create a not supported error
    pub fn not_supported(msg: impl Into<String>) -> Self {
        Error::NotSupported(msg.into())
    }

    /// Create an internal error
    pub fn internal(msg: impl Into<String>) -> Self {
        Error::Internal(msg.into())
    }

    /// Create a WebSocket error
    pub fn websocket(msg: impl Into<String>) -> Self {
        Error::WebSocket(msg.into())
    }

    /// Create an authentication error
    pub fn auth(msg: impl Into<String>) -> Self {
        Error::Auth(msg.into())
    }

    /// Create a cryptographic error
    pub fn crypto(msg: impl Into<String>) -> Self {
        Error::Crypto(msg.into())
    }

    /// Create an HTTP error
    pub fn http(status: u16, message: impl Into<String>) -> Self {
        Error::Http {
            status,
            message: message.into(),
        }
    }

    /// Create a rate limit error
    pub fn rate_limited(retry_after_secs: u64) -> Self {
        Error::RateLimited { retry_after_secs }
    }
}
