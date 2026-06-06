//! Error types for the WIA Perception Clarity SDK

use thiserror::Error;

/// Result type alias for WIA Perception Clarity operations
pub type Result<T> = std::result::Result<T, Error>;

/// Error types for the WIA Perception Clarity SDK
#[derive(Debug, Error)]
pub enum Error {
    /// Validation error (Phase 1 §6 consistency rules)
    #[error("Validation error: {0}")]
    Validation(String),

    /// Serialization / deserialization error
    #[error("Serialization error: {0}")]
    Serialization(#[from] serde_json::Error),

    /// PCI weight set does not sum to 1.00 (±0.001) — Phase 1 §6.2
    #[error("Weights sum invalid: occlusion+distance+mtf must equal 1.00 (±0.001), got {0}")]
    WeightsSumInvalid(f64),

    /// Reported `state` is inconsistent with the reported `pci` band — Phase 1 §6.3
    #[error("State/PCI conflict: pci {pci} maps to {expected:?} but report says {actual:?}")]
    StatePciConflict {
        pci: u8,
        expected: String,
        actual: String,
    },

    /// A value outside its allowed [0.0, 1.0] range
    #[error("Out of range: {field} must be within {min}..={max}, got {value}")]
    OutOfRange {
        field: String,
        min: f64,
        max: f64,
        value: f64,
    },

    /// Agent not found
    #[error("Agent not found: {0}")]
    AgentNotFound(String),

    /// Sensor not found
    #[error("Sensor not found: {0}")]
    SensorNotFound(String),

    /// Authentication failed (missing / invalid / expired bearer token)
    #[error("Authentication error: {0}")]
    Auth(String),

    /// Insufficient scope (e.g. a reporting token used for a query)
    #[error("Insufficient scope: {0}")]
    InsufficientScope(String),

    /// Message signature verification failed
    #[error("Signature error: {0}")]
    Signature(String),

    /// WebSocket transport error
    #[error("WebSocket error: {0}")]
    WebSocket(String),

    /// HTTP transport error
    #[error("HTTP error: {status} - {message}")]
    Http { status: u16, message: String },

    /// Rate limit exceeded (Phase 2 §11)
    #[error("Rate limit exceeded: retry after {retry_after_secs} seconds")]
    RateLimited { retry_after_secs: u64 },

    /// Internal error
    #[error("Internal error: {0}")]
    Internal(String),
}

impl Error {
    /// Create a validation error
    pub fn validation(msg: impl Into<String>) -> Self {
        Error::Validation(msg.into())
    }

    /// Create an agent-not-found error
    pub fn agent_not_found(id: impl Into<String>) -> Self {
        Error::AgentNotFound(id.into())
    }

    /// Create a sensor-not-found error
    pub fn sensor_not_found(id: impl Into<String>) -> Self {
        Error::SensorNotFound(id.into())
    }

    /// Create an authentication error
    pub fn auth(msg: impl Into<String>) -> Self {
        Error::Auth(msg.into())
    }

    /// Create a signature error
    pub fn signature(msg: impl Into<String>) -> Self {
        Error::Signature(msg.into())
    }

    /// Create a WebSocket error
    pub fn websocket(msg: impl Into<String>) -> Self {
        Error::WebSocket(msg.into())
    }

    /// Create an HTTP error
    pub fn http(status: u16, message: impl Into<String>) -> Self {
        Error::Http {
            status,
            message: message.into(),
        }
    }

    /// Create a rate-limit error
    pub fn rate_limited(retry_after_secs: u64) -> Self {
        Error::RateLimited { retry_after_secs }
    }

    /// Create an internal error
    pub fn internal(msg: impl Into<String>) -> Self {
        Error::Internal(msg.into())
    }
}
