//! WIA Health Standard Error Types
//!
//! Comprehensive error handling for the WIA Health API

use thiserror::Error;

/// Result type alias for WIA Health operations
pub type Result<T> = std::result::Result<T, HealthError>;

/// Main error type for WIA Health operations
#[derive(Error, Debug)]
pub enum HealthError {
    /// Validation error
    #[error("Validation error: {0}")]
    Validation(String),

    /// Serialization error
    #[error("Serialization error: {0}")]
    Serialization(#[from] serde_json::Error),

    /// Missing required field
    #[error("Missing required field: {0}")]
    MissingField(String),

    /// Invalid value
    #[error("Invalid value for {field}: {message}")]
    InvalidValue { field: String, message: String },

    /// Out of range error
    #[error("Value out of range for {field}: {value} (expected {min}-{max})")]
    OutOfRange {
        field: String,
        value: f64,
        min: f64,
        max: f64,
    },

    /// Profile not found
    #[error("Health profile not found: {0}")]
    ProfileNotFound(String),

    /// Measurement error
    #[error("Measurement error: {0}")]
    Measurement(String),

    /// Calculation error
    #[error("Calculation error: {0}")]
    Calculation(String),

    /// Digital twin error
    #[error("Digital twin error: {0}")]
    DigitalTwin(String),

    /// Simulation error
    #[error("Simulation error: {0}")]
    Simulation(String),

    /// Data stream error
    #[error("Data stream error: {0}")]
    DataStream(String),

    /// IO error
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    /// UUID error
    #[error("UUID error: {0}")]
    Uuid(#[from] uuid::Error),

    /// Configuration error
    #[error("Configuration error: {0}")]
    Configuration(String),

    /// Adapter error
    #[error("Adapter error: {0}")]
    Adapter(String),

    /// Protocol error
    #[error("Protocol error: {0}")]
    ProtocolError(String),

    /// Connection error
    #[error("Connection error: {0}")]
    ConnectionError(String),

    /// Serialization error (string variant for protocol)
    #[error("Serialization error: {0}")]
    SerializationError(String),

    /// Internal error
    #[error("Internal error: {0}")]
    Internal(String),
}

impl HealthError {
    /// Create a validation error
    pub fn validation(msg: impl Into<String>) -> Self {
        Self::Validation(msg.into())
    }

    /// Create a missing field error
    pub fn missing_field(field: impl Into<String>) -> Self {
        Self::MissingField(field.into())
    }

    /// Create an invalid value error
    pub fn invalid_value(field: impl Into<String>, message: impl Into<String>) -> Self {
        Self::InvalidValue {
            field: field.into(),
            message: message.into(),
        }
    }

    /// Create an out of range error
    pub fn out_of_range(field: impl Into<String>, value: f64, min: f64, max: f64) -> Self {
        Self::OutOfRange {
            field: field.into(),
            value,
            min,
            max,
        }
    }

    /// Create a calculation error
    pub fn calculation(msg: impl Into<String>) -> Self {
        Self::Calculation(msg.into())
    }

    /// Create a digital twin error
    pub fn digital_twin(msg: impl Into<String>) -> Self {
        Self::DigitalTwin(msg.into())
    }

    /// Create a simulation error
    pub fn simulation(msg: impl Into<String>) -> Self {
        Self::Simulation(msg.into())
    }

    /// Create an adapter error
    pub fn adapter(msg: impl Into<String>) -> Self {
        Self::Adapter(msg.into())
    }

    /// Create an internal error
    pub fn internal(msg: impl Into<String>) -> Self {
        Self::Internal(msg.into())
    }

    /// Create a protocol error
    pub fn protocol(msg: impl Into<String>) -> Self {
        Self::ProtocolError(msg.into())
    }

    /// Create a connection error
    pub fn connection(msg: impl Into<String>) -> Self {
        Self::ConnectionError(msg.into())
    }

    /// Check if error is a validation error
    pub fn is_validation(&self) -> bool {
        matches!(self, Self::Validation(_))
    }

    /// Check if error is a connection error
    pub fn is_connection(&self) -> bool {
        matches!(self, Self::ConnectionError(_))
    }

    /// Check if error is a protocol error
    pub fn is_protocol(&self) -> bool {
        matches!(self, Self::ProtocolError(_))
    }

    /// Check if error is recoverable
    pub fn is_recoverable(&self) -> bool {
        matches!(
            self,
            Self::Validation(_)
                | Self::MissingField(_)
                | Self::InvalidValue { .. }
                | Self::OutOfRange { .. }
                | Self::ConnectionError(_)
        )
    }
}

/// Error codes for API responses
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorCode {
    ValidationError,
    NotFound,
    Unauthorized,
    Forbidden,
    Conflict,
    InternalError,
    ServiceUnavailable,
}

impl ErrorCode {
    /// Get HTTP status code equivalent
    pub fn http_status(&self) -> u16 {
        match self {
            Self::ValidationError => 400,
            Self::NotFound => 404,
            Self::Unauthorized => 401,
            Self::Forbidden => 403,
            Self::Conflict => 409,
            Self::InternalError => 500,
            Self::ServiceUnavailable => 503,
        }
    }

    /// Get error code string
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::ValidationError => "VALIDATION_ERROR",
            Self::NotFound => "NOT_FOUND",
            Self::Unauthorized => "UNAUTHORIZED",
            Self::Forbidden => "FORBIDDEN",
            Self::Conflict => "CONFLICT",
            Self::InternalError => "INTERNAL_ERROR",
            Self::ServiceUnavailable => "SERVICE_UNAVAILABLE",
        }
    }
}

impl From<&HealthError> for ErrorCode {
    fn from(error: &HealthError) -> Self {
        match error {
            HealthError::Validation(_)
            | HealthError::MissingField(_)
            | HealthError::InvalidValue { .. }
            | HealthError::OutOfRange { .. } => ErrorCode::ValidationError,
            HealthError::ProfileNotFound(_) => ErrorCode::NotFound,
            _ => ErrorCode::InternalError,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validation_error() {
        let error = HealthError::validation("invalid age");
        assert!(error.is_validation());
        assert!(error.is_recoverable());
    }

    #[test]
    fn test_out_of_range_error() {
        let error = HealthError::out_of_range("age", 200.0, 0.0, 150.0);
        assert!(error.is_recoverable());
        assert!(error.to_string().contains("200"));
    }

    #[test]
    fn test_error_code_conversion() {
        let error = HealthError::validation("test");
        let code: ErrorCode = (&error).into();
        assert_eq!(code, ErrorCode::ValidationError);
        assert_eq!(code.http_status(), 400);
    }
}
