//! Error types for the WIA Climate library

use thiserror::Error;

/// Result type alias using ClimateError
pub type Result<T> = std::result::Result<T, ClimateError>;

/// Main error type for the WIA Climate library
#[derive(Error, Debug)]
pub enum ClimateError {
    /// Validation error for data that doesn't conform to the schema
    #[error("Validation error: {0}")]
    Validation(String),

    /// Serialization/deserialization error
    #[error("Serialization error: {0}")]
    Serialization(#[from] serde_json::Error),

    /// Invalid data type
    #[error("Invalid data type: {0}")]
    InvalidDataType(String),

    /// Missing required field
    #[error("Missing required field: {0}")]
    MissingField(String),

    /// Invalid coordinate values
    #[error("Invalid coordinates: latitude={lat}, longitude={lon}")]
    InvalidCoordinates {
        /// Latitude value
        lat: f64,
        /// Longitude value
        lon: f64,
    },

    /// Value out of valid range
    #[error("Value out of range: {field} = {value} (expected {min} to {max})")]
    OutOfRange {
        /// Field name
        field: String,
        /// Actual value
        value: f64,
        /// Minimum allowed value
        min: f64,
        /// Maximum allowed value
        max: f64,
    },

    /// Adapter error (simulator, sensor, etc.)
    #[error("Adapter error: {0}")]
    Adapter(String),

    /// IO error
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    /// Parse error for timestamps
    #[error("Parse error: {0}")]
    Parse(String),

    /// Builder error for incomplete message construction
    #[error("Builder error: {0}")]
    Builder(String),

    /// Connection error for transport layer
    #[error("Connection error: {0}")]
    ConnectionError(String),

    /// Serialization error (string version for protocol)
    #[error("Serialization error: {0}")]
    SerializationError(String),

    /// Protocol error
    #[error("Protocol error: {0}")]
    ProtocolError(String),

    /// Unknown error
    #[error("Unknown error: {0}")]
    Unknown(String),
}

impl ClimateError {
    /// Create a validation error
    pub fn validation(msg: impl Into<String>) -> Self {
        ClimateError::Validation(msg.into())
    }

    /// Create a missing field error
    pub fn missing_field(field: impl Into<String>) -> Self {
        ClimateError::MissingField(field.into())
    }

    /// Create an out of range error
    pub fn out_of_range(field: impl Into<String>, value: f64, min: f64, max: f64) -> Self {
        ClimateError::OutOfRange {
            field: field.into(),
            value,
            min,
            max,
        }
    }

    /// Create an adapter error
    pub fn adapter(msg: impl Into<String>) -> Self {
        ClimateError::Adapter(msg.into())
    }

    /// Create a builder error
    pub fn builder(msg: impl Into<String>) -> Self {
        ClimateError::Builder(msg.into())
    }

    /// Create a connection error
    pub fn connection(msg: impl Into<String>) -> Self {
        ClimateError::ConnectionError(msg.into())
    }

    /// Create a protocol error
    pub fn protocol(msg: impl Into<String>) -> Self {
        ClimateError::ProtocolError(msg.into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validation_error() {
        let err = ClimateError::validation("Invalid value");
        assert!(err.to_string().contains("Validation error"));
    }

    #[test]
    fn test_out_of_range_error() {
        let err = ClimateError::out_of_range("temperature", 150.0, -50.0, 100.0);
        assert!(err.to_string().contains("out of range"));
        assert!(err.to_string().contains("temperature"));
    }

    #[test]
    fn test_invalid_coordinates_error() {
        let err = ClimateError::InvalidCoordinates { lat: 100.0, lon: 200.0 };
        assert!(err.to_string().contains("Invalid coordinates"));
    }
}
