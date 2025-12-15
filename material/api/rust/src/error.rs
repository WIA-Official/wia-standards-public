//! Error types for WIA Material SDK
//!
//! This module provides comprehensive error handling for material science operations.

use thiserror::Error;

/// Main error type for WIA Material operations
#[derive(Error, Debug)]
pub enum MaterialError {
    /// Validation error for material data
    #[error("Validation error: {0}")]
    ValidationError(String),

    /// Serialization/Deserialization error
    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),

    /// Invalid material type
    #[error("Invalid material type: {0}")]
    InvalidMaterialType(String),

    /// Invalid material ID format
    #[error("Invalid material ID format: {0}")]
    InvalidMaterialId(String),

    /// Missing required field
    #[error("Missing required field: {0}")]
    MissingField(String),

    /// Value out of range
    #[error("Value out of range for {field}: {value} (expected {min} to {max})")]
    OutOfRange {
        field: String,
        value: f64,
        min: f64,
        max: f64,
    },

    /// Invalid unit
    #[error("Invalid unit for {field}: expected {expected}, got {actual}")]
    InvalidUnit {
        field: String,
        expected: String,
        actual: String,
    },

    /// Adapter error
    #[error("Adapter error: {0}")]
    AdapterError(String),

    /// Connection error
    #[error("Connection error: {0}")]
    ConnectionError(String),

    /// Timeout error
    #[error("Operation timed out")]
    Timeout,

    /// Not found error
    #[error("Material not found: {0}")]
    NotFound(String),

    /// Transport error
    #[error("Transport error: {0}")]
    Transport(String),

    /// Protocol error
    #[error("Protocol error: {0}")]
    Protocol(String),

    /// Parse error
    #[error("Parse error: {0}")]
    Parse(String),

    /// Authentication error
    #[error("Authentication error: {0}")]
    AuthError(String),

    /// Rate limit error
    #[error("Rate limit exceeded")]
    RateLimited,

    /// IO error
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    /// Configuration error
    #[error("Configuration error: {0}")]
    ConfigError(String),

    /// Physical constraint violation
    #[error("Physical constraint violation: {0}")]
    PhysicalConstraint(String),

    /// Unknown error
    #[error("Unknown error: {0}")]
    Unknown(String),
}

/// Result type alias for Material operations
pub type MaterialResult<T> = Result<T, MaterialError>;

impl MaterialError {
    /// Create a validation error with field details
    pub fn validation(field: &str, message: &str) -> Self {
        MaterialError::ValidationError(format!("{}: {}", field, message))
    }

    /// Create an out of range error
    pub fn out_of_range(field: &str, value: f64, min: f64, max: f64) -> Self {
        MaterialError::OutOfRange {
            field: field.to_string(),
            value,
            min,
            max,
        }
    }

    /// Create a physical constraint error for superconductors
    pub fn superconductor_constraint(message: &str) -> Self {
        MaterialError::PhysicalConstraint(format!("Superconductor: {}", message))
    }

    /// Create a physical constraint error for topological insulators
    pub fn topological_constraint(message: &str) -> Self {
        MaterialError::PhysicalConstraint(format!("Topological Insulator: {}", message))
    }
}

/// Validation helper for temperature values
pub fn validate_temperature(value: f64, field: &str) -> MaterialResult<()> {
    if value < 0.0 {
        return Err(MaterialError::out_of_range(field, value, 0.0, f64::MAX));
    }
    Ok(())
}

/// Validation helper for pressure values
pub fn validate_pressure(value: f64, field: &str) -> MaterialResult<()> {
    if value < 0.0 {
        return Err(MaterialError::out_of_range(field, value, 0.0, f64::MAX));
    }
    Ok(())
}

/// Validation helper for percentage values
pub fn validate_percentage(value: f64, field: &str) -> MaterialResult<()> {
    if value < 0.0 || value > 100.0 {
        return Err(MaterialError::out_of_range(field, value, 0.0, 100.0));
    }
    Ok(())
}

/// Validation helper for confidence values
pub fn validate_confidence(value: f64, field: &str) -> MaterialResult<()> {
    if value < 0.0 || value > 1.0 {
        return Err(MaterialError::out_of_range(field, value, 0.0, 1.0));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validation_error() {
        let err = MaterialError::validation("temperature_k", "must be positive");
        assert!(err.to_string().contains("temperature_k"));
    }

    #[test]
    fn test_out_of_range_error() {
        let err = MaterialError::out_of_range("confidence", 1.5, 0.0, 1.0);
        assert!(err.to_string().contains("1.5"));
        assert!(err.to_string().contains("0"));
        assert!(err.to_string().contains("1"));
    }

    #[test]
    fn test_validate_temperature() {
        assert!(validate_temperature(300.0, "test").is_ok());
        assert!(validate_temperature(-10.0, "test").is_err());
    }

    #[test]
    fn test_validate_percentage() {
        assert!(validate_percentage(50.0, "test").is_ok());
        assert!(validate_percentage(101.0, "test").is_err());
        assert!(validate_percentage(-1.0, "test").is_err());
    }
}
