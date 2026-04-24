//! Error types for WIA Space Standard
//!
//! This module defines all error types used throughout the WIA Space API.

use thiserror::Error;

/// Result type alias for WIA Space operations
pub type SpaceResult<T> = Result<T, SpaceError>;

/// Main error type for WIA Space operations
#[derive(Error, Debug)]
pub enum SpaceError {
    /// Invalid technology category
    #[error("Invalid technology category: {0}")]
    InvalidCategory(String),

    /// Invalid technology readiness level
    #[error("Invalid TRL: {0} (must be 1-9)")]
    InvalidTRL(i32),

    /// Validation error
    #[error("Validation error: {0}")]
    ValidationError(String),

    /// Serialization error
    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),

    /// Invalid value range
    #[error("Value out of range: {field} = {value} (expected {min}..{max})")]
    OutOfRange {
        field: String,
        value: f64,
        min: f64,
        max: f64,
    },

    /// Missing required field
    #[error("Missing required field: {0}")]
    MissingField(String),

    /// Invalid ID format
    #[error("Invalid ID format: {0}")]
    InvalidId(String),

    /// Physics constraint violation
    #[error("Physics constraint violation: {0}")]
    PhysicsViolation(String),

    /// IO error
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    /// Parse error
    #[error("Parse error: {0}")]
    ParseError(String),

    /// Simulation error
    #[error("Simulation error: {0}")]
    SimulationError(String),

    /// Not implemented
    #[error("Not implemented: {0}")]
    NotImplemented(String),
}

impl SpaceError {
    /// Create a validation error
    pub fn validation(msg: impl Into<String>) -> Self {
        SpaceError::ValidationError(msg.into())
    }

    /// Create an out of range error
    pub fn out_of_range(field: impl Into<String>, value: f64, min: f64, max: f64) -> Self {
        SpaceError::OutOfRange {
            field: field.into(),
            value,
            min,
            max,
        }
    }

    /// Create a physics violation error
    pub fn physics(msg: impl Into<String>) -> Self {
        SpaceError::PhysicsViolation(msg.into())
    }

    /// Create a missing field error
    pub fn missing(field: impl Into<String>) -> Self {
        SpaceError::MissingField(field.into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = SpaceError::InvalidTRL(10);
        assert_eq!(err.to_string(), "Invalid TRL: 10 (must be 1-9)");
    }

    #[test]
    fn test_out_of_range() {
        let err = SpaceError::out_of_range("velocity_c", 1.5, 0.0, 1.0);
        assert!(err.to_string().contains("velocity_c"));
    }
}
