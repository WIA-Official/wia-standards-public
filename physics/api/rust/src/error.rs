//! Error types for WIA Physics SDK
//!
//! Provides comprehensive error handling for physics data operations.

use thiserror::Error;

/// Main error type for WIA Physics operations
#[derive(Error, Debug)]
pub enum PhysicsError {
    /// Validation error for physics data
    #[error("Validation error: {0}")]
    Validation(String),

    /// Measurement error (e.g., invalid uncertainty)
    #[error("Measurement error: {0}")]
    Measurement(String),

    /// Unit conversion error
    #[error("Unit conversion error: cannot convert {from} to {to}")]
    UnitConversion { from: String, to: String },

    /// Schema validation error
    #[error("Schema validation error: {0}")]
    SchemaValidation(String),

    /// Serialization/deserialization error
    #[error("Serialization error: {0}")]
    Serialization(#[from] serde_json::Error),

    /// I/O error
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// Simulation error
    #[error("Simulation error: {0}")]
    Simulation(String),

    /// Domain-specific errors
    #[error("Fusion error: {0}")]
    Fusion(String),

    #[error("Particle physics error: {0}")]
    ParticlePhysics(String),

    #[error("Dark matter error: {0}")]
    DarkMatter(String),

    #[error("Antimatter error: {0}")]
    Antimatter(String),

    #[error("Time crystal error: {0}")]
    TimeCrystal(String),

    #[error("Quantum gravity error: {0}")]
    QuantumGravity(String),

    /// Not found error
    #[error("Not found: {0}")]
    NotFound(String),

    /// Configuration error
    #[error("Configuration error: {0}")]
    Configuration(String),

    /// Protocol error
    #[error("Protocol error: {0}")]
    Protocol(String),

    /// Parse error
    #[error("Parse error: {0}")]
    Parse(String),

    /// Connection error
    #[error("Connection error: {0}")]
    Connection(String),

    /// Timeout error
    #[error("Timeout error: {0}")]
    Timeout(String),
}

/// Result type alias for physics operations
pub type PhysicsResult<T> = Result<T, PhysicsError>;

impl PhysicsError {
    /// Create a validation error
    pub fn validation<S: Into<String>>(msg: S) -> Self {
        PhysicsError::Validation(msg.into())
    }

    /// Create a measurement error
    pub fn measurement<S: Into<String>>(msg: S) -> Self {
        PhysicsError::Measurement(msg.into())
    }

    /// Create a unit conversion error
    pub fn unit_conversion<S: Into<String>>(from: S, to: S) -> Self {
        PhysicsError::UnitConversion {
            from: from.into(),
            to: to.into(),
        }
    }

    /// Create a simulation error
    pub fn simulation<S: Into<String>>(msg: S) -> Self {
        PhysicsError::Simulation(msg.into())
    }

    /// Create a not found error
    pub fn not_found<S: Into<String>>(msg: S) -> Self {
        PhysicsError::NotFound(msg.into())
    }

    /// Create a protocol error
    pub fn protocol<S: Into<String>>(msg: S) -> Self {
        PhysicsError::Protocol(msg.into())
    }

    /// Create a parse error
    pub fn parse<S: Into<String>>(msg: S) -> Self {
        PhysicsError::Parse(msg.into())
    }

    /// Create a serialization error from a string
    pub fn serialization<S: Into<String>>(msg: S) -> Self {
        // Convert string message to a JSON error for consistency
        PhysicsError::Parse(msg.into())
    }

    /// Create a connection error
    pub fn connection<S: Into<String>>(msg: S) -> Self {
        PhysicsError::Connection(msg.into())
    }

    /// Create a timeout error
    pub fn timeout<S: Into<String>>(msg: S) -> Self {
        PhysicsError::Timeout(msg.into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = PhysicsError::validation("invalid temperature");
        assert_eq!(err.to_string(), "Validation error: invalid temperature");
    }

    #[test]
    fn test_unit_conversion_error() {
        let err = PhysicsError::unit_conversion("keV", "kg");
        assert_eq!(
            err.to_string(),
            "Unit conversion error: cannot convert keV to kg"
        );
    }
}
