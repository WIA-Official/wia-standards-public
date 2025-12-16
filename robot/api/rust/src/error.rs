//! Error types for WIA Robot SDK
//!
//! This module defines all error types used throughout the library.

use thiserror::Error;

/// Main error type for robot operations
#[derive(Error, Debug)]
pub enum RobotError {
    /// Invalid parameter provided
    #[error("Invalid parameter: {0}")]
    InvalidParameter(String),

    /// Safety violation detected
    #[error("Safety violation: {0}")]
    SafetyViolation(String),

    /// Control system error
    #[error("Control error: {0}")]
    ControlError(String),

    /// Sensor error
    #[error("Sensor error: {0}")]
    SensorError(String),

    /// Actuator error
    #[error("Actuator error: {0}")]
    ActuatorError(String),

    /// Communication error
    #[error("Communication error: {0}")]
    CommunicationError(String),

    /// Calibration required
    #[error("Calibration required: {0}")]
    CalibrationRequired(String),

    /// Emergency stop activated
    #[error("Emergency stop activated")]
    EmergencyStop,

    /// Device not found
    #[error("Device not found: {0}")]
    DeviceNotFound(String),

    /// Operation timeout
    #[error("Operation timeout: {0}")]
    Timeout(String),

    /// Serialization error
    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),

    /// IO error
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
}

/// Result type alias for robot operations
pub type RobotResult<T> = std::result::Result<T, RobotError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = RobotError::InvalidParameter("test param".to_string());
        assert_eq!(format!("{}", err), "Invalid parameter: test param");

        let err = RobotError::EmergencyStop;
        assert_eq!(format!("{}", err), "Emergency stop activated");
    }

    #[test]
    fn test_error_from_json() {
        let json_err = serde_json::from_str::<i32>("invalid").unwrap_err();
        let robot_err: RobotError = json_err.into();
        assert!(matches!(robot_err, RobotError::SerializationError(_)));
    }
}
