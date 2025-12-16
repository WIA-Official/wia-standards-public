//! Error types for WIA Medical Device Accessibility

use thiserror::Error;

/// Result type alias for WIA Medical operations
pub type Result<T> = std::result::Result<T, MedicalError>;

/// WIA Medical error types
#[derive(Error, Debug)]
pub enum MedicalError {
    /// Profile not found
    #[error("Profile not found: {0}")]
    ProfileNotFound(String),

    /// Device not found
    #[error("Device not found: {0}")]
    DeviceNotFound(String),

    /// Invalid profile data
    #[error("Invalid profile: {0}")]
    InvalidProfile(String),

    /// Invalid device configuration
    #[error("Invalid device configuration: {0}")]
    InvalidDeviceConfig(String),

    /// Validation error
    #[error("Validation error: {0}")]
    ValidationError(String),

    /// Serialization error
    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),

    /// IO error
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    /// Adapter error
    #[error("Adapter error: {0}")]
    AdapterError(String),

    /// Connection error
    #[error("Connection error: {0}")]
    ConnectionError(String),

    /// Alarm system error
    #[error("Alarm system error: {0}")]
    AlarmError(String),

    /// WIA integration error
    #[error("WIA integration error: {0}")]
    WIAIntegrationError(String),

    /// Regulatory compliance error
    #[error("Regulatory compliance error: {0}")]
    ComplianceError(String),

    /// Accessibility score calculation error
    #[error("Accessibility score error: {0}")]
    AccessibilityScoreError(String),

    /// Configuration error
    #[error("Configuration error: {0}")]
    ConfigError(String),

    /// Timeout error
    #[error("Operation timed out: {0}")]
    TimeoutError(String),

    /// Permission denied
    #[error("Permission denied: {0}")]
    PermissionDenied(String),

    /// Not supported
    #[error("Operation not supported: {0}")]
    NotSupported(String),
}

impl MedicalError {
    /// Check if error is recoverable
    pub fn is_recoverable(&self) -> bool {
        matches!(
            self,
            MedicalError::TimeoutError(_)
                | MedicalError::ConnectionError(_)
                | MedicalError::AdapterError(_)
        )
    }

    /// Get error code for logging/monitoring
    pub fn error_code(&self) -> &'static str {
        match self {
            MedicalError::ProfileNotFound(_) => "E001",
            MedicalError::DeviceNotFound(_) => "E002",
            MedicalError::InvalidProfile(_) => "E003",
            MedicalError::InvalidDeviceConfig(_) => "E004",
            MedicalError::ValidationError(_) => "E005",
            MedicalError::SerializationError(_) => "E006",
            MedicalError::IoError(_) => "E007",
            MedicalError::AdapterError(_) => "E008",
            MedicalError::ConnectionError(_) => "E009",
            MedicalError::AlarmError(_) => "E010",
            MedicalError::WIAIntegrationError(_) => "E011",
            MedicalError::ComplianceError(_) => "E012",
            MedicalError::AccessibilityScoreError(_) => "E013",
            MedicalError::ConfigError(_) => "E014",
            MedicalError::TimeoutError(_) => "E015",
            MedicalError::PermissionDenied(_) => "E016",
            MedicalError::NotSupported(_) => "E017",
        }
    }
}
