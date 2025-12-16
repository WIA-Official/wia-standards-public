//! WIA Smart Home Error Types
//! 弘益人間 - Benefit All Humanity

use thiserror::Error;
use uuid::Uuid;

/// WIA Smart Home API Error
#[derive(Error, Debug)]
pub enum SmartHomeError {
    #[error("Device not found: {0}")]
    DeviceNotFound(Uuid),

    #[error("Zone not found: {0}")]
    ZoneNotFound(Uuid),

    #[error("Home not found: {0}")]
    HomeNotFound(Uuid),

    #[error("User profile not found: {0}")]
    ProfileNotFound(Uuid),

    #[error("Automation not found: {0}")]
    AutomationNotFound(Uuid),

    #[error("Device offline: {0}")]
    DeviceOffline(Uuid),

    #[error("Device error: {0}")]
    DeviceError(String),

    #[error("Command failed: {0}")]
    CommandFailed(String),

    #[error("Modality not supported: {0}")]
    ModalityNotSupported(String),

    #[error("Accessibility feature not available: {0}")]
    AccessibilityFeatureNotAvailable(String),

    #[error("Invalid configuration: {0}")]
    InvalidConfiguration(String),

    #[error("Validation error: {0}")]
    ValidationError(String),

    #[error("Permission denied: {0}")]
    PermissionDenied(String),

    #[error("Automation error: {0}")]
    AutomationError(String),

    #[error("Notification error: {0}")]
    NotificationError(String),

    #[error("Communication error: {0}")]
    CommunicationError(String),

    #[error("Timeout: {0}")]
    Timeout(String),

    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),

    #[error("Internal error: {0}")]
    Internal(String),
}

/// Result type for Smart Home operations
pub type Result<T> = std::result::Result<T, SmartHomeError>;
