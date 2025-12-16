//! Error types for the WIA CareBot SDK

use thiserror::Error;

/// Result type alias for CareBot operations
pub type CareBotResult<T> = Result<T, CareBotError>;

/// Errors that can occur in CareBot operations
#[derive(Error, Debug)]
pub enum CareBotError {
    /// Invalid recipient ID format
    #[error("Invalid recipient ID: {0}")]
    InvalidRecipientId(String),

    /// Invalid device ID format
    #[error("Invalid device ID: {0}")]
    InvalidDeviceId(String),

    /// Recipient not found
    #[error("Recipient not found: {0}")]
    RecipientNotFound(String),

    /// Device not found
    #[error("Device not found: {0}")]
    DeviceNotFound(String),

    /// Invalid emotion value (must be 0.0-1.0)
    #[error("Invalid emotion value: {0} (must be between 0.0 and 1.0)")]
    InvalidEmotionValue(f64),

    /// Invalid vital sign reading
    #[error("Invalid vital sign: {0}")]
    InvalidVitalSign(String),

    /// Emergency response failed
    #[error("Emergency response failed: {0}")]
    EmergencyResponseFailed(String),

    /// Medication schedule conflict
    #[error("Medication schedule conflict: {0}")]
    MedicationConflict(String),

    /// Cognitive assessment error
    #[error("Cognitive assessment error: {0}")]
    CognitiveAssessmentError(String),

    /// Communication error
    #[error("Communication error: {0}")]
    CommunicationError(String),

    /// Serialization error
    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),

    /// Privacy consent required
    #[error("Privacy consent required for: {0}")]
    ConsentRequired(String),

    /// Invalid configuration
    #[error("Invalid configuration: {0}")]
    InvalidConfiguration(String),

    /// Sensor error
    #[error("Sensor error: {0}")]
    SensorError(String),

    /// AI model error
    #[error("AI model error: {0}")]
    AiModelError(String),
}
