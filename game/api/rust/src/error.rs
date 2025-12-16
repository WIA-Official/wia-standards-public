//! WIA Game Error Types
//! 弘益人間 - Gaming for Everyone

use thiserror::Error;
use uuid::Uuid;

/// WIA Game API Error
#[derive(Error, Debug)]
pub enum GameError {
    #[error("Profile not found: {0}")]
    ProfileNotFound(Uuid),

    #[error("Preset not found: {0}")]
    PresetNotFound(Uuid),

    #[error("Controller config not found: {0}")]
    ControllerConfigNotFound(Uuid),

    #[error("Game not found: {0}")]
    GameNotFound(Uuid),

    #[error("Not found: {0}")]
    NotFound(String),

    #[error("Invalid profile: {0}")]
    InvalidProfile(String),

    #[error("Invalid settings: {0}")]
    InvalidSettings(String),

    #[error("Validation error: {0}")]
    ValidationError(String),

    #[error("Invalid configuration: {0}")]
    InvalidConfiguration(String),

    #[error("Feature not supported: {0}")]
    FeatureNotSupported(String),

    #[error("Device not connected: {0}")]
    DeviceNotConnected(String),

    #[error("Device error: {0}")]
    DeviceError(String),

    #[error("Import error: {0}")]
    ImportError(String),

    #[error("Export error: {0}")]
    ExportError(String),

    #[error("Schema version mismatch: expected {expected}, got {actual}")]
    VersionMismatch { expected: String, actual: String },

    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    #[error("Internal error: {0}")]
    Internal(String),
}

/// Result type for Game operations
pub type Result<T> = std::result::Result<T, GameError>;
