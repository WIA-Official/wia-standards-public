//! Error types for WIA XR Accessibility API
//!
//! Comprehensive error handling for XR accessibility operations.

use thiserror::Error;

/// Main error type for XR accessibility operations
#[derive(Error, Debug)]
pub enum XRAccessibilityError {
    /// Profile-related errors
    #[error("Profile error: {0}")]
    Profile(#[from] ProfileError),

    /// Device-related errors
    #[error("Device error: {0}")]
    Device(#[from] DeviceError),

    /// Adaptation errors
    #[error("Adaptation error: {0}")]
    Adaptation(#[from] AdaptationError),

    /// Integration errors with other WIA systems
    #[error("WIA integration error: {0}")]
    WIAIntegration(#[from] WIAIntegrationError),

    /// Serialization/deserialization errors
    #[error("Serialization error: {0}")]
    Serialization(#[from] serde_json::Error),

    /// I/O errors
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// Configuration errors
    #[error("Configuration error: {0}")]
    Configuration(String),

    /// Validation errors
    #[error("Validation error: {0}")]
    Validation(String),

    /// Runtime errors
    #[error("Runtime error: {0}")]
    Runtime(String),
}

/// Profile-specific errors
#[derive(Error, Debug)]
pub enum ProfileError {
    #[error("Profile not found: {0}")]
    NotFound(String),

    #[error("Profile already exists: {0}")]
    AlreadyExists(String),

    #[error("Invalid profile data: {0}")]
    InvalidData(String),

    #[error("Profile version mismatch: expected {expected}, got {actual}")]
    VersionMismatch { expected: String, actual: String },

    #[error("Profile migration failed: {0}")]
    MigrationFailed(String),

    #[error("Profile schema validation failed: {0}")]
    SchemaValidation(String),

    #[error("Profile merge conflict: {0}")]
    MergeConflict(String),

    #[error("Profile export failed: {0}")]
    ExportFailed(String),

    #[error("Profile import failed: {0}")]
    ImportFailed(String),
}

/// Device-specific errors
#[derive(Error, Debug)]
pub enum DeviceError {
    #[error("Device not found: {0}")]
    NotFound(String),

    #[error("Device not supported: {0}")]
    NotSupported(String),

    #[error("Device connection failed: {0}")]
    ConnectionFailed(String),

    #[error("Device disconnected: {0}")]
    Disconnected(String),

    #[error("Device capability not available: {0}")]
    CapabilityUnavailable(String),

    #[error("Device firmware version not supported: {device} requires {required}, has {actual}")]
    FirmwareVersion {
        device: String,
        required: String,
        actual: String,
    },

    #[error("Device configuration error: {0}")]
    Configuration(String),

    #[error("Device timeout: {0}")]
    Timeout(String),

    #[error("Device resource exhausted: {0}")]
    ResourceExhausted(String),
}

/// Adaptation-specific errors
#[derive(Error, Debug)]
pub enum AdaptationError {
    #[error("Adaptation not applicable: {0}")]
    NotApplicable(String),

    #[error("Adaptation failed: {0}")]
    Failed(String),

    #[error("Adaptation conflict: {adaptations:?}")]
    Conflict { adaptations: Vec<String> },

    #[error("Adaptation resource unavailable: {0}")]
    ResourceUnavailable(String),

    #[error("Adaptation requires feature: {0}")]
    RequiresFeature(String),

    #[error("Adaptation performance impact too high: {impact}% exceeds threshold {threshold}%")]
    PerformanceImpact { impact: u32, threshold: u32 },

    #[error("Caption generation failed: {0}")]
    CaptionFailed(String),

    #[error("Audio description generation failed: {0}")]
    AudioDescriptionFailed(String),

    #[error("Sign language avatar failed: {0}")]
    SignLanguageFailed(String),

    #[error("Haptic feedback failed: {0}")]
    HapticFailed(String),

    #[error("Voice control adaptation failed: {0}")]
    VoiceControlFailed(String),

    #[error("Eye tracking adaptation failed: {0}")]
    EyeTrackingFailed(String),
}

/// WIA integration errors
#[derive(Error, Debug)]
pub enum WIAIntegrationError {
    #[error("Exoskeleton integration error: {0}")]
    Exoskeleton(String),

    #[error("Bionic eye integration error: {0}")]
    BionicEye(String),

    #[error("Voice-Sign integration error: {0}")]
    VoiceSign(String),

    #[error("WIA service unavailable: {0}")]
    ServiceUnavailable(String),

    #[error("WIA protocol version mismatch: {0}")]
    ProtocolMismatch(String),

    #[error("WIA authentication failed: {0}")]
    AuthenticationFailed(String),

    #[error("WIA sync failed: {0}")]
    SyncFailed(String),

    #[error("WIA event dispatch failed: {0}")]
    EventDispatchFailed(String),
}

/// Result type alias for XR accessibility operations
pub type Result<T> = std::result::Result<T, XRAccessibilityError>;

/// Result type alias for profile operations
pub type ProfileResult<T> = std::result::Result<T, ProfileError>;

/// Result type alias for device operations
pub type DeviceResult<T> = std::result::Result<T, DeviceError>;

/// Result type alias for adaptation operations
pub type AdaptationResult<T> = std::result::Result<T, AdaptationError>;

/// Result type alias for WIA integration operations
pub type WIAResult<T> = std::result::Result<T, WIAIntegrationError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let error = XRAccessibilityError::Profile(ProfileError::NotFound("test-profile".into()));
        assert!(error.to_string().contains("test-profile"));

        let device_error = DeviceError::FirmwareVersion {
            device: "Quest 3".into(),
            required: "2.0".into(),
            actual: "1.5".into(),
        };
        assert!(device_error.to_string().contains("Quest 3"));
    }

    #[test]
    fn test_error_conversion() {
        let profile_error = ProfileError::NotFound("test".into());
        let xr_error: XRAccessibilityError = profile_error.into();
        assert!(matches!(xr_error, XRAccessibilityError::Profile(_)));
    }
}
