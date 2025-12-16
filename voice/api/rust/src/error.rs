//! Error types for WIA Voice-Sign API

use thiserror::Error;

/// Main error type for Voice-Sign operations
#[derive(Debug, Error)]
pub enum VoiceSignError {
    /// Audio processing errors
    #[error("Audio error: {0}")]
    Audio(#[from] AudioError),

    /// Transcription (ASR) errors
    #[error("Transcription error: {0}")]
    Transcription(#[from] TranscriptionError),

    /// Translation errors
    #[error("Translation error: {0}")]
    Translation(#[from] TranslationError),

    /// Pose generation errors
    #[error("Pose generation error: {0}")]
    PoseGeneration(#[from] PoseGenerationError),

    /// Rendering errors
    #[error("Rendering error: {0}")]
    Rendering(#[from] RenderingError),

    /// Configuration errors
    #[error("Configuration error: {0}")]
    Configuration(String),

    /// Validation errors
    #[error("Validation error: {0}")]
    Validation(#[from] ValidationError),

    /// IO errors
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    /// Serialization errors
    #[error("Serialization error: {0}")]
    Serialization(#[from] serde_json::Error),

    /// Internal errors
    #[error("Internal error: {0}")]
    Internal(String),
}

/// Audio processing errors
#[derive(Debug, Error)]
pub enum AudioError {
    #[error("Unsupported audio format: {0}")]
    UnsupportedFormat(String),

    #[error("Invalid sample rate: {0}Hz (expected one of: 8000, 16000, 22050, 44100, 48000)")]
    InvalidSampleRate(u32),

    #[error("Audio duration too long: {0}ms (max: 3600000ms)")]
    DurationTooLong(f64),

    #[error("Audio data is empty")]
    EmptyAudio,

    #[error("Failed to decode audio: {0}")]
    DecodingFailed(String),

    #[error("Audio stream error: {0}")]
    StreamError(String),
}

/// Transcription (ASR) errors
#[derive(Debug, Error)]
pub enum TranscriptionError {
    #[error("ASR model not found: {0}")]
    ModelNotFound(String),

    #[error("Language not supported: {0}")]
    UnsupportedLanguage(String),

    #[error("Transcription failed: {0}")]
    Failed(String),

    #[error("Transcription timeout after {0}ms")]
    Timeout(u64),

    #[error("No speech detected in audio")]
    NoSpeechDetected,
}

/// Translation errors
#[derive(Debug, Error)]
pub enum TranslationError {
    #[error("Translation model not found: {0}")]
    ModelNotFound(String),

    #[error("Sign language not supported: {0}")]
    UnsupportedSignLanguage(String),

    #[error("Translation failed: {0}")]
    Failed(String),

    #[error("No translation available for: {0}")]
    NoTranslation(String),

    #[error("Gloss not found: {0}")]
    GlossNotFound(String),

    #[error("Translation timeout after {0}ms")]
    Timeout(u64),
}

/// Pose generation errors
#[derive(Debug, Error)]
pub enum PoseGenerationError {
    #[error("Skeleton standard not supported: {0}")]
    UnsupportedSkeleton(String),

    #[error("Animation generation failed: {0}")]
    AnimationFailed(String),

    #[error("Invalid frame rate: {0}fps")]
    InvalidFrameRate(u32),

    #[error("Pose interpolation failed: {0}")]
    InterpolationFailed(String),
}

/// Rendering errors
#[derive(Debug, Error)]
pub enum RenderingError {
    #[error("Avatar not found: {0}")]
    AvatarNotFound(String),

    #[error("Unsupported output format: {0}")]
    UnsupportedFormat(String),

    #[error("Invalid resolution: {0}x{1}")]
    InvalidResolution(u32, u32),

    #[error("Rendering failed: {0}")]
    Failed(String),

    #[error("Rendering timeout after {0}ms")]
    Timeout(u64),
}

/// Validation errors
#[derive(Debug, Error)]
pub enum ValidationError {
    #[error("Missing required field: {0}")]
    MissingField(String),

    #[error("Invalid field value for '{field}': {message}")]
    InvalidValue { field: String, message: String },

    #[error("Value out of range for '{field}': {value} (expected {min} to {max})")]
    OutOfRange {
        field: String,
        value: String,
        min: String,
        max: String,
    },

    #[error("Either audio or text input is required")]
    NoInput,

    #[error("Cannot specify both audio and text input")]
    ConflictingInput,
}

/// Result type alias for Voice-Sign operations
pub type Result<T> = std::result::Result<T, VoiceSignError>;

impl VoiceSignError {
    /// Get error code for API responses
    pub fn code(&self) -> &'static str {
        match self {
            VoiceSignError::Audio(e) => e.code(),
            VoiceSignError::Transcription(e) => e.code(),
            VoiceSignError::Translation(e) => e.code(),
            VoiceSignError::PoseGeneration(e) => e.code(),
            VoiceSignError::Rendering(e) => e.code(),
            VoiceSignError::Configuration(_) => "CONFIGURATION_ERROR",
            VoiceSignError::Validation(e) => e.code(),
            VoiceSignError::Io(_) => "IO_ERROR",
            VoiceSignError::Serialization(_) => "SERIALIZATION_ERROR",
            VoiceSignError::Internal(_) => "INTERNAL_ERROR",
        }
    }

    /// Convert to ErrorInfo for API responses
    pub fn to_error_info(&self) -> crate::types::ErrorInfo {
        crate::types::ErrorInfo {
            code: self.code().to_string(),
            message: self.to_string(),
            details: None,
        }
    }
}

impl AudioError {
    pub fn code(&self) -> &'static str {
        match self {
            AudioError::UnsupportedFormat(_) => "AUDIO_UNSUPPORTED_FORMAT",
            AudioError::InvalidSampleRate(_) => "AUDIO_INVALID_SAMPLE_RATE",
            AudioError::DurationTooLong(_) => "AUDIO_DURATION_TOO_LONG",
            AudioError::EmptyAudio => "AUDIO_EMPTY",
            AudioError::DecodingFailed(_) => "AUDIO_DECODING_FAILED",
            AudioError::StreamError(_) => "AUDIO_STREAM_ERROR",
        }
    }
}

impl TranscriptionError {
    pub fn code(&self) -> &'static str {
        match self {
            TranscriptionError::ModelNotFound(_) => "ASR_MODEL_NOT_FOUND",
            TranscriptionError::UnsupportedLanguage(_) => "ASR_UNSUPPORTED_LANGUAGE",
            TranscriptionError::Failed(_) => "ASR_FAILED",
            TranscriptionError::Timeout(_) => "ASR_TIMEOUT",
            TranscriptionError::NoSpeechDetected => "ASR_NO_SPEECH",
        }
    }
}

impl TranslationError {
    pub fn code(&self) -> &'static str {
        match self {
            TranslationError::ModelNotFound(_) => "TRANSLATION_MODEL_NOT_FOUND",
            TranslationError::UnsupportedSignLanguage(_) => "TRANSLATION_UNSUPPORTED_LANGUAGE",
            TranslationError::Failed(_) => "TRANSLATION_FAILED",
            TranslationError::NoTranslation(_) => "TRANSLATION_NOT_AVAILABLE",
            TranslationError::GlossNotFound(_) => "TRANSLATION_GLOSS_NOT_FOUND",
            TranslationError::Timeout(_) => "TRANSLATION_TIMEOUT",
        }
    }
}

impl PoseGenerationError {
    pub fn code(&self) -> &'static str {
        match self {
            PoseGenerationError::UnsupportedSkeleton(_) => "POSE_UNSUPPORTED_SKELETON",
            PoseGenerationError::AnimationFailed(_) => "POSE_ANIMATION_FAILED",
            PoseGenerationError::InvalidFrameRate(_) => "POSE_INVALID_FRAME_RATE",
            PoseGenerationError::InterpolationFailed(_) => "POSE_INTERPOLATION_FAILED",
        }
    }
}

impl RenderingError {
    pub fn code(&self) -> &'static str {
        match self {
            RenderingError::AvatarNotFound(_) => "RENDER_AVATAR_NOT_FOUND",
            RenderingError::UnsupportedFormat(_) => "RENDER_UNSUPPORTED_FORMAT",
            RenderingError::InvalidResolution(_, _) => "RENDER_INVALID_RESOLUTION",
            RenderingError::Failed(_) => "RENDER_FAILED",
            RenderingError::Timeout(_) => "RENDER_TIMEOUT",
        }
    }
}

impl ValidationError {
    pub fn code(&self) -> &'static str {
        match self {
            ValidationError::MissingField(_) => "VALIDATION_MISSING_FIELD",
            ValidationError::InvalidValue { .. } => "VALIDATION_INVALID_VALUE",
            ValidationError::OutOfRange { .. } => "VALIDATION_OUT_OF_RANGE",
            ValidationError::NoInput => "VALIDATION_NO_INPUT",
            ValidationError::ConflictingInput => "VALIDATION_CONFLICTING_INPUT",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_codes() {
        let err = VoiceSignError::Audio(AudioError::EmptyAudio);
        assert_eq!(err.code(), "AUDIO_EMPTY");

        let err = VoiceSignError::Validation(ValidationError::NoInput);
        assert_eq!(err.code(), "VALIDATION_NO_INPUT");
    }

    #[test]
    fn test_error_display() {
        let err = AudioError::InvalidSampleRate(12000);
        assert!(err.to_string().contains("12000Hz"));

        let err = TranslationError::UnsupportedSignLanguage("XYZ".to_string());
        assert!(err.to_string().contains("XYZ"));
    }
}
