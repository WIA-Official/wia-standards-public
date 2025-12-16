//! WIA Education Error Types
//! 弘益人間 - Education for Everyone

use thiserror::Error;
use uuid::Uuid;

/// WIA Education API Error
#[derive(Error, Debug)]
pub enum EduError {
    #[error("Profile not found: {0}")]
    ProfileNotFound(Uuid),

    #[error("Course not found: {0}")]
    CourseNotFound(Uuid),

    #[error("Module not found: {0}")]
    ModuleNotFound(Uuid),

    #[error("Content not found: {0}")]
    ContentNotFound(Uuid),

    #[error("Assessment not found: {0}")]
    AssessmentNotFound(Uuid),

    #[error("Question not found: {0}")]
    QuestionNotFound(Uuid),

    #[error("Invalid profile: {0}")]
    InvalidProfile(String),

    #[error("Invalid course: {0}")]
    InvalidCourse(String),

    #[error("Invalid assessment: {0}")]
    InvalidAssessment(String),

    #[error("Invalid settings: {0}")]
    InvalidSettings(String),

    #[error("Validation error: {0}")]
    ValidationError(String),

    #[error("Invalid configuration: {0}")]
    InvalidConfiguration(String),

    #[error("Feature not supported: {0}")]
    FeatureNotSupported(String),

    #[error("Accommodation not available: {0}")]
    AccommodationNotAvailable(String),

    #[error("Import error: {0}")]
    ImportError(String),

    #[error("Export error: {0}")]
    ExportError(String),

    #[error("Schema version mismatch: expected {expected}, got {actual}")]
    VersionMismatch { expected: String, actual: String },

    #[error("Accessibility requirement not met: {0}")]
    AccessibilityRequirementNotMet(String),

    #[error("WCAG conformance error: {criterion} - {description}")]
    WCAGConformanceError {
        criterion: String,
        description: String,
    },

    #[error("WIA integration error: {0}")]
    WIAIntegrationError(String),

    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    #[error("Not found: {0}")]
    NotFound(String),

    #[error("Internal error: {0}")]
    Internal(String),
}

/// Result type for Education operations
pub type Result<T> = std::result::Result<T, EduError>;
