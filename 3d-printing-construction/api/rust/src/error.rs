//! Error types for 3D Printing Construction SDK
//!
//! 弘益人間 - Clear errors help all developers

use thiserror::Error;

/// Result type alias for SDK operations
pub type Result<T> = std::result::Result<T, Error>;

/// Error types for 3D printing construction operations
#[derive(Error, Debug)]
pub enum Error {
    /// Network communication error
    #[error("Network error: {0}")]
    NetworkError(String),

    /// API returned an error
    #[error("API error: {0}")]
    ApiError(String),

    /// Failed to parse response
    #[error("Parse error: {0}")]
    ParseError(String),

    /// Invalid input provided
    #[error("Invalid input: {0}")]
    InvalidInput(String),

    /// Building design validation error
    #[error("Design validation error: {0}")]
    DesignValidationError(String),

    /// Material specification error
    #[error("Material error: {0}")]
    MaterialError(String),

    /// Printer configuration error
    #[error("Printer configuration error: {0}")]
    PrinterConfigError(String),

    /// Quality check failed
    #[error("Quality check failed: {0}")]
    QualityCheckError(String),

    /// Safety violation detected
    #[error("Safety violation: {0}")]
    SafetyViolation(String),

    /// Print job error
    #[error("Print job error: {0}")]
    PrintJobError(String),

    /// Authentication error
    #[error("Authentication error: {0}")]
    AuthenticationError(String),

    /// Resource not found
    #[error("Resource not found: {0}")]
    NotFound(String),

    /// Unauthorized access
    #[error("Unauthorized: {0}")]
    Unauthorized(String),

    /// Internal error
    #[error("Internal error: {0}")]
    InternalError(String),
}

impl Error {
    /// Check if error is retryable
    pub fn is_retryable(&self) -> bool {
        matches!(self, Error::NetworkError(_) | Error::ApiError(_))
    }

    /// Get error code for logging
    pub fn error_code(&self) -> &str {
        match self {
            Error::NetworkError(_) => "NETWORK_ERROR",
            Error::ApiError(_) => "API_ERROR",
            Error::ParseError(_) => "PARSE_ERROR",
            Error::InvalidInput(_) => "INVALID_INPUT",
            Error::DesignValidationError(_) => "DESIGN_VALIDATION_ERROR",
            Error::MaterialError(_) => "MATERIAL_ERROR",
            Error::PrinterConfigError(_) => "PRINTER_CONFIG_ERROR",
            Error::QualityCheckError(_) => "QUALITY_CHECK_ERROR",
            Error::SafetyViolation(_) => "SAFETY_VIOLATION",
            Error::PrintJobError(_) => "PRINT_JOB_ERROR",
            Error::AuthenticationError(_) => "AUTHENTICATION_ERROR",
            Error::NotFound(_) => "NOT_FOUND",
            Error::Unauthorized(_) => "UNAUTHORIZED",
            Error::InternalError(_) => "INTERNAL_ERROR",
        }
    }
}
