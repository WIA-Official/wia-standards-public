//! WIA Biotech Error Types
//!
//! This module defines error types for the WIA Biotechnology Standard API.

use thiserror::Error;

/// Main error type for WIA Biotech operations
#[derive(Error, Debug)]
pub enum BioError {
    /// Validation error for data structures
    #[error("Validation error: {0}")]
    ValidationError(String),

    /// Sequence-related errors
    #[error("Sequence error: {0}")]
    SequenceError(String),

    /// CRISPR experiment errors
    #[error("CRISPR error: {0}")]
    CrisprError(String),

    /// Protein structure errors
    #[error("Structure error: {0}")]
    StructureError(String),

    /// Synthetic biology part errors
    #[error("Part error: {0}")]
    PartError(String),

    /// File I/O errors
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    /// JSON serialization/deserialization errors
    #[error("JSON error: {0}")]
    JsonError(#[from] serde_json::Error),

    /// Invalid identifier format
    #[error("Invalid ID format: {0}")]
    InvalidIdError(String),

    /// Resource not found
    #[error("Not found: {0}")]
    NotFoundError(String),

    /// Configuration error
    #[error("Configuration error: {0}")]
    ConfigError(String),

    /// Adapter/device error
    #[error("Adapter error: {0}")]
    AdapterError(String),

    /// Conversion error between formats
    #[error("Conversion error: {0}")]
    ConversionError(String),

    /// Invalid operation
    #[error("Invalid operation: {0}")]
    InvalidOperation(String),
}

/// Result type alias for BioError
pub type BioResult<T> = Result<T, BioError>;

impl BioError {
    /// Create a validation error
    pub fn validation(msg: impl Into<String>) -> Self {
        BioError::ValidationError(msg.into())
    }

    /// Create a sequence error
    pub fn sequence(msg: impl Into<String>) -> Self {
        BioError::SequenceError(msg.into())
    }

    /// Create a CRISPR error
    pub fn crispr(msg: impl Into<String>) -> Self {
        BioError::CrisprError(msg.into())
    }

    /// Create a structure error
    pub fn structure(msg: impl Into<String>) -> Self {
        BioError::StructureError(msg.into())
    }

    /// Create a part error
    pub fn part(msg: impl Into<String>) -> Self {
        BioError::PartError(msg.into())
    }

    /// Create a not found error
    pub fn not_found(msg: impl Into<String>) -> Self {
        BioError::NotFoundError(msg.into())
    }

    /// Create an adapter error
    pub fn adapter(msg: impl Into<String>) -> Self {
        BioError::AdapterError(msg.into())
    }

    /// Create a conversion error
    pub fn conversion(msg: impl Into<String>) -> Self {
        BioError::ConversionError(msg.into())
    }
}

/// Validation error details
#[derive(Debug, Clone)]
pub struct ValidationDetails {
    pub field: String,
    pub message: String,
    pub value: Option<String>,
}

impl ValidationDetails {
    pub fn new(field: impl Into<String>, message: impl Into<String>) -> Self {
        Self {
            field: field.into(),
            message: message.into(),
            value: None,
        }
    }

    pub fn with_value(mut self, value: impl Into<String>) -> Self {
        self.value = Some(value.into());
        self
    }
}

impl std::fmt::Display for ValidationDetails {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match &self.value {
            Some(v) => write!(f, "{}: {} (value: {})", self.field, self.message, v),
            None => write!(f, "{}: {}", self.field, self.message),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = BioError::validation("Invalid sequence format");
        assert_eq!(err.to_string(), "Validation error: Invalid sequence format");
    }

    #[test]
    fn test_validation_details() {
        let details = ValidationDetails::new("sequence", "Invalid characters")
            .with_value("ATCGX");
        assert!(details.to_string().contains("sequence"));
        assert!(details.to_string().contains("ATCGX"));
    }
}
