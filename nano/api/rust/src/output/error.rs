//! Output error types

use std::path::PathBuf;
use thiserror::Error;

/// Output operation errors
#[derive(Debug, Error)]
pub enum OutputError {
    /// Invalid molecule structure
    #[error("Invalid molecule: {0}")]
    InvalidMolecule(String),

    /// Unsupported output format
    #[error("Unsupported format: {0}")]
    UnsupportedFormat(String),

    /// IO error
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    /// Format-specific error
    #[error("Format error: {0}")]
    FormatError(String),

    /// Missing required field
    #[error("Missing required field: {0}")]
    MissingField(String),

    /// Invalid coordinate
    #[error("Invalid coordinate: {0}")]
    InvalidCoordinate(String),

    /// File write error
    #[error("Failed to write file: {path}")]
    WriteError { path: PathBuf },

    /// Adapter not found
    #[error("Output adapter not found: {0}")]
    AdapterNotFound(String),
}

/// Result type for output operations
pub type OutputResult<T> = Result<T, OutputError>;
