//! Data Exporters
//!
//! Export WIA Security data to various SIEM, analytics, and reporting systems.

pub mod splunk;
pub mod elasticsearch;
pub mod pdf;
pub mod grafana;

pub use splunk::*;
pub use elasticsearch::*;
pub use pdf::*;
pub use grafana::*;

use thiserror::Error;

/// Export error types
#[derive(Debug, Error)]
pub enum ExportError {
    #[error("Connection error: {0}")]
    ConnectionError(String),

    #[error("Authentication error: {0}")]
    AuthError(String),

    #[error("Serialization error: {0}")]
    SerializationError(String),

    #[error("Rate limit exceeded")]
    RateLimitError,

    #[error("Export failed: {0}")]
    ExportFailed(String),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    #[error("Template error: {0}")]
    TemplateError(String),
}

/// Export result type
pub type ExportResult<T> = Result<T, ExportError>;

/// Export status
#[derive(Debug, Clone)]
pub struct ExportStatus {
    pub success: bool,
    pub records_exported: u64,
    pub errors: Vec<String>,
    pub duration_ms: u64,
}
