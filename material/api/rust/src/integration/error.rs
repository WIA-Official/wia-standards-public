//! Integration error types

use thiserror::Error;

/// Integration error types
#[derive(Error, Debug)]
pub enum IntegrationError {
    /// Provider not found
    #[error("Provider not found: {0}")]
    ProviderNotFound(String),

    /// Provider not connected
    #[error("Provider not connected: {0}")]
    ProviderNotConnected(String),

    /// External ID not found
    #[error("External ID not found: {0}")]
    ExternalIdNotFound(String),

    /// Unsupported format
    #[error("Unsupported format: {0}")]
    UnsupportedFormat(String),

    /// Parse error
    #[error("Parse error: {0}")]
    ParseError(String),

    /// Export error
    #[error("Export error: {0}")]
    ExportError(String),

    /// Import error
    #[error("Import error: {0}")]
    ImportError(String),

    /// Conversion error
    #[error("Conversion error: {0}")]
    ConversionError(String),

    /// Network error
    #[error("Network error: {0}")]
    NetworkError(String),

    /// Authentication error
    #[error("Authentication error: {0}")]
    AuthError(String),

    /// Rate limited
    #[error("Rate limited")]
    RateLimited,

    /// Invalid configuration
    #[error("Invalid configuration: {0}")]
    InvalidConfig(String),

    /// Missing structure data
    #[error("Missing structure data for export")]
    MissingStructure,

    /// IO error
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    /// Material error
    #[error("Material error: {0}")]
    MaterialError(#[from] crate::error::MaterialError),
}

/// Result type for integration operations
pub type IntegrationResult<T> = Result<T, IntegrationError>;

impl IntegrationError {
    /// Create a parse error with details
    pub fn parse(format: &str, details: &str) -> Self {
        IntegrationError::ParseError(format!("{}: {}", format, details))
    }

    /// Create an export error
    pub fn export(format: &str, details: &str) -> Self {
        IntegrationError::ExportError(format!("{}: {}", format, details))
    }

    /// Create a conversion error
    pub fn conversion(from: &str, to: &str, details: &str) -> Self {
        IntegrationError::ConversionError(format!("{} -> {}: {}", from, to, details))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_provider_not_found() {
        let err = IntegrationError::ProviderNotFound("test".to_string());
        assert!(err.to_string().contains("test"));
    }

    #[test]
    fn test_parse_error() {
        let err = IntegrationError::parse("CIF", "invalid header");
        assert!(err.to_string().contains("CIF"));
        assert!(err.to_string().contains("invalid header"));
    }
}
