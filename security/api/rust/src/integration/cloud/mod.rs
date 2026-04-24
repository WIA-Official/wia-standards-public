//! Cloud Security Integrations
//!
//! Integrate with cloud security platforms for centralized security management.

pub mod aws;
pub mod azure;
pub mod gcp;

pub use aws::*;
pub use azure::*;
pub use gcp::*;

use thiserror::Error;

/// Cloud integration error types
#[derive(Debug, Error)]
pub enum CloudError {
    #[error("Authentication error: {0}")]
    AuthError(String),

    #[error("API error: {0}")]
    ApiError(String),

    #[error("Rate limit exceeded")]
    RateLimitError,

    #[error("Resource not found: {0}")]
    NotFoundError(String),

    #[error("Invalid configuration: {0}")]
    ConfigError(String),

    #[error("Network error: {0}")]
    NetworkError(String),

    #[error("Serialization error: {0}")]
    SerializationError(String),
}

/// Cloud integration result type
pub type CloudResult<T> = Result<T, CloudError>;

/// Common severity mapping for cloud platforms
pub fn map_severity_to_label(severity: &str) -> &'static str {
    match severity.to_lowercase().as_str() {
        "critical" => "CRITICAL",
        "high" => "HIGH",
        "medium" => "MEDIUM",
        "low" => "LOW",
        "info" | "informational" => "INFORMATIONAL",
        _ => "UNKNOWN",
    }
}

/// Convert CVSS score to normalized severity
pub fn cvss_to_severity(score: f64) -> &'static str {
    if score >= 9.0 {
        "critical"
    } else if score >= 7.0 {
        "high"
    } else if score >= 4.0 {
        "medium"
    } else if score > 0.0 {
        "low"
    } else {
        "info"
    }
}
