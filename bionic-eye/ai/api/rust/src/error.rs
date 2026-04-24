//! WIA AI Error Types
//!
//! This module defines error types for the WIA AI SDK.

use thiserror::Error;

/// WIA AI SDK Error
#[derive(Error, Debug)]
pub enum WiaAiError {
    /// Serialization/deserialization error
    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),

    /// Validation error
    #[error("Validation error: {message}")]
    ValidationError {
        /// Error message
        message: String,
        /// Field that failed validation
        field: Option<String>,
    },

    /// Model not found
    #[error("Model not found: {0}")]
    ModelNotFound(String),

    /// Agent not found
    #[error("Agent not found: {0}")]
    AgentNotFound(String),

    /// Tool not found
    #[error("Tool not found: {0}")]
    ToolNotFound(String),

    /// Tool execution error
    #[error("Tool execution error: {0}")]
    ToolExecutionError(String),

    /// API error
    #[error("API error: {message} (status: {status_code})")]
    ApiError {
        /// HTTP status code
        status_code: u16,
        /// Error message
        message: String,
    },

    /// Rate limit exceeded
    #[error("Rate limit exceeded: {0}")]
    RateLimitExceeded(String),

    /// Timeout error
    #[error("Operation timed out: {0}")]
    Timeout(String),

    /// Configuration error
    #[error("Configuration error: {0}")]
    ConfigurationError(String),

    /// IO error
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    /// Network error
    #[error("Network error: {0}")]
    NetworkError(String),

    /// Authentication error
    #[error("Authentication error: {0}")]
    AuthenticationError(String),

    /// Permission denied
    #[error("Permission denied: {0}")]
    PermissionDenied(String),

    /// Resource exhausted
    #[error("Resource exhausted: {0}")]
    ResourceExhausted(String),

    /// Invalid argument
    #[error("Invalid argument: {0}")]
    InvalidArgument(String),

    /// Internal error
    #[error("Internal error: {0}")]
    InternalError(String),

    /// Agent error
    #[error("Agent error: {message}")]
    AgentError {
        /// Agent ID
        agent_id: String,
        /// Error message
        message: String,
    },

    /// Swarm error
    #[error("Swarm error: {message}")]
    SwarmError {
        /// Swarm ID
        swarm_id: String,
        /// Error message
        message: String,
    },

    /// Safety violation
    #[error("Safety violation: {0}")]
    SafetyViolation(String),

    /// Experiment error
    #[error("Experiment error: {0}")]
    ExperimentError(String),

    /// Evaluation error
    #[error("Evaluation error: {0}")]
    EvaluationError(String),

    /// Not found error
    #[error("Not found: {0}")]
    NotFound(String),

    /// Not supported error
    #[error("Not supported: {0}")]
    NotSupported(String),

    /// Connection error
    #[error("Connection error: {0}")]
    ConnectionError(String),
}

/// Result type alias for WIA AI operations
pub type Result<T> = std::result::Result<T, WiaAiError>;

impl WiaAiError {
    /// Create a validation error
    pub fn validation(message: impl Into<String>) -> Self {
        WiaAiError::ValidationError {
            message: message.into(),
            field: None,
        }
    }

    /// Create a validation error with field name
    pub fn validation_field(message: impl Into<String>, field: impl Into<String>) -> Self {
        WiaAiError::ValidationError {
            message: message.into(),
            field: Some(field.into()),
        }
    }

    /// Create an API error
    pub fn api(status_code: u16, message: impl Into<String>) -> Self {
        WiaAiError::ApiError {
            status_code,
            message: message.into(),
        }
    }

    /// Create an agent error
    pub fn agent(agent_id: impl Into<String>, message: impl Into<String>) -> Self {
        WiaAiError::AgentError {
            agent_id: agent_id.into(),
            message: message.into(),
        }
    }

    /// Create a swarm error
    pub fn swarm(swarm_id: impl Into<String>, message: impl Into<String>) -> Self {
        WiaAiError::SwarmError {
            swarm_id: swarm_id.into(),
            message: message.into(),
        }
    }

    /// Check if error is retryable
    pub fn is_retryable(&self) -> bool {
        match self {
            WiaAiError::NetworkError(_)
            | WiaAiError::Timeout(_)
            | WiaAiError::RateLimitExceeded(_) => true,
            WiaAiError::ApiError { status_code, .. } => *status_code >= 500,
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validation_error() {
        let err = WiaAiError::validation("Invalid model ID");
        assert!(err.to_string().contains("Invalid model ID"));
    }

    #[test]
    fn test_validation_field_error() {
        let err = WiaAiError::validation_field("Must be positive", "temperature");
        if let WiaAiError::ValidationError { field, .. } = err {
            assert_eq!(field, Some("temperature".to_string()));
        } else {
            panic!("Expected ValidationError");
        }
    }

    #[test]
    fn test_is_retryable() {
        assert!(WiaAiError::NetworkError("connection failed".to_string()).is_retryable());
        assert!(WiaAiError::Timeout("request timed out".to_string()).is_retryable());
        assert!(WiaAiError::api(503, "Service unavailable").is_retryable());
        assert!(!WiaAiError::api(400, "Bad request").is_retryable());
        assert!(!WiaAiError::ValidationError {
            message: "test".to_string(),
            field: None
        }
        .is_retryable());
    }
}
