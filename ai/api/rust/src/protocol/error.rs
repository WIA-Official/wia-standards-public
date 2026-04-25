//! Protocol Error Types
//!
//! This module defines error types for the WIA AI Communication Protocol.

use std::fmt;
use thiserror::Error;

use super::message::ErrorCode;

/// Protocol error type
#[derive(Debug, Error)]
pub struct ProtocolError {
    /// Error code
    pub code: ErrorCode,

    /// Error message
    pub message: String,

    /// Additional details
    pub details: Option<String>,

    /// Whether this error is retryable
    pub retryable: bool,

    /// Suggested retry delay in milliseconds
    pub retry_after_ms: Option<u64>,
}

impl fmt::Display for ProtocolError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[{}] {}", self.code.name(), self.message)?;
        if let Some(ref details) = self.details {
            write!(f, ": {}", details)?;
        }
        Ok(())
    }
}

impl ProtocolError {
    /// Create a new protocol error
    pub fn new(code: ErrorCode, message: impl Into<String>) -> Self {
        Self {
            code,
            message: message.into(),
            details: None,
            retryable: code.is_retryable(),
            retry_after_ms: None,
        }
    }

    /// Create a connection error
    pub fn connection_error(code: ErrorCode, message: impl Into<String>) -> Self {
        Self::new(code, message)
    }

    /// Create a connection lost error
    pub fn connection_lost(message: impl Into<String>) -> Self {
        Self::new(ErrorCode::ConnectionLost, message)
    }

    /// Create an agent not found error
    pub fn agent_not_found(agent_id: impl Into<String>) -> Self {
        Self::new(
            ErrorCode::AgentNotFound,
            format!("Agent not found: {}", agent_id.into()),
        )
    }

    /// Create an agent busy error
    pub fn agent_busy(agent_id: impl Into<String>) -> Self {
        let mut error = Self::new(
            ErrorCode::AgentBusy,
            format!("Agent is busy: {}", agent_id.into()),
        );
        error.retry_after_ms = Some(5000);
        error
    }

    /// Create a tool not found error
    pub fn tool_not_found(tool_name: impl Into<String>) -> Self {
        Self::new(
            ErrorCode::ToolNotFound,
            format!("Tool not found: {}", tool_name.into()),
        )
    }

    /// Create a tool execution error
    pub fn tool_execution_failed(message: impl Into<String>) -> Self {
        Self::new(ErrorCode::ToolExecutionFailed, message)
    }

    /// Create a tool timeout error
    pub fn tool_timeout(tool_name: impl Into<String>) -> Self {
        Self::new(
            ErrorCode::ToolTimeout,
            format!("Tool execution timed out: {}", tool_name.into()),
        )
    }

    /// Create a rate limited error
    pub fn rate_limited(retry_after_ms: u64) -> Self {
        let mut error = Self::new(ErrorCode::RateLimited, "Rate limit exceeded");
        error.retry_after_ms = Some(retry_after_ms);
        error
    }

    /// Create an authentication failed error
    pub fn auth_failed(message: impl Into<String>) -> Self {
        Self::new(ErrorCode::AuthFailed, message)
    }

    /// Create a permission denied error
    pub fn permission_denied(message: impl Into<String>) -> Self {
        Self::new(ErrorCode::PermissionDenied, message)
    }

    /// Create a validation error
    pub fn validation_error(message: impl Into<String>) -> Self {
        Self::new(ErrorCode::InvalidMessage, message)
    }

    /// Create a serialization error
    pub fn serialization_error(message: impl Into<String>) -> Self {
        Self::new(ErrorCode::ProtocolError, message)
    }

    /// Add details to the error
    pub fn with_details(mut self, details: impl Into<String>) -> Self {
        self.details = Some(details.into());
        self
    }

    /// Set retry after
    pub fn with_retry_after(mut self, ms: u64) -> Self {
        self.retry_after_ms = Some(ms);
        self.retryable = true;
        self
    }

    /// Check if the error is retryable
    pub fn is_retryable(&self) -> bool {
        self.retryable
    }
}

/// Result type for protocol operations
pub type ProtocolResult<T> = Result<T, ProtocolError>;

/// Retry policy configuration
#[derive(Debug, Clone)]
pub struct RetryPolicy {
    /// Maximum number of retries
    pub max_retries: u32,

    /// Backoff strategy
    pub backoff_strategy: BackoffStrategy,

    /// Initial delay in milliseconds
    pub initial_delay_ms: u64,

    /// Maximum delay in milliseconds
    pub max_delay_ms: u64,

    /// Whether to add jitter
    pub jitter: bool,
}

impl Default for RetryPolicy {
    fn default() -> Self {
        Self {
            max_retries: 3,
            backoff_strategy: BackoffStrategy::Exponential,
            initial_delay_ms: 1000,
            max_delay_ms: 30000,
            jitter: true,
        }
    }
}

impl RetryPolicy {
    /// Create a new retry policy
    pub fn new(max_retries: u32) -> Self {
        Self {
            max_retries,
            ..Default::default()
        }
    }

    /// Set the backoff strategy
    pub fn with_backoff(mut self, strategy: BackoffStrategy) -> Self {
        self.backoff_strategy = strategy;
        self
    }

    /// Set the initial delay
    pub fn with_initial_delay(mut self, ms: u64) -> Self {
        self.initial_delay_ms = ms;
        self
    }

    /// Set the max delay
    pub fn with_max_delay(mut self, ms: u64) -> Self {
        self.max_delay_ms = ms;
        self
    }

    /// Enable or disable jitter
    pub fn with_jitter(mut self, jitter: bool) -> Self {
        self.jitter = jitter;
        self
    }

    /// Check if retry should be attempted
    pub fn should_retry(&self, attempt: u32) -> bool {
        attempt < self.max_retries
    }

    /// Calculate delay for a given attempt
    pub fn calculate_delay(&self, attempt: u32) -> u64 {
        let base_delay = match self.backoff_strategy {
            BackoffStrategy::None => 0,
            BackoffStrategy::Linear => self.initial_delay_ms * (attempt as u64 + 1),
            BackoffStrategy::Exponential => self.initial_delay_ms * 2u64.pow(attempt),
        };

        let delay = base_delay.min(self.max_delay_ms);

        if self.jitter {
            // Add up to 10% jitter
            let jitter_range = delay / 10;
            let jitter = rand_jitter(jitter_range);
            delay.saturating_add(jitter)
        } else {
            delay
        }
    }
}

/// Backoff strategy
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BackoffStrategy {
    /// No backoff
    None,
    /// Linear backoff (delay * attempt)
    Linear,
    /// Exponential backoff (delay * 2^attempt)
    Exponential,
}

/// Simple pseudo-random jitter (not cryptographically secure)
fn rand_jitter(range: u64) -> u64 {
    if range == 0 {
        return 0;
    }

    use std::time::{SystemTime, UNIX_EPOCH};
    let seed = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64;

    seed % range
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_creation() {
        let error = ProtocolError::agent_not_found("agent-001");
        assert_eq!(error.code, ErrorCode::AgentNotFound);
        assert!(error.message.contains("agent-001"));
        assert!(!error.is_retryable());
    }

    #[test]
    fn test_retryable_error() {
        let error = ProtocolError::rate_limited(5000);
        assert!(error.is_retryable());
        assert_eq!(error.retry_after_ms, Some(5000));
    }

    #[test]
    fn test_retry_policy() {
        let policy = RetryPolicy::new(3)
            .with_backoff(BackoffStrategy::Exponential)
            .with_initial_delay(1000)
            .with_jitter(false);

        assert_eq!(policy.calculate_delay(0), 1000);
        assert_eq!(policy.calculate_delay(1), 2000);
        assert_eq!(policy.calculate_delay(2), 4000);
    }
}
