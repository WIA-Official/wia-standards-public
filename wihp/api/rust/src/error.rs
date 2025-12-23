//! WIHP error types

#[cfg(not(feature = "std"))]
use alloc::string::String;

use thiserror::Error;

/// WIHP conversion error
#[derive(Debug, Error)]
pub enum WihpError {
    /// Unknown IPA symbol encountered
    #[error("Unknown IPA symbol: '{0}'")]
    UnknownSymbol(char),

    /// Invalid IPA syntax
    #[error("Invalid IPA syntax: {0}")]
    InvalidSyntax(String),

    /// Invalid Hangul input for reverse conversion
    #[error("Invalid Hangul input: {0}")]
    InvalidHangul(String),

    /// Encoding error
    #[error("Encoding error: {0}")]
    EncodingError(String),

    /// Empty input
    #[error("Empty input")]
    EmptyInput,
}

impl WihpError {
    /// Check if the error is recoverable
    pub fn is_recoverable(&self) -> bool {
        matches!(self, WihpError::UnknownSymbol(_))
    }

    /// Get the problematic character if applicable
    pub fn problematic_char(&self) -> Option<char> {
        match self {
            WihpError::UnknownSymbol(c) => Some(*c),
            _ => None,
        }
    }
}

/// Result type for WIHP operations
pub type WihpResult<T> = Result<T, WihpError>;

/// Warning during conversion (non-fatal)
#[derive(Debug, Clone)]
pub struct WihpWarning {
    /// Warning message
    pub message: String,
    /// Position in input (if applicable)
    pub position: Option<usize>,
    /// Suggested fix
    pub suggestion: Option<String>,
}

impl WihpWarning {
    /// Create a new warning
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
            position: None,
            suggestion: None,
        }
    }

    /// Set position
    pub fn at_position(mut self, pos: usize) -> Self {
        self.position = Some(pos);
        self
    }

    /// Set suggestion
    pub fn with_suggestion(mut self, suggestion: impl Into<String>) -> Self {
        self.suggestion = Some(suggestion.into());
        self
    }
}
