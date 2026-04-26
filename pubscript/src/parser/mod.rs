//! Parser module
//!
//! Parsers convert various input formats to WIA PubScript IR.

pub mod markdown;

use crate::ir::PubScriptDocument;

/// Error type for parsing operations
#[derive(Debug)]
pub enum ParserError {
    /// Parse error with message
    ParseError(String),

    /// IO error
    IoError(std::io::Error),
}

impl std::fmt::Display for ParserError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ParserError::ParseError(msg) => write!(f, "Parse error: {}", msg),
            ParserError::IoError(e) => write!(f, "IO error: {}", e),
        }
    }
}

impl std::error::Error for ParserError {}

impl From<std::io::Error> for ParserError {
    fn from(e: std::io::Error) -> Self {
        ParserError::IoError(e)
    }
}

/// Parser trait for converting input to IR
pub trait Parser {
    /// Parse input string into a PubScriptDocument
    fn parse(&self, input: &str) -> Result<PubScriptDocument, ParserError>;
}
