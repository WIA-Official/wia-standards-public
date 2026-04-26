//! Renderer module
//!
//! Renderers convert WIA PubScript IR to various output formats.
//!
//! **Philosophy: ALL output formats are EQUAL**
//! - Braille is NOT an "additional feature"
//! - SSML/TTS is NOT an "additional feature"
//! - HTML is NOT an "additional feature"
//! - They are all first-class citizens

pub mod braille;
pub mod html;
pub mod ssml;

use crate::ir::PubScriptDocument;

/// Error type for rendering operations
#[derive(Debug)]
pub enum RenderError {
    /// Render error with message
    RenderError(String),

    /// IO error
    IoError(std::io::Error),
}

impl std::fmt::Display for RenderError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RenderError::RenderError(msg) => write!(f, "Render error: {}", msg),
            RenderError::IoError(e) => write!(f, "IO error: {}", e),
        }
    }
}

impl std::error::Error for RenderError {}

impl From<std::io::Error> for RenderError {
    fn from(e: std::io::Error) -> Self {
        RenderError::IoError(e)
    }
}

/// Renderer trait for converting IR to output format
pub trait Renderer {
    /// Render a PubScriptDocument to output string
    fn render(&self, doc: &PubScriptDocument) -> Result<String, RenderError>;
}
