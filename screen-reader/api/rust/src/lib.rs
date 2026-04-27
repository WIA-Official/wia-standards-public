//! WIA Screen Reader Standard - Rust Engine
//!
//! High-performance screen reader accessibility engine supporting 211 languages.
//!
//! # Features
//! - WIHP (WIA International Hangul Pronunciation) conversion
//! - Universal Braille conversion (Grade 1, Grade 2, WIA format)
//! - Fast processing with zero-copy where possible
//!
//! # Example
//! ```rust
//! use wia_screen_reader::WIAScreenReader;
//!
//! let reader = WIAScreenReader::new();
//! let result = reader.process("Hello World");
//! println!("WIHP: {}", result.pronunciation.wihp);  // "헬로우 월드"
//! println!("Braille: {}", result.braille.grade1);   // "⠓⠑⠇⠇⠕ ⠺⠕⠗⠇⠙"
//! ```
//!
//! 弘益人間 (홍익인간) - Benefit All Humanity

pub mod braille;
pub mod error;
pub mod types;
pub mod wihp;

use std::time::Instant;

pub use braille::BrailleEngine;
pub use error::WIAError;
pub use types::*;
pub use wihp::WIHPEngine;

/// WIA Screen Reader SDK
///
/// Main entry point for screen reader functionality.
pub struct WIAScreenReader {
    wihp_engine: WIHPEngine,
    braille_engine: BrailleEngine,
    config: Config,
}

/// SDK Configuration
#[derive(Debug, Clone)]
pub struct Config {
    pub default_language: String,
    pub default_braille_grade: BrailleGrade,
    pub auto_detect_language: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            default_language: "en".to_string(),
            default_braille_grade: BrailleGrade::Grade1,
            auto_detect_language: true,
        }
    }
}

impl WIAScreenReader {
    /// Create a new WIA Screen Reader instance with default configuration
    pub fn new() -> Self {
        Self::with_config(Config::default())
    }

    /// Create a new WIA Screen Reader instance with custom configuration
    pub fn with_config(config: Config) -> Self {
        Self {
            wihp_engine: WIHPEngine::new(&config.default_language),
            braille_engine: BrailleEngine::new(config.default_braille_grade),
            config,
        }
    }

    /// Process text and return screen reader data
    pub fn process(&self, text: &str) -> ScreenReaderResult {
        self.process_with_language(text, &self.config.default_language)
    }

    /// Process text with specific language
    pub fn process_with_language(&self, text: &str, language: &str) -> ScreenReaderResult {
        let start = Instant::now();

        let wihp = self.wihp_engine.convert(text, language);
        let ipa = self.wihp_engine.get_ipa(text);
        let braille = self.braille_engine.convert(text);

        let processing_time_ms = start.elapsed().as_secs_f64() * 1000.0;

        ScreenReaderResult {
            text: text.to_string(),
            language: language.to_string(),
            pronunciation: Pronunciation {
                ipa,
                wihp,
                romanized: Some(text.to_lowercase()),
                syllables: None,
            },
            braille,
            tts: None,
            context: None,
            metadata: Metadata {
                processing_time_ms,
                engine_version: env!("CARGO_PKG_VERSION").to_string(),
                confidence: Some(0.95),
            },
        }
    }

    /// Get WIHP pronunciation only
    pub fn get_wihp(&self, text: &str) -> String {
        self.wihp_engine.convert(text, &self.config.default_language)
    }

    /// Get braille output only
    pub fn get_braille(&self, text: &str) -> BrailleOutput {
        self.braille_engine.convert(text)
    }

    /// Set default language
    pub fn set_language(&mut self, language: &str) {
        self.config.default_language = language.to_string();
        self.wihp_engine = WIHPEngine::new(language);
    }

    /// Set default braille grade
    pub fn set_braille_grade(&mut self, grade: BrailleGrade) {
        self.config.default_braille_grade = grade;
        self.braille_engine = BrailleEngine::new(grade);
    }

    /// Convert result to JSON string
    pub fn to_json(&self, result: &ScreenReaderResult) -> Result<String, WIAError> {
        serde_json::to_string_pretty(&WIAScreenReaderData {
            wia_screen_reader: result.clone(),
        })
        .map_err(|e| WIAError::Serialization(e.to_string()))
    }
}

impl Default for WIAScreenReader {
    fn default() -> Self {
        Self::new()
    }
}

/// Wrapper for JSON serialization
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct WIAScreenReaderData {
    pub wia_screen_reader: ScreenReaderResult,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_processing() {
        let reader = WIAScreenReader::new();
        let result = reader.process("Hello World");

        assert_eq!(result.text, "Hello World");
        assert!(!result.pronunciation.wihp.is_empty());
        assert!(!result.braille.grade1.is_empty());
    }

    #[test]
    fn test_wihp_conversion() {
        let reader = WIAScreenReader::new();
        let wihp = reader.get_wihp("hello");

        assert_eq!(wihp, "헬로우");
    }

    #[test]
    fn test_braille_conversion() {
        let reader = WIAScreenReader::new();
        let braille = reader.get_braille("abc");

        assert_eq!(braille.grade1, "⠁⠃⠉");
    }
}
