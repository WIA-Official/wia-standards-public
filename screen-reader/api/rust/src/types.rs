//! Type definitions for WIA Screen Reader

use serde::{Deserialize, Serialize};

/// Braille grade types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BrailleGrade {
    #[serde(rename = "1")]
    Grade1,
    #[serde(rename = "2")]
    Grade2,
}

impl Default for BrailleGrade {
    fn default() -> Self {
        Self::Grade1
    }
}

/// Pronunciation output
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pronunciation {
    /// International Phonetic Alphabet
    pub ipa: String,
    /// WIA International Hangul Pronunciation
    pub wihp: String,
    /// Romanized form
    #[serde(skip_serializing_if = "Option::is_none")]
    pub romanized: Option<String>,
    /// Syllable breakdown
    #[serde(skip_serializing_if = "Option::is_none")]
    pub syllables: Option<Vec<String>>,
}

/// Braille cell representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BrailleCell {
    /// Source character
    pub char: char,
    /// Active dots (1-8)
    pub dots: Vec<u8>,
    /// Unicode braille character
    pub unicode: char,
}

/// Braille output
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BrailleOutput {
    /// Grade 1 (uncontracted) braille
    pub grade1: String,
    /// Grade 2 (contracted) braille
    pub grade2: String,
    /// WIA Korean-based braille format
    pub wia: String,
    /// Dot patterns for each cell
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dots: Option<Vec<BrailleCell>>,
    /// Total number of cells
    pub cells: usize,
}

/// TTS configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TTSConfig {
    /// Speech rate (0.1-10, default 1.0)
    #[serde(default = "default_rate")]
    pub rate: f64,
    /// Voice pitch (0.1-2.0, default 1.0)
    #[serde(default = "default_pitch")]
    pub pitch: f64,
    /// Volume (0-1, default 1.0)
    #[serde(default = "default_volume")]
    pub volume: f64,
    /// Voice identifier
    #[serde(skip_serializing_if = "Option::is_none")]
    pub voice: Option<String>,
}

fn default_rate() -> f64 { 1.0 }
fn default_pitch() -> f64 { 1.0 }
fn default_volume() -> f64 { 1.0 }

impl Default for TTSConfig {
    fn default() -> Self {
        Self {
            rate: 1.0,
            pitch: 1.0,
            volume: 1.0,
            voice: None,
        }
    }
}

/// Element type
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ElementType {
    Heading,
    Paragraph,
    List,
    ListItem,
    Link,
    Button,
    TextBox,
    CheckBox,
    Radio,
    Slider,
    Table,
    Row,
    Cell,
    Image,
    Figure,
    Form,
    Navigation,
    Main,
    Article,
    Section,
}

/// Landmark role
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum LandmarkRole {
    Banner,
    Main,
    Navigation,
    ContentInfo,
    Search,
    Form,
    Complementary,
    Region,
}

/// Element state
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ElementState {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub expanded: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub selected: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub checked: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub disabled: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub readonly: Option<bool>,
}

/// Semantic context
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Context {
    /// Element type
    #[serde(skip_serializing_if = "Option::is_none")]
    pub element_type: Option<ElementType>,
    /// Heading level (1-6)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub level: Option<u8>,
    /// ARIA landmark
    #[serde(skip_serializing_if = "Option::is_none")]
    pub landmark: Option<LandmarkRole>,
    /// Element state
    #[serde(skip_serializing_if = "Option::is_none")]
    pub state: Option<ElementState>,
}

/// Processing metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Metadata {
    /// Processing time in milliseconds
    pub processing_time_ms: f64,
    /// Engine version
    pub engine_version: String,
    /// Confidence score (0-1)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidence: Option<f64>,
}

/// Screen reader result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScreenReaderResult {
    /// Original text
    pub text: String,
    /// Source language
    pub language: String,
    /// Pronunciation data
    pub pronunciation: Pronunciation,
    /// Braille output
    pub braille: BrailleOutput,
    /// TTS configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tts: Option<TTSConfig>,
    /// Semantic context
    #[serde(skip_serializing_if = "Option::is_none")]
    pub context: Option<Context>,
    /// Processing metadata
    pub metadata: Metadata,
}
