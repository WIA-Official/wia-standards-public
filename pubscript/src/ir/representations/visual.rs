//! Visual representation (시각)

use serde::{Deserialize, Serialize};

/// Visual representation
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VisualRep {
    /// Text content
    pub text: Option<String>,

    /// Image data
    pub image: Option<ImageData>,

    /// Layout mode
    pub layout: Layout,

    /// Visual style
    pub style: VisualStyle,
}

/// Image data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImageData {
    pub url: String,
    pub alt_text: Option<String>,
    pub width: Option<u32>,
    pub height: Option<u32>,
}

/// Layout modes
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Layout {
    Block,
    Inline,
    Flex,
    Grid,
}

impl Default for Layout {
    fn default() -> Self {
        Layout::Block
    }
}

/// Visual styling
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VisualStyle {
    pub font_family: Option<String>,
    pub font_size: Option<f32>,
    pub font_weight: Option<FontWeight>,
    pub color: Option<Color>,
    pub background: Option<Color>,
}

/// Font weight
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum FontWeight {
    Normal,
    Bold,
    Light,
    Custom(u16),
}

/// RGBA color
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8,
}

impl Color {
    /// Create a new color
    pub fn rgba(r: u8, g: u8, b: u8, a: u8) -> Self {
        Self { r, g, b, a }
    }

    /// Create an opaque color
    pub fn rgb(r: u8, g: u8, b: u8) -> Self {
        Self::rgba(r, g, b, 255)
    }

    /// Black color
    pub fn black() -> Self {
        Self::rgb(0, 0, 0)
    }

    /// White color
    pub fn white() -> Self {
        Self::rgb(255, 255, 255)
    }
}

impl VisualRep {
    /// Create a simple text visual representation
    pub fn text(text: impl Into<String>) -> Self {
        Self {
            text: Some(text.into()),
            ..Default::default()
        }
    }

    /// Set text
    pub fn with_text(mut self, text: impl Into<String>) -> Self {
        self.text = Some(text.into());
        self
    }

    /// Set style
    pub fn with_style(mut self, style: VisualStyle) -> Self {
        self.style = style;
        self
    }
}
