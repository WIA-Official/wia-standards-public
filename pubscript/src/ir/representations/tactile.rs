//! Tactile representation (촉각)

use serde::{Deserialize, Serialize};

/// Tactile representation
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TactileRep {
    /// Braille text
    pub braille: Option<String>,

    /// Haptic pattern
    pub haptic_pattern: Option<HapticPattern>,

    /// Texture
    pub texture: Option<Texture>,
}

/// Haptic feedback patterns
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HapticPattern {
    SectionBoundary,
    Heading,
    Link,
    Custom(Vec<HapticEvent>),
}

/// A single haptic event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticEvent {
    pub intensity: f32, // 0.0 ~ 1.0
    pub duration: f64,  // seconds
}

impl HapticEvent {
    pub fn new(intensity: f32, duration: f64) -> Self {
        Self {
            intensity,
            duration,
        }
    }
}

/// Texture type
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Texture {
    pub texture_type: TextureType,
    pub intensity: f32,
}

/// Types of textures
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TextureType {
    Smooth,
    Rough,
    Bumpy,
    Custom(String),
}

impl TactileRep {
    /// Create a simple braille representation
    pub fn braille(text: impl Into<String>) -> Self {
        Self {
            braille: Some(text.into()),
            ..Default::default()
        }
    }

    /// Set braille text
    pub fn with_braille(mut self, text: impl Into<String>) -> Self {
        self.braille = Some(text.into());
        self
    }

    /// Set haptic pattern
    pub fn with_haptic_pattern(mut self, pattern: HapticPattern) -> Self {
        self.haptic_pattern = Some(pattern);
        self
    }

    /// Set texture
    pub fn with_texture(mut self, texture: Texture) -> Self {
        self.texture = Some(texture);
        self
    }
}
