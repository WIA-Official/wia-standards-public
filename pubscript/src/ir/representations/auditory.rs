//! Auditory representation (청각)

use serde::{Deserialize, Serialize};

/// Auditory representation
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AuditoryRep {
    /// Speech synthesis text
    pub speech_text: Option<String>,

    /// SSML (Speech Synthesis Markup Language)
    pub ssml: Option<String>,

    /// Audio file URL
    pub audio_file: Option<String>,

    /// Sound effects
    pub sound_effects: Vec<SoundEffect>,

    /// Spatial audio
    pub spatial_audio: Option<SpatialAudio>,
}

/// Sound effect
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SoundEffect {
    pub effect_type: SoundEffectType,
    pub volume: f32, // 0.0 ~ 1.0
    pub duration: Option<f64>, // seconds
}

/// Sound effect types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SoundEffectType {
    Beep,
    Click,
    Whoosh,
    Custom(String),
}

/// Spatial audio positioning
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpatialAudio {
    pub position: Vec3,
    pub direction: Vec3,
}

/// 3D vector
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }
}

impl AuditoryRep {
    /// Create a simple speech representation
    pub fn speech(text: impl Into<String>) -> Self {
        Self {
            speech_text: Some(text.into()),
            ..Default::default()
        }
    }

    /// Set speech text
    pub fn with_speech(mut self, text: impl Into<String>) -> Self {
        self.speech_text = Some(text.into());
        self
    }

    /// Set SSML
    pub fn with_ssml(mut self, ssml: impl Into<String>) -> Self {
        self.ssml = Some(ssml.into());
        self
    }

    /// Add sound effect
    pub fn add_sound_effect(mut self, effect: SoundEffect) -> Self {
        self.sound_effects.push(effect);
        self
    }
}
