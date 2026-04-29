//! Emotion detection and analysis

use serde::{Deserialize, Serialize};
use crate::types::{DetectionMethod, Timestamp};

/// Emotion category
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EmotionCategory {
    Happy,
    Sad,
    Angry,
    Fearful,
    Surprised,
    Disgusted,
    Neutral,
    Content,
    Anxious,
    Confused,
    Lonely,
}

/// Emotion state detection result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmotionState {
    /// Timestamp of detection
    pub timestamp: Timestamp,
    /// Recipient ID
    pub recipient_id: String,
    /// Device ID
    pub device_id: Option<String>,
    /// Detection method used
    pub detection_method: DetectionMethod,
    /// Primary detected emotion
    pub primary_emotion: EmotionScore,
    /// Secondary emotions
    pub secondary_emotions: Vec<EmotionScore>,
    /// Dimensional emotion values
    pub dimensional: DimensionalEmotion,
    /// Facial analysis results
    pub facial_analysis: Option<FacialAnalysis>,
    /// Voice analysis results
    pub voice_analysis: Option<VoiceAnalysis>,
    /// Behavioral context
    pub behavioral_context: Option<BehavioralContext>,
    /// Emotion trend
    pub trend: Option<EmotionTrend>,
}

impl EmotionState {
    /// Create a new emotion state for a recipient
    pub fn new(recipient_id: &str) -> Self {
        Self {
            timestamp: Timestamp::now(),
            recipient_id: recipient_id.to_string(),
            device_id: None,
            detection_method: DetectionMethod::Multimodal,
            primary_emotion: EmotionScore {
                category: EmotionCategory::Neutral,
                confidence: 0.5,
                intensity: None,
            },
            secondary_emotions: Vec::new(),
            dimensional: DimensionalEmotion::default(),
            facial_analysis: None,
            voice_analysis: None,
            behavioral_context: None,
            trend: None,
        }
    }

    /// Set primary emotion
    pub fn with_primary(mut self, category: EmotionCategory, confidence: f64) -> Self {
        self.primary_emotion = EmotionScore {
            category,
            confidence,
            intensity: None,
        };
        self
    }

    /// Set valence (positive/negative dimension)
    pub fn with_valence(mut self, valence: f64) -> Self {
        self.dimensional.valence = Some(valence);
        self
    }

    /// Set arousal (calm/excited dimension)
    pub fn with_arousal(mut self, arousal: f64) -> Self {
        self.dimensional.arousal = Some(arousal);
        self
    }

    /// Set detection method
    pub fn with_detection_method(mut self, method: DetectionMethod) -> Self {
        self.detection_method = method;
        self
    }

    /// Add secondary emotion
    pub fn add_secondary(&mut self, category: EmotionCategory, confidence: f64) {
        self.secondary_emotions.push(EmotionScore {
            category,
            confidence,
            intensity: None,
        });
    }

    /// Check if emotion is positive
    pub fn is_positive(&self) -> bool {
        matches!(
            self.primary_emotion.category,
            EmotionCategory::Happy | EmotionCategory::Content | EmotionCategory::Surprised
        ) || self.dimensional.valence.map(|v| v > 0.0).unwrap_or(false)
    }

    /// Check if emotion is negative
    pub fn is_negative(&self) -> bool {
        matches!(
            self.primary_emotion.category,
            EmotionCategory::Sad
                | EmotionCategory::Angry
                | EmotionCategory::Fearful
                | EmotionCategory::Anxious
                | EmotionCategory::Lonely
        ) || self.dimensional.valence.map(|v| v < 0.0).unwrap_or(false)
    }

    /// Check if emotion requires attention (negative with high intensity)
    pub fn requires_attention(&self) -> bool {
        self.is_negative()
            && self
                .primary_emotion
                .confidence
                .max(self.primary_emotion.intensity.unwrap_or(0.0))
                > 0.7
    }
}

/// Emotion score with category and confidence
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmotionScore {
    /// Emotion category
    pub category: EmotionCategory,
    /// Confidence (0.0-1.0)
    pub confidence: f64,
    /// Intensity (0.0-1.0)
    pub intensity: Option<f64>,
}

/// Dimensional emotion model (VAD - Valence, Arousal, Dominance)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DimensionalEmotion {
    /// Valence: negative (-1) to positive (+1)
    pub valence: Option<f64>,
    /// Arousal: calm (0) to excited (1)
    pub arousal: Option<f64>,
    /// Dominance: submissive (0) to dominant (1)
    pub dominance: Option<f64>,
}

/// Facial analysis results
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FacialAnalysis {
    /// Face detected
    pub detected: bool,
    /// Landmark detection quality (0.0-1.0)
    pub landmarks_quality: f64,
    /// Facial expressions
    pub expressions: FacialExpressions,
}

/// Facial expression measurements
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct FacialExpressions {
    /// Smile intensity (0.0-1.0)
    pub smile: f64,
    /// Eye openness (0.0-1.0)
    pub eye_openness: f64,
    /// Brow raise (0.0-1.0)
    pub brow_raise: f64,
}

/// Voice analysis results
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceAnalysis {
    /// Voice detected
    pub detected: bool,
    /// Mean pitch in Hz
    pub pitch_mean_hz: f64,
    /// Pitch variation
    pub pitch_variation: f64,
    /// Voice energy level
    pub energy: f64,
    /// Speech rate
    pub speech_rate: SpeechRate,
}

/// Speech rate classification
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpeechRate {
    VerySlow,
    Slow,
    Normal,
    Fast,
    VeryFast,
}

/// Behavioral context
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BehavioralContext {
    /// Current activity
    pub activity: String,
    /// Social context
    pub social_context: String,
    /// Time of day
    pub time_of_day: String,
}

/// Emotion trend over time
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmotionTrend {
    /// Comparison to 1 hour ago
    pub vs_1_hour_ago: String,
    /// Comparison to yesterday
    pub vs_yesterday: String,
    /// Weekly average
    pub weekly_average: String,
}
