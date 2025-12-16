//! Common types for the WIA CareBot SDK

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};

/// Cognitive level assessment
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CognitiveLevel {
    /// Normal cognitive function
    Normal,
    /// Mild cognitive impairment
    MildImpairment,
    /// Moderate cognitive impairment
    ModerateImpairment,
    /// Severe cognitive impairment
    SevereImpairment,
}

/// Mobility level assessment
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MobilityLevel {
    /// Fully independent movement
    Independent,
    /// Requires assistance for walking
    AssistedWalking,
    /// Uses wheelchair
    Wheelchair,
    /// Confined to bed
    Bedridden,
}

/// Hearing level assessment
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HearingLevel {
    Normal,
    MildLoss,
    ModerateLoss,
    SevereLoss,
}

/// Vision level assessment
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VisionLevel {
    Normal,
    CorrectedNormal,
    LowVision,
    Blind,
}

/// Voice speed preference
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VoiceSpeed {
    Slow,
    Normal,
    Fast,
}

/// Voice volume preference
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VoiceVolume {
    Quiet,
    Normal,
    Loud,
}

/// Gender
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Gender {
    Male,
    Female,
    Other,
}

/// Severity level for events
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Severity {
    Info,
    Warning,
    Urgent,
    Critical,
    Emergency,
}

/// Activity type for daily routines
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ActivityType {
    /// Regular routine activity
    Routine,
    /// Medication-related activity
    Medication,
    /// Physical exercise
    Exercise,
    /// Meal time
    Meal,
    /// Rest or nap
    Rest,
    /// Social activity
    Social,
    /// Cognitive exercise
    Cognitive,
    /// Personal care
    PersonalCare,
    /// Medical appointment
    Medical,
}

/// Emotion detection method
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DetectionMethod {
    /// Facial expression analysis
    Facial,
    /// Voice analysis
    Voice,
    /// Combined multimodal analysis
    Multimodal,
    /// Behavioral pattern analysis
    Behavioral,
}

/// Location within the care environment
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Location {
    /// Room name
    pub room: String,
    /// X coordinate (optional)
    pub x: Option<f64>,
    /// Y coordinate (optional)
    pub y: Option<f64>,
    /// Location confidence (0.0-1.0)
    pub confidence: Option<f64>,
}

impl Location {
    /// Create a new location with just the room name
    pub fn new(room: &str) -> Self {
        Self {
            room: room.to_string(),
            x: None,
            y: None,
            confidence: None,
        }
    }

    /// Create a location with coordinates
    pub fn with_coordinates(room: &str, x: f64, y: f64) -> Self {
        Self {
            room: room.to_string(),
            x: Some(x),
            y: Some(y),
            confidence: None,
        }
    }
}

/// Timestamp wrapper for consistent datetime handling
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Timestamp(pub DateTime<Utc>);

impl Timestamp {
    /// Get current timestamp
    pub fn now() -> Self {
        Self(Utc::now())
    }

    /// Create from DateTime
    pub fn from_datetime(dt: DateTime<Utc>) -> Self {
        Self(dt)
    }
}

impl Default for Timestamp {
    fn default() -> Self {
        Self::now()
    }
}
