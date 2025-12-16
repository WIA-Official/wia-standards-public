//! External Gaming Platform Integrations
//!
//! Provides integration with Xbox, PlayStation, Steam, and cloud gaming platforms.

pub mod playstation;
pub mod steam;
pub mod xbox;

pub use playstation::*;
pub use steam::*;
pub use xbox::*;

use serde::{Deserialize, Serialize};

/// Cloud gaming settings for latency compensation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CloudGamingSettings {
    /// Enable latency compensation
    pub latency_compensation: LatencyCompensation,
    /// Input buffering settings
    pub input_buffering: InputBufferSettings,
    /// Enable predictive input
    pub predictive_input: bool,
    /// Quality vs latency preference
    pub quality_preference: QualityPreference,
}

/// Latency compensation settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LatencyCompensation {
    pub enabled: bool,
    /// Boost aim assist to compensate for latency
    pub aim_assist_boost: f32,
    /// Predict input ahead by this many ms
    pub input_prediction_ms: u32,
    /// Reduce dead zones for better responsiveness
    pub dead_zone_reduction: f32,
}

/// Input buffering settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InputBufferSettings {
    pub enabled: bool,
    /// Buffer size in frames
    pub buffer_frames: u8,
    /// Buffer timeout in ms
    pub timeout_ms: u32,
}

/// Quality vs latency preference
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum QualityPreference {
    Quality,
    Balanced,
    Performance,
}

impl Default for CloudGamingSettings {
    fn default() -> Self {
        Self {
            latency_compensation: LatencyCompensation {
                enabled: true,
                aim_assist_boost: 0.2,
                input_prediction_ms: 50,
                dead_zone_reduction: 0.1,
            },
            input_buffering: InputBufferSettings {
                enabled: true,
                buffer_frames: 3,
                timeout_ms: 100,
            },
            predictive_input: true,
            quality_preference: QualityPreference::Balanced,
        }
    }
}
