//! Type definitions for 3D Touch Standard
//!
//! 弘益人間 - Touch that connects all humanity

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::{DateTime, Utc};

/// 3D Touch event data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TouchEvent {
    pub id: Uuid,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub pressure: f64,
    pub timestamp: DateTime<Utc>,
    pub touch_type: TouchType,
    pub gesture: Option<GestureType>,
}

impl TouchEvent {
    pub fn new(x: f64, y: f64, pressure: f64) -> Self {
        Self {
            id: Uuid::new_v4(),
            x,
            y,
            z: 0.0,
            pressure,
            timestamp: Utc::now(),
            touch_type: TouchType::Direct,
            gesture: None,
        }
    }
}

/// Touch interaction types
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum TouchType {
    #[serde(rename = "direct")]
    Direct,
    #[serde(rename = "hover")]
    Hover,
    #[serde(rename = "force")]
    Force,
    #[serde(rename = "multi")]
    Multi,
}

/// Gesture types
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum GestureType {
    #[serde(rename = "tap")]
    Tap,
    #[serde(rename = "double_tap")]
    DoubleTap,
    #[serde(rename = "long_press")]
    LongPress,
    #[serde(rename = "swipe")]
    Swipe,
    #[serde(rename = "pinch")]
    Pinch,
    #[serde(rename = "rotate")]
    Rotate,
    #[serde(rename = "pan")]
    Pan,
}

/// Haptic feedback pattern
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticPattern {
    pub id: Uuid,
    pub name: String,
    pub intensity: f64,
    pub duration_ms: u64,
    pub frequency_hz: f64,
    pub waveform: WaveformType,
    pub repetitions: u32,
}

impl HapticPattern {
    pub fn click() -> Self {
        Self {
            id: Uuid::new_v4(),
            name: "click".to_string(),
            intensity: 0.5,
            duration_ms: 10,
            frequency_hz: 250.0,
            waveform: WaveformType::Sharp,
            repetitions: 1,
        }
    }

    pub fn notification() -> Self {
        Self {
            id: Uuid::new_v4(),
            name: "notification".to_string(),
            intensity: 0.7,
            duration_ms: 100,
            frequency_hz: 200.0,
            waveform: WaveformType::Smooth,
            repetitions: 2,
        }
    }
}

/// Haptic waveform types
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum WaveformType {
    #[serde(rename = "sharp")]
    Sharp,
    #[serde(rename = "smooth")]
    Smooth,
    #[serde(rename = "pulse")]
    Pulse,
    #[serde(rename = "wave")]
    Wave,
}

/// Touch surface configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TouchSurface {
    pub id: Uuid,
    pub name: String,
    pub width_mm: f64,
    pub height_mm: f64,
    pub resolution_dpi: u32,
    pub max_pressure: f64,
    pub multi_touch_points: u32,
    pub haptic_enabled: bool,
    pub force_sensing: bool,
}

/// Touch calibration data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationData {
    pub surface_id: Uuid,
    pub calibrated_at: DateTime<Utc>,
    pub pressure_min: f64,
    pub pressure_max: f64,
    pub position_accuracy_mm: f64,
    pub force_accuracy_n: f64,
    pub touch_points: Vec<CalibrationPoint>,
}

/// Calibration point
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationPoint {
    pub x: f64,
    pub y: f64,
    pub expected_value: f64,
    pub measured_value: f64,
    pub deviation: f64,
}

/// Gesture recognition result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GestureResult {
    pub gesture: GestureType,
    pub confidence: f64,
    pub start_time: DateTime<Utc>,
    pub end_time: DateTime<Utc>,
    pub touch_points: Vec<TouchPoint>,
    pub velocity: Option<Velocity>,
}

/// Touch point in gesture
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TouchPoint {
    pub x: f64,
    pub y: f64,
    pub pressure: f64,
    pub timestamp: DateTime<Utc>,
}

/// Velocity data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Velocity {
    pub vx: f64,
    pub vy: f64,
    pub magnitude: f64,
}

/// Touch device capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceCapabilities {
    pub device_id: String,
    pub supports_3d: bool,
    pub supports_haptic: bool,
    pub supports_force: bool,
    pub max_touch_points: u32,
    pub pressure_levels: u32,
    pub haptic_actuators: u32,
}
