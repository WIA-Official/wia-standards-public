//! WIA Eye Gaze Standard - Type Definitions
//!
//! 弘益人間 - 널리 인간을 이롭게

use serde::{Deserialize, Serialize};

// ============================================
// Basic Types
// ============================================

/// 2D coordinate
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct Vector2D {
    pub x: f64,
    pub y: f64,
}

impl Vector2D {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

/// 3D coordinate (mm)
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct Vector3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

/// Bounding box (normalized coordinates)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BoundingBox {
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
}

impl BoundingBox {
    pub fn new(x: f64, y: f64, width: f64, height: f64) -> Self {
        Self { x, y, width, height }
    }

    pub fn contains(&self, px: f64, py: f64) -> bool {
        px >= self.x && px <= self.x + self.width &&
        py >= self.y && py <= self.y + self.height
    }
}

// ============================================
// Gaze Data Types
// ============================================

/// Data for a single eye
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EyeData {
    pub gaze: Vector2D,
    pub valid: bool,
    pub pupil_diameter: Option<f64>,
    pub pupil_center: Option<Vector2D>,
    pub gaze_origin: Option<Vector3D>,
    pub gaze_direction: Option<Vector3D>,
    pub eye_openness: Option<f64>,
}

/// Single gaze data point
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GazePoint {
    pub timestamp: u64,
    pub x: f64,
    pub y: f64,
    pub confidence: f64,
    pub valid: bool,
    pub left_eye: Option<EyeData>,
    pub right_eye: Option<EyeData>,
    pub fixation: Option<bool>,
    pub saccade: Option<bool>,
    pub fixation_id: Option<String>,
    pub device_timestamp: Option<u64>,
}

impl GazePoint {
    pub fn new(timestamp: u64, x: f64, y: f64, confidence: f64, valid: bool) -> Self {
        Self {
            timestamp,
            x,
            y,
            confidence,
            valid,
            left_eye: None,
            right_eye: None,
            fixation: None,
            saccade: None,
            fixation_id: None,
            device_timestamp: None,
        }
    }
}

/// Gaze event types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GazeEventType {
    FixationStart,
    FixationUpdate,
    FixationEnd,
    SaccadeStart,
    SaccadeEnd,
    SmoothPursuitStart,
    SmoothPursuitEnd,
    BlinkStart,
    BlinkEnd,
    Blink,
    WinkLeft,
    WinkRight,
    DoubleBlink,
    DwellStart,
    DwellProgress,
    DwellComplete,
    DwellCancel,
    GazeEnter,
    GazeLeave,
    CalibrationStart,
    CalibrationPoint,
    CalibrationEnd,
    TrackingLost,
    TrackingRecovered,
    DeviceConnected,
    DeviceDisconnected,
}

/// Target semantic type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TargetSemanticType {
    Button,
    Link,
    Text,
    Input,
    Image,
    Video,
    Menu,
    MenuItem,
    ListItem,
    Scrollbar,
    KeyboardKey,
    AacSymbol,
    Custom,
}

/// Gaze target
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GazeTarget {
    pub element_id: String,
    pub bounding_box: BoundingBox,
    pub semantic_type: TargetSemanticType,
    pub label: Option<String>,
}

impl GazeTarget {
    pub fn new(
        element_id: impl Into<String>,
        bounding_box: BoundingBox,
        semantic_type: TargetSemanticType,
    ) -> Self {
        Self {
            element_id: element_id.into(),
            bounding_box,
            semantic_type,
            label: None,
        }
    }

    pub fn with_label(mut self, label: impl Into<String>) -> Self {
        self.label = Some(label.into());
        self
    }
}

/// Gaze event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GazeEvent {
    pub event_type: GazeEventType,
    pub timestamp: u64,
    pub event_id: String,
    pub duration: Option<u64>,
    pub position: Option<Vector2D>,
    pub target: Option<GazeTarget>,
    pub gaze_data: Option<GazePoint>,
}

// ============================================
// Device Capability Types
// ============================================

/// Device type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeviceType {
    ScreenBased,
    Wearable,
    Remote,
    Integrated,
    WebcamBased,
    Unknown,
}

/// Device information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EyeTrackerInfo {
    pub device_id: String,
    pub vendor: String,
    pub model: String,
    pub firmware_version: String,
    pub protocol_version: String,
    pub device_type: Option<DeviceType>,
}

/// Sampling rate specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SamplingRateSpec {
    pub supported: Vec<u32>,
    pub default: u32,
    pub current: Option<u32>,
}

/// Tracking capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackingCapabilities {
    pub binocular: bool,
    pub head_tracking: bool,
    pub gaze_3d: bool,
    pub sampling_rate: SamplingRateSpec,
    pub accuracy_typical: f64,
    pub precision_typical: f64,
    pub latency_average: f64,
}

/// Complete device capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EyeTrackerCapabilities {
    pub device: EyeTrackerInfo,
    pub tracking: TrackingCapabilities,
    pub supported_features: Vec<String>,
}

// ============================================
// Calibration Types
// ============================================

/// Calibration point
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CalibrationPoint {
    pub x: f64,
    pub y: f64,
    pub index: u32,
}

impl CalibrationPoint {
    pub fn new(x: f64, y: f64, index: u32) -> Self {
        Self { x, y, index }
    }
}

/// Point calibration result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PointCalibrationResult {
    pub point_index: u32,
    pub position: Vector2D,
    pub accuracy: f64,
    pub precision: f64,
    pub valid: bool,
}

/// Calibration result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationResult {
    pub success: bool,
    pub timestamp: u64,
    pub average_accuracy: Option<f64>,
    pub average_precision: Option<f64>,
    pub point_results: Option<Vec<PointCalibrationResult>>,
}

/// Calibration quality
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationQuality {
    pub overall: String,
    pub accuracy: f64,
    pub precision: f64,
    pub coverage: f64,
}

// ============================================
// Tracker Status Types
// ============================================

/// Tracker state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TrackerState {
    Disconnected,
    Connecting,
    Connected,
    Calibrating,
    Tracking,
    Error,
}

/// Tracker status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackerStatus {
    pub state: TrackerState,
    pub connected: bool,
    pub tracking: bool,
    pub calibrated: bool,
    pub error: Option<String>,
}

impl Default for TrackerStatus {
    fn default() -> Self {
        Self {
            state: TrackerState::Disconnected,
            connected: false,
            tracking: false,
            calibrated: false,
            error: None,
        }
    }
}

// ============================================
// Dwell Types
// ============================================

/// Dwell feedback type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DwellFeedbackType {
    CircularFill,
    LinearBar,
    ShrinkingRing,
    AudioOnly,
    Custom,
}

/// Dwell configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DwellConfig {
    pub threshold_ms: u64,
    pub progress_interval_ms: u64,
    pub visual_feedback: bool,
    pub cooldown_period_ms: u64,
}

impl Default for DwellConfig {
    fn default() -> Self {
        Self {
            threshold_ms: 800,
            progress_interval_ms: 50,
            visual_feedback: true,
            cooldown_period_ms: 500,
        }
    }
}

/// Dwell state
#[derive(Debug, Clone)]
pub struct DwellState {
    pub active: bool,
    pub progress: f64,
    pub target: Option<GazeTarget>,
    pub start_time: Option<u64>,
}

impl Default for DwellState {
    fn default() -> Self {
        Self {
            active: false,
            progress: 0.0,
            target: None,
            start_time: None,
        }
    }
}

// ============================================
// Error Types
// ============================================

/// Eye tracker error
#[derive(Debug, thiserror::Error)]
pub enum EyeTrackerError {
    #[error("Not connected")]
    NotConnected,

    #[error("Connection failed: {0}")]
    ConnectionFailed(String),

    #[error("Calibration failed: {0}")]
    CalibrationFailed(String),

    #[error("Tracking error: {0}")]
    TrackingError(String),

    #[error("Device error: {0}")]
    DeviceError(String),
}

pub type Result<T> = std::result::Result<T, EyeTrackerError>;
