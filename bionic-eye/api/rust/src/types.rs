//! Core type definitions for WIA Bionic Eye

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};

// ============================================================================
// Image and Frame Types
// ============================================================================

/// Captured frame from camera system
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CapturedFrame {
    pub frame_id: String,
    pub timestamp: i64,
    pub sequence_number: u64,
    pub device_id: Option<String>,
    pub image: ImageData,
    pub camera: CameraParams,
    pub sensors: Option<SensorData>,
}

/// Image data container
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImageData {
    pub width: u32,
    pub height: u32,
    pub format: ImageFormat,
    #[serde(with = "serde_bytes")]
    pub data: Vec<u8>,
    pub encoding: ImageEncoding,
    pub compression: Option<u8>,
}

/// Supported image formats
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ImageFormat {
    Gray8,
    Gray16,
    Rgb24,
    Rgbd32,
    Depth16,
    Ir8,
}

/// Image encoding types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ImageEncoding {
    Raw,
    Jpeg,
    H264,
    H265,
}

/// Camera parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraParams {
    pub fov: FieldOfView,
    pub exposure: f32,
    pub gain: f32,
    pub white_balance: u32,
    pub focus_distance: Option<f32>,
}

/// Field of view
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FieldOfView {
    pub horizontal: f32,
    pub vertical: f32,
}

/// Sensor data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorData {
    pub ambient_light: f32,
    pub proximity: f32,
    pub imu: ImuData,
    pub timestamp: i64,
}

/// IMU sensor data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImuData {
    pub accelerometer: [f32; 3],
    pub gyroscope: [f32; 3],
    pub magnetometer: Option<[f32; 3]>,
    pub orientation: Orientation,
}

/// Orientation data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Orientation {
    pub quaternion: Option<[f32; 4]>,
    pub euler: EulerAngles,
}

/// Euler angles
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EulerAngles {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

// ============================================================================
// Processed Visual Types
// ============================================================================

/// Processed visual data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProcessedVisual {
    pub frame_id: String,
    pub processing_time_ms: f32,
    pub objects: Vec<DetectedObject>,
    pub faces: Vec<DetectedFace>,
    pub depth: Option<DepthData>,
    pub text_regions: Vec<TextRegion>,
}

/// Detected object
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectedObject {
    pub object_id: String,
    pub label: String,
    pub label_korean: String,
    pub confidence: f32,
    pub bounding_box: BoundingBox,
    pub distance: f32,
    pub priority: ObjectPriority,
    pub threat: bool,
    pub moving: bool,
}

/// Bounding box (normalized 0-1)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BoundingBox {
    pub x: f32,
    pub y: f32,
    pub width: f32,
    pub height: f32,
}

/// Object priority levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[repr(u8)]
pub enum ObjectPriority {
    Low = 1,
    Medium = 2,
    High = 3,
    Critical = 4,
}

/// Detected face
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectedFace {
    pub face_id: String,
    pub bounding_box: BoundingBox,
    pub landmarks: FaceLandmarks,
    pub identity: Option<Identity>,
    pub distance: f32,
}

/// Face landmarks
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaceLandmarks {
    pub left_eye: [f32; 2],
    pub right_eye: [f32; 2],
    pub nose: [f32; 2],
    pub left_mouth: [f32; 2],
    pub right_mouth: [f32; 2],
}

/// Person identity
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Identity {
    pub name: String,
    pub confidence: f32,
}

/// Depth data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DepthData {
    pub width: u32,
    pub height: u32,
    pub min_distance: f32,
    pub max_distance: f32,
    #[serde(skip)]
    pub map: Vec<f32>,
}

/// Text region from OCR
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TextRegion {
    pub text: String,
    pub bounding_box: BoundingBox,
    pub confidence: f32,
    pub language: String,
}

// ============================================================================
// Stimulation Types
// ============================================================================

/// Stimulation parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StimulationParams {
    pub waveform: WaveformType,
    pub amplitude_ua: f32,
    pub pulse_width_us: f32,
    pub frequency_hz: f32,
    pub interphase_gap_us: f32,
    pub pulses_per_burst: u32,
}

/// Waveform types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WaveformType {
    BiphasicSymmetric,
    BiphasicAsymmetric,
    Triphasic,
}

/// Stimulation mapping strategy
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MappingStrategy {
    Direct,
    Scoreboard,
    AxonMap,
    Phosphene,
    Saliency,
}

/// Stimulation map for a frame
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StimulationMap {
    pub frame_id: String,
    pub timestamp: i64,
    pub active_electrodes: Vec<u32>,
    pub intensities: Vec<u8>,
    pub strategy: MappingStrategy,
}
