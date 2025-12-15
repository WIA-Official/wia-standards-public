//! WIA Exoskeleton Intent Detection Module
//!
//! This module provides intent detection interfaces for rehabilitation exoskeletons,
//! including EMG, GRF, and IMU-based detection with sensor fusion.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

// ============================================================================
// Enumerations
// ============================================================================

/// User movement intents
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum UserIntent {
    StandUp,
    SitDown,
    WalkForward,
    WalkBackward,
    TurnLeft,
    TurnRight,
    Stop,
    StairAscend,
    StairDescend,
    StepOver,
    Kick,
    Balance,
    Idle,
}

/// Intent detection sources
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IntentSource {
    /// Electromyography
    Emg,
    /// Ground Reaction Force
    Grf,
    /// Inertial Measurement Unit
    Imu,
    /// Physical buttons
    Button,
    /// Brain-Computer Interface
    Bci,
    /// Voice commands
    Voice,
}

/// Intent detection state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IntentState {
    Idle,
    Detecting,
    Confirmed,
    Uncertain,
    Rejected,
    Executing,
    Completed,
}

/// Confirmation mode for intent execution
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConfirmationMode {
    /// Automatic execution
    Auto,
    /// Confirm first detection
    ConfirmOnce,
    /// Always confirm
    ConfirmAlways,
    /// Manual input only
    ManualOnly,
}

/// Sensor fusion methods
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FusionMethod {
    /// Weighted average
    Weighted,
    /// Majority voting
    Voting,
    /// Bayesian fusion
    Bayesian,
    /// Dempster-Shafer
    DempsterShafer,
}

// ============================================================================
// Vector Types
// ============================================================================

/// 2D vector
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Vector2D {
    pub x: f64,
    pub y: f64,
}

/// 3D vector
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Vector3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3D {
    /// Calculate magnitude
    pub fn magnitude(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }
}

// ============================================================================
// EMG Types
// ============================================================================

/// EMG signal features
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EmgFeatures {
    /// Mean Absolute Value
    pub mav: f64,
    /// Root Mean Square
    pub rms: f64,
    /// Waveform Length
    pub wl: f64,
    /// Zero Crossings
    pub zc: u32,
    /// Slope Sign Changes
    pub ssc: u32,
    /// Integrated EMG
    pub iemg: f64,
    /// Normalized activation (% MVC)
    pub activation: f64,
}

/// EMG data from multiple channels
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmgData {
    pub timestamp: u64,
    pub channels: HashMap<u8, EmgFeatures>,
}

/// Extract EMG features from raw signal
pub fn extract_emg_features(signal: &[f64], mvc_value: f64) -> EmgFeatures {
    let n = signal.len();
    if n == 0 {
        return EmgFeatures::default();
    }

    // Mean Absolute Value
    let mav: f64 = signal.iter().map(|v| v.abs()).sum::<f64>() / n as f64;

    // Root Mean Square
    let rms = (signal.iter().map(|v| v.powi(2)).sum::<f64>() / n as f64).sqrt();

    // Waveform Length
    let wl: f64 = signal.windows(2).map(|w| (w[1] - w[0]).abs()).sum();

    // Zero Crossings
    let zc = signal
        .windows(2)
        .filter(|w| (w[0] > 0.0 && w[1] < 0.0) || (w[0] < 0.0 && w[1] > 0.0))
        .count() as u32;

    // Slope Sign Changes
    let ssc = signal
        .windows(3)
        .filter(|w| {
            let diff1 = w[1] - w[0];
            let diff2 = w[2] - w[1];
            diff1 * diff2 < 0.0
        })
        .count() as u32;

    // Integrated EMG
    let iemg: f64 = signal.iter().map(|v| v.abs()).sum();

    // Normalized activation
    let activation = if mvc_value > 0.0 { mav / mvc_value } else { 0.0 };

    EmgFeatures {
        mav,
        rms,
        wl,
        zc,
        ssc,
        iemg,
        activation,
    }
}

// ============================================================================
// GRF Types
// ============================================================================

/// Single foot GRF data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct FootGrf {
    /// Anterior-posterior force (N)
    pub fx: f64,
    /// Medial-lateral force (N)
    pub fy: f64,
    /// Vertical force (N)
    pub fz: f64,
    /// Center of Pressure (m)
    pub cop: Vector2D,
}

/// Ground Reaction Force data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GrfData {
    pub timestamp: u64,
    pub left: FootGrf,
    pub right: FootGrf,
}

/// GRF features for intent detection
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GrfFeatures {
    /// CoP shift in X direction (cm)
    pub cop_shift_x: f64,
    /// CoP shift in Y direction (cm)
    pub cop_shift_y: f64,
    /// Vertical force change (N)
    pub fz_change: f64,
    /// Left-right asymmetry ratio
    pub asymmetry: f64,
    /// Loading rate (N/s)
    pub loading_rate: f64,
    /// Total vertical force (N)
    pub total_vertical_force: f64,
}

// ============================================================================
// IMU Types
// ============================================================================

/// IMU sensor data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImuData {
    pub timestamp: u64,
    pub location: String,
    pub accelerometer: Vector3D,
    pub gyroscope: Vector3D,
    #[serde(default)]
    pub magnetometer: Option<Vector3D>,
    #[serde(default)]
    pub orientation: Option<Orientation>,
}

/// Orientation (Euler angles)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Orientation {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

/// IMU features for intent detection
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ImuFeatures {
    pub pelvis_pitch: f64,
    pub pelvis_roll: f64,
    pub pelvis_yaw: f64,
    pub pelvis_yaw_rate: f64,
    pub trunk_inclination: f64,
    pub acceleration_magnitude: f64,
    pub jerk: f64,
}

// ============================================================================
// Intent Detection Results
// ============================================================================

/// Result from a single source
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SourceResult {
    pub source: IntentSource,
    pub intent: UserIntent,
    pub confidence: f64,
    pub latency: u64,
}

/// Final intent detection result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IntentDetection {
    pub id: String,
    pub timestamp: u64,
    pub intent: UserIntent,
    pub confidence: f64,
    pub source: IntentSource,
    pub all_sources: Vec<SourceResult>,
    pub state: IntentState,
}

// ============================================================================
// Configuration
// ============================================================================

/// Intent detector configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IntentDetectorConfig {
    pub enabled_sources: Vec<IntentSource>,
    pub fusion_method: FusionMethod,
    pub confidence_threshold: f64,
    pub detection_window: u64,
    pub debounce_time: u64,
    pub max_latency: u64,
    pub confirmation_mode: ConfirmationMode,
}

impl Default for IntentDetectorConfig {
    fn default() -> Self {
        Self {
            enabled_sources: vec![IntentSource::Grf, IntentSource::Imu],
            fusion_method: FusionMethod::Weighted,
            confidence_threshold: 0.7,
            detection_window: 200,
            debounce_time: 500,
            max_latency: 300,
            confirmation_mode: ConfirmationMode::Auto,
        }
    }
}

// ============================================================================
// Source Detectors
// ============================================================================

/// GRF-based intent detector
pub struct GrfIntentDetector {
    previous_data: Option<GrfData>,
    force_threshold: f64,
}

impl GrfIntentDetector {
    pub fn new(force_threshold: f64) -> Self {
        Self {
            previous_data: None,
            force_threshold,
        }
    }

    /// Extract features from GRF data
    pub fn extract_features(&mut self, data: &GrfData) -> GrfFeatures {
        let avg_cop_x = (data.left.cop.x + data.right.cop.x) / 2.0;
        let avg_cop_y = (data.left.cop.y + data.right.cop.y) / 2.0;

        let (cop_shift_x, cop_shift_y, fz_change) = if let Some(prev) = &self.previous_data {
            let prev_cop_x = (prev.left.cop.x + prev.right.cop.x) / 2.0;
            let prev_cop_y = (prev.left.cop.y + prev.right.cop.y) / 2.0;
            let prev_fz = prev.left.fz + prev.right.fz;
            let current_fz = data.left.fz + data.right.fz;

            (
                (avg_cop_x - prev_cop_x) * 100.0,
                (avg_cop_y - prev_cop_y) * 100.0,
                current_fz - prev_fz,
            )
        } else {
            (0.0, 0.0, 0.0)
        };

        let total_vertical_force = data.left.fz + data.right.fz;
        let asymmetry = (data.left.fz - data.right.fz).abs()
            / (data.left.fz + data.right.fz + 0.001);

        self.previous_data = Some(data.clone());

        GrfFeatures {
            cop_shift_x,
            cop_shift_y,
            fz_change,
            asymmetry,
            loading_rate: fz_change.abs() * 200.0, // Assuming 200 Hz
            total_vertical_force,
        }
    }

    /// Detect intent from GRF data
    pub fn detect(&mut self, data: &GrfData) -> SourceResult {
        let start = now();
        let features = self.extract_features(data);

        let mut intent = UserIntent::Idle;
        let mut confidence = 0.0;

        // Step initiation
        if features.cop_shift_x > 3.0 && features.fz_change < -self.force_threshold {
            intent = UserIntent::WalkForward;
            confidence = (features.cop_shift_x.abs() / 5.0).min(1.0);
        }

        // Stand up
        if features.cop_shift_x > 3.0 && features.fz_change > self.force_threshold * 2.0 {
            intent = UserIntent::StandUp;
            confidence = (features.fz_change / 100.0).min(1.0);
        }

        // Sit down
        if features.cop_shift_x < -3.0 && features.fz_change < -self.force_threshold {
            intent = UserIntent::SitDown;
            confidence = (features.cop_shift_x.abs() / 5.0).min(1.0);
        }

        // Turn detection
        if features.cop_shift_y.abs() > 5.0 {
            intent = if features.cop_shift_y > 0.0 {
                UserIntent::TurnLeft
            } else {
                UserIntent::TurnRight
            };
            confidence = (features.cop_shift_y.abs() / 10.0).min(1.0);
        }

        SourceResult {
            source: IntentSource::Grf,
            intent,
            confidence,
            latency: now() - start,
        }
    }
}

/// IMU-based intent detector
pub struct ImuIntentDetector {
    previous_pelvis: Option<ImuData>,
    gyro_deadband: f64,
}

impl ImuIntentDetector {
    pub fn new(gyro_deadband: f64) -> Self {
        Self {
            previous_pelvis: None,
            gyro_deadband,
        }
    }

    /// Extract features from IMU data
    pub fn extract_features(&mut self, data: &[ImuData]) -> ImuFeatures {
        let pelvis = data.iter().find(|d| d.location == "pelvis");

        let Some(pelvis) = pelvis else {
            return ImuFeatures::default();
        };

        let orientation = pelvis.orientation.clone().unwrap_or_default();
        let acc_magnitude = pelvis.accelerometer.magnitude();

        let jerk = if let Some(prev) = &self.previous_pelvis {
            let dt = (pelvis.timestamp - prev.timestamp) as f64 / 1000.0;
            if dt > 0.0 {
                let dx = (pelvis.accelerometer.x - prev.accelerometer.x) / dt;
                let dy = (pelvis.accelerometer.y - prev.accelerometer.y) / dt;
                let dz = (pelvis.accelerometer.z - prev.accelerometer.z) / dt;
                (dx.powi(2) + dy.powi(2) + dz.powi(2)).sqrt()
            } else {
                0.0
            }
        } else {
            0.0
        };

        let trunk_inclination =
            (orientation.pitch.powi(2) + orientation.roll.powi(2)).sqrt();

        self.previous_pelvis = Some(pelvis.clone());

        ImuFeatures {
            pelvis_pitch: orientation.pitch,
            pelvis_roll: orientation.roll,
            pelvis_yaw: orientation.yaw,
            pelvis_yaw_rate: pelvis.gyroscope.z,
            trunk_inclination,
            acceleration_magnitude: acc_magnitude,
            jerk,
        }
    }

    /// Detect intent from IMU data
    pub fn detect(&mut self, data: &[ImuData]) -> SourceResult {
        let start = now();
        let features = self.extract_features(data);

        let mut intent = UserIntent::Idle;
        let mut confidence = 0.0;

        // Walk forward
        if features.pelvis_pitch > 5.0 {
            intent = UserIntent::WalkForward;
            confidence = (features.pelvis_pitch / 15.0).min(1.0);
        }

        // Walk backward
        if features.pelvis_pitch < -5.0 {
            intent = UserIntent::WalkBackward;
            confidence = (features.pelvis_pitch.abs() / 15.0).min(1.0);
        }

        // Turn detection
        if features.pelvis_yaw_rate.abs() > 10.0 {
            intent = if features.pelvis_yaw_rate < 0.0 {
                UserIntent::TurnLeft
            } else {
                UserIntent::TurnRight
            };
            confidence = (features.pelvis_yaw_rate.abs() / 30.0).min(1.0);
        }

        // Stair ascend
        if features.pelvis_pitch > 10.0 && features.jerk > 5.0 {
            intent = UserIntent::StairAscend;
            confidence = (features.pelvis_pitch / 20.0).min(1.0);
        }

        // Stair descend
        if features.pelvis_pitch < -5.0 && features.trunk_inclination > 10.0 {
            intent = UserIntent::StairDescend;
            confidence = (features.pelvis_pitch.abs() / 15.0).min(1.0);
        }

        SourceResult {
            source: IntentSource::Imu,
            intent,
            confidence,
            latency: now() - start,
        }
    }
}

// ============================================================================
// Intent Detector (Fusion)
// ============================================================================

/// Main intent detector with sensor fusion
pub struct IntentDetector {
    config: IntentDetectorConfig,
    grf_detector: Option<GrfIntentDetector>,
    imu_detector: Option<ImuIntentDetector>,
    source_results: HashMap<IntentSource, SourceResult>,
    last_detection: Option<IntentDetection>,
    last_detection_time: u64,
}

impl IntentDetector {
    /// Create a new intent detector
    pub fn new(config: IntentDetectorConfig) -> Self {
        let grf_detector = if config.enabled_sources.contains(&IntentSource::Grf) {
            Some(GrfIntentDetector::new(20.0))
        } else {
            None
        };

        let imu_detector = if config.enabled_sources.contains(&IntentSource::Imu) {
            Some(ImuIntentDetector::new(0.5))
        } else {
            None
        };

        Self {
            config,
            grf_detector,
            imu_detector,
            source_results: HashMap::new(),
            last_detection: None,
            last_detection_time: 0,
        }
    }

    /// Process GRF data
    pub fn process_grf(&mut self, data: &GrfData) {
        if let Some(detector) = &mut self.grf_detector {
            let result = detector.detect(data);
            self.source_results.insert(IntentSource::Grf, result);
        }
    }

    /// Process IMU data
    pub fn process_imu(&mut self, data: &[ImuData]) {
        if let Some(detector) = &mut self.imu_detector {
            let result = detector.detect(data);
            self.source_results.insert(IntentSource::Imu, result);
        }
    }

    /// Detect intent using sensor fusion
    pub fn detect_intent(&mut self) -> IntentDetection {
        let current_time = now();

        // Debounce check
        if current_time - self.last_detection_time < self.config.debounce_time {
            if let Some(detection) = &self.last_detection {
                return detection.clone();
            }
        }

        // Collect valid results
        let all_sources: Vec<SourceResult> = self
            .source_results
            .values()
            .filter(|r| r.latency <= self.config.max_latency)
            .cloned()
            .collect();

        if all_sources.is_empty() {
            return self.create_idle_detection();
        }

        // Fuse results
        let fused = self.fuse_results(&all_sources);

        // Check confidence threshold
        if fused.confidence < self.config.confidence_threshold {
            return self.create_idle_detection();
        }

        let detection = IntentDetection {
            id: generate_detection_id(),
            timestamp: current_time,
            intent: fused.intent,
            confidence: fused.confidence,
            source: fused.source,
            all_sources,
            state: IntentState::Confirmed,
        };

        self.last_detection = Some(detection.clone());
        self.last_detection_time = current_time;

        detection
    }

    /// Fuse results from multiple sources
    fn fuse_results(&self, results: &[SourceResult]) -> SourceResult {
        match self.config.fusion_method {
            FusionMethod::Weighted => self.weighted_fusion(results),
            FusionMethod::Voting => self.voting_fusion(results),
            _ => self.weighted_fusion(results),
        }
    }

    /// Weighted fusion algorithm
    fn weighted_fusion(&self, results: &[SourceResult]) -> SourceResult {
        let weights: HashMap<IntentSource, f64> = [
            (IntentSource::Button, 1.0),
            (IntentSource::Grf, 0.9),
            (IntentSource::Emg, 0.8),
            (IntentSource::Imu, 0.7),
            (IntentSource::Bci, 0.6),
            (IntentSource::Voice, 0.5),
        ]
        .into_iter()
        .collect();

        let mut intent_scores: HashMap<UserIntent, (f64, u32, IntentSource)> = HashMap::new();

        for result in results {
            if result.intent == UserIntent::Idle {
                continue;
            }

            let weight = weights.get(&result.source).copied().unwrap_or(0.5);
            let score = result.confidence * weight;

            intent_scores
                .entry(result.intent)
                .and_modify(|(s, c, src)| {
                    if score > *s / *c as f64 {
                        *src = result.source;
                    }
                    *s += score;
                    *c += 1;
                })
                .or_insert((score, 1, result.source));
        }

        let (best_intent, best_score, best_source) = intent_scores
            .into_iter()
            .map(|(intent, (score, count, source))| (intent, score / count as f64, source))
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap_or((UserIntent::Idle, 0.0, IntentSource::Imu));

        SourceResult {
            source: best_source,
            intent: best_intent,
            confidence: best_score,
            latency: results.iter().map(|r| r.latency).max().unwrap_or(0),
        }
    }

    /// Voting fusion algorithm
    fn voting_fusion(&self, results: &[SourceResult]) -> SourceResult {
        let mut votes: HashMap<UserIntent, u32> = HashMap::new();

        for result in results {
            if result.intent != UserIntent::Idle {
                *votes.entry(result.intent).or_insert(0) += 1;
            }
        }

        let (best_intent, max_votes) = votes
            .into_iter()
            .max_by_key(|(_, v)| *v)
            .unwrap_or((UserIntent::Idle, 0));

        let confidence = max_votes as f64 / results.len() as f64;
        let primary = results.iter().find(|r| r.intent == best_intent);

        SourceResult {
            source: primary.map(|r| r.source).unwrap_or(IntentSource::Imu),
            intent: best_intent,
            confidence,
            latency: results.iter().map(|r| r.latency).max().unwrap_or(0),
        }
    }

    /// Create idle detection
    fn create_idle_detection(&self) -> IntentDetection {
        IntentDetection {
            id: generate_detection_id(),
            timestamp: now(),
            intent: UserIntent::Idle,
            confidence: 1.0,
            source: IntentSource::Imu,
            all_sources: vec![],
            state: IntentState::Idle,
        }
    }

    /// Get last detection
    pub fn get_last_detection(&self) -> Option<&IntentDetection> {
        self.last_detection.as_ref()
    }

    /// Get configuration
    pub fn get_config(&self) -> &IntentDetectorConfig {
        &self.config
    }
}

// ============================================================================
// Utility Functions
// ============================================================================

/// Get current timestamp in milliseconds
fn now() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

/// Generate detection ID
fn generate_detection_id() -> String {
    format!("intent-{}-{:x}", now(), rand::random::<u32>())
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_emg_features() {
        let signal = vec![0.1, -0.2, 0.3, -0.1, 0.2];
        let features = extract_emg_features(&signal, 1.0);
        assert!(features.mav > 0.0);
        assert!(features.rms > 0.0);
    }

    #[test]
    fn test_vector3d_magnitude() {
        let v = Vector3D { x: 3.0, y: 4.0, z: 0.0 };
        assert!((v.magnitude() - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_intent_detector_creation() {
        let config = IntentDetectorConfig::default();
        let detector = IntentDetector::new(config);
        assert!(detector.grf_detector.is_some());
        assert!(detector.imu_detector.is_some());
    }

    #[test]
    fn test_idle_detection() {
        let config = IntentDetectorConfig::default();
        let mut detector = IntentDetector::new(config);
        let detection = detector.detect_intent();
        assert_eq!(detection.intent, UserIntent::Idle);
    }
}
