//! Prosthetics adapter
//!
//! Provides control for prosthetic hands, arms, and legs.

use crate::error::{RobotError, RobotResult};
use crate::types::Side;
use serde::{Deserialize, Serialize};

/// Prosthetic type
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ProstheticType {
    Hand,
    Arm,
    Leg,
    Foot,
}

/// Grip type for prosthetic hands
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum GripType {
    #[default]
    Power,
    Precision,
    Lateral,
    Hook,
    Tripod,
    Custom,
}

/// User grip intent detected from EMG
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GripIntent {
    Rest,
    Open,
    Close,
    Hold,
    Adjust,
}

/// Finger data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Finger {
    pub name: String,
    pub position: f64,
    pub velocity: f64,
    pub force_n: f64,
    pub target_position: f64,
    pub tactile_pressure: f64,
}

impl Finger {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            position: 0.0,
            velocity: 0.0,
            force_n: 0.0,
            target_position: 0.0,
            tactile_pressure: 0.0,
        }
    }

    /// Position error
    pub fn position_error(&self) -> f64 {
        self.target_position - self.position
    }
}

/// EMG sensor data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmgSensor {
    pub channel: u8,
    pub muscle_site: String,
    pub signal_mv: f64,
    pub signal_filtered_mv: f64,
    pub activation_level: f64,
    pub contraction_detected: bool,
}

impl EmgSensor {
    pub fn new(channel: u8, muscle_site: &str) -> Self {
        Self {
            channel,
            muscle_site: muscle_site.to_string(),
            signal_mv: 0.0,
            signal_filtered_mv: 0.0,
            activation_level: 0.0,
            contraction_detected: false,
        }
    }

    /// Update contraction detection based on threshold
    pub fn update_contraction(&mut self, threshold: f64) {
        self.contraction_detected = self.activation_level > threshold;
    }
}

/// Sensory feedback configuration
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct SensoryFeedback {
    pub tactile_enabled: bool,
    pub vibration_intensity: f64,
    pub temperature_c: Option<f64>,
}

/// Adaptation settings
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AdaptationSettings {
    pub terrain_mode: Option<TerrainMode>,
    pub speed_mode: Option<SpeedMode>,
    pub activity_mode: Option<ActivityMode>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum TerrainMode {
    Flat,
    Stairs,
    Ramp,
    Uneven,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum SpeedMode {
    Slow,
    Normal,
    Fast,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ActivityMode {
    Walking,
    Running,
    Sitting,
    Cycling,
    Standing,
}

/// Prosthetic specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProstheticSpec {
    pub prosthetic_type: ProstheticType,
    pub side: Side,
    pub dof: u8,
    pub fingers: Vec<Finger>,
    pub emg_sensors: Vec<EmgSensor>,
    pub grip_type: GripType,
    pub grip_force_n: f64,
    pub sensory_feedback: SensoryFeedback,
    pub adaptation: AdaptationSettings,
}

impl ProstheticSpec {
    /// Create a new prosthetic hand
    pub fn new_hand(side: Side) -> Self {
        Self {
            prosthetic_type: ProstheticType::Hand,
            side,
            dof: 6,
            fingers: vec![
                Finger::new("thumb"),
                Finger::new("index"),
                Finger::new("middle"),
                Finger::new("ring"),
                Finger::new("pinky"),
            ],
            emg_sensors: vec![
                EmgSensor::new(1, "flexor_carpi_radialis"),
                EmgSensor::new(2, "extensor_carpi_radialis"),
            ],
            grip_type: GripType::Power,
            grip_force_n: 0.0,
            sensory_feedback: SensoryFeedback::default(),
            adaptation: AdaptationSettings::default(),
        }
    }

    /// Classify user intent from EMG signals
    pub fn classify_intent(&self) -> RobotResult<GripIntent> {
        if self.emg_sensors.is_empty() {
            return Err(RobotError::SensorError(
                "No EMG sensors available".into(),
            ));
        }

        let avg_activation: f64 = self
            .emg_sensors
            .iter()
            .map(|s| s.activation_level)
            .sum::<f64>()
            / self.emg_sensors.len() as f64;

        let variance: f64 = self
            .emg_sensors
            .iter()
            .map(|s| (s.activation_level - avg_activation).powi(2))
            .sum::<f64>()
            / self.emg_sensors.len() as f64;

        if avg_activation < 0.2 {
            Ok(GripIntent::Rest)
        } else if avg_activation > 0.7 {
            Ok(GripIntent::Close)
        } else if variance > 0.1 {
            Ok(GripIntent::Adjust)
        } else {
            Ok(GripIntent::Hold)
        }
    }

    /// Control a finger position
    pub fn control_finger(&mut self, finger_name: &str, target: f64) -> RobotResult<()> {
        if target < 0.0 || target > 1.0 {
            return Err(RobotError::InvalidParameter(
                "Target position must be between 0.0 and 1.0".into(),
            ));
        }

        let finger = self
            .fingers
            .iter_mut()
            .find(|f| f.name == finger_name)
            .ok_or_else(|| {
                RobotError::InvalidParameter(format!("Finger not found: {}", finger_name))
            })?;

        finger.target_position = target;
        Ok(())
    }

    /// Set grip force with safety limits
    pub fn set_grip_force(&mut self, force_n: f64) -> RobotResult<()> {
        const MAX_GRIP_FORCE: f64 = 100.0;

        if force_n < 0.0 {
            return Err(RobotError::InvalidParameter(
                "Grip force must be non-negative".into(),
            ));
        }

        if force_n > MAX_GRIP_FORCE {
            return Err(RobotError::SafetyViolation(format!(
                "Grip force {:.1} N exceeds maximum {:.1} N",
                force_n, MAX_GRIP_FORCE
            )));
        }

        self.grip_force_n = force_n;
        Ok(())
    }

    /// Set all fingers to a grip pattern
    pub fn set_grip(&mut self, grip_type: GripType) -> RobotResult<()> {
        self.grip_type = grip_type;

        let positions = match grip_type {
            GripType::Power => [0.9, 0.9, 0.9, 0.9, 0.8],
            GripType::Precision => [0.6, 0.6, 0.3, 0.2, 0.2],
            GripType::Lateral => [0.5, 0.8, 0.3, 0.3, 0.3],
            GripType::Hook => [0.2, 0.9, 0.9, 0.9, 0.9],
            GripType::Tripod => [0.6, 0.6, 0.6, 0.3, 0.3],
            GripType::Custom => return Ok(()),
        };

        for (finger, &pos) in self.fingers.iter_mut().zip(positions.iter()) {
            finger.target_position = pos;
        }

        Ok(())
    }

    /// Open the hand (all fingers to 0)
    pub fn open(&mut self) {
        for finger in &mut self.fingers {
            finger.target_position = 0.0;
        }
    }

    /// Close the hand (all fingers to 1)
    pub fn close(&mut self) {
        for finger in &mut self.fingers {
            finger.target_position = 1.0;
        }
    }

    /// Get average tactile pressure
    pub fn average_tactile_pressure(&self) -> f64 {
        if self.fingers.is_empty() {
            return 0.0;
        }
        self.fingers.iter().map(|f| f.tactile_pressure).sum::<f64>()
            / self.fingers.len() as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_hand() {
        let hand = ProstheticSpec::new_hand(Side::Right);
        assert_eq!(hand.prosthetic_type, ProstheticType::Hand);
        assert_eq!(hand.fingers.len(), 5);
        assert_eq!(hand.emg_sensors.len(), 2);
    }

    #[test]
    fn test_classify_intent() {
        let hand = ProstheticSpec::new_hand(Side::Right);
        let intent = hand.classify_intent().unwrap();
        assert_eq!(intent, GripIntent::Rest);
    }

    #[test]
    fn test_control_finger() {
        let mut hand = ProstheticSpec::new_hand(Side::Right);
        assert!(hand.control_finger("thumb", 0.5).is_ok());
        assert!(hand.control_finger("thumb", 1.5).is_err());
        assert!(hand.control_finger("invalid", 0.5).is_err());
    }

    #[test]
    fn test_set_grip_force() {
        let mut hand = ProstheticSpec::new_hand(Side::Right);
        assert!(hand.set_grip_force(50.0).is_ok());
        assert!(hand.set_grip_force(150.0).is_err());
        assert!(hand.set_grip_force(-10.0).is_err());
    }

    #[test]
    fn test_set_grip() {
        let mut hand = ProstheticSpec::new_hand(Side::Right);
        assert!(hand.set_grip(GripType::Precision).is_ok());
        assert_eq!(hand.grip_type, GripType::Precision);
    }
}
