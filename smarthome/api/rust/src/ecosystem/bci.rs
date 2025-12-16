//! # BCI (Brain-Computer Interface) Smart Home Integration
//!
//! Enables brain-controlled smart home devices using EEG signals.

use super::*;
use crate::error::{Result, SmartHomeError};
use crate::types::DeviceId;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// BCI signal paradigms
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BCIParadigm {
    /// Steady-State Visual Evoked Potential
    SSVEP,
    /// P300 Event-Related Potential
    P300,
    /// Motor Imagery (left/right hand, feet)
    MotorImagery,
    /// Error-Related Potential
    ErrorPotential,
    /// Hybrid (combination of paradigms)
    Hybrid,
}

impl Default for BCIParadigm {
    fn default() -> Self {
        Self::SSVEP
    }
}

/// Motor imagery direction
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MotorImageryClass {
    LeftHand,
    RightHand,
    BothFeet,
    Tongue,
    Rest,
}

/// SSVEP frequency target
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SSVEPTarget {
    pub id: String,
    pub frequency_hz: f32,
    pub device_id: DeviceId,
    pub action: DeviceAction,
    pub label: String,
}

/// P300 target in speller/grid
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct P300Target {
    pub id: String,
    pub row: u32,
    pub col: u32,
    pub device_id: DeviceId,
    pub action: DeviceAction,
    pub label: String,
}

/// BCI classification result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BCIClassification {
    pub paradigm: BCIParadigm,
    pub class_id: String,
    pub confidence: f32,
    pub timestamp_ms: u64,
    pub raw_scores: Option<Vec<f32>>,
}

/// BCI adapter for smart home control
#[derive(Debug)]
pub struct BCIAdapter {
    config: BCIConfig,
    ssvep_targets: Vec<SSVEPTarget>,
    p300_targets: Vec<P300Target>,
    motor_imagery_mapping: HashMap<MotorImageryClass, (DeviceId, DeviceAction)>,
    calibration: BCICalibration,
    intent_buffer: IntentBuffer,
}

/// BCI configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BCIConfig {
    /// Minimum confidence to accept classification
    pub min_confidence: f32,
    /// Higher threshold for safety-critical actions
    pub safety_confidence: f32,
    /// Enable error potential detection
    pub error_detection_enabled: bool,
    /// Number of trials to average for P300
    pub p300_trials: u32,
    /// SSVEP detection window (seconds)
    pub ssvep_window_sec: f32,
    /// Require confirmation for actions
    pub confirmation_required: bool,
    /// Maximum session duration (minutes)
    pub max_session_minutes: u32,
}

impl Default for BCIConfig {
    fn default() -> Self {
        Self {
            min_confidence: 0.7,
            safety_confidence: 0.9,
            error_detection_enabled: true,
            p300_trials: 5,
            ssvep_window_sec: 2.0,
            confirmation_required: true,
            max_session_minutes: 30,
        }
    }
}

/// BCI calibration state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BCICalibration {
    pub is_calibrated: bool,
    pub paradigm: BCIParadigm,
    pub accuracy: f32,
    pub samples_collected: u32,
    pub last_calibration_ms: u64,
}

impl Default for BCICalibration {
    fn default() -> Self {
        Self {
            is_calibrated: false,
            paradigm: BCIParadigm::SSVEP,
            accuracy: 0.0,
            samples_collected: 0,
            last_calibration_ms: 0,
        }
    }
}

/// Buffer for intent confirmation
#[derive(Debug, Default)]
struct IntentBuffer {
    pending_intent: Option<BCIIntent>,
    confirmation_count: u32,
    last_intent_ms: u64,
}

/// Detected BCI intent
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BCIIntent {
    pub device_id: DeviceId,
    pub action: DeviceAction,
    pub paradigm: BCIParadigm,
    pub confidence: f32,
    pub requires_confirmation: bool,
}

impl BCIAdapter {
    pub fn new() -> Self {
        Self {
            config: BCIConfig::default(),
            ssvep_targets: Vec::new(),
            p300_targets: Vec::new(),
            motor_imagery_mapping: HashMap::new(),
            calibration: BCICalibration::default(),
            intent_buffer: IntentBuffer::default(),
        }
    }

    pub fn with_config(mut self, config: BCIConfig) -> Self {
        self.config = config;
        self
    }

    /// Add SSVEP target
    pub fn add_ssvep_target(&mut self, target: SSVEPTarget) {
        self.ssvep_targets.push(target);
    }

    /// Add P300 target
    pub fn add_p300_target(&mut self, target: P300Target) {
        self.p300_targets.push(target);
    }

    /// Set motor imagery mapping
    pub fn set_motor_imagery_mapping(
        &mut self,
        class: MotorImageryClass,
        device_id: DeviceId,
        action: DeviceAction,
    ) {
        self.motor_imagery_mapping.insert(class, (device_id, action));
    }

    /// Process BCI classification result
    pub fn process_classification(
        &mut self,
        classification: BCIClassification,
    ) -> Option<BCIIntent> {
        // Check minimum confidence
        if classification.confidence < self.config.min_confidence {
            return None;
        }

        // Find corresponding intent based on paradigm
        let intent = match classification.paradigm {
            BCIParadigm::SSVEP => self.process_ssvep(&classification),
            BCIParadigm::P300 => self.process_p300(&classification),
            BCIParadigm::MotorImagery => self.process_motor_imagery(&classification),
            BCIParadigm::ErrorPotential => {
                // Error detected - cancel pending intent
                self.intent_buffer.pending_intent = None;
                return None;
            }
            BCIParadigm::Hybrid => self.process_hybrid(&classification),
        };

        // Check if safety-critical action needs higher confidence
        if let Some(ref intent) = intent {
            if intent.requires_confirmation && classification.confidence < self.config.safety_confidence {
                return None;
            }
        }

        intent
    }

    fn process_ssvep(&self, classification: &BCIClassification) -> Option<BCIIntent> {
        // Find target by class_id (frequency)
        self.ssvep_targets
            .iter()
            .find(|t| t.id == classification.class_id)
            .map(|target| BCIIntent {
                device_id: target.device_id,
                action: target.action.clone(),
                paradigm: BCIParadigm::SSVEP,
                confidence: classification.confidence,
                requires_confirmation: self.is_safety_action(&target.action),
            })
    }

    fn process_p300(&self, classification: &BCIClassification) -> Option<BCIIntent> {
        // Find target by class_id (row_col format)
        self.p300_targets
            .iter()
            .find(|t| t.id == classification.class_id)
            .map(|target| BCIIntent {
                device_id: target.device_id,
                action: target.action.clone(),
                paradigm: BCIParadigm::P300,
                confidence: classification.confidence,
                requires_confirmation: self.is_safety_action(&target.action),
            })
    }

    fn process_motor_imagery(&self, classification: &BCIClassification) -> Option<BCIIntent> {
        // Parse motor imagery class from class_id
        let mi_class = match classification.class_id.as_str() {
            "left_hand" => MotorImageryClass::LeftHand,
            "right_hand" => MotorImageryClass::RightHand,
            "both_feet" => MotorImageryClass::BothFeet,
            "tongue" => MotorImageryClass::Tongue,
            _ => return None,
        };

        self.motor_imagery_mapping
            .get(&mi_class)
            .map(|(device_id, action)| BCIIntent {
                device_id: *device_id,
                action: action.clone(),
                paradigm: BCIParadigm::MotorImagery,
                confidence: classification.confidence,
                requires_confirmation: self.is_safety_action(action),
            })
    }

    fn process_hybrid(&self, classification: &BCIClassification) -> Option<BCIIntent> {
        // Hybrid processing - try SSVEP first, then P300
        self.process_ssvep(classification)
            .or_else(|| self.process_p300(classification))
    }

    fn is_safety_action(&self, action: &DeviceAction) -> bool {
        matches!(
            action,
            DeviceAction::Lock { .. } | DeviceAction::Door { .. }
        )
    }

    /// Convert BCI intent to unified command
    pub fn intent_to_command(&self, intent: BCIIntent) -> UnifiedCommand {
        let confirmation = if intent.requires_confirmation {
            ConfirmationRequirement::Double
        } else {
            ConfirmationRequirement::None
        };

        UnifiedCommand::new(
            CommandSource::BCI {
                paradigm: intent.paradigm,
                confidence: intent.confidence,
                channel_data: None,
            },
            CommandTarget::Device(intent.device_id),
            intent.action,
        )
        .with_confirmation(confirmation)
    }

    /// Set calibration state
    pub fn set_calibration(&mut self, calibration: BCICalibration) {
        self.calibration = calibration;
    }

    /// Check if calibrated
    pub fn is_calibrated(&self) -> bool {
        self.calibration.is_calibrated
    }

    /// Get calibration accuracy
    pub fn get_accuracy(&self) -> f32 {
        self.calibration.accuracy
    }

    /// Create default SSVEP setup for smart home
    pub fn setup_default_ssvep(&mut self, devices: Vec<(DeviceId, String, DeviceAction)>) {
        // Common SSVEP frequencies that don't cause discomfort
        let frequencies = [7.5, 8.57, 10.0, 12.0, 15.0, 20.0];

        for (idx, (device_id, label, action)) in devices.into_iter().enumerate() {
            if idx >= frequencies.len() {
                break;
            }

            self.add_ssvep_target(SSVEPTarget {
                id: format!("ssvep_{}", idx),
                frequency_hz: frequencies[idx],
                device_id,
                action,
                label,
            });
        }
    }

    /// Create default motor imagery mapping
    pub fn setup_default_motor_imagery(
        &mut self,
        on_device: DeviceId,
        off_device: DeviceId,
    ) {
        // Left hand = OFF, Right hand = ON (intuitive for most users)
        self.set_motor_imagery_mapping(
            MotorImageryClass::LeftHand,
            off_device,
            DeviceAction::Power { on: false },
        );
        self.set_motor_imagery_mapping(
            MotorImageryClass::RightHand,
            on_device,
            DeviceAction::Power { on: true },
        );
    }
}

impl Default for BCIAdapter {
    fn default() -> Self {
        Self::new()
    }
}

/// Trait for BCI smart home control
pub trait BCISmartHome {
    /// Process BCI intent into device command
    fn process_intent(&self, intent: BCIIntent) -> Result<DeviceAction>;
    /// Calibrate BCI for user
    fn calibrate(&mut self, samples: &[BCISample]) -> Result<()>;
    /// Get current confidence level
    fn get_confidence(&self) -> f32;
}

/// BCI sample for calibration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BCISample {
    pub channels: Vec<f32>,
    pub label: String,
    pub timestamp_ms: u64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bci_adapter_creation() {
        let adapter = BCIAdapter::new();
        assert!(!adapter.is_calibrated());
        assert_eq!(adapter.config.min_confidence, 0.7);
    }

    #[test]
    fn test_ssvep_processing() {
        let mut adapter = BCIAdapter::new();

        let device_id = Uuid::new_v4();
        adapter.add_ssvep_target(SSVEPTarget {
            id: "ssvep_0".to_string(),
            frequency_hz: 12.0,
            device_id,
            action: DeviceAction::Power { on: true },
            label: "Light On".to_string(),
        });

        let classification = BCIClassification {
            paradigm: BCIParadigm::SSVEP,
            class_id: "ssvep_0".to_string(),
            confidence: 0.85,
            timestamp_ms: 0,
            raw_scores: None,
        };

        let intent = adapter.process_classification(classification);
        assert!(intent.is_some());

        let intent = intent.unwrap();
        assert_eq!(intent.device_id, device_id);
        assert_eq!(intent.confidence, 0.85);
    }

    #[test]
    fn test_low_confidence_rejection() {
        let mut adapter = BCIAdapter::new();

        let device_id = Uuid::new_v4();
        adapter.add_ssvep_target(SSVEPTarget {
            id: "ssvep_0".to_string(),
            frequency_hz: 12.0,
            device_id,
            action: DeviceAction::Power { on: true },
            label: "Light On".to_string(),
        });

        let classification = BCIClassification {
            paradigm: BCIParadigm::SSVEP,
            class_id: "ssvep_0".to_string(),
            confidence: 0.5, // Below threshold
            timestamp_ms: 0,
            raw_scores: None,
        };

        let intent = adapter.process_classification(classification);
        assert!(intent.is_none());
    }

    #[test]
    fn test_motor_imagery_mapping() {
        let mut adapter = BCIAdapter::new();

        let device_id = Uuid::new_v4();
        adapter.set_motor_imagery_mapping(
            MotorImageryClass::RightHand,
            device_id,
            DeviceAction::Power { on: true },
        );

        let classification = BCIClassification {
            paradigm: BCIParadigm::MotorImagery,
            class_id: "right_hand".to_string(),
            confidence: 0.8,
            timestamp_ms: 0,
            raw_scores: None,
        };

        let intent = adapter.process_classification(classification);
        assert!(intent.is_some());
    }

    #[test]
    fn test_safety_action_detection() {
        let adapter = BCIAdapter::new();

        assert!(adapter.is_safety_action(&DeviceAction::Lock { locked: true }));
        assert!(adapter.is_safety_action(&DeviceAction::Door { open: true }));
        assert!(!adapter.is_safety_action(&DeviceAction::Power { on: true }));
    }
}
