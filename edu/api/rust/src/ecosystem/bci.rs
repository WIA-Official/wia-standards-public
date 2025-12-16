//! BCI (Brain-Computer Interface) Integration
//! 弘益人間 - Enable BCI users to control learning activities

use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::error::{EduError, Result};
use crate::types::QuestionType;

/// BCI Education trait for learning control
pub trait BCIEducation: Send + Sync {
    /// Process BCI intent into learning action
    fn process_intent(&self, intent: &BCIIntent) -> Result<LearningAction>;

    /// Select answer from options using BCI
    fn select_answer(&self, options: &[AnswerOption], selection: &BCISelection) -> Result<SelectedAnswer>;

    /// Navigate content using BCI
    fn navigate_content(&self, direction: &NavigationIntent) -> Result<NavigationResult>;

    /// Calibrate BCI for the user
    fn calibrate(&mut self, samples: &[BCISample]) -> Result<CalibrationResult>;

    /// Get current confidence level
    fn get_confidence(&self) -> f32;

    /// Check if BCI is connected and ready
    fn is_ready(&self) -> bool;
}

/// BCI Intent types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum BCIIntent {
    /// P300 evoked potential (odd-ball paradigm)
    P300 {
        /// Target index selected
        target_index: usize,
        /// Confidence level
        confidence: f32,
    },
    /// SSVEP (Steady-State Visual Evoked Potential)
    SSVEP {
        /// Frequency detected (Hz)
        frequency: f32,
        /// Confidence level
        confidence: f32,
    },
    /// Motor Imagery
    MotorImagery {
        /// Imagined movement type
        movement: MotorImageryType,
        /// Confidence level
        confidence: f32,
    },
    /// Error-Related Negativity (for error detection)
    ERN {
        /// Error detected
        error_detected: bool,
        /// Confidence level
        confidence: f32,
    },
}

/// Motor imagery types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MotorImageryType {
    /// Left hand movement
    LeftHand,
    /// Right hand movement
    RightHand,
    /// Both hands
    BothHands,
    /// Feet movement
    Feet,
    /// Tongue movement
    Tongue,
    /// Rest (no imagery)
    Rest,
}

/// BCI Selection result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BCISelection {
    /// Selection type
    pub selection_type: SelectionType,
    /// Selected index
    pub index: usize,
    /// Confidence level
    pub confidence: f32,
    /// Time to selection (ms)
    pub time_ms: u32,
}

/// Selection type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SelectionType {
    /// P300-based selection
    P300,
    /// SSVEP-based selection
    SSVEP,
    /// Motor imagery binary selection
    MotorImagery,
}

/// Learning action from BCI
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LearningAction {
    /// Select an answer
    SelectAnswer { index: usize },
    /// Navigate forward
    NavigateForward,
    /// Navigate backward
    NavigateBack,
    /// Confirm current selection
    Confirm,
    /// Cancel/Undo
    Cancel,
    /// Request help
    RequestHelp,
    /// Pause learning
    Pause,
    /// Resume learning
    Resume,
    /// No action (noise/rest)
    NoAction,
}

/// Navigation intent
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NavigationIntent {
    /// Next page/item
    Next,
    /// Previous page/item
    Previous,
    /// Jump to beginning
    JumpToStart,
    /// Jump to end
    JumpToEnd,
    /// Scroll up
    ScrollUp,
    /// Scroll down
    ScrollDown,
}

/// Navigation result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavigationResult {
    /// Whether navigation succeeded
    pub success: bool,
    /// New position/page
    pub new_position: Option<usize>,
    /// Feedback message
    pub feedback: String,
}

/// Answer option for selection
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnswerOption {
    /// Option ID
    pub id: String,
    /// Option text
    pub text: String,
    /// Display order
    pub order: usize,
    /// SSVEP frequency for this option (Hz)
    pub ssvep_frequency: Option<f32>,
}

/// Selected answer
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SelectedAnswer {
    /// Answer ID
    pub answer_id: Uuid,
    /// Selected option
    pub option: AnswerOption,
    /// Selection confidence
    pub confidence: f32,
    /// BCI selection details
    pub selection: BCISelection,
}

/// BCI sample for calibration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BCISample {
    /// Timestamp (ms)
    pub timestamp: u64,
    /// Channel data
    pub channels: Vec<f32>,
    /// Label (for supervised calibration)
    pub label: Option<String>,
}

/// Calibration result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationResult {
    /// Calibration success
    pub success: bool,
    /// Accuracy achieved
    pub accuracy: f32,
    /// Number of samples used
    pub samples_used: usize,
    /// Feedback message
    pub message: String,
}

/// BCI Adapter implementation
pub struct BCIAdapter {
    /// Calibrated status
    calibrated: bool,
    /// Current confidence threshold
    confidence_threshold: f32,
    /// SSVEP frequency mappings
    ssvep_frequencies: Vec<f32>,
    /// P300 configuration
    p300_config: P300Config,
    /// Motor imagery classifier state
    mi_state: MotorImageryState,
    /// Connection status
    connected: bool,
}

/// P300 configuration
#[derive(Debug, Clone)]
struct P300Config {
    /// Flash duration (ms)
    flash_duration_ms: u32,
    /// Inter-stimulus interval (ms)
    isi_ms: u32,
    /// Number of repetitions
    repetitions: u32,
}

/// Motor imagery state
#[derive(Debug, Clone)]
struct MotorImageryState {
    /// Last detected movement
    last_movement: Option<MotorImageryType>,
    /// Cumulative confidence
    cumulative_confidence: f32,
}

impl BCIAdapter {
    /// Create a new BCI adapter
    pub fn new() -> Self {
        Self {
            calibrated: false,
            confidence_threshold: 0.7,
            ssvep_frequencies: vec![8.0, 10.0, 12.0, 15.0], // Common SSVEP frequencies
            p300_config: P300Config {
                flash_duration_ms: 100,
                isi_ms: 75,
                repetitions: 10,
            },
            mi_state: MotorImageryState {
                last_movement: None,
                cumulative_confidence: 0.0,
            },
            connected: true,
        }
    }

    /// Set confidence threshold
    pub fn set_confidence_threshold(&mut self, threshold: f32) {
        self.confidence_threshold = threshold.clamp(0.0, 1.0);
    }

    /// Set SSVEP frequencies for options
    pub fn set_ssvep_frequencies(&mut self, frequencies: Vec<f32>) {
        self.ssvep_frequencies = frequencies;
    }

    /// Create answer options with SSVEP frequencies
    pub fn create_ssvep_options(&self, texts: &[&str]) -> Vec<AnswerOption> {
        texts.iter().enumerate().map(|(i, text)| {
            AnswerOption {
                id: format!("option_{}", i),
                text: text.to_string(),
                order: i,
                ssvep_frequency: self.ssvep_frequencies.get(i).copied(),
            }
        }).collect()
    }

    /// Set connection status
    pub fn set_connected(&mut self, connected: bool) {
        self.connected = connected;
    }

    /// Map motor imagery to learning action
    fn map_motor_imagery(&self, movement: MotorImageryType) -> LearningAction {
        match movement {
            MotorImageryType::LeftHand => LearningAction::NavigateBack,
            MotorImageryType::RightHand => LearningAction::NavigateForward,
            MotorImageryType::BothHands => LearningAction::Confirm,
            MotorImageryType::Feet => LearningAction::Cancel,
            MotorImageryType::Tongue => LearningAction::RequestHelp,
            MotorImageryType::Rest => LearningAction::NoAction,
        }
    }

    /// Find SSVEP frequency index
    fn find_ssvep_index(&self, frequency: f32) -> Option<usize> {
        self.ssvep_frequencies.iter().position(|&f| (f - frequency).abs() < 0.5)
    }
}

impl Default for BCIAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl BCIEducation for BCIAdapter {
    fn process_intent(&self, intent: &BCIIntent) -> Result<LearningAction> {
        let (confidence, action) = match intent {
            BCIIntent::P300 { target_index, confidence } => {
                (*confidence, LearningAction::SelectAnswer { index: *target_index })
            }
            BCIIntent::SSVEP { frequency, confidence } => {
                if let Some(index) = self.find_ssvep_index(*frequency) {
                    (*confidence, LearningAction::SelectAnswer { index })
                } else {
                    return Err(EduError::ValidationError(
                        format!("Unknown SSVEP frequency: {}", frequency)
                    ));
                }
            }
            BCIIntent::MotorImagery { movement, confidence } => {
                (*confidence, self.map_motor_imagery(*movement))
            }
            BCIIntent::ERN { error_detected, confidence } => {
                if *error_detected {
                    (*confidence, LearningAction::Cancel)
                } else {
                    (*confidence, LearningAction::NoAction)
                }
            }
        };

        if confidence >= self.confidence_threshold {
            Ok(action)
        } else {
            Ok(LearningAction::NoAction)
        }
    }

    fn select_answer(&self, options: &[AnswerOption], selection: &BCISelection) -> Result<SelectedAnswer> {
        if selection.index >= options.len() {
            return Err(EduError::ValidationError(
                format!("Selection index {} out of bounds ({})", selection.index, options.len())
            ));
        }

        if selection.confidence < self.confidence_threshold {
            return Err(EduError::ValidationError(
                format!("Confidence {} below threshold {}", selection.confidence, self.confidence_threshold)
            ));
        }

        let option = options[selection.index].clone();

        Ok(SelectedAnswer {
            answer_id: Uuid::new_v4(),
            option,
            confidence: selection.confidence,
            selection: selection.clone(),
        })
    }

    fn navigate_content(&self, direction: &NavigationIntent) -> Result<NavigationResult> {
        let feedback = match direction {
            NavigationIntent::Next => "Moving to next item",
            NavigationIntent::Previous => "Moving to previous item",
            NavigationIntent::JumpToStart => "Jumping to start",
            NavigationIntent::JumpToEnd => "Jumping to end",
            NavigationIntent::ScrollUp => "Scrolling up",
            NavigationIntent::ScrollDown => "Scrolling down",
        };

        Ok(NavigationResult {
            success: true,
            new_position: None,
            feedback: feedback.to_string(),
        })
    }

    fn calibrate(&mut self, samples: &[BCISample]) -> Result<CalibrationResult> {
        if samples.len() < 10 {
            return Err(EduError::ValidationError(
                "Need at least 10 samples for calibration".to_string()
            ));
        }

        // Simulated calibration
        let accuracy = 0.85; // Simulated accuracy
        self.calibrated = accuracy >= 0.7;

        Ok(CalibrationResult {
            success: self.calibrated,
            accuracy,
            samples_used: samples.len(),
            message: if self.calibrated {
                format!("Calibration successful with {}% accuracy", (accuracy * 100.0) as u32)
            } else {
                "Calibration failed. Please try again.".to_string()
            },
        })
    }

    fn get_confidence(&self) -> f32 {
        if self.calibrated {
            0.85
        } else {
            0.0
        }
    }

    fn is_ready(&self) -> bool {
        self.connected && self.calibrated
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_bci_adapter() {
        let adapter = BCIAdapter::new();
        assert!(!adapter.is_ready()); // Not calibrated yet
        assert_eq!(adapter.get_confidence(), 0.0);
    }

    #[test]
    fn test_process_p300_intent() {
        let adapter = BCIAdapter::new();
        let intent = BCIIntent::P300 {
            target_index: 2,
            confidence: 0.9,
        };

        let action = adapter.process_intent(&intent).unwrap();
        match action {
            LearningAction::SelectAnswer { index } => assert_eq!(index, 2),
            _ => panic!("Expected SelectAnswer action"),
        }
    }

    #[test]
    fn test_process_ssvep_intent() {
        let adapter = BCIAdapter::new();
        let intent = BCIIntent::SSVEP {
            frequency: 10.0,
            confidence: 0.8,
        };

        let action = adapter.process_intent(&intent).unwrap();
        match action {
            LearningAction::SelectAnswer { index } => assert_eq!(index, 1), // 10Hz is index 1
            _ => panic!("Expected SelectAnswer action"),
        }
    }

    #[test]
    fn test_process_motor_imagery() {
        let adapter = BCIAdapter::new();
        let intent = BCIIntent::MotorImagery {
            movement: MotorImageryType::RightHand,
            confidence: 0.8,
        };

        let action = adapter.process_intent(&intent).unwrap();
        assert!(matches!(action, LearningAction::NavigateForward));
    }

    #[test]
    fn test_low_confidence_returns_no_action() {
        let adapter = BCIAdapter::new();
        let intent = BCIIntent::P300 {
            target_index: 1,
            confidence: 0.5, // Below threshold
        };

        let action = adapter.process_intent(&intent).unwrap();
        assert!(matches!(action, LearningAction::NoAction));
    }

    #[test]
    fn test_select_answer() {
        let adapter = BCIAdapter::new();
        let options = adapter.create_ssvep_options(&["A", "B", "C", "D"]);

        let selection = BCISelection {
            selection_type: SelectionType::P300,
            index: 1,
            confidence: 0.85,
            time_ms: 2500,
        };

        let answer = adapter.select_answer(&options, &selection).unwrap();
        assert_eq!(answer.option.text, "B");
        assert_eq!(answer.confidence, 0.85);
    }

    #[test]
    fn test_calibration() {
        let mut adapter = BCIAdapter::new();

        // Create sample data
        let samples: Vec<BCISample> = (0..20).map(|i| BCISample {
            timestamp: i * 100,
            channels: vec![0.0; 8],
            label: Some("left".to_string()),
        }).collect();

        let result = adapter.calibrate(&samples).unwrap();
        assert!(result.success);
        assert!(adapter.is_ready());
    }

    #[test]
    fn test_navigate_content() {
        let adapter = BCIAdapter::new();

        let result = adapter.navigate_content(&NavigationIntent::Next).unwrap();
        assert!(result.success);
        assert!(result.feedback.contains("next"));
    }
}
