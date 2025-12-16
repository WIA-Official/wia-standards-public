//! # Haptic Feedback Smart Home Integration
//!
//! Enables tactile feedback for smart home device states and controls.

use super::*;
use crate::error::{Result, SmartHomeError};
use crate::types::DeviceId;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Haptic feedback patterns
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum HapticPattern {
    /// Single short pulse
    SinglePulse,
    /// Double pulse for confirmation
    DoublePulse,
    /// Triple pulse for error/alert
    TriplePulse,
    /// Continuous vibration
    Continuous,
    /// Rising intensity
    Ramp,
    /// Success feedback
    Success,
    /// Error feedback
    Error,
    /// Alert/warning
    Alert,
    /// Navigation pulse (directional)
    NavigationLeft,
    NavigationRight,
    NavigationUp,
    NavigationDown,
    /// Custom pattern
    Custom { pulses: u8, duration_ms: u32, intensity: u8 },
}

impl Default for HapticPattern {
    fn default() -> Self {
        Self::SinglePulse
    }
}

/// Haptic gesture input
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum HapticGesture {
    /// Single tap
    Tap,
    /// Double tap
    DoubleTap,
    /// Long press
    LongPress,
    /// Swipe left
    SwipeLeft,
    /// Swipe right
    SwipeRight,
    /// Swipe up
    SwipeUp,
    /// Swipe down
    SwipeDown,
    /// Circle gesture
    Circle,
}

/// Haptic feedback request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticFeedback {
    pub pattern: HapticPattern,
    pub intensity: u8,       // 0-100
    pub duration_ms: u32,
    pub repeat_count: u8,
    pub delay_between_ms: u32,
}

impl HapticFeedback {
    pub fn success() -> Self {
        Self {
            pattern: HapticPattern::Success,
            intensity: 70,
            duration_ms: 200,
            repeat_count: 1,
            delay_between_ms: 0,
        }
    }

    pub fn error() -> Self {
        Self {
            pattern: HapticPattern::Error,
            intensity: 100,
            duration_ms: 300,
            repeat_count: 3,
            delay_between_ms: 100,
        }
    }

    pub fn alert() -> Self {
        Self {
            pattern: HapticPattern::Alert,
            intensity: 100,
            duration_ms: 1000,
            repeat_count: 1,
            delay_between_ms: 0,
        }
    }

    pub fn navigation(direction: NavigationDirection) -> Self {
        let pattern = match direction {
            NavigationDirection::Left => HapticPattern::NavigationLeft,
            NavigationDirection::Right => HapticPattern::NavigationRight,
            NavigationDirection::Up => HapticPattern::NavigationUp,
            NavigationDirection::Down => HapticPattern::NavigationDown,
        };

        Self {
            pattern,
            intensity: 50,
            duration_ms: 100,
            repeat_count: 1,
            delay_between_ms: 0,
        }
    }
}

/// Navigation direction
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NavigationDirection {
    Left,
    Right,
    Up,
    Down,
}

/// Device state encoding for haptic feedback
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticStateEncoding {
    pub device_type: String,
    pub state_patterns: HashMap<String, HapticFeedback>,
}

/// Haptic adapter for smart home feedback
#[derive(Debug)]
pub struct HapticAdapter {
    config: HapticConfig,
    gesture_mapping: HashMap<HapticGesture, GestureAction>,
    state_encodings: HashMap<String, HapticStateEncoding>,
    feedback_queue: Vec<HapticFeedback>,
}

/// Haptic adapter configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticConfig {
    /// Enable haptic feedback
    pub enabled: bool,
    /// Global intensity multiplier (0.0-2.0)
    pub intensity_multiplier: f32,
    /// Minimum time between feedback (ms)
    pub debounce_ms: u32,
    /// Enable gesture input
    pub gesture_input_enabled: bool,
    /// Enable state feedback
    pub state_feedback_enabled: bool,
}

impl Default for HapticConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            intensity_multiplier: 1.0,
            debounce_ms: 100,
            gesture_input_enabled: true,
            state_feedback_enabled: true,
        }
    }
}

/// Action triggered by gesture
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GestureAction {
    pub device_id: Option<DeviceId>,
    pub action: DeviceAction,
    pub scope: GestureScope,
}

/// Scope of gesture action
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GestureScope {
    /// Apply to currently selected device
    SelectedDevice,
    /// Apply to all devices in zone
    CurrentZone,
    /// Apply to specific device
    SpecificDevice,
    /// Apply to scene
    Scene,
}

impl HapticAdapter {
    pub fn new() -> Self {
        let mut adapter = Self {
            config: HapticConfig::default(),
            gesture_mapping: HashMap::new(),
            state_encodings: HashMap::new(),
            feedback_queue: Vec::new(),
        };

        adapter.setup_default_gestures();
        adapter.setup_default_encodings();

        adapter
    }

    pub fn with_config(mut self, config: HapticConfig) -> Self {
        self.config = config;
        self
    }

    fn setup_default_gestures(&mut self) {
        // Tap = Toggle power
        self.gesture_mapping.insert(
            HapticGesture::Tap,
            GestureAction {
                device_id: None,
                action: DeviceAction::Power { on: true },
                scope: GestureScope::SelectedDevice,
            },
        );

        // Double tap = Toggle off
        self.gesture_mapping.insert(
            HapticGesture::DoubleTap,
            GestureAction {
                device_id: None,
                action: DeviceAction::Power { on: false },
                scope: GestureScope::SelectedDevice,
            },
        );

        // Long press = Emergency/help
        self.gesture_mapping.insert(
            HapticGesture::LongPress,
            GestureAction {
                device_id: None,
                action: DeviceAction::Custom {
                    name: "emergency".to_string(),
                    params: serde_json::json!({}),
                },
                scope: GestureScope::CurrentZone,
            },
        );

        // Swipe up = Increase brightness/volume
        self.gesture_mapping.insert(
            HapticGesture::SwipeUp,
            GestureAction {
                device_id: None,
                action: DeviceAction::Brightness { level: 100 },
                scope: GestureScope::SelectedDevice,
            },
        );

        // Swipe down = Decrease brightness/volume
        self.gesture_mapping.insert(
            HapticGesture::SwipeDown,
            GestureAction {
                device_id: None,
                action: DeviceAction::Brightness { level: 0 },
                scope: GestureScope::SelectedDevice,
            },
        );
    }

    fn setup_default_encodings(&mut self) {
        // Light state encoding
        let mut light_patterns = HashMap::new();
        light_patterns.insert(
            "off".to_string(),
            HapticFeedback {
                pattern: HapticPattern::SinglePulse,
                intensity: 0,
                duration_ms: 0,
                repeat_count: 0,
                delay_between_ms: 0,
            },
        );
        light_patterns.insert(
            "25".to_string(),
            HapticFeedback {
                pattern: HapticPattern::SinglePulse,
                intensity: 25,
                duration_ms: 100,
                repeat_count: 1,
                delay_between_ms: 1000,
            },
        );
        light_patterns.insert(
            "50".to_string(),
            HapticFeedback {
                pattern: HapticPattern::SinglePulse,
                intensity: 50,
                duration_ms: 100,
                repeat_count: 2,
                delay_between_ms: 500,
            },
        );
        light_patterns.insert(
            "75".to_string(),
            HapticFeedback {
                pattern: HapticPattern::SinglePulse,
                intensity: 75,
                duration_ms: 100,
                repeat_count: 3,
                delay_between_ms: 333,
            },
        );
        light_patterns.insert(
            "100".to_string(),
            HapticFeedback {
                pattern: HapticPattern::SinglePulse,
                intensity: 100,
                duration_ms: 100,
                repeat_count: 4,
                delay_between_ms: 250,
            },
        );

        self.state_encodings.insert(
            "light".to_string(),
            HapticStateEncoding {
                device_type: "light".to_string(),
                state_patterns: light_patterns,
            },
        );

        // Door state encoding
        let mut door_patterns = HashMap::new();
        door_patterns.insert(
            "locked".to_string(),
            HapticFeedback {
                pattern: HapticPattern::DoublePulse,
                intensity: 80,
                duration_ms: 150,
                repeat_count: 1,
                delay_between_ms: 0,
            },
        );
        door_patterns.insert(
            "unlocked".to_string(),
            HapticFeedback {
                pattern: HapticPattern::SinglePulse,
                intensity: 50,
                duration_ms: 100,
                repeat_count: 1,
                delay_between_ms: 0,
            },
        );
        door_patterns.insert(
            "open".to_string(),
            HapticFeedback {
                pattern: HapticPattern::Ramp,
                intensity: 60,
                duration_ms: 300,
                repeat_count: 1,
                delay_between_ms: 0,
            },
        );

        self.state_encodings.insert(
            "door".to_string(),
            HapticStateEncoding {
                device_type: "door".to_string(),
                state_patterns: door_patterns,
            },
        );
    }

    /// Set gesture mapping
    pub fn set_gesture(&mut self, gesture: HapticGesture, action: GestureAction) {
        self.gesture_mapping.insert(gesture, action);
    }

    /// Process gesture input
    pub fn process_gesture(&self, gesture: HapticGesture) -> Option<HapticGestureResult> {
        if !self.config.gesture_input_enabled {
            return None;
        }

        self.gesture_mapping.get(&gesture).map(|action| HapticGestureResult {
            gesture,
            action: action.clone(),
            feedback: HapticFeedback::success(),
        })
    }

    /// Get haptic feedback for device state
    pub fn get_state_feedback(
        &self,
        device_type: &str,
        state: &str,
    ) -> Option<HapticFeedback> {
        if !self.config.state_feedback_enabled {
            return None;
        }

        self.state_encodings
            .get(device_type)
            .and_then(|encoding| encoding.state_patterns.get(state))
            .map(|feedback| {
                let mut fb = feedback.clone();
                fb.intensity = (fb.intensity as f32 * self.config.intensity_multiplier) as u8;
                fb.intensity = fb.intensity.min(100);
                fb
            })
    }

    /// Generate feedback for command result
    pub fn get_result_feedback(&self, success: bool) -> HapticFeedback {
        let mut feedback = if success {
            HapticFeedback::success()
        } else {
            HapticFeedback::error()
        };

        feedback.intensity = (feedback.intensity as f32 * self.config.intensity_multiplier) as u8;
        feedback.intensity = feedback.intensity.min(100);

        feedback
    }

    /// Get navigation feedback
    pub fn get_navigation_feedback(&self, direction: NavigationDirection) -> HapticFeedback {
        let mut feedback = HapticFeedback::navigation(direction);
        feedback.intensity = (feedback.intensity as f32 * self.config.intensity_multiplier) as u8;
        feedback
    }

    /// Queue feedback
    pub fn queue_feedback(&mut self, feedback: HapticFeedback) {
        self.feedback_queue.push(feedback);
    }

    /// Get and clear feedback queue
    pub fn drain_feedback_queue(&mut self) -> Vec<HapticFeedback> {
        std::mem::take(&mut self.feedback_queue)
    }

    /// Convert gesture to unified command
    pub fn gesture_to_command(
        &self,
        result: HapticGestureResult,
        device_id: DeviceId,
    ) -> UnifiedCommand {
        UnifiedCommand::new(
            CommandSource::Haptic {
                gesture: result.gesture,
            },
            CommandTarget::Device(device_id),
            result.action.action,
        )
    }

    /// Add custom state encoding
    pub fn add_state_encoding(&mut self, device_type: String, encoding: HapticStateEncoding) {
        self.state_encodings.insert(device_type, encoding);
    }
}

impl Default for HapticAdapter {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of gesture processing
#[derive(Debug, Clone)]
pub struct HapticGestureResult {
    pub gesture: HapticGesture,
    pub action: GestureAction,
    pub feedback: HapticFeedback,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_haptic_adapter_creation() {
        let adapter = HapticAdapter::new();
        assert!(adapter.config.enabled);
    }

    #[test]
    fn test_gesture_processing() {
        let adapter = HapticAdapter::new();

        let result = adapter.process_gesture(HapticGesture::Tap);
        assert!(result.is_some());

        let result = result.unwrap();
        assert_eq!(result.gesture, HapticGesture::Tap);
    }

    #[test]
    fn test_state_feedback() {
        let adapter = HapticAdapter::new();

        let feedback = adapter.get_state_feedback("light", "50");
        assert!(feedback.is_some());

        let feedback = feedback.unwrap();
        assert_eq!(feedback.repeat_count, 2);
    }

    #[test]
    fn test_door_state_encoding() {
        let adapter = HapticAdapter::new();

        let locked = adapter.get_state_feedback("door", "locked");
        assert!(locked.is_some());
        assert!(matches!(
            locked.unwrap().pattern,
            HapticPattern::DoublePulse
        ));

        let unlocked = adapter.get_state_feedback("door", "unlocked");
        assert!(unlocked.is_some());
        assert!(matches!(
            unlocked.unwrap().pattern,
            HapticPattern::SinglePulse
        ));
    }

    #[test]
    fn test_intensity_multiplier() {
        let mut adapter = HapticAdapter::new();
        adapter.config.intensity_multiplier = 0.5;

        let feedback = adapter.get_state_feedback("light", "100");
        assert!(feedback.is_some());
        assert_eq!(feedback.unwrap().intensity, 50);
    }

    #[test]
    fn test_feedback_queue() {
        let mut adapter = HapticAdapter::new();

        adapter.queue_feedback(HapticFeedback::success());
        adapter.queue_feedback(HapticFeedback::alert());

        let queue = adapter.drain_feedback_queue();
        assert_eq!(queue.len(), 2);

        // Queue should be empty now
        let empty = adapter.drain_feedback_queue();
        assert!(empty.is_empty());
    }
}
