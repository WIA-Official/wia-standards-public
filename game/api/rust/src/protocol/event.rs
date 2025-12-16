//! Input Event System
//! 弘益人間 - Gaming for Everyone

use super::device::DeviceId;
use serde::{Deserialize, Serialize};
use std::time::{SystemTime, UNIX_EPOCH};

/// Input event types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum InputEvent {
    /// Button pressed
    ButtonPressed {
        device_id: DeviceId,
        button: Button,
        timestamp: u64,
    },
    /// Button released
    ButtonReleased {
        device_id: DeviceId,
        button: Button,
        timestamp: u64,
    },
    /// Axis moved
    AxisMoved {
        device_id: DeviceId,
        axis: Axis,
        value: f32,
        timestamp: u64,
    },
    /// D-Pad direction
    DPadChanged {
        device_id: DeviceId,
        direction: DPadDirection,
        timestamp: u64,
    },
    /// Eye gaze point
    GazePoint {
        device_id: DeviceId,
        x: f32,
        y: f32,
        validity: GazeValidity,
        timestamp: u64,
    },
    /// Gaze with full data
    GazeData {
        device_id: DeviceId,
        data: FullGazeData,
        timestamp: u64,
    },
    /// Head pose
    HeadPose {
        device_id: DeviceId,
        yaw: f32,
        pitch: f32,
        roll: f32,
        timestamp: u64,
    },
    /// Voice command recognized
    VoiceCommand {
        command: String,
        confidence: f32,
        timestamp: u64,
    },
    /// Touch event
    Touch {
        device_id: DeviceId,
        touch_id: u8,
        x: f32,
        y: f32,
        state: TouchState,
        timestamp: u64,
    },
    /// Custom event
    Custom {
        device_id: DeviceId,
        event_type: String,
        data: Vec<u8>,
        timestamp: u64,
    },
}

impl InputEvent {
    /// Get timestamp of the event
    pub fn timestamp(&self) -> u64 {
        match self {
            InputEvent::ButtonPressed { timestamp, .. } => *timestamp,
            InputEvent::ButtonReleased { timestamp, .. } => *timestamp,
            InputEvent::AxisMoved { timestamp, .. } => *timestamp,
            InputEvent::DPadChanged { timestamp, .. } => *timestamp,
            InputEvent::GazePoint { timestamp, .. } => *timestamp,
            InputEvent::GazeData { timestamp, .. } => *timestamp,
            InputEvent::HeadPose { timestamp, .. } => *timestamp,
            InputEvent::VoiceCommand { timestamp, .. } => *timestamp,
            InputEvent::Touch { timestamp, .. } => *timestamp,
            InputEvent::Custom { timestamp, .. } => *timestamp,
        }
    }

    /// Get device ID if applicable
    pub fn device_id(&self) -> Option<DeviceId> {
        match self {
            InputEvent::ButtonPressed { device_id, .. } => Some(*device_id),
            InputEvent::ButtonReleased { device_id, .. } => Some(*device_id),
            InputEvent::AxisMoved { device_id, .. } => Some(*device_id),
            InputEvent::DPadChanged { device_id, .. } => Some(*device_id),
            InputEvent::GazePoint { device_id, .. } => Some(*device_id),
            InputEvent::GazeData { device_id, .. } => Some(*device_id),
            InputEvent::HeadPose { device_id, .. } => Some(*device_id),
            InputEvent::VoiceCommand { .. } => None,
            InputEvent::Touch { device_id, .. } => Some(*device_id),
            InputEvent::Custom { device_id, .. } => Some(*device_id),
        }
    }

    /// Create current timestamp in microseconds
    pub fn now() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_micros() as u64
    }
}

/// Standard button enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Button {
    // Face buttons
    A,
    B,
    X,
    Y,
    // PlayStation equivalents
    Cross,
    Circle,
    Square,
    Triangle,
    // Shoulder buttons
    LeftBumper,
    RightBumper,
    LeftTrigger,
    RightTrigger,
    // Special buttons
    Start,
    Select,
    Home,
    Share,
    // Stick clicks
    LeftStick,
    RightStick,
    // D-Pad as buttons
    DPadUp,
    DPadDown,
    DPadLeft,
    DPadRight,
    // Touchpad
    Touchpad,
    // Extra buttons (adaptive controllers)
    Extra1,
    Extra2,
    Extra3,
    Extra4,
    // Switch buttons (for switch arrays)
    Switch(u8),
}

/// Axis enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Axis {
    LeftStickX,
    LeftStickY,
    RightStickX,
    RightStickY,
    LeftTrigger,
    RightTrigger,
    // Gyro
    GyroX,
    GyroY,
    GyroZ,
    // Accelerometer
    AccelX,
    AccelY,
    AccelZ,
}

/// D-Pad direction
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DPadDirection {
    Neutral,
    Up,
    UpRight,
    Right,
    DownRight,
    Down,
    DownLeft,
    Left,
    UpLeft,
}

/// Gaze validity flags
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GazeValidity {
    pub left_eye: bool,
    pub right_eye: bool,
}

impl GazeValidity {
    pub fn both_valid() -> Self {
        Self {
            left_eye: true,
            right_eye: true,
        }
    }

    pub fn is_valid(&self) -> bool {
        self.left_eye || self.right_eye
    }
}

/// Full gaze data from eye tracker
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FullGazeData {
    /// Normalized screen coordinates (0.0-1.0)
    pub gaze_point: Point2D,
    /// Left eye gaze origin in mm
    pub left_eye_origin: Option<Point3D>,
    /// Right eye gaze origin in mm
    pub right_eye_origin: Option<Point3D>,
    /// Left eye gaze direction (unit vector)
    pub left_eye_direction: Option<Point3D>,
    /// Right eye gaze direction (unit vector)
    pub right_eye_direction: Option<Point3D>,
    /// Head position in mm
    pub head_position: Option<Point3D>,
    /// Head rotation
    pub head_rotation: Option<HeadRotation>,
    /// Validity flags
    pub validity: GazeValidity,
}

/// 2D point
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Point2D {
    pub x: f32,
    pub y: f32,
}

/// 3D point
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Point3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Head rotation
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct HeadRotation {
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
}

/// Touch state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TouchState {
    Began,
    Moved,
    Ended,
    Cancelled,
}

/// Event handler trait
pub trait EventHandler: Send + Sync + std::fmt::Debug {
    fn handle_event(&self, event: &InputEvent);
}

/// Event filter configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EventFilterConfig {
    /// Button debounce time in ms
    pub debounce_ms: u32,
    /// Axis dead zone (0.0-1.0)
    pub dead_zone: f32,
    /// Gaze smoothing factor (0.0-1.0)
    pub gaze_smoothing: f32,
    /// Trigger activation threshold (0.0-1.0)
    pub trigger_threshold: f32,
}

impl Default for EventFilterConfig {
    fn default() -> Self {
        Self {
            debounce_ms: 50,
            dead_zone: 0.15,
            gaze_smoothing: 0.3,
            trigger_threshold: 0.1,
        }
    }
}

/// Event filter for processing raw input
#[derive(Debug)]
pub struct EventFilter {
    config: EventFilterConfig,
    last_button_times: std::collections::HashMap<Button, u64>,
    smoothed_gaze: Point2D,
}

impl EventFilter {
    pub fn new(config: EventFilterConfig) -> Self {
        Self {
            config,
            last_button_times: std::collections::HashMap::new(),
            smoothed_gaze: Point2D::default(),
        }
    }

    /// Filter a button event (returns None if debounced)
    pub fn filter_button(&mut self, button: Button, timestamp: u64) -> bool {
        if let Some(&last_time) = self.last_button_times.get(&button) {
            let elapsed = timestamp.saturating_sub(last_time) / 1000; // Convert to ms
            if elapsed < self.config.debounce_ms as u64 {
                return false;
            }
        }
        self.last_button_times.insert(button, timestamp);
        true
    }

    /// Filter axis value (applies dead zone)
    pub fn filter_axis(&self, value: f32) -> f32 {
        if value.abs() < self.config.dead_zone {
            0.0
        } else {
            // Rescale to maintain full range after dead zone
            let sign = value.signum();
            let normalized = (value.abs() - self.config.dead_zone) / (1.0 - self.config.dead_zone);
            sign * normalized
        }
    }

    /// Filter trigger value (applies threshold)
    pub fn filter_trigger(&self, value: f32) -> f32 {
        if value < self.config.trigger_threshold {
            0.0
        } else {
            (value - self.config.trigger_threshold) / (1.0 - self.config.trigger_threshold)
        }
    }

    /// Smooth gaze point
    pub fn smooth_gaze(&mut self, point: Point2D) -> Point2D {
        let alpha = self.config.gaze_smoothing;
        self.smoothed_gaze.x = self.smoothed_gaze.x * alpha + point.x * (1.0 - alpha);
        self.smoothed_gaze.y = self.smoothed_gaze.y * alpha + point.y * (1.0 - alpha);
        self.smoothed_gaze
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_event_filter_dead_zone() {
        let filter = EventFilter::new(EventFilterConfig::default());

        // Value within dead zone
        assert_eq!(filter.filter_axis(0.1), 0.0);

        // Value outside dead zone
        let filtered = filter.filter_axis(0.5);
        assert!(filtered > 0.0);
        assert!(filtered < 0.5);
    }

    #[test]
    fn test_event_filter_debounce() {
        let mut filter = EventFilter::new(EventFilterConfig::default());

        let now = InputEvent::now();

        // First press should pass
        assert!(filter.filter_button(Button::A, now));

        // Immediate second press should be filtered
        assert!(!filter.filter_button(Button::A, now + 10_000)); // 10ms

        // After debounce time should pass
        assert!(filter.filter_button(Button::A, now + 100_000)); // 100ms
    }

    #[test]
    fn test_gaze_validity() {
        let valid = GazeValidity::both_valid();
        assert!(valid.is_valid());

        let invalid = GazeValidity {
            left_eye: false,
            right_eye: false,
        };
        assert!(!invalid.is_valid());
    }
}
