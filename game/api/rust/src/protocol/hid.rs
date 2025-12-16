//! HID Protocol Layer
//! 弘益人間 - Gaming for Everyone
//!
//! USB HID gamepad communication layer.

use super::device::{vendor, product, DeviceCapabilities, DeviceId, DeviceType};
use super::event::{Axis, Button, DPadDirection, InputEvent};
use serde::{Deserialize, Serialize};

/// HID Report type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HidReportType {
    Input,
    Output,
    Feature,
}

/// Gamepad HID report structure
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GamepadReport {
    /// Button states (bitfield)
    pub buttons: u32,
    /// Left stick X (-32768 to 32767)
    pub left_stick_x: i16,
    /// Left stick Y (-32768 to 32767)
    pub left_stick_y: i16,
    /// Right stick X (-32768 to 32767)
    pub right_stick_x: i16,
    /// Right stick Y (-32768 to 32767)
    pub right_stick_y: i16,
    /// Left trigger (0-255)
    pub left_trigger: u8,
    /// Right trigger (0-255)
    pub right_trigger: u8,
    /// D-Pad direction (0-8, 0=neutral)
    pub dpad: u8,
}

impl GamepadReport {
    /// Check if a button is pressed
    pub fn is_button_pressed(&self, button: HidButton) -> bool {
        (self.buttons & (1 << button as u32)) != 0
    }

    /// Get normalized axis value (-1.0 to 1.0)
    pub fn get_axis(&self, axis: Axis) -> f32 {
        match axis {
            Axis::LeftStickX => self.left_stick_x as f32 / 32767.0,
            Axis::LeftStickY => self.left_stick_y as f32 / 32767.0,
            Axis::RightStickX => self.right_stick_x as f32 / 32767.0,
            Axis::RightStickY => self.right_stick_y as f32 / 32767.0,
            Axis::LeftTrigger => self.left_trigger as f32 / 255.0,
            Axis::RightTrigger => self.right_trigger as f32 / 255.0,
            _ => 0.0,
        }
    }

    /// Get D-Pad direction
    pub fn get_dpad(&self) -> DPadDirection {
        match self.dpad {
            0 => DPadDirection::Up,
            1 => DPadDirection::UpRight,
            2 => DPadDirection::Right,
            3 => DPadDirection::DownRight,
            4 => DPadDirection::Down,
            5 => DPadDirection::DownLeft,
            6 => DPadDirection::Left,
            7 => DPadDirection::UpLeft,
            _ => DPadDirection::Neutral,
        }
    }

    /// Convert to input events by comparing with previous state
    pub fn diff(&self, previous: &GamepadReport, device_id: DeviceId) -> Vec<InputEvent> {
        let mut events = Vec::new();
        let timestamp = InputEvent::now();

        // Check button changes
        for i in 0..20 {
            let current = (self.buttons & (1 << i)) != 0;
            let prev = (previous.buttons & (1 << i)) != 0;

            if current != prev {
                if let Some(button) = HidButton::from_index(i).and_then(|b| b.to_standard()) {
                    if current {
                        events.push(InputEvent::ButtonPressed {
                            device_id,
                            button,
                            timestamp,
                        });
                    } else {
                        events.push(InputEvent::ButtonReleased {
                            device_id,
                            button,
                            timestamp,
                        });
                    }
                }
            }
        }

        // Check axis changes (with threshold)
        const AXIS_THRESHOLD: i16 = 1000;

        if (self.left_stick_x - previous.left_stick_x).abs() > AXIS_THRESHOLD {
            events.push(InputEvent::AxisMoved {
                device_id,
                axis: Axis::LeftStickX,
                value: self.get_axis(Axis::LeftStickX),
                timestamp,
            });
        }

        if (self.left_stick_y - previous.left_stick_y).abs() > AXIS_THRESHOLD {
            events.push(InputEvent::AxisMoved {
                device_id,
                axis: Axis::LeftStickY,
                value: self.get_axis(Axis::LeftStickY),
                timestamp,
            });
        }

        if (self.right_stick_x - previous.right_stick_x).abs() > AXIS_THRESHOLD {
            events.push(InputEvent::AxisMoved {
                device_id,
                axis: Axis::RightStickX,
                value: self.get_axis(Axis::RightStickX),
                timestamp,
            });
        }

        if (self.right_stick_y - previous.right_stick_y).abs() > AXIS_THRESHOLD {
            events.push(InputEvent::AxisMoved {
                device_id,
                axis: Axis::RightStickY,
                value: self.get_axis(Axis::RightStickY),
                timestamp,
            });
        }

        // Check D-Pad changes
        if self.dpad != previous.dpad {
            events.push(InputEvent::DPadChanged {
                device_id,
                direction: self.get_dpad(),
                timestamp,
            });
        }

        events
    }
}

/// HID button indices
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum HidButton {
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    LeftBumper = 4,
    RightBumper = 5,
    View = 6,
    Menu = 7,
    LeftStick = 8,
    RightStick = 9,
    Xbox = 10,
    Share = 11,
    // Extra buttons for adaptive controller
    Extra1 = 12,
    Extra2 = 13,
    Extra3 = 14,
    Extra4 = 15,
}

impl HidButton {
    /// Convert from bit index
    pub fn from_index(index: u32) -> Option<Self> {
        match index {
            0 => Some(Self::A),
            1 => Some(Self::B),
            2 => Some(Self::X),
            3 => Some(Self::Y),
            4 => Some(Self::LeftBumper),
            5 => Some(Self::RightBumper),
            6 => Some(Self::View),
            7 => Some(Self::Menu),
            8 => Some(Self::LeftStick),
            9 => Some(Self::RightStick),
            10 => Some(Self::Xbox),
            11 => Some(Self::Share),
            12 => Some(Self::Extra1),
            13 => Some(Self::Extra2),
            14 => Some(Self::Extra3),
            15 => Some(Self::Extra4),
            _ => None,
        }
    }

    /// Convert to standard button
    pub fn to_standard(&self) -> Option<Button> {
        match self {
            Self::A => Some(Button::A),
            Self::B => Some(Button::B),
            Self::X => Some(Button::X),
            Self::Y => Some(Button::Y),
            Self::LeftBumper => Some(Button::LeftBumper),
            Self::RightBumper => Some(Button::RightBumper),
            Self::View => Some(Button::Select),
            Self::Menu => Some(Button::Start),
            Self::LeftStick => Some(Button::LeftStick),
            Self::RightStick => Some(Button::RightStick),
            Self::Xbox => Some(Button::Home),
            Self::Share => Some(Button::Share),
            Self::Extra1 => Some(Button::Extra1),
            Self::Extra2 => Some(Button::Extra2),
            Self::Extra3 => Some(Button::Extra3),
            Self::Extra4 => Some(Button::Extra4),
        }
    }
}

/// HID device descriptor
#[derive(Debug, Clone)]
pub struct HidDescriptor {
    pub vendor_id: u16,
    pub product_id: u16,
    pub version: u16,
    pub usage_page: u16,
    pub usage: u16,
    pub input_report_length: u16,
    pub output_report_length: u16,
    pub feature_report_length: u16,
}

impl HidDescriptor {
    /// Create descriptor for Xbox Adaptive Controller
    pub fn xbox_adaptive() -> Self {
        Self {
            vendor_id: vendor::MICROSOFT,
            product_id: product::XBOX_ADAPTIVE,
            version: 0x0100,
            usage_page: 0x0001, // Generic Desktop
            usage: 0x0005,     // Gamepad
            input_report_length: 16,
            output_report_length: 8,
            feature_report_length: 0,
        }
    }

    /// Check if this is an Xbox controller
    pub fn is_xbox(&self) -> bool {
        self.vendor_id == vendor::MICROSOFT
            && (self.product_id == product::XBOX_ADAPTIVE || self.product_id == product::XBOX_WIRELESS)
    }

    /// Check if this is a PlayStation controller
    pub fn is_playstation(&self) -> bool {
        self.vendor_id == vendor::SONY
    }

    /// Get device type from descriptor
    pub fn device_type(&self) -> DeviceType {
        if self.vendor_id == vendor::MICROSOFT {
            if self.product_id == product::XBOX_ADAPTIVE {
                DeviceType::XboxAdaptive
            } else {
                DeviceType::Gamepad
            }
        } else if self.vendor_id == vendor::SONY {
            DeviceType::Gamepad
        } else if self.vendor_id == vendor::HORI {
            if self.product_id == product::HORI_FLEX {
                DeviceType::HoriFlex
            } else {
                DeviceType::Gamepad
            }
        } else {
            DeviceType::Gamepad
        }
    }

    /// Get device capabilities from descriptor
    pub fn capabilities(&self) -> DeviceCapabilities {
        match self.device_type() {
            DeviceType::XboxAdaptive => DeviceCapabilities::xbox_adaptive(),
            DeviceType::HoriFlex => DeviceCapabilities::standard_gamepad(),
            _ => DeviceCapabilities::standard_gamepad(),
        }
    }
}

/// Parse HID report from raw bytes
pub fn parse_gamepad_report(data: &[u8]) -> Option<GamepadReport> {
    if data.len() < 10 {
        return None;
    }

    Some(GamepadReport {
        buttons: u32::from_le_bytes([
            data.get(0).copied().unwrap_or(0),
            data.get(1).copied().unwrap_or(0),
            data.get(2).copied().unwrap_or(0),
            0,
        ]),
        left_stick_x: i16::from_le_bytes([
            data.get(3).copied().unwrap_or(0),
            data.get(4).copied().unwrap_or(0),
        ]),
        left_stick_y: i16::from_le_bytes([
            data.get(5).copied().unwrap_or(0),
            data.get(6).copied().unwrap_or(0),
        ]),
        right_stick_x: i16::from_le_bytes([
            data.get(7).copied().unwrap_or(0),
            data.get(8).copied().unwrap_or(0),
        ]),
        right_stick_y: i16::from_le_bytes([
            data.get(9).copied().unwrap_or(0),
            data.get(10).copied().unwrap_or(0),
        ]),
        left_trigger: data.get(11).copied().unwrap_or(0),
        right_trigger: data.get(12).copied().unwrap_or(0),
        dpad: data.get(13).copied().unwrap_or(8), // 8 = neutral
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gamepad_report_buttons() {
        let mut report = GamepadReport::default();
        report.buttons = 0b0000_0001; // A pressed

        assert!(report.is_button_pressed(HidButton::A));
        assert!(!report.is_button_pressed(HidButton::B));
    }

    #[test]
    fn test_gamepad_report_axes() {
        let mut report = GamepadReport::default();
        report.left_stick_x = 16383; // Half right

        let value = report.get_axis(Axis::LeftStickX);
        assert!(value > 0.4 && value < 0.6);
    }

    #[test]
    fn test_hid_descriptor() {
        let desc = HidDescriptor::xbox_adaptive();
        assert!(desc.is_xbox());
        assert_eq!(desc.device_type(), DeviceType::XboxAdaptive);
    }

    #[test]
    fn test_report_diff() {
        let device_id = uuid::Uuid::new_v4();

        let mut prev = GamepadReport::default();
        let mut curr = GamepadReport::default();
        curr.buttons = 0b0000_0001; // A pressed

        let events = curr.diff(&prev, device_id);
        assert!(!events.is_empty());

        match &events[0] {
            InputEvent::ButtonPressed { button, .. } => {
                assert_eq!(*button, Button::A);
            }
            _ => panic!("Expected ButtonPressed event"),
        }
    }
}
