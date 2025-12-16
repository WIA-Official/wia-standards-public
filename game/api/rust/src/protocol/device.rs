//! Device Types and Abstractions
//! 弘益人間 - Gaming for Everyone

use serde::{Deserialize, Serialize};
use uuid::Uuid;

/// Device identifier
pub type DeviceId = Uuid;

/// Connected input device
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectedDevice {
    pub device_id: DeviceId,
    pub name: String,
    pub device_type: DeviceType,
    pub connection: ConnectionType,
    pub vendor_id: u16,
    pub product_id: u16,
    pub battery_level: Option<u8>,
    pub capabilities: DeviceCapabilities,
}

/// Device type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeviceType {
    /// Standard gamepad/controller
    Gamepad,
    /// Xbox Adaptive Controller
    XboxAdaptive,
    /// PlayStation Access Controller
    PlayStationAccess,
    /// HORI Flex Controller
    HoriFlex,
    /// QuadStick
    QuadStick,
    /// Switch/button array
    SwitchArray,
    /// Eye tracker
    EyeTracker,
    /// Head tracker
    HeadTracker,
    /// Sip-and-puff device
    SipAndPuff,
    /// Foot pedals
    FootPedals,
    /// Mouth controller
    MouthController,
    /// Keyboard
    Keyboard,
    /// Mouse
    Mouse,
    /// Touch screen
    TouchScreen,
    /// Custom/unknown device
    Custom,
}

/// Connection type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConnectionType {
    Usb,
    Bluetooth,
    BluetoothLe,
    Wireless24Ghz,
    Wired,
    Virtual,
}

/// Device capabilities
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DeviceCapabilities {
    pub has_buttons: bool,
    pub button_count: u8,
    pub has_axes: bool,
    pub axis_count: u8,
    pub has_triggers: bool,
    pub has_dpad: bool,
    pub has_touchpad: bool,
    pub has_gyro: bool,
    pub has_accelerometer: bool,
    pub has_haptics: bool,
    pub has_led: bool,
    pub has_audio: bool,
    pub has_microphone: bool,
    pub supports_copilot: bool,
}

impl DeviceCapabilities {
    /// Capabilities for standard gamepad
    pub fn standard_gamepad() -> Self {
        Self {
            has_buttons: true,
            button_count: 16,
            has_axes: true,
            axis_count: 4,
            has_triggers: true,
            has_dpad: true,
            has_touchpad: false,
            has_gyro: false,
            has_accelerometer: false,
            has_haptics: true,
            has_led: true,
            has_audio: false,
            has_microphone: false,
            supports_copilot: true,
        }
    }

    /// Capabilities for Xbox Adaptive Controller
    pub fn xbox_adaptive() -> Self {
        Self {
            has_buttons: true,
            button_count: 19, // A, B, X, Y, LB, RB, View, Menu, Xbox, L3, R3, Dpad(4), Profile(4)
            has_axes: true,
            axis_count: 4,
            has_triggers: true,
            has_dpad: true,
            has_touchpad: false,
            has_gyro: false,
            has_accelerometer: false,
            has_haptics: true,
            has_led: true,
            has_audio: true,
            has_microphone: false,
            supports_copilot: true,
        }
    }

    /// Capabilities for eye tracker
    pub fn eye_tracker() -> Self {
        Self {
            has_buttons: false,
            button_count: 0,
            has_axes: false,
            axis_count: 0,
            has_triggers: false,
            has_dpad: false,
            has_touchpad: false,
            has_gyro: false,
            has_accelerometer: false,
            has_haptics: false,
            has_led: true,
            has_audio: false,
            has_microphone: false,
            supports_copilot: false,
        }
    }

    /// Capabilities for switch array
    pub fn switch_array(button_count: u8) -> Self {
        Self {
            has_buttons: true,
            button_count,
            has_axes: false,
            axis_count: 0,
            has_triggers: false,
            has_dpad: false,
            has_touchpad: false,
            has_gyro: false,
            has_accelerometer: false,
            has_haptics: false,
            has_led: false,
            has_audio: false,
            has_microphone: false,
            supports_copilot: false,
        }
    }
}

/// Device event for hotplug
#[derive(Debug, Clone)]
pub enum DeviceEvent {
    /// Device connected
    Connected { device_id: DeviceId, info: ConnectedDevice },
    /// Device disconnected
    Disconnected { device_id: DeviceId },
    /// Device configuration changed
    ConfigChanged { device_id: DeviceId },
    /// Battery low warning
    BatteryLow { device_id: DeviceId, level: u8 },
    /// Device error
    Error { device_id: DeviceId, error: DeviceError },
}

/// Device error types
#[derive(Debug, Clone)]
pub enum DeviceError {
    ConnectionLost,
    InitializationFailed,
    CommunicationError(String),
    UnsupportedFeature(String),
    Timeout,
}

/// Well-known vendor IDs
pub mod vendor {
    /// Microsoft
    pub const MICROSOFT: u16 = 0x045E;
    /// Sony
    pub const SONY: u16 = 0x054C;
    /// Nintendo
    pub const NINTENDO: u16 = 0x057E;
    /// HORI
    pub const HORI: u16 = 0x0F0D;
    /// Tobii
    pub const TOBII: u16 = 0x2104;
}

/// Well-known product IDs
pub mod product {
    /// Xbox Adaptive Controller
    pub const XBOX_ADAPTIVE: u16 = 0x0B12;
    /// Xbox Wireless Controller
    pub const XBOX_WIRELESS: u16 = 0x0B13;
    /// DualSense
    pub const DUALSENSE: u16 = 0x0CE6;
    /// DualShock 4
    pub const DUALSHOCK4: u16 = 0x05C4;
    /// HORI Flex
    pub const HORI_FLEX: u16 = 0x00C1;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_device_capabilities() {
        let standard = DeviceCapabilities::standard_gamepad();
        assert!(standard.has_buttons);
        assert_eq!(standard.button_count, 16);
        assert!(standard.has_haptics);

        let adaptive = DeviceCapabilities::xbox_adaptive();
        assert!(adaptive.supports_copilot);
        assert!(adaptive.has_audio);

        let eye = DeviceCapabilities::eye_tracker();
        assert!(!eye.has_buttons);
        assert!(!eye.supports_copilot);
    }

    #[test]
    fn test_vendor_ids() {
        assert_eq!(vendor::MICROSOFT, 0x045E);
        assert_eq!(vendor::SONY, 0x054C);
    }
}
