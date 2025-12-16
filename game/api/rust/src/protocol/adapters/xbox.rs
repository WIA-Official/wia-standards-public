//! Xbox Adaptive Controller Adapter
//! 弘益人間 - Gaming for Everyone

use super::DeviceAdapter;
use crate::error::{GameError, Result};
use crate::protocol::device::*;
use crate::protocol::event::*;
use crate::protocol::hid::*;
use async_trait::async_trait;
use std::sync::{Arc, Mutex};

/// Xbox Adaptive Controller adapter
#[derive(Debug)]
pub struct XboxAdaptiveAdapter {
    device: ConnectedDevice,
    connected: bool,
    last_report: Arc<Mutex<GamepadReport>>,
    event_queue: Arc<Mutex<Vec<InputEvent>>>,
}

impl XboxAdaptiveAdapter {
    /// Create a new Xbox Adaptive Controller adapter
    pub fn new(device_id: DeviceId) -> Self {
        let device = ConnectedDevice {
            device_id,
            name: "Xbox Adaptive Controller".to_string(),
            device_type: DeviceType::XboxAdaptive,
            connection: ConnectionType::Usb,
            vendor_id: vendor::MICROSOFT,
            product_id: product::XBOX_ADAPTIVE,
            battery_level: Some(100),
            capabilities: DeviceCapabilities::xbox_adaptive(),
        };

        Self {
            device,
            connected: false,
            last_report: Arc::new(Mutex::new(GamepadReport::default())),
            event_queue: Arc::new(Mutex::new(Vec::new())),
        }
    }

    /// Create for simulation/testing
    pub fn simulated() -> Self {
        Self::new(uuid::Uuid::new_v4())
    }

    /// Simulate a button press
    pub fn simulate_button_press(&self, button: Button) {
        let mut queue = self.event_queue.lock().unwrap();
        queue.push(InputEvent::ButtonPressed {
            device_id: self.device.device_id,
            button,
            timestamp: InputEvent::now(),
        });
    }

    /// Simulate a button release
    pub fn simulate_button_release(&self, button: Button) {
        let mut queue = self.event_queue.lock().unwrap();
        queue.push(InputEvent::ButtonReleased {
            device_id: self.device.device_id,
            button,
            timestamp: InputEvent::now(),
        });
    }

    /// Simulate axis movement
    pub fn simulate_axis_move(&self, axis: Axis, value: f32) {
        let mut queue = self.event_queue.lock().unwrap();
        queue.push(InputEvent::AxisMoved {
            device_id: self.device.device_id,
            axis,
            value,
            timestamp: InputEvent::now(),
        });
    }

    /// Process raw HID report
    pub fn process_report(&self, data: &[u8]) -> Vec<InputEvent> {
        if let Some(report) = parse_gamepad_report(data) {
            let mut last = self.last_report.lock().unwrap();
            let events = report.diff(&last, self.device.device_id);
            *last = report;
            events
        } else {
            Vec::new()
        }
    }

    /// Get button mapping for left USB port
    pub fn left_port_mapping() -> Vec<(u8, Button)> {
        vec![
            (0, Button::Extra1),      // X1
            (1, Button::Extra2),      // X2
            (2, Button::LeftStick),   // ThumbBtnL
            (3, Button::A),           // A
            (4, Button::B),           // B
            (5, Button::Select),      // View
            (6, Button::Start),       // Menu
        ]
    }

    /// Get button mapping for right USB port
    pub fn right_port_mapping() -> Vec<(u8, Button)> {
        vec![
            (0, Button::Share),       // View+Menu
            (1, Button::RightStick),  // ThumbBtnR
            (2, Button::X),           // X
            (3, Button::Y),           // Y
            (4, Button::Extra3),      // X1
            (5, Button::Extra4),      // X2
        ]
    }
}

#[async_trait]
impl DeviceAdapter for XboxAdaptiveAdapter {
    fn device_info(&self) -> &ConnectedDevice {
        &self.device
    }

    fn is_connected(&self) -> bool {
        self.connected
    }

    fn poll(&mut self) -> Vec<InputEvent> {
        let mut queue = self.event_queue.lock().unwrap();
        let events = queue.drain(..).collect();
        events
    }

    async fn start(&mut self) -> Result<()> {
        // In a real implementation, this would initialize USB/Bluetooth connection
        self.connected = true;
        Ok(())
    }

    async fn stop(&mut self) -> Result<()> {
        self.connected = false;
        Ok(())
    }

    async fn rumble(&self, left_motor: f32, right_motor: f32, duration_ms: u32) -> Result<()> {
        if !self.connected {
            return Err(GameError::DeviceNotConnected(self.device.device_id.to_string()));
        }

        // Validate values
        let left = (left_motor.clamp(0.0, 1.0) * 255.0) as u8;
        let right = (right_motor.clamp(0.0, 1.0) * 255.0) as u8;

        // In a real implementation, this would send HID output report
        println!(
            "🎮 XAC Rumble: left={}, right={}, duration={}ms",
            left, right, duration_ms
        );

        Ok(())
    }

    async fn set_led(&self, r: u8, g: u8, b: u8) -> Result<()> {
        // Xbox Adaptive Controller doesn't have RGB LEDs
        Err(GameError::FeatureNotSupported(
            "Xbox Adaptive Controller does not support RGB LED".to_string(),
        ))
    }
}

/// Xbox Wireless Controller adapter (standard controller)
#[derive(Debug)]
pub struct XboxWirelessAdapter {
    device: ConnectedDevice,
    connected: bool,
    last_report: GamepadReport,
}

impl XboxWirelessAdapter {
    pub fn new(device_id: DeviceId, connection: ConnectionType) -> Self {
        let device = ConnectedDevice {
            device_id,
            name: "Xbox Wireless Controller".to_string(),
            device_type: DeviceType::Gamepad,
            connection,
            vendor_id: vendor::MICROSOFT,
            product_id: product::XBOX_WIRELESS,
            battery_level: Some(100),
            capabilities: DeviceCapabilities::standard_gamepad(),
        };

        Self {
            device,
            connected: false,
            last_report: GamepadReport::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_xbox_adaptive_creation() {
        let adapter = XboxAdaptiveAdapter::simulated();
        assert_eq!(adapter.device.device_type, DeviceType::XboxAdaptive);
        assert!(adapter.device.capabilities.supports_copilot);
    }

    #[test]
    fn test_simulate_input() {
        let adapter = XboxAdaptiveAdapter::simulated();

        adapter.simulate_button_press(Button::A);
        adapter.simulate_axis_move(Axis::LeftStickX, 0.5);

        let mut adapter = adapter;
        let events = adapter.poll();

        assert_eq!(events.len(), 2);
    }

    #[tokio::test]
    async fn test_adapter_lifecycle() {
        let mut adapter = XboxAdaptiveAdapter::simulated();

        assert!(!adapter.is_connected());

        adapter.start().await.unwrap();
        assert!(adapter.is_connected());

        adapter.stop().await.unwrap();
        assert!(!adapter.is_connected());
    }

    #[test]
    fn test_port_mappings() {
        let left = XboxAdaptiveAdapter::left_port_mapping();
        let right = XboxAdaptiveAdapter::right_port_mapping();

        assert!(!left.is_empty());
        assert!(!right.is_empty());
    }
}
