//! Device management for WIA Robot SDK

use crate::error::{RobotError, RobotResult};
use crate::safety::SafetyStatus;
use crate::types::{DeviceInfo, DeviceState, DeviceStatus, RobotType};
use serde::{Deserialize, Serialize};

/// Generic robot device
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotDevice {
    pub info: DeviceInfo,
    pub state: DeviceState,
    pub safety: SafetyStatus,
}

impl RobotDevice {
    /// Create a new robot device
    pub fn new(
        id: &str,
        device_type: RobotType,
        name: &str,
        manufacturer: &str,
        model: &str,
    ) -> Self {
        Self {
            info: DeviceInfo {
                id: id.to_string(),
                device_type,
                name: name.to_string(),
                manufacturer: manufacturer.to_string(),
                model: model.to_string(),
                firmware_version: "1.0.0".to_string(),
                serial_number: None,
                capabilities: Vec::new(),
            },
            state: DeviceState::default(),
            safety: SafetyStatus::new_safe(),
        }
    }

    /// Check if device is operational
    pub fn is_operational(&self) -> bool {
        self.state.status == DeviceStatus::Operational && self.safety.is_safe()
    }

    /// Set device status
    pub fn set_status(&mut self, status: DeviceStatus) {
        self.state.status = status;
    }

    /// Add capability to device
    pub fn add_capability(&mut self, capability: &str) {
        self.info.capabilities.push(capability.to_string());
    }

    /// Check if device has capability
    pub fn has_capability(&self, capability: &str) -> bool {
        self.info.capabilities.iter().any(|c| c == capability)
    }

    /// Validate device for operation
    pub fn validate(&self) -> RobotResult<()> {
        self.safety.validate()?;

        if self.state.status == DeviceStatus::Error {
            return Err(RobotError::ControlError(
                "Device in error state".into(),
            ));
        }

        if self.state.status == DeviceStatus::Maintenance {
            return Err(RobotError::ControlError(
                "Device in maintenance".into(),
            ));
        }

        if self.state.battery_percent < 5 {
            return Err(RobotError::ControlError(
                "Battery critically low".into(),
            ));
        }

        Ok(())
    }

    /// Update uptime
    pub fn update_uptime(&mut self, seconds: u64) {
        self.state.uptime_seconds = seconds;
    }

    /// Set battery level
    pub fn set_battery(&mut self, percent: u8) {
        self.state.battery_percent = percent.min(100);
    }

    /// Trigger emergency stop
    pub fn emergency_stop(&mut self, source: crate::safety::EStopSource) {
        self.safety.trigger_estop(source);
        self.state.status = DeviceStatus::Error;
    }

    /// Reset from emergency stop
    pub fn reset(&mut self) -> RobotResult<()> {
        self.safety.reset_estop()?;
        self.state.status = DeviceStatus::Standby;
        Ok(())
    }
}

/// Device registry for managing multiple devices
#[derive(Debug, Default)]
pub struct DeviceRegistry {
    devices: Vec<RobotDevice>,
}

impl DeviceRegistry {
    /// Create a new device registry
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a device
    pub fn register(&mut self, device: RobotDevice) {
        self.devices.push(device);
    }

    /// Get device by ID
    pub fn get(&self, id: &str) -> Option<&RobotDevice> {
        self.devices.iter().find(|d| d.info.id == id)
    }

    /// Get mutable device by ID
    pub fn get_mut(&mut self, id: &str) -> Option<&mut RobotDevice> {
        self.devices.iter_mut().find(|d| d.info.id == id)
    }

    /// Get all devices of a specific type
    pub fn get_by_type(&self, device_type: RobotType) -> Vec<&RobotDevice> {
        self.devices
            .iter()
            .filter(|d| d.info.device_type == device_type)
            .collect()
    }

    /// Get all operational devices
    pub fn get_operational(&self) -> Vec<&RobotDevice> {
        self.devices.iter().filter(|d| d.is_operational()).collect()
    }

    /// Remove device by ID
    pub fn unregister(&mut self, id: &str) -> Option<RobotDevice> {
        if let Some(pos) = self.devices.iter().position(|d| d.info.id == id) {
            Some(self.devices.remove(pos))
        } else {
            None
        }
    }

    /// Get device count
    pub fn count(&self) -> usize {
        self.devices.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_robot_device_creation() {
        let device = RobotDevice::new(
            "device-001",
            RobotType::Exoskeleton,
            "Test Exo",
            "WIA",
            "Model 1",
        );
        assert_eq!(device.info.id, "device-001");
        assert_eq!(device.info.device_type, RobotType::Exoskeleton);
    }

    #[test]
    fn test_device_operational() {
        let mut device = RobotDevice::new(
            "device-001",
            RobotType::Exoskeleton,
            "Test Exo",
            "WIA",
            "Model 1",
        );
        device.set_status(DeviceStatus::Operational);
        assert!(device.is_operational());
    }

    #[test]
    fn test_device_registry() {
        let mut registry = DeviceRegistry::new();

        let device1 = RobotDevice::new("exo-001", RobotType::Exoskeleton, "Exo 1", "WIA", "M1");
        let device2 = RobotDevice::new("care-001", RobotType::CareRobot, "Care 1", "WIA", "M1");

        registry.register(device1);
        registry.register(device2);

        assert_eq!(registry.count(), 2);
        assert!(registry.get("exo-001").is_some());
        assert_eq!(registry.get_by_type(RobotType::Exoskeleton).len(), 1);
    }
}
