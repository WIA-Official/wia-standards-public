//! WIA Game Protocol Module
//! 弘益人間 - Gaming for Everyone
//!
//! Communication protocol layer for adaptive controllers and input devices.

pub mod adapters;
pub mod device;
pub mod event;
pub mod hid;
pub mod macro_system;
pub mod switch_access;

pub use adapters::*;
pub use device::*;
pub use event::*;
pub use hid::*;
pub use macro_system::*;
pub use switch_access::*;

use crate::error::Result;
use crate::types::ControllerConfig;
use std::collections::HashMap;
use std::sync::{Arc, RwLock};

/// Protocol manager for handling input device communication
#[derive(Debug)]
pub struct ProtocolManager {
    devices: Arc<RwLock<HashMap<DeviceId, ConnectedDevice>>>,
    event_handlers: Arc<RwLock<Vec<Box<dyn EventHandler>>>>,
    macro_engine: MacroEngine,
    switch_controller: SwitchAccessController,
    copilot_config: Option<CopilotConfig>,
}

impl ProtocolManager {
    /// Create a new protocol manager
    pub fn new() -> Self {
        Self {
            devices: Arc::new(RwLock::new(HashMap::new())),
            event_handlers: Arc::new(RwLock::new(Vec::new())),
            macro_engine: MacroEngine::new(),
            switch_controller: SwitchAccessController::new(),
            copilot_config: None,
        }
    }

    /// Register a device
    pub fn register_device(&self, device: ConnectedDevice) -> Result<()> {
        let mut devices = self.devices.write().unwrap();
        devices.insert(device.device_id, device);
        Ok(())
    }

    /// Unregister a device
    pub fn unregister_device(&self, device_id: DeviceId) -> Result<()> {
        let mut devices = self.devices.write().unwrap();
        devices.remove(&device_id);
        Ok(())
    }

    /// Get all connected devices
    pub fn get_devices(&self) -> Vec<ConnectedDevice> {
        let devices = self.devices.read().unwrap();
        devices.values().cloned().collect()
    }

    /// Get a specific device
    pub fn get_device(&self, device_id: DeviceId) -> Option<ConnectedDevice> {
        let devices = self.devices.read().unwrap();
        devices.get(&device_id).cloned()
    }

    /// Add an event handler
    pub fn add_event_handler(&self, handler: Box<dyn EventHandler>) {
        let mut handlers = self.event_handlers.write().unwrap();
        handlers.push(handler);
    }

    /// Process an input event
    pub fn process_event(&mut self, event: InputEvent) -> Result<()> {
        // Check for macro triggers
        if let Some(macro_event) = self.macro_engine.check_trigger(&event) {
            self.dispatch_event(macro_event)?;
        }

        // Process through switch access if enabled
        if self.switch_controller.is_enabled() {
            if let Some(switch_event) = self.switch_controller.process_event(&event) {
                self.dispatch_event(switch_event)?;
            }
        }

        // Process copilot merge if enabled
        let final_event = if let Some(ref config) = self.copilot_config {
            self.merge_copilot_event(event, config)
        } else {
            event
        };

        self.dispatch_event(final_event)
    }

    /// Dispatch event to all handlers
    fn dispatch_event(&self, event: InputEvent) -> Result<()> {
        let handlers = self.event_handlers.read().unwrap();
        for handler in handlers.iter() {
            handler.handle_event(&event);
        }
        Ok(())
    }

    /// Merge events for copilot mode
    fn merge_copilot_event(&self, event: InputEvent, config: &CopilotConfig) -> InputEvent {
        // In a real implementation, this would merge inputs from two controllers
        // For now, just pass through
        event
    }

    /// Enable copilot mode
    pub fn enable_copilot(&mut self, config: CopilotConfig) {
        self.copilot_config = Some(config);
    }

    /// Disable copilot mode
    pub fn disable_copilot(&mut self) {
        self.copilot_config = None;
    }

    /// Get macro engine
    pub fn macro_engine(&mut self) -> &mut MacroEngine {
        &mut self.macro_engine
    }

    /// Get switch access controller
    pub fn switch_controller(&mut self) -> &mut SwitchAccessController {
        &mut self.switch_controller
    }

    /// Apply controller configuration
    pub fn apply_config(&self, device_id: DeviceId, config: &ControllerConfig) -> Result<()> {
        let devices = self.devices.read().unwrap();
        if devices.contains_key(&device_id) {
            // In a real implementation, this would configure the device
            Ok(())
        } else {
            Err(crate::error::GameError::DeviceNotConnected(device_id.to_string()))
        }
    }
}

impl Default for ProtocolManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Copilot configuration for two-controller operation
#[derive(Debug, Clone)]
pub struct CopilotConfig {
    pub enabled: bool,
    pub primary_device: DeviceId,
    pub secondary_device: DeviceId,
    pub merge_mode: MergeMode,
}

/// Merge mode for copilot inputs
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MergeMode {
    /// Use the first input received
    FirstInput,
    /// Use the last input received
    LastInput,
    /// Sum axis values
    Sum,
    /// Secondary overrides primary
    Override,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_protocol_manager() {
        let manager = ProtocolManager::new();

        let device = ConnectedDevice {
            device_id: uuid::Uuid::new_v4(),
            name: "Test Controller".to_string(),
            device_type: DeviceType::Gamepad,
            connection: ConnectionType::Usb,
            vendor_id: 0x045E,
            product_id: 0x0B12,
            battery_level: Some(75),
            capabilities: DeviceCapabilities::default(),
        };

        manager.register_device(device.clone()).unwrap();

        let devices = manager.get_devices();
        assert_eq!(devices.len(), 1);
        assert_eq!(devices[0].name, "Test Controller");
    }

    #[test]
    fn test_copilot_config() {
        let mut manager = ProtocolManager::new();

        let config = CopilotConfig {
            enabled: true,
            primary_device: uuid::Uuid::new_v4(),
            secondary_device: uuid::Uuid::new_v4(),
            merge_mode: MergeMode::Override,
        };

        manager.enable_copilot(config);
        assert!(manager.copilot_config.is_some());

        manager.disable_copilot();
        assert!(manager.copilot_config.is_none());
    }
}
