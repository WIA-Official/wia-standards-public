//! Matter Protocol Adapter
//! 弘益人間 - Benefit All Humanity

use super::clusters::*;
use crate::error::{Result, SmartHomeError};
use crate::protocol::messages::{
    AccessibilityContext as MsgAccessibilityContext, MessageType, WiaMessage,
};
use crate::types::DeviceType;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use uuid::Uuid;

/// Matter device representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MatterDevice {
    /// Node ID
    pub node_id: u64,
    /// Device ID
    pub device_id: Uuid,
    /// Device name
    pub name: String,
    /// Device type
    pub device_type: DeviceType,
    /// Vendor ID
    pub vendor_id: u16,
    /// Product ID
    pub product_id: u16,
    /// Supported clusters
    pub clusters: Vec<u16>,
    /// Online status
    pub online: bool,
    /// On/Off state (if supported)
    pub on_off_state: Option<OnOffState>,
    /// Level state (if supported)
    pub level_state: Option<LevelControlState>,
    /// Color state (if supported)
    pub color_state: Option<ColorControlState>,
    /// Lock state (if supported)
    pub lock_state: Option<DoorLockState>,
    /// Thermostat state (if supported)
    pub thermostat_state: Option<ThermostatState>,
}

impl MatterDevice {
    /// Create a new Matter device
    pub fn new(
        node_id: u64,
        device_id: Uuid,
        name: String,
        device_type: DeviceType,
    ) -> Self {
        let clusters = Self::default_clusters_for_type(&device_type);
        let mut device = Self {
            node_id,
            device_id,
            name,
            device_type,
            vendor_id: 0,
            product_id: 0,
            clusters: clusters.clone(),
            online: true,
            on_off_state: None,
            level_state: None,
            color_state: None,
            lock_state: None,
            thermostat_state: None,
        };

        // Initialize states based on clusters
        if clusters.contains(&cluster_ids::ON_OFF) {
            device.on_off_state = Some(OnOffState::default());
        }
        if clusters.contains(&cluster_ids::LEVEL_CONTROL) {
            device.level_state = Some(LevelControlState {
                max_level: 254,
                ..Default::default()
            });
        }
        if clusters.contains(&cluster_ids::COLOR_CONTROL) {
            device.color_state = Some(ColorControlState::default());
        }
        if clusters.contains(&cluster_ids::DOOR_LOCK) {
            device.lock_state = Some(DoorLockState::default());
        }
        if clusters.contains(&cluster_ids::THERMOSTAT) {
            device.thermostat_state = Some(ThermostatState::default());
        }

        device
    }

    /// Get default clusters for device type
    fn default_clusters_for_type(device_type: &DeviceType) -> Vec<u16> {
        match device_type {
            DeviceType::Light => vec![cluster_ids::ON_OFF],
            DeviceType::LightDimmer => vec![cluster_ids::ON_OFF, cluster_ids::LEVEL_CONTROL],
            DeviceType::LightColor => vec![
                cluster_ids::ON_OFF,
                cluster_ids::LEVEL_CONTROL,
                cluster_ids::COLOR_CONTROL,
            ],
            DeviceType::Switch | DeviceType::Outlet => vec![cluster_ids::ON_OFF],
            DeviceType::Lock => vec![cluster_ids::DOOR_LOCK],
            DeviceType::Thermostat => vec![cluster_ids::THERMOSTAT],
            DeviceType::Fan => vec![cluster_ids::ON_OFF, cluster_ids::FAN_CONTROL],
            DeviceType::Blind | DeviceType::Curtain => vec![cluster_ids::WINDOW_COVERING],
            _ => vec![cluster_ids::ON_OFF],
        }
    }

    /// Check if cluster is supported
    pub fn supports_cluster(&self, cluster_id: u16) -> bool {
        self.clusters.contains(&cluster_id)
    }

    /// Process a command
    pub fn process_command(
        &mut self,
        cluster_id: u16,
        command_id: u8,
        data: &[u8],
    ) -> Result<Vec<u8>> {
        if !self.supports_cluster(cluster_id) {
            return Err(SmartHomeError::CommandFailed(
                format!("Cluster 0x{:04X} not supported", cluster_id)
            ));
        }

        match cluster_id {
            cluster_ids::ON_OFF => {
                if let Some(ref mut state) = self.on_off_state {
                    state.process_command(command_id, data)
                        .map_err(|e| SmartHomeError::CommandFailed(e.to_string()))
                } else {
                    Err(SmartHomeError::CommandFailed("On/Off not initialized".to_string()))
                }
            }
            cluster_ids::LEVEL_CONTROL => {
                if let Some(ref mut state) = self.level_state {
                    // Simple level control implementation
                    match command_id {
                        level_control_commands::MOVE_TO_LEVEL
                        | level_control_commands::MOVE_TO_LEVEL_WITH_ON_OFF => {
                            if !data.is_empty() {
                                state.current_level = data[0].min(state.max_level);
                                // Also turn on if using MOVE_TO_LEVEL_WITH_ON_OFF
                                if command_id == level_control_commands::MOVE_TO_LEVEL_WITH_ON_OFF {
                                    if let Some(ref mut on_off) = self.on_off_state {
                                        on_off.on_off = state.current_level > 0;
                                    }
                                }
                                Ok(vec![0x00])
                            } else {
                                Err(SmartHomeError::CommandFailed("Missing level data".to_string()))
                            }
                        }
                        level_control_commands::STOP => Ok(vec![0x00]),
                        _ => Err(SmartHomeError::CommandFailed(
                            format!("Unknown level command: 0x{:02X}", command_id)
                        )),
                    }
                } else {
                    Err(SmartHomeError::CommandFailed("Level control not initialized".to_string()))
                }
            }
            cluster_ids::DOOR_LOCK => {
                if let Some(ref mut state) = self.lock_state {
                    match command_id {
                        0x00 => {
                            // Lock
                            state.lock_state = LockState::Locked;
                            Ok(vec![0x00])
                        }
                        0x01 => {
                            // Unlock
                            state.lock_state = LockState::Unlocked;
                            Ok(vec![0x00])
                        }
                        0x02 => {
                            // Toggle
                            state.lock_state = match state.lock_state {
                                LockState::Locked => LockState::Unlocked,
                                _ => LockState::Locked,
                            };
                            Ok(vec![0x00])
                        }
                        _ => Err(SmartHomeError::CommandFailed(
                            format!("Unknown lock command: 0x{:02X}", command_id)
                        )),
                    }
                } else {
                    Err(SmartHomeError::CommandFailed("Door lock not initialized".to_string()))
                }
            }
            cluster_ids::THERMOSTAT => {
                if let Some(ref mut state) = self.thermostat_state {
                    match command_id {
                        0x00 => {
                            // Set setpoint
                            if data.len() >= 2 {
                                let mode = data[0];
                                let amount = data[1] as i8;
                                match mode {
                                    0x00 => state.occupied_heating_setpoint += amount as i16 * 10,
                                    0x01 => state.occupied_cooling_setpoint += amount as i16 * 10,
                                    _ => {}
                                }
                                Ok(vec![0x00])
                            } else {
                                Err(SmartHomeError::CommandFailed("Missing setpoint data".to_string()))
                            }
                        }
                        _ => Err(SmartHomeError::CommandFailed(
                            format!("Unknown thermostat command: 0x{:02X}", command_id)
                        )),
                    }
                } else {
                    Err(SmartHomeError::CommandFailed("Thermostat not initialized".to_string()))
                }
            }
            _ => Err(SmartHomeError::CommandFailed(
                format!("Unimplemented cluster: 0x{:04X}", cluster_id)
            )),
        }
    }

    /// Get current state as HashMap
    pub fn get_state(&self) -> HashMap<String, serde_json::Value> {
        let mut state = HashMap::new();

        if let Some(ref on_off) = self.on_off_state {
            state.insert("power".to_string(), serde_json::json!(on_off.on_off));
        }

        if let Some(ref level) = self.level_state {
            state.insert("brightness".to_string(), serde_json::json!(level.current_level));
        }

        if let Some(ref color) = self.color_state {
            state.insert("hue".to_string(), serde_json::json!(color.current_hue));
            state.insert("saturation".to_string(), serde_json::json!(color.current_saturation));
            state.insert("color_temp".to_string(), serde_json::json!(color.color_temperature_mireds));
        }

        if let Some(ref lock) = self.lock_state {
            state.insert("locked".to_string(), serde_json::json!(lock.lock_state == LockState::Locked));
        }

        if let Some(ref thermo) = self.thermostat_state {
            state.insert("temperature".to_string(), serde_json::json!(thermo.local_temperature as f32 / 100.0));
            state.insert("cooling_setpoint".to_string(), serde_json::json!(thermo.occupied_cooling_setpoint as f32 / 100.0));
            state.insert("heating_setpoint".to_string(), serde_json::json!(thermo.occupied_heating_setpoint as f32 / 100.0));
        }

        state
    }
}

/// Matter Protocol Adapter
pub struct MatterAdapter {
    /// Local node ID
    local_node_id: u64,
    /// Registered devices
    devices: Arc<RwLock<HashMap<u64, MatterDevice>>>,
    /// Node ID to device ID mapping
    node_to_device: Arc<RwLock<HashMap<u64, Uuid>>>,
    /// Message counter
    message_counter: std::sync::atomic::AtomicU32,
}

impl MatterAdapter {
    /// Create a new Matter adapter
    pub fn new(local_node_id: u64) -> Self {
        Self {
            local_node_id,
            devices: Arc::new(RwLock::new(HashMap::new())),
            node_to_device: Arc::new(RwLock::new(HashMap::new())),
            message_counter: std::sync::atomic::AtomicU32::new(0),
        }
    }

    /// Register a device
    pub async fn register_device(&self, device: MatterDevice) -> Result<()> {
        let node_id = device.node_id;
        let device_id = device.device_id;

        self.devices.write().await.insert(node_id, device);
        self.node_to_device.write().await.insert(node_id, device_id);

        Ok(())
    }

    /// Unregister a device
    pub async fn unregister_device(&self, node_id: u64) -> Result<()> {
        self.devices.write().await.remove(&node_id);
        self.node_to_device.write().await.remove(&node_id);
        Ok(())
    }

    /// Get device by node ID
    pub async fn get_device(&self, node_id: u64) -> Option<MatterDevice> {
        self.devices.read().await.get(&node_id).cloned()
    }

    /// Find node ID by device ID
    pub async fn find_node_by_device_id(&self, device_id: Uuid) -> Option<u64> {
        let mapping = self.node_to_device.read().await;
        for (node_id, dev_id) in mapping.iter() {
            if *dev_id == device_id {
                return Some(*node_id);
            }
        }
        None
    }

    /// Send a command
    pub async fn send_command(
        &self,
        dest_node: u64,
        cluster_id: u16,
        command_id: u8,
        data: Vec<u8>,
        accessibility: Option<MsgAccessibilityContext>,
    ) -> Result<WiaMessage> {
        // Get device
        let mut devices = self.devices.write().await;
        let device = devices.get_mut(&dest_node)
            .ok_or(SmartHomeError::DeviceNotFound(Uuid::nil()))?;

        if !device.online {
            return Err(SmartHomeError::DeviceOffline(device.device_id));
        }

        // Process command
        let response_data = device.process_command(cluster_id, command_id, &data)?;

        // Create response message
        let mut response = WiaMessage::response(
            dest_node,
            self.local_node_id,
            self.message_counter.fetch_add(1, std::sync::atomic::Ordering::SeqCst),
            cluster_id,
            response_data,
        );

        if let Some(ctx) = accessibility {
            response = response.with_accessibility(ctx);
        }

        Ok(response)
    }

    /// Get device state
    pub async fn get_device_state(&self, node_id: u64) -> Result<HashMap<String, serde_json::Value>> {
        let devices = self.devices.read().await;
        let device = devices.get(&node_id)
            .ok_or(SmartHomeError::DeviceNotFound(Uuid::nil()))?;

        Ok(device.get_state())
    }

    /// Set device online status
    pub async fn set_device_online(&self, node_id: u64, online: bool) -> Result<()> {
        let mut devices = self.devices.write().await;
        let device = devices.get_mut(&node_id)
            .ok_or(SmartHomeError::DeviceNotFound(Uuid::nil()))?;

        device.online = online;
        Ok(())
    }

    /// List all devices
    pub async fn list_devices(&self) -> Vec<MatterDevice> {
        self.devices.read().await.values().cloned().collect()
    }

    /// Process incoming message
    pub async fn process_message(&self, message: WiaMessage) -> Result<Option<WiaMessage>> {
        match message.header.message_type {
            MessageType::Command => {
                let response = self.send_command(
                    message.header.dest_node,
                    message.payload.cluster_id,
                    message.payload.command_id,
                    message.payload.data,
                    message.payload.accessibility,
                ).await?;
                Ok(Some(response))
            }
            MessageType::Discovery => {
                // Return device list
                let devices = self.list_devices().await;
                let data = serde_json::to_vec(&devices)
                    .map_err(|e| SmartHomeError::Internal(e.to_string()))?;

                let response = WiaMessage::response(
                    self.local_node_id,
                    message.header.source_node,
                    message.header.message_id,
                    0,
                    data,
                );
                Ok(Some(response))
            }
            _ => Ok(None),
        }
    }
}

impl Default for MatterAdapter {
    fn default() -> Self {
        Self::new(1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_matter_device_creation() {
        let device = MatterDevice::new(
            1,
            Uuid::new_v4(),
            "Test Light".to_string(),
            DeviceType::LightDimmer,
        );

        assert!(device.supports_cluster(cluster_ids::ON_OFF));
        assert!(device.supports_cluster(cluster_ids::LEVEL_CONTROL));
        assert!(device.on_off_state.is_some());
        assert!(device.level_state.is_some());
    }

    #[tokio::test]
    async fn test_matter_adapter() {
        let adapter = MatterAdapter::new(0);
        let device_id = Uuid::new_v4();

        let device = MatterDevice::new(
            1,
            device_id,
            "Living Room Light".to_string(),
            DeviceType::Light,
        );

        adapter.register_device(device).await.unwrap();

        // Send On command
        let response = adapter.send_command(
            1,
            cluster_ids::ON_OFF,
            on_off_commands::ON,
            vec![],
            None,
        ).await.unwrap();

        assert_eq!(response.header.message_type, MessageType::Response);

        // Check state
        let state = adapter.get_device_state(1).await.unwrap();
        assert_eq!(state.get("power"), Some(&serde_json::json!(true)));
    }

    #[tokio::test]
    async fn test_lock_device() {
        let adapter = MatterAdapter::new(0);

        let device = MatterDevice::new(
            2,
            Uuid::new_v4(),
            "Front Door".to_string(),
            DeviceType::Lock,
        );

        adapter.register_device(device).await.unwrap();

        // Lock
        adapter.send_command(2, cluster_ids::DOOR_LOCK, 0x00, vec![], None).await.unwrap();

        let state = adapter.get_device_state(2).await.unwrap();
        assert_eq!(state.get("locked"), Some(&serde_json::json!(true)));

        // Unlock
        adapter.send_command(2, cluster_ids::DOOR_LOCK, 0x01, vec![], None).await.unwrap();

        let state = adapter.get_device_state(2).await.unwrap();
        assert_eq!(state.get("locked"), Some(&serde_json::json!(false)));
    }
}
