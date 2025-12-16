//! # Apple HomeKit Integration
//!
//! HomeKit Accessory Protocol (HAP) integration.

use super::*;
use crate::error::{Result, SmartHomeError};
use crate::types::DeviceId;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// HomeKit accessory categories
pub mod homekit_categories {
    pub const BRIDGE: u8 = 2;
    pub const LIGHTBULB: u8 = 5;
    pub const OUTLET: u8 = 7;
    pub const SWITCH: u8 = 8;
    pub const THERMOSTAT: u8 = 9;
    pub const DOOR: u8 = 12;
    pub const LOCK: u8 = 6;
    pub const FAN: u8 = 3;
    pub const SENSOR: u8 = 10;
}

/// HomeKit service types
pub mod homekit_services {
    pub const LIGHTBULB: &str = "43";
    pub const OUTLET: &str = "47";
    pub const SWITCH: &str = "49";
    pub const THERMOSTAT: &str = "4A";
    pub const LOCK_MECHANISM: &str = "45";
    pub const DOOR: &str = "81";
    pub const FAN: &str = "40";
    pub const TEMPERATURE_SENSOR: &str = "8A";
}

/// HomeKit characteristic types
pub mod homekit_characteristics {
    pub const ON: &str = "25";
    pub const BRIGHTNESS: &str = "08";
    pub const HUE: &str = "13";
    pub const SATURATION: &str = "2F";
    pub const CURRENT_TEMPERATURE: &str = "11";
    pub const TARGET_TEMPERATURE: &str = "35";
    pub const LOCK_CURRENT_STATE: &str = "1D";
    pub const LOCK_TARGET_STATE: &str = "1E";
    pub const CURRENT_DOOR_STATE: &str = "0E";
    pub const TARGET_DOOR_STATE: &str = "32";
}

/// HomeKit accessory
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HomeKitAccessory {
    pub aid: u64,
    pub services: Vec<HomeKitService>,
}

/// HomeKit service
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HomeKitService {
    pub iid: u64,
    #[serde(rename = "type")]
    pub service_type: String,
    pub characteristics: Vec<HomeKitCharacteristic>,
}

/// HomeKit characteristic
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HomeKitCharacteristic {
    pub iid: u64,
    #[serde(rename = "type")]
    pub char_type: String,
    pub value: Option<serde_json::Value>,
    pub perms: Vec<String>,
    pub format: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_value: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_value: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_step: Option<f64>,
}

/// HomeKit characteristic write request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HomeKitWriteRequest {
    pub characteristics: Vec<HomeKitCharWrite>,
}

/// Single characteristic write
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HomeKitCharWrite {
    pub aid: u64,
    pub iid: u64,
    pub value: serde_json::Value,
}

/// HomeKit characteristic read response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HomeKitReadResponse {
    pub characteristics: Vec<HomeKitCharRead>,
}

/// Single characteristic read
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HomeKitCharRead {
    pub aid: u64,
    pub iid: u64,
    pub value: serde_json::Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub status: Option<i32>,
}

/// HomeKit adapter
#[derive(Debug)]
pub struct HomeKitAdapter {
    config: HomeKitConfig,
    accessories: HashMap<u64, HomeKitAccessory>,
    device_mapping: HashMap<u64, DeviceId>,
    char_to_device: HashMap<(u64, u64), DeviceId>,
    connected: bool,
    last_sync_ms: u64,
}

/// HomeKit configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HomeKitConfig {
    pub bridge_name: String,
    pub manufacturer: String,
    pub model: String,
    pub serial_number: String,
    pub setup_code: String,
    pub accessibility_services: HomeKitAccessibility,
}

impl Default for HomeKitConfig {
    fn default() -> Self {
        Self {
            bridge_name: "WIA Smart Home".to_string(),
            manufacturer: "WIA".to_string(),
            model: "SmartHome Bridge".to_string(),
            serial_number: "WIA-001".to_string(),
            setup_code: "123-45-678".to_string(),
            accessibility_services: HomeKitAccessibility::default(),
        }
    }
}

/// HomeKit accessibility services
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HomeKitAccessibility {
    pub voiceover_support: bool,
    pub switch_control_support: bool,
    pub dwell_control_support: bool,
}

impl Default for HomeKitAccessibility {
    fn default() -> Self {
        Self {
            voiceover_support: true,
            switch_control_support: true,
            dwell_control_support: true,
        }
    }
}

impl HomeKitAdapter {
    pub fn new(config: HomeKitConfig) -> Self {
        let mut adapter = Self {
            config,
            accessories: HashMap::new(),
            device_mapping: HashMap::new(),
            char_to_device: HashMap::new(),
            connected: false,
            last_sync_ms: 0,
        };

        // Add bridge accessory (aid=1)
        adapter.add_bridge_accessory();

        adapter
    }

    fn add_bridge_accessory(&mut self) {
        let bridge = HomeKitAccessory {
            aid: 1,
            services: vec![
                // Accessory Information Service
                HomeKitService {
                    iid: 1,
                    service_type: "3E".to_string(), // Accessory Information
                    characteristics: vec![
                        HomeKitCharacteristic {
                            iid: 2,
                            char_type: "14".to_string(), // Identify
                            value: None,
                            perms: vec!["pw".to_string()],
                            format: "bool".to_string(),
                            min_value: None,
                            max_value: None,
                            min_step: None,
                        },
                        HomeKitCharacteristic {
                            iid: 3,
                            char_type: "20".to_string(), // Manufacturer
                            value: Some(serde_json::json!(self.config.manufacturer)),
                            perms: vec!["pr".to_string()],
                            format: "string".to_string(),
                            min_value: None,
                            max_value: None,
                            min_step: None,
                        },
                        HomeKitCharacteristic {
                            iid: 4,
                            char_type: "21".to_string(), // Model
                            value: Some(serde_json::json!(self.config.model)),
                            perms: vec!["pr".to_string()],
                            format: "string".to_string(),
                            min_value: None,
                            max_value: None,
                            min_step: None,
                        },
                        HomeKitCharacteristic {
                            iid: 5,
                            char_type: "23".to_string(), // Name
                            value: Some(serde_json::json!(self.config.bridge_name)),
                            perms: vec!["pr".to_string()],
                            format: "string".to_string(),
                            min_value: None,
                            max_value: None,
                            min_step: None,
                        },
                    ],
                },
            ],
        };

        self.accessories.insert(1, bridge);
    }

    /// Register a light accessory
    pub fn register_light(&mut self, device_id: DeviceId, name: &str) -> u64 {
        let aid = self.accessories.len() as u64 + 1;

        let accessory = HomeKitAccessory {
            aid,
            services: vec![
                // Accessory Information
                self.create_accessory_info_service(1, name),
                // Lightbulb Service
                HomeKitService {
                    iid: 10,
                    service_type: homekit_services::LIGHTBULB.to_string(),
                    characteristics: vec![
                        HomeKitCharacteristic {
                            iid: 11,
                            char_type: homekit_characteristics::ON.to_string(),
                            value: Some(serde_json::json!(false)),
                            perms: vec!["pr".to_string(), "pw".to_string(), "ev".to_string()],
                            format: "bool".to_string(),
                            min_value: None,
                            max_value: None,
                            min_step: None,
                        },
                        HomeKitCharacteristic {
                            iid: 12,
                            char_type: homekit_characteristics::BRIGHTNESS.to_string(),
                            value: Some(serde_json::json!(100)),
                            perms: vec!["pr".to_string(), "pw".to_string(), "ev".to_string()],
                            format: "int".to_string(),
                            min_value: Some(0.0),
                            max_value: Some(100.0),
                            min_step: Some(1.0),
                        },
                    ],
                },
            ],
        };

        self.accessories.insert(aid, accessory);
        self.device_mapping.insert(aid, device_id);
        self.char_to_device.insert((aid, 11), device_id);
        self.char_to_device.insert((aid, 12), device_id);

        aid
    }

    /// Register a lock accessory
    pub fn register_lock(&mut self, device_id: DeviceId, name: &str) -> u64 {
        let aid = self.accessories.len() as u64 + 1;

        let accessory = HomeKitAccessory {
            aid,
            services: vec![
                self.create_accessory_info_service(1, name),
                HomeKitService {
                    iid: 10,
                    service_type: homekit_services::LOCK_MECHANISM.to_string(),
                    characteristics: vec![
                        HomeKitCharacteristic {
                            iid: 11,
                            char_type: homekit_characteristics::LOCK_CURRENT_STATE.to_string(),
                            value: Some(serde_json::json!(1)), // 1 = Secured
                            perms: vec!["pr".to_string(), "ev".to_string()],
                            format: "uint8".to_string(),
                            min_value: Some(0.0),
                            max_value: Some(3.0),
                            min_step: Some(1.0),
                        },
                        HomeKitCharacteristic {
                            iid: 12,
                            char_type: homekit_characteristics::LOCK_TARGET_STATE.to_string(),
                            value: Some(serde_json::json!(1)),
                            perms: vec!["pr".to_string(), "pw".to_string(), "ev".to_string()],
                            format: "uint8".to_string(),
                            min_value: Some(0.0),
                            max_value: Some(1.0),
                            min_step: Some(1.0),
                        },
                    ],
                },
            ],
        };

        self.accessories.insert(aid, accessory);
        self.device_mapping.insert(aid, device_id);
        self.char_to_device.insert((aid, 12), device_id);

        aid
    }

    /// Register a thermostat accessory
    pub fn register_thermostat(&mut self, device_id: DeviceId, name: &str) -> u64 {
        let aid = self.accessories.len() as u64 + 1;

        let accessory = HomeKitAccessory {
            aid,
            services: vec![
                self.create_accessory_info_service(1, name),
                HomeKitService {
                    iid: 10,
                    service_type: homekit_services::THERMOSTAT.to_string(),
                    characteristics: vec![
                        HomeKitCharacteristic {
                            iid: 11,
                            char_type: homekit_characteristics::CURRENT_TEMPERATURE.to_string(),
                            value: Some(serde_json::json!(22.0)),
                            perms: vec!["pr".to_string(), "ev".to_string()],
                            format: "float".to_string(),
                            min_value: Some(0.0),
                            max_value: Some(100.0),
                            min_step: Some(0.1),
                        },
                        HomeKitCharacteristic {
                            iid: 12,
                            char_type: homekit_characteristics::TARGET_TEMPERATURE.to_string(),
                            value: Some(serde_json::json!(22.0)),
                            perms: vec!["pr".to_string(), "pw".to_string(), "ev".to_string()],
                            format: "float".to_string(),
                            min_value: Some(10.0),
                            max_value: Some(38.0),
                            min_step: Some(0.5),
                        },
                    ],
                },
            ],
        };

        self.accessories.insert(aid, accessory);
        self.device_mapping.insert(aid, device_id);
        self.char_to_device.insert((aid, 12), device_id);

        aid
    }

    fn create_accessory_info_service(&self, iid: u64, name: &str) -> HomeKitService {
        HomeKitService {
            iid,
            service_type: "3E".to_string(),
            characteristics: vec![
                HomeKitCharacteristic {
                    iid: iid + 1,
                    char_type: "23".to_string(), // Name
                    value: Some(serde_json::json!(name)),
                    perms: vec!["pr".to_string()],
                    format: "string".to_string(),
                    min_value: None,
                    max_value: None,
                    min_step: None,
                },
                HomeKitCharacteristic {
                    iid: iid + 2,
                    char_type: "20".to_string(), // Manufacturer
                    value: Some(serde_json::json!(self.config.manufacturer)),
                    perms: vec!["pr".to_string()],
                    format: "string".to_string(),
                    min_value: None,
                    max_value: None,
                    min_step: None,
                },
            ],
        }
    }

    /// Handle characteristic write
    pub async fn handle_write(&mut self, request: HomeKitWriteRequest) -> Result<()> {
        for write in request.characteristics {
            if let Some(accessory) = self.accessories.get_mut(&write.aid) {
                for service in &mut accessory.services {
                    for char in &mut service.characteristics {
                        if char.iid == write.iid {
                            char.value = Some(write.value.clone());
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// Handle characteristic read
    pub async fn handle_read(&self, aids_iids: &[(u64, u64)]) -> HomeKitReadResponse {
        let characteristics = aids_iids
            .iter()
            .filter_map(|(aid, iid)| {
                self.accessories.get(aid).and_then(|accessory| {
                    accessory
                        .services
                        .iter()
                        .flat_map(|s| &s.characteristics)
                        .find(|c| c.iid == *iid)
                        .map(|c| HomeKitCharRead {
                            aid: *aid,
                            iid: *iid,
                            value: c.value.clone().unwrap_or(serde_json::json!(null)),
                            status: Some(0),
                        })
                })
            })
            .collect();

        HomeKitReadResponse { characteristics }
    }

    /// Convert write to unified command
    pub fn write_to_command(&self, write: &HomeKitCharWrite) -> Option<UnifiedCommand> {
        let device_id = self.char_to_device.get(&(write.aid, write.iid))?;

        // Determine action based on characteristic type
        let accessory = self.accessories.get(&write.aid)?;
        let char_type = accessory
            .services
            .iter()
            .flat_map(|s| &s.characteristics)
            .find(|c| c.iid == write.iid)?
            .char_type
            .clone();

        let action = match char_type.as_str() {
            "25" => {
                // ON
                let on = write.value.as_bool().unwrap_or(false);
                DeviceAction::Power { on }
            }
            "08" => {
                // BRIGHTNESS
                let level = write.value.as_u64().unwrap_or(100) as u8;
                DeviceAction::Brightness { level }
            }
            "35" => {
                // TARGET_TEMPERATURE
                let celsius = write.value.as_f64().unwrap_or(22.0) as f32;
                DeviceAction::Temperature { celsius }
            }
            "1E" => {
                // LOCK_TARGET_STATE
                let locked = write.value.as_u64().unwrap_or(1) == 1;
                DeviceAction::Lock { locked }
            }
            _ => return None,
        };

        Some(UnifiedCommand::new(
            CommandSource::HomeKit {
                characteristic: char_type,
                value: write.value.clone(),
            },
            CommandTarget::Device(*device_id),
            action,
        ))
    }

    /// Get all accessories
    pub fn get_accessories(&self) -> Vec<&HomeKitAccessory> {
        self.accessories.values().collect()
    }

    /// Get status
    pub fn get_status(&self) -> PlatformStatus {
        PlatformStatus {
            platform: ExternalPlatform::HomeKit,
            connected: self.connected,
            last_sync_ms: self.last_sync_ms,
            device_count: (self.accessories.len() - 1) as u32, // Exclude bridge
            error: None,
        }
    }

    /// Process external request
    pub async fn process_request(&self, request: ExternalRequest) -> Result<ExternalResponse> {
        // HomeKit requests would typically come through HAP protocol
        // This is a simplified representation
        let response = serde_json::json!({
            "accessories": self.get_accessories()
        });

        Ok(ExternalResponse::success(response))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_homekit_adapter_creation() {
        let adapter = HomeKitAdapter::new(HomeKitConfig::default());
        assert_eq!(adapter.accessories.len(), 1); // Bridge only
    }

    #[test]
    fn test_register_light() {
        let mut adapter = HomeKitAdapter::new(HomeKitConfig::default());

        let device_id = Uuid::new_v4();
        let aid = adapter.register_light(device_id, "Living Room Light");

        assert_eq!(aid, 2);
        assert_eq!(adapter.accessories.len(), 2);
        assert!(adapter.device_mapping.contains_key(&aid));
    }

    #[test]
    fn test_register_lock() {
        let mut adapter = HomeKitAdapter::new(HomeKitConfig::default());

        let device_id = Uuid::new_v4();
        let aid = adapter.register_lock(device_id, "Front Door");

        assert_eq!(aid, 2);
        assert!(adapter.accessories.get(&aid).is_some());
    }

    #[tokio::test]
    async fn test_handle_write() {
        let mut adapter = HomeKitAdapter::new(HomeKitConfig::default());

        let device_id = Uuid::new_v4();
        let aid = adapter.register_light(device_id, "Test Light");

        let request = HomeKitWriteRequest {
            characteristics: vec![HomeKitCharWrite {
                aid,
                iid: 11, // ON characteristic
                value: serde_json::json!(true),
            }],
        };

        adapter.handle_write(request).await.unwrap();

        // Verify the value was written
        let accessory = adapter.accessories.get(&aid).unwrap();
        let on_char = accessory
            .services
            .iter()
            .flat_map(|s| &s.characteristics)
            .find(|c| c.iid == 11)
            .unwrap();

        assert_eq!(on_char.value, Some(serde_json::json!(true)));
    }

    #[test]
    fn test_write_to_command() {
        let mut adapter = HomeKitAdapter::new(HomeKitConfig::default());

        let device_id = Uuid::new_v4();
        let aid = adapter.register_light(device_id, "Test Light");

        let write = HomeKitCharWrite {
            aid,
            iid: 11,
            value: serde_json::json!(true),
        };

        let command = adapter.write_to_command(&write);
        assert!(command.is_some());

        let command = command.unwrap();
        assert!(matches!(command.action, DeviceAction::Power { on: true }));
    }
}
