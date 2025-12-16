//! Smart home integration (Matter, HomeKit)

use serde::{Deserialize, Serialize};

/// Matter device descriptor
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MatterDevice {
    /// Device type (0x0022 for CareBot)
    pub device_type: String,
    /// Device name
    pub device_name: String,
    /// Vendor ID
    pub vendor_id: String,
    /// Product ID
    pub product_id: String,
    /// Supported clusters
    pub clusters: MatterClusters,
}

impl Default for MatterDevice {
    fn default() -> Self {
        Self {
            device_type: "0x0022".to_string(),
            device_name: "WIA CareBot".to_string(),
            vendor_id: "0x1234".to_string(),
            product_id: "0x0001".to_string(),
            clusters: MatterClusters::default(),
        }
    }
}

/// Matter clusters supported by CareBot
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MatterClusters {
    /// Basic information cluster
    pub basic_information: BasicInformationCluster,
    /// On/Off cluster support
    pub on_off: bool,
    /// Care status custom cluster
    pub care_status: CareStatusCluster,
}

impl Default for MatterClusters {
    fn default() -> Self {
        Self {
            basic_information: BasicInformationCluster::default(),
            on_off: true,
            care_status: CareStatusCluster::default(),
        }
    }
}

/// Basic information cluster
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BasicInformationCluster {
    pub cluster_id: String,
    pub vendor_name: String,
    pub product_name: String,
    pub serial_number: String,
    pub software_version: String,
}

impl Default for BasicInformationCluster {
    fn default() -> Self {
        Self {
            cluster_id: "0x0028".to_string(),
            vendor_name: "WIA".to_string(),
            product_name: "CareBot".to_string(),
            serial_number: "CB-001-2024".to_string(),
            software_version: "1.0.0".to_string(),
        }
    }
}

/// Custom care status cluster
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CareStatusCluster {
    pub cluster_id: String,
    pub manufacturer_specific: bool,
    pub attributes: CareStatusAttributes,
}

impl Default for CareStatusCluster {
    fn default() -> Self {
        Self {
            cluster_id: "0xFC00".to_string(),
            manufacturer_specific: true,
            attributes: CareStatusAttributes::default(),
        }
    }
}

/// Care status attributes
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CareStatusAttributes {
    /// Current recipient status
    pub recipient_status: RecipientMatterStatus,
    /// Last interaction timestamp
    pub last_interaction_time: Option<String>,
    /// Activity level (0-100)
    pub activity_level: u8,
    /// Current emotion state
    pub emotion_state: Option<String>,
}

/// Recipient status for Matter
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RecipientMatterStatus {
    #[default]
    Normal,
    AttentionNeeded,
    Emergency,
}

/// Smart home automation trigger
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutomationTrigger {
    /// Trigger name
    pub name: String,
    /// Device that triggers
    pub device: String,
    /// Event type
    pub event: SmartHomeEvent,
    /// Conditions (optional)
    pub conditions: Vec<AutomationCondition>,
    /// Actions to perform
    pub actions: Vec<AutomationAction>,
}

/// Smart home events
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SmartHomeEvent {
    FallDetected,
    SosPressed,
    WakeTime,
    SleepTime,
    MedicationTime,
    ActivityLow,
    EmotionConcern,
    RecipientLeftZone,
}

/// Automation condition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutomationCondition {
    pub condition_type: ConditionType,
    pub value: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConditionType {
    TimeRange,
    Location,
    DeviceState,
    WeatherCondition,
}

/// Automation action
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutomationAction {
    /// Service to call
    pub service: SmartHomeService,
    /// Target device/entity
    pub target: String,
    /// Parameters
    pub params: serde_json::Value,
}

/// Smart home services
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SmartHomeService {
    LightTurnOn,
    LightTurnOff,
    LightDim,
    LockUnlock,
    LockLock,
    CameraRecord,
    MediaPlayerPlay,
    MediaPlayerStop,
    ClimateSetTemp,
    NotifySend,
    CarebotSpeak,
}

/// Pre-defined automation for fall detection
pub fn fall_detection_automation() -> AutomationTrigger {
    AutomationTrigger {
        name: "fall_detected".to_string(),
        device: "carebot".to_string(),
        event: SmartHomeEvent::FallDetected,
        conditions: Vec::new(),
        actions: vec![
            AutomationAction {
                service: SmartHomeService::LightTurnOn,
                target: "all_lights".to_string(),
                params: serde_json::json!({"brightness": 100}),
            },
            AutomationAction {
                service: SmartHomeService::LockUnlock,
                target: "front_door".to_string(),
                params: serde_json::json!({}),
            },
            AutomationAction {
                service: SmartHomeService::CameraRecord,
                target: "living_room_camera".to_string(),
                params: serde_json::json!({"duration": 300}),
            },
            AutomationAction {
                service: SmartHomeService::NotifySend,
                target: "emergency_contacts".to_string(),
                params: serde_json::json!({"message": "낙상이 감지되었습니다"}),
            },
        ],
    }
}

/// Pre-defined automation for bedtime
pub fn bedtime_automation() -> AutomationTrigger {
    AutomationTrigger {
        name: "bedtime_routine".to_string(),
        device: "carebot".to_string(),
        event: SmartHomeEvent::SleepTime,
        conditions: vec![AutomationCondition {
            condition_type: ConditionType::Location,
            value: "bedroom".to_string(),
        }],
        actions: vec![
            AutomationAction {
                service: SmartHomeService::LightDim,
                target: "bedroom_light".to_string(),
                params: serde_json::json!({"brightness": 20, "transition": 60}),
            },
            AutomationAction {
                service: SmartHomeService::MediaPlayerStop,
                target: "living_room_tv".to_string(),
                params: serde_json::json!({}),
            },
            AutomationAction {
                service: SmartHomeService::ClimateSetTemp,
                target: "bedroom_ac".to_string(),
                params: serde_json::json!({"temperature": 24}),
            },
        ],
    }
}

/// Pre-defined automation for wake up
pub fn wakeup_automation() -> AutomationTrigger {
    AutomationTrigger {
        name: "wake_up_routine".to_string(),
        device: "carebot".to_string(),
        event: SmartHomeEvent::WakeTime,
        conditions: Vec::new(),
        actions: vec![
            AutomationAction {
                service: SmartHomeService::LightTurnOn,
                target: "bedroom_light".to_string(),
                params: serde_json::json!({"brightness": 50, "transition": 300}),
            },
            AutomationAction {
                service: SmartHomeService::CarebotSpeak,
                target: "carebot".to_string(),
                params: serde_json::json!({
                    "message": "좋은 아침이에요, 오늘도 건강한 하루 되세요"
                }),
            },
        ],
    }
}

/// HomeKit accessory definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HomeKitAccessory {
    /// Accessory category
    pub category: String,
    /// Accessory name
    pub name: String,
    /// Services
    pub services: Vec<HomeKitService>,
}

impl Default for HomeKitAccessory {
    fn default() -> Self {
        Self {
            category: "other".to_string(),
            name: "WIA CareBot".to_string(),
            services: vec![
                HomeKitService::accessory_information(),
                HomeKitService::occupancy_sensor("돌봄 대상 감지"),
                HomeKitService::motion_sensor("활동 감지"),
                HomeKitService::contact_sensor("안전 상태"),
            ],
        }
    }
}

/// HomeKit service
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HomeKitService {
    /// Service type
    #[serde(rename = "type")]
    pub service_type: String,
    /// Service name
    pub name: Option<String>,
    /// Characteristics
    pub characteristics: serde_json::Value,
}

impl HomeKitService {
    /// Create accessory information service
    pub fn accessory_information() -> Self {
        Self {
            service_type: "AccessoryInformation".to_string(),
            name: None,
            characteristics: serde_json::json!({
                "Manufacturer": "WIA",
                "Model": "CareBot-1",
                "SerialNumber": "CB-001"
            }),
        }
    }

    /// Create occupancy sensor service
    pub fn occupancy_sensor(name: &str) -> Self {
        Self {
            service_type: "OccupancySensor".to_string(),
            name: Some(name.to_string()),
            characteristics: serde_json::json!({
                "OccupancyDetected": false
            }),
        }
    }

    /// Create motion sensor service
    pub fn motion_sensor(name: &str) -> Self {
        Self {
            service_type: "MotionSensor".to_string(),
            name: Some(name.to_string()),
            characteristics: serde_json::json!({
                "MotionDetected": false
            }),
        }
    }

    /// Create contact sensor service
    pub fn contact_sensor(name: &str) -> Self {
        Self {
            service_type: "ContactSensor".to_string(),
            name: Some(name.to_string()),
            characteristics: serde_json::json!({
                "ContactSensorState": 0
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_matter_device_default() {
        let device = MatterDevice::default();
        assert_eq!(device.device_name, "WIA CareBot");
        assert!(device.clusters.on_off);
    }

    #[test]
    fn test_fall_detection_automation() {
        let automation = fall_detection_automation();
        assert_eq!(automation.event, SmartHomeEvent::FallDetected);
        assert_eq!(automation.actions.len(), 4);
    }

    #[test]
    fn test_homekit_accessory() {
        let accessory = HomeKitAccessory::default();
        assert_eq!(accessory.name, "WIA CareBot");
        assert_eq!(accessory.services.len(), 4);
    }
}
