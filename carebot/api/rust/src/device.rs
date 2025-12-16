//! CareBot device specification and management

use serde::{Deserialize, Serialize};
use crate::types::Timestamp;

/// CareBot device specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CareBotDevice {
    /// Unique device identifier
    pub device_id: String,
    /// Device model name
    pub model: String,
    /// Manufacturer name
    pub manufacturer: String,
    /// Firmware version
    pub firmware_version: String,
    /// Device capabilities
    pub capabilities: DeviceCapabilities,
    /// Sensor configuration
    pub sensors: DeviceSensors,
    /// AI models loaded
    pub ai_models: AiModels,
    /// Network configuration
    pub network: NetworkConfig,
    /// Battery status
    pub battery: BatteryStatus,
    /// Last update timestamp
    pub last_updated: Timestamp,
}

impl CareBotDevice {
    /// Create a new CareBot device
    pub fn new(device_id: &str, model: &str, manufacturer: &str) -> Self {
        Self {
            device_id: device_id.to_string(),
            model: model.to_string(),
            manufacturer: manufacturer.to_string(),
            firmware_version: "1.0.0".to_string(),
            capabilities: DeviceCapabilities::default(),
            sensors: DeviceSensors::default(),
            ai_models: AiModels::default(),
            network: NetworkConfig::default(),
            battery: BatteryStatus::default(),
            last_updated: Timestamp::now(),
        }
    }

    /// Set firmware version
    pub fn with_firmware(mut self, version: &str) -> Self {
        self.firmware_version = version.to_string();
        self
    }

    /// Check if device supports a specific capability
    pub fn supports_emotion_detection(&self) -> bool {
        self.capabilities.emotion_detection
    }

    /// Check if device supports fall detection
    pub fn supports_fall_detection(&self) -> bool {
        self.capabilities.fall_detection
    }

    /// Check if device supports video calls
    pub fn supports_video_call(&self) -> bool {
        self.capabilities.video_call
    }
}

/// Device capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceCapabilities {
    /// AI conversation capability
    pub ai_conversation: bool,
    /// Emotion detection capability
    pub emotion_detection: bool,
    /// Fall detection capability
    pub fall_detection: bool,
    /// Medication reminder capability
    pub medication_reminder: bool,
    /// Video call capability
    pub video_call: bool,
    /// Vital sign monitoring
    pub vital_monitoring: bool,
    /// Autonomous navigation
    pub autonomous_navigation: bool,
    /// Emergency response
    pub emergency_response: bool,
    /// Cognitive games
    pub cognitive_games: bool,
}

impl Default for DeviceCapabilities {
    fn default() -> Self {
        Self {
            ai_conversation: true,
            emotion_detection: true,
            fall_detection: true,
            medication_reminder: true,
            video_call: true,
            vital_monitoring: true,
            autonomous_navigation: true,
            emergency_response: true,
            cognitive_games: true,
        }
    }
}

/// Device sensors
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceSensors {
    /// RGB camera available
    pub camera_rgb: bool,
    /// Depth camera (ToF) available
    pub camera_depth: bool,
    /// Thermal camera available
    pub camera_thermal: bool,
    /// Microphone array
    pub microphone_array: bool,
    /// LIDAR sensor
    pub lidar: bool,
    /// IMU sensor
    pub imu: bool,
    /// Proximity sensors
    pub proximity: bool,
    /// Touch sensors
    pub touch: bool,
}

impl Default for DeviceSensors {
    fn default() -> Self {
        Self {
            camera_rgb: true,
            camera_depth: true,
            camera_thermal: false,
            microphone_array: true,
            lidar: true,
            imu: true,
            proximity: true,
            touch: true,
        }
    }
}

/// AI models configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AiModels {
    /// Emotion recognition model
    pub emotion_recognition: Option<ModelInfo>,
    /// Speech recognition model
    pub speech_recognition: Option<ModelInfo>,
    /// Natural language understanding
    pub nlu: Option<ModelInfo>,
    /// Fall detection model
    pub fall_detection: Option<ModelInfo>,
    /// Activity recognition model
    pub activity_recognition: Option<ModelInfo>,
}

impl Default for AiModels {
    fn default() -> Self {
        Self {
            emotion_recognition: Some(ModelInfo::new("emotion-v2", "2.1.0")),
            speech_recognition: Some(ModelInfo::new("speech-kr", "1.5.0")),
            nlu: Some(ModelInfo::new("nlu-care", "3.0.0")),
            fall_detection: Some(ModelInfo::new("fall-detect", "1.2.0")),
            activity_recognition: Some(ModelInfo::new("activity-v1", "1.0.0")),
        }
    }
}

/// AI model information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ModelInfo {
    /// Model name
    pub name: String,
    /// Model version
    pub version: String,
    /// Model accuracy (if available)
    pub accuracy: Option<f64>,
}

impl ModelInfo {
    /// Create new model info
    pub fn new(name: &str, version: &str) -> Self {
        Self {
            name: name.to_string(),
            version: version.to_string(),
            accuracy: None,
        }
    }
}

/// Network configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NetworkConfig {
    /// WiFi connected
    pub wifi_connected: bool,
    /// LTE connected
    pub lte_connected: bool,
    /// Cloud connected
    pub cloud_connected: bool,
    /// Local server connected
    pub local_server_connected: bool,
}

impl Default for NetworkConfig {
    fn default() -> Self {
        Self {
            wifi_connected: true,
            lte_connected: false,
            cloud_connected: true,
            local_server_connected: true,
        }
    }
}

/// Battery status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BatteryStatus {
    /// Battery level (0-100)
    pub level: u8,
    /// Currently charging
    pub charging: bool,
    /// Estimated remaining hours
    pub estimated_hours: Option<f32>,
}

impl Default for BatteryStatus {
    fn default() -> Self {
        Self {
            level: 100,
            charging: false,
            estimated_hours: Some(8.0),
        }
    }
}
