//! Type definitions for WIA-HOME standard
//!
//! 弘益人間 - Benefit All Humanity through smart home technology

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};
use uuid::Uuid;

/// Device type enumeration
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum DeviceType {
    Light,
    Thermostat,
    Lock,
    Camera,
    Sensor,
    Switch,
    Outlet,
    Speaker,
    Display,
    Appliance,
    Custom(String),
}

/// Device status
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum DeviceStatus {
    Online,
    Offline,
    Maintenance,
    Error,
}

/// Device information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Device {
    pub id: Uuid,
    pub name: String,
    pub device_type: DeviceType,
    pub status: DeviceStatus,
    pub room: Option<String>,
    pub capabilities: Vec<String>,
    pub metadata: serde_json::Value,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
}

/// Room configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Room {
    pub id: Uuid,
    pub name: String,
    pub floor: Option<i32>,
    pub devices: Vec<Uuid>,
    pub metadata: serde_json::Value,
}

/// Scene configuration for automation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Scene {
    pub id: Uuid,
    pub name: String,
    pub description: Option<String>,
    pub actions: Vec<SceneAction>,
    pub triggers: Vec<SceneTrigger>,
    pub enabled: bool,
    pub created_at: DateTime<Utc>,
}

/// Scene action
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SceneAction {
    pub device_id: Uuid,
    pub action: String,
    pub parameters: serde_json::Value,
    pub delay_seconds: Option<u32>,
}

/// Scene trigger
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SceneTrigger {
    pub trigger_type: TriggerType,
    pub condition: serde_json::Value,
}

/// Trigger type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum TriggerType {
    Schedule,
    DeviceState,
    Sensor,
    Manual,
    Geofence,
}

/// Energy consumption data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnergyData {
    pub device_id: Uuid,
    pub timestamp: DateTime<Utc>,
    pub consumption_kwh: f64,
    pub cost: Option<f64>,
    pub currency: Option<String>,
}

/// Home configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HomeConfig {
    pub home_id: Uuid,
    pub name: String,
    pub address: Option<String>,
    pub timezone: String,
    pub rooms: Vec<Room>,
    pub metadata: serde_json::Value,
}

/// API Response wrapper
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApiResponse<T> {
    pub success: bool,
    pub data: Option<T>,
    pub error: Option<String>,
    pub timestamp: DateTime<Utc>,
}
