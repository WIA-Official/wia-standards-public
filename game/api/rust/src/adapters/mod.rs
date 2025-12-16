//! WIA Game Adapters Module
//! 弘益人間 - Gaming for Everyone

mod simulator;
mod storage;

pub use simulator::*;
pub use storage::*;

use crate::error::Result;
use crate::types::*;
use async_trait::async_trait;

/// Profile storage adapter trait
#[async_trait]
pub trait ProfileStorage: Send + Sync + std::fmt::Debug {
    /// Save a profile
    async fn save_profile(&self, profile: &PlayerProfile) -> Result<()>;

    /// Load a profile by ID
    async fn load_profile(&self, profile_id: ProfileId) -> Result<PlayerProfile>;

    /// Delete a profile
    async fn delete_profile(&self, profile_id: ProfileId) -> Result<()>;

    /// List all profile IDs
    async fn list_profiles(&self) -> Result<Vec<ProfileId>>;

    /// Check if profile exists
    async fn exists(&self, profile_id: ProfileId) -> Result<bool>;
}

/// Game settings adapter trait
#[async_trait]
pub trait GameSettingsAdapter: Send + Sync {
    /// Apply profile settings to the game
    async fn apply_settings(&self, profile: &PlayerProfile) -> Result<()>;

    /// Get current game settings as a profile
    async fn get_current_settings(&self) -> Result<PlayerProfile>;

    /// Check which features are supported
    async fn get_supported_features(&self) -> Result<Vec<String>>;

    /// Reset to default settings
    async fn reset_defaults(&self) -> Result<()>;
}

/// Input device adapter trait
#[async_trait]
pub trait InputDeviceAdapter: Send + Sync {
    /// Get connected devices
    async fn get_devices(&self) -> Result<Vec<ConnectedDevice>>;

    /// Apply controller config
    async fn apply_config(&self, config: &ControllerConfig) -> Result<()>;

    /// Get current config for a device
    async fn get_config(&self, device_id: &str) -> Result<ControllerConfig>;

    /// Test vibration/haptics
    async fn test_haptics(&self, device_id: &str, intensity: f32) -> Result<()>;
}

/// Connected input device info
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct ConnectedDevice {
    pub device_id: String,
    pub name: String,
    pub device_type: InputDeviceType,
    pub connected: bool,
    pub battery_level: Option<u8>,
    pub has_haptics: bool,
    pub has_gyro: bool,
}
