//! Simulator Adapters for Testing
//! 弘益人間 - Gaming for Everyone

use super::*;
use crate::error::{GameError, Result};
use crate::types::*;
use async_trait::async_trait;
use std::collections::HashMap;
use std::sync::Mutex;

/// Simulated profile storage for testing
#[derive(Debug, Default)]
pub struct SimulatorProfileStorage {
    profiles: Mutex<HashMap<ProfileId, PlayerProfile>>,
}

impl SimulatorProfileStorage {
    pub fn new() -> Self {
        Self {
            profiles: Mutex::new(HashMap::new()),
        }
    }
}

#[async_trait]
impl ProfileStorage for SimulatorProfileStorage {
    async fn save_profile(&self, profile: &PlayerProfile) -> Result<()> {
        let mut profiles = self.profiles.lock().unwrap();
        profiles.insert(profile.profile_id, profile.clone());
        Ok(())
    }

    async fn load_profile(&self, profile_id: ProfileId) -> Result<PlayerProfile> {
        let profiles = self.profiles.lock().unwrap();
        profiles
            .get(&profile_id)
            .cloned()
            .ok_or(GameError::ProfileNotFound(profile_id))
    }

    async fn delete_profile(&self, profile_id: ProfileId) -> Result<()> {
        let mut profiles = self.profiles.lock().unwrap();
        profiles
            .remove(&profile_id)
            .ok_or(GameError::ProfileNotFound(profile_id))?;
        Ok(())
    }

    async fn list_profiles(&self) -> Result<Vec<ProfileId>> {
        let profiles = self.profiles.lock().unwrap();
        Ok(profiles.keys().copied().collect())
    }

    async fn exists(&self, profile_id: ProfileId) -> Result<bool> {
        let profiles = self.profiles.lock().unwrap();
        Ok(profiles.contains_key(&profile_id))
    }
}

/// Simulated game settings adapter for testing
#[derive(Debug)]
pub struct SimulatorGameAdapter {
    current_profile: Mutex<Option<PlayerProfile>>,
    supported_features: Vec<String>,
}

impl SimulatorGameAdapter {
    pub fn new() -> Self {
        Self {
            current_profile: Mutex::new(None),
            supported_features: vec![
                "screen_reader".to_string(),
                "colorblind_mode".to_string(),
                "high_contrast".to_string(),
                "subtitles".to_string(),
                "closed_captions".to_string(),
                "visual_sound_cues".to_string(),
                "button_remapping".to_string(),
                "aim_assist".to_string(),
                "one_handed_mode".to_string(),
                "difficulty_options".to_string(),
                "hint_system".to_string(),
            ],
        }
    }

    pub fn with_features(features: Vec<String>) -> Self {
        Self {
            current_profile: Mutex::new(None),
            supported_features: features,
        }
    }
}

impl Default for SimulatorGameAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl GameSettingsAdapter for SimulatorGameAdapter {
    async fn apply_settings(&self, profile: &PlayerProfile) -> Result<()> {
        let mut current = self.current_profile.lock().unwrap();
        *current = Some(profile.clone());
        Ok(())
    }

    async fn get_current_settings(&self) -> Result<PlayerProfile> {
        let current = self.current_profile.lock().unwrap();
        current
            .clone()
            .ok_or(GameError::InvalidSettings("No settings applied".to_string()))
    }

    async fn get_supported_features(&self) -> Result<Vec<String>> {
        Ok(self.supported_features.clone())
    }

    async fn reset_defaults(&self) -> Result<()> {
        let mut current = self.current_profile.lock().unwrap();
        *current = Some(PlayerProfile::default());
        Ok(())
    }
}

/// Simulated input device adapter for testing
#[derive(Debug)]
pub struct SimulatorInputAdapter {
    devices: Vec<ConnectedDevice>,
    configs: Mutex<HashMap<String, ControllerConfig>>,
}

impl SimulatorInputAdapter {
    pub fn new() -> Self {
        Self {
            devices: vec![
                ConnectedDevice {
                    device_id: "sim-controller-1".to_string(),
                    name: "Simulated Controller".to_string(),
                    device_type: InputDeviceType::StandardController,
                    connected: true,
                    battery_level: Some(75),
                    has_haptics: true,
                    has_gyro: true,
                },
                ConnectedDevice {
                    device_id: "sim-adaptive-1".to_string(),
                    name: "Simulated Xbox Adaptive Controller".to_string(),
                    device_type: InputDeviceType::XboxAdaptiveController,
                    connected: true,
                    battery_level: Some(100),
                    has_haptics: true,
                    has_gyro: false,
                },
            ],
            configs: Mutex::new(HashMap::new()),
        }
    }

    pub fn with_devices(devices: Vec<ConnectedDevice>) -> Self {
        Self {
            devices,
            configs: Mutex::new(HashMap::new()),
        }
    }
}

impl Default for SimulatorInputAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl InputDeviceAdapter for SimulatorInputAdapter {
    async fn get_devices(&self) -> Result<Vec<ConnectedDevice>> {
        Ok(self.devices.clone())
    }

    async fn apply_config(&self, config: &ControllerConfig) -> Result<()> {
        let mut configs = self.configs.lock().unwrap();
        let device_id = config.config_id.map(|id| id.to_string()).unwrap_or_default();
        configs.insert(device_id, config.clone());
        Ok(())
    }

    async fn get_config(&self, device_id: &str) -> Result<ControllerConfig> {
        let configs = self.configs.lock().unwrap();
        configs
            .get(device_id)
            .cloned()
            .ok_or(GameError::ControllerConfigNotFound(uuid::Uuid::nil()))
    }

    async fn test_haptics(&self, device_id: &str, intensity: f32) -> Result<()> {
        // Check if device exists and has haptics
        let device = self
            .devices
            .iter()
            .find(|d| d.device_id == device_id)
            .ok_or_else(|| GameError::DeviceNotConnected(device_id.to_string()))?;

        if !device.has_haptics {
            return Err(GameError::FeatureNotSupported(
                "Haptics not supported on this device".to_string(),
            ));
        }

        if intensity < 0.0 || intensity > 1.0 {
            return Err(GameError::ValidationError(
                "Haptic intensity must be between 0.0 and 1.0".to_string(),
            ));
        }

        // Simulate haptic feedback
        println!("🎮 Simulated haptic feedback: device={}, intensity={:.2}", device_id, intensity);

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_profile_storage() {
        let storage = SimulatorProfileStorage::new();
        let profile = PlayerProfile::default();
        let id = profile.profile_id;

        storage.save_profile(&profile).await.unwrap();
        assert!(storage.exists(id).await.unwrap());

        let loaded = storage.load_profile(id).await.unwrap();
        assert_eq!(loaded.profile_id, id);

        storage.delete_profile(id).await.unwrap();
        assert!(!storage.exists(id).await.unwrap());
    }

    #[tokio::test]
    async fn test_game_adapter() {
        let adapter = SimulatorGameAdapter::new();
        let profile = PlayerProfile::default();

        adapter.apply_settings(&profile).await.unwrap();
        let current = adapter.get_current_settings().await.unwrap();
        assert_eq!(current.profile_id, profile.profile_id);

        let features = adapter.get_supported_features().await.unwrap();
        assert!(!features.is_empty());
    }

    #[tokio::test]
    async fn test_input_adapter() {
        let adapter = SimulatorInputAdapter::new();

        let devices = adapter.get_devices().await.unwrap();
        assert!(!devices.is_empty());

        let result = adapter.test_haptics("sim-controller-1", 0.5).await;
        assert!(result.is_ok());
    }
}
