//! WIA Smart Home Core Implementation
//! 弘益人間 - Benefit All Humanity

use crate::error::{Result, SmartHomeError};
use crate::types::*;
use async_trait::async_trait;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use uuid::Uuid;

// ============================================================================
// Device Adapter Trait
// ============================================================================

/// Device adapter trait for communicating with devices
#[async_trait]
pub trait DeviceAdapter: Send + Sync {
    /// Get device status
    async fn get_status(&self, device_id: Uuid) -> Result<DeviceStatus>;

    /// Send command to device
    async fn send_command(
        &self,
        device_id: Uuid,
        command: &str,
        params: &HashMap<String, serde_json::Value>,
    ) -> Result<()>;

    /// Get device state
    async fn get_state(&self, device_id: Uuid) -> Result<HashMap<String, serde_json::Value>>;

    /// Set device state
    async fn set_state(
        &self,
        device_id: Uuid,
        state: HashMap<String, serde_json::Value>,
    ) -> Result<()>;
}

// ============================================================================
// Notification Service Trait
// ============================================================================

/// Notification service trait
#[async_trait]
pub trait NotificationService: Send + Sync {
    /// Send notification
    async fn send(&self, notification: &Notification) -> Result<()>;

    /// Send TTS announcement
    async fn announce(&self, text: &str, zone_ids: &[Uuid]) -> Result<()>;

    /// Send haptic feedback
    async fn haptic(&self, device_ids: &[Uuid], pattern: HapticPattern) -> Result<()>;
}

// ============================================================================
// Smart Home Controller
// ============================================================================

/// Smart Home Controller - Main API entry point
pub struct SmartHomeController {
    homes: Arc<RwLock<HashMap<Uuid, Home>>>,
    zones: Arc<RwLock<HashMap<Uuid, Zone>>>,
    devices: Arc<RwLock<HashMap<Uuid, Device>>>,
    profiles: Arc<RwLock<HashMap<Uuid, UserProfile>>>,
    automations: Arc<RwLock<HashMap<Uuid, Automation>>>,
    device_adapter: Option<Arc<dyn DeviceAdapter>>,
    notification_service: Option<Arc<dyn NotificationService>>,
}

impl SmartHomeController {
    /// Create new SmartHomeController
    pub fn new() -> Self {
        Self {
            homes: Arc::new(RwLock::new(HashMap::new())),
            zones: Arc::new(RwLock::new(HashMap::new())),
            devices: Arc::new(RwLock::new(HashMap::new())),
            profiles: Arc::new(RwLock::new(HashMap::new())),
            automations: Arc::new(RwLock::new(HashMap::new())),
            device_adapter: None,
            notification_service: None,
        }
    }

    /// Set device adapter
    pub fn with_device_adapter(mut self, adapter: Arc<dyn DeviceAdapter>) -> Self {
        self.device_adapter = Some(adapter);
        self
    }

    /// Set notification service
    pub fn with_notification_service(mut self, service: Arc<dyn NotificationService>) -> Self {
        self.notification_service = Some(service);
        self
    }

    // ========================================================================
    // Home Management
    // ========================================================================

    /// Create a new home
    pub async fn create_home(&self, name: String, owner_profile_id: Uuid) -> Result<Home> {
        let home = Home::new(Uuid::new_v4(), name, owner_profile_id);
        self.homes.write().await.insert(home.home_id, home.clone());
        Ok(home)
    }

    /// Get home by ID
    pub async fn get_home(&self, home_id: Uuid) -> Result<Home> {
        self.homes
            .read()
            .await
            .get(&home_id)
            .cloned()
            .ok_or(SmartHomeError::HomeNotFound(home_id))
    }

    /// Update home
    pub async fn update_home(&self, home: Home) -> Result<()> {
        let mut homes = self.homes.write().await;
        if !homes.contains_key(&home.home_id) {
            return Err(SmartHomeError::HomeNotFound(home.home_id));
        }
        homes.insert(home.home_id, home);
        Ok(())
    }

    /// Delete home
    pub async fn delete_home(&self, home_id: Uuid) -> Result<()> {
        self.homes
            .write()
            .await
            .remove(&home_id)
            .ok_or(SmartHomeError::HomeNotFound(home_id))?;
        Ok(())
    }

    /// List all homes
    pub async fn list_homes(&self) -> Vec<Home> {
        self.homes.read().await.values().cloned().collect()
    }

    // ========================================================================
    // Zone Management
    // ========================================================================

    /// Create a new zone
    pub async fn create_zone(
        &self,
        home_id: Uuid,
        name: String,
        zone_type: ZoneType,
    ) -> Result<Zone> {
        // Verify home exists
        if !self.homes.read().await.contains_key(&home_id) {
            return Err(SmartHomeError::HomeNotFound(home_id));
        }

        let zone = Zone::new(Uuid::new_v4(), home_id, name, zone_type);
        self.zones.write().await.insert(zone.zone_id, zone.clone());

        // Add zone to home
        if let Some(home) = self.homes.write().await.get_mut(&home_id) {
            home.zones.push(zone.zone_id);
        }

        Ok(zone)
    }

    /// Get zone by ID
    pub async fn get_zone(&self, zone_id: Uuid) -> Result<Zone> {
        self.zones
            .read()
            .await
            .get(&zone_id)
            .cloned()
            .ok_or(SmartHomeError::ZoneNotFound(zone_id))
    }

    /// Update zone
    pub async fn update_zone(&self, zone: Zone) -> Result<()> {
        let mut zones = self.zones.write().await;
        if !zones.contains_key(&zone.zone_id) {
            return Err(SmartHomeError::ZoneNotFound(zone.zone_id));
        }
        zones.insert(zone.zone_id, zone);
        Ok(())
    }

    /// Delete zone
    pub async fn delete_zone(&self, zone_id: Uuid) -> Result<()> {
        let zone = self
            .zones
            .write()
            .await
            .remove(&zone_id)
            .ok_or(SmartHomeError::ZoneNotFound(zone_id))?;

        // Remove zone from home
        if let Some(home) = self.homes.write().await.get_mut(&zone.home_id) {
            home.zones.retain(|id| *id != zone_id);
        }

        Ok(())
    }

    /// List zones in a home
    pub async fn list_zones(&self, home_id: Uuid) -> Vec<Zone> {
        self.zones
            .read()
            .await
            .values()
            .filter(|z| z.home_id == home_id)
            .cloned()
            .collect()
    }

    // ========================================================================
    // Device Management
    // ========================================================================

    /// Register a new device
    pub async fn register_device(
        &self,
        device_type: DeviceType,
        name: String,
        home_id: Uuid,
        zone_id: Option<Uuid>,
    ) -> Result<Device> {
        // Verify home exists
        if !self.homes.read().await.contains_key(&home_id) {
            return Err(SmartHomeError::HomeNotFound(home_id));
        }

        // Verify zone exists if provided
        if let Some(zid) = zone_id {
            if !self.zones.read().await.contains_key(&zid) {
                return Err(SmartHomeError::ZoneNotFound(zid));
            }
        }

        let mut device = Device::new(Uuid::new_v4(), device_type, name);
        device.home_id = Some(home_id);
        device.zone_id = zone_id;

        self.devices
            .write()
            .await
            .insert(device.device_id, device.clone());

        // Add device to home
        if let Some(home) = self.homes.write().await.get_mut(&home_id) {
            home.devices.push(device.device_id);
        }

        // Add device to zone
        if let Some(zid) = zone_id {
            if let Some(zone) = self.zones.write().await.get_mut(&zid) {
                zone.devices.push(device.device_id);
            }
        }

        Ok(device)
    }

    /// Get device by ID
    pub async fn get_device(&self, device_id: Uuid) -> Result<Device> {
        self.devices
            .read()
            .await
            .get(&device_id)
            .cloned()
            .ok_or(SmartHomeError::DeviceNotFound(device_id))
    }

    /// Update device
    pub async fn update_device(&self, device: Device) -> Result<()> {
        let mut devices = self.devices.write().await;
        if !devices.contains_key(&device.device_id) {
            return Err(SmartHomeError::DeviceNotFound(device.device_id));
        }
        devices.insert(device.device_id, device);
        Ok(())
    }

    /// Unregister device
    pub async fn unregister_device(&self, device_id: Uuid) -> Result<()> {
        let device = self
            .devices
            .write()
            .await
            .remove(&device_id)
            .ok_or(SmartHomeError::DeviceNotFound(device_id))?;

        // Remove device from home
        if let Some(home_id) = device.home_id {
            if let Some(home) = self.homes.write().await.get_mut(&home_id) {
                home.devices.retain(|id| *id != device_id);
            }
        }

        // Remove device from zone
        if let Some(zone_id) = device.zone_id {
            if let Some(zone) = self.zones.write().await.get_mut(&zone_id) {
                zone.devices.retain(|id| *id != device_id);
            }
        }

        Ok(())
    }

    /// List devices in a home
    pub async fn list_devices(&self, home_id: Uuid) -> Vec<Device> {
        self.devices
            .read()
            .await
            .values()
            .filter(|d| d.home_id == Some(home_id))
            .cloned()
            .collect()
    }

    /// List devices in a zone
    pub async fn list_devices_in_zone(&self, zone_id: Uuid) -> Vec<Device> {
        self.devices
            .read()
            .await
            .values()
            .filter(|d| d.zone_id == Some(zone_id))
            .cloned()
            .collect()
    }

    // ========================================================================
    // Device Control (Accessibility-Aware)
    // ========================================================================

    /// Control device with accessibility context
    pub async fn control_device(
        &self,
        device_id: Uuid,
        command: &str,
        params: HashMap<String, serde_json::Value>,
        user_profile: Option<&UserProfile>,
    ) -> Result<()> {
        let device = self.get_device(device_id).await?;

        // Check device status
        if device.status == DeviceStatus::Offline {
            return Err(SmartHomeError::DeviceOffline(device_id));
        }

        // Get adapter
        let adapter = self
            .device_adapter
            .as_ref()
            .ok_or_else(|| SmartHomeError::Internal("No device adapter configured".to_string()))?;

        // Execute command
        adapter.send_command(device_id, command, &params).await?;

        // Provide accessibility feedback
        if let Some(profile) = user_profile {
            self.provide_feedback(&device, command, profile).await?;
        }

        Ok(())
    }

    /// Provide accessibility feedback after command execution
    async fn provide_feedback(
        &self,
        device: &Device,
        command: &str,
        profile: &UserProfile,
    ) -> Result<()> {
        let service = match &self.notification_service {
            Some(s) => s,
            None => return Ok(()),
        };

        // Check user's preferred output modalities
        let preferred_outputs = &profile.interaction_preferences.preferred_output_modalities;

        // Find confirmation phrase
        let confirmation = device
            .accessibility_features
            .voice_commands
            .iter()
            .find(|vc| vc.action == command)
            .and_then(|vc| vc.confirmation_phrase.clone());

        if let Some(phrase) = confirmation {
            // Send TTS if user prefers audio
            if preferred_outputs.contains(&OutputModality::AudioTts) {
                service.announce(&phrase, &[]).await?;
            }

            // Send haptic if user prefers haptic
            if preferred_outputs.contains(&OutputModality::Haptic) {
                service.haptic(&[device.device_id], HapticPattern::ShortTap).await?;
            }
        }

        Ok(())
    }

    /// Execute voice command
    pub async fn execute_voice_command(
        &self,
        device_id: Uuid,
        command_text: &str,
        user_profile: &UserProfile,
    ) -> Result<()> {
        let device = self.get_device(device_id).await?;

        // Find matching voice command
        let voice_cmd = device
            .accessibility_features
            .voice_commands
            .iter()
            .find(|vc| {
                vc.command.to_lowercase() == command_text.to_lowercase()
                    || vc.aliases.iter().any(|a| a.to_lowercase() == command_text.to_lowercase())
            })
            .ok_or_else(|| {
                SmartHomeError::CommandFailed(format!(
                    "Voice command not found: {}",
                    command_text
                ))
            })?;

        // Execute the action
        self.control_device(
            device_id,
            &voice_cmd.action,
            voice_cmd.parameters.clone(),
            Some(user_profile),
        )
        .await
    }

    // ========================================================================
    // User Profile Management
    // ========================================================================

    /// Create user profile
    pub async fn create_profile(&self) -> Result<UserProfile> {
        let profile = UserProfile::new(Uuid::new_v4());
        self.profiles
            .write()
            .await
            .insert(profile.profile_id, profile.clone());
        Ok(profile)
    }

    /// Get user profile
    pub async fn get_profile(&self, profile_id: Uuid) -> Result<UserProfile> {
        self.profiles
            .read()
            .await
            .get(&profile_id)
            .cloned()
            .ok_or(SmartHomeError::ProfileNotFound(profile_id))
    }

    /// Update user profile
    pub async fn update_profile(&self, profile: UserProfile) -> Result<()> {
        let mut profiles = self.profiles.write().await;
        if !profiles.contains_key(&profile.profile_id) {
            return Err(SmartHomeError::ProfileNotFound(profile.profile_id));
        }
        profiles.insert(profile.profile_id, profile);
        Ok(())
    }

    /// Delete user profile
    pub async fn delete_profile(&self, profile_id: Uuid) -> Result<()> {
        self.profiles
            .write()
            .await
            .remove(&profile_id)
            .ok_or(SmartHomeError::ProfileNotFound(profile_id))?;
        Ok(())
    }

    // ========================================================================
    // Automation Management
    // ========================================================================

    /// Create automation
    pub async fn create_automation(
        &self,
        name: String,
        home_id: Uuid,
        trigger: Trigger,
        actions: Vec<Action>,
    ) -> Result<Automation> {
        // Verify home exists
        if !self.homes.read().await.contains_key(&home_id) {
            return Err(SmartHomeError::HomeNotFound(home_id));
        }

        let automation = Automation::new(Uuid::new_v4(), name, home_id, trigger, actions);
        self.automations
            .write()
            .await
            .insert(automation.automation_id, automation.clone());
        Ok(automation)
    }

    /// Get automation
    pub async fn get_automation(&self, automation_id: Uuid) -> Result<Automation> {
        self.automations
            .read()
            .await
            .get(&automation_id)
            .cloned()
            .ok_or(SmartHomeError::AutomationNotFound(automation_id))
    }

    /// Update automation
    pub async fn update_automation(&self, automation: Automation) -> Result<()> {
        let mut automations = self.automations.write().await;
        if !automations.contains_key(&automation.automation_id) {
            return Err(SmartHomeError::AutomationNotFound(automation.automation_id));
        }
        automations.insert(automation.automation_id, automation);
        Ok(())
    }

    /// Delete automation
    pub async fn delete_automation(&self, automation_id: Uuid) -> Result<()> {
        self.automations
            .write()
            .await
            .remove(&automation_id)
            .ok_or(SmartHomeError::AutomationNotFound(automation_id))?;
        Ok(())
    }

    /// List automations in a home
    pub async fn list_automations(&self, home_id: Uuid) -> Vec<Automation> {
        self.automations
            .read()
            .await
            .values()
            .filter(|a| a.home_id == home_id)
            .cloned()
            .collect()
    }

    /// Execute automation (manual trigger)
    pub async fn execute_automation(
        &self,
        automation_id: Uuid,
        user_profile: Option<&UserProfile>,
    ) -> Result<()> {
        let automation = self.get_automation(automation_id).await?;

        if !automation.enabled {
            return Err(SmartHomeError::AutomationError(
                "Automation is disabled".to_string(),
            ));
        }

        // Check conditions
        // (simplified - real implementation would evaluate conditions)

        // Announce if configured
        if automation.accessibility_settings.announce_activation {
            if let Some(service) = &self.notification_service {
                let default_text = format!("Executing automation: {}", automation.name);
                let text = automation
                    .accessibility_settings
                    .announcement_text
                    .as_deref()
                    .unwrap_or(&default_text);
                service.announce(text, &automation.zone_ids).await?;
            }
        }

        // Execute actions
        for action in &automation.actions {
            self.execute_action(action, user_profile).await?;
        }

        // Update trigger count
        let mut automations = self.automations.write().await;
        if let Some(a) = automations.get_mut(&automation_id) {
            a.trigger_count += 1;
            a.last_triggered_at = Some(chrono::Utc::now());
        }

        Ok(())
    }

    /// Execute a single action
    async fn execute_action(
        &self,
        action: &Action,
        user_profile: Option<&UserProfile>,
    ) -> Result<()> {
        match action.action_type {
            ActionType::DeviceControl => {
                if let (Some(device_id), Some(command)) = (action.device_id, &action.command) {
                    self.control_device(device_id, command, action.parameters.clone(), user_profile)
                        .await?;
                }
            }
            ActionType::Delay => {
                if let Some(delay_ms) = action.delay_ms {
                    tokio::time::sleep(tokio::time::Duration::from_millis(delay_ms as u64)).await;
                }
            }
            ActionType::VoiceAnnouncement => {
                if let (Some(service), Some(text)) =
                    (&self.notification_service, action.parameters.get("text"))
                {
                    if let Some(text_str) = text.as_str() {
                        service.announce(text_str, &[]).await?;
                    }
                }
            }
            ActionType::Notification => {
                // Handle notification action
            }
            _ => {
                // Other action types
            }
        }
        Ok(())
    }

    // ========================================================================
    // Notification Management
    // ========================================================================

    /// Send notification with accessibility support
    pub async fn send_notification(
        &self,
        notification: Notification,
        target_profiles: &[Uuid],
    ) -> Result<()> {
        let service = self
            .notification_service
            .as_ref()
            .ok_or_else(|| SmartHomeError::Internal("No notification service".to_string()))?;

        // Adapt notification for each user's accessibility needs
        for profile_id in target_profiles {
            if let Ok(profile) = self.get_profile(*profile_id).await {
                let adapted = self.adapt_notification(&notification, &profile);
                service.send(&adapted).await?;
            }
        }

        Ok(())
    }

    /// Adapt notification to user's accessibility requirements
    fn adapt_notification(&self, notification: &Notification, profile: &UserProfile) -> Notification {
        let mut adapted = notification.clone();

        // Adjust modalities based on user preferences
        let preferred = &profile.interaction_preferences.preferred_output_modalities;
        adapted.delivery.modalities = adapted
            .delivery
            .modalities
            .iter()
            .filter(|m| preferred.contains(m))
            .cloned()
            .collect();

        // Ensure at least one modality
        if adapted.delivery.modalities.is_empty() {
            adapted.delivery.modalities = preferred.clone();
        }

        // Adjust TTS settings
        if let Some(audio) = &mut adapted.delivery.audio {
            audio.tts_language = profile.personal_info.preferred_language.clone();
            if let Some(voice_id) = &profile.interaction_preferences.voice_settings.voice_id {
                audio.tts_voice = Some(voice_id.clone());
            }
            audio.tts_rate = Some(profile.interaction_preferences.voice_settings.speech_rate);
        }

        adapted
    }
}

impl Default for SmartHomeController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_create_home() {
        let controller = SmartHomeController::new();
        let profile = controller.create_profile().await.unwrap();
        let home = controller
            .create_home("My Home".to_string(), profile.profile_id)
            .await
            .unwrap();
        assert_eq!(home.name, "My Home");
        assert_eq!(home.owner_profile_id, profile.profile_id);
    }

    #[tokio::test]
    async fn test_create_zone() {
        let controller = SmartHomeController::new();
        let profile = controller.create_profile().await.unwrap();
        let home = controller
            .create_home("My Home".to_string(), profile.profile_id)
            .await
            .unwrap();
        let zone = controller
            .create_zone(home.home_id, "Living Room".to_string(), ZoneType::LivingRoom)
            .await
            .unwrap();
        assert_eq!(zone.name, "Living Room");
        assert_eq!(zone.home_id, home.home_id);
    }

    #[tokio::test]
    async fn test_register_device() {
        let controller = SmartHomeController::new();
        let profile = controller.create_profile().await.unwrap();
        let home = controller
            .create_home("My Home".to_string(), profile.profile_id)
            .await
            .unwrap();
        let zone = controller
            .create_zone(home.home_id, "Living Room".to_string(), ZoneType::LivingRoom)
            .await
            .unwrap();
        let device = controller
            .register_device(
                DeviceType::Light,
                "Main Light".to_string(),
                home.home_id,
                Some(zone.zone_id),
            )
            .await
            .unwrap();
        assert_eq!(device.name, "Main Light");
        assert_eq!(device.device_type, DeviceType::Light);
    }

    #[tokio::test]
    async fn test_create_automation() {
        let controller = SmartHomeController::new();
        let profile = controller.create_profile().await.unwrap();
        let home = controller
            .create_home("My Home".to_string(), profile.profile_id)
            .await
            .unwrap();

        let trigger = Trigger {
            trigger_type: TriggerType::Time,
            config: HashMap::new(),
        };
        let actions = vec![Action {
            action_type: ActionType::VoiceAnnouncement,
            device_id: None,
            command: None,
            parameters: HashMap::new(),
            delay_ms: None,
        }];

        let automation = controller
            .create_automation("Morning Routine".to_string(), home.home_id, trigger, actions)
            .await
            .unwrap();
        assert_eq!(automation.name, "Morning Routine");
    }

    #[tokio::test]
    async fn test_user_profile() {
        let controller = SmartHomeController::new();
        let mut profile = controller.create_profile().await.unwrap();

        // Update accessibility requirements
        profile
            .accessibility_requirements
            .primary_disabilities
            .push(DisabilityType::VisualLowVision);
        profile
            .interaction_preferences
            .preferred_output_modalities
            .push(OutputModality::AudioTts);

        controller.update_profile(profile.clone()).await.unwrap();

        let retrieved = controller.get_profile(profile.profile_id).await.unwrap();
        assert!(!retrieved
            .accessibility_requirements
            .primary_disabilities
            .is_empty());
    }
}
