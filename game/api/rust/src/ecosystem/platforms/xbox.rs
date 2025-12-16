//! Xbox Platform Integration
//!
//! Integration with Xbox accessibility features and Copilot mode.

use crate::error::GameError;
use crate::types::PlayerProfile;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Xbox platform integration
#[derive(Debug, Clone)]
pub struct XboxIntegration {
    pub config: XboxConfig,
    pub accessibility: XboxAccessibilityFeatures,
    connected: bool,
}

/// Xbox configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct XboxConfig {
    /// Enable Xbox integration
    pub enabled: bool,
    /// Copilot mode enabled
    pub copilot_enabled: bool,
    /// Cloud save sync
    pub cloud_save_sync: bool,
    /// Auto-detect controller
    pub auto_detect_controller: bool,
}

/// Xbox accessibility features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct XboxAccessibilityFeatures {
    /// Narrator enabled
    pub narrator: bool,
    /// Magnifier enabled
    pub magnifier: bool,
    /// High contrast mode
    pub high_contrast: bool,
    /// Button remapping
    pub button_remapping: HashMap<XboxButton, XboxButton>,
    /// Copilot partner profile ID
    pub copilot_partner: Option<String>,
    /// Mono audio
    pub mono_audio: bool,
    /// Notification toasts position
    pub notification_position: NotificationPosition,
}

/// Xbox button identifiers
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum XboxButton {
    A,
    B,
    X,
    Y,
    LB,
    RB,
    LT,
    RT,
    LS,
    RS,
    DPadUp,
    DPadDown,
    DPadLeft,
    DPadRight,
    Start,
    Back,
    Guide,
}

/// Notification toast position
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NotificationPosition {
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight,
    Center,
}

/// Xbox Adaptive Controller configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdaptiveControllerConfig {
    /// Connected port mappings
    pub port_mappings: HashMap<u8, XboxButton>,
    /// Profile slots
    pub profile_slots: [Option<String>; 3],
    /// Button hold time for long press (ms)
    pub long_press_ms: u32,
}

impl XboxIntegration {
    /// Create new Xbox integration
    pub fn new() -> Self {
        Self {
            config: XboxConfig::default(),
            accessibility: XboxAccessibilityFeatures::default(),
            connected: false,
        }
    }

    /// Configure Xbox integration
    pub fn configure(&mut self, config: XboxConfig) {
        self.config = config;
    }

    /// Connect to Xbox services (simulated)
    pub async fn connect(&mut self) -> Result<(), GameError> {
        // Simulated connection
        self.connected = true;
        Ok(())
    }

    /// Check if connected
    pub fn is_connected(&self) -> bool {
        self.connected
    }

    /// Sync WIA profile to Xbox accessibility settings
    pub async fn sync_profile(&self, profile: &PlayerProfile) -> Result<(), GameError> {
        if !self.connected && !self.config.enabled {
            return Err(GameError::InvalidConfiguration(
                "Xbox not connected".to_string(),
            ));
        }

        // Convert WIA profile to Xbox settings
        let _xbox_settings = self.convert_to_xbox_settings(profile);

        // Would call Xbox API here
        Ok(())
    }

    /// Convert WIA profile to Xbox accessibility settings
    fn convert_to_xbox_settings(&self, profile: &PlayerProfile) -> XboxAccessibilityFeatures {
        XboxAccessibilityFeatures {
            narrator: profile.visual_settings.screen_reader.enabled,
            magnifier: profile.visual_settings.magnification.enabled,
            high_contrast: profile.visual_settings.color_settings.high_contrast,
            button_remapping: self.convert_remapping(profile),
            copilot_partner: None, // Would need copilot_mode in profile
            mono_audio: profile.audio_settings.mono_audio.enabled,
            notification_position: NotificationPosition::TopRight,
        }
    }

    /// Convert WIA remapping to Xbox format
    fn convert_remapping(
        &self,
        profile: &PlayerProfile,
    ) -> HashMap<XboxButton, XboxButton> {
        let mut result = HashMap::new();

        // Convert from ControllerConfig button_mapping if available
        if let Some(ref config) = profile.motor_settings.button_remapping {
            for (from, _action) in &config.button_mapping {
                // Map button names to Xbox buttons
                if let Some(xbox_from) = self.string_to_xbox_button(from) {
                    // For simplicity, we just validate the button exists
                    result.insert(xbox_from, xbox_from);
                }
            }
        }

        result
    }

    /// Convert string button name to Xbox button
    fn string_to_xbox_button(&self, name: &str) -> Option<XboxButton> {
        match name.to_uppercase().as_str() {
            "A" => Some(XboxButton::A),
            "B" => Some(XboxButton::B),
            "X" => Some(XboxButton::X),
            "Y" => Some(XboxButton::Y),
            "LB" | "LEFT_BUMPER" => Some(XboxButton::LB),
            "RB" | "RIGHT_BUMPER" => Some(XboxButton::RB),
            "LT" | "LEFT_TRIGGER" => Some(XboxButton::LT),
            "RT" | "RIGHT_TRIGGER" => Some(XboxButton::RT),
            "LS" | "LEFT_STICK" => Some(XboxButton::LS),
            "RS" | "RIGHT_STICK" => Some(XboxButton::RS),
            "START" => Some(XboxButton::Start),
            "BACK" | "SELECT" => Some(XboxButton::Back),
            _ => None,
        }
    }

    /// Enable Copilot mode
    pub async fn enable_copilot(&mut self, partner_id: String) -> Result<(), GameError> {
        if !self.connected {
            return Err(GameError::InvalidConfiguration(
                "Xbox not connected".to_string(),
            ));
        }

        self.accessibility.copilot_partner = Some(partner_id);
        self.config.copilot_enabled = true;
        Ok(())
    }

    /// Disable Copilot mode
    pub fn disable_copilot(&mut self) {
        self.accessibility.copilot_partner = None;
        self.config.copilot_enabled = false;
    }

    /// Get Adaptive Controller configuration
    pub fn get_adaptive_config(&self) -> AdaptiveControllerConfig {
        AdaptiveControllerConfig::default()
    }
}

impl Default for XboxIntegration {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for XboxConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            copilot_enabled: false,
            cloud_save_sync: true,
            auto_detect_controller: true,
        }
    }
}

impl Default for XboxAccessibilityFeatures {
    fn default() -> Self {
        Self {
            narrator: false,
            magnifier: false,
            high_contrast: false,
            button_remapping: HashMap::new(),
            copilot_partner: None,
            mono_audio: false,
            notification_position: NotificationPosition::TopRight,
        }
    }
}

impl Default for AdaptiveControllerConfig {
    fn default() -> Self {
        Self {
            port_mappings: HashMap::new(),
            profile_slots: [None, None, None],
            long_press_ms: 500,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_xbox_integration_creation() {
        let xbox = XboxIntegration::new();
        assert!(!xbox.is_connected());
    }

    #[tokio::test]
    async fn test_connect() {
        let mut xbox = XboxIntegration::new();
        let result = xbox.connect().await;
        assert!(result.is_ok());
        assert!(xbox.is_connected());
    }

    #[test]
    fn test_button_conversion() {
        let xbox = XboxIntegration::new();
        assert_eq!(xbox.string_to_xbox_button("A"), Some(XboxButton::A));
        assert_eq!(xbox.string_to_xbox_button("LB"), Some(XboxButton::LB));
        assert_eq!(xbox.string_to_xbox_button("invalid"), None);
    }

    #[tokio::test]
    async fn test_copilot() {
        let mut xbox = XboxIntegration::new();
        xbox.connect().await.unwrap();

        xbox.enable_copilot("partner_123".to_string()).await.unwrap();
        assert!(xbox.config.copilot_enabled);
        assert!(xbox.accessibility.copilot_partner.is_some());

        xbox.disable_copilot();
        assert!(!xbox.config.copilot_enabled);
    }
}
