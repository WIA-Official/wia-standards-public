//! PlayStation Platform Integration
//!
//! Integration with PlayStation Access controller and accessibility features.

use crate::error::GameError;
use crate::types::PlayerProfile;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// PlayStation platform integration
#[derive(Debug, Clone)]
pub struct PlayStationIntegration {
    pub config: PlayStationConfig,
    pub access_controller: Option<AccessControllerConfig>,
    connected: bool,
}

/// PlayStation configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlayStationConfig {
    /// Enable PlayStation integration
    pub enabled: bool,
    /// Custom button assignments
    pub custom_buttons: HashMap<PSButton, PSButton>,
    /// Touch pad settings
    pub touch_pad: TouchPadSettings,
    /// Motion control settings
    pub motion_control: MotionControlSettings,
}

/// PlayStation button identifiers
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PSButton {
    Cross,
    Circle,
    Square,
    Triangle,
    L1,
    R1,
    L2,
    R2,
    L3,
    R3,
    DPadUp,
    DPadDown,
    DPadLeft,
    DPadRight,
    Options,
    Create,
    PS,
    TouchPad,
}

/// Touch pad configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TouchPadSettings {
    pub enabled: bool,
    /// Swipe gestures
    pub swipe_enabled: bool,
    /// Tap to click
    pub tap_to_click: bool,
    /// Sensitivity (0.0-1.0)
    pub sensitivity: f32,
    /// Touch regions for button mapping
    pub regions: Vec<TouchRegion>,
}

/// Touch region for mapping
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TouchRegion {
    /// Region name
    pub name: String,
    /// X range (0.0-1.0)
    pub x_range: (f32, f32),
    /// Y range (0.0-1.0)
    pub y_range: (f32, f32),
    /// Mapped button
    pub button: PSButton,
}

/// Motion control settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotionControlSettings {
    pub enabled: bool,
    /// Gyro aiming
    pub gyro_aiming: bool,
    /// Gyro sensitivity
    pub gyro_sensitivity: f32,
    /// Accelerometer input
    pub accelerometer: bool,
}

/// PlayStation Access controller configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessControllerConfig {
    /// Profile slots
    pub profiles: [Option<AccessProfile>; 4],
    /// Button mapping
    pub button_mapping: HashMap<AccessButton, PSButton>,
    /// Stick sensitivity
    pub stick_sensitivity: (f32, f32),
    /// Controller orientation
    pub orientation: ControllerOrientation,
    /// Expansion port assignments
    pub expansion_ports: [Option<PSButton>; 4],
}

/// Access controller profile
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessProfile {
    pub name: String,
    pub button_map: HashMap<AccessButton, PSButton>,
    pub stick_config: StickConfig,
}

/// Access controller button identifiers
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum AccessButton {
    Button1,
    Button2,
    Button3,
    Button4,
    Button5,
    Button6,
    Button7,
    Button8,
    StickButton,
    StickUp,
    StickDown,
    StickLeft,
    StickRight,
    Expansion1,
    Expansion2,
    Expansion3,
    Expansion4,
}

/// Controller orientation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ControllerOrientation {
    Standard,
    Rotated90,
    Rotated180,
    Rotated270,
}

/// Stick configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StickConfig {
    /// Dead zone (0.0-1.0)
    pub dead_zone: f32,
    /// Sensitivity curve
    pub curve: SensitivityCurve,
    /// Invert X axis
    pub invert_x: bool,
    /// Invert Y axis
    pub invert_y: bool,
}

/// Sensitivity curves
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SensitivityCurve {
    Linear,
    Smooth,
    Aggressive,
    Custom,
}

impl PlayStationIntegration {
    /// Create new PlayStation integration
    pub fn new() -> Self {
        Self {
            config: PlayStationConfig::default(),
            access_controller: None,
            connected: false,
        }
    }

    /// Configure PlayStation integration
    pub fn configure(&mut self, config: PlayStationConfig) {
        self.config = config;
    }

    /// Connect to PlayStation services (simulated)
    pub async fn connect(&mut self) -> Result<(), GameError> {
        self.connected = true;
        Ok(())
    }

    /// Check if connected
    pub fn is_connected(&self) -> bool {
        self.connected
    }

    /// Sync WIA profile to PlayStation settings
    pub async fn sync_profile(&self, profile: &PlayerProfile) -> Result<(), GameError> {
        if !self.connected && !self.config.enabled {
            return Err(GameError::InvalidConfiguration(
                "PlayStation not connected".to_string(),
            ));
        }

        // Convert and sync profile
        let _ps_config = self.convert_to_ps_config(profile);
        Ok(())
    }

    /// Convert WIA profile to PlayStation config
    fn convert_to_ps_config(&self, profile: &PlayerProfile) -> PlayStationConfig {
        let mut config = PlayStationConfig::default();

        // Convert button remapping from ControllerConfig if available
        if let Some(ref ctrl_config) = profile.motor_settings.button_remapping {
            for (button_name, _action) in &ctrl_config.button_mapping {
                if let Some(ps_btn) = self.string_to_ps_button(button_name) {
                    // Map to same button for now (actual remapping would need action parsing)
                    config.custom_buttons.insert(ps_btn, ps_btn);
                }
            }
        }

        // Motion control based on profile - using aim_assist enabled as proxy
        config.motion_control.gyro_aiming = profile.motor_settings.aim_assist.enabled;

        config
    }

    /// Convert string to PlayStation button
    fn string_to_ps_button(&self, name: &str) -> Option<PSButton> {
        match name.to_uppercase().as_str() {
            "A" | "CROSS" => Some(PSButton::Cross),
            "B" | "CIRCLE" | "O" => Some(PSButton::Circle),
            "X" | "SQUARE" => Some(PSButton::Square),
            "Y" | "TRIANGLE" => Some(PSButton::Triangle),
            "LB" | "L1" => Some(PSButton::L1),
            "RB" | "R1" => Some(PSButton::R1),
            "LT" | "L2" => Some(PSButton::L2),
            "RT" | "R2" => Some(PSButton::R2),
            "LS" | "L3" => Some(PSButton::L3),
            "RS" | "R3" => Some(PSButton::R3),
            "START" | "OPTIONS" => Some(PSButton::Options),
            "BACK" | "SELECT" | "CREATE" => Some(PSButton::Create),
            _ => None,
        }
    }

    /// Enable Access controller
    pub fn enable_access_controller(&mut self, config: AccessControllerConfig) {
        self.access_controller = Some(config);
    }

    /// Get Access controller config
    pub fn get_access_controller(&self) -> Option<&AccessControllerConfig> {
        self.access_controller.as_ref()
    }

    /// Switch Access controller profile
    pub fn switch_access_profile(&mut self, slot: usize) -> Result<(), GameError> {
        if let Some(ref mut access) = self.access_controller {
            if slot < 4 && access.profiles[slot].is_some() {
                // Would switch profile
                Ok(())
            } else {
                Err(GameError::NotFound("Profile slot empty".to_string()))
            }
        } else {
            Err(GameError::InvalidConfiguration(
                "Access controller not enabled".to_string(),
            ))
        }
    }
}

impl Default for PlayStationIntegration {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for PlayStationConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            custom_buttons: HashMap::new(),
            touch_pad: TouchPadSettings {
                enabled: true,
                swipe_enabled: true,
                tap_to_click: true,
                sensitivity: 0.5,
                regions: Vec::new(),
            },
            motion_control: MotionControlSettings {
                enabled: false,
                gyro_aiming: false,
                gyro_sensitivity: 0.5,
                accelerometer: false,
            },
        }
    }
}

impl Default for AccessControllerConfig {
    fn default() -> Self {
        Self {
            profiles: [None, None, None, None],
            button_mapping: HashMap::new(),
            stick_sensitivity: (0.5, 0.5),
            orientation: ControllerOrientation::Standard,
            expansion_ports: [None, None, None, None],
        }
    }
}

impl Default for StickConfig {
    fn default() -> Self {
        Self {
            dead_zone: 0.1,
            curve: SensitivityCurve::Linear,
            invert_x: false,
            invert_y: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_playstation_creation() {
        let ps = PlayStationIntegration::new();
        assert!(!ps.is_connected());
    }

    #[tokio::test]
    async fn test_connect() {
        let mut ps = PlayStationIntegration::new();
        let result = ps.connect().await;
        assert!(result.is_ok());
        assert!(ps.is_connected());
    }

    #[test]
    fn test_button_conversion() {
        let ps = PlayStationIntegration::new();
        assert_eq!(ps.string_to_ps_button("CROSS"), Some(PSButton::Cross));
        assert_eq!(ps.string_to_ps_button("L1"), Some(PSButton::L1));
        assert_eq!(ps.string_to_ps_button("invalid"), None);
    }

    #[test]
    fn test_access_controller() {
        let mut ps = PlayStationIntegration::new();
        assert!(ps.get_access_controller().is_none());

        ps.enable_access_controller(AccessControllerConfig::default());
        assert!(ps.get_access_controller().is_some());
    }
}
