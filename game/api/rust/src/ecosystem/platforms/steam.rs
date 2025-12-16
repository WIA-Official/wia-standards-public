//! Steam Platform Integration
//!
//! Integration with Steam Input API and cloud saves.

use crate::error::GameError;
use crate::types::PlayerProfile;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Steam platform integration
#[derive(Debug, Clone)]
pub struct SteamIntegration {
    pub config: SteamConfig,
    pub input_config: SteamInputConfig,
    connected: bool,
}

/// Steam configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SteamConfig {
    /// Enable Steam integration
    pub enabled: bool,
    /// Cloud save sync
    pub cloud_save: bool,
    /// Remote Play settings
    pub remote_play: RemotePlaySettings,
    /// Big Picture mode preference
    pub big_picture_mode: bool,
    /// Steam Deck optimizations
    pub steam_deck: bool,
}

/// Remote Play settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RemotePlaySettings {
    pub enabled: bool,
    /// Hardware encoding
    pub hardware_encoding: bool,
    /// Limit bandwidth (Mbps, 0 = unlimited)
    pub bandwidth_limit: u32,
    /// Enable Steam Link optimization
    pub steam_link: bool,
}

/// Steam Input configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SteamInputConfig {
    /// Controller type
    pub controller_type: SteamControllerType,
    /// Action sets
    pub action_sets: HashMap<String, ActionSet>,
    /// Gyro settings
    pub gyro: Option<GyroSettings>,
    /// Touch settings (for Steam Controller/Deck)
    pub touch: Option<TouchSettings>,
    /// Haptic settings
    pub haptics: HapticSettings,
}

/// Steam controller types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SteamControllerType {
    SteamController,
    SteamDeck,
    Xbox,
    PlayStation,
    Switch,
    Generic,
}

/// Action set for Steam Input
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActionSet {
    /// Set name
    pub name: String,
    /// Digital actions (buttons)
    pub digital_actions: HashMap<String, DigitalAction>,
    /// Analog actions (sticks, triggers)
    pub analog_actions: HashMap<String, AnalogAction>,
}

/// Digital action binding
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DigitalAction {
    pub name: String,
    pub bindings: Vec<SteamBinding>,
}

/// Analog action binding
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalogAction {
    pub name: String,
    pub mode: AnalogMode,
    pub bindings: Vec<SteamBinding>,
}

/// Analog input modes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AnalogMode {
    AbsoluteMouse,
    RelativeMouse,
    JoystickMove,
    JoystickCamera,
    ScrollWheel,
    Trigger,
}

/// Steam input binding
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SteamBinding {
    pub input: SteamInput,
    pub activator: Option<Activator>,
}

/// Steam input sources
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SteamInput {
    Button(String),
    Stick(String),
    Trigger(String),
    Gyro,
    TouchPad(String),
}

/// Binding activators
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Activator {
    FullPress,
    SoftPress,
    LongPress { duration_ms: u32 },
    DoublePress,
    Chorded { with: String },
}

/// Gyro settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GyroSettings {
    pub enabled: bool,
    /// Sensitivity
    pub sensitivity: f32,
    /// Gyro activation mode
    pub activation: GyroActivation,
    /// Invert axes
    pub invert_x: bool,
    pub invert_y: bool,
}

/// Gyro activation modes
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum GyroActivation {
    AlwaysOn,
    OnTouch { touch_pad: String },
    OnButton { button: String },
    Off,
}

/// Touch pad settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TouchSettings {
    /// Touch pad mode
    pub mode: TouchMode,
    /// Sensitivity
    pub sensitivity: f32,
    /// Haptic feedback strength
    pub haptic_intensity: f32,
}

/// Touch pad modes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TouchMode {
    Mouse,
    Joystick,
    DPad,
    RadialMenu,
    TouchMenu,
}

/// Haptic settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticSettings {
    pub enabled: bool,
    /// Intensity (0.0-1.0)
    pub intensity: f32,
    /// Enable HD haptics (Steam Controller/Deck)
    pub hd_haptics: bool,
}

impl SteamIntegration {
    /// Create new Steam integration
    pub fn new() -> Self {
        Self {
            config: SteamConfig::default(),
            input_config: SteamInputConfig::default(),
            connected: false,
        }
    }

    /// Configure Steam integration
    pub fn configure(&mut self, config: SteamConfig) {
        self.config = config;
    }

    /// Connect to Steam (simulated)
    pub async fn connect(&mut self) -> Result<(), GameError> {
        self.connected = true;
        Ok(())
    }

    /// Check if connected
    pub fn is_connected(&self) -> bool {
        self.connected
    }

    /// Sync WIA profile to Steam Input config
    pub async fn sync_profile(&self, profile: &PlayerProfile) -> Result<(), GameError> {
        if !self.connected && !self.config.enabled {
            return Err(GameError::InvalidConfiguration(
                "Steam not connected".to_string(),
            ));
        }

        let _steam_config = self.convert_to_steam_config(profile);
        Ok(())
    }

    /// Convert WIA profile to Steam Input config
    fn convert_to_steam_config(&self, profile: &PlayerProfile) -> SteamInputConfig {
        let mut config = SteamInputConfig::default();

        // Set up gyro based on aim assist enabled
        if profile.motor_settings.aim_assist.enabled {
            config.gyro = Some(GyroSettings {
                enabled: true,
                sensitivity: profile.motor_settings.stick_settings.left_sensitivity,
                activation: GyroActivation::OnTouch {
                    touch_pad: "right".to_string(),
                },
                invert_x: false,
                invert_y: false,
            });
        }

        // Create action set from remapping
        let mut main_set = ActionSet {
            name: "WIA_Accessibility".to_string(),
            digital_actions: HashMap::new(),
            analog_actions: HashMap::new(),
        };

        // Convert from ControllerConfig if available
        if let Some(ref ctrl_config) = profile.motor_settings.button_remapping {
            for (button_name, action) in &ctrl_config.button_mapping {
                main_set.digital_actions.insert(
                    button_name.clone(),
                    DigitalAction {
                        name: button_name.clone(),
                        bindings: vec![SteamBinding {
                            input: SteamInput::Button(format!("{:?}", action)),
                            activator: None,
                        }],
                    },
                );
            }
        }

        config.action_sets.insert("WIA_Accessibility".to_string(), main_set);
        config
    }

    /// Export configuration file for Steam Input
    pub fn export_vdf(&self) -> String {
        // Would generate Steam VDF format
        r#"
"controller_mappings"
{
    "version"       "3"
    "title"         "WIA Accessibility Profile"
    "description"   "Generated from WIA Game profile"
}
"#.to_string()
    }

    /// Enable Steam Deck optimizations
    pub fn enable_steam_deck(&mut self) {
        self.config.steam_deck = true;
        self.input_config.controller_type = SteamControllerType::SteamDeck;
    }

    /// Set up touch pads
    pub fn configure_touch(&mut self, left: TouchMode, right: TouchMode) {
        self.input_config.touch = Some(TouchSettings {
            mode: right,
            sensitivity: 0.5,
            haptic_intensity: 0.5,
        });
    }
}

impl Default for SteamIntegration {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for SteamConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            cloud_save: true,
            remote_play: RemotePlaySettings {
                enabled: true,
                hardware_encoding: true,
                bandwidth_limit: 0,
                steam_link: true,
            },
            big_picture_mode: false,
            steam_deck: false,
        }
    }
}

impl Default for SteamInputConfig {
    fn default() -> Self {
        Self {
            controller_type: SteamControllerType::Generic,
            action_sets: HashMap::new(),
            gyro: None,
            touch: None,
            haptics: HapticSettings {
                enabled: true,
                intensity: 0.5,
                hd_haptics: false,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_steam_creation() {
        let steam = SteamIntegration::new();
        assert!(!steam.is_connected());
    }

    #[tokio::test]
    async fn test_connect() {
        let mut steam = SteamIntegration::new();
        let result = steam.connect().await;
        assert!(result.is_ok());
        assert!(steam.is_connected());
    }

    #[test]
    fn test_steam_deck() {
        let mut steam = SteamIntegration::new();
        steam.enable_steam_deck();
        assert!(steam.config.steam_deck);
        assert_eq!(steam.input_config.controller_type, SteamControllerType::SteamDeck);
    }

    #[test]
    fn test_export_vdf() {
        let steam = SteamIntegration::new();
        let vdf = steam.export_vdf();
        assert!(vdf.contains("controller_mappings"));
    }
}
