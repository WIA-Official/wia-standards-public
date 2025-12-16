//! Smart Home Gaming Integration
//!
//! Provides environmental effects synchronized with game state.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Smart home adapter for gaming
#[derive(Debug)]
pub struct SmartHomeAdapter {
    config: SmartHomeConfig,
    current_state: EnvironmentState,
    zones: HashMap<String, ZoneConfig>,
    effects_enabled: bool,
}

/// Smart home configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SmartHomeConfig {
    /// Enable smart home integration
    pub enabled: bool,
    /// Lighting sync settings
    pub lighting_sync: LightingSyncConfig,
    /// Climate sync settings
    pub climate_sync: ClimateSyncConfig,
    /// Do not disturb settings
    pub do_not_disturb: DoNotDisturbConfig,
    /// Connected zones
    pub zones: Vec<String>,
}

/// Lighting synchronization config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LightingSyncConfig {
    pub enabled: bool,
    /// Zones to control
    pub zones: Vec<String>,
    /// Effect intensity (0.0-1.0)
    pub intensity: f32,
    /// Transition speed
    pub transition_speed: TransitionSpeed,
    /// Color temperature range (2700K-6500K)
    pub color_temp_range: (u32, u32),
}

/// Climate synchronization config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClimateSyncConfig {
    pub enabled: bool,
    /// Enable fan for hot game environments
    pub fan_on_hot_environments: bool,
    /// Temperature offset for immersion
    pub temp_offset_enabled: bool,
    /// Max offset in degrees
    pub max_offset_degrees: f32,
}

/// Do not disturb settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DoNotDisturbConfig {
    pub enabled: bool,
    /// Mute doorbell during gameplay
    pub mute_doorbell: bool,
    /// Dim notification lights
    pub dim_notifications: bool,
    /// Reduce phone notifications
    pub phone_dnd: bool,
}

/// Transition speed for lighting
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TransitionSpeed {
    Instant,
    Fast,
    Smooth,
    Slow,
}

/// Zone configuration
#[derive(Debug, Clone)]
pub struct ZoneConfig {
    pub name: String,
    pub devices: Vec<DeviceType>,
    pub current_state: ZoneState,
}

/// Device types in a zone
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceType {
    Light,
    ColorLight,
    Fan,
    Thermostat,
    Blinds,
    Speaker,
}

/// Current zone state
#[derive(Debug, Clone, Default)]
pub struct ZoneState {
    pub brightness: f32,
    pub color: Option<RGBColor>,
    pub color_temp: Option<u32>,
    pub fan_speed: Option<f32>,
    pub temperature: Option<f32>,
}

/// RGB color
#[derive(Debug, Clone, Copy)]
pub struct RGBColor {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

/// Current environment state
#[derive(Debug, Clone, Default)]
pub struct EnvironmentState {
    pub brightness: f32,
    pub color: Option<RGBColor>,
    pub color_temp: u32,
    pub fan_active: bool,
    pub dnd_active: bool,
}

/// Environment command to send to smart home
#[derive(Debug, Clone)]
pub enum EnvironmentCommand {
    SetBrightness { zone: String, brightness: f32 },
    SetColor { zone: String, color: RGBColor },
    SetColorTemp { zone: String, kelvin: u32 },
    Flash { zone: String, color: RGBColor, count: u8 },
    Pulse { zone: String, color: RGBColor, duration_ms: u32 },
    SetFan { zone: String, speed: f32 },
    SetDND { enabled: bool },
    RunScene { scene_name: String },
}

impl SmartHomeAdapter {
    /// Create a new smart home adapter
    pub fn new() -> Self {
        Self {
            config: SmartHomeConfig::default(),
            current_state: EnvironmentState::default(),
            zones: HashMap::new(),
            effects_enabled: true,
        }
    }

    /// Configure the adapter
    pub fn configure(&mut self, config: SmartHomeConfig) {
        self.config = config;
    }

    /// Add a zone
    pub fn add_zone(&mut self, name: String, devices: Vec<DeviceType>) {
        self.zones.insert(
            name.clone(),
            ZoneConfig {
                name,
                devices,
                current_state: ZoneState::default(),
            },
        );
    }

    /// Enable or disable effects
    pub fn set_effects_enabled(&mut self, enabled: bool) {
        self.effects_enabled = enabled;
    }

    /// Sync environment with game state
    pub fn sync_with_game(&mut self, state: &super::GameState) -> Vec<EnvironmentCommand> {
        if !self.config.enabled || !self.effects_enabled {
            return Vec::new();
        }

        let mut commands = Vec::new();

        // Handle paused state
        if state.is_paused {
            commands.extend(self.restore_normal_lighting());
            return commands;
        }

        // Lighting based on time of day
        if self.config.lighting_sync.enabled {
            commands.extend(self.sync_time_of_day(&state.time_of_day));
            commands.extend(self.sync_weather(&state.weather));
            commands.extend(self.sync_combat(state.combat_active, state.stealth_mode));
            commands.extend(self.sync_health(state.player_health_percent));
        }

        // Climate sync
        if self.config.climate_sync.enabled {
            commands.extend(self.sync_climate(&state.weather));
        }

        commands
    }

    /// Sync lighting with time of day
    fn sync_time_of_day(&self, time: &super::TimeOfDay) -> Vec<EnvironmentCommand> {
        use super::TimeOfDay;

        let (brightness, color_temp) = match time {
            TimeOfDay::Dawn => (0.5, 3500),
            TimeOfDay::Day => (0.8, 5500),
            TimeOfDay::Dusk => (0.4, 3000),
            TimeOfDay::Night => (0.2, 2700),
        };

        let intensity = self.config.lighting_sync.intensity;

        self.config.lighting_sync.zones.iter().flat_map(|zone| {
            vec![
                EnvironmentCommand::SetBrightness {
                    zone: zone.clone(),
                    brightness: brightness * intensity,
                },
                EnvironmentCommand::SetColorTemp {
                    zone: zone.clone(),
                    kelvin: color_temp,
                },
            ]
        }).collect()
    }

    /// Sync lighting with weather
    fn sync_weather(&self, weather: &super::Weather) -> Vec<EnvironmentCommand> {
        use super::Weather;

        let color = match weather {
            Weather::Clear => None,
            Weather::Cloudy => Some(RGBColor { r: 200, g: 200, b: 210 }),
            Weather::Rain => Some(RGBColor { r: 100, g: 120, b: 180 }),
            Weather::Storm => Some(RGBColor { r: 80, g: 90, b: 150 }),
            Weather::Snow => Some(RGBColor { r: 220, g: 230, b: 255 }),
            Weather::Fog => Some(RGBColor { r: 180, g: 180, b: 190 }),
            Weather::Fire => Some(RGBColor { r: 255, g: 100, b: 50 }),
            Weather::Sandstorm => Some(RGBColor { r: 200, g: 150, b: 100 }),
        };

        match color {
            Some(c) => self.config.lighting_sync.zones.iter().map(|zone| {
                EnvironmentCommand::SetColor {
                    zone: zone.clone(),
                    color: c,
                }
            }).collect(),
            None => Vec::new(),
        }
    }

    /// Sync lighting with combat state
    fn sync_combat(&self, combat: bool, stealth: bool) -> Vec<EnvironmentCommand> {
        if stealth {
            // Dim lights for stealth
            return self.config.lighting_sync.zones.iter().map(|zone| {
                EnvironmentCommand::SetBrightness {
                    zone: zone.clone(),
                    brightness: 0.1,
                }
            }).collect();
        }

        if combat {
            // Alert lighting for combat
            return self.config.lighting_sync.zones.iter().map(|zone| {
                EnvironmentCommand::SetColor {
                    zone: zone.clone(),
                    color: RGBColor { r: 255, g: 80, b: 80 },
                }
            }).collect();
        }

        Vec::new()
    }

    /// Sync lighting with player health
    fn sync_health(&self, health_percent: f32) -> Vec<EnvironmentCommand> {
        if health_percent < 0.25 {
            // Critical health - pulse red
            return self.config.lighting_sync.zones.iter().map(|zone| {
                EnvironmentCommand::Pulse {
                    zone: zone.clone(),
                    color: RGBColor { r: 255, g: 0, b: 0 },
                    duration_ms: 1000,
                }
            }).collect();
        }

        Vec::new()
    }

    /// Sync climate with weather
    fn sync_climate(&self, weather: &super::Weather) -> Vec<EnvironmentCommand> {
        use super::Weather;

        if !self.config.climate_sync.fan_on_hot_environments {
            return Vec::new();
        }

        let fan_speed = match weather {
            Weather::Fire | Weather::Sandstorm => Some(0.8),
            Weather::Clear => Some(0.3), // Light breeze
            _ => None,
        };

        match fan_speed {
            Some(speed) => self.config.lighting_sync.zones.iter().map(|zone| {
                EnvironmentCommand::SetFan {
                    zone: zone.clone(),
                    speed,
                }
            }).collect(),
            None => Vec::new(),
        }
    }

    /// Restore normal lighting
    fn restore_normal_lighting(&self) -> Vec<EnvironmentCommand> {
        self.config.lighting_sync.zones.iter().flat_map(|zone| {
            vec![
                EnvironmentCommand::SetBrightness {
                    zone: zone.clone(),
                    brightness: 0.8,
                },
                EnvironmentCommand::SetColorTemp {
                    zone: zone.clone(),
                    kelvin: 4000,
                },
            ]
        }).collect()
    }

    /// Flash effect for events
    pub fn flash(&self, color: RGBColor, count: u8) -> Vec<EnvironmentCommand> {
        if !self.config.enabled || !self.effects_enabled {
            return Vec::new();
        }

        self.config.lighting_sync.zones.iter().map(|zone| {
            EnvironmentCommand::Flash {
                zone: zone.clone(),
                color,
                count,
            }
        }).collect()
    }

    /// Trigger damage flash
    pub fn damage_flash(&self) -> Vec<EnvironmentCommand> {
        self.flash(RGBColor { r: 255, g: 0, b: 0 }, 1)
    }

    /// Trigger victory effect
    pub fn victory_effect(&self) -> Vec<EnvironmentCommand> {
        if !self.config.enabled || !self.effects_enabled {
            return Vec::new();
        }

        self.config.lighting_sync.zones.iter().map(|zone| {
            EnvironmentCommand::RunScene {
                scene_name: "victory_celebration".to_string(),
            }
        }).collect()
    }

    /// Enable DND mode
    pub fn enable_dnd(&mut self) -> Option<EnvironmentCommand> {
        if !self.config.do_not_disturb.enabled {
            return None;
        }

        self.current_state.dnd_active = true;
        Some(EnvironmentCommand::SetDND { enabled: true })
    }

    /// Disable DND mode
    pub fn disable_dnd(&mut self) -> Option<EnvironmentCommand> {
        self.current_state.dnd_active = false;
        Some(EnvironmentCommand::SetDND { enabled: false })
    }
}

impl Default for SmartHomeAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for SmartHomeConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            lighting_sync: LightingSyncConfig {
                enabled: true,
                zones: vec!["gaming_room".to_string()],
                intensity: 0.7,
                transition_speed: TransitionSpeed::Smooth,
                color_temp_range: (2700, 6500),
            },
            climate_sync: ClimateSyncConfig {
                enabled: false,
                fan_on_hot_environments: false,
                temp_offset_enabled: false,
                max_offset_degrees: 2.0,
            },
            do_not_disturb: DoNotDisturbConfig {
                enabled: true,
                mute_doorbell: true,
                dim_notifications: true,
                phone_dnd: false,
            },
            zones: vec!["gaming_room".to_string()],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_smart_home_adapter_creation() {
        let adapter = SmartHomeAdapter::new();
        assert!(adapter.config.enabled);
    }

    #[test]
    fn test_sync_time_of_day() {
        let adapter = SmartHomeAdapter::new();
        let commands = adapter.sync_time_of_day(&super::super::TimeOfDay::Night);

        assert!(!commands.is_empty());
    }

    #[test]
    fn test_damage_flash() {
        let adapter = SmartHomeAdapter::new();
        let commands = adapter.damage_flash();

        assert!(!commands.is_empty());
        assert!(matches!(commands[0], EnvironmentCommand::Flash { .. }));
    }

    #[test]
    fn test_dnd_mode() {
        let mut adapter = SmartHomeAdapter::new();

        let cmd = adapter.enable_dnd();
        assert!(cmd.is_some());
        assert!(adapter.current_state.dnd_active);

        let cmd = adapter.disable_dnd();
        assert!(cmd.is_some());
        assert!(!adapter.current_state.dnd_active);
    }

    #[test]
    fn test_effects_disabled() {
        let mut adapter = SmartHomeAdapter::new();
        adapter.set_effects_enabled(false);

        let commands = adapter.damage_flash();
        assert!(commands.is_empty());
    }
}
