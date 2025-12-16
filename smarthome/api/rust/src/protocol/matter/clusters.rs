//! Matter Cluster Definitions
//! 弘益人間 - Benefit All Humanity

use serde::{Deserialize, Serialize};

/// Standard Matter Cluster IDs
pub mod cluster_ids {
    /// On/Off cluster
    pub const ON_OFF: u16 = 0x0006;
    /// Level Control cluster
    pub const LEVEL_CONTROL: u16 = 0x0008;
    /// Color Control cluster
    pub const COLOR_CONTROL: u16 = 0x0300;
    /// Door Lock cluster
    pub const DOOR_LOCK: u16 = 0x0101;
    /// Thermostat cluster
    pub const THERMOSTAT: u16 = 0x0201;
    /// Fan Control cluster
    pub const FAN_CONTROL: u16 = 0x0202;
    /// Window Covering cluster
    pub const WINDOW_COVERING: u16 = 0x0102;
    /// Occupancy Sensing cluster
    pub const OCCUPANCY_SENSING: u16 = 0x0406;
    /// Temperature Measurement cluster
    pub const TEMPERATURE_MEASUREMENT: u16 = 0x0402;
    /// Humidity Measurement cluster
    pub const HUMIDITY_MEASUREMENT: u16 = 0x0405;
}

/// WIA Accessibility Cluster IDs (Custom)
pub mod wia_cluster_ids {
    /// Voice Command cluster
    pub const VOICE_COMMAND: u16 = 0xFFF1;
    /// Audio Feedback cluster
    pub const AUDIO_FEEDBACK: u16 = 0xFFF2;
    /// Visual Feedback cluster
    pub const VISUAL_FEEDBACK: u16 = 0xFFF3;
    /// Haptic Feedback cluster
    pub const HAPTIC_FEEDBACK: u16 = 0xFFF4;
    /// Switch Access cluster
    pub const SWITCH_ACCESS: u16 = 0xFFF5;
    /// Dwell Control cluster
    pub const DWELL_CONTROL: u16 = 0xFFF6;
}

// ============================================================================
// On/Off Cluster (0x0006)
// ============================================================================

/// On/Off cluster command IDs
pub mod on_off_commands {
    pub const OFF: u8 = 0x00;
    pub const ON: u8 = 0x01;
    pub const TOGGLE: u8 = 0x02;
}

/// On/Off cluster state
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct OnOffState {
    /// Current on/off state
    pub on_off: bool,
    /// Global scene control
    pub global_scene_control: bool,
    /// On time (in 1/10th seconds)
    pub on_time: u16,
    /// Off wait time (in 1/10th seconds)
    pub off_wait_time: u16,
}

/// On/Off command
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum OnOffCommand {
    Off,
    On,
    Toggle,
    OffWithEffect { effect_id: u8, effect_variant: u8 },
    OnWithRecall,
    OnWithTimedOff { on_off_control: u8, on_time: u16, off_wait_time: u16 },
}

impl OnOffCommand {
    pub fn to_bytes(&self) -> Vec<u8> {
        match self {
            OnOffCommand::Off => vec![on_off_commands::OFF],
            OnOffCommand::On => vec![on_off_commands::ON],
            OnOffCommand::Toggle => vec![on_off_commands::TOGGLE],
            OnOffCommand::OffWithEffect { effect_id, effect_variant } => {
                vec![0x40, *effect_id, *effect_variant]
            }
            OnOffCommand::OnWithRecall => vec![0x41],
            OnOffCommand::OnWithTimedOff { on_off_control, on_time, off_wait_time } => {
                let mut data = vec![0x42, *on_off_control];
                data.extend_from_slice(&on_time.to_le_bytes());
                data.extend_from_slice(&off_wait_time.to_le_bytes());
                data
            }
        }
    }
}

// ============================================================================
// Level Control Cluster (0x0008)
// ============================================================================

/// Level Control cluster command IDs
pub mod level_control_commands {
    pub const MOVE_TO_LEVEL: u8 = 0x00;
    pub const MOVE: u8 = 0x01;
    pub const STEP: u8 = 0x02;
    pub const STOP: u8 = 0x03;
    pub const MOVE_TO_LEVEL_WITH_ON_OFF: u8 = 0x04;
}

/// Level Control state
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LevelControlState {
    /// Current level (0-254)
    pub current_level: u8,
    /// Remaining time
    pub remaining_time: u16,
    /// Min level
    pub min_level: u8,
    /// Max level
    pub max_level: u8,
    /// On level
    pub on_level: Option<u8>,
}

/// Move mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MoveMode {
    Up = 0,
    Down = 1,
}

/// Level Control command
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LevelControlCommand {
    MoveToLevel { level: u8, transition_time: u16 },
    Move { mode: MoveMode, rate: u8 },
    Step { mode: MoveMode, step_size: u8, transition_time: u16 },
    Stop,
    MoveToLevelWithOnOff { level: u8, transition_time: u16 },
}

impl LevelControlCommand {
    pub fn to_bytes(&self) -> Vec<u8> {
        match self {
            LevelControlCommand::MoveToLevel { level, transition_time } => {
                let mut data = vec![level_control_commands::MOVE_TO_LEVEL, *level];
                data.extend_from_slice(&transition_time.to_le_bytes());
                data
            }
            LevelControlCommand::Move { mode, rate } => {
                vec![level_control_commands::MOVE, *mode as u8, *rate]
            }
            LevelControlCommand::Step { mode, step_size, transition_time } => {
                let mut data = vec![level_control_commands::STEP, *mode as u8, *step_size];
                data.extend_from_slice(&transition_time.to_le_bytes());
                data
            }
            LevelControlCommand::Stop => vec![level_control_commands::STOP],
            LevelControlCommand::MoveToLevelWithOnOff { level, transition_time } => {
                let mut data = vec![level_control_commands::MOVE_TO_LEVEL_WITH_ON_OFF, *level];
                data.extend_from_slice(&transition_time.to_le_bytes());
                data
            }
        }
    }
}

// ============================================================================
// Color Control Cluster (0x0300)
// ============================================================================

/// Color Control state
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ColorControlState {
    /// Current hue (0-254)
    pub current_hue: u8,
    /// Current saturation (0-254)
    pub current_saturation: u8,
    /// Current X (CIE)
    pub current_x: u16,
    /// Current Y (CIE)
    pub current_y: u16,
    /// Color temperature mireds
    pub color_temperature_mireds: u16,
    /// Color mode
    pub color_mode: ColorMode,
}

/// Color mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum ColorMode {
    #[default]
    CurrentHueSaturation = 0,
    CurrentXY = 1,
    ColorTemperature = 2,
}

/// Color Control command
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ColorControlCommand {
    MoveToHue { hue: u8, direction: u8, transition_time: u16 },
    MoveToSaturation { saturation: u8, transition_time: u16 },
    MoveToHueAndSaturation { hue: u8, saturation: u8, transition_time: u16 },
    MoveToColor { x: u16, y: u16, transition_time: u16 },
    MoveToColorTemperature { color_temperature: u16, transition_time: u16 },
}

// ============================================================================
// Door Lock Cluster (0x0101)
// ============================================================================

/// Lock state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum LockState {
    NotFullyLocked = 0,
    Locked = 1,
    #[default]
    Unlocked = 2,
    Unlatched = 3,
}

/// Door Lock state
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DoorLockState {
    /// Lock state
    pub lock_state: LockState,
    /// Lock type
    pub lock_type: u8,
    /// Actuator enabled
    pub actuator_enabled: bool,
    /// Door state
    pub door_state: Option<u8>,
}

/// Door Lock command
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DoorLockCommand {
    Lock { pin_code: Option<Vec<u8>> },
    Unlock { pin_code: Option<Vec<u8>> },
    Toggle,
}

// ============================================================================
// Thermostat Cluster (0x0201)
// ============================================================================

/// Thermostat mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum ThermostatMode {
    #[default]
    Off = 0,
    Auto = 1,
    Cool = 3,
    Heat = 4,
    EmergencyHeat = 5,
    Precooling = 6,
    FanOnly = 7,
}

/// Thermostat state
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ThermostatState {
    /// Local temperature (in 0.01°C)
    pub local_temperature: i16,
    /// Occupied cooling setpoint
    pub occupied_cooling_setpoint: i16,
    /// Occupied heating setpoint
    pub occupied_heating_setpoint: i16,
    /// System mode
    pub system_mode: ThermostatMode,
}

/// Thermostat command
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ThermostatCommand {
    SetSetpoint { mode: ThermostatMode, amount: i8 },
    SetWeeklySchedule { /* ... */ },
    GetWeeklySchedule { days: u8, mode: u8 },
    ClearWeeklySchedule,
}

// ============================================================================
// Cluster Trait
// ============================================================================

/// Trait for Matter clusters
pub trait Cluster {
    /// Get cluster ID
    fn cluster_id(&self) -> u16;

    /// Process command and return response data
    fn process_command(&mut self, command_id: u8, data: &[u8]) -> Result<Vec<u8>, ClusterError>;

    /// Get attribute value
    fn get_attribute(&self, attribute_id: u16) -> Result<Vec<u8>, ClusterError>;

    /// Set attribute value
    fn set_attribute(&mut self, attribute_id: u16, value: &[u8]) -> Result<(), ClusterError>;
}

/// Cluster error
#[derive(Debug, Clone, thiserror::Error)]
pub enum ClusterError {
    #[error("Unknown command: {0}")]
    UnknownCommand(u8),
    #[error("Unknown attribute: {0}")]
    UnknownAttribute(u16),
    #[error("Invalid data")]
    InvalidData,
    #[error("Read only attribute")]
    ReadOnly,
    #[error("Unsupported")]
    Unsupported,
}

// ============================================================================
// On/Off Cluster Implementation
// ============================================================================

impl Cluster for OnOffState {
    fn cluster_id(&self) -> u16 {
        cluster_ids::ON_OFF
    }

    fn process_command(&mut self, command_id: u8, _data: &[u8]) -> Result<Vec<u8>, ClusterError> {
        match command_id {
            on_off_commands::OFF => {
                self.on_off = false;
                Ok(vec![0x00]) // Success
            }
            on_off_commands::ON => {
                self.on_off = true;
                Ok(vec![0x00])
            }
            on_off_commands::TOGGLE => {
                self.on_off = !self.on_off;
                Ok(vec![0x00])
            }
            _ => Err(ClusterError::UnknownCommand(command_id)),
        }
    }

    fn get_attribute(&self, attribute_id: u16) -> Result<Vec<u8>, ClusterError> {
        match attribute_id {
            0x0000 => Ok(vec![self.on_off as u8]), // OnOff attribute
            0x4000 => Ok(vec![self.global_scene_control as u8]),
            0x4001 => Ok(self.on_time.to_le_bytes().to_vec()),
            0x4002 => Ok(self.off_wait_time.to_le_bytes().to_vec()),
            _ => Err(ClusterError::UnknownAttribute(attribute_id)),
        }
    }

    fn set_attribute(&mut self, attribute_id: u16, value: &[u8]) -> Result<(), ClusterError> {
        match attribute_id {
            0x4001 if value.len() >= 2 => {
                self.on_time = u16::from_le_bytes([value[0], value[1]]);
                Ok(())
            }
            0x4002 if value.len() >= 2 => {
                self.off_wait_time = u16::from_le_bytes([value[0], value[1]]);
                Ok(())
            }
            0x0000 | 0x4000 => Err(ClusterError::ReadOnly),
            _ => Err(ClusterError::UnknownAttribute(attribute_id)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_on_off_cluster() {
        let mut state = OnOffState::default();
        assert!(!state.on_off);

        // Turn on
        state.process_command(on_off_commands::ON, &[]).unwrap();
        assert!(state.on_off);

        // Toggle
        state.process_command(on_off_commands::TOGGLE, &[]).unwrap();
        assert!(!state.on_off);

        // Get attribute
        let value = state.get_attribute(0x0000).unwrap();
        assert_eq!(value, vec![0x00]);
    }

    #[test]
    fn test_level_control_command() {
        let cmd = LevelControlCommand::MoveToLevel {
            level: 128,
            transition_time: 10,
        };
        let bytes = cmd.to_bytes();
        assert_eq!(bytes[0], level_control_commands::MOVE_TO_LEVEL);
        assert_eq!(bytes[1], 128);
    }
}
