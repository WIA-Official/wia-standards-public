//! Smart Wheelchair Gaming Integration
//!
//! Provides wheelchair motion as game input and comfort synchronization.

use serde::{Deserialize, Serialize};
use std::time::{Duration, Instant};

/// Wheelchair adapter for gaming
#[derive(Debug)]
pub struct WheelchairAdapter {
    config: WheelchairConfig,
    current_state: WheelchairState,
    comfort_state: ComfortState,
    last_pressure_reminder: Option<Instant>,
    simulated_input: Option<WheelchairInput>,
}

/// Wheelchair configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WheelchairConfig {
    /// Enable tilt as game input
    pub tilt_input_enabled: bool,
    /// Tilt sensitivity (0.0-1.0)
    pub tilt_sensitivity: f32,
    /// Dead zone for tilt
    pub tilt_dead_zone: f32,
    /// Enable motion input
    pub motion_input_enabled: bool,
    /// Comfort sync settings
    pub comfort_sync: ComfortSyncConfig,
    /// Vibration feedback settings
    pub vibration: VibrationConfig,
}

/// Comfort sync configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComfortSyncConfig {
    /// Enable posture check reminders
    pub posture_check_enabled: bool,
    /// Posture check interval (minutes)
    pub posture_check_interval_mins: u32,
    /// Auto-recline during cutscenes
    pub auto_recline_on_cutscene: bool,
    /// Pressure relief reminder
    pub pressure_relief_reminder: PressureReliefConfig,
}

/// Pressure relief reminder settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PressureReliefConfig {
    pub enabled: bool,
    /// Reminder interval (minutes)
    pub interval_mins: u32,
    /// Pause game for relief
    pub pause_game: bool,
    /// Notification type
    pub notification: NotificationType,
}

/// Notification types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NotificationType {
    Overlay,
    Sound,
    Haptic,
    All,
}

/// Vibration feedback settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VibrationConfig {
    pub enabled: bool,
    /// Intensity multiplier (0.0-1.0)
    pub intensity_multiplier: f32,
}

/// Current wheelchair state
#[derive(Debug, Clone, Default)]
pub struct WheelchairState {
    /// Tilt X (-1.0 left to 1.0 right)
    pub tilt_x: f32,
    /// Tilt Y (-1.0 back to 1.0 forward)
    pub tilt_y: f32,
    /// Recline angle (0-45 degrees)
    pub recline_angle: f32,
    /// Current speed
    pub speed: f32,
    /// Is stationary
    pub is_stationary: bool,
    /// Last update time
    pub last_update: Option<Instant>,
}

/// Comfort monitoring state
#[derive(Debug, Clone, Default)]
pub struct ComfortState {
    /// Time in current position
    pub position_duration: Duration,
    /// Pressure distribution quality (0.0-1.0)
    pub pressure_distribution: f32,
    /// Needs posture adjustment
    pub needs_posture_check: bool,
    /// Needs pressure relief
    pub needs_pressure_relief: bool,
}

/// Wheelchair input data
#[derive(Debug, Clone)]
pub struct WheelchairInput {
    /// Tilt values
    pub tilt: (f32, f32),
    /// Recline angle
    pub recline_angle: f32,
    /// Movement speed
    pub speed: f32,
    /// Is stationary
    pub is_stationary: bool,
    /// Quick spin detected
    pub quick_spin: bool,
    /// Comfort alerts
    pub comfort_alerts: Vec<ComfortAlert>,
}

/// Comfort alert types
#[derive(Debug, Clone)]
pub enum ComfortAlert {
    PostureCheck,
    PressureRelief,
    LongSession { duration_mins: u32 },
}

/// Game action from wheelchair
#[derive(Debug, Clone)]
pub enum WheelchairGameAction {
    /// Movement based on tilt
    Move { direction: (f32, f32), speed: f32 },
    /// Camera rotation
    Camera { rotation: (f32, f32) },
    /// Steady aim mode (reclined)
    SteadyAim { active: bool },
    /// Quick turn
    QuickTurn { degrees: f32 },
    /// Comfort reminder
    ComfortReminder { alert: ComfortAlert, pause_game: bool },
}

impl WheelchairAdapter {
    /// Create a new wheelchair adapter
    pub fn new() -> Self {
        Self {
            config: WheelchairConfig::default(),
            current_state: WheelchairState::default(),
            comfort_state: ComfortState::default(),
            last_pressure_reminder: None,
            simulated_input: None,
        }
    }

    /// Configure the adapter
    pub fn configure(&mut self, config: WheelchairConfig) {
        self.config = config;
    }

    /// Update wheelchair state
    pub fn update(&mut self, tilt_x: f32, tilt_y: f32, recline: f32, speed: f32) {
        let now = Instant::now();

        // Update duration tracking
        if let Some(last) = self.current_state.last_update {
            let elapsed = now.duration_since(last);
            self.comfort_state.position_duration += elapsed;
        }

        self.current_state = WheelchairState {
            tilt_x,
            tilt_y,
            recline_angle: recline,
            speed,
            is_stationary: speed < 0.01,
            last_update: Some(now),
        };

        // Check comfort alerts
        self.check_comfort_state();
    }

    /// Check comfort state and generate alerts
    fn check_comfort_state(&mut self) {
        let mins = self.comfort_state.position_duration.as_secs() / 60;

        // Check posture
        if self.config.comfort_sync.posture_check_enabled {
            let interval = self.config.comfort_sync.posture_check_interval_mins as u64;
            self.comfort_state.needs_posture_check = mins >= interval && mins % interval == 0;
        }

        // Check pressure relief
        if self.config.comfort_sync.pressure_relief_reminder.enabled {
            let interval = self.config.comfort_sync.pressure_relief_reminder.interval_mins as u64;
            if mins >= interval {
                if let Some(last) = self.last_pressure_reminder {
                    if last.elapsed().as_secs() / 60 >= interval {
                        self.comfort_state.needs_pressure_relief = true;
                    }
                } else {
                    self.comfort_state.needs_pressure_relief = true;
                }
            }
        }
    }

    /// Set simulated input for testing
    pub fn set_simulated_input(&mut self, input: WheelchairInput) {
        let tilt_x = input.tilt.0;
        let tilt_y = input.tilt.1;
        let recline = input.recline_angle;
        let speed = input.speed;
        self.simulated_input = Some(input);
        self.update(tilt_x, tilt_y, recline, speed);
    }

    /// Apply dead zone to value
    fn apply_dead_zone(&self, value: f32) -> f32 {
        if value.abs() < self.config.tilt_dead_zone {
            0.0
        } else {
            let sign = value.signum();
            let adjusted = (value.abs() - self.config.tilt_dead_zone)
                / (1.0 - self.config.tilt_dead_zone);
            sign * adjusted * self.config.tilt_sensitivity
        }
    }

    /// Detect quick spin gesture
    fn detect_quick_spin(&self) -> Option<f32> {
        // Would need motion history for actual detection
        // Simplified: high rotation speed
        if self.current_state.tilt_x.abs() > 0.8 && !self.current_state.is_stationary {
            Some(if self.current_state.tilt_x > 0.0 { 180.0 } else { -180.0 })
        } else {
            None
        }
    }

    /// Poll for input
    pub fn poll(&mut self) -> Option<WheelchairInput> {
        if let Some(input) = self.simulated_input.take() {
            return Some(input);
        }

        let state = &self.current_state;
        if state.last_update.is_none() {
            return None;
        }

        let mut alerts = Vec::new();
        if self.comfort_state.needs_posture_check {
            alerts.push(ComfortAlert::PostureCheck);
            self.comfort_state.needs_posture_check = false;
        }
        if self.comfort_state.needs_pressure_relief {
            alerts.push(ComfortAlert::PressureRelief);
            self.last_pressure_reminder = Some(Instant::now());
            self.comfort_state.needs_pressure_relief = false;
        }

        Some(WheelchairInput {
            tilt: (state.tilt_x, state.tilt_y),
            recline_angle: state.recline_angle,
            speed: state.speed,
            is_stationary: state.is_stationary,
            quick_spin: self.detect_quick_spin().is_some(),
            comfort_alerts: alerts,
        })
    }

    /// Get game action from current state
    pub fn get_action(&self) -> Option<WheelchairGameAction> {
        let state = &self.current_state;

        // Check for comfort alerts first
        if self.comfort_state.needs_posture_check {
            return Some(WheelchairGameAction::ComfortReminder {
                alert: ComfortAlert::PostureCheck,
                pause_game: false,
            });
        }

        if self.comfort_state.needs_pressure_relief {
            return Some(WheelchairGameAction::ComfortReminder {
                alert: ComfortAlert::PressureRelief,
                pause_game: self.config.comfort_sync.pressure_relief_reminder.pause_game,
            });
        }

        // Check for steady aim (reclined position)
        if state.recline_angle > 20.0 {
            return Some(WheelchairGameAction::SteadyAim { active: true });
        }

        // Check for quick spin
        if let Some(degrees) = self.detect_quick_spin() {
            return Some(WheelchairGameAction::QuickTurn { degrees });
        }

        // Check for tilt input
        if self.config.tilt_input_enabled {
            let x = self.apply_dead_zone(state.tilt_x);
            let y = self.apply_dead_zone(state.tilt_y);

            if x.abs() > 0.0 || y.abs() > 0.0 {
                return Some(WheelchairGameAction::Move {
                    direction: (x, y),
                    speed: state.speed.max(0.5), // Minimum speed
                });
            }
        }

        None
    }

    /// Get current comfort state
    pub fn get_comfort_state(&self) -> &ComfortState {
        &self.comfort_state
    }

    /// Acknowledge pressure relief
    pub fn acknowledge_pressure_relief(&mut self) {
        self.comfort_state.needs_pressure_relief = false;
        self.comfort_state.position_duration = Duration::ZERO;
        self.last_pressure_reminder = Some(Instant::now());
    }
}

impl Default for WheelchairAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for WheelchairConfig {
    fn default() -> Self {
        Self {
            tilt_input_enabled: true,
            tilt_sensitivity: 1.0,
            tilt_dead_zone: 0.1,
            motion_input_enabled: true,
            comfort_sync: ComfortSyncConfig {
                posture_check_enabled: true,
                posture_check_interval_mins: 30,
                auto_recline_on_cutscene: true,
                pressure_relief_reminder: PressureReliefConfig {
                    enabled: true,
                    interval_mins: 45,
                    pause_game: false,
                    notification: NotificationType::Overlay,
                },
            },
            vibration: VibrationConfig {
                enabled: true,
                intensity_multiplier: 0.8,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wheelchair_adapter_creation() {
        let adapter = WheelchairAdapter::new();
        assert!(adapter.current_state.last_update.is_none());
    }

    #[test]
    fn test_tilt_input() {
        let mut adapter = WheelchairAdapter::new();
        adapter.update(0.5, 0.3, 0.0, 0.0);

        let action = adapter.get_action();
        assert!(matches!(action, Some(WheelchairGameAction::Move { .. })));
    }

    #[test]
    fn test_dead_zone() {
        let mut adapter = WheelchairAdapter::new();
        adapter.config.tilt_dead_zone = 0.2;

        // Small tilt within dead zone
        adapter.update(0.1, 0.1, 0.0, 0.0);
        let action = adapter.get_action();
        assert!(action.is_none());

        // Tilt outside dead zone
        adapter.update(0.5, 0.5, 0.0, 0.0);
        let action = adapter.get_action();
        assert!(matches!(action, Some(WheelchairGameAction::Move { .. })));
    }

    #[test]
    fn test_steady_aim_reclined() {
        let mut adapter = WheelchairAdapter::new();
        adapter.update(0.0, 0.0, 25.0, 0.0);

        let action = adapter.get_action();
        assert!(matches!(action, Some(WheelchairGameAction::SteadyAim { active: true })));
    }

    #[test]
    fn test_poll_input() {
        let mut adapter = WheelchairAdapter::new();
        adapter.set_simulated_input(WheelchairInput {
            tilt: (0.3, 0.4),
            recline_angle: 10.0,
            speed: 1.0,
            is_stationary: false,
            quick_spin: false,
            comfort_alerts: Vec::new(),
        });

        let input = adapter.poll();
        assert!(input.is_some());
    }
}
