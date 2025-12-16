//! Eye Gaze Gaming Integration
//!
//! Provides gaze-based game controls including aiming, camera control, and UI navigation.

use serde::{Deserialize, Serialize};
use std::time::{Duration, Instant};
use uuid::Uuid;

/// Eye gaze adapter for gaming
#[derive(Debug)]
pub struct EyeGazeAdapter {
    config: EyeGazeConfig,
    current_gaze: Option<GazePoint>,
    dwell_state: DwellState,
    calibration_quality: f32,
    last_update: Option<Instant>,
    simulated_gaze: Option<(f32, f32)>,
}

/// Configuration for eye gaze gaming
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EyeGazeConfig {
    /// Enable gaze-based aiming
    pub aiming_enabled: bool,
    /// Aiming mode
    pub aiming_mode: GazeAimingMode,
    /// Sensitivity (0.0-1.0)
    pub sensitivity: f32,
    /// Smoothing factor (0.0-1.0)
    pub smoothing: f32,
    /// Dead zone radius (normalized)
    pub dead_zone_radius: f32,
    /// Aim magnetism settings
    pub aim_magnetism: AimMagnetism,
    /// Dwell selection settings
    pub dwell_select: DwellSelectConfig,
    /// Camera control settings
    pub camera_control: CameraControlConfig,
}

/// Gaze-based aiming modes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GazeAimingMode {
    /// Direct aim - crosshair follows gaze
    Direct,
    /// Aim assist - gaze influences aim magnetism
    AimAssist,
    /// Hybrid - direct aim with magnetism boost
    Hybrid,
    /// Disabled
    Disabled,
}

/// Aim magnetism configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AimMagnetism {
    pub enabled: bool,
    /// Magnetism strength (0.0-1.0)
    pub strength: f32,
    /// Target priority order
    pub target_priority: Vec<TargetType>,
    /// Maximum magnetism range (game units)
    pub max_range: f32,
}

/// Types of targets for aim magnetism
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TargetType {
    Enemy,
    Interactable,
    Ally,
    Neutral,
    Objective,
}

/// Dwell selection configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DwellSelectConfig {
    pub enabled: bool,
    /// Time to dwell before selection (ms)
    pub duration_ms: u32,
    /// Visual feedback type
    pub visual_feedback: DwellFeedback,
    /// Require confirmation after dwell
    pub require_confirmation: bool,
}

/// Visual feedback for dwell selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DwellFeedback {
    ShrinkingCircle,
    FillingCircle,
    ProgressBar,
    ColorChange,
    None,
}

/// Camera control configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraControlConfig {
    pub enabled: bool,
    /// Edge-based camera rotation
    pub edge_rotation: bool,
    /// Edge threshold (0.0-0.5, distance from edge)
    pub edge_threshold: f32,
    /// Rotation speed
    pub rotation_speed: f32,
}

/// Current gaze point data
#[derive(Debug, Clone)]
pub struct GazePoint {
    /// Normalized X coordinate (0.0-1.0)
    pub x: f32,
    /// Normalized Y coordinate (0.0-1.0)
    pub y: f32,
    /// Gaze validity (0.0-1.0)
    pub validity: f32,
    /// Timestamp
    pub timestamp: Instant,
}

/// Dwell state tracking
#[derive(Debug)]
struct DwellState {
    target_point: Option<(f32, f32)>,
    start_time: Option<Instant>,
    accumulated_ms: u32,
}

/// Input data from eye gaze
#[derive(Debug, Clone)]
pub struct EyeGazeInput {
    /// Current gaze point
    pub gaze_point: (f32, f32),
    /// Calculated aim vector
    pub aim_vector: (f32, f32),
    /// Current dwell target entity
    pub dwell_target: Option<Uuid>,
    /// Dwell progress (0.0-1.0)
    pub dwell_progress: f32,
    /// Blink detected this frame
    pub blink_detected: bool,
    /// Calibration quality
    pub calibration_quality: f32,
}

/// Gaze action for game input
#[derive(Debug, Clone)]
pub enum GazeAction {
    /// Aim at a position
    AimAt { x: f32, y: f32, magnetism: f32 },
    /// Look/rotate camera
    LookAt { delta_x: f32, delta_y: f32 },
    /// Dwell selection completed
    DwellSelect { x: f32, y: f32 },
    /// Quick glance at UI element
    QuickGlance { element: String },
    /// Blink action
    Blink { action: BlinkAction },
}

/// Blink-based actions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlinkAction {
    Confirm,
    Cancel,
    Fire,
    Interact,
}

impl EyeGazeAdapter {
    /// Create a new eye gaze adapter
    pub fn new() -> Self {
        Self {
            config: EyeGazeConfig::default(),
            current_gaze: None,
            dwell_state: DwellState {
                target_point: None,
                start_time: None,
                accumulated_ms: 0,
            },
            calibration_quality: 0.0,
            last_update: None,
            simulated_gaze: None,
        }
    }

    /// Configure the adapter
    pub fn configure(&mut self, config: EyeGazeConfig) {
        self.config = config;
    }

    /// Set simulated gaze point for testing
    pub fn set_simulated_gaze(&mut self, x: f32, y: f32) {
        self.simulated_gaze = Some((x, y));
        self.calibration_quality = 1.0;
    }

    /// Update gaze data from eye tracker
    pub fn update(&mut self, x: f32, y: f32, validity: f32) {
        let now = Instant::now();

        // Apply smoothing if we have previous data
        let (smoothed_x, smoothed_y) = if let Some(ref prev) = self.current_gaze {
            let s = self.config.smoothing;
            (
                prev.x * s + x * (1.0 - s),
                prev.y * s + y * (1.0 - s),
            )
        } else {
            (x, y)
        };

        self.current_gaze = Some(GazePoint {
            x: smoothed_x,
            y: smoothed_y,
            validity,
            timestamp: now,
        });

        self.last_update = Some(now);
        self.update_dwell(smoothed_x, smoothed_y);
    }

    /// Update dwell state
    fn update_dwell(&mut self, x: f32, y: f32) {
        if !self.config.dwell_select.enabled {
            return;
        }

        let dwell_threshold = 0.05; // 5% screen distance

        if let Some((tx, ty)) = self.dwell_state.target_point {
            let dist = ((x - tx).powi(2) + (y - ty).powi(2)).sqrt();

            if dist < dwell_threshold {
                // Still on target, accumulate time
                if let Some(start) = self.dwell_state.start_time {
                    self.dwell_state.accumulated_ms = start.elapsed().as_millis() as u32;
                }
            } else {
                // Moved off target, reset
                self.dwell_state.target_point = Some((x, y));
                self.dwell_state.start_time = Some(Instant::now());
                self.dwell_state.accumulated_ms = 0;
            }
        } else {
            // First gaze point
            self.dwell_state.target_point = Some((x, y));
            self.dwell_state.start_time = Some(Instant::now());
            self.dwell_state.accumulated_ms = 0;
        }
    }

    /// Check if dwell selection is complete
    pub fn is_dwell_complete(&self) -> bool {
        self.dwell_state.accumulated_ms >= self.config.dwell_select.duration_ms
    }

    /// Get dwell progress (0.0-1.0)
    pub fn get_dwell_progress(&self) -> f32 {
        if self.config.dwell_select.duration_ms == 0 {
            return 0.0;
        }
        (self.dwell_state.accumulated_ms as f32 / self.config.dwell_select.duration_ms as f32)
            .min(1.0)
    }

    /// Poll for input
    pub fn poll(&mut self) -> Option<EyeGazeInput> {
        // Use simulated gaze if set
        if let Some((x, y)) = self.simulated_gaze {
            self.update(x, y, 1.0);
        }

        let gaze = self.current_gaze.as_ref()?;

        // Check if data is stale
        if let Some(last) = self.last_update {
            if last.elapsed() > Duration::from_millis(100) {
                return None;
            }
        }

        // Calculate aim vector (centered, -1 to 1)
        let aim_x = (gaze.x - 0.5) * 2.0 * self.config.sensitivity;
        let aim_y = (gaze.y - 0.5) * 2.0 * self.config.sensitivity;

        Some(EyeGazeInput {
            gaze_point: (gaze.x, gaze.y),
            aim_vector: (aim_x, aim_y),
            dwell_target: None, // Would need game entity lookup
            dwell_progress: self.get_dwell_progress(),
            blink_detected: false, // Would need blink detection
            calibration_quality: self.calibration_quality,
        })
    }

    /// Process gaze into game action
    pub fn get_action(&self) -> Option<GazeAction> {
        let gaze = self.current_gaze.as_ref()?;

        // Check for dwell selection
        if self.is_dwell_complete() {
            return Some(GazeAction::DwellSelect {
                x: gaze.x,
                y: gaze.y,
            });
        }

        // Check for edge-based camera rotation
        if self.config.camera_control.enabled && self.config.camera_control.edge_rotation {
            let threshold = self.config.camera_control.edge_threshold;
            let speed = self.config.camera_control.rotation_speed;

            let mut delta_x = 0.0;
            let mut delta_y = 0.0;

            if gaze.x < threshold {
                delta_x = -(threshold - gaze.x) / threshold * speed;
            } else if gaze.x > 1.0 - threshold {
                delta_x = (gaze.x - (1.0 - threshold)) / threshold * speed;
            }

            if gaze.y < threshold {
                delta_y = -(threshold - gaze.y) / threshold * speed;
            } else if gaze.y > 1.0 - threshold {
                delta_y = (gaze.y - (1.0 - threshold)) / threshold * speed;
            }

            if delta_x != 0.0 || delta_y != 0.0 {
                return Some(GazeAction::LookAt { delta_x, delta_y });
            }
        }

        // Default: aim at gaze point
        if self.config.aiming_enabled && self.config.aiming_mode != GazeAimingMode::Disabled {
            let magnetism = if self.config.aim_magnetism.enabled {
                self.config.aim_magnetism.strength
            } else {
                0.0
            };

            return Some(GazeAction::AimAt {
                x: gaze.x,
                y: gaze.y,
                magnetism,
            });
        }

        None
    }
}

impl Default for EyeGazeAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for EyeGazeConfig {
    fn default() -> Self {
        Self {
            aiming_enabled: true,
            aiming_mode: GazeAimingMode::AimAssist,
            sensitivity: 0.7,
            smoothing: 0.3,
            dead_zone_radius: 0.05,
            aim_magnetism: AimMagnetism {
                enabled: true,
                strength: 0.8,
                target_priority: vec![TargetType::Enemy, TargetType::Interactable],
                max_range: 100.0,
            },
            dwell_select: DwellSelectConfig {
                enabled: true,
                duration_ms: 800,
                visual_feedback: DwellFeedback::ShrinkingCircle,
                require_confirmation: false,
            },
            camera_control: CameraControlConfig {
                enabled: true,
                edge_rotation: true,
                edge_threshold: 0.1,
                rotation_speed: 1.0,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_eye_gaze_adapter_creation() {
        let adapter = EyeGazeAdapter::new();
        assert!(adapter.current_gaze.is_none());
    }

    #[test]
    fn test_gaze_update() {
        let mut adapter = EyeGazeAdapter::new();
        adapter.update(0.5, 0.5, 1.0);

        assert!(adapter.current_gaze.is_some());
        let gaze = adapter.current_gaze.as_ref().unwrap();
        assert!((gaze.x - 0.5).abs() < 0.01);
        assert!((gaze.y - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_gaze_smoothing() {
        let mut adapter = EyeGazeAdapter::new();
        adapter.config.smoothing = 0.5;

        adapter.update(0.0, 0.0, 1.0);
        adapter.update(1.0, 1.0, 1.0);

        let gaze = adapter.current_gaze.as_ref().unwrap();
        // Should be smoothed between 0 and 1
        assert!(gaze.x > 0.4 && gaze.x < 0.6);
    }

    #[test]
    fn test_dwell_selection() {
        let mut adapter = EyeGazeAdapter::new();
        adapter.config.dwell_select.duration_ms = 100;

        // Look at same point
        adapter.update(0.5, 0.5, 1.0);
        std::thread::sleep(std::time::Duration::from_millis(150));
        adapter.update(0.5, 0.5, 1.0);

        assert!(adapter.is_dwell_complete());
    }

    #[test]
    fn test_poll_input() {
        let mut adapter = EyeGazeAdapter::new();
        adapter.set_simulated_gaze(0.5, 0.5);

        let input = adapter.poll();
        assert!(input.is_some());

        let input = input.unwrap();
        assert!((input.gaze_point.0 - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_camera_edge_rotation() {
        let mut adapter = EyeGazeAdapter::new();
        adapter.config.camera_control.enabled = true;
        adapter.config.camera_control.edge_threshold = 0.1;

        // Look at left edge
        adapter.update(0.05, 0.5, 1.0);

        let action = adapter.get_action();
        assert!(matches!(action, Some(GazeAction::LookAt { delta_x, .. }) if delta_x < 0.0));
    }
}
