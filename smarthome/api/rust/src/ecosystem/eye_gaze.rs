//! # Eye Gaze Smart Home Integration
//!
//! Enables gaze-based control of smart home devices.

use super::*;
use crate::error::{Result, SmartHomeError};
use crate::types::DeviceId;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Gaze point on screen (normalized 0.0-1.0)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GazePoint {
    pub x: f32,
    pub y: f32,
    pub timestamp_ms: u64,
}

impl GazePoint {
    pub fn new(x: f32, y: f32) -> Self {
        Self {
            x: x.clamp(0.0, 1.0),
            y: y.clamp(0.0, 1.0),
            timestamp_ms: 0,
        }
    }

    pub fn with_timestamp(mut self, timestamp_ms: u64) -> Self {
        self.timestamp_ms = timestamp_ms;
        self
    }

    /// Calculate distance to another point
    pub fn distance_to(&self, other: &GazePoint) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}

/// Gaze interaction patterns
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GazePattern {
    /// Look at target for duration to select
    DwellSelect,
    /// Gaze to select, blink to confirm
    GazeBlink,
    /// Gaze to select, external switch to confirm
    GazeSwitch,
    /// Follow moving target to control value
    SmoothPursuit,
}

impl Default for GazePattern {
    fn default() -> Self {
        Self::DwellSelect
    }
}

/// Screen zone for device mapping
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScreenZone {
    pub id: String,
    pub device_id: DeviceId,
    pub bounds: ZoneBounds,
    pub label: String,
    pub icon: Option<String>,
}

/// Zone boundaries (normalized 0.0-1.0)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ZoneBounds {
    pub x_min: f32,
    pub y_min: f32,
    pub x_max: f32,
    pub y_max: f32,
}

impl ZoneBounds {
    pub fn new(x_min: f32, y_min: f32, x_max: f32, y_max: f32) -> Self {
        Self {
            x_min,
            y_min,
            x_max,
            y_max,
        }
    }

    pub fn contains(&self, point: &GazePoint) -> bool {
        point.x >= self.x_min
            && point.x <= self.x_max
            && point.y >= self.y_min
            && point.y <= self.y_max
    }

    pub fn center(&self) -> GazePoint {
        GazePoint::new(
            (self.x_min + self.x_max) / 2.0,
            (self.y_min + self.y_max) / 2.0,
        )
    }
}

/// Eye gaze adapter for smart home control
#[derive(Debug)]
pub struct EyeGazeAdapter {
    config: EyeGazeConfig,
    zones: Vec<ScreenZone>,
    dwell_tracker: DwellTracker,
    calibration: CalibrationState,
}

/// Eye gaze configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EyeGazeConfig {
    /// Dwell time to activate (ms)
    pub dwell_time_ms: u32,
    /// Dwell tolerance radius
    pub dwell_tolerance: f32,
    /// Default interaction pattern
    pub default_pattern: GazePattern,
    /// Enable smooth pursuit for sliders
    pub smooth_pursuit_enabled: bool,
    /// Gaze smoothing factor (0.0-1.0)
    pub smoothing_factor: f32,
}

impl Default for EyeGazeConfig {
    fn default() -> Self {
        Self {
            dwell_time_ms: 1000,
            dwell_tolerance: 0.05,
            default_pattern: GazePattern::DwellSelect,
            smooth_pursuit_enabled: true,
            smoothing_factor: 0.3,
        }
    }
}

/// Tracks dwell state for gaze selection
#[derive(Debug, Default)]
struct DwellTracker {
    current_zone: Option<String>,
    dwell_start_ms: u64,
    accumulated_dwell_ms: u64,
    last_gaze: Option<GazePoint>,
}

/// Calibration state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationState {
    pub is_calibrated: bool,
    pub quality: CalibrationQuality,
    pub points_collected: u32,
    pub last_calibration_ms: u64,
}

impl Default for CalibrationState {
    fn default() -> Self {
        Self {
            is_calibrated: false,
            quality: CalibrationQuality::NotCalibrated,
            points_collected: 0,
            last_calibration_ms: 0,
        }
    }
}

/// Calibration quality level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CalibrationQuality {
    NotCalibrated,
    Poor,
    Fair,
    Good,
    Excellent,
}

impl EyeGazeAdapter {
    pub fn new() -> Self {
        Self {
            config: EyeGazeConfig::default(),
            zones: Vec::new(),
            dwell_tracker: DwellTracker::default(),
            calibration: CalibrationState::default(),
        }
    }

    pub fn with_config(mut self, config: EyeGazeConfig) -> Self {
        self.config = config;
        self
    }

    /// Add a screen zone for device mapping
    pub fn add_zone(&mut self, zone: ScreenZone) {
        self.zones.push(zone);
    }

    /// Create a grid layout of zones
    pub fn create_grid_layout(&mut self, rows: u32, cols: u32, devices: Vec<(DeviceId, String)>) {
        self.zones.clear();

        let zone_width = 1.0 / cols as f32;
        let zone_height = 1.0 / rows as f32;

        for (idx, (device_id, label)) in devices.into_iter().enumerate() {
            let row = idx as u32 / cols;
            let col = idx as u32 % cols;

            if row >= rows {
                break;
            }

            let bounds = ZoneBounds::new(
                col as f32 * zone_width,
                row as f32 * zone_height,
                (col + 1) as f32 * zone_width,
                (row + 1) as f32 * zone_height,
            );

            self.zones.push(ScreenZone {
                id: format!("zone_{}_{}", row, col),
                device_id,
                bounds,
                label,
                icon: None,
            });
        }
    }

    /// Process gaze point and check for selection
    pub fn process_gaze(&mut self, gaze: GazePoint, current_time_ms: u64) -> Option<GazeSelection> {
        // Find zone under gaze
        let zone = self.zones.iter().find(|z| z.bounds.contains(&gaze));

        match zone {
            Some(z) => {
                let zone_id = z.id.clone();
                let device_id = z.device_id;

                // Check if same zone as before
                if self.dwell_tracker.current_zone.as_ref() == Some(&zone_id) {
                    // Check if gaze is stable (within tolerance)
                    let is_stable = self
                        .dwell_tracker
                        .last_gaze
                        .map(|last| gaze.distance_to(&last) < self.config.dwell_tolerance)
                        .unwrap_or(true);

                    if is_stable {
                        // Accumulate dwell time
                        let elapsed =
                            current_time_ms.saturating_sub(self.dwell_tracker.dwell_start_ms);

                        if elapsed >= self.config.dwell_time_ms as u64 {
                            // Selection complete!
                            self.dwell_tracker.current_zone = None;
                            self.dwell_tracker.accumulated_dwell_ms = 0;

                            return Some(GazeSelection {
                                zone_id,
                                device_id,
                                gaze_point: gaze,
                                dwell_ms: elapsed as u32,
                                pattern: self.config.default_pattern,
                            });
                        }
                    } else {
                        // Gaze moved, reset dwell
                        self.dwell_tracker.dwell_start_ms = current_time_ms;
                    }
                } else {
                    // New zone, start tracking
                    self.dwell_tracker.current_zone = Some(zone_id);
                    self.dwell_tracker.dwell_start_ms = current_time_ms;
                    self.dwell_tracker.accumulated_dwell_ms = 0;
                }

                self.dwell_tracker.last_gaze = Some(gaze);
            }
            None => {
                // No zone under gaze, reset tracker
                self.dwell_tracker.current_zone = None;
                self.dwell_tracker.accumulated_dwell_ms = 0;
            }
        }

        None
    }

    /// Get current dwell progress (0.0-1.0)
    pub fn get_dwell_progress(&self, current_time_ms: u64) -> f32 {
        if self.dwell_tracker.current_zone.is_some() {
            let elapsed = current_time_ms.saturating_sub(self.dwell_tracker.dwell_start_ms);
            (elapsed as f32 / self.config.dwell_time_ms as f32).min(1.0)
        } else {
            0.0
        }
    }

    /// Get zone under current gaze
    pub fn get_current_zone(&self) -> Option<&ScreenZone> {
        self.dwell_tracker
            .current_zone
            .as_ref()
            .and_then(|id| self.zones.iter().find(|z| &z.id == id))
    }

    /// Convert gaze selection to unified command
    pub fn selection_to_command(
        &self,
        selection: GazeSelection,
        action: DeviceAction,
    ) -> UnifiedCommand {
        UnifiedCommand::new(
            CommandSource::EyeGaze {
                gaze_point: selection.gaze_point,
                dwell_ms: selection.dwell_ms,
                pattern: selection.pattern,
            },
            CommandTarget::Device(selection.device_id),
            action,
        )
    }

    /// Get all zones
    pub fn get_zones(&self) -> &[ScreenZone] {
        &self.zones
    }

    /// Set calibration state
    pub fn set_calibration(&mut self, calibration: CalibrationState) {
        self.calibration = calibration;
    }

    /// Check if calibrated
    pub fn is_calibrated(&self) -> bool {
        self.calibration.is_calibrated
    }
}

impl Default for EyeGazeAdapter {
    fn default() -> Self {
        Self::new()
    }
}

/// Gaze selection result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GazeSelection {
    pub zone_id: String,
    pub device_id: DeviceId,
    pub gaze_point: GazePoint,
    pub dwell_ms: u32,
    pub pattern: GazePattern,
}

/// Trait for eye gaze smart home control
pub trait EyeGazeSmartHome {
    /// Select device with gaze
    fn gaze_select_device(&self, gaze: GazePoint) -> Option<DeviceId>;
    /// Activate with dwell
    fn dwell_activate(&mut self, device: DeviceId, dwell_ms: u32) -> Result<()>;
    /// Scroll with gaze direction
    fn gaze_scroll(&self, direction: ScrollDirection) -> Result<()>;
}

/// Scroll direction for gaze control
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScrollDirection {
    Up,
    Down,
    Left,
    Right,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gaze_point() {
        let p1 = GazePoint::new(0.5, 0.5);
        let p2 = GazePoint::new(0.6, 0.5);

        let dist = p1.distance_to(&p2);
        assert!((dist - 0.1).abs() < 0.001);
    }

    #[test]
    fn test_zone_bounds() {
        let bounds = ZoneBounds::new(0.0, 0.0, 0.5, 0.5);

        assert!(bounds.contains(&GazePoint::new(0.25, 0.25)));
        assert!(!bounds.contains(&GazePoint::new(0.75, 0.75)));
    }

    #[test]
    fn test_grid_layout() {
        let mut adapter = EyeGazeAdapter::new();

        let devices = vec![
            (Uuid::new_v4(), "Light 1".to_string()),
            (Uuid::new_v4(), "Light 2".to_string()),
            (Uuid::new_v4(), "Fan".to_string()),
            (Uuid::new_v4(), "TV".to_string()),
        ];

        adapter.create_grid_layout(2, 2, devices);

        assert_eq!(adapter.zones.len(), 4);
        assert_eq!(adapter.zones[0].bounds.x_min, 0.0);
        assert_eq!(adapter.zones[0].bounds.x_max, 0.5);
    }

    #[test]
    fn test_dwell_selection() {
        let mut adapter = EyeGazeAdapter::new();
        adapter.config.dwell_time_ms = 100; // Short for testing

        let device_id = Uuid::new_v4();
        adapter.add_zone(ScreenZone {
            id: "test".to_string(),
            device_id,
            bounds: ZoneBounds::new(0.0, 0.0, 0.5, 0.5),
            label: "Test".to_string(),
            icon: None,
        });

        // First gaze - starts tracking
        let gaze = GazePoint::new(0.25, 0.25);
        let result = adapter.process_gaze(gaze, 0);
        assert!(result.is_none());

        // After dwell time - should select
        let result = adapter.process_gaze(gaze, 150);
        assert!(result.is_some());

        let selection = result.unwrap();
        assert_eq!(selection.device_id, device_id);
    }
}
