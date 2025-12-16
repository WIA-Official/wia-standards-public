//! Surgical robot adapter
//!
//! Provides control for surgical assistant robots.

use crate::error::{RobotError, RobotResult};
use crate::types::{Orientation, Position3D};
use serde::{Deserialize, Serialize};

/// Surgical procedure type
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum SurgicalType {
    #[default]
    MinimallyInvasive,
    Orthopedic,
    Neurosurgical,
    Cardiac,
    Urological,
    General,
}

/// Procedure phase
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum ProcedurePhase {
    #[default]
    Setup,
    Docking,
    Operating,
    Undocking,
    Complete,
}

/// Instrument state
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum InstrumentState {
    #[default]
    Inactive,
    Open,
    Closed,
    Active,
}

/// Camera type
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum CameraType {
    #[default]
    Endoscope,
    Microscope,
    External,
}

/// Workspace boundary type
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum BoundaryType {
    #[default]
    Sphere,
    Cylinder,
    Box,
}

/// Surgical instrument
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SurgicalInstrument {
    pub arm_id: u8,
    pub instrument_type: String,
    pub instrument_id: Option<String>,
    pub position: Position3D,
    pub orientation: Orientation,
    pub velocity: Option<Position3D>,
    pub state: InstrumentState,
    pub articulation_deg: f64,
    pub force_n: Option<f64>,
    pub power_w: Option<f64>,
    pub insertion_depth_mm: f64,
    pub usage_count: u32,
}

impl SurgicalInstrument {
    pub fn new(arm_id: u8, instrument_type: &str) -> Self {
        Self {
            arm_id,
            instrument_type: instrument_type.to_string(),
            instrument_id: None,
            position: Position3D::zero(),
            orientation: Orientation::default(),
            velocity: None,
            state: InstrumentState::Inactive,
            articulation_deg: 0.0,
            force_n: None,
            power_w: None,
            insertion_depth_mm: 0.0,
            usage_count: 0,
        }
    }
}

/// Teleoperation data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Teleoperation {
    pub surgeon_console_id: String,
    pub connected: bool,
    pub latency_ms: f64,
    pub motion_scaling: f64,
    pub tremor_filtering: bool,
    pub clutch_engaged: bool,
}

impl Default for Teleoperation {
    fn default() -> Self {
        Self {
            surgeon_console_id: "console-001".to_string(),
            connected: false,
            latency_ms: 0.0,
            motion_scaling: 3.0,
            tremor_filtering: true,
            clutch_engaged: false,
        }
    }
}

/// Camera data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SurgicalCamera {
    pub arm_id: u8,
    pub camera_type: CameraType,
    pub zoom_level: f64,
    pub focus_distance_mm: Option<f64>,
    pub stereo_enabled: bool,
    pub resolution: String,
    pub frame_rate_fps: u32,
    pub field_of_view_deg: f64,
}

impl Default for SurgicalCamera {
    fn default() -> Self {
        Self {
            arm_id: 0,
            camera_type: CameraType::Endoscope,
            zoom_level: 1.0,
            focus_distance_mm: None,
            stereo_enabled: true,
            resolution: "4K".to_string(),
            frame_rate_fps: 60,
            field_of_view_deg: 70.0,
        }
    }
}

/// Workspace boundary
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorkspaceBoundary {
    pub boundary_type: BoundaryType,
    pub center: Position3D,
    pub radius_mm: Option<f64>,
    pub dimensions: Option<Position3D>,
}

impl Default for WorkspaceBoundary {
    fn default() -> Self {
        Self {
            boundary_type: BoundaryType::Sphere,
            center: Position3D::zero(),
            radius_mm: Some(150.0),
            dimensions: None,
        }
    }
}

impl WorkspaceBoundary {
    /// Check if a position is within the workspace
    pub fn contains(&self, pos: &Position3D) -> bool {
        match self.boundary_type {
            BoundaryType::Sphere => {
                if let Some(radius) = self.radius_mm {
                    pos.distance_to(&self.center) <= radius
                } else {
                    true
                }
            }
            BoundaryType::Box => {
                if let Some(dim) = &self.dimensions {
                    (pos.x - self.center.x).abs() <= dim.x / 2.0
                        && (pos.y - self.center.y).abs() <= dim.y / 2.0
                        && (pos.z - self.center.z).abs() <= dim.z / 2.0
                } else {
                    true
                }
            }
            BoundaryType::Cylinder => {
                if let (Some(radius), Some(dim)) = (self.radius_mm, &self.dimensions) {
                    let horizontal_dist = ((pos.x - self.center.x).powi(2)
                        + (pos.y - self.center.y).powi(2))
                    .sqrt();
                    horizontal_dist <= radius && (pos.z - self.center.z).abs() <= dim.z / 2.0
                } else {
                    true
                }
            }
        }
    }
}

/// Surgical safety data
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct SurgicalSafety {
    pub workspace_boundary_ok: bool,
    pub collision_detection: bool,
    pub force_limit_exceeded: bool,
    pub instrument_count_verified: bool,
    pub all_arms_visible: bool,
}

impl SurgicalSafety {
    pub fn is_safe(&self) -> bool {
        self.workspace_boundary_ok
            && !self.force_limit_exceeded
            && self.instrument_count_verified
            && self.all_arms_visible
    }
}

/// Recording data
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Recording {
    pub active: bool,
    pub duration_s: u32,
    pub storage_available_gb: f64,
}

/// Surgical robot specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SurgicalSpec {
    pub surgical_type: SurgicalType,
    pub procedure_phase: ProcedurePhase,
    pub instruments: Vec<SurgicalInstrument>,
    pub teleoperation: Teleoperation,
    pub camera: SurgicalCamera,
    pub workspace: WorkspaceBoundary,
    pub safety: SurgicalSafety,
    pub recording: Recording,
}

impl Default for SurgicalSpec {
    fn default() -> Self {
        Self {
            surgical_type: SurgicalType::MinimallyInvasive,
            procedure_phase: ProcedurePhase::Setup,
            instruments: Vec::new(),
            teleoperation: Teleoperation::default(),
            camera: SurgicalCamera::default(),
            workspace: WorkspaceBoundary::default(),
            safety: SurgicalSafety {
                workspace_boundary_ok: true,
                collision_detection: true,
                force_limit_exceeded: false,
                instrument_count_verified: true,
                all_arms_visible: true,
            },
            recording: Recording::default(),
        }
    }
}

impl SurgicalSpec {
    /// Create a new minimally invasive surgical robot
    pub fn new_minimally_invasive() -> Self {
        let mut spec = Self::default();
        spec.instruments = vec![
            SurgicalInstrument::new(1, "grasper"),
            SurgicalInstrument::new(2, "scissors"),
            SurgicalInstrument::new(3, "cautery"),
            SurgicalInstrument::new(4, "endoscope"),
        ];
        spec
    }

    /// Connect to surgeon console
    pub fn connect_console(&mut self, console_id: &str) -> RobotResult<()> {
        self.teleoperation.surgeon_console_id = console_id.to_string();
        self.teleoperation.connected = true;
        Ok(())
    }

    /// Disconnect from surgeon console
    pub fn disconnect_console(&mut self) {
        self.teleoperation.connected = false;
    }

    /// Move instrument to position with safety check
    pub fn move_instrument(&mut self, arm_id: u8, position: Position3D) -> RobotResult<()> {
        if !self.teleoperation.connected {
            return Err(RobotError::ControlError(
                "Console not connected".into(),
            ));
        }

        if !self.workspace.contains(&position) {
            self.safety.workspace_boundary_ok = false;
            return Err(RobotError::SafetyViolation(
                "Position outside workspace boundary".into(),
            ));
        }

        let instrument = self
            .instruments
            .iter_mut()
            .find(|i| i.arm_id == arm_id)
            .ok_or_else(|| {
                RobotError::InvalidParameter(format!("Instrument not found: arm {}", arm_id))
            })?;

        instrument.position = position;
        self.safety.workspace_boundary_ok = true;
        Ok(())
    }

    /// Set instrument state
    pub fn set_instrument_state(&mut self, arm_id: u8, state: InstrumentState) -> RobotResult<()> {
        let instrument = self
            .instruments
            .iter_mut()
            .find(|i| i.arm_id == arm_id)
            .ok_or_else(|| {
                RobotError::InvalidParameter(format!("Instrument not found: arm {}", arm_id))
            })?;

        instrument.state = state;
        Ok(())
    }

    /// Start recording
    pub fn start_recording(&mut self) -> RobotResult<()> {
        if self.recording.storage_available_gb < 1.0 {
            return Err(RobotError::ControlError(
                "Insufficient storage for recording".into(),
            ));
        }
        self.recording.active = true;
        Ok(())
    }

    /// Stop recording
    pub fn stop_recording(&mut self) {
        self.recording.active = false;
    }

    /// Check if system is ready for operation
    pub fn is_ready_for_operation(&self) -> bool {
        self.teleoperation.connected
            && self.safety.is_safe()
            && self.procedure_phase == ProcedurePhase::Operating
    }

    /// Advance procedure phase
    pub fn advance_phase(&mut self) -> RobotResult<()> {
        self.procedure_phase = match self.procedure_phase {
            ProcedurePhase::Setup => ProcedurePhase::Docking,
            ProcedurePhase::Docking => ProcedurePhase::Operating,
            ProcedurePhase::Operating => ProcedurePhase::Undocking,
            ProcedurePhase::Undocking => ProcedurePhase::Complete,
            ProcedurePhase::Complete => {
                return Err(RobotError::ControlError(
                    "Procedure already complete".into(),
                ))
            }
        };
        Ok(())
    }

    /// Get total latency (network + processing)
    pub fn total_latency_ms(&self) -> f64 {
        self.teleoperation.latency_ms + 2.0 // Add 2ms processing latency
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_minimally_invasive() {
        let spec = SurgicalSpec::new_minimally_invasive();
        assert_eq!(spec.instruments.len(), 4);
        assert_eq!(spec.surgical_type, SurgicalType::MinimallyInvasive);
    }

    #[test]
    fn test_workspace_boundary_sphere() {
        let boundary = WorkspaceBoundary {
            boundary_type: BoundaryType::Sphere,
            center: Position3D::zero(),
            radius_mm: Some(100.0),
            dimensions: None,
        };
        assert!(boundary.contains(&Position3D::new(50.0, 50.0, 0.0)));
        assert!(!boundary.contains(&Position3D::new(100.0, 100.0, 100.0)));
    }

    #[test]
    fn test_connect_console() {
        let mut spec = SurgicalSpec::default();
        assert!(spec.connect_console("console-main").is_ok());
        assert!(spec.teleoperation.connected);
    }

    #[test]
    fn test_move_instrument_safety() {
        let mut spec = SurgicalSpec::new_minimally_invasive();
        spec.teleoperation.connected = true;

        // Within workspace
        assert!(spec.move_instrument(1, Position3D::new(50.0, 50.0, 50.0)).is_ok());

        // Outside workspace
        assert!(spec.move_instrument(1, Position3D::new(500.0, 500.0, 500.0)).is_err());
    }

    #[test]
    fn test_advance_phase() {
        let mut spec = SurgicalSpec::default();
        assert!(spec.advance_phase().is_ok());
        assert_eq!(spec.procedure_phase, ProcedurePhase::Docking);
    }
}
