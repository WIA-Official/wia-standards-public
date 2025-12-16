//! Exoskeleton robot adapter
//!
//! Provides control and monitoring for lower/upper body exoskeletons.

use crate::error::{RobotError, RobotResult};
use crate::types::{ImuData, Joint, Position3D, Side};
use serde::{Deserialize, Serialize};

/// Exoskeleton type
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ExoskeletonType {
    LowerBody,
    UpperBody,
    FullBody,
}

/// Control mode for exoskeleton
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum ExoControlMode {
    #[default]
    Assist,
    Resist,
    Passive,
    Transparent,
}

/// Gait phase
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum GaitPhase {
    #[default]
    Stance,
    Swing,
    DoubleSupport,
    HeelStrike,
    ToeOff,
    MidStance,
}

/// Gait data
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct GaitData {
    pub phase: GaitPhase,
    pub side: Option<Side>,
    pub cycle_percent: f64,
    pub step_count: u32,
    pub cadence_steps_min: f64,
    pub stride_length_cm: f64,
    pub step_width_cm: f64,
    pub velocity_m_s: f64,
    pub symmetry_index: f64,
}

/// Impedance control parameters
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ImpedanceParams {
    pub stiffness: f64,
    pub damping: f64,
    pub inertia: Option<f64>,
    pub equilibrium_angle: f64,
}

impl Default for ImpedanceParams {
    fn default() -> Self {
        Self {
            stiffness: 50.0,
            damping: 5.0,
            inertia: None,
            equilibrium_angle: 0.0,
        }
    }
}

/// Exoskeleton specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExoskeletonSpec {
    pub exo_type: ExoskeletonType,
    pub joints: Vec<Joint>,
    pub gait: GaitData,
    pub imu: ImuData,
    pub control_mode: ExoControlMode,
    pub assist_level: f64,
    pub impedance: Option<ImpedanceParams>,
}

impl ExoskeletonSpec {
    /// Create a new lower body exoskeleton with default joints
    pub fn new_lower_body() -> Self {
        Self {
            exo_type: ExoskeletonType::LowerBody,
            joints: vec![
                Joint::new("hip_left", -20.0, 100.0),
                Joint::new("hip_right", -20.0, 100.0),
                Joint::new("knee_left", 0.0, 120.0),
                Joint::new("knee_right", 0.0, 120.0),
                Joint::new("ankle_left", -20.0, 30.0),
                Joint::new("ankle_right", -20.0, 30.0),
            ],
            gait: GaitData::default(),
            imu: ImuData::default(),
            control_mode: ExoControlMode::Assist,
            assist_level: 0.5,
            impedance: Some(ImpedanceParams::default()),
        }
    }

    /// Calculate gait cycle duration in seconds
    pub fn gait_cycle_duration(&self) -> f64 {
        if self.gait.cadence_steps_min <= 0.0 {
            return 0.0;
        }
        60.0 / self.gait.cadence_steps_min
    }

    /// Calculate walking speed in m/s
    pub fn walking_speed_m_s(&self) -> f64 {
        let stride_m = self.gait.stride_length_cm / 100.0;
        let cadence_hz = self.gait.cadence_steps_min / 60.0;
        stride_m * cadence_hz
    }

    /// Calculate assistance torque for a joint
    pub fn calculate_assist_torque(&self, joint_name: &str) -> RobotResult<f64> {
        let joint = self
            .joints
            .iter()
            .find(|j| j.name == joint_name)
            .ok_or_else(|| {
                RobotError::InvalidParameter(format!("Joint not found: {}", joint_name))
            })?;

        let base_torque = joint.torque_nm;
        let assist_torque = base_torque * self.assist_level;
        Ok(assist_torque)
    }

    /// Detect fall risk based on IMU data
    pub fn detect_fall_risk(&self) -> bool {
        const PITCH_THRESHOLD: f64 = 30.0;
        const ROLL_THRESHOLD: f64 = 25.0;

        self.imu.orientation.pitch.abs() > PITCH_THRESHOLD
            || self.imu.orientation.roll.abs() > ROLL_THRESHOLD
    }

    /// Calculate total walking distance based on step count and stride length
    pub fn total_distance_m(&self) -> f64 {
        (self.gait.step_count as f64 * self.gait.stride_length_cm) / 100.0
    }

    /// Get joint by name
    pub fn get_joint(&self, name: &str) -> Option<&Joint> {
        self.joints.iter().find(|j| j.name == name)
    }

    /// Get mutable joint by name
    pub fn get_joint_mut(&mut self, name: &str) -> Option<&mut Joint> {
        self.joints.iter_mut().find(|j| j.name == name)
    }

    /// Set target angle for a joint
    pub fn set_joint_target(&mut self, name: &str, target_deg: f64) -> RobotResult<()> {
        let joint = self.get_joint_mut(name).ok_or_else(|| {
            RobotError::InvalidParameter(format!("Joint not found: {}", name))
        })?;

        if target_deg < joint.min_angle_deg || target_deg > joint.max_angle_deg {
            return Err(RobotError::SafetyViolation(format!(
                "Target angle {:.1} outside limits [{:.1}, {:.1}]",
                target_deg, joint.min_angle_deg, joint.max_angle_deg
            )));
        }

        joint.target_angle_deg = target_deg;
        Ok(())
    }

    /// Update gait phase based on sensor data
    pub fn update_gait_phase(&mut self, left_foot_force: f64, right_foot_force: f64) {
        const FORCE_THRESHOLD: f64 = 50.0;

        let left_stance = left_foot_force > FORCE_THRESHOLD;
        let right_stance = right_foot_force > FORCE_THRESHOLD;

        self.gait.phase = match (left_stance, right_stance) {
            (true, true) => GaitPhase::DoubleSupport,
            (true, false) => {
                self.gait.side = Some(Side::Left);
                GaitPhase::Stance
            }
            (false, true) => {
                self.gait.side = Some(Side::Right);
                GaitPhase::Stance
            }
            (false, false) => GaitPhase::Swing,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_lower_body() {
        let exo = ExoskeletonSpec::new_lower_body();
        assert_eq!(exo.exo_type, ExoskeletonType::LowerBody);
        assert_eq!(exo.joints.len(), 6);
    }

    #[test]
    fn test_gait_cycle_duration() {
        let mut exo = ExoskeletonSpec::new_lower_body();
        exo.gait.cadence_steps_min = 60.0;
        assert!((exo.gait_cycle_duration() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_walking_speed() {
        let mut exo = ExoskeletonSpec::new_lower_body();
        exo.gait.cadence_steps_min = 60.0;
        exo.gait.stride_length_cm = 100.0;
        assert!((exo.walking_speed_m_s() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_fall_risk_detection() {
        let mut exo = ExoskeletonSpec::new_lower_body();
        assert!(!exo.detect_fall_risk());

        exo.imu.orientation.pitch = 35.0;
        assert!(exo.detect_fall_risk());
    }

    #[test]
    fn test_set_joint_target() {
        let mut exo = ExoskeletonSpec::new_lower_body();
        assert!(exo.set_joint_target("knee_left", 45.0).is_ok());
        assert!(exo.set_joint_target("knee_left", 150.0).is_err());
        assert!(exo.set_joint_target("invalid", 0.0).is_err());
    }
}
