//! Safety system for WIA Robot SDK
//!
//! This module provides safety validation and monitoring functionality.

use crate::error::{RobotError, RobotResult};
use serde::{Deserialize, Serialize};

/// Safety status of the robot
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct SafetyStatus {
    /// Emergency stop activated
    pub emergency_stop: bool,
    /// Emergency stop source
    pub emergency_stop_source: Option<EStopSource>,
    /// Fall detected
    pub fall_detection: bool,
    /// Collision avoidance enabled
    pub collision_avoidance: bool,
    /// Force within limits
    pub force_limit_ok: bool,
    /// Within workspace boundaries
    pub workspace_boundary_ok: bool,
    /// User vital signs normal
    pub vital_signs_ok: bool,
    /// Overall safety score (0-100)
    pub safety_score: u8,
}

/// Emergency stop source
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum EStopSource {
    /// User pressed button
    UserButton,
    /// Remote command
    Remote,
    /// Software triggered
    Software,
    /// Overload detected
    Overload,
    /// Fall detected
    Fall,
    /// Obstacle detected
    Obstacle,
}

impl SafetyStatus {
    /// Create a new safe status (all checks passed)
    pub fn new_safe() -> Self {
        Self {
            emergency_stop: false,
            emergency_stop_source: None,
            fall_detection: false,
            collision_avoidance: true,
            force_limit_ok: true,
            workspace_boundary_ok: true,
            vital_signs_ok: true,
            safety_score: 100,
        }
    }

    /// Check if the robot is safe to operate
    pub fn is_safe(&self) -> bool {
        !self.emergency_stop
            && !self.fall_detection
            && self.force_limit_ok
            && self.workspace_boundary_ok
            && self.vital_signs_ok
    }

    /// Validate safety status and return error if unsafe
    pub fn validate(&self) -> RobotResult<()> {
        if self.emergency_stop {
            return Err(RobotError::EmergencyStop);
        }

        if self.fall_detection {
            return Err(RobotError::SafetyViolation(
                "Fall detected - operation halted".into(),
            ));
        }

        if !self.vital_signs_ok {
            return Err(RobotError::SafetyViolation(
                "Vital signs abnormal".into(),
            ));
        }

        if !self.workspace_boundary_ok {
            return Err(RobotError::SafetyViolation(
                "Workspace boundary exceeded".into(),
            ));
        }

        if !self.force_limit_ok {
            return Err(RobotError::SafetyViolation(
                "Force limit exceeded".into(),
            ));
        }

        Ok(())
    }

    /// Trigger emergency stop
    pub fn trigger_estop(&mut self, source: EStopSource) {
        self.emergency_stop = true;
        self.emergency_stop_source = Some(source);
        self.safety_score = 0;
    }

    /// Reset emergency stop (requires explicit action)
    pub fn reset_estop(&mut self) -> RobotResult<()> {
        if self.fall_detection {
            return Err(RobotError::SafetyViolation(
                "Cannot reset E-Stop while fall is detected".into(),
            ));
        }

        self.emergency_stop = false;
        self.emergency_stop_source = None;
        self.recalculate_safety_score();
        Ok(())
    }

    /// Recalculate safety score based on current status
    pub fn recalculate_safety_score(&mut self) {
        let mut score = 100u8;

        if self.emergency_stop {
            score = 0;
        } else {
            if self.fall_detection {
                score = score.saturating_sub(50);
            }
            if !self.force_limit_ok {
                score = score.saturating_sub(30);
            }
            if !self.workspace_boundary_ok {
                score = score.saturating_sub(20);
            }
            if !self.vital_signs_ok {
                score = score.saturating_sub(25);
            }
            if !self.collision_avoidance {
                score = score.saturating_sub(10);
            }
        }

        self.safety_score = score;
    }
}

/// Safety constraints for robot operation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetyConstraints {
    /// Maximum velocity (m/s)
    pub max_velocity_m_s: f64,
    /// Maximum acceleration (m/s^2)
    pub max_acceleration_m_s2: f64,
    /// Maximum force (N)
    pub max_force_n: f64,
    /// Maximum torque (Nm)
    pub max_torque_nm: f64,
    /// Workspace limits
    pub workspace_limits: WorkspaceLimits,
}

impl Default for SafetyConstraints {
    fn default() -> Self {
        Self {
            max_velocity_m_s: 1.5,
            max_acceleration_m_s2: 2.0,
            max_force_n: 100.0,
            max_torque_nm: 50.0,
            workspace_limits: WorkspaceLimits::default(),
        }
    }
}

impl SafetyConstraints {
    /// Check if velocity is within limits
    pub fn check_velocity(&self, velocity: f64) -> RobotResult<()> {
        if velocity.abs() > self.max_velocity_m_s {
            return Err(RobotError::SafetyViolation(format!(
                "Velocity {:.2} m/s exceeds limit {:.2} m/s",
                velocity.abs(),
                self.max_velocity_m_s
            )));
        }
        Ok(())
    }

    /// Check if force is within limits
    pub fn check_force(&self, force: f64) -> RobotResult<()> {
        if force.abs() > self.max_force_n {
            return Err(RobotError::SafetyViolation(format!(
                "Force {:.2} N exceeds limit {:.2} N",
                force.abs(),
                self.max_force_n
            )));
        }
        Ok(())
    }

    /// Check if torque is within limits
    pub fn check_torque(&self, torque: f64) -> RobotResult<()> {
        if torque.abs() > self.max_torque_nm {
            return Err(RobotError::SafetyViolation(format!(
                "Torque {:.2} Nm exceeds limit {:.2} Nm",
                torque.abs(),
                self.max_torque_nm
            )));
        }
        Ok(())
    }
}

/// Workspace limits in 3D space
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorkspaceLimits {
    pub min_x: f64,
    pub max_x: f64,
    pub min_y: f64,
    pub max_y: f64,
    pub min_z: f64,
    pub max_z: f64,
}

impl Default for WorkspaceLimits {
    fn default() -> Self {
        Self {
            min_x: -1.0,
            max_x: 1.0,
            min_y: -1.0,
            max_y: 1.0,
            min_z: 0.0,
            max_z: 2.0,
        }
    }
}

impl WorkspaceLimits {
    /// Check if a point is within the workspace
    pub fn contains(&self, x: f64, y: f64, z: f64) -> bool {
        x >= self.min_x
            && x <= self.max_x
            && y >= self.min_y
            && y <= self.max_y
            && z >= self.min_z
            && z <= self.max_z
    }

    /// Check if position is within workspace, return error if not
    pub fn validate_position(&self, x: f64, y: f64, z: f64) -> RobotResult<()> {
        if !self.contains(x, y, z) {
            return Err(RobotError::SafetyViolation(format!(
                "Position ({:.2}, {:.2}, {:.2}) outside workspace bounds",
                x, y, z
            )));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_safety_status_new_safe() {
        let status = SafetyStatus::new_safe();
        assert!(status.is_safe());
        assert_eq!(status.safety_score, 100);
    }

    #[test]
    fn test_safety_status_estop() {
        let mut status = SafetyStatus::new_safe();
        status.trigger_estop(EStopSource::UserButton);

        assert!(!status.is_safe());
        assert!(status.emergency_stop);
        assert_eq!(status.emergency_stop_source, Some(EStopSource::UserButton));
        assert_eq!(status.safety_score, 0);
    }

    #[test]
    fn test_safety_status_validate() {
        let status = SafetyStatus::new_safe();
        assert!(status.validate().is_ok());

        let mut unsafe_status = SafetyStatus::new_safe();
        unsafe_status.emergency_stop = true;
        assert!(unsafe_status.validate().is_err());
    }

    #[test]
    fn test_workspace_limits_contains() {
        let limits = WorkspaceLimits::default();
        assert!(limits.contains(0.0, 0.0, 1.0));
        assert!(!limits.contains(2.0, 0.0, 1.0));
    }

    #[test]
    fn test_safety_constraints_check_velocity() {
        let constraints = SafetyConstraints::default();
        assert!(constraints.check_velocity(1.0).is_ok());
        assert!(constraints.check_velocity(2.0).is_err());
    }
}
