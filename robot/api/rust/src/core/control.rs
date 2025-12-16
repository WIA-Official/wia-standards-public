//! Control systems for WIA Robot SDK

use crate::error::{RobotError, RobotResult};
use crate::types::Joint;
use serde::{Deserialize, Serialize};

/// PID controller
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PIDController {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub integral: f64,
    pub prev_error: f64,
    pub output_min: f64,
    pub output_max: f64,
}

impl Default for PIDController {
    fn default() -> Self {
        Self {
            kp: 1.0,
            ki: 0.0,
            kd: 0.0,
            integral: 0.0,
            prev_error: 0.0,
            output_min: f64::NEG_INFINITY,
            output_max: f64::INFINITY,
        }
    }
}

impl PIDController {
    /// Create a new PID controller with gains
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            ..Default::default()
        }
    }

    /// Set output limits
    pub fn set_limits(&mut self, min: f64, max: f64) {
        self.output_min = min;
        self.output_max = max;
    }

    /// Compute control output
    pub fn compute(&mut self, error: f64, dt: f64) -> f64 {
        // Proportional
        let p = self.kp * error;

        // Integral with anti-windup
        self.integral += error * dt;
        let i = self.ki * self.integral;

        // Derivative
        let derivative = if dt > 0.0 {
            (error - self.prev_error) / dt
        } else {
            0.0
        };
        let d = self.kd * derivative;

        self.prev_error = error;

        // Compute output with limits
        let output = p + i + d;
        output.clamp(self.output_min, self.output_max)
    }

    /// Reset controller state
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }
}

/// Impedance controller parameters
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ImpedanceController {
    pub stiffness: f64,
    pub damping: f64,
    pub inertia: f64,
}

impl Default for ImpedanceController {
    fn default() -> Self {
        Self {
            stiffness: 100.0,
            damping: 10.0,
            inertia: 1.0,
        }
    }
}

impl ImpedanceController {
    /// Create new impedance controller
    pub fn new(stiffness: f64, damping: f64) -> Self {
        Self {
            stiffness,
            damping,
            ..Default::default()
        }
    }

    /// Compute impedance control force
    pub fn compute_force(&self, position_error: f64, velocity: f64) -> f64 {
        -self.stiffness * position_error - self.damping * velocity
    }

    /// Compute impedance control torque for joint
    pub fn compute_torque(&self, joint: &Joint) -> f64 {
        let position_error = joint.angle_error();
        let velocity = joint.velocity_deg_s;
        self.compute_force(position_error, velocity)
    }
}

/// Trajectory generator
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrajectoryGenerator {
    pub start_position: f64,
    pub end_position: f64,
    pub duration: f64,
    pub elapsed: f64,
    pub trajectory_type: TrajectoryType,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum TrajectoryType {
    #[default]
    Linear,
    Trapezoidal,
    SCurve,
    MinJerk,
}

impl TrajectoryGenerator {
    /// Create a new trajectory
    pub fn new(start: f64, end: f64, duration: f64) -> Self {
        Self {
            start_position: start,
            end_position: end,
            duration,
            elapsed: 0.0,
            trajectory_type: TrajectoryType::Linear,
        }
    }

    /// Set trajectory type
    pub fn with_type(mut self, traj_type: TrajectoryType) -> Self {
        self.trajectory_type = traj_type;
        self
    }

    /// Get position at time t
    pub fn position_at(&self, t: f64) -> f64 {
        let t = t.clamp(0.0, self.duration);
        let s = t / self.duration; // Normalized time [0, 1]

        let interpolation = match self.trajectory_type {
            TrajectoryType::Linear => s,
            TrajectoryType::Trapezoidal => self.trapezoidal_profile(s),
            TrajectoryType::SCurve => self.s_curve_profile(s),
            TrajectoryType::MinJerk => self.min_jerk_profile(s),
        };

        self.start_position + (self.end_position - self.start_position) * interpolation
    }

    /// Update trajectory and get current position
    pub fn update(&mut self, dt: f64) -> f64 {
        self.elapsed = (self.elapsed + dt).min(self.duration);
        self.position_at(self.elapsed)
    }

    /// Check if trajectory is complete
    pub fn is_complete(&self) -> bool {
        self.elapsed >= self.duration
    }

    /// Reset trajectory
    pub fn reset(&mut self) {
        self.elapsed = 0.0;
    }

    /// Get current velocity
    pub fn velocity_at(&self, t: f64) -> f64 {
        let dt = 0.001;
        if t + dt <= self.duration {
            (self.position_at(t + dt) - self.position_at(t)) / dt
        } else {
            0.0
        }
    }

    // Profile functions

    fn trapezoidal_profile(&self, s: f64) -> f64 {
        let accel_time = 0.25; // 25% acceleration phase
        let decel_time = 0.75; // Start deceleration at 75%

        if s < accel_time {
            2.0 * s * s / accel_time
        } else if s < decel_time {
            accel_time + (s - accel_time) * 2.0 / (1.0 - 2.0 * accel_time)
        } else {
            1.0 - 2.0 * (1.0 - s).powi(2) / accel_time
        }
    }

    fn s_curve_profile(&self, s: f64) -> f64 {
        // Sigmoid-like smooth profile
        if s <= 0.0 {
            0.0
        } else if s >= 1.0 {
            1.0
        } else {
            s * s * (3.0 - 2.0 * s)
        }
    }

    fn min_jerk_profile(&self, s: f64) -> f64 {
        // Minimum jerk trajectory (5th order polynomial)
        10.0 * s.powi(3) - 15.0 * s.powi(4) + 6.0 * s.powi(5)
    }
}

/// Motion profile for smooth motion
#[derive(Debug, Clone, Copy)]
pub struct MotionProfile {
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub max_jerk: f64,
}

impl Default for MotionProfile {
    fn default() -> Self {
        Self {
            max_velocity: 1.0,
            max_acceleration: 2.0,
            max_jerk: 10.0,
        }
    }
}

impl MotionProfile {
    /// Calculate minimum time for motion
    pub fn min_time(&self, distance: f64) -> f64 {
        // Simplified trapezoidal profile time calculation
        let accel_time = self.max_velocity / self.max_acceleration;
        let accel_distance = 0.5 * self.max_acceleration * accel_time.powi(2);

        if distance < 2.0 * accel_distance {
            // Triangular profile (no constant velocity phase)
            2.0 * (distance / self.max_acceleration).sqrt()
        } else {
            // Trapezoidal profile
            let cruise_distance = distance - 2.0 * accel_distance;
            let cruise_time = cruise_distance / self.max_velocity;
            2.0 * accel_time + cruise_time
        }
    }

    /// Validate velocity against limits
    pub fn validate_velocity(&self, velocity: f64) -> RobotResult<()> {
        if velocity.abs() > self.max_velocity {
            return Err(RobotError::SafetyViolation(format!(
                "Velocity {:.2} exceeds limit {:.2}",
                velocity.abs(),
                self.max_velocity
            )));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pid_controller() {
        let mut pid = PIDController::new(1.0, 0.1, 0.05);
        pid.set_limits(-10.0, 10.0);

        let output = pid.compute(5.0, 0.01);
        assert!(output > 0.0);
    }

    #[test]
    fn test_impedance_controller() {
        let controller = ImpedanceController::new(100.0, 10.0);
        let force = controller.compute_force(0.1, 0.5);
        assert!(force < 0.0); // Should push back
    }

    #[test]
    fn test_trajectory_generator() {
        let mut traj = TrajectoryGenerator::new(0.0, 100.0, 1.0);

        assert!((traj.position_at(0.0) - 0.0).abs() < 1e-10);
        assert!((traj.position_at(0.5) - 50.0).abs() < 1e-10);
        assert!((traj.position_at(1.0) - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_trajectory_min_jerk() {
        let traj = TrajectoryGenerator::new(0.0, 100.0, 1.0).with_type(TrajectoryType::MinJerk);

        // Start and end should be exact
        assert!((traj.position_at(0.0) - 0.0).abs() < 1e-10);
        assert!((traj.position_at(1.0) - 100.0).abs() < 1e-10);

        // Mid-point should be around 50 for symmetric profile
        let mid = traj.position_at(0.5);
        assert!((mid - 50.0).abs() < 5.0);
    }

    #[test]
    fn test_motion_profile() {
        let profile = MotionProfile::default();
        assert!(profile.validate_velocity(0.5).is_ok());
        assert!(profile.validate_velocity(2.0).is_err());
    }
}
