//! WIA Exoskeleton Control Module
//!
//! This module provides control interfaces for rehabilitation exoskeletons,
//! including position, velocity, torque, and impedance control modes.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

// ============================================================================
// Enumerations
// ============================================================================

/// Joint types supported by the exoskeleton
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointType {
    Hip,
    Knee,
    Ankle,
}

/// Body side
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Side {
    Left,
    Right,
}

/// Control modes for the exoskeleton
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlMode {
    /// Position tracking control
    Position,
    /// Velocity tracking control
    Velocity,
    /// Direct torque control
    Torque,
    /// Impedance control (virtual spring-damper)
    Impedance,
    /// Admittance control (force to motion)
    Admittance,
    /// Zero torque / transparent mode
    ZeroTorque,
}

/// Assistance modes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssistanceMode {
    /// No motor resistance, free movement
    Passive,
    /// Motor assists user movement
    ActiveAssist,
    /// Motor resists user movement
    ActiveResist,
    /// Minimal interference, friction/inertia compensation
    Transparent,
    /// Adaptive resistance based on performance
    Challenge,
}

/// Command priority levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[repr(u8)]
pub enum CommandPriority {
    /// Emergency stop commands
    Emergency = 0,
    /// Safety-related commands
    Safety = 1,
    /// Therapeutic/rehabilitation commands
    Therapeutic = 2,
    /// Normal control commands
    Normal = 3,
    /// Background tasks
    Background = 4,
}

/// Command response status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ResponseStatus {
    Accepted,
    Executing,
    Completed,
    Rejected,
    Timeout,
    Error,
}

/// Adaptive assistance algorithms
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AdaptiveAlgorithm {
    /// Based on trajectory tracking error
    ErrorBased,
    /// Based on EMG muscle activation
    EmgBased,
    /// Based on detected fatigue
    FatigueBased,
    /// Based on gait performance metrics
    PerformanceBased,
}

// ============================================================================
// Control Parameters
// ============================================================================

/// Position control parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PositionControlParams {
    /// Target angle in degrees
    pub target_angle: f64,
    /// Maximum velocity in deg/s
    #[serde(default = "default_max_velocity")]
    pub max_velocity: f64,
    /// Maximum acceleration in deg/s²
    #[serde(default = "default_max_acceleration")]
    pub max_acceleration: f64,
    /// Proportional gain (Nm/deg)
    #[serde(default = "default_kp")]
    pub kp: f64,
    /// Derivative gain (Nm·s/deg)
    #[serde(default = "default_kd")]
    pub kd: f64,
}

fn default_max_velocity() -> f64 { 100.0 }
fn default_max_acceleration() -> f64 { 200.0 }
fn default_kp() -> f64 { 2.5 }
fn default_kd() -> f64 { 0.1 }

/// Velocity control parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VelocityControlParams {
    /// Target velocity in deg/s
    pub target_velocity: f64,
    /// Maximum torque in Nm
    #[serde(default = "default_max_torque")]
    pub max_torque: f64,
    /// Velocity gain
    #[serde(default = "default_kv")]
    pub kv: f64,
    /// Integral gain
    #[serde(default = "default_ki")]
    pub ki: f64,
}

fn default_max_torque() -> f64 { 50.0 }
fn default_kv() -> f64 { 1.0 }
fn default_ki() -> f64 { 0.1 }

/// Torque control parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TorqueControlParams {
    /// Target torque in Nm
    pub target_torque: f64,
    /// Torque ramp rate in Nm/s
    #[serde(default = "default_ramp_rate")]
    pub ramp_rate: f64,
    /// Feedforward torque in Nm
    #[serde(default)]
    pub feedforward: f64,
}

fn default_ramp_rate() -> f64 { 50.0 }

/// Impedance control parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImpedanceParams {
    /// Virtual stiffness in Nm/rad (K)
    pub stiffness: f64,
    /// Virtual damping in Nm·s/rad (B)
    pub damping: f64,
    /// Equilibrium angle in degrees (θ₀)
    pub equilibrium_angle: f64,
    /// Virtual inertia in kg·m² (optional)
    #[serde(default)]
    pub inertia: Option<f64>,
}

/// Admittance control parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdmittanceParams {
    /// Virtual mass in kg (M)
    pub virtual_mass: f64,
    /// Virtual damping in Ns/m (B)
    pub virtual_damping: f64,
    /// Virtual stiffness in N/m (K)
    pub virtual_stiffness: f64,
    /// Force threshold for movement initiation in N
    #[serde(default = "default_force_threshold")]
    pub force_threshold: f64,
}

fn default_force_threshold() -> f64 { 5.0 }

/// Zero torque mode parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ZeroTorqueParams {
    /// Enable friction compensation
    pub friction_compensation: bool,
    /// Enable gravity compensation
    pub gravity_compensation: bool,
    /// Enable inertia compensation
    #[serde(default)]
    pub inertia_compensation: bool,
}

/// Control parameters enum
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ControlParams {
    Position(PositionControlParams),
    Velocity(VelocityControlParams),
    Torque(TorqueControlParams),
    Impedance(ImpedanceParams),
    Admittance(AdmittanceParams),
    ZeroTorque(ZeroTorqueParams),
}

// ============================================================================
// Control Commands
// ============================================================================

/// Control command structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControlCommand {
    /// Unique command identifier
    pub id: String,
    /// Command timestamp (Unix milliseconds)
    pub timestamp: u64,
    /// Target joint
    pub joint: JointType,
    /// Body side
    pub side: Side,
    /// Control mode
    pub mode: ControlMode,
    /// Control parameters
    pub params: ControlParams,
    /// Command priority
    pub priority: CommandPriority,
    /// Command timeout in milliseconds
    #[serde(default)]
    pub timeout: Option<u64>,
}

/// Control error information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControlError {
    /// Error code
    pub code: String,
    /// Error message
    pub message: String,
    /// Additional details
    #[serde(default)]
    pub details: Option<HashMap<String, String>>,
}

/// Control command response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControlResponse {
    /// Original command ID
    pub command_id: String,
    /// Response status
    pub status: ResponseStatus,
    /// Response timestamp
    pub timestamp: u64,
    /// Actual achieved value
    #[serde(default)]
    pub actual_value: Option<f64>,
    /// Error information if applicable
    #[serde(default)]
    pub error: Option<ControlError>,
}

// ============================================================================
// Joint Limits
// ============================================================================

/// Range limit for a single joint
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RangeLimit {
    /// Minimum angle (degrees)
    pub min: f64,
    /// Maximum angle (degrees)
    pub max: f64,
    /// Maximum velocity (deg/s)
    #[serde(default)]
    pub velocity_limit: Option<f64>,
    /// Maximum torque (Nm)
    #[serde(default)]
    pub torque_limit: Option<f64>,
}

/// Joint limits for all joints
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointLimits {
    pub hip: RangeLimit,
    pub knee: RangeLimit,
    pub ankle: RangeLimit,
}

impl Default for JointLimits {
    fn default() -> Self {
        Self {
            hip: RangeLimit {
                min: -30.0,
                max: 120.0,
                velocity_limit: Some(200.0),
                torque_limit: Some(60.0),
            },
            knee: RangeLimit {
                min: 0.0,
                max: 135.0,
                velocity_limit: Some(250.0),
                torque_limit: Some(80.0),
            },
            ankle: RangeLimit {
                min: -30.0,
                max: 50.0,
                velocity_limit: Some(150.0),
                torque_limit: Some(40.0),
            },
        }
    }
}

// ============================================================================
// Assistance Control
// ============================================================================

/// Adaptive assistance configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdaptiveAssistanceConfig {
    /// Enable adaptive control
    pub enabled: bool,
    /// Adaptation algorithm
    pub algorithm: AdaptiveAlgorithm,
    /// Minimum assistance level (0-100%)
    pub min_assistance: f64,
    /// Maximum assistance level (0-100%)
    pub max_assistance: f64,
    /// Adaptation rate (%/s)
    pub adaptation_rate: f64,
    /// Evaluation window size (ms)
    #[serde(default = "default_window_size")]
    pub window_size: u64,
    /// High error threshold
    #[serde(default = "default_error_threshold_high")]
    pub error_threshold_high: f64,
    /// Low error threshold
    #[serde(default = "default_error_threshold_low")]
    pub error_threshold_low: f64,
}

fn default_window_size() -> u64 { 1000 }
fn default_error_threshold_high() -> f64 { 10.0 }
fn default_error_threshold_low() -> f64 { 5.0 }

impl Default for AdaptiveAssistanceConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            algorithm: AdaptiveAlgorithm::ErrorBased,
            min_assistance: 0.0,
            max_assistance: 100.0,
            adaptation_rate: 2.0,
            window_size: 1000,
            error_threshold_high: 10.0,
            error_threshold_low: 5.0,
        }
    }
}

/// Joint-specific assistance levels
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct JointAssistanceLevels {
    pub hip: f64,
    pub knee: f64,
    pub ankle: f64,
}

/// Assistance state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssistanceState {
    /// Global assistance level (0-100%)
    pub global_level: f64,
    /// Left side joint levels
    pub left: JointAssistanceLevels,
    /// Right side joint levels
    pub right: JointAssistanceLevels,
    /// Adaptive configuration
    pub adaptive_config: AdaptiveAssistanceConfig,
    /// Current assistance mode
    pub mode: AssistanceMode,
}

// ============================================================================
// Trajectory Generation
// ============================================================================

/// Trajectory point
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrajectoryPoint {
    /// Time from start (ms)
    pub time: f64,
    /// Angle (degrees)
    pub angle: f64,
    /// Velocity (deg/s)
    pub velocity: f64,
    /// Acceleration (deg/s²)
    pub acceleration: f64,
}

/// Generate minimum jerk trajectory
pub fn generate_minimum_jerk_trajectory(
    start_angle: f64,
    end_angle: f64,
    duration_ms: f64,
    sample_rate: f64,
) -> Vec<TrajectoryPoint> {
    let num_samples = ((duration_ms / 1000.0) * sample_rate) as usize;
    let t_total = duration_ms / 1000.0;
    let mut points = Vec::with_capacity(num_samples + 1);

    for i in 0..=num_samples {
        let t = (i as f64 / num_samples as f64) * t_total;
        let tau = t / t_total;
        let tau2 = tau * tau;
        let tau3 = tau2 * tau;
        let tau4 = tau3 * tau;
        let tau5 = tau4 * tau;

        // Position: θ(t) = θ0 + (θf - θ0) × [10τ³ - 15τ⁴ + 6τ⁵]
        let s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
        let angle = start_angle + (end_angle - start_angle) * s;

        // Velocity: θ̇(t)
        let ds = (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4) / t_total;
        let velocity = (end_angle - start_angle) * ds;

        // Acceleration: θ̈(t)
        let dds = (60.0 * tau - 180.0 * tau2 + 120.0 * tau3) / (t_total * t_total);
        let acceleration = (end_angle - start_angle) * dds;

        points.push(TrajectoryPoint {
            time: t * 1000.0,
            angle,
            velocity,
            acceleration,
        });
    }

    points
}

/// Generate cycloid trajectory
pub fn generate_cycloid_trajectory(
    amplitude: f64,
    period_ms: f64,
    cycles: usize,
    sample_rate: f64,
) -> Vec<TrajectoryPoint> {
    let duration_ms = period_ms * cycles as f64;
    let num_samples = ((duration_ms / 1000.0) * sample_rate) as usize;
    let omega = 2.0 * std::f64::consts::PI / (period_ms / 1000.0);
    let mut points = Vec::with_capacity(num_samples + 1);

    for i in 0..=num_samples {
        let t = (i as f64 / num_samples as f64) * (duration_ms / 1000.0);
        let phase = omega * t;

        let angle = amplitude * (1.0 - phase.cos()) / 2.0;
        let velocity = amplitude * omega * phase.sin() / 2.0;
        let acceleration = amplitude * omega * omega * phase.cos() / 2.0;

        points.push(TrajectoryPoint {
            time: t * 1000.0,
            angle,
            velocity,
            acceleration,
        });
    }

    points
}

// ============================================================================
// Controller Implementation
// ============================================================================

/// Controller state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControllerState {
    pub mode: ControlMode,
    pub assistance_mode: AssistanceMode,
    pub is_active: bool,
    pub current_command: Option<ControlCommand>,
    pub last_response: Option<ControlResponse>,
    pub error_state: Option<ControlError>,
}

/// Exoskeleton controller
pub struct ExoController {
    state: ControllerState,
    assistance_state: AssistanceState,
    joint_limits: HashMap<Side, JointLimits>,
}

impl ExoController {
    /// Create a new controller
    pub fn new(initial_assistance: f64) -> Self {
        let mut joint_limits = HashMap::new();
        joint_limits.insert(Side::Left, JointLimits::default());
        joint_limits.insert(Side::Right, JointLimits::default());

        Self {
            state: ControllerState {
                mode: ControlMode::ZeroTorque,
                assistance_mode: AssistanceMode::Passive,
                is_active: false,
                current_command: None,
                last_response: None,
                error_state: None,
            },
            assistance_state: AssistanceState {
                global_level: initial_assistance,
                left: JointAssistanceLevels {
                    hip: initial_assistance,
                    knee: initial_assistance,
                    ankle: initial_assistance,
                },
                right: JointAssistanceLevels {
                    hip: initial_assistance,
                    knee: initial_assistance,
                    ankle: initial_assistance,
                },
                adaptive_config: AdaptiveAssistanceConfig::default(),
                mode: AssistanceMode::Passive,
            },
            joint_limits,
        }
    }

    /// Get current timestamp
    fn now() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0)
    }

    /// Generate command ID
    fn generate_command_id() -> String {
        format!("cmd-{}-{:x}", Self::now(), rand::random::<u32>())
    }

    /// Set control mode
    pub fn set_control_mode(&mut self, mode: ControlMode) -> ControlResponse {
        self.state.mode = mode;
        ControlResponse {
            command_id: Self::generate_command_id(),
            status: ResponseStatus::Completed,
            timestamp: Self::now(),
            actual_value: None,
            error: None,
        }
    }

    /// Get current control mode
    pub fn get_control_mode(&self) -> ControlMode {
        self.state.mode
    }

    /// Position control
    pub fn position_control(
        &mut self,
        joint: JointType,
        side: Side,
        params: PositionControlParams,
    ) -> ControlResponse {
        let limits = self.joint_limits.get(&side).unwrap();
        let limit = match joint {
            JointType::Hip => &limits.hip,
            JointType::Knee => &limits.knee,
            JointType::Ankle => &limits.ankle,
        };

        // Validate target angle
        if params.target_angle < limit.min || params.target_angle > limit.max {
            return ControlResponse {
                command_id: Self::generate_command_id(),
                status: ResponseStatus::Error,
                timestamp: Self::now(),
                actual_value: None,
                error: Some(ControlError {
                    code: "ANGLE_OUT_OF_RANGE".to_string(),
                    message: format!(
                        "Target angle {}° is outside limits [{}°, {}°]",
                        params.target_angle, limit.min, limit.max
                    ),
                    details: None,
                }),
            };
        }

        let command = ControlCommand {
            id: Self::generate_command_id(),
            timestamp: Self::now(),
            joint,
            side,
            mode: ControlMode::Position,
            params: ControlParams::Position(params),
            priority: CommandPriority::Normal,
            timeout: Some(5000),
        };

        self.execute_command(command)
    }

    /// Impedance control
    pub fn impedance_control(
        &mut self,
        joint: JointType,
        side: Side,
        params: ImpedanceParams,
    ) -> ControlResponse {
        if params.stiffness < 0.0 || params.damping < 0.0 {
            return ControlResponse {
                command_id: Self::generate_command_id(),
                status: ResponseStatus::Error,
                timestamp: Self::now(),
                actual_value: None,
                error: Some(ControlError {
                    code: "INVALID_IMPEDANCE_PARAMS".to_string(),
                    message: "Stiffness and damping must be non-negative".to_string(),
                    details: None,
                }),
            };
        }

        let command = ControlCommand {
            id: Self::generate_command_id(),
            timestamp: Self::now(),
            joint,
            side,
            mode: ControlMode::Impedance,
            params: ControlParams::Impedance(params),
            priority: CommandPriority::Normal,
            timeout: Some(10000),
        };

        self.execute_command(command)
    }

    /// Emergency stop
    pub fn emergency_stop(&mut self) -> ControlResponse {
        self.state.is_active = false;
        self.state.mode = ControlMode::ZeroTorque;
        self.state.current_command = None;

        ControlResponse {
            command_id: Self::generate_command_id(),
            status: ResponseStatus::Completed,
            timestamp: Self::now(),
            actual_value: None,
            error: None,
        }
    }

    /// Execute command
    fn execute_command(&mut self, command: ControlCommand) -> ControlResponse {
        self.state.current_command = Some(command.clone());
        self.state.mode = command.mode;
        self.state.is_active = true;

        let response = ControlResponse {
            command_id: command.id,
            status: ResponseStatus::Completed,
            timestamp: Self::now(),
            actual_value: None,
            error: None,
        };

        self.state.last_response = Some(response.clone());
        response
    }

    /// Set global assistance level
    pub fn set_assistance_level(&mut self, percent: f64) {
        let level = percent.clamp(0.0, 100.0);
        self.assistance_state.global_level = level;
        self.assistance_state.left.hip = level;
        self.assistance_state.left.knee = level;
        self.assistance_state.left.ankle = level;
        self.assistance_state.right.hip = level;
        self.assistance_state.right.knee = level;
        self.assistance_state.right.ankle = level;
    }

    /// Get global assistance level
    pub fn get_assistance_level(&self) -> f64 {
        self.assistance_state.global_level
    }

    /// Set assistance mode
    pub fn set_assistance_mode(&mut self, mode: AssistanceMode) {
        self.assistance_state.mode = mode;
        self.state.assistance_mode = mode;
    }

    /// Get assistance mode
    pub fn get_assistance_mode(&self) -> AssistanceMode {
        self.assistance_state.mode
    }

    /// Get controller state
    pub fn get_state(&self) -> &ControllerState {
        &self.state
    }

    /// Get assistance state
    pub fn get_assistance_state(&self) -> &AssistanceState {
        &self.assistance_state
    }

    /// Update adaptive assistance based on tracking error
    pub fn update_adaptive_assistance(
        &mut self,
        target_trajectory: &[f64],
        actual_trajectory: &[f64],
    ) -> f64 {
        let config = &self.assistance_state.adaptive_config;

        if !config.enabled {
            return self.assistance_state.global_level;
        }

        let rmse = calculate_rmse(target_trajectory, actual_trajectory);
        let current = self.assistance_state.global_level;

        let new_assistance = if rmse > config.error_threshold_high {
            (current + config.adaptation_rate).min(config.max_assistance)
        } else if rmse < config.error_threshold_low {
            (current - config.adaptation_rate / 2.0).max(config.min_assistance)
        } else {
            current
        };

        self.set_assistance_level(new_assistance);
        new_assistance
    }
}

/// Calculate RMSE between two trajectories
fn calculate_rmse(target: &[f64], actual: &[f64]) -> f64 {
    if target.len() != actual.len() || target.is_empty() {
        return f64::INFINITY;
    }

    let sum_squared_error: f64 = target
        .iter()
        .zip(actual.iter())
        .map(|(t, a)| (t - a).powi(2))
        .sum();

    (sum_squared_error / target.len() as f64).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_controller_creation() {
        let controller = ExoController::new(50.0);
        assert_eq!(controller.get_assistance_level(), 50.0);
        assert_eq!(controller.get_control_mode(), ControlMode::ZeroTorque);
    }

    #[test]
    fn test_minimum_jerk_trajectory() {
        let trajectory = generate_minimum_jerk_trajectory(0.0, 90.0, 1000.0, 100.0);
        assert!(!trajectory.is_empty());
        assert!((trajectory.first().unwrap().angle - 0.0).abs() < 0.01);
        assert!((trajectory.last().unwrap().angle - 90.0).abs() < 0.01);
    }

    #[test]
    fn test_emergency_stop() {
        let mut controller = ExoController::new(50.0);
        controller.set_control_mode(ControlMode::Position);
        let response = controller.emergency_stop();
        assert_eq!(response.status, ResponseStatus::Completed);
        assert_eq!(controller.get_control_mode(), ControlMode::ZeroTorque);
    }
}
