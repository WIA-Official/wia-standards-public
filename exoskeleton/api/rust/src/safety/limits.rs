//! Joint Limits Module
//!
//! Implements joint range of motion (ROM) limits, velocity limits,
//! and torque limits for safe exoskeleton operation.

use serde::{Deserialize, Serialize};
use crate::control::{JointType, Side};

// ============================================================================
// Enumerations
// ============================================================================

/// Zone classification for limit enforcement
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LimitZone {
    /// Normal operation zone
    Safe,
    /// Approaching limit, reduce velocity
    Warning,
    /// Near limit, resist movement
    Danger,
    /// At hard limit, movement blocked
    Blocked,
}

/// Enforcement action
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EnforcementAction {
    None,
    ReduceVelocity,
    Resist,
    EStop,
}

/// Limit violation severity
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ViolationSeverity {
    Info,
    Warning,
    Error,
    Critical,
}

// ============================================================================
// Data Structures
// ============================================================================

/// Angle limit configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AngleLimits {
    /// Hardware minimum (mechanical stop)
    pub hard_min: f64,
    /// Hardware maximum (mechanical stop)
    pub hard_max: f64,
    /// Software minimum
    pub soft_min: f64,
    /// Software maximum
    pub soft_max: f64,
    /// Warning zone margin before soft limit
    pub soft_limit_margin: f64,
}

/// Velocity limit configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VelocityLimits {
    /// Maximum velocity (deg/s)
    pub max: f64,
    /// Maximum acceleration (deg/s²)
    pub ramp_rate: f64,
    /// Approach speed near limits (deg/s)
    pub approach_speed: f64,
}

/// Torque limit configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TorqueLimits {
    /// Continuous maximum torque (Nm)
    pub continuous: f64,
    /// Peak torque (Nm)
    pub peak: f64,
    /// Maximum peak duration (ms)
    pub peak_duration: u64,
    /// Maximum assistance torque (Nm)
    pub assist_max: f64,
    /// Maximum resistance torque (Nm)
    pub resist_max: f64,
}

/// Complete joint limits
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointLimits {
    pub joint: JointType,
    pub angle: AngleLimits,
    pub velocity: VelocityLimits,
    pub torque: TorqueLimits,
}

/// Default joint limits
impl JointLimits {
    pub fn default_hip() -> Self {
        Self {
            joint: JointType::Hip,
            angle: AngleLimits {
                hard_min: -35.0,
                hard_max: 125.0,
                soft_min: -20.0,
                soft_max: 110.0,
                soft_limit_margin: 10.0,
            },
            velocity: VelocityLimits {
                max: 200.0,
                ramp_rate: 400.0,
                approach_speed: 50.0,
            },
            torque: TorqueLimits {
                continuous: 40.0,
                peak: 60.0,
                peak_duration: 1000,
                assist_max: 50.0,
                resist_max: 30.0,
            },
        }
    }

    pub fn default_knee() -> Self {
        Self {
            joint: JointType::Knee,
            angle: AngleLimits {
                hard_min: -5.0,
                hard_max: 145.0,
                soft_min: 0.0,
                soft_max: 130.0,
                soft_limit_margin: 10.0,
            },
            velocity: VelocityLimits {
                max: 250.0,
                ramp_rate: 500.0,
                approach_speed: 60.0,
            },
            torque: TorqueLimits {
                continuous: 50.0,
                peak: 80.0,
                peak_duration: 1000,
                assist_max: 60.0,
                resist_max: 40.0,
            },
        }
    }

    pub fn default_ankle() -> Self {
        Self {
            joint: JointType::Ankle,
            angle: AngleLimits {
                hard_min: -35.0,
                hard_max: 55.0,
                soft_min: -25.0,
                soft_max: 45.0,
                soft_limit_margin: 8.0,
            },
            velocity: VelocityLimits {
                max: 150.0,
                ramp_rate: 300.0,
                approach_speed: 40.0,
            },
            torque: TorqueLimits {
                continuous: 25.0,
                peak: 40.0,
                peak_duration: 1000,
                assist_max: 35.0,
                resist_max: 20.0,
            },
        }
    }
}

/// Enforcement result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnforcementResult {
    pub zone: LimitZone,
    pub action: EnforcementAction,
    pub torque: f64,
    pub max_velocity: Option<f64>,
    pub reason: Option<String>,
}

/// Velocity check result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VelocityCheckResult {
    pub allowed: bool,
    pub velocity: f64,
    pub limited: bool,
    pub reason: Option<String>,
}

/// Limit violation event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LimitViolation {
    pub id: String,
    pub timestamp: u64,
    pub joint: JointType,
    pub side: Side,
    pub limit_type: LimitType,
    pub severity: ViolationSeverity,
    pub current_value: f64,
    pub limit_value: f64,
    pub exceedance: f64,
    pub duration: u64,
    pub action: EnforcementAction,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LimitType {
    Angle,
    Velocity,
    Torque,
}

// ============================================================================
// Joint Limits System
// ============================================================================

/// Joint limits system
pub struct JointLimitsSystem {
    limits: std::collections::HashMap<(JointType, Side), JointLimits>,
    violations: Vec<LimitViolation>,
}

impl JointLimitsSystem {
    /// Create a new joint limits system with default limits
    pub fn new() -> Self {
        let mut limits = std::collections::HashMap::new();

        // Set default limits for all joints
        for side in [Side::Left, Side::Right] {
            limits.insert((JointType::Hip, side), JointLimits::default_hip());
            limits.insert((JointType::Knee, side), JointLimits::default_knee());
            limits.insert((JointType::Ankle, side), JointLimits::default_ankle());
        }

        Self {
            limits,
            violations: Vec::new(),
        }
    }

    /// Get limits for a specific joint
    pub fn get_limits(&self, joint: JointType, side: Side) -> Option<&JointLimits> {
        self.limits.get(&(joint, side))
    }

    /// Set limits for a specific joint
    pub fn set_limits(&mut self, joint: JointType, side: Side, limits: JointLimits) {
        self.limits.insert((joint, side), limits);
    }

    /// Set soft limits (must be within hard limits)
    pub fn set_soft_limits(
        &mut self,
        joint: JointType,
        side: Side,
        min: f64,
        max: f64,
    ) -> Result<(), String> {
        let limits = self.limits.get_mut(&(joint, side))
            .ok_or_else(|| "Joint limits not found".to_string())?;

        if min < limits.angle.hard_min {
            return Err(format!(
                "Soft min {} is below hard min {}",
                min, limits.angle.hard_min
            ));
        }
        if max > limits.angle.hard_max {
            return Err(format!(
                "Soft max {} is above hard max {}",
                max, limits.angle.hard_max
            ));
        }
        if min >= max {
            return Err("Soft min must be less than soft max".to_string());
        }

        limits.angle.soft_min = min;
        limits.angle.soft_max = max;
        Ok(())
    }

    /// Determine current zone based on angle
    pub fn get_zone(&self, joint: JointType, side: Side, angle: f64) -> LimitZone {
        let Some(limits) = self.get_limits(joint, side) else {
            return LimitZone::Blocked;
        };

        let angle_limits = &limits.angle;

        // Check hard limits
        if angle <= angle_limits.hard_min || angle >= angle_limits.hard_max {
            return LimitZone::Blocked;
        }

        // Check soft limits (danger zone)
        if angle < angle_limits.soft_min || angle > angle_limits.soft_max {
            return LimitZone::Danger;
        }

        // Check warning zone (margin before soft limits)
        let warning_min = angle_limits.soft_min + angle_limits.soft_limit_margin;
        let warning_max = angle_limits.soft_max - angle_limits.soft_limit_margin;

        if angle < warning_min || angle > warning_max {
            return LimitZone::Warning;
        }

        LimitZone::Safe
    }

    /// Enforce limits on commanded torque
    pub fn enforce_limits(
        &self,
        joint: JointType,
        side: Side,
        current_angle: f64,
        commanded_torque: f64,
    ) -> EnforcementResult {
        let Some(limits) = self.get_limits(joint, side) else {
            return EnforcementResult {
                zone: LimitZone::Blocked,
                action: EnforcementAction::EStop,
                torque: 0.0,
                max_velocity: None,
                reason: Some("Limits not configured".to_string()),
            };
        };

        let zone = self.get_zone(joint, side, current_angle);

        match zone {
            LimitZone::Blocked => EnforcementResult {
                zone,
                action: EnforcementAction::EStop,
                torque: 0.0,
                max_velocity: Some(0.0),
                reason: Some("Hard limit reached".to_string()),
            },

            LimitZone::Danger => {
                // Calculate resistance torque
                let resist_torque = self.calculate_resist_torque(
                    current_angle,
                    &limits.angle,
                    commanded_torque,
                    limits.torque.resist_max,
                );

                EnforcementResult {
                    zone,
                    action: EnforcementAction::Resist,
                    torque: resist_torque,
                    max_velocity: Some(limits.velocity.approach_speed * 0.4),
                    reason: Some("Soft limit exceeded".to_string()),
                }
            }

            LimitZone::Warning => EnforcementResult {
                zone,
                action: EnforcementAction::ReduceVelocity,
                torque: commanded_torque,
                max_velocity: Some(limits.velocity.approach_speed),
                reason: Some("Approaching soft limit".to_string()),
            },

            LimitZone::Safe => EnforcementResult {
                zone,
                action: EnforcementAction::None,
                torque: commanded_torque,
                max_velocity: Some(limits.velocity.max),
                reason: None,
            },
        }
    }

    /// Calculate resistance torque when in danger zone
    fn calculate_resist_torque(
        &self,
        current_angle: f64,
        angle_limits: &AngleLimits,
        commanded_torque: f64,
        max_resist: f64,
    ) -> f64 {
        // Calculate distance to hard limit
        let dist_to_min = current_angle - angle_limits.hard_min;
        let dist_to_max = angle_limits.hard_max - current_angle;

        // Determine which limit we're approaching
        let (dist_to_limit, approaching_min) = if dist_to_min < dist_to_max {
            (dist_to_min, true)
        } else {
            (dist_to_max, false)
        };

        // Calculate resistance factor (increases as we get closer to limit)
        let danger_zone_size = if approaching_min {
            angle_limits.soft_min - angle_limits.hard_min
        } else {
            angle_limits.hard_max - angle_limits.soft_max
        };

        let resist_factor = 1.0 - (dist_to_limit / danger_zone_size).clamp(0.0, 1.0);

        // Calculate resistance torque
        let resist_torque = resist_factor * max_resist;

        // Apply resistance in opposite direction to commanded torque
        if approaching_min && commanded_torque < 0.0 {
            // Moving toward min limit, resist
            (commanded_torque + resist_torque).max(0.0)
        } else if !approaching_min && commanded_torque > 0.0 {
            // Moving toward max limit, resist
            (commanded_torque - resist_torque).min(0.0)
        } else {
            // Moving away from limit, allow
            commanded_torque
        }
    }

    /// Check velocity limit
    pub fn check_velocity_limit(
        &self,
        joint: JointType,
        side: Side,
        current_velocity: f64,
        target_velocity: f64,
        current_angle: f64,
        dt: f64,
    ) -> VelocityCheckResult {
        let Some(limits) = self.get_limits(joint, side) else {
            return VelocityCheckResult {
                allowed: false,
                velocity: 0.0,
                limited: true,
                reason: Some("Limits not configured".to_string()),
            };
        };

        let zone = self.get_zone(joint, side, current_angle);

        // Determine max allowed velocity based on zone
        let max_velocity = match zone {
            LimitZone::Safe => limits.velocity.max,
            LimitZone::Warning => limits.velocity.approach_speed,
            LimitZone::Danger => limits.velocity.approach_speed * 0.4,
            LimitZone::Blocked => 0.0,
        };

        // Clamp target velocity
        let clamped_velocity = target_velocity.signum()
            * target_velocity.abs().min(max_velocity);

        // Check acceleration limit
        let acceleration = (clamped_velocity - current_velocity) / dt;
        let max_accel = limits.velocity.ramp_rate;

        let final_velocity = if acceleration.abs() > max_accel {
            current_velocity + acceleration.signum() * max_accel * dt
        } else {
            clamped_velocity
        };

        let limited = (final_velocity - target_velocity).abs() > 0.001;

        VelocityCheckResult {
            allowed: true,
            velocity: final_velocity,
            limited,
            reason: if limited {
                Some("Velocity/acceleration limited".to_string())
            } else {
                None
            },
        }
    }

    /// Check torque limit
    pub fn check_torque_limit(
        &self,
        joint: JointType,
        side: Side,
        torque: f64,
        is_peak: bool,
    ) -> (f64, bool) {
        let Some(limits) = self.get_limits(joint, side) else {
            return (0.0, true);
        };

        let max_torque = if is_peak {
            limits.torque.peak
        } else {
            limits.torque.continuous
        };

        let clamped = torque.signum() * torque.abs().min(max_torque);
        let limited = (clamped - torque).abs() > 0.001;

        (clamped, limited)
    }

    /// Record a violation
    pub fn record_violation(&mut self, violation: LimitViolation) {
        self.violations.push(violation);

        // Keep only last 1000 violations
        if self.violations.len() > 1000 {
            self.violations.remove(0);
        }
    }

    /// Get recent violations
    pub fn get_violations(&self, limit: Option<usize>) -> &[LimitViolation] {
        let limit = limit.unwrap_or(self.violations.len());
        let start = self.violations.len().saturating_sub(limit);
        &self.violations[start..]
    }

    /// Clear violations
    pub fn clear_violations(&mut self) {
        self.violations.clear();
    }
}

impl Default for JointLimitsSystem {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zone_detection() {
        let system = JointLimitsSystem::new();

        // Safe zone (middle of range)
        assert_eq!(
            system.get_zone(JointType::Knee, Side::Right, 60.0),
            LimitZone::Safe
        );

        // Warning zone (near soft limit)
        assert_eq!(
            system.get_zone(JointType::Knee, Side::Right, 125.0),
            LimitZone::Warning
        );

        // Danger zone (past soft limit)
        assert_eq!(
            system.get_zone(JointType::Knee, Side::Right, 135.0),
            LimitZone::Danger
        );
    }

    #[test]
    fn test_soft_limit_validation() {
        let mut system = JointLimitsSystem::new();

        // Valid soft limits
        assert!(system.set_soft_limits(JointType::Knee, Side::Right, 5.0, 120.0).is_ok());

        // Invalid: below hard min
        assert!(system.set_soft_limits(JointType::Knee, Side::Right, -10.0, 120.0).is_err());

        // Invalid: above hard max
        assert!(system.set_soft_limits(JointType::Knee, Side::Right, 5.0, 150.0).is_err());
    }

    #[test]
    fn test_enforcement() {
        let system = JointLimitsSystem::new();

        // Safe zone - no restriction
        let result = system.enforce_limits(JointType::Knee, Side::Right, 60.0, 10.0);
        assert_eq!(result.zone, LimitZone::Safe);
        assert_eq!(result.action, EnforcementAction::None);

        // Danger zone - resist
        let result = system.enforce_limits(JointType::Knee, Side::Right, 140.0, 10.0);
        assert_eq!(result.zone, LimitZone::Danger);
        assert_eq!(result.action, EnforcementAction::Resist);
    }
}
