//! WIA Exoskeleton Safety Module
//!
//! This module provides safety systems for rehabilitation exoskeletons,
//! including emergency stop, joint limits, and overload protection.
//!
//! # Modules
//!
//! - [`estop`]: Emergency stop system
//! - [`limits`]: Joint range of motion limits
//! - [`overload`]: Overload detection and protection
//!
//! # Safety Architecture
//!
//! The safety system implements a multi-layer protection approach:
//!
//! 1. **E-Stop** - Immediate system shutdown on critical events
//! 2. **Joint Limits** - Prevent movement beyond safe ranges
//! 3. **Overload Protection** - Protect motors, structure, and battery
//!
//! All safety systems operate independently and can trigger E-Stop
//! when critical conditions are detected.

pub mod estop;
pub mod limits;
pub mod overload;

// Re-exports
pub use estop::{
    EmergencyStopSystem,
    EStopConfig,
    EStopEvent,
    EStopTrigger,
    EStopTriggerType,
    EStopSource,
    EStopAction,
    EStopStatus,
    EStopResetConfig,
    EStopResetInfo,
    ResetEligibility,
    ResetResult,
    StopCategory,
    SelfTestLevel,
};

pub use limits::{
    JointLimitsSystem,
    JointLimits,
    AngleLimits,
    VelocityLimits,
    TorqueLimits,
    LimitZone,
    EnforcementAction,
    EnforcementResult,
    VelocityCheckResult,
    LimitViolation,
    ViolationSeverity,
    LimitType,
};

pub use overload::{
    OverloadDetector,
    OverloadConfig,
    OverloadType,
    OverloadLevel,
    OverloadAction,
    OverloadEvent,
    OverloadStatus,
    MotorOverloadConfig,
    BatteryOverloadConfig,
    StructuralOverloadConfig,
    RecoveryConfig,
    MotorStatus,
    BatteryStatus,
};

/// Unified safety system combining E-Stop, limits, and overload protection
pub struct SafetySystem {
    pub estop: EmergencyStopSystem,
    pub limits: JointLimitsSystem,
    pub overload: OverloadDetector,
}

impl SafetySystem {
    /// Create a new safety system with default configuration
    pub fn new() -> Self {
        Self {
            estop: EmergencyStopSystem::new(EStopConfig::default()),
            limits: JointLimitsSystem::new(),
            overload: OverloadDetector::new(OverloadConfig::default()),
        }
    }

    /// Create a new safety system with custom configuration
    pub fn with_config(
        estop_config: EStopConfig,
        overload_config: OverloadConfig,
    ) -> Self {
        Self {
            estop: EmergencyStopSystem::new(estop_config),
            limits: JointLimitsSystem::new(),
            overload: OverloadDetector::new(overload_config),
        }
    }

    /// Check if any safety system is in fault state
    pub fn has_fault(&self) -> bool {
        self.estop.is_active() ||
        self.overload.get_status().overall >= OverloadLevel::Limit
    }

    /// Get overall safety status
    pub fn get_status(&self) -> SafetyStatus {
        SafetyStatus {
            estop_active: self.estop.is_active(),
            estop_status: self.estop.get_status(),
            overload_status: self.overload.get_status(),
            safe_to_operate: !self.has_fault(),
        }
    }

    /// Update watchdog (should be called periodically)
    pub fn heartbeat(&mut self) {
        self.estop.heartbeat();
        self.estop.check_watchdog();
    }
}

impl Default for SafetySystem {
    fn default() -> Self {
        Self::new()
    }
}

/// Combined safety status
#[derive(Debug, Clone)]
pub struct SafetyStatus {
    pub estop_active: bool,
    pub estop_status: EStopStatus,
    pub overload_status: OverloadStatus,
    pub safe_to_operate: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_safety_system_creation() {
        let safety = SafetySystem::new();
        assert!(!safety.has_fault());
        assert!(safety.get_status().safe_to_operate);
    }

    #[test]
    fn test_safety_fault_detection() {
        let mut safety = SafetySystem::new();

        // Trigger E-Stop
        safety.estop.trigger(
            EStopTriggerType::HardwareButton,
            EStopSource::UserInput,
            "Test",
            None,
        );

        assert!(safety.has_fault());
        assert!(!safety.get_status().safe_to_operate);
    }
}
