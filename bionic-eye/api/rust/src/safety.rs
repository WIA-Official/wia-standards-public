//! Safety systems for WIA Bionic Eye

use serde::{Deserialize, Serialize};
use std::time::{Duration, Instant};

/// Emergency stop system
pub struct EmergencyStopSystem {
    config: EmergencyStopConfig,
    state: EStopState,
    last_trigger: Option<EmergencyEvent>,
    trigger_count: u32,
    cooldown_start: Option<Instant>,
}

/// E-Stop configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmergencyStopConfig {
    pub over_current: TriggerConfig,
    pub over_voltage: TriggerConfig,
    pub impedance_anomaly: ImpedanceTriggerConfig,
    pub thermal_overload: ThermalTriggerConfig,
    pub charge_imbalance: ChargeTriggerConfig,
    pub cooldown_period_s: u32,
    pub max_auto_restarts: u32,
}

/// Basic trigger configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TriggerConfig {
    pub enabled: bool,
    pub threshold: f32,
    pub response_time_us: u32,
}

/// Impedance trigger configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImpedanceTriggerConfig {
    pub enabled: bool,
    pub sudden_change_threshold_percent: f32,
    pub absolute_high_kohm: f32,
    pub absolute_low_kohm: f32,
}

/// Thermal trigger configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThermalTriggerConfig {
    pub enabled: bool,
    pub max_temperature_rise_c: f32,
    pub absolute_max_c: f32,
}

/// Charge imbalance trigger configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChargeTriggerConfig {
    pub enabled: bool,
    pub threshold_percent: f32,
    pub window_ms: u32,
}

/// E-Stop state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EStopState {
    Normal,
    Triggered,
    Cooldown,
}

/// Emergency event record
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmergencyEvent {
    pub timestamp: i64,
    pub trigger_type: TriggerType,
    pub trigger_value: f32,
    pub threshold: f32,
    pub electrodes_active: Vec<u32>,
}

/// Trigger types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TriggerType {
    OverCurrent,
    OverVoltage,
    ImpedanceAnomaly,
    ThermalOverload,
    ChargeImbalance,
    ManualPatient,
    ManualCaregiver,
    VoiceCommand,
    CommunicationLoss,
}

/// Safety check result
#[derive(Debug, Clone)]
pub struct SafetyCheckResult {
    pub safe: bool,
    pub violations: Vec<SafetyViolation>,
    pub warnings: Vec<String>,
}

/// Safety violation
#[derive(Debug, Clone)]
pub struct SafetyViolation {
    pub violation_type: TriggerType,
    pub value: f32,
    pub limit: f32,
    pub electrode: Option<u32>,
}

impl EmergencyStopSystem {
    /// Create a new emergency stop system
    pub fn new(config: EmergencyStopConfig) -> Self {
        Self {
            config,
            state: EStopState::Normal,
            last_trigger: None,
            trigger_count: 0,
            cooldown_start: None,
        }
    }

    /// Get current state
    pub fn state(&self) -> EStopState {
        self.state
    }

    /// Trigger emergency stop
    pub fn trigger(&mut self, event: EmergencyEvent) {
        self.state = EStopState::Triggered;
        self.last_trigger = Some(event);
        self.trigger_count += 1;
    }

    /// Manual trigger by patient
    pub fn trigger_manual_patient(&mut self) {
        self.trigger(EmergencyEvent {
            timestamp: chrono::Utc::now().timestamp_millis(),
            trigger_type: TriggerType::ManualPatient,
            trigger_value: 0.0,
            threshold: 0.0,
            electrodes_active: Vec::new(),
        });
    }

    /// Check if restart is allowed
    pub fn can_restart(&self) -> bool {
        match self.state {
            EStopState::Normal => true,
            EStopState::Triggered => false,
            EStopState::Cooldown => {
                if let Some(start) = self.cooldown_start {
                    start.elapsed() >= Duration::from_secs(self.config.cooldown_period_s as u64)
                } else {
                    false
                }
            }
        }
    }

    /// Start cooldown period
    pub fn start_cooldown(&mut self) {
        if self.state == EStopState::Triggered {
            self.state = EStopState::Cooldown;
            self.cooldown_start = Some(Instant::now());
        }
    }

    /// Reset to normal state
    pub fn reset(&mut self) -> Result<(), &'static str> {
        if !self.can_restart() {
            return Err("Cannot reset: still in cooldown or triggered state");
        }
        if self.trigger_count >= self.config.max_auto_restarts {
            return Err("Maximum restart attempts exceeded");
        }
        self.state = EStopState::Normal;
        self.cooldown_start = None;
        Ok(())
    }

    /// Check current values against safety limits
    pub fn check_safety(
        &self,
        current_ua: &[f32],
        voltage_v: f32,
        impedance_kohm: &[f32],
        temperature_c: f32,
    ) -> SafetyCheckResult {
        let mut result = SafetyCheckResult {
            safe: true,
            violations: Vec::new(),
            warnings: Vec::new(),
        };

        // Check over-current
        if self.config.over_current.enabled {
            for (idx, &current) in current_ua.iter().enumerate() {
                if current > self.config.over_current.threshold {
                    result.safe = false;
                    result.violations.push(SafetyViolation {
                        violation_type: TriggerType::OverCurrent,
                        value: current,
                        limit: self.config.over_current.threshold,
                        electrode: Some(idx as u32),
                    });
                }
            }
        }

        // Check over-voltage
        if self.config.over_voltage.enabled {
            if voltage_v > self.config.over_voltage.threshold {
                result.safe = false;
                result.violations.push(SafetyViolation {
                    violation_type: TriggerType::OverVoltage,
                    value: voltage_v,
                    limit: self.config.over_voltage.threshold,
                    electrode: None,
                });
            }
        }

        // Check impedance
        if self.config.impedance_anomaly.enabled {
            for (idx, &imp) in impedance_kohm.iter().enumerate() {
                if imp > self.config.impedance_anomaly.absolute_high_kohm {
                    result.safe = false;
                    result.violations.push(SafetyViolation {
                        violation_type: TriggerType::ImpedanceAnomaly,
                        value: imp,
                        limit: self.config.impedance_anomaly.absolute_high_kohm,
                        electrode: Some(idx as u32),
                    });
                } else if imp < self.config.impedance_anomaly.absolute_low_kohm {
                    result.safe = false;
                    result.violations.push(SafetyViolation {
                        violation_type: TriggerType::ImpedanceAnomaly,
                        value: imp,
                        limit: self.config.impedance_anomaly.absolute_low_kohm,
                        electrode: Some(idx as u32),
                    });
                }
            }
        }

        // Check thermal
        if self.config.thermal_overload.enabled {
            if temperature_c > self.config.thermal_overload.absolute_max_c {
                result.safe = false;
                result.violations.push(SafetyViolation {
                    violation_type: TriggerType::ThermalOverload,
                    value: temperature_c,
                    limit: self.config.thermal_overload.absolute_max_c,
                    electrode: None,
                });
            }
        }

        result
    }
}

impl Default for EmergencyStopConfig {
    fn default() -> Self {
        Self {
            over_current: TriggerConfig {
                enabled: true,
                threshold: 1000.0,  // μA
                response_time_us: 100,
            },
            over_voltage: TriggerConfig {
                enabled: true,
                threshold: 10.0,  // V
                response_time_us: 100,
            },
            impedance_anomaly: ImpedanceTriggerConfig {
                enabled: true,
                sudden_change_threshold_percent: 30.0,
                absolute_high_kohm: 100.0,
                absolute_low_kohm: 0.5,
            },
            thermal_overload: ThermalTriggerConfig {
                enabled: true,
                max_temperature_rise_c: 2.0,
                absolute_max_c: 40.0,
            },
            charge_imbalance: ChargeTriggerConfig {
                enabled: true,
                threshold_percent: 5.0,
                window_ms: 100,
            },
            cooldown_period_s: 30,
            max_auto_restarts: 3,
        }
    }
}
