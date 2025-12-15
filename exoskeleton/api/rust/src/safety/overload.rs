//! Overload Detection Module
//!
//! Implements overload detection and protection for motors, structure,
//! and battery systems.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

// ============================================================================
// Enumerations
// ============================================================================

/// Overload types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OverloadType {
    MotorCurrent,
    MotorTemperature,
    MotorVoltage,
    StructuralForce,
    StructuralTorque,
    BatteryVoltageLow,
    BatteryVoltageHigh,
    BatteryCurrent,
    BatteryTemperature,
    ProcessorTemperature,
    DriverTemperature,
    CommunicationTimeout,
}

/// Overload severity level
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OverloadLevel {
    Ok,
    Warning,
    Limit,
    Shutdown,
}

/// Overload response action
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OverloadAction {
    Log,
    Alert,
    ReducePower,
    DisableJoint,
    EStop,
}

// ============================================================================
// Data Structures
// ============================================================================

/// Motor overload configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorOverloadConfig {
    /// Current thresholds
    pub current_warning: f64,      // % of rated
    pub current_limit: f64,        // % of rated
    pub current_shutdown: f64,     // % of rated
    pub rated_current: f64,        // A

    /// Temperature thresholds
    pub temp_warning: f64,         // °C
    pub temp_limit: f64,           // °C
    pub temp_shutdown: f64,        // °C

    /// I²t protection
    pub i2t_capacity: f64,         // A²·s
    pub i2t_warning: f64,          // % of capacity
    pub cooling_rate: f64,         // A²/s
}

impl Default for MotorOverloadConfig {
    fn default() -> Self {
        Self {
            current_warning: 80.0,
            current_limit: 100.0,
            current_shutdown: 150.0,
            rated_current: 10.0,
            temp_warning: 60.0,
            temp_limit: 75.0,
            temp_shutdown: 85.0,
            i2t_capacity: 1000.0,
            i2t_warning: 70.0,
            cooling_rate: 5.0,
        }
    }
}

/// Battery overload configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BatteryOverloadConfig {
    /// Voltage thresholds
    pub voltage_warning_low: f64,   // V
    pub voltage_shutdown_low: f64,  // V
    pub voltage_warning_high: f64,  // V
    pub voltage_shutdown_high: f64, // V

    /// Current thresholds
    pub current_warning: f64,       // A
    pub current_shutdown: f64,      // A

    /// Temperature thresholds
    pub temp_warning: f64,          // °C
    pub temp_shutdown: f64,         // °C
}

impl Default for BatteryOverloadConfig {
    fn default() -> Self {
        Self {
            voltage_warning_low: 22.0,
            voltage_shutdown_low: 18.0,
            voltage_warning_high: 56.0,
            voltage_shutdown_high: 60.0,
            current_warning: 40.0,
            current_shutdown: 50.0,
            temp_warning: 50.0,
            temp_shutdown: 70.0,
        }
    }
}

/// Structural overload configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StructuralOverloadConfig {
    /// Force thresholds (% of rated)
    pub force_warning: f64,
    pub force_shutdown: f64,

    /// Torque thresholds (% of rated)
    pub torque_warning: f64,
    pub torque_shutdown: f64,
}

impl Default for StructuralOverloadConfig {
    fn default() -> Self {
        Self {
            force_warning: 80.0,
            force_shutdown: 100.0,
            torque_warning: 80.0,
            torque_shutdown: 100.0,
        }
    }
}

/// Recovery configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecoveryConfig {
    pub cooldown_period: u64,       // ms
    pub recovery_threshold: f64,    // % of warning
    pub gradual_recovery: bool,
    pub recovery_rate: f64,         // % per second
}

impl Default for RecoveryConfig {
    fn default() -> Self {
        Self {
            cooldown_period: 30000,
            recovery_threshold: 80.0,
            gradual_recovery: true,
            recovery_rate: 10.0,
        }
    }
}

/// Complete overload configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OverloadConfig {
    pub motor: MotorOverloadConfig,
    pub battery: BatteryOverloadConfig,
    pub structural: StructuralOverloadConfig,
    pub recovery: RecoveryConfig,
}

impl Default for OverloadConfig {
    fn default() -> Self {
        Self {
            motor: MotorOverloadConfig::default(),
            battery: BatteryOverloadConfig::default(),
            structural: StructuralOverloadConfig::default(),
            recovery: RecoveryConfig::default(),
        }
    }
}

/// I²t protection state
#[derive(Debug, Clone, Default)]
pub struct I2tState {
    pub accumulator: f64,
    pub last_update: u64,
}

/// Motor status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorStatus {
    pub motor_id: u8,
    pub current: f64,
    pub temperature: f64,
    pub i2t_level: f64,
    pub current_level: OverloadLevel,
    pub temp_level: OverloadLevel,
    pub i2t_level_status: OverloadLevel,
}

/// Battery status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BatteryStatus {
    pub voltage: f64,
    pub current: f64,
    pub temperature: f64,
    pub soc: f64,
    pub voltage_level: OverloadLevel,
    pub current_level: OverloadLevel,
    pub temp_level: OverloadLevel,
}

/// Overload event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OverloadEvent {
    pub id: String,
    pub timestamp: u64,
    pub overload_type: OverloadType,
    pub level: OverloadLevel,
    pub source: String,
    pub value: f64,
    pub threshold: f64,
    pub unit: String,
    pub duration: Option<u64>,
    pub action: OverloadAction,
    pub resolved: bool,
    pub resolved_at: Option<u64>,
}

/// Overall overload status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OverloadStatus {
    pub timestamp: u64,
    pub overall: OverloadLevel,
    pub motors: Vec<MotorStatus>,
    pub battery: BatteryStatus,
    pub active_overloads: Vec<OverloadEvent>,
    pub warnings: Vec<OverloadEvent>,
}

// ============================================================================
// Overload Detector Implementation
// ============================================================================

/// Overload detection system
pub struct OverloadDetector {
    config: OverloadConfig,
    motor_i2t: HashMap<u8, I2tState>,
    active_events: Vec<OverloadEvent>,
    event_history: Vec<OverloadEvent>,
    power_reduction: f64,
    callbacks: Vec<Box<dyn Fn(&OverloadEvent) + Send + Sync>>,
}

impl OverloadDetector {
    /// Create a new overload detector
    pub fn new(config: OverloadConfig) -> Self {
        Self {
            config,
            motor_i2t: HashMap::new(),
            active_events: Vec::new(),
            event_history: Vec::new(),
            power_reduction: 1.0,
            callbacks: Vec::new(),
        }
    }

    /// Get current timestamp
    fn now() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0)
    }

    /// Generate event ID
    fn generate_event_id() -> String {
        format!("overload-{}-{:x}", Self::now(), rand::random::<u32>())
    }

    /// Check motor current overload
    pub fn check_motor_current(&mut self, motor_id: u8, current: f64) -> OverloadLevel {
        let config = &self.config.motor;
        let rated = config.rated_current;

        let percent = (current / rated) * 100.0;

        let level = if percent >= config.current_shutdown {
            OverloadLevel::Shutdown
        } else if percent >= config.current_limit {
            OverloadLevel::Limit
        } else if percent >= config.current_warning {
            OverloadLevel::Warning
        } else {
            OverloadLevel::Ok
        };

        if level >= OverloadLevel::Warning {
            self.record_event(
                OverloadType::MotorCurrent,
                level,
                format!("motor_{}", motor_id),
                current,
                rated * (config.current_warning / 100.0),
                "A".to_string(),
            );
        }

        level
    }

    /// Check motor temperature overload
    pub fn check_motor_temperature(&mut self, motor_id: u8, temperature: f64) -> OverloadLevel {
        let config = &self.config.motor;

        let level = if temperature >= config.temp_shutdown {
            OverloadLevel::Shutdown
        } else if temperature >= config.temp_limit {
            OverloadLevel::Limit
        } else if temperature >= config.temp_warning {
            OverloadLevel::Warning
        } else {
            OverloadLevel::Ok
        };

        if level >= OverloadLevel::Warning {
            self.record_event(
                OverloadType::MotorTemperature,
                level,
                format!("motor_{}", motor_id),
                temperature,
                config.temp_warning,
                "°C".to_string(),
            );
        }

        level
    }

    /// Update I²t protection
    pub fn update_i2t(&mut self, motor_id: u8, current: f64, dt: f64) -> OverloadLevel {
        let config = &self.config.motor;

        let state = self.motor_i2t.entry(motor_id).or_insert(I2tState::default());

        // Calculate I²t contribution
        let i2t_contribution = current * current * dt;

        // Calculate cooling
        let cooling = config.cooling_rate * dt;

        // Update accumulator
        state.accumulator = (state.accumulator + i2t_contribution - cooling).max(0.0);
        state.last_update = Self::now();

        // Check thresholds
        let percent = (state.accumulator / config.i2t_capacity) * 100.0;

        let level = if state.accumulator >= config.i2t_capacity {
            OverloadLevel::Shutdown
        } else if percent >= config.i2t_warning {
            OverloadLevel::Warning
        } else {
            OverloadLevel::Ok
        };

        if level >= OverloadLevel::Warning {
            self.record_event(
                OverloadType::MotorCurrent,
                level,
                format!("motor_{}_i2t", motor_id),
                state.accumulator,
                config.i2t_capacity * (config.i2t_warning / 100.0),
                "A²s".to_string(),
            );
        }

        level
    }

    /// Check battery voltage
    pub fn check_battery_voltage(&mut self, voltage: f64) -> OverloadLevel {
        let config = &self.config.battery;

        // Low voltage check
        if voltage <= config.voltage_shutdown_low {
            self.record_event(
                OverloadType::BatteryVoltageLow,
                OverloadLevel::Shutdown,
                "battery".to_string(),
                voltage,
                config.voltage_shutdown_low,
                "V".to_string(),
            );
            return OverloadLevel::Shutdown;
        }

        if voltage <= config.voltage_warning_low {
            self.record_event(
                OverloadType::BatteryVoltageLow,
                OverloadLevel::Warning,
                "battery".to_string(),
                voltage,
                config.voltage_warning_low,
                "V".to_string(),
            );
            return OverloadLevel::Warning;
        }

        // High voltage check
        if voltage >= config.voltage_shutdown_high {
            self.record_event(
                OverloadType::BatteryVoltageHigh,
                OverloadLevel::Shutdown,
                "battery".to_string(),
                voltage,
                config.voltage_shutdown_high,
                "V".to_string(),
            );
            return OverloadLevel::Shutdown;
        }

        if voltage >= config.voltage_warning_high {
            self.record_event(
                OverloadType::BatteryVoltageHigh,
                OverloadLevel::Warning,
                "battery".to_string(),
                voltage,
                config.voltage_warning_high,
                "V".to_string(),
            );
            return OverloadLevel::Warning;
        }

        OverloadLevel::Ok
    }

    /// Check battery current
    pub fn check_battery_current(&mut self, current: f64) -> OverloadLevel {
        let config = &self.config.battery;

        let level = if current >= config.current_shutdown {
            OverloadLevel::Shutdown
        } else if current >= config.current_warning {
            OverloadLevel::Warning
        } else {
            OverloadLevel::Ok
        };

        if level >= OverloadLevel::Warning {
            self.record_event(
                OverloadType::BatteryCurrent,
                level,
                "battery".to_string(),
                current,
                config.current_warning,
                "A".to_string(),
            );
        }

        level
    }

    /// Check battery temperature
    pub fn check_battery_temperature(&mut self, temperature: f64) -> OverloadLevel {
        let config = &self.config.battery;

        let level = if temperature >= config.temp_shutdown {
            OverloadLevel::Shutdown
        } else if temperature >= config.temp_warning {
            OverloadLevel::Warning
        } else {
            OverloadLevel::Ok
        };

        if level >= OverloadLevel::Warning {
            self.record_event(
                OverloadType::BatteryTemperature,
                level,
                "battery".to_string(),
                temperature,
                config.temp_warning,
                "°C".to_string(),
            );
        }

        level
    }

    /// Record overload event
    fn record_event(
        &mut self,
        overload_type: OverloadType,
        level: OverloadLevel,
        source: String,
        value: f64,
        threshold: f64,
        unit: String,
    ) {
        // Check if similar event already active
        let existing = self.active_events.iter_mut().find(|e| {
            e.overload_type == overload_type && e.source == source && !e.resolved
        });

        if let Some(event) = existing {
            // Update existing event
            event.level = level;
            event.value = value;
            event.duration = Some(Self::now() - event.timestamp);
        } else {
            // Create new event
            let action = match level {
                OverloadLevel::Shutdown => OverloadAction::EStop,
                OverloadLevel::Limit => OverloadAction::ReducePower,
                OverloadLevel::Warning => OverloadAction::Alert,
                OverloadLevel::Ok => OverloadAction::Log,
            };

            let event = OverloadEvent {
                id: Self::generate_event_id(),
                timestamp: Self::now(),
                overload_type,
                level,
                source,
                value,
                threshold,
                unit,
                duration: None,
                action,
                resolved: false,
                resolved_at: None,
            };

            // Notify callbacks
            for callback in &self.callbacks {
                callback(&event);
            }

            self.active_events.push(event);
        }
    }

    /// Get overall status
    pub fn get_status(&self) -> OverloadStatus {
        let overall = self
            .active_events
            .iter()
            .filter(|e| !e.resolved)
            .map(|e| e.level)
            .max()
            .unwrap_or(OverloadLevel::Ok);

        OverloadStatus {
            timestamp: Self::now(),
            overall,
            motors: vec![],  // Would be populated from sensor data
            battery: BatteryStatus {
                voltage: 0.0,
                current: 0.0,
                temperature: 0.0,
                soc: 0.0,
                voltage_level: OverloadLevel::Ok,
                current_level: OverloadLevel::Ok,
                temp_level: OverloadLevel::Ok,
            },
            active_overloads: self.active_events.iter().filter(|e| !e.resolved).cloned().collect(),
            warnings: self.active_events.iter().filter(|e| e.level == OverloadLevel::Warning).cloned().collect(),
        }
    }

    /// Get current power reduction factor
    pub fn get_power_reduction(&self) -> f64 {
        self.power_reduction
    }

    /// Reduce power output
    pub fn reduce_power(&mut self, factor: f64) {
        self.power_reduction = (self.power_reduction * factor).clamp(0.0, 1.0);
    }

    /// Restore power (gradual)
    pub fn restore_power(&mut self, rate: f64, dt: f64) {
        if self.config.recovery.gradual_recovery {
            self.power_reduction = (self.power_reduction + rate * dt / 100.0).min(1.0);
        } else {
            self.power_reduction = 1.0;
        }
    }

    /// Reset specific overload type
    pub fn reset(&mut self, overload_type: Option<OverloadType>) {
        let now = Self::now();

        for event in &mut self.active_events {
            if overload_type.is_none() || Some(event.overload_type) == overload_type {
                event.resolved = true;
                event.resolved_at = Some(now);
            }
        }

        // Move resolved events to history
        let resolved: Vec<_> = self.active_events
            .iter()
            .filter(|e| e.resolved)
            .cloned()
            .collect();

        self.active_events.retain(|e| !e.resolved);
        self.event_history.extend(resolved);

        // Keep history limited
        if self.event_history.len() > 1000 {
            self.event_history.drain(0..self.event_history.len() - 1000);
        }
    }

    /// Register callback for overload events
    pub fn on_overload<F>(&mut self, callback: F)
    where
        F: Fn(&OverloadEvent) + Send + Sync + 'static,
    {
        self.callbacks.push(Box::new(callback));
    }

    /// Get event history
    pub fn get_history(&self, limit: Option<usize>) -> &[OverloadEvent] {
        let limit = limit.unwrap_or(self.event_history.len());
        let start = self.event_history.len().saturating_sub(limit);
        &self.event_history[start..]
    }

    /// Get configuration
    pub fn get_config(&self) -> &OverloadConfig {
        &self.config
    }

    /// Update configuration
    pub fn configure(&mut self, config: OverloadConfig) {
        self.config = config;
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_motor_current_check() {
        let config = OverloadConfig::default();
        let mut detector = OverloadDetector::new(config);

        // Normal current (50% of rated)
        assert_eq!(
            detector.check_motor_current(0, 5.0),
            OverloadLevel::Ok
        );

        // Warning level (85% of rated)
        assert_eq!(
            detector.check_motor_current(0, 8.5),
            OverloadLevel::Warning
        );

        // Shutdown level (160% of rated)
        assert_eq!(
            detector.check_motor_current(0, 16.0),
            OverloadLevel::Shutdown
        );
    }

    #[test]
    fn test_battery_voltage_check() {
        let config = OverloadConfig::default();
        let mut detector = OverloadDetector::new(config);

        // Normal voltage
        assert_eq!(detector.check_battery_voltage(48.0), OverloadLevel::Ok);

        // Low warning
        assert_eq!(detector.check_battery_voltage(21.0), OverloadLevel::Warning);

        // Low shutdown
        assert_eq!(detector.check_battery_voltage(17.0), OverloadLevel::Shutdown);

        // High warning
        assert_eq!(detector.check_battery_voltage(57.0), OverloadLevel::Warning);
    }

    #[test]
    fn test_i2t_protection() {
        let config = OverloadConfig::default();
        let mut detector = OverloadDetector::new(config);

        // Initial state should be OK
        assert_eq!(detector.update_i2t(0, 5.0, 0.001), OverloadLevel::Ok);

        // After sustained high current, should trigger warning
        for _ in 0..10000 {
            detector.update_i2t(0, 15.0, 0.001);
        }

        // Check that accumulator has increased
        let state = detector.motor_i2t.get(&0).unwrap();
        assert!(state.accumulator > 0.0);
    }
}
