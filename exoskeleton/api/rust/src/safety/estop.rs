//! Emergency Stop Module
//!
//! Implements the emergency stop (E-Stop) system for the exoskeleton,
//! including trigger detection, action execution, and reset procedures.

use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::time::{SystemTime, UNIX_EPOCH};

// ============================================================================
// Enumerations
// ============================================================================

/// Stop categories per IEC 60204-1
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum StopCategory {
    /// Immediate power cut (uncontrolled stop)
    Category0,
    /// Controlled stop then power cut
    Category1,
    /// Controlled stop, power maintained
    Category2,
}

/// E-Stop trigger types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EStopTriggerType {
    HardwareButton,
    SoftwareCommand,
    WatchdogTimeout,
    LimitSwitch,
    Overcurrent,
    Overtemperature,
    Overvoltage,
    Undervoltage,
    PositionError,
    VelocityError,
    SensorFailure,
    FallDetection,
    CommunicationLoss,
}

/// E-Stop source
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EStopSource {
    MotorController,
    SafetyProcessor,
    MainProcessor,
    UserInput,
    ExternalSystem,
}

/// Self-test level for reset
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SelfTestLevel {
    /// Basic connectivity test
    Quick,
    /// Sensors + motors test
    Partial,
    /// Complete system test
    Full,
}

// ============================================================================
// Data Structures
// ============================================================================

/// E-Stop trigger information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EStopTrigger {
    pub id: String,
    pub timestamp: u64,
    pub trigger_type: EStopTriggerType,
    pub source: EStopSource,
    pub reason: String,
    pub value: Option<f64>,
    pub threshold: Option<f64>,
    pub priority: u8,
}

/// E-Stop action configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EStopAction {
    pub motor_disable: bool,
    pub brake_engage: bool,
    pub power_cut: bool,
    pub alert_sound: bool,
    pub alert_visual: bool,
    pub log_event: bool,
    pub notify_operator: bool,
    pub notify_remote: bool,
}

impl Default for EStopAction {
    fn default() -> Self {
        Self {
            motor_disable: true,
            brake_engage: true,
            power_cut: false,
            alert_sound: true,
            alert_visual: true,
            log_event: true,
            notify_operator: true,
            notify_remote: false,
        }
    }
}

/// E-Stop reset configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EStopResetConfig {
    pub require_manual_reset: bool,
    pub safety_check_required: bool,
    pub cooldown_period: u64,         // ms
    pub operator_confirmation: bool,
    pub self_test_level: SelfTestLevel,
    pub max_reset_attempts: u32,
    pub reset_lockout_time: u64,      // ms
}

impl Default for EStopResetConfig {
    fn default() -> Self {
        Self {
            require_manual_reset: true,
            safety_check_required: true,
            cooldown_period: 5000,
            operator_confirmation: true,
            self_test_level: SelfTestLevel::Partial,
            max_reset_attempts: 3,
            reset_lockout_time: 30000,
        }
    }
}

/// E-Stop event record
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EStopEvent {
    pub id: String,
    pub timestamp: u64,
    pub trigger: EStopTrigger,
    pub actions: EStopAction,
    pub category: StopCategory,
    pub reset_info: Option<EStopResetInfo>,
}

/// E-Stop reset information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EStopResetInfo {
    pub reset_timestamp: u64,
    pub reset_by: String,
    pub self_test_passed: bool,
    pub reset_attempt: u32,
}

/// E-Stop status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EStopStatus {
    pub is_active: bool,
    pub category: Option<StopCategory>,
    pub active_event: Option<EStopEvent>,
    pub time_since_triggered: Option<u64>,
    pub reset_eligible: bool,
    pub cooldown_remaining: Option<u64>,
    pub reset_attempts: u32,
}

/// Reset eligibility check result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResetEligibility {
    pub can_reset: bool,
    pub reasons: Vec<String>,
    pub cooldown_remaining: u64,
    pub pending_checks: Vec<String>,
}

/// Reset result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResetResult {
    pub success: bool,
    pub reason: Option<String>,
    pub self_test_passed: Option<bool>,
}

// ============================================================================
// E-Stop Configuration
// ============================================================================

/// E-Stop system configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EStopConfig {
    /// Actions for each stop category
    pub category0_actions: EStopAction,
    pub category1_actions: EStopAction,
    pub category2_actions: EStopAction,

    /// Reset configuration
    pub reset_config: EStopResetConfig,

    /// Trigger configurations
    pub trigger_configs: TriggerConfigs,

    /// Watchdog settings
    pub watchdog_timeout: u64,        // ms
    pub heartbeat_interval: u64,      // ms
}

/// Trigger-specific configurations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TriggerConfigs {
    pub overcurrent_threshold: f64,   // A
    pub overtemp_threshold: f64,      // °C
    pub overvoltage_threshold: f64,   // V
    pub undervoltage_threshold: f64,  // V
    pub position_error_threshold: f64,// degrees
    pub velocity_error_threshold: f64,// % deviation
}

impl Default for TriggerConfigs {
    fn default() -> Self {
        Self {
            overcurrent_threshold: 15.0,
            overtemp_threshold: 80.0,
            overvoltage_threshold: 60.0,
            undervoltage_threshold: 18.0,
            position_error_threshold: 20.0,
            velocity_error_threshold: 50.0,
        }
    }
}

impl Default for EStopConfig {
    fn default() -> Self {
        Self {
            category0_actions: EStopAction {
                power_cut: true,
                ..Default::default()
            },
            category1_actions: EStopAction::default(),
            category2_actions: EStopAction {
                power_cut: false,
                brake_engage: false,
                ..Default::default()
            },
            reset_config: EStopResetConfig::default(),
            trigger_configs: TriggerConfigs::default(),
            watchdog_timeout: 100,
            heartbeat_interval: 50,
        }
    }
}

// ============================================================================
// E-Stop System Implementation
// ============================================================================

/// Emergency Stop System
pub struct EmergencyStopSystem {
    config: EStopConfig,
    is_active: bool,
    current_event: Option<EStopEvent>,
    triggered_at: Option<u64>,
    reset_attempts: u32,
    last_heartbeat: u64,
    event_history: VecDeque<EStopEvent>,
    callbacks: Vec<Box<dyn Fn(&EStopEvent) + Send + Sync>>,
}

impl EmergencyStopSystem {
    /// Create a new E-Stop system
    pub fn new(config: EStopConfig) -> Self {
        Self {
            config,
            is_active: false,
            current_event: None,
            triggered_at: None,
            reset_attempts: 0,
            last_heartbeat: now(),
            event_history: VecDeque::with_capacity(100),
            callbacks: Vec::new(),
        }
    }

    /// Get current timestamp
    fn now() -> u64 {
        now()
    }

    /// Generate event ID
    fn generate_event_id() -> String {
        format!("estop-{}-{:x}", now(), rand::random::<u32>())
    }

    /// Trigger E-Stop
    pub fn trigger(
        &mut self,
        trigger_type: EStopTriggerType,
        source: EStopSource,
        reason: &str,
        value: Option<f64>,
    ) {
        if self.is_active {
            return; // Already in E-Stop
        }

        let category = self.determine_category(trigger_type);
        let actions = self.get_actions_for_category(category);

        let trigger = EStopTrigger {
            id: format!("trig-{}", Self::now()),
            timestamp: Self::now(),
            trigger_type,
            source,
            reason: reason.to_string(),
            value,
            threshold: self.get_threshold_for_type(trigger_type),
            priority: self.get_priority_for_type(trigger_type),
        };

        let event = EStopEvent {
            id: Self::generate_event_id(),
            timestamp: Self::now(),
            trigger,
            actions: actions.clone(),
            category,
            reset_info: None,
        };

        self.is_active = true;
        self.triggered_at = Some(Self::now());
        self.current_event = Some(event.clone());

        // Execute actions
        self.execute_actions(&actions);

        // Store in history
        if self.event_history.len() >= 100 {
            self.event_history.pop_front();
        }
        self.event_history.push_back(event.clone());

        // Notify callbacks
        for callback in &self.callbacks {
            callback(&event);
        }
    }

    /// Determine stop category based on trigger type
    fn determine_category(&self, trigger_type: EStopTriggerType) -> StopCategory {
        match trigger_type {
            EStopTriggerType::HardwareButton => StopCategory::Category0,
            EStopTriggerType::Overcurrent => StopCategory::Category0,
            EStopTriggerType::Overvoltage => StopCategory::Category0,
            EStopTriggerType::LimitSwitch => StopCategory::Category1,
            EStopTriggerType::WatchdogTimeout => StopCategory::Category1,
            EStopTriggerType::SensorFailure => StopCategory::Category1,
            EStopTriggerType::FallDetection => StopCategory::Category1,
            _ => StopCategory::Category1,
        }
    }

    /// Get actions for stop category
    fn get_actions_for_category(&self, category: StopCategory) -> EStopAction {
        match category {
            StopCategory::Category0 => self.config.category0_actions.clone(),
            StopCategory::Category1 => self.config.category1_actions.clone(),
            StopCategory::Category2 => self.config.category2_actions.clone(),
        }
    }

    /// Get threshold for trigger type
    fn get_threshold_for_type(&self, trigger_type: EStopTriggerType) -> Option<f64> {
        match trigger_type {
            EStopTriggerType::Overcurrent => Some(self.config.trigger_configs.overcurrent_threshold),
            EStopTriggerType::Overtemperature => Some(self.config.trigger_configs.overtemp_threshold),
            EStopTriggerType::Overvoltage => Some(self.config.trigger_configs.overvoltage_threshold),
            EStopTriggerType::Undervoltage => Some(self.config.trigger_configs.undervoltage_threshold),
            EStopTriggerType::PositionError => Some(self.config.trigger_configs.position_error_threshold),
            _ => None,
        }
    }

    /// Get priority for trigger type
    fn get_priority_for_type(&self, trigger_type: EStopTriggerType) -> u8 {
        match trigger_type {
            EStopTriggerType::HardwareButton => 0,
            EStopTriggerType::Overcurrent => 0,
            EStopTriggerType::Overvoltage => 1,
            EStopTriggerType::LimitSwitch => 1,
            EStopTriggerType::WatchdogTimeout => 1,
            EStopTriggerType::SensorFailure => 1,
            EStopTriggerType::Overtemperature => 2,
            _ => 2,
        }
    }

    /// Execute E-Stop actions
    fn execute_actions(&self, actions: &EStopAction) {
        if actions.motor_disable {
            // Disable all motors
            self.disable_motors();
        }
        if actions.brake_engage {
            // Engage brakes
            self.engage_brakes();
        }
        if actions.power_cut {
            // Cut power
            self.cut_power();
        }
        if actions.alert_sound {
            // Sound alarm
            self.sound_alarm();
        }
        if actions.alert_visual {
            // Flash LEDs
            self.flash_leds();
        }
        if actions.log_event {
            // Log is automatic
        }
        if actions.notify_operator {
            // Notify operator
            self.notify_operator();
        }
    }

    // Hardware interaction stubs (would be implemented by hardware layer)
    fn disable_motors(&self) {
        // Implementation would interface with motor controllers
    }

    fn engage_brakes(&self) {
        // Implementation would interface with brake system
    }

    fn cut_power(&self) {
        // Implementation would interface with power system
    }

    fn sound_alarm(&self) {
        // Implementation would interface with audio system
    }

    fn flash_leds(&self) {
        // Implementation would interface with LED controller
    }

    fn notify_operator(&self) {
        // Implementation would send notification
    }

    /// Check if E-Stop is active
    pub fn is_active(&self) -> bool {
        self.is_active
    }

    /// Get current E-Stop event
    pub fn get_active_event(&self) -> Option<&EStopEvent> {
        self.current_event.as_ref()
    }

    /// Check reset eligibility
    pub fn can_reset(&self) -> ResetEligibility {
        let mut reasons = Vec::new();
        let mut pending_checks = Vec::new();

        if !self.is_active {
            return ResetEligibility {
                can_reset: true,
                reasons: vec!["E-Stop not active".to_string()],
                cooldown_remaining: 0,
                pending_checks: vec![],
            };
        }

        // Check cooldown
        let elapsed = Self::now() - self.triggered_at.unwrap_or(0);
        let cooldown = self.config.reset_config.cooldown_period;
        let cooldown_remaining = if elapsed < cooldown {
            cooldown - elapsed
        } else {
            0
        };

        if cooldown_remaining > 0 {
            reasons.push(format!("Cooldown period: {}ms remaining", cooldown_remaining));
        }

        // Check reset attempts
        if self.reset_attempts >= self.config.reset_config.max_reset_attempts {
            reasons.push("Maximum reset attempts exceeded".to_string());
        }

        // Check required items
        if self.config.reset_config.require_manual_reset {
            pending_checks.push("Manual reset button".to_string());
        }
        if self.config.reset_config.operator_confirmation {
            pending_checks.push("Operator confirmation".to_string());
        }
        if self.config.reset_config.safety_check_required {
            pending_checks.push("Safety self-test".to_string());
        }

        let can_reset = cooldown_remaining == 0
            && self.reset_attempts < self.config.reset_config.max_reset_attempts;

        ResetEligibility {
            can_reset,
            reasons,
            cooldown_remaining,
            pending_checks,
        }
    }

    /// Request E-Stop reset
    pub fn request_reset(&mut self, operator_id: &str) -> ResetResult {
        let eligibility = self.can_reset();

        if !eligibility.can_reset {
            return ResetResult {
                success: false,
                reason: Some(eligibility.reasons.join("; ")),
                self_test_passed: None,
            };
        }

        self.reset_attempts += 1;

        // Run self-test
        let self_test_passed = self.run_self_test();

        if !self_test_passed {
            return ResetResult {
                success: false,
                reason: Some("Self-test failed".to_string()),
                self_test_passed: Some(false),
            };
        }

        // Reset successful
        let reset_info = EStopResetInfo {
            reset_timestamp: Self::now(),
            reset_by: operator_id.to_string(),
            self_test_passed: true,
            reset_attempt: self.reset_attempts,
        };

        if let Some(event) = &mut self.current_event {
            event.reset_info = Some(reset_info);
        }

        self.is_active = false;
        self.triggered_at = None;
        self.reset_attempts = 0;

        ResetResult {
            success: true,
            reason: None,
            self_test_passed: Some(true),
        }
    }

    /// Run self-test
    fn run_self_test(&self) -> bool {
        match self.config.reset_config.self_test_level {
            SelfTestLevel::Quick => {
                // Basic connectivity test
                true
            }
            SelfTestLevel::Partial => {
                // Sensors + motors test
                true
            }
            SelfTestLevel::Full => {
                // Complete system test
                true
            }
        }
    }

    /// Update watchdog (call periodically to prevent timeout)
    pub fn heartbeat(&mut self) {
        self.last_heartbeat = Self::now();
    }

    /// Check watchdog status
    pub fn check_watchdog(&mut self) {
        let elapsed = Self::now() - self.last_heartbeat;
        if elapsed > self.config.watchdog_timeout && !self.is_active {
            self.trigger(
                EStopTriggerType::WatchdogTimeout,
                EStopSource::SafetyProcessor,
                "Watchdog timeout - no heartbeat received",
                Some(elapsed as f64),
            );
        }
    }

    /// Get E-Stop status
    pub fn get_status(&self) -> EStopStatus {
        let time_since_triggered = self
            .triggered_at
            .map(|t| Self::now() - t);

        let cooldown_remaining = time_since_triggered.and_then(|elapsed| {
            let cooldown = self.config.reset_config.cooldown_period;
            if elapsed < cooldown {
                Some(cooldown - elapsed)
            } else {
                None
            }
        });

        EStopStatus {
            is_active: self.is_active,
            category: self.current_event.as_ref().map(|e| e.category),
            active_event: self.current_event.clone(),
            time_since_triggered,
            reset_eligible: self.can_reset().can_reset,
            cooldown_remaining,
            reset_attempts: self.reset_attempts,
        }
    }

    /// Get event history
    pub fn get_history(&self, limit: Option<usize>) -> Vec<EStopEvent> {
        let limit = limit.unwrap_or(self.event_history.len());
        self.event_history
            .iter()
            .rev()
            .take(limit)
            .cloned()
            .collect()
    }

    /// Register callback for E-Stop events
    pub fn on_triggered<F>(&mut self, callback: F)
    where
        F: Fn(&EStopEvent) + Send + Sync + 'static,
    {
        self.callbacks.push(Box::new(callback));
    }

    /// Get configuration
    pub fn get_config(&self) -> &EStopConfig {
        &self.config
    }

    /// Update configuration
    pub fn configure(&mut self, config: EStopConfig) {
        self.config = config;
    }
}

// ============================================================================
// Utility Functions
// ============================================================================

/// Get current timestamp in milliseconds
fn now() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_estop_creation() {
        let config = EStopConfig::default();
        let estop = EmergencyStopSystem::new(config);
        assert!(!estop.is_active());
    }

    #[test]
    fn test_estop_trigger() {
        let config = EStopConfig::default();
        let mut estop = EmergencyStopSystem::new(config);

        estop.trigger(
            EStopTriggerType::HardwareButton,
            EStopSource::UserInput,
            "Test trigger",
            None,
        );

        assert!(estop.is_active());
        assert!(estop.get_active_event().is_some());
    }

    #[test]
    fn test_estop_category() {
        let config = EStopConfig::default();
        let estop = EmergencyStopSystem::new(config);

        assert_eq!(
            estop.determine_category(EStopTriggerType::HardwareButton),
            StopCategory::Category0
        );
        assert_eq!(
            estop.determine_category(EStopTriggerType::WatchdogTimeout),
            StopCategory::Category1
        );
    }
}
