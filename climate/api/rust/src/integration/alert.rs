//! Alert rule engine for threshold-based notifications

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::HashMap;
use std::time::Duration;

use crate::core::climate::ClimateMessage;
use crate::types::{DataType, Device};

/// Comparison operator for alert conditions
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ComparisonOperator {
    /// Less than (<)
    LessThan,
    /// Less than or equal (<=)
    LessOrEqual,
    /// Greater than (>)
    GreaterThan,
    /// Greater than or equal (>=)
    GreaterOrEqual,
    /// Equal (==)
    Equal,
    /// Not equal (!=)
    NotEqual,
}

impl ComparisonOperator {
    /// Evaluate the comparison
    pub fn evaluate(&self, value: f64, threshold: f64) -> bool {
        match self {
            ComparisonOperator::LessThan => value < threshold,
            ComparisonOperator::LessOrEqual => value <= threshold,
            ComparisonOperator::GreaterThan => value > threshold,
            ComparisonOperator::GreaterOrEqual => value >= threshold,
            ComparisonOperator::Equal => (value - threshold).abs() < f64::EPSILON,
            ComparisonOperator::NotEqual => (value - threshold).abs() >= f64::EPSILON,
        }
    }
}

impl std::fmt::Display for ComparisonOperator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ComparisonOperator::LessThan => write!(f, "<"),
            ComparisonOperator::LessOrEqual => write!(f, "<="),
            ComparisonOperator::GreaterThan => write!(f, ">"),
            ComparisonOperator::GreaterOrEqual => write!(f, ">="),
            ComparisonOperator::Equal => write!(f, "=="),
            ComparisonOperator::NotEqual => write!(f, "!="),
        }
    }
}

/// Alert condition definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlertCondition {
    /// Field path to evaluate (dot notation for nested fields)
    pub field: String,
    /// Comparison operator
    pub operator: ComparisonOperator,
    /// Threshold value
    pub threshold: f64,
}

impl AlertCondition {
    /// Create a new alert condition
    pub fn new(field: impl Into<String>, operator: ComparisonOperator, threshold: f64) -> Self {
        Self {
            field: field.into(),
            operator,
            threshold,
        }
    }

    /// Evaluate the condition against a message
    pub fn evaluate(&self, message: &ClimateMessage) -> Option<bool> {
        let value = self.extract_field_value(message)?;
        Some(self.operator.evaluate(value, self.threshold))
    }

    /// Extract field value from message using dot notation
    fn extract_field_value(&self, message: &ClimateMessage) -> Option<f64> {
        // Convert message to JSON for field extraction
        let json = serde_json::to_value(message).ok()?;
        self.extract_from_json(&json, &self.field)
    }

    fn extract_from_json(&self, json: &Value, path: &str) -> Option<f64> {
        let parts: Vec<&str> = path.split('.').collect();
        let mut current = json;

        for part in parts {
            current = current.get(part)?;
        }

        current.as_f64()
    }
}

/// Alert severity level
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AlertSeverity {
    /// Informational alert
    Info,
    /// Warning alert
    Warning,
    /// Critical alert
    Critical,
}

impl Default for AlertSeverity {
    fn default() -> Self {
        AlertSeverity::Warning
    }
}

impl std::fmt::Display for AlertSeverity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AlertSeverity::Info => write!(f, "info"),
            AlertSeverity::Warning => write!(f, "warning"),
            AlertSeverity::Critical => write!(f, "critical"),
        }
    }
}

/// Current state of an alert
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AlertStateValue {
    /// Condition not met
    Inactive,
    /// Condition met, waiting for duration
    Pending,
    /// Alert is active
    Firing,
    /// Alert was active but is now resolved
    Resolved,
}

impl Default for AlertStateValue {
    fn default() -> Self {
        AlertStateValue::Inactive
    }
}

/// Alert rule definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlertRule {
    /// Unique rule identifier
    pub id: String,
    /// Human-readable name
    pub name: String,
    /// Rule description
    pub description: Option<String>,
    /// Data type filter (None = all types)
    pub data_type: Option<DataType>,
    /// Device filter (glob pattern)
    pub device_filter: Option<String>,
    /// Alert condition
    pub condition: AlertCondition,
    /// How long condition must be true before firing
    #[serde(with = "duration_serde")]
    pub duration: Duration,
    /// Alert severity
    pub severity: AlertSeverity,
    /// Notification target adapter names
    pub notifications: Vec<String>,
    /// Is rule enabled
    pub enabled: bool,
}

mod duration_serde {
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    use std::time::Duration;

    pub fn serialize<S>(duration: &Duration, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        duration.as_secs().serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Duration, D::Error>
    where
        D: Deserializer<'de>,
    {
        let secs = u64::deserialize(deserializer)?;
        Ok(Duration::from_secs(secs))
    }
}

impl AlertRule {
    /// Create a new alert rule builder
    pub fn builder(id: impl Into<String>, name: impl Into<String>) -> AlertRuleBuilder {
        AlertRuleBuilder::new(id, name)
    }

    /// Check if this rule matches the given message
    pub fn matches(&self, message: &ClimateMessage) -> bool {
        // Check data type filter
        if let Some(ref dt) = self.data_type {
            if &message.data_type != dt {
                return false;
            }
        }

        // Check device filter (simple glob matching)
        if let Some(ref filter) = self.device_filter {
            let device_id = get_device_id(&message.device);
            if !glob_match(filter, &device_id) {
                return false;
            }
        }

        true
    }
}

/// Builder for AlertRule
pub struct AlertRuleBuilder {
    rule: AlertRule,
}

impl AlertRuleBuilder {
    /// Create a new builder
    pub fn new(id: impl Into<String>, name: impl Into<String>) -> Self {
        Self {
            rule: AlertRule {
                id: id.into(),
                name: name.into(),
                description: None,
                data_type: None,
                device_filter: None,
                condition: AlertCondition::new("value", ComparisonOperator::GreaterThan, 0.0),
                duration: Duration::from_secs(60),
                severity: AlertSeverity::Warning,
                notifications: Vec::new(),
                enabled: true,
            },
        }
    }

    /// Set description
    pub fn description(mut self, desc: impl Into<String>) -> Self {
        self.rule.description = Some(desc.into());
        self
    }

    /// Set data type filter
    pub fn data_type(mut self, dt: DataType) -> Self {
        self.rule.data_type = Some(dt);
        self
    }

    /// Set device filter
    pub fn device_filter(mut self, filter: impl Into<String>) -> Self {
        self.rule.device_filter = Some(filter.into());
        self
    }

    /// Set condition
    pub fn condition(mut self, field: impl Into<String>, op: ComparisonOperator, threshold: f64) -> Self {
        self.rule.condition = AlertCondition::new(field, op, threshold);
        self
    }

    /// Set duration before firing
    pub fn duration(mut self, duration: Duration) -> Self {
        self.rule.duration = duration;
        self
    }

    /// Set duration in seconds
    pub fn duration_secs(mut self, secs: u64) -> Self {
        self.rule.duration = Duration::from_secs(secs);
        self
    }

    /// Set severity
    pub fn severity(mut self, severity: AlertSeverity) -> Self {
        self.rule.severity = severity;
        self
    }

    /// Add notification target
    pub fn notify(mut self, adapter_name: impl Into<String>) -> Self {
        self.rule.notifications.push(adapter_name.into());
        self
    }

    /// Set enabled state
    pub fn enabled(mut self, enabled: bool) -> Self {
        self.rule.enabled = enabled;
        self
    }

    /// Build the alert rule
    pub fn build(self) -> AlertRule {
        self.rule
    }
}

/// Runtime state for an alert rule
#[derive(Debug, Clone)]
pub struct AlertState {
    /// Alert rule ID
    pub rule_id: String,
    /// Current state
    pub state: AlertStateValue,
    /// When condition was first met
    pub pending_since: Option<DateTime<Utc>>,
    /// When alert started firing
    pub firing_since: Option<DateTime<Utc>>,
    /// When alert was resolved
    pub resolved_at: Option<DateTime<Utc>>,
    /// Last evaluated value
    pub last_value: Option<f64>,
    /// Number of notifications sent
    pub notifications_sent: u32,
}

impl AlertState {
    /// Create a new inactive state
    pub fn new(rule_id: impl Into<String>) -> Self {
        Self {
            rule_id: rule_id.into(),
            state: AlertStateValue::Inactive,
            pending_since: None,
            firing_since: None,
            resolved_at: None,
            last_value: None,
            notifications_sent: 0,
        }
    }
}

/// Alert event generated when an alert fires or resolves
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlertEvent {
    /// Unique event ID
    pub event_id: String,
    /// Event type
    pub event_type: AlertEventType,
    /// Timestamp
    pub timestamp: DateTime<Utc>,
    /// Rule that generated this event
    pub rule_id: String,
    /// Rule name
    pub rule_name: String,
    /// Alert severity
    pub severity: AlertSeverity,
    /// Condition details
    pub condition: AlertCondition,
    /// Current value that triggered the alert
    pub current_value: f64,
    /// Source device info
    pub source_device_id: String,
    /// Source location
    pub source_location: Option<(f64, f64)>,
    /// Data type
    pub data_type: DataType,
}

/// Type of alert event
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AlertEventType {
    /// Alert has started firing
    Firing,
    /// Alert has been resolved
    Resolved,
}

/// Alert rule engine for evaluating rules against messages
pub struct AlertEngine {
    /// Registered rules
    rules: Vec<AlertRule>,
    /// Rule states
    states: HashMap<String, AlertState>,
}

impl AlertEngine {
    /// Create a new alert engine
    pub fn new() -> Self {
        Self {
            rules: Vec::new(),
            states: HashMap::new(),
        }
    }

    /// Add a rule to the engine
    pub fn add_rule(&mut self, rule: AlertRule) {
        let rule_id = rule.id.clone();
        self.rules.push(rule);
        self.states.insert(rule_id.clone(), AlertState::new(rule_id));
    }

    /// Remove a rule by ID
    pub fn remove_rule(&mut self, rule_id: &str) -> bool {
        if let Some(idx) = self.rules.iter().position(|r| r.id == rule_id) {
            self.rules.remove(idx);
            self.states.remove(rule_id);
            true
        } else {
            false
        }
    }

    /// Get all rules
    pub fn rules(&self) -> &[AlertRule] {
        &self.rules
    }

    /// Get rule by ID
    pub fn get_rule(&self, rule_id: &str) -> Option<&AlertRule> {
        self.rules.iter().find(|r| r.id == rule_id)
    }

    /// Get state for a rule
    pub fn get_state(&self, rule_id: &str) -> Option<&AlertState> {
        self.states.get(rule_id)
    }

    /// Evaluate all rules against a message
    ///
    /// Returns a list of alert events that were generated.
    pub fn evaluate(&mut self, message: &ClimateMessage) -> Vec<AlertEvent> {
        let mut events = Vec::new();
        let now = Utc::now();

        for rule in &self.rules {
            if !rule.enabled {
                continue;
            }

            if !rule.matches(message) {
                continue;
            }

            let condition_met = rule.condition.evaluate(message).unwrap_or(false);
            let current_value = rule.condition.extract_field_value(message);

            let state = self.states.get_mut(&rule.id).unwrap();
            state.last_value = current_value;

            match (state.state, condition_met) {
                (AlertStateValue::Inactive, true) => {
                    // Start pending
                    state.state = AlertStateValue::Pending;
                    state.pending_since = Some(now);
                }
                (AlertStateValue::Pending, true) => {
                    // Check if duration has elapsed
                    if let Some(pending_since) = state.pending_since {
                        let elapsed = now.signed_duration_since(pending_since);
                        if elapsed >= chrono::Duration::from_std(rule.duration).unwrap_or_default() {
                            // Fire the alert
                            state.state = AlertStateValue::Firing;
                            state.firing_since = Some(now);
                            state.notifications_sent += 1;

                            events.push(AlertEvent {
                                event_id: uuid::Uuid::new_v4().to_string(),
                                event_type: AlertEventType::Firing,
                                timestamp: now,
                                rule_id: rule.id.clone(),
                                rule_name: rule.name.clone(),
                                severity: rule.severity,
                                condition: rule.condition.clone(),
                                current_value: current_value.unwrap_or(0.0),
                                source_device_id: get_device_id(&message.device),
                                source_location: Some((message.location.latitude, message.location.longitude)),
                                data_type: message.data_type.clone(),
                            });
                        }
                    }
                }
                (AlertStateValue::Firing, true) => {
                    // Still firing, no event
                }
                (AlertStateValue::Pending, false) | (AlertStateValue::Inactive, false) => {
                    // Reset to inactive
                    state.state = AlertStateValue::Inactive;
                    state.pending_since = None;
                }
                (AlertStateValue::Firing, false) | (AlertStateValue::Resolved, false) => {
                    // Resolve the alert
                    let was_firing = state.state == AlertStateValue::Firing;
                    state.state = AlertStateValue::Resolved;
                    state.resolved_at = Some(now);

                    if was_firing {
                        events.push(AlertEvent {
                            event_id: uuid::Uuid::new_v4().to_string(),
                            event_type: AlertEventType::Resolved,
                            timestamp: now,
                            rule_id: rule.id.clone(),
                            rule_name: rule.name.clone(),
                            severity: rule.severity,
                            condition: rule.condition.clone(),
                            current_value: current_value.unwrap_or(0.0),
                            source_device_id: get_device_id(&message.device),
                            source_location: Some((message.location.latitude, message.location.longitude)),
                            data_type: message.data_type.clone(),
                        });
                    }
                }
                (AlertStateValue::Resolved, true) => {
                    // Start pending again
                    state.state = AlertStateValue::Pending;
                    state.pending_since = Some(now);
                    state.resolved_at = None;
                }
            }
        }

        events
    }

    /// Get all currently firing alerts
    pub fn firing_alerts(&self) -> Vec<&AlertState> {
        self.states
            .values()
            .filter(|s| s.state == AlertStateValue::Firing)
            .collect()
    }

    /// Reset all alert states
    pub fn reset(&mut self) {
        for state in self.states.values_mut() {
            state.state = AlertStateValue::Inactive;
            state.pending_since = None;
            state.firing_since = None;
            state.resolved_at = None;
            state.last_value = None;
        }
    }
}

impl Default for AlertEngine {
    fn default() -> Self {
        Self::new()
    }
}

/// Get a device identifier from the Device struct
///
/// Uses serial number if available, otherwise combines manufacturer and model.
fn get_device_id(device: &Device) -> String {
    device.serial.clone().unwrap_or_else(|| {
        format!("{}-{}", device.manufacturer, device.model)
    })
}

/// Simple glob pattern matching (supports * and ?)
fn glob_match(pattern: &str, text: &str) -> bool {
    let pattern_chars: Vec<char> = pattern.chars().collect();
    let text_chars: Vec<char> = text.chars().collect();

    fn match_impl(p: &[char], t: &[char]) -> bool {
        match (p.first(), t.first()) {
            (None, None) => true,
            (Some('*'), _) => {
                // * matches zero or more characters
                match_impl(&p[1..], t) || (!t.is_empty() && match_impl(p, &t[1..]))
            }
            (Some('?'), Some(_)) => {
                // ? matches exactly one character
                match_impl(&p[1..], &t[1..])
            }
            (Some(pc), Some(tc)) if pc == tc => match_impl(&p[1..], &t[1..]),
            _ => false,
        }
    }

    match_impl(&pattern_chars, &text_chars)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_comparison_operators() {
        assert!(ComparisonOperator::LessThan.evaluate(5.0, 10.0));
        assert!(!ComparisonOperator::LessThan.evaluate(10.0, 5.0));

        assert!(ComparisonOperator::GreaterThan.evaluate(10.0, 5.0));
        assert!(!ComparisonOperator::GreaterThan.evaluate(5.0, 10.0));

        assert!(ComparisonOperator::Equal.evaluate(5.0, 5.0));
        assert!(!ComparisonOperator::Equal.evaluate(5.0, 5.1));
    }

    #[test]
    fn test_alert_condition() {
        let condition = AlertCondition::new("value", ComparisonOperator::GreaterThan, 100.0);
        assert_eq!(condition.field, "value");
        assert_eq!(condition.threshold, 100.0);
    }

    #[test]
    fn test_alert_rule_builder() {
        let rule = AlertRule::builder("test-rule", "Test Rule")
            .description("A test alert rule")
            .data_type(DataType::CarbonCapture)
            .condition("capture_rate", ComparisonOperator::LessThan, 50.0)
            .duration_secs(300)
            .severity(AlertSeverity::Critical)
            .notify("webhook-1")
            .build();

        assert_eq!(rule.id, "test-rule");
        assert_eq!(rule.name, "Test Rule");
        assert_eq!(rule.severity, AlertSeverity::Critical);
        assert_eq!(rule.duration, Duration::from_secs(300));
        assert!(rule.enabled);
    }

    #[test]
    fn test_glob_match() {
        assert!(glob_match("*", "anything"));
        assert!(glob_match("test*", "testing"));
        assert!(glob_match("*test", "mytest"));
        assert!(glob_match("te?t", "test"));
        assert!(glob_match("te?t", "text"));
        assert!(!glob_match("te?t", "teest"));
        assert!(glob_match("sensor-*", "sensor-001"));
        assert!(!glob_match("sensor-*", "device-001"));
    }

    #[test]
    fn test_alert_severity_ordering() {
        assert!(AlertSeverity::Info < AlertSeverity::Warning);
        assert!(AlertSeverity::Warning < AlertSeverity::Critical);
    }

    #[test]
    fn test_alert_engine_creation() {
        let mut engine = AlertEngine::new();
        assert!(engine.rules().is_empty());

        let rule = AlertRule::builder("test", "Test")
            .condition("value", ComparisonOperator::GreaterThan, 100.0)
            .build();

        engine.add_rule(rule);
        assert_eq!(engine.rules().len(), 1);
        assert!(engine.get_rule("test").is_some());
        assert!(engine.get_state("test").is_some());
    }
}
