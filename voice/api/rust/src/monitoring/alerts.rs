//! Alert definitions and management

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::Duration;

/// Alert severity level
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum AlertSeverity {
    Info,
    Warning,
    Critical,
}

/// Alert status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum AlertStatus {
    Firing,
    Resolved,
    Silenced,
}

/// Alert definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlertRule {
    /// Alert name
    pub name: String,

    /// Alert group
    pub group: String,

    /// Description
    pub description: String,

    /// Expression to evaluate
    pub expression: String,

    /// Duration the condition must be true
    pub duration: Duration,

    /// Severity
    pub severity: AlertSeverity,

    /// Labels
    pub labels: HashMap<String, String>,

    /// Annotations
    pub annotations: HashMap<String, String>,
}

/// Active alert instance
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Alert {
    /// Alert ID
    pub id: String,

    /// Alert name
    pub name: String,

    /// Severity
    pub severity: AlertSeverity,

    /// Status
    pub status: AlertStatus,

    /// Message
    pub message: String,

    /// Source
    pub source: String,

    /// Labels
    pub labels: HashMap<String, String>,

    /// Annotations
    pub annotations: HashMap<String, String>,

    /// Started at
    pub started_at: DateTime<Utc>,

    /// Ended at
    pub ended_at: Option<DateTime<Utc>>,

    /// Last evaluation
    pub last_evaluation: DateTime<Utc>,
}

impl Alert {
    /// Create a new firing alert
    pub fn new(
        id: impl Into<String>,
        name: impl Into<String>,
        severity: AlertSeverity,
        message: impl Into<String>,
        source: impl Into<String>,
    ) -> Self {
        let now = Utc::now();
        Self {
            id: id.into(),
            name: name.into(),
            severity,
            status: AlertStatus::Firing,
            message: message.into(),
            source: source.into(),
            labels: HashMap::new(),
            annotations: HashMap::new(),
            started_at: now,
            ended_at: None,
            last_evaluation: now,
        }
    }

    /// Add label
    pub fn with_label(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.labels.insert(key.into(), value.into());
        self
    }

    /// Add annotation
    pub fn with_annotation(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.annotations.insert(key.into(), value.into());
        self
    }

    /// Resolve the alert
    pub fn resolve(&mut self) {
        self.status = AlertStatus::Resolved;
        self.ended_at = Some(Utc::now());
    }

    /// Silence the alert
    pub fn silence(&mut self) {
        self.status = AlertStatus::Silenced;
    }

    /// Get duration
    pub fn duration(&self) -> chrono::Duration {
        let end = self.ended_at.unwrap_or_else(Utc::now);
        end - self.started_at
    }
}

/// Alert manager
pub struct AlertManager {
    alerts: HashMap<String, Alert>,
    rules: Vec<AlertRule>,
    notification_channels: Vec<Box<dyn NotificationChannel>>,
}

impl AlertManager {
    /// Create a new alert manager
    pub fn new() -> Self {
        Self {
            alerts: HashMap::new(),
            rules: Vec::new(),
            notification_channels: Vec::new(),
        }
    }

    /// Add an alert rule
    pub fn add_rule(&mut self, rule: AlertRule) {
        self.rules.push(rule);
    }

    /// Add a notification channel
    pub fn add_channel(&mut self, channel: Box<dyn NotificationChannel>) {
        self.notification_channels.push(channel);
    }

    /// Fire an alert
    pub fn fire(&mut self, alert: Alert) {
        let id = alert.id.clone();
        let severity = alert.severity;

        // Check if already firing
        if let Some(existing) = self.alerts.get(&id) {
            if existing.status == AlertStatus::Firing {
                return;
            }
        }

        // Store alert
        self.alerts.insert(id, alert.clone());

        // Send notifications
        for channel in &self.notification_channels {
            if channel.should_notify(&alert) {
                let _ = channel.send(&alert);
            }
        }
    }

    /// Resolve an alert
    pub fn resolve(&mut self, id: &str) {
        if let Some(alert) = self.alerts.get_mut(id) {
            alert.resolve();

            // Send resolution notification
            for channel in &self.notification_channels {
                if channel.should_notify(alert) {
                    let _ = channel.send_resolved(alert);
                }
            }
        }
    }

    /// Silence an alert
    pub fn silence(&mut self, id: &str, duration: Duration) {
        if let Some(alert) = self.alerts.get_mut(id) {
            alert.silence();
        }
    }

    /// Get active alerts
    pub fn active_alerts(&self) -> Vec<&Alert> {
        self.alerts
            .values()
            .filter(|a| a.status == AlertStatus::Firing)
            .collect()
    }

    /// Get alerts by severity
    pub fn alerts_by_severity(&self, severity: AlertSeverity) -> Vec<&Alert> {
        self.alerts
            .values()
            .filter(|a| a.severity == severity && a.status == AlertStatus::Firing)
            .collect()
    }

    /// Get all alerts
    pub fn all_alerts(&self) -> Vec<&Alert> {
        self.alerts.values().collect()
    }
}

impl Default for AlertManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Notification channel trait
pub trait NotificationChannel: Send + Sync {
    /// Channel name
    fn name(&self) -> &str;

    /// Check if should notify for this alert
    fn should_notify(&self, alert: &Alert) -> bool;

    /// Send alert notification
    fn send(&self, alert: &Alert) -> Result<(), NotificationError>;

    /// Send resolution notification
    fn send_resolved(&self, alert: &Alert) -> Result<(), NotificationError>;
}

/// Notification error
#[derive(Debug, thiserror::Error)]
pub enum NotificationError {
    #[error("Failed to send notification: {0}")]
    SendFailed(String),

    #[error("Channel not configured: {0}")]
    NotConfigured(String),

    #[error("Rate limited")]
    RateLimited,
}

/// Slack notification channel
pub struct SlackChannel {
    webhook_url: String,
    channel: String,
    min_severity: AlertSeverity,
}

impl SlackChannel {
    pub fn new(webhook_url: String, channel: String) -> Self {
        Self {
            webhook_url,
            channel,
            min_severity: AlertSeverity::Warning,
        }
    }

    pub fn with_min_severity(mut self, severity: AlertSeverity) -> Self {
        self.min_severity = severity;
        self
    }
}

impl NotificationChannel for SlackChannel {
    fn name(&self) -> &str {
        "slack"
    }

    fn should_notify(&self, alert: &Alert) -> bool {
        alert.severity >= self.min_severity
    }

    fn send(&self, alert: &Alert) -> Result<(), NotificationError> {
        // In production, send HTTP request to Slack webhook
        log::info!(
            "Slack notification: [{}] {} - {}",
            alert.severity as u8,
            alert.name,
            alert.message
        );
        Ok(())
    }

    fn send_resolved(&self, alert: &Alert) -> Result<(), NotificationError> {
        log::info!("Slack resolution: {} resolved", alert.name);
        Ok(())
    }
}

/// PagerDuty notification channel
pub struct PagerDutyChannel {
    service_key: String,
    min_severity: AlertSeverity,
}

impl PagerDutyChannel {
    pub fn new(service_key: String) -> Self {
        Self {
            service_key,
            min_severity: AlertSeverity::Critical,
        }
    }
}

impl NotificationChannel for PagerDutyChannel {
    fn name(&self) -> &str {
        "pagerduty"
    }

    fn should_notify(&self, alert: &Alert) -> bool {
        alert.severity >= self.min_severity
    }

    fn send(&self, alert: &Alert) -> Result<(), NotificationError> {
        // In production, send to PagerDuty API
        log::info!(
            "PagerDuty alert: [{}] {} - {}",
            alert.severity as u8,
            alert.name,
            alert.message
        );
        Ok(())
    }

    fn send_resolved(&self, alert: &Alert) -> Result<(), NotificationError> {
        log::info!("PagerDuty resolution: {} resolved", alert.name);
        Ok(())
    }
}

/// Pre-defined alert rules for Voice-Sign
pub fn voice_sign_alert_rules() -> Vec<AlertRule> {
    vec![
        // Availability alerts
        AlertRule {
            name: "VoiceSignApiDown".to_string(),
            group: "voice-sign-availability".to_string(),
            description: "Voice-Sign API is down".to_string(),
            expression: "up{job=\"voice-sign-api\"} == 0".to_string(),
            duration: Duration::from_secs(60),
            severity: AlertSeverity::Critical,
            labels: HashMap::new(),
            annotations: {
                let mut m = HashMap::new();
                m.insert(
                    "summary".to_string(),
                    "Voice-Sign API is down".to_string(),
                );
                m
            },
        },
        AlertRule {
            name: "VoiceSignHighErrorRate".to_string(),
            group: "voice-sign-availability".to_string(),
            description: "High error rate detected".to_string(),
            expression: "sum(rate(voice_sign_http_requests_total{status=~\"5..\"}[5m])) / sum(rate(voice_sign_http_requests_total[5m])) > 0.01".to_string(),
            duration: Duration::from_secs(300),
            severity: AlertSeverity::Critical,
            labels: HashMap::new(),
            annotations: {
                let mut m = HashMap::new();
                m.insert("summary".to_string(), "High error rate detected".to_string());
                m
            },
        },
        // Latency alerts
        AlertRule {
            name: "VoiceSignHighLatency".to_string(),
            group: "voice-sign-latency".to_string(),
            description: "High latency detected".to_string(),
            expression: "histogram_quantile(0.99, sum(rate(voice_sign_http_request_duration_seconds_bucket[5m])) by (le)) > 1".to_string(),
            duration: Duration::from_secs(300),
            severity: AlertSeverity::Warning,
            labels: HashMap::new(),
            annotations: {
                let mut m = HashMap::new();
                m.insert("summary".to_string(), "High latency detected".to_string());
                m
            },
        },
        // Quality alerts
        AlertRule {
            name: "VoiceSignLowQualityScore".to_string(),
            group: "voice-sign-quality".to_string(),
            description: "Translation quality is degraded".to_string(),
            expression: "histogram_quantile(0.5, sum(rate(voice_sign_translation_quality_score_bucket[1h])) by (le)) < 0.8".to_string(),
            duration: Duration::from_secs(1800),
            severity: AlertSeverity::Warning,
            labels: HashMap::new(),
            annotations: {
                let mut m = HashMap::new();
                m.insert("summary".to_string(), "Translation quality is degraded".to_string());
                m
            },
        },
        // Resource alerts
        AlertRule {
            name: "VoiceSignHighCpuUsage".to_string(),
            group: "voice-sign-resources".to_string(),
            description: "High CPU usage".to_string(),
            expression: "voice_sign_cpu_usage_percent > 85".to_string(),
            duration: Duration::from_secs(600),
            severity: AlertSeverity::Warning,
            labels: HashMap::new(),
            annotations: {
                let mut m = HashMap::new();
                m.insert("summary".to_string(), "High CPU usage".to_string());
                m
            },
        },
        // Emergency alert
        AlertRule {
            name: "VoiceSignEmergencyBacklog".to_string(),
            group: "voice-sign-emergency".to_string(),
            description: "Emergency translations queued".to_string(),
            expression: "voice_sign_translation_queue_depth{priority=\"emergency\"} > 0".to_string(),
            duration: Duration::from_secs(30),
            severity: AlertSeverity::Critical,
            labels: HashMap::new(),
            annotations: {
                let mut m = HashMap::new();
                m.insert("summary".to_string(), "Emergency translations waiting".to_string());
                m
            },
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_alert_creation() {
        let alert = Alert::new(
            "alert-001",
            "TestAlert",
            AlertSeverity::Warning,
            "Test message",
            "test",
        )
        .with_label("service", "voice-sign");

        assert_eq!(alert.status, AlertStatus::Firing);
        assert!(alert.labels.contains_key("service"));
    }

    #[test]
    fn test_alert_resolve() {
        let mut alert = Alert::new(
            "alert-001",
            "TestAlert",
            AlertSeverity::Warning,
            "Test message",
            "test",
        );

        alert.resolve();
        assert_eq!(alert.status, AlertStatus::Resolved);
        assert!(alert.ended_at.is_some());
    }

    #[test]
    fn test_alert_manager() {
        let mut manager = AlertManager::new();

        let alert = Alert::new(
            "alert-001",
            "TestAlert",
            AlertSeverity::Critical,
            "Test message",
            "test",
        );

        manager.fire(alert);
        assert_eq!(manager.active_alerts().len(), 1);

        manager.resolve("alert-001");
        assert_eq!(manager.active_alerts().len(), 0);
    }

    #[test]
    fn test_voice_sign_rules() {
        let rules = voice_sign_alert_rules();
        assert!(!rules.is_empty());

        let critical_rules: Vec<_> = rules
            .iter()
            .filter(|r| r.severity == AlertSeverity::Critical)
            .collect();
        assert!(!critical_rules.is_empty());
    }
}
