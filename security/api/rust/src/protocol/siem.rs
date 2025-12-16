//! SIEM Integration Protocol
//!
//! Integration with Security Information and Event Management systems.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use uuid::Uuid;
use chrono::Utc;

// ============================================================================
// SIEM Types
// ============================================================================

/// SIEM vendor type
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SiemVendor {
    Splunk,
    Elastic,
    Chronicle,
    Sentinel,
    QRadar,
    Sumo,
    Custom,
}

/// Output format for SIEM integration
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SiemOutputFormat {
    WiaNative,
    SplunkHec,
    ElasticEcs,
    Chronicle,
    Syslog,
    Cef,
    Leef,
    Custom,
}

// ============================================================================
// Event Subscription
// ============================================================================

/// Delivery method for events
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeliveryMethod {
    Webhook,
    WebSocket,
    Kafka,
    Sqs,
    PubSub,
    Kinesis,
    EventHub,
}

/// Event filter configuration
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EventFilter {
    /// Minimum severity (0-10)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub severity_min: Option<f64>,
    /// Priority levels to include
    #[serde(skip_serializing_if = "Option::is_none")]
    pub priorities: Option<Vec<String>>,
    /// Event types to include
    #[serde(skip_serializing_if = "Option::is_none")]
    pub event_types: Option<Vec<String>>,
    /// Categories to include
    #[serde(skip_serializing_if = "Option::is_none")]
    pub categories: Option<Vec<String>>,
    /// Source types to include
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source_types: Option<Vec<String>>,
    /// Custom filter expressions
    #[serde(skip_serializing_if = "Option::is_none")]
    pub custom: Option<HashMap<String, serde_json::Value>>,
}

impl EventFilter {
    /// Create filter for critical and high priority
    pub fn critical_high() -> Self {
        Self {
            severity_min: Some(7.0),
            priorities: Some(vec!["critical".to_string(), "high".to_string()]),
            ..Default::default()
        }
    }

    /// Create filter for specific event types
    pub fn for_types(types: Vec<String>) -> Self {
        Self {
            event_types: Some(types),
            ..Default::default()
        }
    }
}

/// Delivery configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeliveryConfig {
    /// Delivery method
    pub method: DeliveryMethod,
    /// Endpoint URL
    pub endpoint: String,
    /// Output format
    pub format: SiemOutputFormat,
    /// Batch size
    #[serde(skip_serializing_if = "Option::is_none")]
    pub batch_size: Option<u32>,
    /// Flush interval in seconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub flush_interval_seconds: Option<u32>,
    /// Authentication
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auth: Option<DeliveryAuth>,
    /// Custom headers
    #[serde(skip_serializing_if = "Option::is_none")]
    pub headers: Option<HashMap<String, String>>,
}

/// Authentication for delivery
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeliveryAuth {
    /// Auth type
    #[serde(rename = "type")]
    pub auth_type: String,
    /// Token or credentials
    #[serde(skip_serializing_if = "Option::is_none")]
    pub token: Option<String>,
    /// Username
    #[serde(skip_serializing_if = "Option::is_none")]
    pub username: Option<String>,
    /// Password (should be encrypted)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub password: Option<String>,
}

impl DeliveryAuth {
    /// Create bearer token auth
    pub fn bearer(token: impl Into<String>) -> Self {
        Self {
            auth_type: "bearer".to_string(),
            token: Some(token.into()),
            username: None,
            password: None,
        }
    }

    /// Create basic auth
    pub fn basic(username: impl Into<String>, password: impl Into<String>) -> Self {
        Self {
            auth_type: "basic".to_string(),
            token: None,
            username: Some(username.into()),
            password: Some(password.into()),
        }
    }
}

/// Retry policy
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RetryPolicy {
    /// Maximum retry attempts
    pub max_retries: u32,
    /// Backoff type
    pub backoff_type: String,
    /// Initial delay in milliseconds
    pub initial_delay_ms: u64,
    /// Maximum delay in milliseconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_delay_ms: Option<u64>,
}

impl Default for RetryPolicy {
    fn default() -> Self {
        Self {
            max_retries: 3,
            backoff_type: "exponential".to_string(),
            initial_delay_ms: 1000,
            max_delay_ms: Some(30000),
        }
    }
}

/// Event subscription
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EventSubscription {
    /// Subscription ID
    pub id: String,
    /// Subscription name
    pub name: String,
    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Source types to subscribe
    pub source_types: Vec<String>,
    /// Event types to subscribe
    pub event_types: Vec<String>,
    /// Filters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub filters: Option<EventFilter>,
    /// Delivery configuration
    pub delivery: DeliveryConfig,
    /// Retry policy
    #[serde(skip_serializing_if = "Option::is_none")]
    pub retry_policy: Option<RetryPolicy>,
    /// Enabled status
    pub enabled: bool,
    /// Created at
    #[serde(skip_serializing_if = "Option::is_none")]
    pub created_at: Option<String>,
    /// Updated at
    #[serde(skip_serializing_if = "Option::is_none")]
    pub updated_at: Option<String>,
}

impl EventSubscription {
    /// Create a new subscription
    pub fn new(
        name: impl Into<String>,
        delivery: DeliveryConfig,
    ) -> Self {
        Self {
            id: Uuid::new_v4().to_string(),
            name: name.into(),
            description: None,
            source_types: vec![],
            event_types: vec![],
            filters: None,
            delivery,
            retry_policy: Some(RetryPolicy::default()),
            enabled: true,
            created_at: Some(Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string()),
            updated_at: None,
        }
    }

    /// Set source types
    pub fn with_sources(mut self, sources: Vec<String>) -> Self {
        self.source_types = sources;
        self
    }

    /// Set event types
    pub fn with_events(mut self, events: Vec<String>) -> Self {
        self.event_types = events;
        self
    }

    /// Set filters
    pub fn with_filters(mut self, filters: EventFilter) -> Self {
        self.filters = Some(filters);
        self
    }
}

// ============================================================================
// SIEM Output Formats
// ============================================================================

/// Splunk HEC format
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SplunkHecEvent {
    /// Event timestamp (Unix epoch)
    pub time: i64,
    /// Host
    pub host: String,
    /// Source
    pub source: String,
    /// Source type
    pub sourcetype: String,
    /// Index
    #[serde(skip_serializing_if = "Option::is_none")]
    pub index: Option<String>,
    /// Event data
    pub event: serde_json::Value,
}

impl SplunkHecEvent {
    /// Create from WIA event
    pub fn from_wia_event(
        event: &serde_json::Value,
        host: impl Into<String>,
        index: Option<String>,
    ) -> Self {
        let timestamp = event
            .get("timestamp")
            .and_then(|t| t.as_str())
            .and_then(|t| chrono::DateTime::parse_from_rfc3339(t).ok())
            .map(|dt| dt.timestamp())
            .unwrap_or_else(|| Utc::now().timestamp());

        let event_type = event
            .get("type")
            .and_then(|t| t.as_str())
            .unwrap_or("unknown");

        Self {
            time: timestamp,
            host: host.into(),
            source: "wia-security".to_string(),
            sourcetype: format!("wia:{}", event_type),
            index,
            event: serde_json::json!({
                "wia_event": event
            }),
        }
    }
}

/// Elastic ECS format
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElasticEcsEvent {
    /// Timestamp
    #[serde(rename = "@timestamp")]
    pub timestamp: String,
    /// ECS version
    pub ecs: EcsVersion,
    /// Event metadata
    pub event: EcsEventMeta,
    /// WIA specific fields
    pub wia: serde_json::Value,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcsVersion {
    pub version: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcsEventMeta {
    pub kind: String,
    pub category: Vec<String>,
    #[serde(rename = "type")]
    pub event_type: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub severity: Option<u32>,
}

impl ElasticEcsEvent {
    /// Create from WIA event
    pub fn from_wia_event(event: &serde_json::Value) -> Self {
        let timestamp = event
            .get("timestamp")
            .and_then(|t| t.as_str())
            .unwrap_or(&Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string())
            .to_string();

        let severity = event
            .get("severity")
            .and_then(|s| s.as_f64())
            .map(|s| s as u32);

        let event_type = event
            .get("type")
            .and_then(|t| t.as_str())
            .unwrap_or("alert");

        let (kind, category) = match event_type {
            "alert" => ("alert", vec!["intrusion_detection"]),
            "threat_intel" => ("enrichment", vec!["threat"]),
            "vulnerability" => ("state", vec!["vulnerability"]),
            "incident" => ("alert", vec!["intrusion_detection"]),
            _ => ("event", vec!["process"]),
        };

        Self {
            timestamp,
            ecs: EcsVersion {
                version: "8.0.0".to_string(),
            },
            event: EcsEventMeta {
                kind: kind.to_string(),
                category: category.into_iter().map(|s| s.to_string()).collect(),
                event_type: vec!["info".to_string()],
                severity,
            },
            wia: event.clone(),
        }
    }
}

/// CEF (Common Event Format)
#[derive(Debug, Clone)]
pub struct CefEvent {
    /// CEF version
    pub version: u32,
    /// Vendor
    pub vendor: String,
    /// Product
    pub product: String,
    /// Product version
    pub product_version: String,
    /// Signature ID
    pub signature_id: String,
    /// Event name
    pub name: String,
    /// Severity (0-10)
    pub severity: u32,
    /// Extension fields
    pub extensions: HashMap<String, String>,
}

impl CefEvent {
    /// Create from WIA event
    pub fn from_wia_event(event: &serde_json::Value) -> Self {
        let mut extensions = HashMap::new();

        if let Some(id) = event.get("id").and_then(|v| v.as_str()) {
            extensions.insert("externalId".to_string(), id.to_string());
        }

        if let Some(timestamp) = event.get("timestamp").and_then(|v| v.as_str()) {
            extensions.insert("rt".to_string(), timestamp.to_string());
        }

        let event_type = event
            .get("type")
            .and_then(|t| t.as_str())
            .unwrap_or("unknown");

        let severity = event
            .get("severity")
            .and_then(|s| s.as_f64())
            .map(|s| s as u32)
            .unwrap_or(5);

        Self {
            version: 0,
            vendor: "WIA".to_string(),
            product: "WIA-Security".to_string(),
            product_version: "1.0.0".to_string(),
            signature_id: event_type.to_string(),
            name: event_type.to_string(),
            severity,
            extensions,
        }
    }

    /// Format as CEF string
    pub fn to_cef_string(&self) -> String {
        let ext_str: String = self
            .extensions
            .iter()
            .map(|(k, v)| format!("{}={}", k, v.replace('\\', "\\\\").replace('=', "\\=")))
            .collect::<Vec<_>>()
            .join(" ");

        format!(
            "CEF:{}|{}|{}|{}|{}|{}|{}|{}",
            self.version,
            self.vendor,
            self.product,
            self.product_version,
            self.signature_id,
            self.name,
            self.severity,
            ext_str
        )
    }
}

// ============================================================================
// Subscription Status
// ============================================================================

/// Subscription status
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SubscriptionStatus {
    Active,
    Paused,
    Failed,
    Cancelled,
}

/// Subscription statistics
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SubscriptionStats {
    /// Total events delivered
    pub events_delivered: u64,
    /// Total events failed
    pub events_failed: u64,
    /// Last delivery timestamp
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_delivery_at: Option<String>,
    /// Last error
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_error: Option<String>,
    /// Last error timestamp
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_error_at: Option<String>,
    /// Average delivery latency (ms)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub avg_latency_ms: Option<f64>,
}

/// Subscription response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubscriptionResponse {
    /// Subscription ID
    pub subscription_id: String,
    /// Status
    pub status: SubscriptionStatus,
    /// Channels subscribed
    pub channels: Vec<String>,
    /// Expires at
    #[serde(skip_serializing_if = "Option::is_none")]
    pub expires_at: Option<String>,
    /// Statistics
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stats: Option<SubscriptionStats>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_event_subscription() {
        let delivery = DeliveryConfig {
            method: DeliveryMethod::Webhook,
            endpoint: "https://siem.example.com/api/v1/events".to_string(),
            format: SiemOutputFormat::WiaNative,
            batch_size: Some(100),
            flush_interval_seconds: Some(5),
            auth: Some(DeliveryAuth::bearer("token123")),
            headers: None,
        };

        let subscription = EventSubscription::new("Critical Alerts", delivery)
            .with_sources(vec!["scanner".to_string(), "edr".to_string()])
            .with_events(vec!["alert".to_string(), "incident".to_string()])
            .with_filters(EventFilter::critical_high());

        assert!(subscription.enabled);
        assert_eq!(subscription.source_types.len(), 2);
    }

    #[test]
    fn test_splunk_hec_format() {
        let wia_event = serde_json::json!({
            "type": "alert",
            "timestamp": "2024-12-14T10:00:00.000Z",
            "severity": 8.0
        });

        let hec = SplunkHecEvent::from_wia_event(&wia_event, "scanner-001", Some("security".to_string()));
        assert_eq!(hec.sourcetype, "wia:alert");
        assert!(hec.event.get("wia_event").is_some());
    }

    #[test]
    fn test_cef_format() {
        let wia_event = serde_json::json!({
            "id": "event-001",
            "type": "alert",
            "timestamp": "2024-12-14T10:00:00.000Z",
            "severity": 8.0
        });

        let cef = CefEvent::from_wia_event(&wia_event);
        let cef_string = cef.to_cef_string();
        assert!(cef_string.starts_with("CEF:0|WIA|"));
    }
}
