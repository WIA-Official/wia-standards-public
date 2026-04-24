//! Event system for real-time notifications

use crate::error::NanoResult;
use crate::types::NanoSystemType;
use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::{broadcast, RwLock};

/// Event bus for publishing and subscribing to events
pub struct EventBus {
    channels: Arc<RwLock<HashMap<String, broadcast::Sender<NanoEvent>>>>,
    config: EventBusConfig,
}

impl EventBus {
    /// Create a new event bus
    pub fn new(config: EventBusConfig) -> Self {
        Self {
            channels: Arc::new(RwLock::new(HashMap::new())),
            config,
        }
    }

    /// Create with default config
    pub fn default_bus() -> Self {
        Self::new(EventBusConfig::default())
    }

    /// Publish an event
    pub async fn publish(&self, topic: &str, event: NanoEvent) -> NanoResult<usize> {
        let channels = self.channels.read().await;

        if let Some(sender) = channels.get(topic) {
            Ok(sender.send(event).unwrap_or(0))
        } else {
            // Create channel on demand
            drop(channels);
            let mut channels = self.channels.write().await;
            let (tx, _) = broadcast::channel(self.config.channel_capacity);
            let count = tx.send(event).unwrap_or(0);
            channels.insert(topic.to_string(), tx);
            Ok(count)
        }
    }

    /// Subscribe to a topic
    pub async fn subscribe(&self, topic: &str) -> broadcast::Receiver<NanoEvent> {
        let mut channels = self.channels.write().await;

        if let Some(sender) = channels.get(topic) {
            sender.subscribe()
        } else {
            let (tx, rx) = broadcast::channel(self.config.channel_capacity);
            channels.insert(topic.to_string(), tx);
            rx
        }
    }

    /// Publish to multiple topics
    pub async fn publish_multi(&self, topics: &[&str], event: NanoEvent) -> NanoResult<usize> {
        let mut total = 0;
        for topic in topics {
            total += self.publish(topic, event.clone()).await?;
        }
        Ok(total)
    }

    /// Get subscriber count for a topic
    pub async fn subscriber_count(&self, topic: &str) -> usize {
        let channels = self.channels.read().await;
        channels.get(topic).map(|tx| tx.receiver_count()).unwrap_or(0)
    }

    /// List all topics
    pub async fn list_topics(&self) -> Vec<String> {
        self.channels.read().await.keys().cloned().collect()
    }
}

/// Event bus configuration
#[derive(Debug, Clone)]
pub struct EventBusConfig {
    pub channel_capacity: usize,
    pub max_topics: usize,
}

impl Default for EventBusConfig {
    fn default() -> Self {
        Self {
            channel_capacity: 1024,
            max_topics: 1000,
        }
    }
}

/// Nano system event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NanoEvent {
    pub event_id: String,
    pub event_type: EventType,
    pub source_id: String,
    pub source_type: NanoSystemType,
    pub timestamp: u64,
    pub payload: serde_json::Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<String>,
    pub severity: EventSeverity,
}

impl NanoEvent {
    pub fn new(
        event_type: EventType,
        source_id: impl Into<String>,
        source_type: NanoSystemType,
    ) -> Self {
        Self {
            event_id: uuid::Uuid::new_v4().to_string(),
            event_type,
            source_id: source_id.into(),
            source_type,
            timestamp: chrono::Utc::now().timestamp_millis() as u64,
            payload: serde_json::json!({}),
            correlation_id: None,
            severity: EventSeverity::Info,
        }
    }

    pub fn with_payload(mut self, payload: serde_json::Value) -> Self {
        self.payload = payload;
        self
    }

    pub fn with_correlation(mut self, correlation_id: impl Into<String>) -> Self {
        self.correlation_id = Some(correlation_id.into());
        self
    }

    pub fn with_severity(mut self, severity: EventSeverity) -> Self {
        self.severity = severity;
        self
    }

    /// Create a status change event
    pub fn status_change(
        source_id: impl Into<String>,
        source_type: NanoSystemType,
        old_status: &str,
        new_status: &str,
    ) -> Self {
        Self::new(EventType::StatusChange, source_id, source_type).with_payload(serde_json::json!({
            "old_status": old_status,
            "new_status": new_status
        }))
    }

    /// Create a measurement event
    pub fn measurement(
        source_id: impl Into<String>,
        value: f64,
        unit: &str,
    ) -> Self {
        Self::new(EventType::Measurement, source_id, NanoSystemType::Nanosensor)
            .with_payload(serde_json::json!({
                "value": value,
                "unit": unit
            }))
    }

    /// Create an alert event
    pub fn alert(
        source_id: impl Into<String>,
        source_type: NanoSystemType,
        alert_type: &str,
        message: &str,
    ) -> Self {
        Self::new(EventType::Alert, source_id, source_type)
            .with_payload(serde_json::json!({
                "alert_type": alert_type,
                "message": message
            }))
            .with_severity(EventSeverity::Warning)
    }

    /// Create an error event
    pub fn error(
        source_id: impl Into<String>,
        source_type: NanoSystemType,
        error_code: u32,
        message: &str,
    ) -> Self {
        Self::new(EventType::Error, source_id, source_type)
            .with_payload(serde_json::json!({
                "error_code": error_code,
                "message": message
            }))
            .with_severity(EventSeverity::Error)
    }
}

/// Event types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EventType {
    /// System status changed
    StatusChange,
    /// Position updated
    PositionUpdate,
    /// New measurement
    Measurement,
    /// Alert/warning
    Alert,
    /// Error occurred
    Error,
    /// Mission started
    MissionStart,
    /// Mission completed
    MissionComplete,
    /// Mission failed
    MissionFailed,
    /// Assembly started
    AssemblyStart,
    /// Assembly completed
    AssemblyComplete,
    /// Drug released
    DrugRelease,
    /// Device discovered
    DeviceDiscovered,
    /// Device lost
    DeviceLost,
    /// Swarm formed
    SwarmFormed,
    /// Custom event
    Custom,
}

/// Event severity levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EventSeverity {
    Debug,
    Info,
    Warning,
    Error,
    Critical,
}

/// Event handler trait
#[async_trait]
pub trait EventHandler: Send + Sync {
    /// Handle an event
    async fn handle(&self, event: &NanoEvent) -> NanoResult<()>;

    /// Get handler name
    fn name(&self) -> &str;

    /// Get event types this handler handles
    fn handles(&self) -> Vec<EventType> {
        vec![] // Empty = handles all
    }
}

/// Simple logging event handler
pub struct LoggingHandler {
    name: String,
    min_severity: EventSeverity,
}

impl LoggingHandler {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            min_severity: EventSeverity::Info,
        }
    }

    pub fn with_min_severity(mut self, severity: EventSeverity) -> Self {
        self.min_severity = severity;
        self
    }

    fn severity_value(severity: EventSeverity) -> u8 {
        match severity {
            EventSeverity::Debug => 0,
            EventSeverity::Info => 1,
            EventSeverity::Warning => 2,
            EventSeverity::Error => 3,
            EventSeverity::Critical => 4,
        }
    }
}

#[async_trait]
impl EventHandler for LoggingHandler {
    async fn handle(&self, event: &NanoEvent) -> NanoResult<()> {
        if Self::severity_value(event.severity) >= Self::severity_value(self.min_severity) {
            println!(
                "[{}] {:?} from {} ({:?}): {}",
                self.name,
                event.event_type,
                event.source_id,
                event.severity,
                event.payload
            );
        }
        Ok(())
    }

    fn name(&self) -> &str {
        &self.name
    }
}

/// Event filter
#[derive(Debug, Clone, Default)]
pub struct EventFilter {
    pub event_types: Option<Vec<EventType>>,
    pub source_types: Option<Vec<NanoSystemType>>,
    pub source_ids: Option<Vec<String>>,
    pub min_severity: Option<EventSeverity>,
}

impl EventFilter {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_event_types(mut self, types: Vec<EventType>) -> Self {
        self.event_types = Some(types);
        self
    }

    pub fn with_source_types(mut self, types: Vec<NanoSystemType>) -> Self {
        self.source_types = Some(types);
        self
    }

    pub fn matches(&self, event: &NanoEvent) -> bool {
        // Event type filter
        if let Some(ref types) = self.event_types {
            if !types.contains(&event.event_type) {
                return false;
            }
        }

        // Source type filter
        if let Some(ref types) = self.source_types {
            if !types.contains(&event.source_type) {
                return false;
            }
        }

        // Source ID filter
        if let Some(ref ids) = self.source_ids {
            if !ids.contains(&event.source_id) {
                return false;
            }
        }

        // Severity filter
        if let Some(min) = self.min_severity {
            if LoggingHandler::severity_value(event.severity)
                < LoggingHandler::severity_value(min)
            {
                return false;
            }
        }

        true
    }
}
