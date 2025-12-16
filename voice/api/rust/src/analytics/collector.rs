//! Analytics data collection

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::{Arc, RwLock};

/// Analytics event type
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AnalyticsEventType {
    TranslationRequest,
    TranslationComplete,
    Error,
    SafetyEvent,
    UserAction,
}

/// Analytics event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalyticsEvent {
    /// Event ID
    pub event_id: String,

    /// Event type
    pub event_type: AnalyticsEventType,

    /// Timestamp
    pub timestamp: DateTime<Utc>,

    /// Service name
    pub service: String,

    /// Service version
    pub version: String,

    /// Anonymized session hash
    pub session_hash: Option<String>,

    /// Event metrics
    pub metrics: HashMap<String, MetricValue>,
}

/// Metric value
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum MetricValue {
    String(String),
    Int(i64),
    Float(f64),
    Bool(bool),
}

impl From<String> for MetricValue {
    fn from(s: String) -> Self {
        MetricValue::String(s)
    }
}

impl From<&str> for MetricValue {
    fn from(s: &str) -> Self {
        MetricValue::String(s.to_string())
    }
}

impl From<i64> for MetricValue {
    fn from(i: i64) -> Self {
        MetricValue::Int(i)
    }
}

impl From<f64> for MetricValue {
    fn from(f: f64) -> Self {
        MetricValue::Float(f)
    }
}

impl From<bool> for MetricValue {
    fn from(b: bool) -> Self {
        MetricValue::Bool(b)
    }
}

/// Analytics collector configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CollectorConfig {
    /// Enable analytics collection
    pub enabled: bool,

    /// Buffer size before flush
    pub buffer_size: usize,

    /// Flush interval in seconds
    pub flush_interval_seconds: u64,

    /// Anonymize user data
    pub anonymize: bool,

    /// Sample rate (0.0 - 1.0)
    pub sample_rate: f64,
}

impl Default for CollectorConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            buffer_size: 1000,
            flush_interval_seconds: 60,
            anonymize: true,
            sample_rate: 1.0,
        }
    }
}

/// Analytics collector
pub struct AnalyticsCollector {
    config: CollectorConfig,
    service: String,
    version: String,
    buffer: Arc<RwLock<Vec<AnalyticsEvent>>>,
    sinks: Vec<Box<dyn AnalyticsSink>>,
}

impl AnalyticsCollector {
    /// Create a new analytics collector
    pub fn new(service: String, version: String, config: CollectorConfig) -> Self {
        Self {
            config,
            service,
            version,
            buffer: Arc::new(RwLock::new(Vec::new())),
            sinks: Vec::new(),
        }
    }

    /// Add a sink
    pub fn add_sink(&mut self, sink: Box<dyn AnalyticsSink>) {
        self.sinks.push(sink);
    }

    /// Record an event
    pub fn record(&self, event_type: AnalyticsEventType, metrics: HashMap<String, MetricValue>) {
        if !self.config.enabled {
            return;
        }

        // Sample
        if self.config.sample_rate < 1.0 {
            let sample: f64 = rand::random();
            if sample > self.config.sample_rate {
                return;
            }
        }

        let event = AnalyticsEvent {
            event_id: uuid::Uuid::new_v4().to_string(),
            event_type,
            timestamp: Utc::now(),
            service: self.service.clone(),
            version: self.version.clone(),
            session_hash: None,
            metrics,
        };

        let mut buffer = self.buffer.write().unwrap();
        buffer.push(event);

        // Check if we need to flush
        if buffer.len() >= self.config.buffer_size {
            let events = std::mem::take(&mut *buffer);
            drop(buffer);
            self.flush_events(events);
        }
    }

    /// Record a translation event
    pub fn record_translation(
        &self,
        source_lang: &str,
        target_lang: &str,
        status: &str,
        duration_ms: i64,
        quality_score: Option<f64>,
    ) {
        let mut metrics = HashMap::new();
        metrics.insert("source_lang".to_string(), source_lang.into());
        metrics.insert("target_lang".to_string(), target_lang.into());
        metrics.insert("status".to_string(), status.into());
        metrics.insert("duration_ms".to_string(), duration_ms.into());

        if let Some(score) = quality_score {
            metrics.insert("quality_score".to_string(), score.into());
        }

        self.record(AnalyticsEventType::TranslationComplete, metrics);
    }

    /// Record an error event
    pub fn record_error(&self, error_type: &str, error_code: &str, endpoint: &str) {
        let mut metrics = HashMap::new();
        metrics.insert("error_type".to_string(), error_type.into());
        metrics.insert("error_code".to_string(), error_code.into());
        metrics.insert("endpoint".to_string(), endpoint.into());

        self.record(AnalyticsEventType::Error, metrics);
    }

    /// Record a safety event
    pub fn record_safety_event(&self, category: &str, action: &str) {
        let mut metrics = HashMap::new();
        metrics.insert("category".to_string(), category.into());
        metrics.insert("action".to_string(), action.into());

        self.record(AnalyticsEventType::SafetyEvent, metrics);
    }

    /// Flush buffer
    pub fn flush(&self) {
        let mut buffer = self.buffer.write().unwrap();
        if buffer.is_empty() {
            return;
        }

        let events = std::mem::take(&mut *buffer);
        drop(buffer);
        self.flush_events(events);
    }

    fn flush_events(&self, events: Vec<AnalyticsEvent>) {
        for sink in &self.sinks {
            if let Err(e) = sink.write(&events) {
                log::error!("Failed to write to analytics sink: {}", e);
            }
        }
    }

    /// Get buffer size
    pub fn buffer_size(&self) -> usize {
        self.buffer.read().unwrap().len()
    }
}

/// Random number generator helper
mod rand {
    pub fn random<T: Default>() -> T {
        // Simplified - in production use proper RNG
        T::default()
    }
}

/// Analytics sink trait
pub trait AnalyticsSink: Send + Sync {
    /// Sink name
    fn name(&self) -> &str;

    /// Write events to sink
    fn write(&self, events: &[AnalyticsEvent]) -> Result<(), SinkError>;
}

/// Sink error
#[derive(Debug, thiserror::Error)]
pub enum SinkError {
    #[error("Write failed: {0}")]
    WriteFailed(String),

    #[error("Connection error: {0}")]
    ConnectionError(String),

    #[error("Serialization error: {0}")]
    SerializationError(String),
}

/// Console sink (for development)
pub struct ConsoleSink;

impl AnalyticsSink for ConsoleSink {
    fn name(&self) -> &str {
        "console"
    }

    fn write(&self, events: &[AnalyticsEvent]) -> Result<(), SinkError> {
        for event in events {
            println!(
                "[Analytics] {} {:?}: {:?}",
                event.timestamp, event.event_type, event.metrics
            );
        }
        Ok(())
    }
}

/// File sink
pub struct FileSink {
    path: String,
}

impl FileSink {
    pub fn new(path: String) -> Self {
        Self { path }
    }
}

impl AnalyticsSink for FileSink {
    fn name(&self) -> &str {
        "file"
    }

    fn write(&self, events: &[AnalyticsEvent]) -> Result<(), SinkError> {
        use std::fs::OpenOptions;
        use std::io::Write;

        let mut file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&self.path)
            .map_err(|e| SinkError::WriteFailed(e.to_string()))?;

        for event in events {
            let json = serde_json::to_string(event)
                .map_err(|e| SinkError::SerializationError(e.to_string()))?;
            writeln!(file, "{}", json).map_err(|e| SinkError::WriteFailed(e.to_string()))?;
        }

        Ok(())
    }
}

/// Kafka sink (stub - implement with actual Kafka client)
pub struct KafkaSink {
    topic: String,
    brokers: Vec<String>,
}

impl KafkaSink {
    pub fn new(topic: String, brokers: Vec<String>) -> Self {
        Self { topic, brokers }
    }
}

impl AnalyticsSink for KafkaSink {
    fn name(&self) -> &str {
        "kafka"
    }

    fn write(&self, events: &[AnalyticsEvent]) -> Result<(), SinkError> {
        // In production, use actual Kafka producer
        log::debug!(
            "Would write {} events to Kafka topic {}",
            events.len(),
            self.topic
        );
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_analytics_collector() {
        let config = CollectorConfig::default();
        let collector = AnalyticsCollector::new(
            "voice-sign-api".to_string(),
            "1.0.0".to_string(),
            config,
        );

        collector.record_translation("en", "ASL", "success", 250, Some(0.95));
        assert_eq!(collector.buffer_size(), 1);
    }

    #[test]
    fn test_console_sink() {
        let sink = ConsoleSink;
        let events = vec![AnalyticsEvent {
            event_id: "test-001".to_string(),
            event_type: AnalyticsEventType::TranslationComplete,
            timestamp: Utc::now(),
            service: "test".to_string(),
            version: "1.0.0".to_string(),
            session_hash: None,
            metrics: HashMap::new(),
        }];

        assert!(sink.write(&events).is_ok());
    }
}
