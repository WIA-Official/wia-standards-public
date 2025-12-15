//! WIA Security Client

use std::collections::HashMap;
use std::sync::{Arc, RwLock};

use crate::builder::*;
use crate::converter::*;
use crate::types::*;
use crate::validator::{validate_event, ValidationResult};

// ============================================================================
// Client Configuration
// ============================================================================

/// Security client configuration
#[derive(Debug, Clone, Default)]
pub struct SecurityClientConfig {
    pub default_source: Option<Source>,
    pub auto_generate_ids: bool,
    pub auto_timestamp: bool,
    pub validate_on_create: bool,
}

impl SecurityClientConfig {
    pub fn new() -> Self {
        Self {
            default_source: None,
            auto_generate_ids: true,
            auto_timestamp: true,
            validate_on_create: true,
        }
    }

    pub fn with_default_source(mut self, source: Source) -> Self {
        self.default_source = Some(source);
        self
    }
}

// ============================================================================
// Event Filter
// ============================================================================

/// Filter for querying events
#[derive(Debug, Clone, Default)]
pub struct EventFilter {
    pub event_type: Option<Vec<EventType>>,
    pub severity_min: Option<f64>,
    pub severity_max: Option<f64>,
    pub start_time: Option<String>,
    pub end_time: Option<String>,
    pub tags: Option<Vec<String>>,
    pub limit: Option<usize>,
    pub offset: Option<usize>,
}

// ============================================================================
// Event Store
// ============================================================================

type EventStore = Arc<RwLock<HashMap<uuid::Uuid, WiaSecurityEvent>>>;

// ============================================================================
// Security Client
// ============================================================================

/// WIA Security Client
pub struct SecurityClient {
    config: SecurityClientConfig,
    store: EventStore,
}

impl SecurityClient {
    /// Create a new client
    pub fn new(config: SecurityClientConfig) -> Self {
        Self {
            config,
            store: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Create an alert builder
    pub fn alert(&self) -> AlertBuilder {
        let mut builder = AlertBuilder::new();
        if let Some(ref source) = self.config.default_source {
            builder = builder.source(source.clone());
        }
        builder
    }

    /// Create a threat intel builder
    pub fn threat_intel(&self) -> ThreatIntelBuilder {
        let mut builder = ThreatIntelBuilder::new();
        if let Some(ref source) = self.config.default_source {
            builder = builder.source(source.clone());
        }
        builder
    }

    /// Create a vulnerability builder
    pub fn vulnerability(&self) -> VulnerabilityBuilder {
        let mut builder = VulnerabilityBuilder::new();
        if let Some(ref source) = self.config.default_source {
            builder = builder.source(source.clone());
        }
        builder
    }

    /// Validate an event
    pub fn validate(&self, event: &WiaSecurityEvent) -> ValidationResult {
        validate_event(event)
    }

    /// Save an event
    pub fn save(&self, event: WiaSecurityEvent) -> Result<WiaSecurityEvent, String> {
        if self.config.validate_on_create {
            let result = self.validate(&event);
            if !result.is_valid() {
                return Err(format!("Validation failed: {:?}", result.errors));
            }
        }

        let mut store = self.store.write().map_err(|e| e.to_string())?;
        store.insert(event.id, event.clone());
        Ok(event)
    }

    /// Get an event by ID
    pub fn get(&self, id: uuid::Uuid) -> Option<WiaSecurityEvent> {
        let store = self.store.read().ok()?;
        store.get(&id).cloned()
    }

    /// Query events
    pub fn query(&self, filter: &EventFilter) -> Vec<WiaSecurityEvent> {
        let store = match self.store.read() {
            Ok(s) => s,
            Err(_) => return Vec::new(),
        };

        let mut results: Vec<_> = store.values().cloned().collect();

        // Filter by type
        if let Some(ref types) = filter.event_type {
            results.retain(|e| types.contains(&e.event_type));
        }

        // Filter by severity
        if let Some(min) = filter.severity_min {
            results.retain(|e| e.severity >= min);
        }
        if let Some(max) = filter.severity_max {
            results.retain(|e| e.severity <= max);
        }

        // Filter by time
        if let Some(ref start) = filter.start_time {
            results.retain(|e| &e.timestamp >= start);
        }
        if let Some(ref end) = filter.end_time {
            results.retain(|e| &e.timestamp <= end);
        }

        // Filter by tags
        if let Some(ref tags) = filter.tags {
            results.retain(|e| {
                if let Some(ref meta) = e.meta {
                    if let Some(ref event_tags) = meta.tags {
                        return tags.iter().any(|t| event_tags.contains(t));
                    }
                }
                false
            });
        }

        // Sort by timestamp descending
        results.sort_by(|a, b| b.timestamp.cmp(&a.timestamp));

        // Apply pagination
        if let Some(offset) = filter.offset {
            results = results.into_iter().skip(offset).collect();
        }
        if let Some(limit) = filter.limit {
            results.truncate(limit);
        }

        results
    }

    /// Delete an event
    pub fn delete(&self, id: uuid::Uuid) -> bool {
        if let Ok(mut store) = self.store.write() {
            return store.remove(&id).is_some();
        }
        false
    }

    /// Get all alerts
    pub fn get_alerts(&self) -> Vec<WiaSecurityEvent> {
        self.query(&EventFilter {
            event_type: Some(vec![EventType::Alert]),
            ..Default::default()
        })
    }

    /// Get high severity events
    pub fn get_high_severity(&self, min_severity: f64) -> Vec<WiaSecurityEvent> {
        self.query(&EventFilter {
            severity_min: Some(min_severity),
            ..Default::default()
        })
    }

    // -------------------------------------------------------------------------
    // Export/Convert
    // -------------------------------------------------------------------------

    /// Export event to STIX 2.1 bundle
    pub fn to_stix(&self, event: &WiaSecurityEvent) -> serde_json::Value {
        to_stix_bundle(event)
    }

    /// Export event to ECS format
    pub fn to_ecs(&self, event: &WiaSecurityEvent) -> serde_json::Value {
        to_ecs_event(event)
    }

    /// Export event to OCSF format
    pub fn to_ocsf(&self, event: &WiaSecurityEvent) -> serde_json::Value {
        to_ocsf_event(event)
    }

    /// Export event to Splunk HEC format
    pub fn to_splunk(&self, event: &WiaSecurityEvent, index: Option<&str>) -> serde_json::Value {
        to_splunk_event(event, index)
    }

    /// Export event to Elasticsearch document
    pub fn to_elastic(&self, event: &WiaSecurityEvent, index_prefix: &str) -> serde_json::Value {
        to_elastic_event(event, index_prefix)
    }

    /// Export events to JSON
    pub fn export_json(&self, events: &[WiaSecurityEvent]) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(events)
    }

    /// Import events from JSON
    pub fn import_json(&self, json: &str) -> Result<Vec<WiaSecurityEvent>, String> {
        let events: Vec<WiaSecurityEvent> =
            serde_json::from_str(json).map_err(|e| e.to_string())?;
        let mut saved = Vec::new();
        for event in events {
            saved.push(self.save(event)?);
        }
        Ok(saved)
    }

    // -------------------------------------------------------------------------
    // Statistics
    // -------------------------------------------------------------------------

    /// Get event statistics
    pub fn get_stats(&self) -> EventStats {
        let events = self.query(&EventFilter::default());

        let mut by_type: HashMap<String, usize> = HashMap::new();
        let mut by_severity = SeverityStats::default();
        let mut total_severity = 0.0;

        for event in &events {
            // Count by type
            *by_type.entry(event.event_type.to_string()).or_insert(0) += 1;

            // Count by severity
            total_severity += event.severity;
            if event.severity <= 2.0 {
                by_severity.info += 1;
            } else if event.severity <= 4.0 {
                by_severity.low += 1;
            } else if event.severity <= 6.0 {
                by_severity.medium += 1;
            } else if event.severity <= 8.0 {
                by_severity.high += 1;
            } else {
                by_severity.critical += 1;
            }
        }

        EventStats {
            total: events.len(),
            by_type,
            by_severity,
            avg_severity: if events.is_empty() {
                0.0
            } else {
                total_severity / events.len() as f64
            },
        }
    }
}

impl Default for SecurityClient {
    fn default() -> Self {
        Self::new(SecurityClientConfig::default())
    }
}

/// Event statistics
#[derive(Debug, Clone)]
pub struct EventStats {
    pub total: usize,
    pub by_type: HashMap<String, usize>,
    pub by_severity: SeverityStats,
    pub avg_severity: f64,
}

/// Severity statistics
#[derive(Debug, Clone, Default)]
pub struct SeverityStats {
    pub info: usize,
    pub low: usize,
    pub medium: usize,
    pub high: usize,
    pub critical: usize,
}

/// Create a new client
pub fn create_client(config: SecurityClientConfig) -> SecurityClient {
    SecurityClient::new(config)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_client_save_and_get() {
        let client = SecurityClient::default();

        let event = client
            .alert()
            .source(Source::new(SourceType::Siem, "Test"))
            .alert_id("ALERT-001")
            .title("Test")
            .category("malware")
            .status("new")
            .priority("high")
            .build()
            .unwrap();

        let id = event.id;
        client.save(event).unwrap();

        let retrieved = client.get(id);
        assert!(retrieved.is_some());
    }
}
