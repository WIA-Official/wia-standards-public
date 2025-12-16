//! Splunk HEC (HTTP Event Collector) Exporter
//!
//! Exports WIA Security data to Splunk via HEC API.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use super::{ExportResult, ExportError, ExportStatus};

/// Splunk HEC configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SplunkConfig {
    /// HEC endpoint URL (e.g., https://splunk.example.com:8088)
    pub endpoint: String,
    /// HEC token
    pub token: String,
    /// Default index
    pub index: Option<String>,
    /// Default source
    pub source: Option<String>,
    /// Default sourcetype
    pub sourcetype: Option<String>,
    /// Enable SSL verification
    pub verify_ssl: bool,
    /// Batch size for bulk exports
    pub batch_size: usize,
    /// Connection timeout in seconds
    pub timeout_secs: u64,
}

impl Default for SplunkConfig {
    fn default() -> Self {
        Self {
            endpoint: "https://localhost:8088".to_string(),
            token: String::new(),
            index: Some("wia_security".to_string()),
            source: Some("wia-security-api".to_string()),
            sourcetype: Some("wia:security:event".to_string()),
            verify_ssl: true,
            batch_size: 100,
            timeout_secs: 30,
        }
    }
}

/// Splunk HEC event format
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SplunkEvent {
    /// Event timestamp (epoch seconds)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub time: Option<f64>,
    /// Target index
    #[serde(skip_serializing_if = "Option::is_none")]
    pub index: Option<String>,
    /// Event source
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<String>,
    /// Event sourcetype
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sourcetype: Option<String>,
    /// Host that generated the event
    #[serde(skip_serializing_if = "Option::is_none")]
    pub host: Option<String>,
    /// Event data
    pub event: SplunkEventData,
    /// Custom fields
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fields: Option<HashMap<String, String>>,
}

/// Splunk event data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SplunkEventData {
    /// Event category
    pub category: String,
    /// Event type
    pub event_type: String,
    /// Severity level
    pub severity: String,
    /// Event message
    pub message: String,
    /// Additional details
    #[serde(flatten)]
    pub details: HashMap<String, serde_json::Value>,
}

/// WIA Security event for Splunk export
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaSecurityEvent {
    pub event_id: String,
    pub timestamp: String,
    pub event_type: String,
    pub severity: String,
    pub source_ip: Option<String>,
    pub dest_ip: Option<String>,
    pub user: Option<String>,
    pub action: String,
    pub result: String,
    pub description: String,
    pub tags: Vec<String>,
    pub metadata: HashMap<String, String>,
}

/// Splunk HEC Exporter
pub struct SplunkExporter {
    config: SplunkConfig,
}

impl SplunkExporter {
    /// Create new Splunk exporter
    pub fn new(config: SplunkConfig) -> Self {
        Self { config }
    }

    /// Convert WIA security event to Splunk format
    pub fn convert_event(&self, event: &WiaSecurityEvent) -> SplunkEvent {
        let mut details = HashMap::new();

        if let Some(ref src) = event.source_ip {
            details.insert("src_ip".to_string(), serde_json::Value::String(src.clone()));
        }
        if let Some(ref dst) = event.dest_ip {
            details.insert("dest_ip".to_string(), serde_json::Value::String(dst.clone()));
        }
        if let Some(ref user) = event.user {
            details.insert("user".to_string(), serde_json::Value::String(user.clone()));
        }
        details.insert("action".to_string(), serde_json::Value::String(event.action.clone()));
        details.insert("result".to_string(), serde_json::Value::String(event.result.clone()));
        details.insert("tags".to_string(), serde_json::Value::Array(
            event.tags.iter().map(|t| serde_json::Value::String(t.clone())).collect()
        ));

        // Parse timestamp to epoch
        let time = chrono::DateTime::parse_from_rfc3339(&event.timestamp)
            .ok()
            .map(|dt| dt.timestamp() as f64);

        let mut fields = HashMap::new();
        for (k, v) in &event.metadata {
            fields.insert(k.clone(), v.clone());
        }

        SplunkEvent {
            time,
            index: self.config.index.clone(),
            source: self.config.source.clone(),
            sourcetype: self.config.sourcetype.clone(),
            host: event.source_ip.clone(),
            event: SplunkEventData {
                category: "security".to_string(),
                event_type: event.event_type.clone(),
                severity: event.severity.clone(),
                message: event.description.clone(),
                details,
            },
            fields: Some(fields),
        }
    }

    /// Convert vulnerability finding to Splunk format
    pub fn convert_vulnerability(&self, vuln: &super::super::importers::WiaFinding, host: &str) -> SplunkEvent {
        let mut details = HashMap::new();

        details.insert("vuln_id".to_string(), serde_json::Value::String(vuln.id.clone()));
        details.insert("title".to_string(), serde_json::Value::String(vuln.title.clone()));

        if let Some(score) = vuln.cvss_score {
            details.insert("cvss_score".to_string(), serde_json::Value::Number(
                serde_json::Number::from_f64(score).unwrap_or(serde_json::Number::from(0))
            ));
        }
        if let Some(ref port) = vuln.port {
            details.insert("port".to_string(), serde_json::Value::Number(serde_json::Number::from(*port)));
        }
        if !vuln.cve.is_empty() {
            details.insert("cve".to_string(), serde_json::Value::Array(
                vuln.cve.iter().map(|c| serde_json::Value::String(c.clone())).collect()
            ));
        }
        details.insert("exploit_available".to_string(), serde_json::Value::Bool(vuln.exploit_available));

        SplunkEvent {
            time: Some(chrono::Utc::now().timestamp() as f64),
            index: self.config.index.clone(),
            source: self.config.source.clone(),
            sourcetype: Some("wia:security:vulnerability".to_string()),
            host: Some(host.to_string()),
            event: SplunkEventData {
                category: "vulnerability".to_string(),
                event_type: "finding".to_string(),
                severity: vuln.severity.clone(),
                message: vuln.description.clone(),
                details,
            },
            fields: None,
        }
    }

    /// Serialize events to NDJSON for HEC raw endpoint
    pub fn to_ndjson(&self, events: &[SplunkEvent]) -> ExportResult<String> {
        let mut lines = Vec::new();
        for event in events {
            let json = serde_json::to_string(event)
                .map_err(|e| ExportError::SerializationError(e.to_string()))?;
            lines.push(json);
        }
        Ok(lines.join("\n"))
    }

    /// Build HEC request headers
    pub fn build_headers(&self) -> HashMap<String, String> {
        let mut headers = HashMap::new();
        headers.insert("Authorization".to_string(), format!("Splunk {}", self.config.token));
        headers.insert("Content-Type".to_string(), "application/json".to_string());
        headers
    }

    /// Get HEC event endpoint URL
    pub fn event_endpoint(&self) -> String {
        format!("{}/services/collector/event", self.config.endpoint)
    }

    /// Get HEC raw endpoint URL (for batch)
    pub fn raw_endpoint(&self) -> String {
        format!("{}/services/collector/raw", self.config.endpoint)
    }

    /// Create sample events for testing
    pub fn sample_events() -> Vec<WiaSecurityEvent> {
        vec![
            WiaSecurityEvent {
                event_id: "evt-001".to_string(),
                timestamp: chrono::Utc::now().to_rfc3339(),
                event_type: "authentication".to_string(),
                severity: "medium".to_string(),
                source_ip: Some("192.168.1.100".to_string()),
                dest_ip: Some("10.0.0.1".to_string()),
                user: Some("admin".to_string()),
                action: "login".to_string(),
                result: "failure".to_string(),
                description: "Failed login attempt from suspicious IP".to_string(),
                tags: vec!["brute-force".to_string(), "suspicious".to_string()],
                metadata: HashMap::new(),
            },
            WiaSecurityEvent {
                event_id: "evt-002".to_string(),
                timestamp: chrono::Utc::now().to_rfc3339(),
                event_type: "access_control".to_string(),
                severity: "high".to_string(),
                source_ip: Some("192.168.1.50".to_string()),
                dest_ip: Some("10.0.0.5".to_string()),
                user: Some("guest".to_string()),
                action: "access".to_string(),
                result: "denied".to_string(),
                description: "Unauthorized access attempt to sensitive resource".to_string(),
                tags: vec!["unauthorized".to_string(), "policy-violation".to_string()],
                metadata: HashMap::new(),
            },
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_convert_event() {
        let config = SplunkConfig::default();
        let exporter = SplunkExporter::new(config);

        let events = SplunkExporter::sample_events();
        let splunk_event = exporter.convert_event(&events[0]);

        assert_eq!(splunk_event.event.category, "security");
        assert_eq!(splunk_event.event.severity, "medium");
    }

    #[test]
    fn test_to_ndjson() {
        let config = SplunkConfig::default();
        let exporter = SplunkExporter::new(config);

        let events = SplunkExporter::sample_events();
        let splunk_events: Vec<_> = events.iter().map(|e| exporter.convert_event(e)).collect();

        let ndjson = exporter.to_ndjson(&splunk_events).unwrap();
        assert!(ndjson.contains('\n'));
        assert!(ndjson.contains("security"));
    }

    #[test]
    fn test_build_headers() {
        let mut config = SplunkConfig::default();
        config.token = "test-token".to_string();
        let exporter = SplunkExporter::new(config);

        let headers = exporter.build_headers();
        assert_eq!(headers.get("Authorization"), Some(&"Splunk test-token".to_string()));
    }
}
