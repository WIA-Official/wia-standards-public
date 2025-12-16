//! Elasticsearch Exporter
//!
//! Exports WIA Security data to Elasticsearch using ECS (Elastic Common Schema).

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use super::{ExportResult, ExportError, ExportStatus};

/// Elasticsearch configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElasticsearchConfig {
    /// Elasticsearch URL(s)
    pub hosts: Vec<String>,
    /// Index prefix
    pub index_prefix: String,
    /// Enable authentication
    pub auth: Option<ElasticsearchAuth>,
    /// Enable SSL
    pub use_ssl: bool,
    /// Verify SSL certificates
    pub verify_certs: bool,
    /// Bulk size
    pub bulk_size: usize,
    /// Refresh policy
    pub refresh: RefreshPolicy,
}

/// Authentication options
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ElasticsearchAuth {
    Basic { username: String, password: String },
    ApiKey { id: String, api_key: String },
    Bearer { token: String },
}

/// Index refresh policy
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum RefreshPolicy {
    #[serde(rename = "true")]
    Immediate,
    #[serde(rename = "false")]
    None,
    #[serde(rename = "wait_for")]
    WaitFor,
}

impl Default for ElasticsearchConfig {
    fn default() -> Self {
        Self {
            hosts: vec!["http://localhost:9200".to_string()],
            index_prefix: "wia-security".to_string(),
            auth: None,
            use_ssl: false,
            verify_certs: true,
            bulk_size: 500,
            refresh: RefreshPolicy::None,
        }
    }
}

/// ECS Base document
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcsDocument {
    /// Timestamp
    #[serde(rename = "@timestamp")]
    pub timestamp: String,
    /// ECS version
    pub ecs: EcsVersion,
    /// Event details
    pub event: EcsEvent,
    /// Source information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<EcsSource>,
    /// Destination information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub destination: Option<EcsDestination>,
    /// User information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub user: Option<EcsUser>,
    /// Host information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub host: Option<EcsHost>,
    /// Vulnerability information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vulnerability: Option<EcsVulnerability>,
    /// Threat information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub threat: Option<EcsThreat>,
    /// Tags
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub tags: Vec<String>,
    /// Labels (custom fields)
    #[serde(skip_serializing_if = "HashMap::is_empty")]
    pub labels: HashMap<String, String>,
    /// Message
    #[serde(skip_serializing_if = "Option::is_none")]
    pub message: Option<String>,
}

/// ECS version
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcsVersion {
    pub version: String,
}

impl Default for EcsVersion {
    fn default() -> Self {
        Self {
            version: "8.11.0".to_string(),
        }
    }
}

/// ECS event field
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcsEvent {
    /// Event ID
    pub id: String,
    /// Event kind
    pub kind: String,
    /// Event category
    pub category: Vec<String>,
    /// Event type
    #[serde(rename = "type")]
    pub event_type: Vec<String>,
    /// Event action
    #[serde(skip_serializing_if = "Option::is_none")]
    pub action: Option<String>,
    /// Event outcome
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outcome: Option<String>,
    /// Severity
    pub severity: u8,
    /// Risk score
    #[serde(skip_serializing_if = "Option::is_none")]
    pub risk_score: Option<f64>,
    /// Dataset
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dataset: Option<String>,
    /// Module
    #[serde(skip_serializing_if = "Option::is_none")]
    pub module: Option<String>,
}

/// ECS source field
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcsSource {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ip: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub port: Option<u16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub domain: Option<String>,
}

/// ECS destination field
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcsDestination {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ip: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub port: Option<u16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub domain: Option<String>,
}

/// ECS user field
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcsUser {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub email: Option<String>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub roles: Vec<String>,
}

/// ECS host field
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcsHost {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hostname: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ip: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub os: Option<EcsOs>,
}

/// ECS OS field
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcsOs {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub family: Option<String>,
}

/// ECS vulnerability field
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcsVulnerability {
    /// Vulnerability ID
    pub id: String,
    /// CVE ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cve: Option<String>,
    /// Severity
    pub severity: String,
    /// CVSS score
    #[serde(skip_serializing_if = "Option::is_none")]
    pub score: Option<EcsCvssScore>,
    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Classification
    #[serde(skip_serializing_if = "Option::is_none")]
    pub classification: Option<String>,
    /// Enumeration (e.g., CVE, CWE)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub enumeration: Option<String>,
    /// Scanner details
    #[serde(skip_serializing_if = "Option::is_none")]
    pub scanner: Option<EcsScanner>,
}

/// ECS CVSS score
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcsCvssScore {
    pub base: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
}

/// ECS scanner info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcsScanner {
    pub vendor: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
}

/// ECS threat field
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcsThreat {
    /// Threat framework
    #[serde(skip_serializing_if = "Option::is_none")]
    pub framework: Option<String>,
    /// Tactics
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub tactic: Vec<EcsTactic>,
    /// Techniques
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub technique: Vec<EcsTechnique>,
    /// Indicator
    #[serde(skip_serializing_if = "Option::is_none")]
    pub indicator: Option<EcsIndicator>,
}

/// ECS tactic
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcsTactic {
    pub id: String,
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reference: Option<String>,
}

/// ECS technique
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcsTechnique {
    pub id: String,
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reference: Option<String>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub subtechnique: Vec<EcsTechnique>,
}

/// ECS threat indicator
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcsIndicator {
    #[serde(rename = "type")]
    pub indicator_type: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidence: Option<String>,
}

/// Elasticsearch bulk action
#[derive(Debug, Clone, Serialize)]
pub struct BulkAction {
    pub index: BulkIndex,
}

/// Bulk index metadata
#[derive(Debug, Clone, Serialize)]
pub struct BulkIndex {
    #[serde(rename = "_index")]
    pub index: String,
    #[serde(rename = "_id", skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
}

/// Elasticsearch Exporter
pub struct ElasticsearchExporter {
    config: ElasticsearchConfig,
}

impl ElasticsearchExporter {
    /// Create new Elasticsearch exporter
    pub fn new(config: ElasticsearchConfig) -> Self {
        Self { config }
    }

    /// Severity string to numeric
    fn severity_to_numeric(severity: &str) -> u8 {
        match severity.to_lowercase().as_str() {
            "critical" => 90,
            "high" => 70,
            "medium" => 50,
            "low" => 30,
            "info" | "informational" => 10,
            _ => 0,
        }
    }

    /// Convert WIA finding to ECS document
    pub fn convert_vulnerability(&self, finding: &super::super::importers::WiaFinding, host: &str) -> EcsDocument {
        let severity_num = Self::severity_to_numeric(&finding.severity);

        EcsDocument {
            timestamp: chrono::Utc::now().to_rfc3339(),
            ecs: EcsVersion::default(),
            event: EcsEvent {
                id: finding.id.clone(),
                kind: "alert".to_string(),
                category: vec!["vulnerability".to_string()],
                event_type: vec!["indicator".to_string()],
                action: None,
                outcome: None,
                severity: severity_num,
                risk_score: finding.cvss_score,
                dataset: Some("wia.vulnerability".to_string()),
                module: Some("wia-security".to_string()),
            },
            source: None,
            destination: finding.port.map(|p| EcsDestination {
                ip: Some(host.to_string()),
                port: Some(p),
                domain: None,
            }),
            user: None,
            host: Some(EcsHost {
                hostname: Some(host.to_string()),
                ip: Some(vec![host.to_string()]),
                os: None,
            }),
            vulnerability: Some(EcsVulnerability {
                id: finding.id.clone(),
                cve: finding.cve.first().cloned(),
                severity: finding.severity.clone(),
                score: finding.cvss_score.map(|s| EcsCvssScore {
                    base: s,
                    version: Some("3.1".to_string()),
                }),
                description: Some(finding.description.clone()),
                classification: finding.cwe.first().cloned(),
                enumeration: Some("CVE".to_string()),
                scanner: Some(EcsScanner {
                    vendor: "WIA Security".to_string(),
                    version: Some("1.0.0".to_string()),
                }),
            }),
            threat: None,
            tags: vec!["vulnerability".to_string(), finding.severity.clone()],
            labels: HashMap::new(),
            message: Some(finding.title.clone()),
        }
    }

    /// Convert security event to ECS document
    pub fn convert_security_event(&self, event: &super::splunk::WiaSecurityEvent) -> EcsDocument {
        let severity_num = Self::severity_to_numeric(&event.severity);

        EcsDocument {
            timestamp: event.timestamp.clone(),
            ecs: EcsVersion::default(),
            event: EcsEvent {
                id: event.event_id.clone(),
                kind: "event".to_string(),
                category: vec!["authentication".to_string()],
                event_type: vec![event.event_type.clone()],
                action: Some(event.action.clone()),
                outcome: Some(event.result.clone()),
                severity: severity_num,
                risk_score: None,
                dataset: Some("wia.security".to_string()),
                module: Some("wia-security".to_string()),
            },
            source: event.source_ip.clone().map(|ip| EcsSource {
                ip: Some(ip),
                port: None,
                domain: None,
            }),
            destination: event.dest_ip.clone().map(|ip| EcsDestination {
                ip: Some(ip),
                port: None,
                domain: None,
            }),
            user: event.user.clone().map(|name| EcsUser {
                id: None,
                name: Some(name),
                email: None,
                roles: vec![],
            }),
            host: None,
            vulnerability: None,
            threat: None,
            tags: event.tags.clone(),
            labels: event.metadata.clone(),
            message: Some(event.description.clone()),
        }
    }

    /// Generate index name with date pattern
    pub fn index_name(&self, suffix: &str) -> String {
        let date = chrono::Utc::now().format("%Y.%m.%d");
        format!("{}-{}-{}", self.config.index_prefix, suffix, date)
    }

    /// Create bulk request body
    pub fn to_bulk_ndjson(&self, documents: &[(String, EcsDocument)]) -> ExportResult<String> {
        let mut lines = Vec::new();

        for (index, doc) in documents {
            let action = BulkAction {
                index: BulkIndex {
                    index: index.clone(),
                    id: Some(doc.event.id.clone()),
                },
            };

            let action_json = serde_json::to_string(&action)
                .map_err(|e| ExportError::SerializationError(e.to_string()))?;
            let doc_json = serde_json::to_string(doc)
                .map_err(|e| ExportError::SerializationError(e.to_string()))?;

            lines.push(action_json);
            lines.push(doc_json);
        }

        Ok(lines.join("\n") + "\n")
    }

    /// Get bulk endpoint URL
    pub fn bulk_endpoint(&self) -> String {
        format!("{}/_bulk", self.config.hosts.first().unwrap_or(&"http://localhost:9200".to_string()))
    }

    /// Create index template for WIA Security
    pub fn index_template() -> serde_json::Value {
        serde_json::json!({
            "index_patterns": ["wia-security-*"],
            "template": {
                "settings": {
                    "number_of_shards": 1,
                    "number_of_replicas": 1,
                    "index.lifecycle.name": "wia-security-policy"
                },
                "mappings": {
                    "properties": {
                        "@timestamp": { "type": "date" },
                        "event.severity": { "type": "integer" },
                        "event.risk_score": { "type": "float" },
                        "vulnerability.score.base": { "type": "float" },
                        "source.ip": { "type": "ip" },
                        "destination.ip": { "type": "ip" },
                        "host.ip": { "type": "ip" }
                    }
                }
            },
            "priority": 100
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::integration::importers::WiaFinding;

    #[test]
    fn test_convert_vulnerability() {
        let config = ElasticsearchConfig::default();
        let exporter = ElasticsearchExporter::new(config);

        let finding = WiaFinding {
            id: "TEST-001".to_string(),
            title: "Test Vulnerability".to_string(),
            description: "A test vulnerability".to_string(),
            severity: "high".to_string(),
            cvss_score: Some(7.5),
            cvss_vector: None,
            port: Some(443),
            protocol: Some("tcp".to_string()),
            service: Some("https".to_string()),
            cve: vec!["CVE-2024-1234".to_string()],
            cwe: vec!["CWE-79".to_string()],
            solution: Some("Update the software".to_string()),
            references: vec![],
            exploit_available: false,
            patch_available: true,
            plugin_output: None,
        };

        let doc = exporter.convert_vulnerability(&finding, "192.168.1.100");

        assert_eq!(doc.event.kind, "alert");
        assert_eq!(doc.event.severity, 70);
        assert!(doc.vulnerability.is_some());
    }

    #[test]
    fn test_index_name() {
        let config = ElasticsearchConfig::default();
        let exporter = ElasticsearchExporter::new(config);

        let name = exporter.index_name("vulnerabilities");
        assert!(name.starts_with("wia-security-vulnerabilities-"));
    }

    #[test]
    fn test_severity_mapping() {
        assert_eq!(ElasticsearchExporter::severity_to_numeric("critical"), 90);
        assert_eq!(ElasticsearchExporter::severity_to_numeric("high"), 70);
        assert_eq!(ElasticsearchExporter::severity_to_numeric("medium"), 50);
        assert_eq!(ElasticsearchExporter::severity_to_numeric("low"), 30);
        assert_eq!(ElasticsearchExporter::severity_to_numeric("info"), 10);
    }
}
