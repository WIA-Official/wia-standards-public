//! Azure Defender (Microsoft Defender for Cloud) Integration
//!
//! Send security findings to Microsoft Defender for Cloud.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use super::{CloudResult, CloudError};

/// Azure Defender configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AzureDefenderConfig {
    /// Azure Subscription ID
    pub subscription_id: String,
    /// Resource Group
    pub resource_group: String,
    /// Workspace ID (Log Analytics)
    pub workspace_id: String,
    /// Tenant ID
    pub tenant_id: String,
    /// Client ID (App Registration)
    pub client_id: String,
    /// Client Secret
    pub client_secret: Option<String>,
    /// Use Managed Identity
    pub use_managed_identity: bool,
    /// Azure environment
    pub environment: AzureEnvironment,
}

/// Azure environment
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AzureEnvironment {
    Public,
    Government,
    China,
}

impl Default for AzureDefenderConfig {
    fn default() -> Self {
        Self {
            subscription_id: String::new(),
            resource_group: String::new(),
            workspace_id: String::new(),
            tenant_id: String::new(),
            client_id: String::new(),
            client_secret: None,
            use_managed_identity: false,
            environment: AzureEnvironment::Public,
        }
    }
}

/// Azure Security Alert
/// Based on Microsoft Defender for Cloud alert schema
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AzureSecurityAlert {
    /// Alert ID
    pub id: String,
    /// Alert name
    pub name: String,
    /// Alert type
    #[serde(rename = "type")]
    pub alert_type: String,
    /// Location
    pub location: String,
    /// Properties
    pub properties: AzureAlertProperties,
}

/// Azure alert properties
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AzureAlertProperties {
    /// Alert display name
    pub alert_display_name: String,
    /// Description
    pub description: String,
    /// Severity
    pub severity: String,
    /// Status
    pub status: String,
    /// Intent
    pub intent: Option<String>,
    /// Start time (UTC)
    pub start_time_utc: String,
    /// End time (UTC)
    pub end_time_utc: Option<String>,
    /// Resource identifiers
    pub resource_identifiers: Vec<AzureResourceIdentifier>,
    /// Compromised entity
    pub compromised_entity: Option<String>,
    /// Remediation steps
    pub remediation_steps: Vec<String>,
    /// Vendor name
    pub vendor_name: String,
    /// Product name
    pub product_name: String,
    /// Product component name
    pub product_component_name: Option<String>,
    /// Alert URI
    pub alert_uri: Option<String>,
    /// Confidence level
    pub confidence_level: Option<String>,
    /// Confidence score
    pub confidence_score: Option<f64>,
    /// Extended properties
    pub extended_properties: HashMap<String, String>,
    /// Entities
    pub entities: Vec<AzureAlertEntity>,
    /// Is incident
    pub is_incident: bool,
    /// Correlation key
    pub correlation_key: Option<String>,
    /// Extended links
    pub extended_links: Vec<AzureExtendedLink>,
    /// Techniques
    pub techniques: Vec<String>,
    /// Sub-techniques
    pub sub_techniques: Vec<String>,
}

/// Azure resource identifier
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AzureResourceIdentifier {
    /// Identifier type
    #[serde(rename = "type")]
    pub id_type: String,
    /// Azure resource ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub azure_resource_id: Option<String>,
    /// Workspace ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub workspace_id: Option<String>,
    /// Agent ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub agent_id: Option<String>,
}

/// Azure alert entity
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AzureAlertEntity {
    /// Entity type
    #[serde(rename = "type")]
    pub entity_type: String,
    /// Entity properties
    #[serde(flatten)]
    pub properties: HashMap<String, serde_json::Value>,
}

/// Azure extended link
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AzureExtendedLink {
    /// Link category
    pub category: String,
    /// Link label
    pub label: String,
    /// Link href
    pub href: String,
    /// Link type
    #[serde(rename = "type")]
    pub link_type: String,
}

/// Log Analytics custom log entry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LogAnalyticsEntry {
    /// Timestamp
    #[serde(rename = "TimeGenerated")]
    pub time_generated: String,
    /// Record type
    #[serde(rename = "Type")]
    pub record_type: String,
    /// Finding ID
    #[serde(rename = "FindingId")]
    pub finding_id: String,
    /// Title
    #[serde(rename = "Title")]
    pub title: String,
    /// Description
    #[serde(rename = "Description")]
    pub description: String,
    /// Severity
    #[serde(rename = "Severity")]
    pub severity: String,
    /// Severity number
    #[serde(rename = "SeverityNumber")]
    pub severity_number: u8,
    /// Host
    #[serde(rename = "Host")]
    pub host: String,
    /// Port
    #[serde(rename = "Port")]
    pub port: Option<u16>,
    /// Protocol
    #[serde(rename = "Protocol")]
    pub protocol: Option<String>,
    /// CVE
    #[serde(rename = "CVE")]
    pub cve: String,
    /// CVSS Score
    #[serde(rename = "CVSSScore")]
    pub cvss_score: Option<f64>,
    /// Solution
    #[serde(rename = "Solution")]
    pub solution: Option<String>,
    /// Scanner
    #[serde(rename = "Scanner")]
    pub scanner: String,
    /// Exploit Available
    #[serde(rename = "ExploitAvailable")]
    pub exploit_available: bool,
}

/// Azure Defender/Sentinel Exporter
pub struct AzureDefenderExporter {
    config: AzureDefenderConfig,
}

impl AzureDefenderExporter {
    /// Create new exporter
    pub fn new(config: AzureDefenderConfig) -> Self {
        Self { config }
    }

    /// Severity to Azure severity level
    fn to_azure_severity(severity: &str) -> String {
        match severity.to_lowercase().as_str() {
            "critical" => "High".to_string(),
            "high" => "High".to_string(),
            "medium" => "Medium".to_string(),
            "low" => "Low".to_string(),
            _ => "Informational".to_string(),
        }
    }

    /// Severity to numeric value
    fn severity_to_number(severity: &str) -> u8 {
        match severity.to_lowercase().as_str() {
            "critical" => 4,
            "high" => 3,
            "medium" => 2,
            "low" => 1,
            _ => 0,
        }
    }

    /// Convert WIA finding to Azure Security Alert
    pub fn convert_to_alert(&self, finding: &super::super::importers::WiaFinding, host: &str) -> AzureSecurityAlert {
        let now = chrono::Utc::now().to_rfc3339();
        let alert_id = format!(
            "/subscriptions/{}/resourceGroups/{}/providers/Microsoft.Security/alerts/{}",
            self.config.subscription_id,
            self.config.resource_group,
            finding.id
        );

        let mut extended_properties = HashMap::new();
        if let Some(score) = finding.cvss_score {
            extended_properties.insert("cvssScore".to_string(), score.to_string());
        }
        if let Some(ref vector) = finding.cvss_vector {
            extended_properties.insert("cvssVector".to_string(), vector.clone());
        }
        if !finding.cve.is_empty() {
            extended_properties.insert("cve".to_string(), finding.cve.join(", "));
        }
        extended_properties.insert("exploitAvailable".to_string(), finding.exploit_available.to_string());

        let remediation_steps = finding.solution.clone()
            .map(|s| vec![s])
            .unwrap_or_default();

        let mut entities = Vec::new();

        // Add host entity
        let mut host_props = HashMap::new();
        host_props.insert("hostName".to_string(), serde_json::Value::String(host.to_string()));
        if let Some(port) = finding.port {
            host_props.insert("port".to_string(), serde_json::Value::Number(serde_json::Number::from(port)));
        }
        entities.push(AzureAlertEntity {
            entity_type: "Host".to_string(),
            properties: host_props,
        });

        let extended_links = finding.references.iter().map(|url| AzureExtendedLink {
            category: "Reference".to_string(),
            label: "More Information".to_string(),
            href: url.clone(),
            link_type: "WebLink".to_string(),
        }).collect();

        AzureSecurityAlert {
            id: alert_id.clone(),
            name: finding.id.clone(),
            alert_type: "Microsoft.Security/alerts".to_string(),
            location: "global".to_string(),
            properties: AzureAlertProperties {
                alert_display_name: finding.title.clone(),
                description: finding.description.clone(),
                severity: Self::to_azure_severity(&finding.severity),
                status: "Active".to_string(),
                intent: Some("Exploitation".to_string()),
                start_time_utc: now.clone(),
                end_time_utc: None,
                resource_identifiers: vec![
                    AzureResourceIdentifier {
                        id_type: "AzureResource".to_string(),
                        azure_resource_id: Some(format!(
                            "/subscriptions/{}/resourceGroups/{}",
                            self.config.subscription_id,
                            self.config.resource_group
                        )),
                        workspace_id: Some(self.config.workspace_id.clone()),
                        agent_id: None,
                    },
                ],
                compromised_entity: Some(host.to_string()),
                remediation_steps,
                vendor_name: "WIA Security".to_string(),
                product_name: "WIA Vulnerability Scanner".to_string(),
                product_component_name: Some("Integration".to_string()),
                alert_uri: Some(alert_id),
                confidence_level: Some("High".to_string()),
                confidence_score: Some(0.8),
                extended_properties,
                entities,
                is_incident: false,
                correlation_key: Some(format!("wia-{}", finding.id)),
                extended_links,
                techniques: vec![],
                sub_techniques: vec![],
            },
        }
    }

    /// Convert WIA finding to Log Analytics entry
    pub fn convert_to_log_entry(&self, finding: &super::super::importers::WiaFinding, host: &str) -> LogAnalyticsEntry {
        LogAnalyticsEntry {
            time_generated: chrono::Utc::now().to_rfc3339(),
            record_type: "WIA_SecurityFinding".to_string(),
            finding_id: finding.id.clone(),
            title: finding.title.clone(),
            description: finding.description.clone(),
            severity: finding.severity.clone(),
            severity_number: Self::severity_to_number(&finding.severity),
            host: host.to_string(),
            port: finding.port,
            protocol: finding.protocol.clone(),
            cve: finding.cve.join(", "),
            cvss_score: finding.cvss_score,
            solution: finding.solution.clone(),
            scanner: "WIA Security".to_string(),
            exploit_available: finding.exploit_available,
        }
    }

    /// Convert scan results to Log Analytics entries
    pub fn convert_scan_to_logs(&self, scan: &super::super::importers::WiaScanResult) -> Vec<LogAnalyticsEntry> {
        let mut entries = Vec::new();
        for target in &scan.targets {
            for finding in &target.findings {
                entries.push(self.convert_to_log_entry(finding, &target.host));
            }
        }
        entries
    }

    /// Get Log Analytics Data Collector API endpoint
    pub fn log_analytics_endpoint(&self) -> String {
        match self.config.environment {
            AzureEnvironment::Public => format!(
                "https://{}.ods.opinsights.azure.com/api/logs?api-version=2016-04-01",
                self.config.workspace_id
            ),
            AzureEnvironment::Government => format!(
                "https://{}.ods.opinsights.azure.us/api/logs?api-version=2016-04-01",
                self.config.workspace_id
            ),
            AzureEnvironment::China => format!(
                "https://{}.ods.opinsights.azure.cn/api/logs?api-version=2016-04-01",
                self.config.workspace_id
            ),
        }
    }

    /// Get Security Center alerts API endpoint
    pub fn alerts_endpoint(&self) -> String {
        format!(
            "https://management.azure.com/subscriptions/{}/providers/Microsoft.Security/alerts?api-version=2022-01-01",
            self.config.subscription_id
        )
    }

    /// Serialize entries for Log Analytics
    pub fn to_log_analytics_json(&self, entries: &[LogAnalyticsEntry]) -> CloudResult<String> {
        serde_json::to_string(entries)
            .map_err(|e| CloudError::SerializationError(e.to_string()))
    }

    /// Generate authorization header for Log Analytics
    /// Note: In production, use the Azure SDK or implement HMAC-SHA256 signing
    pub fn log_analytics_headers(&self, content_length: usize, log_type: &str) -> HashMap<String, String> {
        let now = chrono::Utc::now().format("%a, %d %b %Y %H:%M:%S GMT").to_string();

        let mut headers = HashMap::new();
        headers.insert("Content-Type".to_string(), "application/json".to_string());
        headers.insert("Log-Type".to_string(), log_type.to_string());
        headers.insert("x-ms-date".to_string(), now);
        headers.insert("Content-Length".to_string(), content_length.to_string());
        // Authorization header would be computed with HMAC-SHA256
        // headers.insert("Authorization".to_string(), signature);
        headers
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_severity_mapping() {
        assert_eq!(AzureDefenderExporter::to_azure_severity("critical"), "High");
        assert_eq!(AzureDefenderExporter::to_azure_severity("high"), "High");
        assert_eq!(AzureDefenderExporter::to_azure_severity("medium"), "Medium");
        assert_eq!(AzureDefenderExporter::to_azure_severity("low"), "Low");
        assert_eq!(AzureDefenderExporter::to_azure_severity("info"), "Informational");
    }

    #[test]
    fn test_convert_to_log_entry() {
        let config = AzureDefenderConfig {
            subscription_id: "test-sub".to_string(),
            workspace_id: "test-workspace".to_string(),
            ..Default::default()
        };
        let exporter = AzureDefenderExporter::new(config);

        let finding = super::super::super::importers::WiaFinding {
            id: "TEST-001".to_string(),
            title: "Test Vulnerability".to_string(),
            description: "Test description".to_string(),
            severity: "high".to_string(),
            cvss_score: Some(7.5),
            cvss_vector: None,
            port: Some(443),
            protocol: Some("tcp".to_string()),
            service: None,
            cve: vec!["CVE-2024-1234".to_string()],
            cwe: vec![],
            solution: Some("Fix it".to_string()),
            references: vec![],
            exploit_available: false,
            patch_available: true,
            plugin_output: None,
        };

        let entry = exporter.convert_to_log_entry(&finding, "192.168.1.100");

        assert_eq!(entry.finding_id, "TEST-001");
        assert_eq!(entry.severity, "high");
        assert_eq!(entry.severity_number, 3);
    }

    #[test]
    fn test_endpoints() {
        let config = AzureDefenderConfig {
            subscription_id: "test-sub".to_string(),
            workspace_id: "test-workspace".to_string(),
            environment: AzureEnvironment::Public,
            ..Default::default()
        };
        let exporter = AzureDefenderExporter::new(config);

        assert!(exporter.log_analytics_endpoint().contains("opinsights.azure.com"));
        assert!(exporter.alerts_endpoint().contains("management.azure.com"));
    }
}
