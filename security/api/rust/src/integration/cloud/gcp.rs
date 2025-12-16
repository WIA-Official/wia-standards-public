//! GCP Security Command Center Integration
//!
//! Send security findings to Google Cloud Security Command Center.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use super::{CloudResult, CloudError};

/// GCP Security Command Center configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GcpSccConfig {
    /// GCP Project ID
    pub project_id: String,
    /// Organization ID
    pub organization_id: String,
    /// Source ID (registered source in SCC)
    pub source_id: Option<String>,
    /// Service account key path
    pub credentials_path: Option<String>,
    /// Use Application Default Credentials
    pub use_adc: bool,
}

impl Default for GcpSccConfig {
    fn default() -> Self {
        Self {
            project_id: String::new(),
            organization_id: String::new(),
            source_id: None,
            credentials_path: None,
            use_adc: true,
        }
    }
}

/// GCP SCC Finding
/// Based on Security Command Center Finding schema
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SccFinding {
    /// Finding name (resource name)
    pub name: String,
    /// Parent (source name)
    pub parent: String,
    /// Resource name
    pub resource_name: String,
    /// State
    pub state: String,
    /// Category
    pub category: String,
    /// External URI
    #[serde(skip_serializing_if = "Option::is_none")]
    pub external_uri: Option<String>,
    /// Source properties
    pub source_properties: HashMap<String, serde_json::Value>,
    /// Security marks
    #[serde(skip_serializing_if = "Option::is_none")]
    pub security_marks: Option<SccSecurityMarks>,
    /// Event time
    pub event_time: String,
    /// Create time
    #[serde(skip_serializing_if = "Option::is_none")]
    pub create_time: Option<String>,
    /// Severity
    pub severity: String,
    /// Canonical name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub canonical_name: Option<String>,
    /// Mute
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mute: Option<String>,
    /// Finding class
    pub finding_class: String,
    /// Indicator
    #[serde(skip_serializing_if = "Option::is_none")]
    pub indicator: Option<SccIndicator>,
    /// Vulnerability
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vulnerability: Option<SccVulnerability>,
    /// Compliance
    #[serde(skip_serializing_if = "Option::is_none")]
    pub compliance: Option<SccCompliance>,
    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Exfiltration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exfiltration: Option<SccExfiltration>,
    /// Connections
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub connections: Vec<SccConnection>,
    /// Next steps
    #[serde(skip_serializing_if = "Option::is_none")]
    pub next_steps: Option<String>,
}

/// Security marks
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SccSecurityMarks {
    /// Name
    pub name: String,
    /// Marks
    pub marks: HashMap<String, String>,
    /// Canonical name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub canonical_name: Option<String>,
}

/// SCC Indicator
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SccIndicator {
    /// IP addresses
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub ip_addresses: Vec<String>,
    /// Domains
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub domains: Vec<String>,
    /// Signatures
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub signatures: Vec<SccSignature>,
    /// URIs
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub uris: Vec<String>,
}

/// SCC Signature
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SccSignature {
    /// Memory hash signature
    #[serde(skip_serializing_if = "Option::is_none")]
    pub memory_hash_signature: Option<SccMemoryHashSignature>,
    /// YARA rule signature
    #[serde(skip_serializing_if = "Option::is_none")]
    pub yara_rule_signature: Option<SccYaraRuleSignature>,
}

/// Memory hash signature
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SccMemoryHashSignature {
    /// Binary family
    pub binary_family: String,
    /// Detections
    pub detections: Vec<SccDetection>,
}

/// YARA rule signature
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SccYaraRuleSignature {
    /// YARA rule
    pub yara_rule: String,
}

/// Detection
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SccDetection {
    /// Binary
    pub binary: String,
    /// Percent pages matched
    pub percent_pages_matched: f64,
}

/// SCC Vulnerability
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SccVulnerability {
    /// CVE
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cve: Option<SccCve>,
}

/// SCC CVE
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SccCve {
    /// CVE ID
    pub id: String,
    /// References
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub references: Vec<SccReference>,
    /// CVSS v3
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cvssv3: Option<SccCvssV3>,
    /// Upstream fix available
    #[serde(skip_serializing_if = "Option::is_none")]
    pub upstream_fix_available: Option<bool>,
    /// Impact
    #[serde(skip_serializing_if = "Option::is_none")]
    pub impact: Option<String>,
    /// Exploitation activity
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exploitation_activity: Option<String>,
}

/// SCC Reference
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SccReference {
    /// Source
    pub source: String,
    /// URI
    #[serde(skip_serializing_if = "Option::is_none")]
    pub uri: Option<String>,
}

/// SCC CVSS v3
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SccCvssV3 {
    /// Base score
    pub base_score: f64,
    /// Attack vector
    #[serde(skip_serializing_if = "Option::is_none")]
    pub attack_vector: Option<String>,
    /// Attack complexity
    #[serde(skip_serializing_if = "Option::is_none")]
    pub attack_complexity: Option<String>,
    /// Privileges required
    #[serde(skip_serializing_if = "Option::is_none")]
    pub privileges_required: Option<String>,
    /// User interaction
    #[serde(skip_serializing_if = "Option::is_none")]
    pub user_interaction: Option<String>,
    /// Scope
    #[serde(skip_serializing_if = "Option::is_none")]
    pub scope: Option<String>,
    /// Confidentiality impact
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidentiality_impact: Option<String>,
    /// Integrity impact
    #[serde(skip_serializing_if = "Option::is_none")]
    pub integrity_impact: Option<String>,
    /// Availability impact
    #[serde(skip_serializing_if = "Option::is_none")]
    pub availability_impact: Option<String>,
}

/// SCC Compliance
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SccCompliance {
    /// Standards
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub standards: Vec<String>,
    /// Version
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
    /// IDs
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub ids: Vec<String>,
}

/// SCC Exfiltration
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SccExfiltration {
    /// Sources
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub sources: Vec<SccExfilResource>,
    /// Targets
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub targets: Vec<SccExfilResource>,
}

/// Exfil resource
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SccExfilResource {
    /// Name
    pub name: String,
    /// Components
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub components: Vec<String>,
}

/// SCC Connection
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SccConnection {
    /// Destination IP
    #[serde(skip_serializing_if = "Option::is_none")]
    pub destination_ip: Option<String>,
    /// Destination port
    #[serde(skip_serializing_if = "Option::is_none")]
    pub destination_port: Option<i32>,
    /// Source IP
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source_ip: Option<String>,
    /// Source port
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source_port: Option<i32>,
    /// Protocol
    #[serde(skip_serializing_if = "Option::is_none")]
    pub protocol: Option<String>,
}

/// GCP SCC Exporter
pub struct GcpSccExporter {
    config: GcpSccConfig,
}

impl GcpSccExporter {
    /// Create new exporter
    pub fn new(config: GcpSccConfig) -> Self {
        Self { config }
    }

    /// Get source name
    fn source_name(&self) -> String {
        self.config.source_id.clone().unwrap_or_else(|| {
            format!(
                "organizations/{}/sources/wia-security",
                self.config.organization_id
            )
        })
    }

    /// Severity to GCP SCC severity
    fn to_scc_severity(severity: &str) -> String {
        match severity.to_lowercase().as_str() {
            "critical" => "CRITICAL".to_string(),
            "high" => "HIGH".to_string(),
            "medium" => "MEDIUM".to_string(),
            "low" => "LOW".to_string(),
            _ => "LOW".to_string(),
        }
    }

    /// Parse CVSS vector to components
    fn parse_cvss_vector(vector: &str) -> Option<SccCvssV3> {
        // Simple parser for CVSS:3.x vectors
        if !vector.starts_with("CVSS:3") {
            return None;
        }

        let mut cvss = SccCvssV3 {
            base_score: 0.0,
            attack_vector: None,
            attack_complexity: None,
            privileges_required: None,
            user_interaction: None,
            scope: None,
            confidentiality_impact: None,
            integrity_impact: None,
            availability_impact: None,
        };

        for part in vector.split('/') {
            if let Some((key, value)) = part.split_once(':') {
                match key {
                    "AV" => cvss.attack_vector = Some(match value {
                        "N" => "ATTACK_VECTOR_NETWORK",
                        "A" => "ATTACK_VECTOR_ADJACENT",
                        "L" => "ATTACK_VECTOR_LOCAL",
                        "P" => "ATTACK_VECTOR_PHYSICAL",
                        _ => "ATTACK_VECTOR_UNSPECIFIED",
                    }.to_string()),
                    "AC" => cvss.attack_complexity = Some(match value {
                        "L" => "ATTACK_COMPLEXITY_LOW",
                        "H" => "ATTACK_COMPLEXITY_HIGH",
                        _ => "ATTACK_COMPLEXITY_UNSPECIFIED",
                    }.to_string()),
                    "PR" => cvss.privileges_required = Some(match value {
                        "N" => "PRIVILEGES_REQUIRED_NONE",
                        "L" => "PRIVILEGES_REQUIRED_LOW",
                        "H" => "PRIVILEGES_REQUIRED_HIGH",
                        _ => "PRIVILEGES_REQUIRED_UNSPECIFIED",
                    }.to_string()),
                    "UI" => cvss.user_interaction = Some(match value {
                        "N" => "USER_INTERACTION_NONE",
                        "R" => "USER_INTERACTION_REQUIRED",
                        _ => "USER_INTERACTION_UNSPECIFIED",
                    }.to_string()),
                    "S" => cvss.scope = Some(match value {
                        "U" => "SCOPE_UNCHANGED",
                        "C" => "SCOPE_CHANGED",
                        _ => "SCOPE_UNSPECIFIED",
                    }.to_string()),
                    "C" => cvss.confidentiality_impact = Some(match value {
                        "N" => "IMPACT_NONE",
                        "L" => "IMPACT_LOW",
                        "H" => "IMPACT_HIGH",
                        _ => "IMPACT_UNSPECIFIED",
                    }.to_string()),
                    "I" => cvss.integrity_impact = Some(match value {
                        "N" => "IMPACT_NONE",
                        "L" => "IMPACT_LOW",
                        "H" => "IMPACT_HIGH",
                        _ => "IMPACT_UNSPECIFIED",
                    }.to_string()),
                    "A" => cvss.availability_impact = Some(match value {
                        "N" => "IMPACT_NONE",
                        "L" => "IMPACT_LOW",
                        "H" => "IMPACT_HIGH",
                        _ => "IMPACT_UNSPECIFIED",
                    }.to_string()),
                    _ => {}
                }
            }
        }

        Some(cvss)
    }

    /// Convert WIA finding to SCC Finding
    pub fn convert_finding(&self, finding: &super::super::importers::WiaFinding, host: &str) -> SccFinding {
        let now = chrono::Utc::now().to_rfc3339();
        let source_name = self.source_name();
        let finding_name = format!("{}/findings/{}", source_name, finding.id);
        let resource_name = format!(
            "//compute.googleapis.com/projects/{}/zones/global/instances/{}",
            self.config.project_id,
            host.replace('.', "-")
        );

        let mut source_properties = HashMap::new();
        source_properties.insert("scanner".to_string(), serde_json::Value::String("WIA Security".to_string()));
        source_properties.insert("findingId".to_string(), serde_json::Value::String(finding.id.clone()));
        if let Some(port) = finding.port {
            source_properties.insert("port".to_string(), serde_json::Value::Number(serde_json::Number::from(port)));
        }
        if let Some(ref protocol) = finding.protocol {
            source_properties.insert("protocol".to_string(), serde_json::Value::String(protocol.clone()));
        }
        source_properties.insert("exploitAvailable".to_string(), serde_json::Value::Bool(finding.exploit_available));

        let vulnerability = if !finding.cve.is_empty() {
            let cve_id = finding.cve.first().cloned().unwrap_or_default();
            let references: Vec<SccReference> = finding.references.iter().map(|url| SccReference {
                source: "WIA Security".to_string(),
                uri: Some(url.clone()),
            }).collect();

            let mut cvssv3 = finding.cvss_vector.as_ref()
                .and_then(|v| Self::parse_cvss_vector(v));

            if let Some(ref mut cvss) = cvssv3 {
                cvss.base_score = finding.cvss_score.unwrap_or(0.0);
            }

            Some(SccVulnerability {
                cve: Some(SccCve {
                    id: cve_id,
                    references,
                    cvssv3,
                    upstream_fix_available: Some(finding.patch_available),
                    impact: None,
                    exploitation_activity: if finding.exploit_available {
                        Some("WIDE".to_string())
                    } else {
                        Some("NO_KNOWN".to_string())
                    },
                }),
            })
        } else {
            None
        };

        let connections = if let Some(port) = finding.port {
            vec![SccConnection {
                destination_ip: Some(host.to_string()),
                destination_port: Some(port as i32),
                source_ip: None,
                source_port: None,
                protocol: finding.protocol.clone(),
            }]
        } else {
            vec![]
        };

        let indicator = Some(SccIndicator {
            ip_addresses: vec![host.to_string()],
            domains: vec![],
            signatures: vec![],
            uris: vec![],
        });

        SccFinding {
            name: finding_name,
            parent: source_name,
            resource_name,
            state: "ACTIVE".to_string(),
            category: "VULNERABILITY".to_string(),
            external_uri: finding.references.first().cloned(),
            source_properties,
            security_marks: None,
            event_time: now.clone(),
            create_time: Some(now),
            severity: Self::to_scc_severity(&finding.severity),
            canonical_name: None,
            mute: Some("UNMUTED".to_string()),
            finding_class: "VULNERABILITY".to_string(),
            indicator,
            vulnerability,
            compliance: None,
            description: Some(finding.description.clone()),
            exfiltration: None,
            connections,
            next_steps: finding.solution.clone(),
        }
    }

    /// Convert scan results to SCC findings
    pub fn convert_scan_results(&self, scan: &super::super::importers::WiaScanResult) -> Vec<SccFinding> {
        let mut findings = Vec::new();
        for target in &scan.targets {
            for finding in &target.findings {
                findings.push(self.convert_finding(finding, &target.host));
            }
        }
        findings
    }

    /// Get SCC API endpoint
    pub fn api_endpoint(&self) -> String {
        format!(
            "https://securitycenter.googleapis.com/v1/organizations/{}/sources",
            self.config.organization_id
        )
    }

    /// Get findings create endpoint
    pub fn findings_endpoint(&self) -> String {
        format!(
            "https://securitycenter.googleapis.com/v1/{}/findings",
            self.source_name()
        )
    }

    /// Serialize finding for API
    pub fn to_api_json(&self, finding: &SccFinding) -> CloudResult<String> {
        serde_json::to_string_pretty(finding)
            .map_err(|e| CloudError::SerializationError(e.to_string()))
    }

    /// Batch create request
    pub fn to_batch_json(&self, findings: &[SccFinding]) -> CloudResult<String> {
        let request = serde_json::json!({
            "requests": findings.iter().map(|f| {
                serde_json::json!({
                    "parent": f.parent,
                    "findingId": f.name.split('/').last().unwrap_or(&f.name),
                    "finding": f
                })
            }).collect::<Vec<_>>()
        });
        serde_json::to_string_pretty(&request)
            .map_err(|e| CloudError::SerializationError(e.to_string()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_severity_mapping() {
        assert_eq!(GcpSccExporter::to_scc_severity("critical"), "CRITICAL");
        assert_eq!(GcpSccExporter::to_scc_severity("high"), "HIGH");
        assert_eq!(GcpSccExporter::to_scc_severity("medium"), "MEDIUM");
        assert_eq!(GcpSccExporter::to_scc_severity("low"), "LOW");
    }

    #[test]
    fn test_convert_finding() {
        let config = GcpSccConfig {
            project_id: "test-project".to_string(),
            organization_id: "123456789".to_string(),
            ..Default::default()
        };
        let exporter = GcpSccExporter::new(config);

        let finding = super::super::super::importers::WiaFinding {
            id: "TEST-001".to_string(),
            title: "Test Vulnerability".to_string(),
            description: "Test description".to_string(),
            severity: "high".to_string(),
            cvss_score: Some(7.5),
            cvss_vector: Some("CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:H/I:N/A:N".to_string()),
            port: Some(443),
            protocol: Some("tcp".to_string()),
            service: None,
            cve: vec!["CVE-2024-1234".to_string()],
            cwe: vec![],
            solution: Some("Fix it".to_string()),
            references: vec!["https://example.com".to_string()],
            exploit_available: false,
            patch_available: true,
            plugin_output: None,
        };

        let scc = exporter.convert_finding(&finding, "192.168.1.100");

        assert_eq!(scc.severity, "HIGH");
        assert_eq!(scc.state, "ACTIVE");
        assert!(scc.vulnerability.is_some());
    }

    #[test]
    fn test_parse_cvss_vector() {
        let cvss = GcpSccExporter::parse_cvss_vector("CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:H/I:N/A:N");
        assert!(cvss.is_some());

        let cvss = cvss.unwrap();
        assert_eq!(cvss.attack_vector, Some("ATTACK_VECTOR_NETWORK".to_string()));
        assert_eq!(cvss.attack_complexity, Some("ATTACK_COMPLEXITY_LOW".to_string()));
    }
}
