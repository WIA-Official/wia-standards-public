//! AWS Security Hub Integration
//!
//! Send security findings to AWS Security Hub using ASFF format.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use super::{CloudResult, CloudError};

/// AWS Security Hub configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AwsSecurityHubConfig {
    /// AWS Region
    pub region: String,
    /// AWS Account ID
    pub account_id: String,
    /// Product ARN (registered with Security Hub)
    pub product_arn: Option<String>,
    /// Enable batch import
    pub batch_import: bool,
    /// Batch size
    pub batch_size: usize,
    /// AWS credentials (if not using instance profile/env)
    pub credentials: Option<AwsCredentials>,
}

/// AWS credentials
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AwsCredentials {
    pub access_key_id: String,
    pub secret_access_key: String,
    pub session_token: Option<String>,
}

impl Default for AwsSecurityHubConfig {
    fn default() -> Self {
        Self {
            region: "us-east-1".to_string(),
            account_id: String::new(),
            product_arn: None,
            batch_import: true,
            batch_size: 100,
            credentials: None,
        }
    }
}

/// AWS Security Finding Format (ASFF)
/// Based on AWS Security Hub ASFF specification
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct AsffFinding {
    /// Schema version
    pub schema_version: String,
    /// Finding ID (must be unique)
    pub id: String,
    /// Product ARN
    pub product_arn: String,
    /// Generator ID
    pub generator_id: String,
    /// AWS Account ID
    pub aws_account_id: String,
    /// Finding types
    pub types: Vec<String>,
    /// First observed timestamp
    pub first_observed_at: Option<String>,
    /// Last observed timestamp
    pub last_observed_at: Option<String>,
    /// Created timestamp
    pub created_at: String,
    /// Updated timestamp
    pub updated_at: String,
    /// Severity
    pub severity: AsffSeverity,
    /// Confidence (0-100)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidence: Option<u32>,
    /// Criticality (0-100)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub criticality: Option<u32>,
    /// Title
    pub title: String,
    /// Description
    pub description: String,
    /// Remediation
    #[serde(skip_serializing_if = "Option::is_none")]
    pub remediation: Option<AsffRemediation>,
    /// Product fields
    #[serde(skip_serializing_if = "Option::is_none")]
    pub product_fields: Option<HashMap<String, String>>,
    /// User defined fields
    #[serde(skip_serializing_if = "Option::is_none")]
    pub user_defined_fields: Option<HashMap<String, String>>,
    /// Resources affected
    pub resources: Vec<AsffResource>,
    /// Compliance status
    #[serde(skip_serializing_if = "Option::is_none")]
    pub compliance: Option<AsffCompliance>,
    /// Verification state
    #[serde(skip_serializing_if = "Option::is_none")]
    pub verification_state: Option<String>,
    /// Workflow state
    #[serde(skip_serializing_if = "Option::is_none")]
    pub workflow_state: Option<String>,
    /// Workflow status
    #[serde(skip_serializing_if = "Option::is_none")]
    pub workflow: Option<AsffWorkflow>,
    /// Record state
    pub record_state: String,
    /// Related findings
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub related_findings: Vec<AsffRelatedFinding>,
    /// Note
    #[serde(skip_serializing_if = "Option::is_none")]
    pub note: Option<AsffNote>,
    /// Vulnerabilities
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub vulnerabilities: Vec<AsffVulnerability>,
}

/// ASFF Severity
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct AsffSeverity {
    /// Severity label
    pub label: String,
    /// Normalized severity (0-100)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub normalized: Option<u32>,
    /// Original severity from the source
    #[serde(skip_serializing_if = "Option::is_none")]
    pub original: Option<String>,
}

/// ASFF Remediation
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct AsffRemediation {
    /// Recommendation
    pub recommendation: AsffRecommendation,
}

/// ASFF Recommendation
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct AsffRecommendation {
    /// Recommendation text
    pub text: String,
    /// URL for more info
    #[serde(skip_serializing_if = "Option::is_none")]
    pub url: Option<String>,
}

/// ASFF Resource
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct AsffResource {
    /// Resource type
    #[serde(rename = "Type")]
    pub resource_type: String,
    /// Resource ID
    pub id: String,
    /// Partition
    #[serde(skip_serializing_if = "Option::is_none")]
    pub partition: Option<String>,
    /// Region
    #[serde(skip_serializing_if = "Option::is_none")]
    pub region: Option<String>,
    /// Resource role
    #[serde(skip_serializing_if = "Option::is_none")]
    pub resource_role: Option<String>,
    /// Tags
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tags: Option<HashMap<String, String>>,
    /// Details
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
}

/// ASFF Compliance
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct AsffCompliance {
    /// Compliance status
    pub status: String,
    /// Related requirements
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub related_requirements: Vec<String>,
}

/// ASFF Workflow
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct AsffWorkflow {
    /// Status
    pub status: String,
}

/// ASFF Related Finding
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct AsffRelatedFinding {
    /// Product ARN
    pub product_arn: String,
    /// Finding ID
    pub id: String,
}

/// ASFF Note
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct AsffNote {
    /// Note text
    pub text: String,
    /// Updated by
    pub updated_by: String,
    /// Updated at
    pub updated_at: String,
}

/// ASFF Vulnerability
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct AsffVulnerability {
    /// Vulnerability ID
    pub id: String,
    /// Vulnerable packages
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub vulnerable_packages: Vec<AsffPackage>,
    /// CVSS scores
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub cvss: Vec<AsffCvss>,
    /// Related vulnerabilities
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub related_vulnerabilities: Vec<String>,
    /// Vendor
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vendor: Option<AsffVendor>,
    /// Reference URLs
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub reference_urls: Vec<String>,
}

/// ASFF Package
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct AsffPackage {
    /// Package name
    pub name: String,
    /// Version
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
    /// Epoch
    #[serde(skip_serializing_if = "Option::is_none")]
    pub epoch: Option<String>,
    /// Release
    #[serde(skip_serializing_if = "Option::is_none")]
    pub release: Option<String>,
    /// Architecture
    #[serde(skip_serializing_if = "Option::is_none")]
    pub architecture: Option<String>,
}

/// ASFF CVSS
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct AsffCvss {
    /// CVSS version
    pub version: String,
    /// Base score
    pub base_score: f64,
    /// Base vector
    #[serde(skip_serializing_if = "Option::is_none")]
    pub base_vector: Option<String>,
}

/// ASFF Vendor
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct AsffVendor {
    /// Vendor name
    pub name: String,
    /// Vendor URL
    #[serde(skip_serializing_if = "Option::is_none")]
    pub url: Option<String>,
    /// Vendor severity
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vendor_severity: Option<String>,
}

/// AWS Security Hub Exporter
pub struct AwsSecurityHubExporter {
    config: AwsSecurityHubConfig,
}

impl AwsSecurityHubExporter {
    /// Create new exporter
    pub fn new(config: AwsSecurityHubConfig) -> Self {
        Self { config }
    }

    /// Get product ARN
    fn product_arn(&self) -> String {
        self.config.product_arn.clone().unwrap_or_else(|| {
            format!(
                "arn:aws:securityhub:{}:{}:product/{}/wia-security",
                self.config.region,
                self.config.account_id,
                self.config.account_id
            )
        })
    }

    /// Severity string to normalized value (0-100)
    fn severity_to_normalized(severity: &str) -> u32 {
        match severity.to_lowercase().as_str() {
            "critical" => 90,
            "high" => 70,
            "medium" => 50,
            "low" => 30,
            "info" | "informational" => 10,
            _ => 0,
        }
    }

    /// Convert WIA finding to ASFF format
    pub fn convert_finding(&self, finding: &super::super::importers::WiaFinding, host: &str) -> AsffFinding {
        let now = chrono::Utc::now().to_rfc3339();
        let finding_id = format!(
            "arn:aws:securityhub:{}:{}:finding/{}/wia/{}",
            self.config.region,
            self.config.account_id,
            self.config.account_id,
            finding.id
        );

        let severity_label = super::map_severity_to_label(&finding.severity);
        let normalized = Self::severity_to_normalized(&finding.severity);

        let mut product_fields = HashMap::new();
        product_fields.insert("Provider".to_string(), "WIA Security".to_string());
        product_fields.insert("ProviderVersion".to_string(), "1.0.0".to_string());
        if let Some(ref port) = finding.port {
            product_fields.insert("Port".to_string(), port.to_string());
        }
        if let Some(ref protocol) = finding.protocol {
            product_fields.insert("Protocol".to_string(), protocol.clone());
        }

        let remediation = finding.solution.as_ref().map(|sol| AsffRemediation {
            recommendation: AsffRecommendation {
                text: sol.clone(),
                url: finding.references.first().cloned(),
            },
        });

        let vulnerabilities = if !finding.cve.is_empty() {
            finding.cve.iter().map(|cve| AsffVulnerability {
                id: cve.clone(),
                vulnerable_packages: vec![],
                cvss: finding.cvss_score.map(|score| vec![AsffCvss {
                    version: "3.1".to_string(),
                    base_score: score,
                    base_vector: finding.cvss_vector.clone(),
                }]).unwrap_or_default(),
                related_vulnerabilities: vec![],
                vendor: None,
                reference_urls: finding.references.clone(),
            }).collect()
        } else {
            vec![]
        };

        let types = vec![
            "Software and Configuration Checks/Vulnerabilities/CVE".to_string(),
        ];

        AsffFinding {
            schema_version: "2018-10-08".to_string(),
            id: finding_id,
            product_arn: self.product_arn(),
            generator_id: format!("wia-security/{}", finding.id),
            aws_account_id: self.config.account_id.clone(),
            types,
            first_observed_at: Some(now.clone()),
            last_observed_at: Some(now.clone()),
            created_at: now.clone(),
            updated_at: now,
            severity: AsffSeverity {
                label: severity_label.to_string(),
                normalized: Some(normalized),
                original: Some(finding.severity.clone()),
            },
            confidence: Some(80),
            criticality: finding.cvss_score.map(|s| (s * 10.0) as u32),
            title: finding.title.clone(),
            description: finding.description.clone(),
            remediation,
            product_fields: Some(product_fields),
            user_defined_fields: None,
            resources: vec![AsffResource {
                resource_type: "Other".to_string(),
                id: host.to_string(),
                partition: Some("aws".to_string()),
                region: Some(self.config.region.clone()),
                resource_role: Some("Target".to_string()),
                tags: None,
                details: None,
            }],
            compliance: None,
            verification_state: Some("TRUE_POSITIVE".to_string()),
            workflow_state: None,
            workflow: Some(AsffWorkflow {
                status: "NEW".to_string(),
            }),
            record_state: "ACTIVE".to_string(),
            related_findings: vec![],
            note: None,
            vulnerabilities,
        }
    }

    /// Convert scan results to ASFF findings
    pub fn convert_scan_results(&self, scan: &super::super::importers::WiaScanResult) -> Vec<AsffFinding> {
        let mut findings = Vec::new();
        for target in &scan.targets {
            for finding in &target.findings {
                findings.push(self.convert_finding(finding, &target.host));
            }
        }
        findings
    }

    /// Serialize findings for BatchImportFindings API
    pub fn to_batch_import_json(&self, findings: &[AsffFinding]) -> CloudResult<String> {
        let batch = serde_json::json!({
            "Findings": findings
        });
        serde_json::to_string_pretty(&batch)
            .map_err(|e| CloudError::SerializationError(e.to_string()))
    }

    /// Get Security Hub endpoint URL
    pub fn endpoint_url(&self) -> String {
        format!("https://securityhub.{}.amazonaws.com", self.config.region)
    }

    /// Create sample findings for testing
    pub fn sample_findings() -> Vec<AsffFinding> {
        let config = AwsSecurityHubConfig {
            region: "us-east-1".to_string(),
            account_id: "123456789012".to_string(),
            ..Default::default()
        };
        let exporter = AwsSecurityHubExporter::new(config);

        let finding = super::super::importers::WiaFinding {
            id: "TEST-001".to_string(),
            title: "Test Vulnerability".to_string(),
            description: "A test vulnerability for demonstration".to_string(),
            severity: "high".to_string(),
            cvss_score: Some(7.5),
            cvss_vector: Some("CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:H/I:N/A:N".to_string()),
            port: Some(443),
            protocol: Some("tcp".to_string()),
            service: Some("https".to_string()),
            cve: vec!["CVE-2024-1234".to_string()],
            cwe: vec!["CWE-79".to_string()],
            solution: Some("Update to latest version".to_string()),
            references: vec!["https://example.com/advisory".to_string()],
            exploit_available: false,
            patch_available: true,
            plugin_output: None,
        };

        vec![exporter.convert_finding(&finding, "192.168.1.100")]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_convert_finding() {
        let config = AwsSecurityHubConfig {
            region: "us-east-1".to_string(),
            account_id: "123456789012".to_string(),
            ..Default::default()
        };
        let exporter = AwsSecurityHubExporter::new(config);

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

        let asff = exporter.convert_finding(&finding, "192.168.1.100");

        assert_eq!(asff.severity.label, "HIGH");
        assert_eq!(asff.severity.normalized, Some(70));
        assert!(!asff.vulnerabilities.is_empty());
    }

    #[test]
    fn test_severity_mapping() {
        assert_eq!(AwsSecurityHubExporter::severity_to_normalized("critical"), 90);
        assert_eq!(AwsSecurityHubExporter::severity_to_normalized("high"), 70);
        assert_eq!(AwsSecurityHubExporter::severity_to_normalized("medium"), 50);
    }
}
