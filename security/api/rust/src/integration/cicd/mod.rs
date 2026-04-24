//! CI/CD Pipeline Integrations
//!
//! Generate CI/CD pipeline configurations for security scanning integration.

pub mod github;
pub mod gitlab;

pub use github::*;
pub use gitlab::*;

use serde::{Deserialize, Serialize};

/// CI/CD integration error
#[derive(Debug, thiserror::Error)]
pub enum CicdError {
    #[error("Template error: {0}")]
    TemplateError(String),

    #[error("Configuration error: {0}")]
    ConfigError(String),

    #[error("Serialization error: {0}")]
    SerializationError(String),
}

/// CI/CD result type
pub type CicdResult<T> = Result<T, CicdError>;

/// Common security scan configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SecurityScanConfig {
    /// Fail build on critical findings
    pub fail_on_critical: bool,
    /// Fail build on high findings
    pub fail_on_high: bool,
    /// Maximum allowed critical findings
    pub max_critical: u32,
    /// Maximum allowed high findings
    pub max_high: u32,
    /// Scan timeout in minutes
    pub timeout_minutes: u32,
    /// Enable SBOM generation
    pub enable_sbom: bool,
    /// Upload results to cloud
    pub upload_results: bool,
    /// Scan targets
    pub targets: Vec<String>,
}

impl Default for SecurityScanConfig {
    fn default() -> Self {
        Self {
            fail_on_critical: true,
            fail_on_high: false,
            max_critical: 0,
            max_high: 5,
            timeout_minutes: 30,
            enable_sbom: true,
            upload_results: true,
            targets: vec![".".to_string()],
        }
    }
}

/// Scan result summary for CI/CD
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScanResultSummary {
    pub scan_id: String,
    pub status: ScanStatus,
    pub findings_total: u32,
    pub findings_critical: u32,
    pub findings_high: u32,
    pub findings_medium: u32,
    pub findings_low: u32,
    pub sbom_generated: bool,
    pub report_url: Option<String>,
    pub duration_seconds: u64,
}

/// Scan status
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum ScanStatus {
    Success,
    Warning,
    Failed,
    Error,
}

impl ScanResultSummary {
    /// Check if scan should fail the build
    pub fn should_fail(&self, config: &SecurityScanConfig) -> bool {
        if config.fail_on_critical && self.findings_critical > config.max_critical {
            return true;
        }
        if config.fail_on_high && self.findings_high > config.max_high {
            return true;
        }
        false
    }

    /// Get exit code for CI
    pub fn exit_code(&self, config: &SecurityScanConfig) -> i32 {
        match self.status {
            ScanStatus::Error => 2,
            ScanStatus::Failed => 1,
            ScanStatus::Warning => {
                if self.should_fail(config) { 1 } else { 0 }
            }
            ScanStatus::Success => 0,
        }
    }
}

/// Generate SARIF output for GitHub/GitLab integration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SarifReport {
    #[serde(rename = "$schema")]
    pub schema: String,
    pub version: String,
    pub runs: Vec<SarifRun>,
}

/// SARIF run
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SarifRun {
    pub tool: SarifTool,
    pub results: Vec<SarifResult>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub invocations: Vec<SarifInvocation>,
}

/// SARIF tool
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SarifTool {
    pub driver: SarifDriver,
}

/// SARIF driver
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SarifDriver {
    pub name: String,
    pub version: String,
    pub information_uri: Option<String>,
    pub rules: Vec<SarifRule>,
}

/// SARIF rule
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SarifRule {
    pub id: String,
    pub name: String,
    pub short_description: SarifMessage,
    pub full_description: Option<SarifMessage>,
    pub help: Option<SarifMessage>,
    pub default_configuration: SarifConfiguration,
    pub properties: Option<SarifRuleProperties>,
}

/// SARIF message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SarifMessage {
    pub text: String,
}

/// SARIF configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SarifConfiguration {
    pub level: String,
}

/// SARIF rule properties
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SarifRuleProperties {
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub tags: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub precision: Option<String>,
    #[serde(rename = "security-severity", skip_serializing_if = "Option::is_none")]
    pub security_severity: Option<String>,
}

/// SARIF result
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SarifResult {
    pub rule_id: String,
    pub level: String,
    pub message: SarifMessage,
    pub locations: Vec<SarifLocation>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fingerprints: Option<std::collections::HashMap<String, String>>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub related_locations: Vec<SarifLocation>,
}

/// SARIF location
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SarifLocation {
    pub physical_location: SarifPhysicalLocation,
}

/// SARIF physical location
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SarifPhysicalLocation {
    pub artifact_location: SarifArtifactLocation,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub region: Option<SarifRegion>,
}

/// SARIF artifact location
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SarifArtifactLocation {
    pub uri: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub index: Option<usize>,
}

/// SARIF region
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SarifRegion {
    pub start_line: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub start_column: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub end_line: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub end_column: Option<u32>,
}

/// SARIF invocation
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SarifInvocation {
    pub execution_successful: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub command_line: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub working_directory: Option<SarifArtifactLocation>,
}

/// Convert WIA findings to SARIF format
pub fn convert_to_sarif(scan: &super::importers::WiaScanResult) -> SarifReport {
    let mut rules = Vec::new();
    let mut results = Vec::new();

    for target in &scan.targets {
        for finding in &target.findings {
            let level = match finding.severity.as_str() {
                "critical" | "high" => "error",
                "medium" => "warning",
                _ => "note",
            };

            let security_severity = finding.cvss_score.map(|s| s.to_string());

            // Add rule if not already present
            if !rules.iter().any(|r: &SarifRule| r.id == finding.id) {
                rules.push(SarifRule {
                    id: finding.id.clone(),
                    name: finding.title.clone(),
                    short_description: SarifMessage {
                        text: finding.title.clone(),
                    },
                    full_description: Some(SarifMessage {
                        text: finding.description.clone(),
                    }),
                    help: finding.solution.as_ref().map(|s| SarifMessage { text: s.clone() }),
                    default_configuration: SarifConfiguration {
                        level: level.to_string(),
                    },
                    properties: Some(SarifRuleProperties {
                        tags: vec!["security".to_string(), finding.severity.clone()],
                        precision: Some("high".to_string()),
                        security_severity,
                    }),
                });
            }

            results.push(SarifResult {
                rule_id: finding.id.clone(),
                level: level.to_string(),
                message: SarifMessage {
                    text: format!("{}: {}", finding.title, finding.description),
                },
                locations: vec![SarifLocation {
                    physical_location: SarifPhysicalLocation {
                        artifact_location: SarifArtifactLocation {
                            uri: target.host.clone(),
                            index: None,
                        },
                        region: finding.port.map(|p| SarifRegion {
                            start_line: p as u32,
                            start_column: None,
                            end_line: None,
                            end_column: None,
                        }),
                    },
                }],
                fingerprints: None,
                related_locations: vec![],
            });
        }
    }

    SarifReport {
        schema: "https://raw.githubusercontent.com/oasis-tcs/sarif-spec/master/Schemata/sarif-schema-2.1.0.json".to_string(),
        version: "2.1.0".to_string(),
        runs: vec![SarifRun {
            tool: SarifTool {
                driver: SarifDriver {
                    name: "WIA Security Scanner".to_string(),
                    version: "1.0.0".to_string(),
                    information_uri: Some("https://github.com/WIA-Official/wia-standards".to_string()),
                    rules,
                },
            },
            results,
            invocations: vec![SarifInvocation {
                execution_successful: true,
                command_line: Some("wia-security scan".to_string()),
                working_directory: None,
            }],
        }],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scan_result_should_fail() {
        let config = SecurityScanConfig {
            fail_on_critical: true,
            max_critical: 0,
            ..Default::default()
        };

        let result = ScanResultSummary {
            scan_id: "test".to_string(),
            status: ScanStatus::Warning,
            findings_total: 5,
            findings_critical: 1,
            findings_high: 2,
            findings_medium: 1,
            findings_low: 1,
            sbom_generated: false,
            report_url: None,
            duration_seconds: 60,
        };

        assert!(result.should_fail(&config));
    }

    #[test]
    fn test_exit_codes() {
        let config = SecurityScanConfig::default();

        let success = ScanResultSummary {
            scan_id: "test".to_string(),
            status: ScanStatus::Success,
            findings_total: 0,
            findings_critical: 0,
            findings_high: 0,
            findings_medium: 0,
            findings_low: 0,
            sbom_generated: true,
            report_url: None,
            duration_seconds: 30,
        };

        assert_eq!(success.exit_code(&config), 0);

        let error = ScanResultSummary {
            status: ScanStatus::Error,
            ..success.clone()
        };

        assert_eq!(error.exit_code(&config), 2);
    }
}
