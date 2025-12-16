//! Nessus XML Scan Report Importer
//!
//! Parses Nessus .nessus XML files and converts to WIA Security format.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use super::{ImportResult, ImportError};

/// Nessus report structure (from XML)
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename = "NessusClientData_v2")]
pub struct NessusReport {
    #[serde(rename = "Policy")]
    pub policy: Option<NessusPolicy>,
    #[serde(rename = "Report")]
    pub report: NessusReportData,
}

/// Nessus policy
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NessusPolicy {
    #[serde(rename = "policyName")]
    pub policy_name: Option<String>,
}

/// Nessus report data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NessusReportData {
    #[serde(rename = "@name")]
    pub name: Option<String>,
    #[serde(rename = "ReportHost", default)]
    pub hosts: Vec<NessusReportHost>,
}

/// Nessus report host
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NessusReportHost {
    #[serde(rename = "@name")]
    pub name: String,
    #[serde(rename = "HostProperties")]
    pub properties: Option<NessusHostProperties>,
    #[serde(rename = "ReportItem", default)]
    pub items: Vec<NessusReportItem>,
}

/// Nessus host properties
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NessusHostProperties {
    #[serde(rename = "tag", default)]
    pub tags: Vec<NessusTag>,
}

/// Nessus tag
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NessusTag {
    #[serde(rename = "@name")]
    pub name: String,
    #[serde(rename = "$value")]
    pub value: Option<String>,
}

/// Nessus report item
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NessusReportItem {
    #[serde(rename = "@port")]
    pub port: u16,
    #[serde(rename = "@svc_name")]
    pub svc_name: Option<String>,
    #[serde(rename = "@protocol")]
    pub protocol: Option<String>,
    #[serde(rename = "@severity")]
    pub severity: u8,
    #[serde(rename = "@pluginID")]
    pub plugin_id: String,
    #[serde(rename = "@pluginName")]
    pub plugin_name: String,
    #[serde(rename = "@pluginFamily")]
    pub plugin_family: Option<String>,
    pub synopsis: Option<String>,
    pub description: Option<String>,
    pub solution: Option<String>,
    pub risk_factor: Option<String>,
    pub see_also: Option<String>,
    pub cvss_base_score: Option<String>,
    pub cvss_vector: Option<String>,
    pub cvss3_base_score: Option<String>,
    pub cvss3_vector: Option<String>,
    pub cve: Option<String>,
    pub cwe: Option<String>,
    pub xref: Option<String>,
    pub plugin_output: Option<String>,
    pub exploit_available: Option<String>,
    pub exploitability_ease: Option<String>,
    pub patch_publication_date: Option<String>,
    pub vuln_publication_date: Option<String>,
}

/// WIA Security vulnerability scan result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaScanResult {
    pub scan_id: String,
    pub scan_name: String,
    pub scan_time: String,
    pub targets: Vec<WiaScanTarget>,
    pub summary: WiaScanSummary,
}

/// WIA scan target
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaScanTarget {
    pub host: String,
    pub ip: Option<String>,
    pub os: Option<String>,
    pub findings: Vec<WiaFinding>,
}

/// WIA finding
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaFinding {
    pub id: String,
    pub title: String,
    pub description: String,
    pub severity: String,
    pub cvss_score: Option<f64>,
    pub cvss_vector: Option<String>,
    pub port: Option<u16>,
    pub protocol: Option<String>,
    pub service: Option<String>,
    pub cve: Vec<String>,
    pub cwe: Vec<String>,
    pub solution: Option<String>,
    pub references: Vec<String>,
    pub exploit_available: bool,
    pub patch_available: bool,
    pub plugin_output: Option<String>,
}

/// WIA scan summary
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaScanSummary {
    pub total_hosts: u32,
    pub total_findings: u32,
    pub critical: u32,
    pub high: u32,
    pub medium: u32,
    pub low: u32,
    pub info: u32,
}

/// Nessus XML Importer
pub struct NessusImporter;

impl NessusImporter {
    /// Create new importer
    pub fn new() -> Self {
        Self
    }

    /// Parse Nessus XML content (simplified parsing without quick-xml dependency)
    /// In production, use quick-xml or roxmltree for proper XML parsing
    pub fn parse_simplified(&self, items: Vec<NessusReportItem>, host_name: &str) -> WiaScanTarget {
        let mut findings = Vec::new();

        for item in items {
            // Skip informational items (severity 0)
            if item.severity == 0 {
                continue;
            }

            let severity = match item.severity {
                4 => "critical",
                3 => "high",
                2 => "medium",
                1 => "low",
                _ => "info",
            }.to_string();

            let cvss_score = item.cvss3_base_score
                .as_ref()
                .or(item.cvss_base_score.as_ref())
                .and_then(|s| s.parse::<f64>().ok());

            let cvss_vector = item.cvss3_vector
                .clone()
                .or(item.cvss_vector.clone());

            // Parse CVE list
            let cve = item.cve
                .as_ref()
                .map(|s| s.split(',').map(|c| c.trim().to_string()).collect())
                .unwrap_or_default();

            // Parse CWE
            let cwe = item.cwe
                .as_ref()
                .map(|s| vec![s.clone()])
                .unwrap_or_default();

            // Parse references from see_also
            let references = item.see_also
                .as_ref()
                .map(|s| s.lines().map(|l| l.trim().to_string()).collect())
                .unwrap_or_default();

            let exploit_available = item.exploit_available
                .as_ref()
                .map(|s| s == "true" || s == "True")
                .unwrap_or(false);

            let patch_available = item.patch_publication_date.is_some();

            findings.push(WiaFinding {
                id: format!("NESSUS-{}", item.plugin_id),
                title: item.plugin_name.clone(),
                description: item.description.unwrap_or_else(|| item.synopsis.unwrap_or_default()),
                severity,
                cvss_score,
                cvss_vector,
                port: Some(item.port),
                protocol: item.protocol,
                service: item.svc_name,
                cve,
                cwe,
                solution: item.solution,
                references,
                exploit_available,
                patch_available,
                plugin_output: item.plugin_output,
            });
        }

        WiaScanTarget {
            host: host_name.to_string(),
            ip: None,
            os: None,
            findings,
        }
    }

    /// Convert Nessus report to WIA format
    pub fn convert_to_wia(&self, report: &NessusReport) -> WiaScanResult {
        let scan_name = report.report.name.clone().unwrap_or_else(|| "Nessus Scan".to_string());
        let scan_id = format!("nessus-{}", uuid::Uuid::new_v4());

        let mut targets = Vec::new();
        let mut summary = WiaScanSummary {
            total_hosts: 0,
            total_findings: 0,
            critical: 0,
            high: 0,
            medium: 0,
            low: 0,
            info: 0,
        };

        for host in &report.report.hosts {
            let target = self.parse_simplified(host.items.clone(), &host.name);

            // Update summary
            summary.total_hosts += 1;
            for finding in &target.findings {
                summary.total_findings += 1;
                match finding.severity.as_str() {
                    "critical" => summary.critical += 1,
                    "high" => summary.high += 1,
                    "medium" => summary.medium += 1,
                    "low" => summary.low += 1,
                    _ => summary.info += 1,
                }
            }

            targets.push(target);
        }

        WiaScanResult {
            scan_id,
            scan_name,
            scan_time: chrono::Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string(),
            targets,
            summary,
        }
    }

    /// Create sample Nessus data for testing
    pub fn sample_report_items() -> Vec<NessusReportItem> {
        vec![
            NessusReportItem {
                port: 443,
                svc_name: Some("https".to_string()),
                protocol: Some("tcp".to_string()),
                severity: 4,
                plugin_id: "12345".to_string(),
                plugin_name: "SSL Certificate Expired".to_string(),
                plugin_family: Some("SSL/TLS".to_string()),
                synopsis: Some("The remote SSL certificate has expired.".to_string()),
                description: Some("The SSL certificate has expired and should be renewed immediately.".to_string()),
                solution: Some("Renew the SSL certificate.".to_string()),
                risk_factor: Some("Critical".to_string()),
                see_also: Some("https://example.com/ssl-renewal".to_string()),
                cvss_base_score: Some("10.0".to_string()),
                cvss_vector: Some("CVSS2#AV:N/AC:L/Au:N/C:C/I:C/A:C".to_string()),
                cvss3_base_score: Some("9.8".to_string()),
                cvss3_vector: Some("CVSS:3.0/AV:N/AC:L/PR:N/UI:N/S:U/C:H/I:H/A:H".to_string()),
                cve: None,
                cwe: None,
                xref: None,
                plugin_output: Some("Certificate expired on 2024-01-01".to_string()),
                exploit_available: Some("false".to_string()),
                exploitability_ease: None,
                patch_publication_date: None,
                vuln_publication_date: None,
            },
            NessusReportItem {
                port: 22,
                svc_name: Some("ssh".to_string()),
                protocol: Some("tcp".to_string()),
                severity: 3,
                plugin_id: "67890".to_string(),
                plugin_name: "SSH Weak Algorithms".to_string(),
                plugin_family: Some("SSH".to_string()),
                synopsis: Some("The SSH server uses weak algorithms.".to_string()),
                description: Some("The SSH server supports weak encryption algorithms.".to_string()),
                solution: Some("Disable weak ciphers in sshd_config.".to_string()),
                risk_factor: Some("High".to_string()),
                see_also: None,
                cvss_base_score: Some("7.5".to_string()),
                cvss_vector: None,
                cvss3_base_score: Some("7.5".to_string()),
                cvss3_vector: Some("CVSS:3.0/AV:N/AC:L/PR:N/UI:N/S:U/C:H/I:N/A:N".to_string()),
                cve: Some("CVE-2024-1234".to_string()),
                cwe: Some("CWE-327".to_string()),
                xref: None,
                plugin_output: None,
                exploit_available: Some("true".to_string()),
                exploitability_ease: Some("Exploits are available".to_string()),
                patch_publication_date: Some("2024-06-01".to_string()),
                vuln_publication_date: Some("2024-01-15".to_string()),
            },
        ]
    }
}

impl Default for NessusImporter {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simplified() {
        let importer = NessusImporter::new();
        let items = NessusImporter::sample_report_items();

        let target = importer.parse_simplified(items, "192.168.1.100");

        assert_eq!(target.host, "192.168.1.100");
        assert_eq!(target.findings.len(), 2);

        let finding = &target.findings[0];
        assert_eq!(finding.severity, "critical");
        assert_eq!(finding.cvss_score, Some(9.8));
    }

    #[test]
    fn test_severity_mapping() {
        let importer = NessusImporter::new();

        let items = vec![
            NessusReportItem {
                port: 80,
                svc_name: None,
                protocol: None,
                severity: 4,
                plugin_id: "1".to_string(),
                plugin_name: "Critical Issue".to_string(),
                plugin_family: None,
                synopsis: None,
                description: None,
                solution: None,
                risk_factor: None,
                see_also: None,
                cvss_base_score: None,
                cvss_vector: None,
                cvss3_base_score: None,
                cvss3_vector: None,
                cve: None,
                cwe: None,
                xref: None,
                plugin_output: None,
                exploit_available: None,
                exploitability_ease: None,
                patch_publication_date: None,
                vuln_publication_date: None,
            },
        ];

        let target = importer.parse_simplified(items, "test");
        assert_eq!(target.findings[0].severity, "critical");
    }
}
