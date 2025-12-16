//! OpenVAS XML Scan Report Importer
//!
//! Parses OpenVAS XML reports and converts to WIA Security format.

use serde::{Deserialize, Serialize};
use super::{ImportResult, ImportError};
use super::nessus::{WiaScanResult, WiaScanTarget, WiaFinding, WiaScanSummary};

/// OpenVAS report structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenVasReport {
    #[serde(rename = "@id")]
    pub id: Option<String>,
    pub name: Option<String>,
    pub creation_time: Option<String>,
    pub modification_time: Option<String>,
    pub results: OpenVasResults,
}

/// OpenVAS results container
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenVasResults {
    #[serde(rename = "@start")]
    pub start: Option<u32>,
    #[serde(rename = "@max")]
    pub max: Option<u32>,
    #[serde(rename = "result", default)]
    pub results: Vec<OpenVasResult>,
}

/// OpenVAS result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenVasResult {
    #[serde(rename = "@id")]
    pub id: String,
    pub name: String,
    pub host: OpenVasHost,
    pub port: Option<String>,
    pub nvt: OpenVasNvt,
    pub threat: Option<String>,
    pub severity: Option<f64>,
    pub qod: Option<OpenVasQod>,
    pub description: Option<String>,
}

/// OpenVAS host
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenVasHost {
    #[serde(rename = "$value")]
    pub ip: String,
    #[serde(rename = "@hostname")]
    pub hostname: Option<String>,
}

/// OpenVAS NVT (Network Vulnerability Test)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenVasNvt {
    #[serde(rename = "@oid")]
    pub oid: String,
    pub name: Option<String>,
    pub family: Option<String>,
    pub cvss_base: Option<String>,
    pub tags: Option<String>,
    pub solution: Option<OpenVasSolution>,
    pub refs: Option<OpenVasRefs>,
}

/// OpenVAS solution
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenVasSolution {
    #[serde(rename = "@type")]
    pub solution_type: Option<String>,
    #[serde(rename = "$value")]
    pub value: Option<String>,
}

/// OpenVAS references
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenVasRefs {
    #[serde(rename = "ref", default)]
    pub refs: Vec<OpenVasRef>,
}

/// OpenVAS reference
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenVasRef {
    #[serde(rename = "@type")]
    pub ref_type: String,
    #[serde(rename = "@id")]
    pub id: String,
}

/// OpenVAS Quality of Detection
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenVasQod {
    pub value: Option<u32>,
    #[serde(rename = "type")]
    pub qod_type: Option<String>,
}

/// OpenVAS Importer
pub struct OpenVasImporter;

impl OpenVasImporter {
    /// Create new importer
    pub fn new() -> Self {
        Self
    }

    /// Convert threat level to severity string
    fn threat_to_severity(threat: &str) -> String {
        match threat.to_lowercase().as_str() {
            "high" => "critical".to_string(),
            "medium" => "high".to_string(),
            "low" => "medium".to_string(),
            "log" | "debug" => "info".to_string(),
            _ => "low".to_string(),
        }
    }

    /// Parse port string (e.g., "443/tcp")
    fn parse_port(port_str: &str) -> (Option<u16>, Option<String>) {
        let parts: Vec<&str> = port_str.split('/').collect();
        let port = parts.first().and_then(|p| p.parse::<u16>().ok());
        let protocol = parts.get(1).map(|p| p.to_string());
        (port, protocol)
    }

    /// Parse NVT tags for additional info
    fn parse_tags(tags: &str) -> std::collections::HashMap<String, String> {
        let mut map = std::collections::HashMap::new();
        for part in tags.split('|') {
            if let Some((key, value)) = part.split_once('=') {
                map.insert(key.to_string(), value.to_string());
            }
        }
        map
    }

    /// Extract CVE references
    fn extract_cves(refs: &Option<OpenVasRefs>) -> Vec<String> {
        refs.as_ref()
            .map(|r| {
                r.refs.iter()
                    .filter(|ref_item| ref_item.ref_type == "cve")
                    .map(|ref_item| ref_item.id.clone())
                    .collect()
            })
            .unwrap_or_default()
    }

    /// Extract other references
    fn extract_references(refs: &Option<OpenVasRefs>) -> Vec<String> {
        refs.as_ref()
            .map(|r| {
                r.refs.iter()
                    .filter(|ref_item| ref_item.ref_type == "url")
                    .map(|ref_item| ref_item.id.clone())
                    .collect()
            })
            .unwrap_or_default()
    }

    /// Convert OpenVAS result to WIA finding
    pub fn convert_result(&self, result: &OpenVasResult) -> WiaFinding {
        let severity = result.threat
            .as_ref()
            .map(|t| Self::threat_to_severity(t))
            .unwrap_or_else(|| "medium".to_string());

        let cvss_score = result.severity
            .or_else(|| result.nvt.cvss_base.as_ref().and_then(|s| s.parse::<f64>().ok()));

        let (port, protocol) = result.port
            .as_ref()
            .map(|p| Self::parse_port(p))
            .unwrap_or((None, None));

        let cve = Self::extract_cves(&result.nvt.refs);
        let references = Self::extract_references(&result.nvt.refs);

        let solution = result.nvt.solution
            .as_ref()
            .and_then(|s| s.value.clone());

        // Parse tags for additional info
        let tags = result.nvt.tags
            .as_ref()
            .map(|t| Self::parse_tags(t))
            .unwrap_or_default();

        let cvss_vector = tags.get("cvss_base_vector").cloned();

        WiaFinding {
            id: format!("OPENVAS-{}", result.nvt.oid),
            title: result.name.clone(),
            description: result.description.clone().unwrap_or_default(),
            severity,
            cvss_score,
            cvss_vector,
            port,
            protocol,
            service: None,
            cve,
            cwe: vec![],
            solution,
            references,
            exploit_available: false,
            patch_available: solution.is_some(),
            plugin_output: None,
        }
    }

    /// Convert OpenVAS report to WIA format
    pub fn convert_to_wia(&self, report: &OpenVasReport) -> WiaScanResult {
        let scan_id = report.id.clone().unwrap_or_else(|| format!("openvas-{}", uuid::Uuid::new_v4()));
        let scan_name = report.name.clone().unwrap_or_else(|| "OpenVAS Scan".to_string());
        let scan_time = report.creation_time.clone().unwrap_or_else(|| {
            chrono::Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string()
        });

        // Group results by host
        let mut host_map: std::collections::HashMap<String, Vec<WiaFinding>> = std::collections::HashMap::new();

        for result in &report.results.results {
            let finding = self.convert_result(result);
            host_map
                .entry(result.host.ip.clone())
                .or_insert_with(Vec::new)
                .push(finding);
        }

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

        for (ip, findings) in host_map {
            summary.total_hosts += 1;
            for finding in &findings {
                summary.total_findings += 1;
                match finding.severity.as_str() {
                    "critical" => summary.critical += 1,
                    "high" => summary.high += 1,
                    "medium" => summary.medium += 1,
                    "low" => summary.low += 1,
                    _ => summary.info += 1,
                }
            }

            targets.push(WiaScanTarget {
                host: ip.clone(),
                ip: Some(ip),
                os: None,
                findings,
            });
        }

        WiaScanResult {
            scan_id,
            scan_name,
            scan_time,
            targets,
            summary,
        }
    }

    /// Create sample OpenVAS data for testing
    pub fn sample_results() -> Vec<OpenVasResult> {
        vec![
            OpenVasResult {
                id: "result-001".to_string(),
                name: "SSH Weak Key Exchange".to_string(),
                host: OpenVasHost {
                    ip: "192.168.1.10".to_string(),
                    hostname: Some("server1.example.com".to_string()),
                },
                port: Some("22/tcp".to_string()),
                nvt: OpenVasNvt {
                    oid: "1.3.6.1.4.1.25623.1.0.100001".to_string(),
                    name: Some("SSH Weak Key Exchange Algorithm".to_string()),
                    family: Some("SSH".to_string()),
                    cvss_base: Some("7.5".to_string()),
                    tags: Some("cvss_base_vector=AV:N/AC:L/Au:N/C:P/I:P/A:P|solution_type=Mitigation".to_string()),
                    solution: Some(OpenVasSolution {
                        solution_type: Some("Mitigation".to_string()),
                        value: Some("Disable weak key exchange algorithms in sshd_config".to_string()),
                    }),
                    refs: Some(OpenVasRefs {
                        refs: vec![
                            OpenVasRef {
                                ref_type: "cve".to_string(),
                                id: "CVE-2023-12345".to_string(),
                            },
                            OpenVasRef {
                                ref_type: "url".to_string(),
                                id: "https://www.ssh.com/security".to_string(),
                            },
                        ],
                    }),
                },
                threat: Some("High".to_string()),
                severity: Some(7.5),
                qod: Some(OpenVasQod {
                    value: Some(80),
                    qod_type: Some("remote_vul".to_string()),
                }),
                description: Some("The SSH server supports weak key exchange algorithms.".to_string()),
            },
        ]
    }
}

impl Default for OpenVasImporter {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_threat_to_severity() {
        assert_eq!(OpenVasImporter::threat_to_severity("High"), "critical");
        assert_eq!(OpenVasImporter::threat_to_severity("Medium"), "high");
        assert_eq!(OpenVasImporter::threat_to_severity("Low"), "medium");
        assert_eq!(OpenVasImporter::threat_to_severity("Log"), "info");
    }

    #[test]
    fn test_parse_port() {
        let (port, protocol) = OpenVasImporter::parse_port("443/tcp");
        assert_eq!(port, Some(443));
        assert_eq!(protocol, Some("tcp".to_string()));

        let (port2, protocol2) = OpenVasImporter::parse_port("general/tcp");
        assert_eq!(port2, None);
        assert_eq!(protocol2, Some("tcp".to_string()));
    }

    #[test]
    fn test_convert_result() {
        let importer = OpenVasImporter::new();
        let results = OpenVasImporter::sample_results();

        let finding = importer.convert_result(&results[0]);

        assert!(finding.id.starts_with("OPENVAS-"));
        assert_eq!(finding.severity, "critical");
        assert_eq!(finding.cvss_score, Some(7.5));
        assert!(finding.cve.contains(&"CVE-2023-12345".to_string()));
    }
}
