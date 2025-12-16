//! PDF Report Generator
//!
//! Generates PDF security reports from WIA Security data.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use super::{ExportResult, ExportError};

/// PDF report configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PdfConfig {
    /// Report title
    pub title: String,
    /// Organization name
    pub organization: String,
    /// Report author
    pub author: Option<String>,
    /// Include executive summary
    pub include_executive_summary: bool,
    /// Include detailed findings
    pub include_detailed_findings: bool,
    /// Include charts/graphs
    pub include_charts: bool,
    /// Logo path (optional)
    pub logo_path: Option<String>,
    /// Page size
    pub page_size: PageSize,
    /// Color scheme
    pub color_scheme: ColorScheme,
}

/// Page size options
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PageSize {
    A4,
    Letter,
    Legal,
}

/// Color scheme
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ColorScheme {
    pub primary: String,
    pub secondary: String,
    pub critical: String,
    pub high: String,
    pub medium: String,
    pub low: String,
    pub info: String,
}

impl Default for ColorScheme {
    fn default() -> Self {
        Self {
            primary: "#1a365d".to_string(),
            secondary: "#2d3748".to_string(),
            critical: "#9b2c2c".to_string(),
            high: "#c53030".to_string(),
            medium: "#dd6b20".to_string(),
            low: "#d69e2e".to_string(),
            info: "#3182ce".to_string(),
        }
    }
}

impl Default for PdfConfig {
    fn default() -> Self {
        Self {
            title: "Security Assessment Report".to_string(),
            organization: "Organization Name".to_string(),
            author: None,
            include_executive_summary: true,
            include_detailed_findings: true,
            include_charts: true,
            logo_path: None,
            page_size: PageSize::A4,
            color_scheme: ColorScheme::default(),
        }
    }
}

/// Report section
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReportSection {
    pub title: String,
    pub content: SectionContent,
}

/// Section content types
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum SectionContent {
    Text { paragraphs: Vec<String> },
    Table { headers: Vec<String>, rows: Vec<Vec<String>> },
    Chart { chart_type: ChartType, data: ChartData },
    FindingsList { findings: Vec<ReportFinding> },
    Summary { stats: ReportStats },
}

/// Chart types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ChartType {
    Pie,
    Bar,
    Line,
    Donut,
}

/// Chart data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChartData {
    pub labels: Vec<String>,
    pub values: Vec<f64>,
    pub colors: Option<Vec<String>>,
}

/// Report finding
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReportFinding {
    pub id: String,
    pub title: String,
    pub severity: String,
    pub cvss_score: Option<f64>,
    pub description: String,
    pub affected_hosts: Vec<String>,
    pub cve: Vec<String>,
    pub solution: Option<String>,
    pub references: Vec<String>,
}

/// Report statistics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReportStats {
    pub total_hosts: u32,
    pub total_findings: u32,
    pub critical_count: u32,
    pub high_count: u32,
    pub medium_count: u32,
    pub low_count: u32,
    pub info_count: u32,
    pub risk_score: f64,
}

/// PDF Report structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PdfReport {
    pub metadata: ReportMetadata,
    pub sections: Vec<ReportSection>,
}

/// Report metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReportMetadata {
    pub title: String,
    pub subtitle: Option<String>,
    pub organization: String,
    pub author: Option<String>,
    pub date: String,
    pub version: String,
    pub classification: Option<String>,
}

/// PDF Report Generator
pub struct PdfGenerator {
    config: PdfConfig,
}

impl PdfGenerator {
    /// Create new PDF generator
    pub fn new(config: PdfConfig) -> Self {
        Self { config }
    }

    /// Convert scan results to report
    pub fn from_scan_results(&self, scan: &super::super::importers::WiaScanResult) -> PdfReport {
        let mut sections = Vec::new();

        // Executive Summary
        if self.config.include_executive_summary {
            sections.push(self.generate_executive_summary(&scan.summary));
        }

        // Severity Distribution Chart
        if self.config.include_charts {
            sections.push(self.generate_severity_chart(&scan.summary));
        }

        // Findings by Host
        sections.push(self.generate_hosts_summary(scan));

        // Detailed Findings
        if self.config.include_detailed_findings {
            sections.push(self.generate_detailed_findings(scan));
        }

        // Recommendations
        sections.push(self.generate_recommendations(scan));

        PdfReport {
            metadata: ReportMetadata {
                title: self.config.title.clone(),
                subtitle: Some(scan.scan_name.clone()),
                organization: self.config.organization.clone(),
                author: self.config.author.clone(),
                date: chrono::Utc::now().format("%Y-%m-%d").to_string(),
                version: "1.0".to_string(),
                classification: Some("CONFIDENTIAL".to_string()),
            },
            sections,
        }
    }

    /// Generate executive summary section
    fn generate_executive_summary(&self, summary: &super::super::importers::WiaScanSummary) -> ReportSection {
        let risk_level = if summary.critical > 0 {
            "CRITICAL"
        } else if summary.high > 0 {
            "HIGH"
        } else if summary.medium > 0 {
            "MEDIUM"
        } else {
            "LOW"
        };

        let risk_score = self.calculate_risk_score(summary);

        ReportSection {
            title: "Executive Summary".to_string(),
            content: SectionContent::Summary {
                stats: ReportStats {
                    total_hosts: summary.total_hosts,
                    total_findings: summary.total_findings,
                    critical_count: summary.critical,
                    high_count: summary.high,
                    medium_count: summary.medium,
                    low_count: summary.low,
                    info_count: summary.info,
                    risk_score,
                },
            },
        }
    }

    /// Generate severity distribution chart
    fn generate_severity_chart(&self, summary: &super::super::importers::WiaScanSummary) -> ReportSection {
        ReportSection {
            title: "Severity Distribution".to_string(),
            content: SectionContent::Chart {
                chart_type: ChartType::Donut,
                data: ChartData {
                    labels: vec![
                        "Critical".to_string(),
                        "High".to_string(),
                        "Medium".to_string(),
                        "Low".to_string(),
                        "Info".to_string(),
                    ],
                    values: vec![
                        summary.critical as f64,
                        summary.high as f64,
                        summary.medium as f64,
                        summary.low as f64,
                        summary.info as f64,
                    ],
                    colors: Some(vec![
                        self.config.color_scheme.critical.clone(),
                        self.config.color_scheme.high.clone(),
                        self.config.color_scheme.medium.clone(),
                        self.config.color_scheme.low.clone(),
                        self.config.color_scheme.info.clone(),
                    ]),
                },
            },
        }
    }

    /// Generate hosts summary table
    fn generate_hosts_summary(&self, scan: &super::super::importers::WiaScanResult) -> ReportSection {
        let mut rows = Vec::new();

        for target in &scan.targets {
            let mut critical = 0;
            let mut high = 0;
            let mut medium = 0;
            let mut low = 0;

            for finding in &target.findings {
                match finding.severity.as_str() {
                    "critical" => critical += 1,
                    "high" => high += 1,
                    "medium" => medium += 1,
                    "low" => low += 1,
                    _ => {}
                }
            }

            rows.push(vec![
                target.host.clone(),
                target.ip.clone().unwrap_or_default(),
                critical.to_string(),
                high.to_string(),
                medium.to_string(),
                low.to_string(),
                target.findings.len().to_string(),
            ]);
        }

        ReportSection {
            title: "Hosts Summary".to_string(),
            content: SectionContent::Table {
                headers: vec![
                    "Host".to_string(),
                    "IP Address".to_string(),
                    "Critical".to_string(),
                    "High".to_string(),
                    "Medium".to_string(),
                    "Low".to_string(),
                    "Total".to_string(),
                ],
                rows,
            },
        }
    }

    /// Generate detailed findings section
    fn generate_detailed_findings(&self, scan: &super::super::importers::WiaScanResult) -> ReportSection {
        let mut findings = Vec::new();

        for target in &scan.targets {
            for finding in &target.findings {
                findings.push(ReportFinding {
                    id: finding.id.clone(),
                    title: finding.title.clone(),
                    severity: finding.severity.clone(),
                    cvss_score: finding.cvss_score,
                    description: finding.description.clone(),
                    affected_hosts: vec![target.host.clone()],
                    cve: finding.cve.clone(),
                    solution: finding.solution.clone(),
                    references: finding.references.clone(),
                });
            }
        }

        // Sort by severity (critical first)
        findings.sort_by(|a, b| {
            let severity_order = |s: &str| match s {
                "critical" => 0,
                "high" => 1,
                "medium" => 2,
                "low" => 3,
                _ => 4,
            };
            severity_order(&a.severity).cmp(&severity_order(&b.severity))
        });

        ReportSection {
            title: "Detailed Findings".to_string(),
            content: SectionContent::FindingsList { findings },
        }
    }

    /// Generate recommendations section
    fn generate_recommendations(&self, scan: &super::super::importers::WiaScanResult) -> ReportSection {
        let mut paragraphs = Vec::new();

        // Collect unique solutions
        let mut solutions: HashMap<String, Vec<String>> = HashMap::new();
        for target in &scan.targets {
            for finding in &target.findings {
                if let Some(ref solution) = finding.solution {
                    solutions
                        .entry(solution.clone())
                        .or_insert_with(Vec::new)
                        .push(finding.id.clone());
                }
            }
        }

        paragraphs.push("Based on the assessment findings, the following remediation actions are recommended:".to_string());

        let mut priority = 1;
        for (solution, finding_ids) in solutions.iter().take(10) {
            paragraphs.push(format!(
                "{}. {} (Addresses: {})",
                priority,
                solution,
                finding_ids.join(", ")
            ));
            priority += 1;
        }

        if solutions.is_empty() {
            paragraphs.push("No specific remediation actions required based on current findings.".to_string());
        }

        ReportSection {
            title: "Recommendations".to_string(),
            content: SectionContent::Text { paragraphs },
        }
    }

    /// Calculate overall risk score (0-100)
    fn calculate_risk_score(&self, summary: &super::super::importers::WiaScanSummary) -> f64 {
        let weighted_score =
            (summary.critical as f64 * 10.0) +
            (summary.high as f64 * 7.0) +
            (summary.medium as f64 * 4.0) +
            (summary.low as f64 * 1.0);

        // Normalize to 0-100 scale
        let max_possible = (summary.total_findings as f64) * 10.0;
        if max_possible > 0.0 {
            (weighted_score / max_possible * 100.0).min(100.0)
        } else {
            0.0
        }
    }

    /// Export report to JSON (for frontend rendering)
    pub fn to_json(&self, report: &PdfReport) -> ExportResult<String> {
        serde_json::to_string_pretty(report)
            .map_err(|e| ExportError::SerializationError(e.to_string()))
    }

    /// Generate HTML template for PDF conversion
    pub fn to_html(&self, report: &PdfReport) -> String {
        let mut html = String::new();

        html.push_str("<!DOCTYPE html>\n<html>\n<head>\n");
        html.push_str(&format!("<title>{}</title>\n", report.metadata.title));
        html.push_str("<style>\n");
        html.push_str(&self.generate_css());
        html.push_str("</style>\n</head>\n<body>\n");

        // Header
        html.push_str("<header>\n");
        html.push_str(&format!("<h1>{}</h1>\n", report.metadata.title));
        if let Some(ref subtitle) = report.metadata.subtitle {
            html.push_str(&format!("<h2>{}</h2>\n", subtitle));
        }
        html.push_str(&format!("<p class=\"meta\">Organization: {} | Date: {} | Version: {}</p>\n",
            report.metadata.organization,
            report.metadata.date,
            report.metadata.version
        ));
        html.push_str("</header>\n");

        // Sections
        for section in &report.sections {
            html.push_str(&self.section_to_html(section));
        }

        html.push_str("</body>\n</html>");
        html
    }

    /// Generate CSS styles
    fn generate_css(&self) -> String {
        format!(r#"
            body {{ font-family: Arial, sans-serif; margin: 40px; color: {}; }}
            header {{ border-bottom: 2px solid {}; padding-bottom: 20px; margin-bottom: 30px; }}
            h1 {{ color: {}; }}
            h2 {{ color: {}; }}
            .meta {{ color: #666; font-size: 12px; }}
            table {{ width: 100%; border-collapse: collapse; margin: 20px 0; }}
            th, td {{ border: 1px solid #ddd; padding: 8px; text-align: left; }}
            th {{ background-color: {}; color: white; }}
            .severity-critical {{ background-color: {}; color: white; padding: 2px 8px; border-radius: 4px; }}
            .severity-high {{ background-color: {}; color: white; padding: 2px 8px; border-radius: 4px; }}
            .severity-medium {{ background-color: {}; color: white; padding: 2px 8px; border-radius: 4px; }}
            .severity-low {{ background-color: {}; color: white; padding: 2px 8px; border-radius: 4px; }}
            .severity-info {{ background-color: {}; color: white; padding: 2px 8px; border-radius: 4px; }}
            .finding {{ border: 1px solid #ddd; padding: 15px; margin: 10px 0; border-radius: 4px; }}
            .stats {{ display: flex; gap: 20px; flex-wrap: wrap; }}
            .stat-box {{ background: #f5f5f5; padding: 15px; border-radius: 4px; min-width: 120px; text-align: center; }}
            .stat-value {{ font-size: 24px; font-weight: bold; }}
            .stat-label {{ font-size: 12px; color: #666; }}
        "#,
            self.config.color_scheme.secondary,
            self.config.color_scheme.primary,
            self.config.color_scheme.primary,
            self.config.color_scheme.secondary,
            self.config.color_scheme.primary,
            self.config.color_scheme.critical,
            self.config.color_scheme.high,
            self.config.color_scheme.medium,
            self.config.color_scheme.low,
            self.config.color_scheme.info
        )
    }

    /// Convert section to HTML
    fn section_to_html(&self, section: &ReportSection) -> String {
        let mut html = format!("<section>\n<h2>{}</h2>\n", section.title);

        match &section.content {
            SectionContent::Text { paragraphs } => {
                for p in paragraphs {
                    html.push_str(&format!("<p>{}</p>\n", p));
                }
            }
            SectionContent::Table { headers, rows } => {
                html.push_str("<table>\n<thead><tr>\n");
                for h in headers {
                    html.push_str(&format!("<th>{}</th>\n", h));
                }
                html.push_str("</tr></thead>\n<tbody>\n");
                for row in rows {
                    html.push_str("<tr>\n");
                    for cell in row {
                        html.push_str(&format!("<td>{}</td>\n", cell));
                    }
                    html.push_str("</tr>\n");
                }
                html.push_str("</tbody></table>\n");
            }
            SectionContent::Summary { stats } => {
                html.push_str("<div class=\"stats\">\n");
                html.push_str(&format!(
                    "<div class=\"stat-box\"><div class=\"stat-value\">{}</div><div class=\"stat-label\">Hosts Scanned</div></div>\n",
                    stats.total_hosts
                ));
                html.push_str(&format!(
                    "<div class=\"stat-box\"><div class=\"stat-value\">{}</div><div class=\"stat-label\">Total Findings</div></div>\n",
                    stats.total_findings
                ));
                html.push_str(&format!(
                    "<div class=\"stat-box\"><div class=\"stat-value\">{:.1}</div><div class=\"stat-label\">Risk Score</div></div>\n",
                    stats.risk_score
                ));
                html.push_str("</div>\n");
            }
            SectionContent::FindingsList { findings } => {
                for f in findings {
                    html.push_str(&format!(
                        "<div class=\"finding\">\n<h3>{} <span class=\"severity-{}\">{}</span></h3>\n",
                        f.title,
                        f.severity.to_lowercase(),
                        f.severity.to_uppercase()
                    ));
                    html.push_str(&format!("<p><strong>ID:</strong> {}</p>\n", f.id));
                    if let Some(score) = f.cvss_score {
                        html.push_str(&format!("<p><strong>CVSS Score:</strong> {:.1}</p>\n", score));
                    }
                    html.push_str(&format!("<p>{}</p>\n", f.description));
                    if let Some(ref sol) = f.solution {
                        html.push_str(&format!("<p><strong>Solution:</strong> {}</p>\n", sol));
                    }
                    html.push_str("</div>\n");
                }
            }
            SectionContent::Chart { .. } => {
                html.push_str("<p>[Chart placeholder - render with JavaScript]</p>\n");
            }
        }

        html.push_str("</section>\n");
        html
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::integration::importers::{WiaScanResult, WiaScanTarget, WiaFinding, WiaScanSummary};

    fn sample_scan_result() -> WiaScanResult {
        WiaScanResult {
            scan_id: "test-001".to_string(),
            scan_name: "Test Security Scan".to_string(),
            scan_time: "2024-01-15T10:00:00Z".to_string(),
            targets: vec![
                WiaScanTarget {
                    host: "server1.example.com".to_string(),
                    ip: Some("192.168.1.10".to_string()),
                    os: Some("Linux".to_string()),
                    findings: vec![
                        WiaFinding {
                            id: "VULN-001".to_string(),
                            title: "Critical SSL Vulnerability".to_string(),
                            description: "SSL certificate has expired".to_string(),
                            severity: "critical".to_string(),
                            cvss_score: Some(9.8),
                            cvss_vector: None,
                            port: Some(443),
                            protocol: Some("tcp".to_string()),
                            service: Some("https".to_string()),
                            cve: vec!["CVE-2024-1234".to_string()],
                            cwe: vec![],
                            solution: Some("Renew SSL certificate".to_string()),
                            references: vec![],
                            exploit_available: false,
                            patch_available: true,
                            plugin_output: None,
                        },
                    ],
                },
            ],
            summary: WiaScanSummary {
                total_hosts: 1,
                total_findings: 1,
                critical: 1,
                high: 0,
                medium: 0,
                low: 0,
                info: 0,
            },
        }
    }

    #[test]
    fn test_generate_report() {
        let config = PdfConfig::default();
        let generator = PdfGenerator::new(config);
        let scan = sample_scan_result();

        let report = generator.from_scan_results(&scan);

        assert_eq!(report.metadata.title, "Security Assessment Report");
        assert!(!report.sections.is_empty());
    }

    #[test]
    fn test_to_html() {
        let config = PdfConfig::default();
        let generator = PdfGenerator::new(config);
        let scan = sample_scan_result();

        let report = generator.from_scan_results(&scan);
        let html = generator.to_html(&report);

        assert!(html.contains("<!DOCTYPE html>"));
        assert!(html.contains("Security Assessment Report"));
    }

    #[test]
    fn test_risk_score() {
        let config = PdfConfig::default();
        let generator = PdfGenerator::new(config);

        let summary = WiaScanSummary {
            total_hosts: 1,
            total_findings: 10,
            critical: 2,
            high: 3,
            medium: 3,
            low: 2,
            info: 0,
        };

        let score = generator.calculate_risk_score(&summary);
        assert!(score > 0.0 && score <= 100.0);
    }
}
