//! NVD (National Vulnerability Database) Importer
//!
//! Fetches and converts CVE data from NIST NVD API 2.0.

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};
use super::{ImportResult, ImportError};

/// NVD API configuration
#[derive(Debug, Clone)]
pub struct NvdConfig {
    /// API base URL
    pub base_url: String,
    /// API key (optional but recommended)
    pub api_key: Option<String>,
    /// Request timeout in seconds
    pub timeout_seconds: u64,
    /// Rate limit: requests per 30 seconds (6 with key, 5 without)
    pub rate_limit: u32,
}

impl Default for NvdConfig {
    fn default() -> Self {
        Self {
            base_url: "https://services.nvd.nist.gov/rest/json/cves/2.0".to_string(),
            api_key: None,
            timeout_seconds: 30,
            rate_limit: 5,
        }
    }
}

impl NvdConfig {
    /// Create config with API key
    pub fn with_api_key(api_key: impl Into<String>) -> Self {
        Self {
            api_key: Some(api_key.into()),
            rate_limit: 50, // Higher limit with API key
            ..Default::default()
        }
    }
}

/// NVD CVE record
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NvdCve {
    /// CVE ID (e.g., CVE-2024-12345)
    pub id: String,
    /// Source identifier
    #[serde(rename = "sourceIdentifier")]
    pub source_identifier: Option<String>,
    /// Published date
    pub published: Option<String>,
    /// Last modified date
    #[serde(rename = "lastModified")]
    pub last_modified: Option<String>,
    /// Vulnerability status
    #[serde(rename = "vulnStatus")]
    pub vuln_status: Option<String>,
    /// Descriptions
    pub descriptions: Vec<NvdDescription>,
    /// CVSS metrics
    pub metrics: Option<NvdMetrics>,
    /// Weaknesses (CWE)
    pub weaknesses: Option<Vec<NvdWeakness>>,
    /// Configurations (affected products)
    pub configurations: Option<Vec<NvdConfiguration>>,
    /// References
    pub references: Option<Vec<NvdReference>>,
}

/// NVD description
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NvdDescription {
    /// Language (en, es, etc.)
    pub lang: String,
    /// Description value
    pub value: String,
}

/// NVD metrics (CVSS scores)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NvdMetrics {
    /// CVSS v3.1 metrics
    #[serde(rename = "cvssMetricV31")]
    pub cvss_v31: Option<Vec<CvssV31Metric>>,
    /// CVSS v3.0 metrics
    #[serde(rename = "cvssMetricV30")]
    pub cvss_v30: Option<Vec<CvssV30Metric>>,
    /// CVSS v2 metrics
    #[serde(rename = "cvssMetricV2")]
    pub cvss_v2: Option<Vec<CvssV2Metric>>,
}

/// CVSS v3.1 metric
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CvssV31Metric {
    /// Source
    pub source: String,
    /// Primary or secondary
    #[serde(rename = "type")]
    pub metric_type: String,
    /// CVSS data
    #[serde(rename = "cvssData")]
    pub cvss_data: CvssV31Data,
    /// Exploitability score
    #[serde(rename = "exploitabilityScore")]
    pub exploitability_score: Option<f64>,
    /// Impact score
    #[serde(rename = "impactScore")]
    pub impact_score: Option<f64>,
}

/// CVSS v3.1 data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CvssV31Data {
    /// Version
    pub version: String,
    /// Vector string
    #[serde(rename = "vectorString")]
    pub vector_string: String,
    /// Attack vector
    #[serde(rename = "attackVector")]
    pub attack_vector: Option<String>,
    /// Attack complexity
    #[serde(rename = "attackComplexity")]
    pub attack_complexity: Option<String>,
    /// Privileges required
    #[serde(rename = "privilegesRequired")]
    pub privileges_required: Option<String>,
    /// User interaction
    #[serde(rename = "userInteraction")]
    pub user_interaction: Option<String>,
    /// Scope
    pub scope: Option<String>,
    /// Confidentiality impact
    #[serde(rename = "confidentialityImpact")]
    pub confidentiality_impact: Option<String>,
    /// Integrity impact
    #[serde(rename = "integrityImpact")]
    pub integrity_impact: Option<String>,
    /// Availability impact
    #[serde(rename = "availabilityImpact")]
    pub availability_impact: Option<String>,
    /// Base score
    #[serde(rename = "baseScore")]
    pub base_score: f64,
    /// Base severity
    #[serde(rename = "baseSeverity")]
    pub base_severity: String,
}

/// CVSS v3.0 metric (similar structure to v3.1)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CvssV30Metric {
    pub source: String,
    #[serde(rename = "type")]
    pub metric_type: String,
    #[serde(rename = "cvssData")]
    pub cvss_data: CvssV31Data, // Similar structure
}

/// CVSS v2 metric
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CvssV2Metric {
    pub source: String,
    #[serde(rename = "type")]
    pub metric_type: String,
    #[serde(rename = "cvssData")]
    pub cvss_data: CvssV2Data,
}

/// CVSS v2 data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CvssV2Data {
    pub version: String,
    #[serde(rename = "vectorString")]
    pub vector_string: String,
    #[serde(rename = "baseScore")]
    pub base_score: f64,
}

/// NVD weakness (CWE)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NvdWeakness {
    pub source: String,
    #[serde(rename = "type")]
    pub weakness_type: String,
    pub description: Vec<NvdDescription>,
}

/// NVD configuration (affected products)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NvdConfiguration {
    pub nodes: Vec<NvdNode>,
}

/// NVD node
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NvdNode {
    pub operator: Option<String>,
    pub negate: Option<bool>,
    #[serde(rename = "cpeMatch")]
    pub cpe_match: Option<Vec<CpeMatch>>,
}

/// CPE match
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CpeMatch {
    pub vulnerable: bool,
    pub criteria: String,
    #[serde(rename = "versionStartIncluding")]
    pub version_start_including: Option<String>,
    #[serde(rename = "versionEndExcluding")]
    pub version_end_excluding: Option<String>,
    #[serde(rename = "matchCriteriaId")]
    pub match_criteria_id: String,
}

/// NVD reference
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NvdReference {
    pub url: String,
    pub source: Option<String>,
    pub tags: Option<Vec<String>>,
}

/// NVD API response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NvdResponse {
    #[serde(rename = "resultsPerPage")]
    pub results_per_page: u32,
    #[serde(rename = "startIndex")]
    pub start_index: u32,
    #[serde(rename = "totalResults")]
    pub total_results: u32,
    pub format: String,
    pub version: String,
    pub timestamp: String,
    pub vulnerabilities: Vec<NvdVulnerability>,
}

/// NVD vulnerability wrapper
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NvdVulnerability {
    pub cve: NvdCve,
}

/// WIA Security vulnerability format
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaVulnerability {
    pub vuln_id: String,
    pub title: String,
    pub description: String,
    pub cvss: WiaCvss,
    pub cwe: Vec<String>,
    pub affected_products: Vec<WiaAffectedProduct>,
    pub exploit_available: bool,
    pub patch_available: bool,
    pub references: Vec<String>,
    pub published: String,
    pub modified: String,
}

/// WIA CVSS format
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaCvss {
    pub version: String,
    pub score: f64,
    pub vector: String,
    pub severity: String,
}

/// WIA affected product
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaAffectedProduct {
    pub vendor: String,
    pub product: String,
    pub versions: Vec<String>,
    pub cpe: String,
}

/// NVD Importer
pub struct NvdImporter {
    config: NvdConfig,
}

impl NvdImporter {
    /// Create new NVD importer
    pub fn new(config: NvdConfig) -> Self {
        Self { config }
    }

    /// Create with default config
    pub fn default_config() -> Self {
        Self::new(NvdConfig::default())
    }

    /// Build request URL for CVE lookup
    pub fn build_cve_url(&self, cve_id: &str) -> String {
        format!("{}?cveId={}", self.config.base_url, cve_id)
    }

    /// Build request URL for search
    pub fn build_search_url(&self, params: &NvdSearchParams) -> String {
        let mut url = self.config.base_url.clone();
        let mut query_parts = Vec::new();

        if let Some(keyword) = &params.keyword {
            query_parts.push(format!("keywordSearch={}", keyword));
        }
        if let Some(cvss_min) = params.cvss_v3_min {
            query_parts.push(format!("cvssV3Severity={}", cvss_severity(cvss_min)));
        }
        if let Some(pub_start) = &params.pub_start_date {
            query_parts.push(format!("pubStartDate={}", pub_start));
        }
        if let Some(pub_end) = &params.pub_end_date {
            query_parts.push(format!("pubEndDate={}", pub_end));
        }
        if let Some(start_index) = params.start_index {
            query_parts.push(format!("startIndex={}", start_index));
        }
        if let Some(results_per_page) = params.results_per_page {
            query_parts.push(format!("resultsPerPage={}", results_per_page));
        }

        if !query_parts.is_empty() {
            url = format!("{}?{}", url, query_parts.join("&"));
        }

        url
    }

    /// Convert NVD CVE to WIA format
    pub fn convert_to_wia(&self, cve: &NvdCve) -> WiaVulnerability {
        // Extract description (prefer English)
        let description = cve.descriptions
            .iter()
            .find(|d| d.lang == "en")
            .map(|d| d.value.clone())
            .unwrap_or_else(|| cve.descriptions.first().map(|d| d.value.clone()).unwrap_or_default());

        // Extract CVSS info
        let cvss = self.extract_cvss(&cve.metrics);

        // Extract CWE IDs
        let cwe = self.extract_cwe(&cve.weaknesses);

        // Extract affected products from configurations
        let affected_products = self.extract_affected_products(&cve.configurations);

        // Check for exploit/patch in references
        let (exploit_available, patch_available) = self.check_references(&cve.references);

        // Extract reference URLs
        let references = cve.references
            .as_ref()
            .map(|refs| refs.iter().map(|r| r.url.clone()).collect())
            .unwrap_or_default();

        WiaVulnerability {
            vuln_id: cve.id.clone(),
            title: format!("{}: {}", cve.id, truncate(&description, 100)),
            description,
            cvss,
            cwe,
            affected_products,
            exploit_available,
            patch_available,
            references,
            published: cve.published.clone().unwrap_or_default(),
            modified: cve.last_modified.clone().unwrap_or_default(),
        }
    }

    /// Extract CVSS information
    fn extract_cvss(&self, metrics: &Option<NvdMetrics>) -> WiaCvss {
        if let Some(m) = metrics {
            // Prefer CVSS v3.1
            if let Some(v31) = &m.cvss_v31 {
                if let Some(metric) = v31.first() {
                    return WiaCvss {
                        version: "3.1".to_string(),
                        score: metric.cvss_data.base_score,
                        vector: metric.cvss_data.vector_string.clone(),
                        severity: metric.cvss_data.base_severity.clone(),
                    };
                }
            }
            // Fall back to CVSS v3.0
            if let Some(v30) = &m.cvss_v30 {
                if let Some(metric) = v30.first() {
                    return WiaCvss {
                        version: "3.0".to_string(),
                        score: metric.cvss_data.base_score,
                        vector: metric.cvss_data.vector_string.clone(),
                        severity: metric.cvss_data.base_severity.clone(),
                    };
                }
            }
            // Fall back to CVSS v2
            if let Some(v2) = &m.cvss_v2 {
                if let Some(metric) = v2.first() {
                    return WiaCvss {
                        version: "2.0".to_string(),
                        score: metric.cvss_data.base_score,
                        vector: metric.cvss_data.vector_string.clone(),
                        severity: cvss_v2_severity(metric.cvss_data.base_score),
                    };
                }
            }
        }

        WiaCvss {
            version: "unknown".to_string(),
            score: 0.0,
            vector: String::new(),
            severity: "UNKNOWN".to_string(),
        }
    }

    /// Extract CWE IDs
    fn extract_cwe(&self, weaknesses: &Option<Vec<NvdWeakness>>) -> Vec<String> {
        weaknesses
            .as_ref()
            .map(|ws| {
                ws.iter()
                    .flat_map(|w| w.description.iter())
                    .filter(|d| d.lang == "en" && d.value.starts_with("CWE-"))
                    .map(|d| d.value.clone())
                    .collect()
            })
            .unwrap_or_default()
    }

    /// Extract affected products from CPE
    fn extract_affected_products(&self, configs: &Option<Vec<NvdConfiguration>>) -> Vec<WiaAffectedProduct> {
        let mut products = Vec::new();

        if let Some(confs) = configs {
            for conf in confs {
                for node in &conf.nodes {
                    if let Some(matches) = &node.cpe_match {
                        for m in matches {
                            if m.vulnerable {
                                if let Some(product) = parse_cpe(&m.criteria) {
                                    products.push(product);
                                }
                            }
                        }
                    }
                }
            }
        }

        products
    }

    /// Check references for exploit/patch tags
    fn check_references(&self, refs: &Option<Vec<NvdReference>>) -> (bool, bool) {
        let mut exploit = false;
        let mut patch = false;

        if let Some(references) = refs {
            for r in references {
                if let Some(tags) = &r.tags {
                    for tag in tags {
                        match tag.to_lowercase().as_str() {
                            "exploit" => exploit = true,
                            "patch" | "vendor advisory" => patch = true,
                            _ => {}
                        }
                    }
                }
            }
        }

        (exploit, patch)
    }
}

/// NVD search parameters
#[derive(Debug, Clone, Default)]
pub struct NvdSearchParams {
    pub keyword: Option<String>,
    pub cvss_v3_min: Option<f64>,
    pub pub_start_date: Option<String>,
    pub pub_end_date: Option<String>,
    pub start_index: Option<u32>,
    pub results_per_page: Option<u32>,
}

impl NvdSearchParams {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn keyword(mut self, keyword: impl Into<String>) -> Self {
        self.keyword = Some(keyword.into());
        self
    }

    pub fn cvss_min(mut self, score: f64) -> Self {
        self.cvss_v3_min = Some(score);
        self
    }

    pub fn published_after(mut self, date: impl Into<String>) -> Self {
        self.pub_start_date = Some(date.into());
        self
    }

    pub fn page(mut self, start: u32, size: u32) -> Self {
        self.start_index = Some(start);
        self.results_per_page = Some(size);
        self
    }
}

// Helper functions

fn truncate(s: &str, max_len: usize) -> String {
    if s.len() <= max_len {
        s.to_string()
    } else {
        format!("{}...", &s[..max_len])
    }
}

fn cvss_severity(score: f64) -> &'static str {
    match score {
        s if s >= 9.0 => "CRITICAL",
        s if s >= 7.0 => "HIGH",
        s if s >= 4.0 => "MEDIUM",
        s if s >= 0.1 => "LOW",
        _ => "NONE",
    }
}

fn cvss_v2_severity(score: f64) -> String {
    match score {
        s if s >= 7.0 => "HIGH",
        s if s >= 4.0 => "MEDIUM",
        _ => "LOW",
    }.to_string()
}

fn parse_cpe(cpe: &str) -> Option<WiaAffectedProduct> {
    // CPE 2.3 format: cpe:2.3:a:vendor:product:version:...
    let parts: Vec<&str> = cpe.split(':').collect();
    if parts.len() >= 5 {
        Some(WiaAffectedProduct {
            vendor: parts[3].to_string(),
            product: parts[4].to_string(),
            versions: if parts.len() > 5 && parts[5] != "*" {
                vec![parts[5].to_string()]
            } else {
                vec![]
            },
            cpe: cpe.to_string(),
        })
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nvd_config_default() {
        let config = NvdConfig::default();
        assert_eq!(config.rate_limit, 5);
        assert!(config.api_key.is_none());
    }

    #[test]
    fn test_nvd_config_with_api_key() {
        let config = NvdConfig::with_api_key("test-key");
        assert_eq!(config.rate_limit, 50);
        assert_eq!(config.api_key, Some("test-key".to_string()));
    }

    #[test]
    fn test_build_cve_url() {
        let importer = NvdImporter::default_config();
        let url = importer.build_cve_url("CVE-2024-12345");
        assert!(url.contains("cveId=CVE-2024-12345"));
    }

    #[test]
    fn test_search_params() {
        let params = NvdSearchParams::new()
            .keyword("apache")
            .cvss_min(7.0)
            .page(0, 100);

        assert_eq!(params.keyword, Some("apache".to_string()));
        assert_eq!(params.cvss_v3_min, Some(7.0));
    }

    #[test]
    fn test_parse_cpe() {
        let cpe = "cpe:2.3:a:apache:tomcat:9.0.50:*:*:*:*:*:*:*";
        let product = parse_cpe(cpe).unwrap();
        assert_eq!(product.vendor, "apache");
        assert_eq!(product.product, "tomcat");
    }
}
