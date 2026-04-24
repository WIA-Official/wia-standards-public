//! Threat Intelligence Feed Integration
//!
//! MISP, AlienVault OTX, VirusTotal

use serde::{Deserialize, Serialize};
use std::time::Duration;

// ============================================================================
// Types
// ============================================================================

/// Threat intelligence feed configuration
#[derive(Debug, Clone)]
pub struct ThreatIntelFeedConfig {
    pub url: String,
    pub api_key: String,
    pub timeout: Duration,
    pub poll_interval: Duration,
}

impl Default for ThreatIntelFeedConfig {
    fn default() -> Self {
        Self {
            url: String::new(),
            api_key: String::new(),
            timeout: Duration::from_secs(30),
            poll_interval: Duration::from_secs(300),
        }
    }
}

/// Indicator of Compromise
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IoC {
    pub ioc_type: IndicatorType,
    pub value: String,
    pub confidence: Option<f64>,
    pub first_seen: Option<String>,
    pub last_seen: Option<String>,
    pub tags: Vec<String>,
    pub source: String,
}

/// Indicator types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IndicatorType {
    Ipv4,
    Ipv6,
    Domain,
    Url,
    Email,
    Md5,
    Sha1,
    Sha256,
    Ssdeep,
    Imphash,
    Other,
}

impl IndicatorType {
    pub fn from_hash_length(len: usize) -> Self {
        match len {
            32 => Self::Md5,
            40 => Self::Sha1,
            64 => Self::Sha256,
            _ => Self::Other,
        }
    }
}

/// Threat feed data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThreatFeed {
    pub id: String,
    pub name: String,
    pub provider: String,
    pub last_updated: String,
    pub indicators: Vec<IoC>,
}

/// Result type for threat intel operations
pub type ThreatIntelResult<T> = Result<T, ThreatIntelError>;

/// Threat intel error
#[derive(Debug)]
pub enum ThreatIntelError {
    ApiError { status: u16, message: String },
    ParseError(String),
    NetworkError(String),
    RateLimited(Duration),
    NotFound,
}

impl std::fmt::Display for ThreatIntelError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ApiError { status, message } => write!(f, "API error ({}): {}", status, message),
            Self::ParseError(msg) => write!(f, "Parse error: {}", msg),
            Self::NetworkError(msg) => write!(f, "Network error: {}", msg),
            Self::RateLimited(duration) => write!(f, "Rate limited, retry after {:?}", duration),
            Self::NotFound => write!(f, "Not found"),
        }
    }
}

impl std::error::Error for ThreatIntelError {}

// ============================================================================
// MISP Client
// ============================================================================

/// MISP configuration
#[derive(Debug, Clone)]
pub struct MispConfig {
    pub url: String,
    pub api_key: String,
    pub timeout: Duration,
    pub verify_tls: bool,
}

impl Default for MispConfig {
    fn default() -> Self {
        Self {
            url: String::new(),
            api_key: String::new(),
            timeout: Duration::from_secs(30),
            verify_tls: true,
        }
    }
}

/// MISP client
pub struct MispClient {
    _config: MispConfig,
}

impl MispClient {
    /// Create a new MISP client
    pub fn new(config: MispConfig) -> Self {
        Self { _config: config }
    }

    /// Search for events
    pub async fn search_events(&self, query: MispSearchQuery) -> ThreatIntelResult<ThreatFeed> {
        let _body = serde_json::json!({
            "returnFormat": "json",
            "limit": query.limit.unwrap_or(100),
            "type": query.indicator_type,
            "value": query.value,
            "category": query.category,
            "org": query.org,
            "tags": query.tags,
            "from": query.from,
            "to": query.to,
        });

        // In production, make HTTP request to MISP API
        // POST {url}/events/restSearch

        Ok(ThreatFeed {
            id: format!("misp-{}", chrono::Utc::now().timestamp()),
            name: "MISP Feed".to_string(),
            provider: "MISP".to_string(),
            last_updated: chrono::Utc::now().to_rfc3339(),
            indicators: vec![],
        })
    }

    /// Get attributes by type
    pub async fn get_attributes(
        &self,
        attr_type: &str,
        limit: usize,
    ) -> ThreatIntelResult<Vec<IoC>> {
        let _body = serde_json::json!({
            "returnFormat": "json",
            "type": attr_type,
            "limit": limit,
        });

        // In production, make HTTP request
        Ok(vec![])
    }

    /// Add event to MISP
    pub async fn add_event(&self, event: MispEvent) -> ThreatIntelResult<String> {
        let _body = serde_json::json!({
            "Event": {
                "info": event.info,
                "threat_level_id": event.threat_level_id,
                "analysis": event.analysis,
                "distribution": event.distribution,
                "Attribute": event.attributes.iter().map(|attr| {
                    serde_json::json!({
                        "type": attr.attr_type,
                        "value": attr.value,
                        "category": attr.category,
                        "to_ids": attr.to_ids,
                    })
                }).collect::<Vec<_>>(),
            }
        });

        // In production, make HTTP request
        Ok(format!("misp-event-{}", chrono::Utc::now().timestamp()))
    }

    fn _map_misp_type(misp_type: &str) -> IndicatorType {
        match misp_type {
            "ip-src" | "ip-dst" => IndicatorType::Ipv4,
            "domain" | "hostname" => IndicatorType::Domain,
            "url" => IndicatorType::Url,
            "md5" => IndicatorType::Md5,
            "sha1" => IndicatorType::Sha1,
            "sha256" => IndicatorType::Sha256,
            "email-src" | "email-dst" => IndicatorType::Email,
            _ => IndicatorType::Other,
        }
    }
}

/// MISP search query
#[derive(Debug, Clone, Default)]
pub struct MispSearchQuery {
    pub indicator_type: Option<String>,
    pub value: Option<String>,
    pub category: Option<String>,
    pub org: Option<String>,
    pub tags: Option<Vec<String>>,
    pub from: Option<String>,
    pub to: Option<String>,
    pub limit: Option<usize>,
}

/// MISP event for submission
#[derive(Debug, Clone)]
pub struct MispEvent {
    pub info: String,
    pub threat_level_id: u8,
    pub analysis: u8,
    pub distribution: u8,
    pub attributes: Vec<MispAttribute>,
}

/// MISP attribute
#[derive(Debug, Clone)]
pub struct MispAttribute {
    pub attr_type: String,
    pub value: String,
    pub category: String,
    pub to_ids: bool,
}

// ============================================================================
// AlienVault OTX Client
// ============================================================================

/// OTX configuration
#[derive(Debug, Clone)]
pub struct OtxConfig {
    pub api_key: String,
    pub timeout: Duration,
}

impl Default for OtxConfig {
    fn default() -> Self {
        Self {
            api_key: String::new(),
            timeout: Duration::from_secs(30),
        }
    }
}

/// AlienVault OTX client
pub struct OtxClient {
    _config: OtxConfig,
    _base_url: String,
}

impl OtxClient {
    /// Create a new OTX client
    pub fn new(config: OtxConfig) -> Self {
        Self {
            _config: config,
            _base_url: "https://otx.alienvault.com".to_string(),
        }
    }

    /// Get subscribed pulses
    pub async fn get_subscribed_pulses(&self, _limit: usize) -> ThreatIntelResult<Vec<ThreatFeed>> {
        // In production, make HTTP request
        // GET {base_url}/api/v1/pulses/subscribed?limit={limit}
        Ok(vec![])
    }

    /// Get pulse by ID
    pub async fn get_pulse(&self, pulse_id: &str) -> ThreatIntelResult<ThreatFeed> {
        // In production, make HTTP request
        // GET {base_url}/api/v1/pulses/{pulse_id}
        Ok(ThreatFeed {
            id: pulse_id.to_string(),
            name: "OTX Pulse".to_string(),
            provider: "AlienVault OTX".to_string(),
            last_updated: chrono::Utc::now().to_rfc3339(),
            indicators: vec![],
        })
    }

    /// Search indicators
    pub async fn search_indicators(
        &self,
        indicator_type: OtxIndicatorType,
        _value: &str,
    ) -> ThreatIntelResult<Vec<IoC>> {
        let _type_str = match indicator_type {
            OtxIndicatorType::IPv4 => "IPv4",
            OtxIndicatorType::IPv6 => "IPv6",
            OtxIndicatorType::Domain => "domain",
            OtxIndicatorType::Hostname => "hostname",
            OtxIndicatorType::Url => "url",
            OtxIndicatorType::Md5 => "FileHash-MD5",
            OtxIndicatorType::Sha1 => "FileHash-SHA1",
            OtxIndicatorType::Sha256 => "FileHash-SHA256",
        };

        // In production, make HTTP request
        // GET {base_url}/api/v1/indicators/{type}/{value}/general
        Ok(vec![])
    }
}

/// OTX indicator types
#[derive(Debug, Clone, Copy)]
pub enum OtxIndicatorType {
    IPv4,
    IPv6,
    Domain,
    Hostname,
    Url,
    Md5,
    Sha1,
    Sha256,
}

// ============================================================================
// VirusTotal Client
// ============================================================================

/// VirusTotal configuration
#[derive(Debug, Clone)]
pub struct VirusTotalConfig {
    pub api_key: String,
    pub timeout: Duration,
}

impl Default for VirusTotalConfig {
    fn default() -> Self {
        Self {
            api_key: String::new(),
            timeout: Duration::from_secs(30),
        }
    }
}

/// VirusTotal client
pub struct VirusTotalClient {
    _config: VirusTotalConfig,
    _base_url: String,
}

impl VirusTotalClient {
    /// Create a new VirusTotal client
    pub fn new(config: VirusTotalConfig) -> Self {
        Self {
            _config: config,
            _base_url: "https://www.virustotal.com/api/v3".to_string(),
        }
    }

    /// Get file report by hash
    pub async fn get_file_report(&self, _hash: &str) -> ThreatIntelResult<Option<IoC>> {
        // In production, make HTTP request
        // GET {base_url}/files/{hash}
        Ok(None)
    }

    /// Get URL report
    pub async fn get_url_report(&self, url: &str) -> ThreatIntelResult<Option<IoC>> {
        // URL ID is base64 encoded
        let _url_id = base64::Engine::encode(
            &base64::engine::general_purpose::URL_SAFE_NO_PAD,
            url.as_bytes(),
        );

        // In production, make HTTP request
        // GET {base_url}/urls/{url_id}
        Ok(None)
    }

    /// Get domain report
    pub async fn get_domain_report(&self, _domain: &str) -> ThreatIntelResult<Option<IoC>> {
        // In production, make HTTP request
        // GET {base_url}/domains/{domain}
        Ok(None)
    }

    /// Get IP report
    pub async fn get_ip_report(&self, _ip: &str) -> ThreatIntelResult<Option<IoC>> {
        // In production, make HTTP request
        // GET {base_url}/ip_addresses/{ip}
        Ok(None)
    }

    fn _calculate_confidence(stats: &VtAnalysisStats) -> f64 {
        let total = stats.malicious + stats.suspicious + stats.undetected + stats.harmless;
        if total > 0 {
            (stats.malicious as f64 / total as f64) * 100.0
        } else {
            0.0
        }
    }
}

/// VirusTotal analysis stats
#[derive(Debug, Clone, Default)]
pub struct VtAnalysisStats {
    pub malicious: usize,
    pub suspicious: usize,
    pub undetected: usize,
    pub harmless: usize,
}

// ============================================================================
// Factory Functions
// ============================================================================

/// Create MISP client
pub fn create_misp_client(config: MispConfig) -> MispClient {
    MispClient::new(config)
}

/// Create OTX client
pub fn create_otx_client(config: OtxConfig) -> OtxClient {
    OtxClient::new(config)
}

/// Create VirusTotal client
pub fn create_virustotal_client(config: VirusTotalConfig) -> VirusTotalClient {
    VirusTotalClient::new(config)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_indicator_type_from_hash_length() {
        assert_eq!(IndicatorType::from_hash_length(32), IndicatorType::Md5);
        assert_eq!(IndicatorType::from_hash_length(40), IndicatorType::Sha1);
        assert_eq!(IndicatorType::from_hash_length(64), IndicatorType::Sha256);
        assert_eq!(IndicatorType::from_hash_length(128), IndicatorType::Other);
    }
}
