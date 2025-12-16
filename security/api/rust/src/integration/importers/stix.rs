//! STIX/TAXII Threat Intelligence Importer
//!
//! Imports threat intelligence from TAXII 2.1 servers and STIX 2.1 bundles.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use super::{ImportResult, ImportError};

// Re-export TAXII types from protocol module
pub use crate::protocol::taxii::{
    TaxiiDiscovery, TaxiiApiRoot, TaxiiCollection, TaxiiCollections,
    StixBundle, StixObject, StixObjectType, StixIndicator, StixThreatActor,
    KillChainPhase, ExternalReference, TaxiiObjectsResponse,
};

/// TAXII client configuration
#[derive(Debug, Clone)]
pub struct TaxiiClientConfig {
    /// API root URL
    pub api_root: String,
    /// Username for basic auth
    pub username: Option<String>,
    /// Password for basic auth
    pub password: Option<String>,
    /// Bearer token
    pub token: Option<String>,
    /// Request timeout in seconds
    pub timeout_seconds: u64,
    /// Verify SSL certificates
    pub verify_ssl: bool,
}

impl TaxiiClientConfig {
    /// Create with basic auth
    pub fn basic_auth(api_root: impl Into<String>, username: impl Into<String>, password: impl Into<String>) -> Self {
        Self {
            api_root: api_root.into(),
            username: Some(username.into()),
            password: Some(password.into()),
            token: None,
            timeout_seconds: 30,
            verify_ssl: true,
        }
    }

    /// Create with bearer token
    pub fn bearer_token(api_root: impl Into<String>, token: impl Into<String>) -> Self {
        Self {
            api_root: api_root.into(),
            username: None,
            password: None,
            token: Some(token.into()),
            timeout_seconds: 30,
            verify_ssl: true,
        }
    }
}

/// TAXII 2.1 Client
pub struct TaxiiClient {
    config: TaxiiClientConfig,
}

impl TaxiiClient {
    /// Create new TAXII client
    pub fn new(config: TaxiiClientConfig) -> Self {
        Self { config }
    }

    /// Build discovery URL
    pub fn discovery_url(&self) -> String {
        // Extract base URL from API root
        let base = self.config.api_root.trim_end_matches('/');
        if let Some(pos) = base.rfind("/taxii2") {
            format!("{}/taxii2/", &base[..pos])
        } else {
            format!("{}/", base)
        }
    }

    /// Build collections URL
    pub fn collections_url(&self) -> String {
        format!("{}/collections/", self.config.api_root.trim_end_matches('/'))
    }

    /// Build objects URL for a collection
    pub fn objects_url(&self, collection_id: &str) -> String {
        format!(
            "{}/collections/{}/objects/",
            self.config.api_root.trim_end_matches('/'),
            collection_id
        )
    }

    /// Build objects URL with filters
    pub fn objects_url_with_filters(&self, collection_id: &str, filters: &TaxiiFilters) -> String {
        let base_url = self.objects_url(collection_id);
        let mut params = Vec::new();

        if let Some(added_after) = &filters.added_after {
            params.push(format!("added_after={}", added_after));
        }
        if let Some(limit) = filters.limit {
            params.push(format!("limit={}", limit));
        }
        if let Some(types) = &filters.types {
            for t in types {
                params.push(format!("type={}", t));
            }
        }
        if let Some(next) = &filters.next {
            params.push(format!("next={}", next));
        }

        if params.is_empty() {
            base_url
        } else {
            format!("{}?{}", base_url, params.join("&"))
        }
    }

    /// Get authorization header value
    pub fn auth_header(&self) -> Option<String> {
        if let Some(token) = &self.config.token {
            Some(format!("Bearer {}", token))
        } else if let (Some(user), Some(pass)) = (&self.config.username, &self.config.password) {
            let credentials = base64_encode(&format!("{}:{}", user, pass));
            Some(format!("Basic {}", credentials))
        } else {
            None
        }
    }
}

/// TAXII query filters
#[derive(Debug, Clone, Default)]
pub struct TaxiiFilters {
    /// Return objects added after this timestamp
    pub added_after: Option<String>,
    /// Maximum number of objects to return
    pub limit: Option<u32>,
    /// Filter by STIX object types
    pub types: Option<Vec<String>>,
    /// Pagination cursor
    pub next: Option<String>,
}

impl TaxiiFilters {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn added_after(mut self, timestamp: impl Into<String>) -> Self {
        self.added_after = Some(timestamp.into());
        self
    }

    pub fn limit(mut self, limit: u32) -> Self {
        self.limit = Some(limit);
        self
    }

    pub fn types(mut self, types: Vec<String>) -> Self {
        self.types = Some(types);
        self
    }

    pub fn indicators_only(self) -> Self {
        self.types(vec!["indicator".to_string()])
    }
}

/// WIA Threat Intelligence format
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaThreatIntel {
    pub id: String,
    pub intel_type: String,
    pub name: String,
    pub description: Option<String>,
    pub confidence: Option<u32>,
    pub created: String,
    pub modified: String,
    pub indicators: Vec<WiaIndicator>,
    pub threat_actors: Vec<WiaThreatActorInfo>,
    pub kill_chain: Vec<WiaKillChain>,
    pub references: Vec<String>,
    pub labels: Vec<String>,
}

/// WIA Indicator
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaIndicator {
    pub id: String,
    pub pattern: String,
    pub pattern_type: String,
    pub indicator_types: Vec<String>,
    pub valid_from: String,
    pub valid_until: Option<String>,
    pub confidence: Option<u32>,
    pub ioc_type: String,
    pub ioc_value: String,
}

/// WIA Threat Actor Info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaThreatActorInfo {
    pub id: String,
    pub name: String,
    pub aliases: Vec<String>,
    pub sophistication: Option<String>,
    pub resource_level: Option<String>,
    pub motivation: Option<String>,
}

/// WIA Kill Chain
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaKillChain {
    pub kill_chain_name: String,
    pub phase_name: String,
}

/// STIX to WIA converter
pub struct StixConverter;

impl StixConverter {
    /// Convert STIX bundle to WIA threat intel
    pub fn convert_bundle(bundle: &StixBundle) -> Vec<WiaThreatIntel> {
        let mut results = Vec::new();

        // Group related objects
        let indicators: Vec<&serde_json::Value> = bundle.objects.iter()
            .filter(|o| o.get("type").and_then(|t| t.as_str()) == Some("indicator"))
            .collect();

        let threat_actors: Vec<&serde_json::Value> = bundle.objects.iter()
            .filter(|o| o.get("type").and_then(|t| t.as_str()) == Some("threat-actor"))
            .collect();

        // Convert indicators
        for ind in indicators {
            if let Some(intel) = Self::convert_indicator(ind) {
                results.push(intel);
            }
        }

        // Convert threat actors
        for actor in threat_actors {
            if let Some(intel) = Self::convert_threat_actor(actor) {
                results.push(intel);
            }
        }

        results
    }

    /// Convert STIX indicator to WIA format
    fn convert_indicator(obj: &serde_json::Value) -> Option<WiaThreatIntel> {
        let id = obj.get("id")?.as_str()?.to_string();
        let name = obj.get("name")
            .and_then(|n| n.as_str())
            .unwrap_or("Unknown Indicator")
            .to_string();
        let pattern = obj.get("pattern")?.as_str()?.to_string();
        let pattern_type = obj.get("pattern_type")
            .and_then(|p| p.as_str())
            .unwrap_or("stix")
            .to_string();

        let (ioc_type, ioc_value) = Self::parse_pattern(&pattern);

        let indicator = WiaIndicator {
            id: id.clone(),
            pattern: pattern.clone(),
            pattern_type,
            indicator_types: obj.get("indicator_types")
                .and_then(|t| t.as_array())
                .map(|arr| arr.iter().filter_map(|v| v.as_str().map(String::from)).collect())
                .unwrap_or_default(),
            valid_from: obj.get("valid_from")
                .and_then(|v| v.as_str())
                .unwrap_or("")
                .to_string(),
            valid_until: obj.get("valid_until")
                .and_then(|v| v.as_str())
                .map(String::from),
            confidence: obj.get("confidence")
                .and_then(|c| c.as_u64())
                .map(|c| c as u32),
            ioc_type,
            ioc_value,
        };

        let kill_chain = obj.get("kill_chain_phases")
            .and_then(|k| k.as_array())
            .map(|arr| {
                arr.iter()
                    .filter_map(|phase| {
                        Some(WiaKillChain {
                            kill_chain_name: phase.get("kill_chain_name")?.as_str()?.to_string(),
                            phase_name: phase.get("phase_name")?.as_str()?.to_string(),
                        })
                    })
                    .collect()
            })
            .unwrap_or_default();

        Some(WiaThreatIntel {
            id,
            intel_type: "indicator".to_string(),
            name,
            description: obj.get("description").and_then(|d| d.as_str()).map(String::from),
            confidence: obj.get("confidence").and_then(|c| c.as_u64()).map(|c| c as u32),
            created: obj.get("created").and_then(|c| c.as_str()).unwrap_or("").to_string(),
            modified: obj.get("modified").and_then(|m| m.as_str()).unwrap_or("").to_string(),
            indicators: vec![indicator],
            threat_actors: vec![],
            kill_chain,
            references: Self::extract_references(obj),
            labels: obj.get("labels")
                .and_then(|l| l.as_array())
                .map(|arr| arr.iter().filter_map(|v| v.as_str().map(String::from)).collect())
                .unwrap_or_default(),
        })
    }

    /// Convert STIX threat actor to WIA format
    fn convert_threat_actor(obj: &serde_json::Value) -> Option<WiaThreatIntel> {
        let id = obj.get("id")?.as_str()?.to_string();
        let name = obj.get("name")?.as_str()?.to_string();

        let actor_info = WiaThreatActorInfo {
            id: id.clone(),
            name: name.clone(),
            aliases: obj.get("aliases")
                .and_then(|a| a.as_array())
                .map(|arr| arr.iter().filter_map(|v| v.as_str().map(String::from)).collect())
                .unwrap_or_default(),
            sophistication: obj.get("sophistication").and_then(|s| s.as_str()).map(String::from),
            resource_level: obj.get("resource_level").and_then(|r| r.as_str()).map(String::from),
            motivation: obj.get("primary_motivation").and_then(|m| m.as_str()).map(String::from),
        };

        Some(WiaThreatIntel {
            id,
            intel_type: "threat_actor".to_string(),
            name,
            description: obj.get("description").and_then(|d| d.as_str()).map(String::from),
            confidence: None,
            created: obj.get("created").and_then(|c| c.as_str()).unwrap_or("").to_string(),
            modified: obj.get("modified").and_then(|m| m.as_str()).unwrap_or("").to_string(),
            indicators: vec![],
            threat_actors: vec![actor_info],
            kill_chain: vec![],
            references: Self::extract_references(obj),
            labels: obj.get("labels")
                .and_then(|l| l.as_array())
                .map(|arr| arr.iter().filter_map(|v| v.as_str().map(String::from)).collect())
                .unwrap_or_default(),
        })
    }

    /// Parse STIX pattern to extract IOC type and value
    fn parse_pattern(pattern: &str) -> (String, String) {
        // Simple pattern parsing for common IOC types
        // [ipv4-addr:value = '192.168.1.1']
        // [domain-name:value = 'example.com']
        // [file:hashes.'SHA-256' = 'abc123']

        if pattern.contains("ipv4-addr:value") {
            if let Some(value) = Self::extract_pattern_value(pattern) {
                return ("ipv4".to_string(), value);
            }
        }
        if pattern.contains("ipv6-addr:value") {
            if let Some(value) = Self::extract_pattern_value(pattern) {
                return ("ipv6".to_string(), value);
            }
        }
        if pattern.contains("domain-name:value") {
            if let Some(value) = Self::extract_pattern_value(pattern) {
                return ("domain".to_string(), value);
            }
        }
        if pattern.contains("url:value") {
            if let Some(value) = Self::extract_pattern_value(pattern) {
                return ("url".to_string(), value);
            }
        }
        if pattern.contains("file:hashes") {
            if let Some(value) = Self::extract_pattern_value(pattern) {
                return ("file_hash".to_string(), value);
            }
        }
        if pattern.contains("email-addr:value") {
            if let Some(value) = Self::extract_pattern_value(pattern) {
                return ("email".to_string(), value);
            }
        }

        ("unknown".to_string(), pattern.to_string())
    }

    /// Extract value from STIX pattern
    fn extract_pattern_value(pattern: &str) -> Option<String> {
        // Find value between single quotes
        let start = pattern.find('\'')?;
        let end = pattern.rfind('\'')?;
        if start < end {
            Some(pattern[start + 1..end].to_string())
        } else {
            None
        }
    }

    /// Extract external references
    fn extract_references(obj: &serde_json::Value) -> Vec<String> {
        obj.get("external_references")
            .and_then(|refs| refs.as_array())
            .map(|arr| {
                arr.iter()
                    .filter_map(|r| r.get("url").and_then(|u| u.as_str()).map(String::from))
                    .collect()
            })
            .unwrap_or_default()
    }
}

// Helper function for base64 encoding (simplified)
fn base64_encode(input: &str) -> String {
    use std::io::Write;
    let mut encoder = base64::write::EncoderStringWriter::new(&base64::engine::general_purpose::STANDARD);
    encoder.write_all(input.as_bytes()).unwrap();
    encoder.into_inner()
}

// Mock base64 module for the above
mod base64 {
    pub mod write {
        use std::io::{self, Write};

        pub struct EncoderStringWriter {
            buffer: String,
        }

        impl EncoderStringWriter {
            pub fn new(_engine: &super::engine::general_purpose::GeneralPurpose) -> Self {
                Self { buffer: String::new() }
            }

            pub fn into_inner(self) -> String {
                // Simple base64 encoding
                use std::io::Read;
                base64_simple(&self.buffer.as_bytes())
            }
        }

        impl Write for EncoderStringWriter {
            fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
                self.buffer.push_str(&String::from_utf8_lossy(buf));
                Ok(buf.len())
            }

            fn flush(&mut self) -> io::Result<()> {
                Ok(())
            }
        }

        fn base64_simple(input: &[u8]) -> String {
            const ALPHABET: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
            let mut result = String::new();

            for chunk in input.chunks(3) {
                let n = match chunk.len() {
                    3 => ((chunk[0] as u32) << 16) | ((chunk[1] as u32) << 8) | (chunk[2] as u32),
                    2 => ((chunk[0] as u32) << 16) | ((chunk[1] as u32) << 8),
                    1 => (chunk[0] as u32) << 16,
                    _ => continue,
                };

                result.push(ALPHABET[((n >> 18) & 0x3F) as usize] as char);
                result.push(ALPHABET[((n >> 12) & 0x3F) as usize] as char);

                if chunk.len() > 1 {
                    result.push(ALPHABET[((n >> 6) & 0x3F) as usize] as char);
                } else {
                    result.push('=');
                }

                if chunk.len() > 2 {
                    result.push(ALPHABET[(n & 0x3F) as usize] as char);
                } else {
                    result.push('=');
                }
            }

            result
        }
    }

    pub mod engine {
        pub mod general_purpose {
            pub struct GeneralPurpose;
            pub const STANDARD: GeneralPurpose = GeneralPurpose;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_taxii_client_urls() {
        let config = TaxiiClientConfig::bearer_token(
            "https://taxii.example.com/taxii2/api/v1",
            "test-token"
        );
        let client = TaxiiClient::new(config);

        assert_eq!(
            client.collections_url(),
            "https://taxii.example.com/taxii2/api/v1/collections/"
        );
    }

    #[test]
    fn test_parse_pattern() {
        let (ioc_type, value) = StixConverter::parse_pattern("[ipv4-addr:value = '192.168.1.100']");
        assert_eq!(ioc_type, "ipv4");
        assert_eq!(value, "192.168.1.100");

        let (ioc_type2, value2) = StixConverter::parse_pattern("[domain-name:value = 'malware.example.com']");
        assert_eq!(ioc_type2, "domain");
        assert_eq!(value2, "malware.example.com");
    }

    #[test]
    fn test_taxii_filters() {
        let filters = TaxiiFilters::new()
            .added_after("2024-12-01T00:00:00Z")
            .limit(100)
            .indicators_only();

        assert_eq!(filters.added_after, Some("2024-12-01T00:00:00Z".to_string()));
        assert_eq!(filters.limit, Some(100));
        assert_eq!(filters.types, Some(vec!["indicator".to_string()]));
    }
}
