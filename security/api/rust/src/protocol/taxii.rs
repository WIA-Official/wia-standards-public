//! TAXII 2.1 Compatible Protocol
//!
//! Implementation of TAXII 2.1 protocol for threat intelligence sharing.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use uuid::Uuid;
use chrono::Utc;

// ============================================================================
// TAXII Discovery
// ============================================================================

/// TAXII server discovery response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaxiiDiscovery {
    /// Server title
    pub title: String,
    /// Server description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Contact information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub contact: Option<String>,
    /// Default API root
    #[serde(skip_serializing_if = "Option::is_none")]
    pub default: Option<String>,
    /// Available API roots
    pub api_roots: Vec<String>,
}

impl TaxiiDiscovery {
    /// Create a new discovery response
    pub fn new(title: impl Into<String>) -> Self {
        Self {
            title: title.into(),
            description: None,
            contact: None,
            default: None,
            api_roots: vec![],
        }
    }

    /// Add API root
    pub fn with_api_root(mut self, root: impl Into<String>) -> Self {
        let root = root.into();
        if self.default.is_none() {
            self.default = Some(root.clone());
        }
        self.api_roots.push(root);
        self
    }
}

// ============================================================================
// API Root
// ============================================================================

/// TAXII API root information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaxiiApiRoot {
    /// API root title
    pub title: String,
    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Supported TAXII versions
    pub versions: Vec<String>,
    /// Maximum content length
    pub max_content_length: u64,
}

impl Default for TaxiiApiRoot {
    fn default() -> Self {
        Self {
            title: "WIA Security TAXII API".to_string(),
            description: Some("TAXII 2.1 compatible threat intelligence API".to_string()),
            versions: vec!["application/taxii+json;version=2.1".to_string()],
            max_content_length: 104857600, // 100MB
        }
    }
}

// ============================================================================
// Collections
// ============================================================================

/// Collection media types
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollectionMediaType {
    #[serde(rename = "application/stix+json;version=2.1")]
    Stix21,
}

/// TAXII collection
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaxiiCollection {
    /// Collection ID
    pub id: String,
    /// Collection title
    pub title: String,
    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Collection alias
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alias: Option<String>,
    /// Can read objects
    pub can_read: bool,
    /// Can write objects
    pub can_write: bool,
    /// Supported media types
    pub media_types: Vec<String>,
}

impl TaxiiCollection {
    /// Create a new collection
    pub fn new(id: impl Into<String>, title: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            title: title.into(),
            description: None,
            alias: None,
            can_read: true,
            can_write: false,
            media_types: vec!["application/stix+json;version=2.1".to_string()],
        }
    }

    /// Set write permission
    pub fn writable(mut self) -> Self {
        self.can_write = true;
        self
    }

    /// Set alias
    pub fn with_alias(mut self, alias: impl Into<String>) -> Self {
        self.alias = Some(alias.into());
        self
    }
}

/// Collections response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaxiiCollections {
    /// List of collections
    pub collections: Vec<TaxiiCollection>,
}

impl TaxiiCollections {
    /// Create empty collections
    pub fn new() -> Self {
        Self { collections: vec![] }
    }

    /// Add collection
    pub fn add(mut self, collection: TaxiiCollection) -> Self {
        self.collections.push(collection);
        self
    }
}

impl Default for TaxiiCollections {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// STIX Objects
// ============================================================================

/// STIX object types
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum StixObjectType {
    AttackPattern,
    Campaign,
    CourseOfAction,
    Grouping,
    Identity,
    Indicator,
    Infrastructure,
    IntrusionSet,
    Location,
    Malware,
    MalwareAnalysis,
    Note,
    ObservedData,
    Opinion,
    Report,
    ThreatActor,
    Tool,
    Vulnerability,
    Relationship,
    Sighting,
}

/// STIX base object
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StixObject {
    /// Object type
    #[serde(rename = "type")]
    pub object_type: StixObjectType,
    /// STIX spec version
    pub spec_version: String,
    /// Object ID
    pub id: String,
    /// Created timestamp
    pub created: String,
    /// Modified timestamp
    pub modified: String,
    /// Object name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Labels
    #[serde(skip_serializing_if = "Option::is_none")]
    pub labels: Option<Vec<String>>,
    /// Confidence (0-100)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidence: Option<u32>,
    /// External references
    #[serde(skip_serializing_if = "Option::is_none")]
    pub external_references: Option<Vec<ExternalReference>>,
    /// Object marking references
    #[serde(skip_serializing_if = "Option::is_none")]
    pub object_marking_refs: Option<Vec<String>>,
    /// Additional properties
    #[serde(flatten)]
    pub extensions: HashMap<String, serde_json::Value>,
}

impl StixObject {
    /// Create new STIX object
    pub fn new(object_type: StixObjectType) -> Self {
        let now = Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string();
        let id = format!(
            "{}--{}",
            format!("{:?}", object_type).to_lowercase().replace("_", "-"),
            Uuid::new_v4()
        );

        Self {
            object_type,
            spec_version: "2.1".to_string(),
            id,
            created: now.clone(),
            modified: now,
            name: None,
            description: None,
            labels: None,
            confidence: None,
            external_references: None,
            object_marking_refs: None,
            extensions: HashMap::new(),
        }
    }

    /// Set name
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Set description
    pub fn with_description(mut self, description: impl Into<String>) -> Self {
        self.description = Some(description.into());
        self
    }

    /// Set confidence
    pub fn with_confidence(mut self, confidence: u32) -> Self {
        self.confidence = Some(confidence.min(100));
        self
    }

    /// Add extension property
    pub fn with_extension(mut self, key: impl Into<String>, value: serde_json::Value) -> Self {
        self.extensions.insert(key.into(), value);
        self
    }
}

/// External reference
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExternalReference {
    /// Source name
    pub source_name: String,
    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// URL
    #[serde(skip_serializing_if = "Option::is_none")]
    pub url: Option<String>,
    /// External ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub external_id: Option<String>,
}

// ============================================================================
// STIX Indicator
// ============================================================================

/// STIX indicator
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StixIndicator {
    /// Base STIX object
    #[serde(flatten)]
    pub base: StixObject,
    /// Indicator pattern
    pub pattern: String,
    /// Pattern type
    pub pattern_type: String,
    /// Valid from
    pub valid_from: String,
    /// Valid until
    #[serde(skip_serializing_if = "Option::is_none")]
    pub valid_until: Option<String>,
    /// Kill chain phases
    #[serde(skip_serializing_if = "Option::is_none")]
    pub kill_chain_phases: Option<Vec<KillChainPhase>>,
    /// Indicator types
    #[serde(skip_serializing_if = "Option::is_none")]
    pub indicator_types: Option<Vec<String>>,
}

impl StixIndicator {
    /// Create new indicator with pattern
    pub fn new(pattern: impl Into<String>, pattern_type: impl Into<String>) -> Self {
        let now = Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string();
        Self {
            base: StixObject::new(StixObjectType::Indicator),
            pattern: pattern.into(),
            pattern_type: pattern_type.into(),
            valid_from: now,
            valid_until: None,
            kill_chain_phases: None,
            indicator_types: None,
        }
    }

    /// Create IP address indicator
    pub fn ip(ip: impl Into<String>) -> Self {
        Self::new(format!("[ipv4-addr:value = '{}']", ip.into()), "stix")
    }

    /// Create domain indicator
    pub fn domain(domain: impl Into<String>) -> Self {
        Self::new(format!("[domain-name:value = '{}']", domain.into()), "stix")
    }

    /// Create file hash indicator
    pub fn file_hash(hash_type: impl Into<String>, hash: impl Into<String>) -> Self {
        Self::new(
            format!("[file:hashes.'{}' = '{}']", hash_type.into(), hash.into()),
            "stix",
        )
    }

    /// Set name
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.base.name = Some(name.into());
        self
    }

    /// Add kill chain phase
    pub fn with_kill_chain(mut self, kill_chain_name: impl Into<String>, phase: impl Into<String>) -> Self {
        let phase = KillChainPhase {
            kill_chain_name: kill_chain_name.into(),
            phase_name: phase.into(),
        };
        self.kill_chain_phases.get_or_insert_with(Vec::new).push(phase);
        self
    }
}

/// Kill chain phase
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KillChainPhase {
    /// Kill chain name
    pub kill_chain_name: String,
    /// Phase name
    pub phase_name: String,
}

// ============================================================================
// STIX Threat Actor
// ============================================================================

/// STIX threat actor
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StixThreatActor {
    /// Base STIX object
    #[serde(flatten)]
    pub base: StixObject,
    /// Aliases
    #[serde(skip_serializing_if = "Option::is_none")]
    pub aliases: Option<Vec<String>>,
    /// Threat actor types
    #[serde(skip_serializing_if = "Option::is_none")]
    pub threat_actor_types: Option<Vec<String>>,
    /// Roles
    #[serde(skip_serializing_if = "Option::is_none")]
    pub roles: Option<Vec<String>>,
    /// Goals
    #[serde(skip_serializing_if = "Option::is_none")]
    pub goals: Option<Vec<String>>,
    /// Sophistication level
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sophistication: Option<String>,
    /// Resource level
    #[serde(skip_serializing_if = "Option::is_none")]
    pub resource_level: Option<String>,
    /// Primary motivation
    #[serde(skip_serializing_if = "Option::is_none")]
    pub primary_motivation: Option<String>,
}

impl StixThreatActor {
    /// Create new threat actor
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            base: StixObject::new(StixObjectType::ThreatActor).with_name(name),
            aliases: None,
            threat_actor_types: None,
            roles: None,
            goals: None,
            sophistication: None,
            resource_level: None,
            primary_motivation: None,
        }
    }

    /// Set aliases
    pub fn with_aliases(mut self, aliases: Vec<String>) -> Self {
        self.aliases = Some(aliases);
        self
    }

    /// Set sophistication
    pub fn with_sophistication(mut self, level: impl Into<String>) -> Self {
        self.sophistication = Some(level.into());
        self
    }

    /// Set motivation
    pub fn with_motivation(mut self, motivation: impl Into<String>) -> Self {
        self.primary_motivation = Some(motivation.into());
        self
    }
}

// ============================================================================
// STIX Bundle
// ============================================================================

/// STIX bundle
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StixBundle {
    /// Bundle type
    #[serde(rename = "type")]
    pub bundle_type: String,
    /// Bundle ID
    pub id: String,
    /// Objects in bundle
    pub objects: Vec<serde_json::Value>,
}

impl StixBundle {
    /// Create new bundle
    pub fn new() -> Self {
        Self {
            bundle_type: "bundle".to_string(),
            id: format!("bundle--{}", Uuid::new_v4()),
            objects: vec![],
        }
    }

    /// Add object to bundle
    pub fn add_object<T: Serialize>(mut self, object: T) -> Self {
        if let Ok(value) = serde_json::to_value(object) {
            self.objects.push(value);
        }
        self
    }
}

impl Default for StixBundle {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// TAXII Objects Response
// ============================================================================

/// Objects response with pagination
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaxiiObjectsResponse {
    /// Whether there are more objects
    pub more: bool,
    /// Next page ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub next: Option<String>,
    /// Objects
    pub objects: Vec<serde_json::Value>,
}

impl TaxiiObjectsResponse {
    /// Create new response
    pub fn new(objects: Vec<serde_json::Value>) -> Self {
        Self {
            more: false,
            next: None,
            objects,
        }
    }

    /// Set pagination
    pub fn with_pagination(mut self, next: Option<String>) -> Self {
        self.more = next.is_some();
        self.next = next;
        self
    }
}

// ============================================================================
// TAXII Status
// ============================================================================

/// TAXII status (for async operations)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaxiiStatus {
    /// Status ID
    pub id: String,
    /// Status
    pub status: String,
    /// Request timestamp
    pub request_timestamp: String,
    /// Total count
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total_count: Option<u32>,
    /// Success count
    #[serde(skip_serializing_if = "Option::is_none")]
    pub success_count: Option<u32>,
    /// Failure count
    #[serde(skip_serializing_if = "Option::is_none")]
    pub failure_count: Option<u32>,
    /// Pending count
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pending_count: Option<u32>,
    /// Failures
    #[serde(skip_serializing_if = "Option::is_none")]
    pub failures: Option<Vec<TaxiiStatusFailure>>,
}

/// Status failure detail
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaxiiStatusFailure {
    /// Object ID
    pub id: String,
    /// Error message
    pub message: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_taxii_discovery() {
        let discovery = TaxiiDiscovery::new("WIA Threat Intel")
            .with_api_root("https://threat-intel.wia.live/taxii2/api/v1/");

        assert_eq!(discovery.api_roots.len(), 1);
        assert!(discovery.default.is_some());
    }

    #[test]
    fn test_stix_indicator() {
        let indicator = StixIndicator::ip("192.168.1.100")
            .with_name("Malicious IP")
            .with_kill_chain("mitre-attack", "command-and-control");

        assert!(indicator.pattern.contains("192.168.1.100"));
        assert_eq!(indicator.pattern_type, "stix");
        assert!(indicator.kill_chain_phases.is_some());
    }

    #[test]
    fn test_stix_bundle() {
        let indicator = StixIndicator::domain("malware.example.com")
            .with_name("Malicious Domain");

        let bundle = StixBundle::new()
            .add_object(indicator);

        assert_eq!(bundle.objects.len(), 1);
        assert!(bundle.id.starts_with("bundle--"));
    }

    #[test]
    fn test_taxii_collection() {
        let collection = TaxiiCollection::new("collection-001", "WIA Indicators")
            .with_alias("wia-ioc")
            .writable();

        assert!(collection.can_read);
        assert!(collection.can_write);
    }
}
