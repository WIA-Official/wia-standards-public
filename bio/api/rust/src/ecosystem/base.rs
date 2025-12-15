//! Base adapter interface and types

use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use std::fmt::Debug;
use thiserror::Error;
use chrono::{DateTime, Utc};
use std::collections::HashMap;

use crate::types::{Sequence, CrisprExperiment, ProteinStructure, BioPart};

/// Adapter error types
#[derive(Debug, Clone, Error)]
pub enum AdapterError {
    /// Connection failed
    #[error("Connection failed: {0}")]
    ConnectionFailed(String),

    /// Authentication failed
    #[error("Authentication failed: {0}")]
    AuthenticationFailed(String),

    /// Request timeout
    #[error("Request timeout after {0}ms")]
    Timeout(u64),

    /// Resource not found
    #[error("Resource not found: {0}")]
    NotFound(String),

    /// Conversion failed
    #[error("Conversion failed: {0}")]
    ConversionFailed(String),

    /// Rate limited
    #[error("Rate limited: retry after {0}s")]
    RateLimited(u64),

    /// Server error
    #[error("Server error ({code}): {message}")]
    ServerError { code: u16, message: String },

    /// Validation failed
    #[error("Validation failed: {0}")]
    ValidationFailed(String),

    /// Not implemented
    #[error("Not implemented: {0}")]
    NotImplemented(String),

    /// Configuration error
    #[error("Configuration error: {0}")]
    ConfigurationError(String),
}

/// Adapter types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AdapterType {
    /// GA4GH services
    Ga4gh,
    /// HL7 FHIR Genomics
    Fhir,
    /// LIMS systems
    Lims,
    /// ELN platforms
    Eln,
    /// Bioinformatics databases
    Database,
    /// Mock adapter for testing
    Mock,
    /// Custom adapter
    Custom,
}

impl AdapterType {
    /// Get adapter type name
    pub fn name(&self) -> &str {
        match self {
            AdapterType::Ga4gh => "ga4gh",
            AdapterType::Fhir => "fhir",
            AdapterType::Lims => "lims",
            AdapterType::Eln => "eln",
            AdapterType::Database => "database",
            AdapterType::Mock => "mock",
            AdapterType::Custom => "custom",
        }
    }
}

/// Health status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealthStatus {
    /// Is the adapter healthy
    pub healthy: bool,
    /// Latency in milliseconds
    pub latency_ms: u64,
    /// Status message
    pub message: Option<String>,
    /// Timestamp
    pub timestamp: DateTime<Utc>,
}

impl HealthStatus {
    /// Create healthy status
    pub fn healthy(latency_ms: u64) -> Self {
        Self {
            healthy: true,
            latency_ms,
            message: None,
            timestamp: Utc::now(),
        }
    }

    /// Create unhealthy status
    pub fn unhealthy(message: impl Into<String>) -> Self {
        Self {
            healthy: false,
            latency_ms: 0,
            message: Some(message.into()),
            timestamp: Utc::now(),
        }
    }
}

/// Base ecosystem adapter interface
#[async_trait]
pub trait IEcosystemAdapter: Send + Sync + Debug {
    /// Get adapter type
    fn adapter_type(&self) -> AdapterType;

    /// Get adapter name
    fn name(&self) -> &str;

    /// Initialize the adapter
    async fn initialize(&mut self, config: &super::AdapterConfig) -> Result<(), AdapterError>;

    /// Check health status
    async fn health_check(&self) -> Result<HealthStatus, AdapterError>;

    /// Shutdown the adapter
    async fn shutdown(&mut self) -> Result<(), AdapterError>;

    /// Check if initialized
    fn is_initialized(&self) -> bool;
}

/// Export result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExportResult {
    /// Was the export successful
    pub success: bool,
    /// External ID assigned by the target system
    pub external_id: Option<String>,
    /// URL to the exported resource
    pub url: Option<String>,
    /// Message
    pub message: Option<String>,
    /// Metadata
    pub metadata: HashMap<String, String>,
}

impl ExportResult {
    /// Create success result
    pub fn success(external_id: impl Into<String>) -> Self {
        Self {
            success: true,
            external_id: Some(external_id.into()),
            url: None,
            message: None,
            metadata: HashMap::new(),
        }
    }

    /// Create failure result
    pub fn failure(message: impl Into<String>) -> Self {
        Self {
            success: false,
            external_id: None,
            url: None,
            message: Some(message.into()),
            metadata: HashMap::new(),
        }
    }

    /// Add URL
    pub fn with_url(mut self, url: impl Into<String>) -> Self {
        self.url = Some(url.into());
        self
    }

    /// Add metadata
    pub fn with_metadata(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.metadata.insert(key.into(), value.into());
        self
    }
}

/// Export item types
#[derive(Debug, Clone)]
pub enum ExportItem {
    /// Sequence data
    Sequence(Sequence),
    /// CRISPR experiment
    Experiment(CrisprExperiment),
    /// Protein structure
    Structure(ProteinStructure),
    /// BioPart
    Part(BioPart),
}

/// Data exporter interface
#[async_trait]
pub trait IDataExporter: IEcosystemAdapter {
    /// Export a sequence
    async fn export_sequence(&self, sequence: &Sequence) -> Result<ExportResult, AdapterError>;

    /// Export an experiment
    async fn export_experiment(&self, experiment: &CrisprExperiment) -> Result<ExportResult, AdapterError>;

    /// Export a structure
    async fn export_structure(&self, structure: &ProteinStructure) -> Result<ExportResult, AdapterError>;

    /// Export a part
    async fn export_part(&self, part: &BioPart) -> Result<ExportResult, AdapterError>;

    /// Export multiple items
    async fn export_batch(&self, items: Vec<ExportItem>) -> Result<Vec<ExportResult>, AdapterError> {
        let mut results = Vec::new();
        for item in items {
            let result = match item {
                ExportItem::Sequence(ref seq) => self.export_sequence(seq).await,
                ExportItem::Experiment(ref exp) => self.export_experiment(exp).await,
                ExportItem::Structure(ref st) => self.export_structure(st).await,
                ExportItem::Part(ref part) => self.export_part(part).await,
            };
            results.push(result.unwrap_or_else(|e| ExportResult::failure(e.to_string())));
        }
        Ok(results)
    }
}

/// Search query
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SearchQuery {
    /// Text search
    pub text: Option<String>,
    /// Organism filter
    pub organism: Option<String>,
    /// Gene symbol filter
    pub gene: Option<String>,
    /// Limit results
    pub limit: u32,
    /// Offset for pagination
    pub offset: u32,
}

impl Default for SearchQuery {
    fn default() -> Self {
        Self {
            text: None,
            organism: None,
            gene: None,
            limit: 10,
            offset: 0,
        }
    }
}

impl SearchQuery {
    /// Create new search query
    pub fn new(text: impl Into<String>) -> Self {
        Self {
            text: Some(text.into()),
            ..Default::default()
        }
    }

    /// Set organism filter
    pub fn organism(mut self, organism: impl Into<String>) -> Self {
        self.organism = Some(organism.into());
        self
    }

    /// Set gene filter
    pub fn gene(mut self, gene: impl Into<String>) -> Self {
        self.gene = Some(gene.into());
        self
    }

    /// Set limit
    pub fn limit(mut self, limit: u32) -> Self {
        self.limit = limit;
        self
    }

    /// Set offset
    pub fn offset(mut self, offset: u32) -> Self {
        self.offset = offset;
        self
    }
}

/// Data importer interface
#[async_trait]
pub trait IDataImporter: IEcosystemAdapter {
    /// Import a sequence by external ID
    async fn import_sequence(&self, external_id: &str) -> Result<Sequence, AdapterError>;

    /// Search for sequences
    async fn search_sequences(&self, query: &SearchQuery) -> Result<Vec<Sequence>, AdapterError>;

    /// Import a structure by external ID
    async fn import_structure(&self, external_id: &str) -> Result<ProteinStructure, AdapterError>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_adapter_type_names() {
        assert_eq!(AdapterType::Ga4gh.name(), "ga4gh");
        assert_eq!(AdapterType::Fhir.name(), "fhir");
        assert_eq!(AdapterType::Mock.name(), "mock");
    }

    #[test]
    fn test_health_status() {
        let healthy = HealthStatus::healthy(50);
        assert!(healthy.healthy);
        assert_eq!(healthy.latency_ms, 50);

        let unhealthy = HealthStatus::unhealthy("Connection failed");
        assert!(!unhealthy.healthy);
        assert!(unhealthy.message.is_some());
    }

    #[test]
    fn test_export_result() {
        let success = ExportResult::success("ext-123")
            .with_url("https://example.com/resource/ext-123")
            .with_metadata("source", "wia-bio");

        assert!(success.success);
        assert_eq!(success.external_id, Some("ext-123".to_string()));
        assert!(success.url.is_some());
        assert!(success.metadata.contains_key("source"));

        let failure = ExportResult::failure("Export failed");
        assert!(!failure.success);
        assert!(failure.message.is_some());
    }

    #[test]
    fn test_search_query() {
        let query = SearchQuery::new("BRCA1")
            .organism("Homo sapiens")
            .limit(50);

        assert_eq!(query.text, Some("BRCA1".to_string()));
        assert_eq!(query.organism, Some("Homo sapiens".to_string()));
        assert_eq!(query.limit, 50);
    }
}
