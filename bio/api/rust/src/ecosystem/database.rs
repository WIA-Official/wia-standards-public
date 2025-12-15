//! Bioinformatics database adapters

use async_trait::async_trait;
use serde::{Deserialize, Serialize};

use super::base::*;
use super::config::AdapterConfig;
use crate::types::{Sequence, SequenceType, ProteinStructure};

/// NCBI database types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NcbiDatabase {
    /// Nucleotide sequences
    Nucleotide,
    /// Protein sequences
    Protein,
    /// Gene records
    Gene,
    /// PubMed articles
    Pubmed,
    /// Structures
    Structure,
}

impl NcbiDatabase {
    /// Get database name for E-utilities
    pub fn db_name(&self) -> &str {
        match self {
            NcbiDatabase::Nucleotide => "nucleotide",
            NcbiDatabase::Protein => "protein",
            NcbiDatabase::Gene => "gene",
            NcbiDatabase::Pubmed => "pubmed",
            NcbiDatabase::Structure => "structure",
        }
    }
}

/// NCBI E-utilities adapter
#[derive(Debug)]
pub struct NcbiAdapter {
    name: String,
    initialized: bool,
    config: Option<AdapterConfig>,
}

impl NcbiAdapter {
    /// Create a new NCBI adapter
    pub fn new() -> Self {
        Self {
            name: "NCBI E-utilities".to_string(),
            initialized: false,
            config: None,
        }
    }

    /// Default base URL for NCBI E-utilities
    pub const BASE_URL: &'static str = "https://eutils.ncbi.nlm.nih.gov/entrez/eutils";
}

impl Default for NcbiAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl IEcosystemAdapter for NcbiAdapter {
    fn adapter_type(&self) -> AdapterType {
        AdapterType::Database
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn initialize(&mut self, config: &AdapterConfig) -> Result<(), AdapterError> {
        self.config = Some(if config.base_url.is_empty() {
            AdapterConfig::new(Self::BASE_URL)
        } else {
            config.clone()
        });
        self.initialized = true;
        Ok(())
    }

    async fn health_check(&self) -> Result<HealthStatus, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        // Would check /einfo.fcgi in production
        Ok(HealthStatus::healthy(100))
    }

    async fn shutdown(&mut self) -> Result<(), AdapterError> {
        self.initialized = false;
        self.config = None;
        Ok(())
    }

    fn is_initialized(&self) -> bool {
        self.initialized
    }
}

impl NcbiAdapter {
    /// Search NCBI database
    pub async fn search(&self, _database: NcbiDatabase, _query: &str) -> Result<Vec<String>, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        // Would call esearch.fcgi in production
        Err(AdapterError::NotImplemented("NCBI search not yet implemented".to_string()))
    }

    /// Fetch records by ID
    pub async fn fetch(&self, _database: NcbiDatabase, _ids: &[&str]) -> Result<String, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        // Would call efetch.fcgi in production
        Err(AdapterError::NotImplemented("NCBI fetch not yet implemented".to_string()))
    }

    /// Get GenBank sequence
    pub async fn get_genbank_sequence(&self, _accession: &str) -> Result<Sequence, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        Err(AdapterError::NotImplemented("NCBI GenBank not yet implemented".to_string()))
    }
}

/// UniProt adapter
#[derive(Debug)]
pub struct UniProtAdapter {
    name: String,
    initialized: bool,
    config: Option<AdapterConfig>,
}

impl UniProtAdapter {
    /// Create a new UniProt adapter
    pub fn new() -> Self {
        Self {
            name: "UniProt".to_string(),
            initialized: false,
            config: None,
        }
    }

    /// Default base URL for UniProt REST API
    pub const BASE_URL: &'static str = "https://rest.uniprot.org";
}

impl Default for UniProtAdapter {
    fn default() -> Self {
        Self::new()
    }
}

/// UniProt entry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UniProtEntry {
    /// Accession number
    pub accession: String,
    /// Entry name
    pub name: String,
    /// Protein name
    pub protein_name: String,
    /// Organism
    pub organism: String,
    /// Amino acid sequence
    pub sequence: String,
    /// Sequence length
    pub length: u32,
    /// Features
    pub features: Vec<UniProtFeature>,
}

/// UniProt feature
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UniProtFeature {
    /// Feature type
    pub feature_type: String,
    /// Start position
    pub start: u32,
    /// End position
    pub end: u32,
    /// Description
    pub description: Option<String>,
}

/// UniProt variant
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UniProtVariant {
    /// Position
    pub position: u32,
    /// Original amino acid
    pub original: String,
    /// Variant amino acid
    pub variant: String,
    /// Clinical significance
    pub clinical_significance: Option<String>,
    /// dbSNP ID
    pub dbsnp_id: Option<String>,
}

#[async_trait]
impl IEcosystemAdapter for UniProtAdapter {
    fn adapter_type(&self) -> AdapterType {
        AdapterType::Database
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn initialize(&mut self, config: &AdapterConfig) -> Result<(), AdapterError> {
        self.config = Some(if config.base_url.is_empty() {
            AdapterConfig::new(Self::BASE_URL)
        } else {
            config.clone()
        });
        self.initialized = true;
        Ok(())
    }

    async fn health_check(&self) -> Result<HealthStatus, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        Ok(HealthStatus::healthy(100))
    }

    async fn shutdown(&mut self) -> Result<(), AdapterError> {
        self.initialized = false;
        self.config = None;
        Ok(())
    }

    fn is_initialized(&self) -> bool {
        self.initialized
    }
}

impl UniProtAdapter {
    /// Get protein by accession
    pub async fn get_protein(&self, _accession: &str) -> Result<UniProtEntry, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        // Would GET /uniprotkb/{accession} in production
        Err(AdapterError::NotImplemented("UniProt get not yet implemented".to_string()))
    }

    /// Search UniProt
    pub async fn search(&self, _query: &str, _limit: u32) -> Result<Vec<UniProtEntry>, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        // Would GET /uniprotkb/search in production
        Err(AdapterError::NotImplemented("UniProt search not yet implemented".to_string()))
    }

    /// Convert UniProt entry to WIA Sequence
    pub fn to_sequence(&self, entry: &UniProtEntry) -> Sequence {
        Sequence {
            schema: Some("https://wia.live/schemas/bio/sequence.schema.json".to_string()),
            sequence_id: format!("uniprot-{}", entry.accession),
            sequence_info: Some(crate::types::SequenceInfo {
                name: Some(entry.protein_name.clone()),
                description: None,
                organism: Some(entry.organism.clone()),
                taxonomy_id: None,
                gene_symbol: None,
                gene_id: None,
            }),
            sequence_type: SequenceType::Protein,
            length_bp: entry.length as u64,
            gc_content: None,
            source: Some(crate::types::SequenceSource {
                database: Some("UniProt".to_string()),
                accession: Some(entry.accession.clone()),
                version: None,
                retrieved_date: None,
            }),
            annotations: None,
            files: None,
            checksum: None,
        }
    }

    /// Get variants for a protein
    pub async fn get_variants(&self, _accession: &str) -> Result<Vec<UniProtVariant>, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        Err(AdapterError::NotImplemented("UniProt variants not yet implemented".to_string()))
    }
}

/// EBI Proteins API adapter
#[derive(Debug)]
pub struct EbiProteinsAdapter {
    name: String,
    initialized: bool,
    config: Option<AdapterConfig>,
}

impl EbiProteinsAdapter {
    /// Create a new EBI Proteins adapter
    pub fn new() -> Self {
        Self {
            name: "EBI Proteins API".to_string(),
            initialized: false,
            config: None,
        }
    }

    /// Default base URL
    pub const BASE_URL: &'static str = "https://www.ebi.ac.uk/proteins/api";
}

impl Default for EbiProteinsAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl IEcosystemAdapter for EbiProteinsAdapter {
    fn adapter_type(&self) -> AdapterType {
        AdapterType::Database
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn initialize(&mut self, config: &AdapterConfig) -> Result<(), AdapterError> {
        self.config = Some(if config.base_url.is_empty() {
            AdapterConfig::new(Self::BASE_URL)
        } else {
            config.clone()
        });
        self.initialized = true;
        Ok(())
    }

    async fn health_check(&self) -> Result<HealthStatus, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        Ok(HealthStatus::healthy(100))
    }

    async fn shutdown(&mut self) -> Result<(), AdapterError> {
        self.initialized = false;
        self.config = None;
        Ok(())
    }

    fn is_initialized(&self) -> bool {
        self.initialized
    }
}

impl EbiProteinsAdapter {
    /// Get protein features
    pub async fn get_features(&self, _accession: &str) -> Result<Vec<UniProtFeature>, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        Err(AdapterError::NotImplemented("EBI features not yet implemented".to_string()))
    }

    /// Get protein variation
    pub async fn get_variation(&self, _accession: &str) -> Result<Vec<UniProtVariant>, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        Err(AdapterError::NotImplemented("EBI variation not yet implemented".to_string()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_ncbi_adapter_initialize() {
        let mut adapter = NcbiAdapter::new();
        let config = AdapterConfig::default();

        adapter.initialize(&config).await.unwrap();
        assert!(adapter.is_initialized());
    }

    #[tokio::test]
    async fn test_uniprot_adapter_initialize() {
        let mut adapter = UniProtAdapter::new();
        let config = AdapterConfig::default();

        adapter.initialize(&config).await.unwrap();
        assert!(adapter.is_initialized());
    }

    #[test]
    fn test_ncbi_database_names() {
        assert_eq!(NcbiDatabase::Nucleotide.db_name(), "nucleotide");
        assert_eq!(NcbiDatabase::Protein.db_name(), "protein");
        assert_eq!(NcbiDatabase::Gene.db_name(), "gene");
    }

    #[test]
    fn test_uniprot_to_sequence() {
        let adapter = UniProtAdapter::new();
        let entry = UniProtEntry {
            accession: "P53350".to_string(),
            name: "PLK1_HUMAN".to_string(),
            protein_name: "Polo-like kinase 1".to_string(),
            organism: "Homo sapiens".to_string(),
            sequence: "MEPG...".to_string(),
            length: 603,
            features: vec![],
        };

        let sequence = adapter.to_sequence(&entry);

        assert!(sequence.sequence_id.contains("P53350"));
        assert_eq!(sequence.sequence_type, SequenceType::Protein);
        assert_eq!(sequence.length_bp, 603);
    }

    #[tokio::test]
    async fn test_ebi_adapter_initialize() {
        let mut adapter = EbiProteinsAdapter::new();
        let config = AdapterConfig::default();

        adapter.initialize(&config).await.unwrap();
        assert!(adapter.is_initialized());
    }
}
