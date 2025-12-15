//! FHIR Genomics adapter

use async_trait::async_trait;
use serde::{Deserialize, Serialize};

use super::base::*;
use super::config::AdapterConfig;
use crate::types::{Sequence, SequenceType};

/// FHIR version
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FhirVersion {
    /// FHIR R4
    R4,
    /// FHIR R5
    R5,
    /// FHIR R6 (ballot)
    R6,
}

impl FhirVersion {
    /// Get version string
    pub fn version_string(&self) -> &str {
        match self {
            FhirVersion::R4 => "4.0.1",
            FhirVersion::R5 => "5.0.0",
            FhirVersion::R6 => "6.0.0",
        }
    }
}

/// FHIR Genomics adapter
#[derive(Debug)]
pub struct FhirGenomicsAdapter {
    name: String,
    initialized: bool,
    config: Option<AdapterConfig>,
    fhir_version: FhirVersion,
}

impl FhirGenomicsAdapter {
    /// Create a new FHIR adapter
    pub fn new(version: FhirVersion) -> Self {
        Self {
            name: format!("FHIR Genomics {}", version.version_string()),
            initialized: false,
            config: None,
            fhir_version: version,
        }
    }

    /// Get FHIR version
    pub fn fhir_version(&self) -> FhirVersion {
        self.fhir_version
    }

    /// Convert WIA Sequence to FHIR MolecularSequence
    pub fn to_molecular_sequence(&self, sequence: &Sequence) -> FhirMolecularSequence {
        let seq_type = match sequence.sequence_type {
            SequenceType::Dna => "dna",
            SequenceType::Rna | SequenceType::Mrna => "rna",
            SequenceType::Protein => "aa",
            SequenceType::Synthetic => "dna",
        };

        let name = sequence.sequence_info.as_ref()
            .and_then(|i| i.name.clone());

        FhirMolecularSequence {
            resource_type: "MolecularSequence".to_string(),
            id: Some(sequence.sequence_id.clone()),
            r#type: seq_type.to_string(),
            coordinate_system: 0,
            patient: None,
            specimen: None,
            literal: name,
            length: Some(sequence.length_bp as i64),
        }
    }

    /// Convert FHIR MolecularSequence to WIA Sequence
    pub fn from_molecular_sequence(&self, fhir: &FhirMolecularSequence) -> Result<Sequence, AdapterError> {
        let seq_type = match fhir.r#type.as_str() {
            "dna" => SequenceType::Dna,
            "rna" => SequenceType::Rna,
            "aa" => SequenceType::Protein,
            _ => SequenceType::Dna,
        };

        let length = fhir.length.unwrap_or(0) as u64;

        Ok(Sequence {
            schema: Some("https://wia.live/schemas/bio/sequence.schema.json".to_string()),
            sequence_id: fhir.id.clone().unwrap_or_else(|| uuid::Uuid::new_v4().to_string()),
            sequence_info: Some(crate::types::SequenceInfo {
                name: fhir.literal.clone(),
                description: None,
                organism: None,
                taxonomy_id: None,
                gene_symbol: None,
                gene_id: None,
            }),
            sequence_type: seq_type,
            length_bp: length,
            gc_content: None,
            source: None,
            annotations: None,
            files: None,
            checksum: None,
        })
    }
}

impl Default for FhirGenomicsAdapter {
    fn default() -> Self {
        Self::new(FhirVersion::R5)
    }
}

/// FHIR MolecularSequence resource
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FhirMolecularSequence {
    /// Resource type
    pub resource_type: String,
    /// Resource ID
    pub id: Option<String>,
    /// Sequence type (dna, rna, aa)
    pub r#type: String,
    /// Coordinate system (0-based or 1-based)
    pub coordinate_system: i32,
    /// Patient reference
    pub patient: Option<FhirReference>,
    /// Specimen reference
    pub specimen: Option<FhirReference>,
    /// Literal sequence string
    pub literal: Option<String>,
    /// Sequence length
    pub length: Option<i64>,
}

/// FHIR Reference
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirReference {
    /// Reference string
    pub reference: Option<String>,
    /// Display text
    pub display: Option<String>,
}

/// FHIR GenomicStudy resource
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FhirGenomicStudy {
    /// Resource type
    pub resource_type: String,
    /// Resource ID
    pub id: Option<String>,
    /// Status
    pub status: String,
    /// Subject reference
    pub subject: Option<FhirReference>,
    /// Description
    pub description: Option<String>,
    /// Analysis
    pub analysis: Option<Vec<FhirGenomicAnalysis>>,
}

/// FHIR Genomic Analysis
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FhirGenomicAnalysis {
    /// Identifier
    pub identifier: Option<Vec<FhirIdentifier>>,
    /// Method type
    pub method_type: Option<Vec<FhirCodeableConcept>>,
    /// Date
    pub date: Option<String>,
}

/// FHIR Identifier
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirIdentifier {
    /// System
    pub system: Option<String>,
    /// Value
    pub value: Option<String>,
}

/// FHIR CodeableConcept
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirCodeableConcept {
    /// Coding
    pub coding: Option<Vec<FhirCoding>>,
    /// Text
    pub text: Option<String>,
}

/// FHIR Coding
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirCoding {
    /// System
    pub system: Option<String>,
    /// Code
    pub code: Option<String>,
    /// Display
    pub display: Option<String>,
}

#[async_trait]
impl IEcosystemAdapter for FhirGenomicsAdapter {
    fn adapter_type(&self) -> AdapterType {
        AdapterType::Fhir
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn initialize(&mut self, config: &AdapterConfig) -> Result<(), AdapterError> {
        if config.base_url.is_empty() {
            return Err(AdapterError::ConfigurationError("Base URL is required".to_string()));
        }
        self.config = Some(config.clone());
        self.initialized = true;
        Ok(())
    }

    async fn health_check(&self) -> Result<HealthStatus, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        // In production, would GET /metadata
        Ok(HealthStatus::healthy(50))
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

impl FhirGenomicsAdapter {
    /// Create a FHIR resource
    pub async fn create_resource<T: Serialize>(&self, _resource: &T) -> Result<String, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        // In production, would POST to /{resourceType}
        Err(AdapterError::NotImplemented("FHIR create not yet implemented".to_string()))
    }

    /// Get a FHIR resource
    pub async fn get_resource(&self, _resource_type: &str, _id: &str) -> Result<serde_json::Value, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        // In production, would GET /{resourceType}/{id}
        Err(AdapterError::NotImplemented("FHIR get not yet implemented".to_string()))
    }

    /// Search FHIR resources
    pub async fn search(&self, _resource_type: &str, _params: &[(&str, &str)]) -> Result<Vec<serde_json::Value>, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        // In production, would GET /{resourceType}?params
        Err(AdapterError::NotImplemented("FHIR search not yet implemented".to_string()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::create_sequence;

    #[tokio::test]
    async fn test_fhir_adapter_initialize() {
        let mut adapter = FhirGenomicsAdapter::new(FhirVersion::R5);
        let config = AdapterConfig::new("https://fhir.example.org");

        adapter.initialize(&config).await.unwrap();
        assert!(adapter.is_initialized());
        assert_eq!(adapter.fhir_version(), FhirVersion::R5);
    }

    #[test]
    fn test_sequence_to_molecular_sequence() {
        let adapter = FhirGenomicsAdapter::new(FhirVersion::R5);
        let sequence = create_sequence("BRCA1", "ATCGATCG", SequenceType::Dna).unwrap();

        let fhir = adapter.to_molecular_sequence(&sequence);

        assert_eq!(fhir.resource_type, "MolecularSequence");
        assert_eq!(fhir.r#type, "dna");
        assert_eq!(fhir.length, Some(8));
    }

    #[test]
    fn test_molecular_sequence_to_sequence() {
        let adapter = FhirGenomicsAdapter::new(FhirVersion::R5);
        let fhir = FhirMolecularSequence {
            resource_type: "MolecularSequence".to_string(),
            id: Some("test-123".to_string()),
            r#type: "dna".to_string(),
            coordinate_system: 0,
            patient: None,
            specimen: None,
            literal: Some("Test Gene".to_string()),
            length: Some(100),
        };

        let sequence = adapter.from_molecular_sequence(&fhir).unwrap();

        assert_eq!(sequence.sequence_id, "test-123");
        assert_eq!(sequence.sequence_type, SequenceType::Dna);
        assert_eq!(sequence.length_bp, 100);
    }

    #[test]
    fn test_fhir_version_string() {
        assert_eq!(FhirVersion::R4.version_string(), "4.0.1");
        assert_eq!(FhirVersion::R5.version_string(), "5.0.0");
        assert_eq!(FhirVersion::R6.version_string(), "6.0.0");
    }
}
