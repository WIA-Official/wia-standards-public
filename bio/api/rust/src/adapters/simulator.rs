//! WIA Biotech Simulator Adapter
//!
//! This module provides a simulator for testing biotechnology operations
//! without requiring actual laboratory equipment or databases.

use crate::error::{BioError, BioResult};
use crate::types::*;
use crate::core::*;
use async_trait::async_trait;
use std::collections::HashMap;
use std::sync::{Arc, RwLock};

/// Trait for biotech data providers
#[async_trait]
pub trait BioProvider: Send + Sync {
    /// Get a sequence by ID
    async fn get_sequence(&self, id: &str) -> BioResult<Sequence>;

    /// Store a sequence
    async fn store_sequence(&self, sequence: Sequence) -> BioResult<String>;

    /// Get a CRISPR experiment by ID
    async fn get_experiment(&self, id: &str) -> BioResult<CrisprExperiment>;

    /// Store a CRISPR experiment
    async fn store_experiment(&self, experiment: CrisprExperiment) -> BioResult<String>;

    /// Get a protein structure by ID
    async fn get_structure(&self, id: &str) -> BioResult<ProteinStructure>;

    /// Store a protein structure
    async fn store_structure(&self, structure: ProteinStructure) -> BioResult<String>;

    /// Get a synthetic biology part by ID
    async fn get_part(&self, id: &str) -> BioResult<BioPart>;

    /// Store a synthetic biology part
    async fn store_part(&self, part: BioPart) -> BioResult<String>;

    /// List all sequences
    async fn list_sequences(&self) -> BioResult<Vec<String>>;

    /// List all experiments
    async fn list_experiments(&self) -> BioResult<Vec<String>>;

    /// List all structures
    async fn list_structures(&self) -> BioResult<Vec<String>>;

    /// List all parts
    async fn list_parts(&self) -> BioResult<Vec<String>>;
}

/// In-memory simulator for testing
pub struct SimulatorAdapter {
    sequences: Arc<RwLock<HashMap<String, Sequence>>>,
    experiments: Arc<RwLock<HashMap<String, CrisprExperiment>>>,
    structures: Arc<RwLock<HashMap<String, ProteinStructure>>>,
    parts: Arc<RwLock<HashMap<String, BioPart>>>,
}

impl SimulatorAdapter {
    /// Create a new simulator adapter
    pub fn new() -> Self {
        Self {
            sequences: Arc::new(RwLock::new(HashMap::new())),
            experiments: Arc::new(RwLock::new(HashMap::new())),
            structures: Arc::new(RwLock::new(HashMap::new())),
            parts: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Create a simulator with sample data
    pub fn with_sample_data() -> BioResult<Self> {
        let adapter = Self::new();

        // Add sample sequence
        let sample_seq = create_sequence(
            "Sample Gene ABC",
            "ATGCGATCGATCGATCGATCGATCGATCGATCGATCGATCGATCGATCGATCG",
            SequenceType::Dna,
        )?;
        {
            let mut sequences = adapter.sequences.write().unwrap();
            sequences.insert(sample_seq.sequence_id.clone(), sample_seq);
        }

        // Add sample part
        let sample_part = create_biopart(
            "Strong Promoter",
            PartType::Promoter,
            "TTGACGGCTAGCTCAGTCCTAGGTACAGTGCTAGC",
        )?;
        {
            let mut parts = adapter.parts.write().unwrap();
            parts.insert(sample_part.part_id.clone(), sample_part);
        }

        // Add sample structure
        let sample_struct = create_protein_structure(
            "Sample Protein",
            Some("P12345"),
            PredictionMethod::Alphafold,
            500,
        );
        {
            let mut structures = adapter.structures.write().unwrap();
            structures.insert(sample_struct.structure_id.clone(), sample_struct);
        }

        Ok(adapter)
    }

    /// Simulate CRISPR editing results
    pub fn simulate_editing_results(&self, experiment_id: &str) -> BioResult<CrisprResults> {
        // Check if experiment exists
        {
            let experiments = self.experiments.read().unwrap();
            if !experiments.contains_key(experiment_id) {
                return Err(BioError::not_found(format!(
                    "Experiment {} not found",
                    experiment_id
                )));
            }
        }

        // Generate simulated results
        let indel_profile = vec![
            IndelVariant {
                indel_type: IndelType::Deletion,
                size_bp: -1,
                frequency: 0.35,
                sequence: None,
            },
            IndelVariant {
                indel_type: IndelType::Insertion,
                size_bp: 1,
                frequency: 0.25,
                sequence: None,
            },
            IndelVariant {
                indel_type: IndelType::Deletion,
                size_bp: -5,
                frequency: 0.15,
                sequence: None,
            },
        ];

        Ok(CrisprResults {
            schema: Some("https://wia.live/schemas/bio/crispr-results.schema.json".to_string()),
            results_id: format!(
                "results-{}",
                uuid::Uuid::new_v4().to_string().split('-').next().unwrap()
            ),
            experiment_ref: experiment_id.to_string(),
            analysis_date: Some(chrono::Utc::now().date_naive()),
            analysis_method: Some(AnalysisMethod::Ngs),
            editing_efficiency: Some(calculate_editing_efficiency(0.75, None)),
            indel_profile: Some(indel_profile),
            off_target_analysis: Some(OffTargetAnalysis {
                method: Some(OffTargetMethod::Computational),
                sites_detected: 2,
                sites: Some(vec![
                    OffTargetSite {
                        chromosome: Some("chr3".to_string()),
                        position: Some(12345678),
                        mismatches: Some(3),
                        frequency: Some(0.002),
                        gene: None,
                    },
                ]),
            }),
            quality_metrics: Some(QualityMetrics {
                sequencing_depth: Some(10000),
                mapping_rate: Some(0.98),
                confidence_score: Some(0.95),
            }),
            files: None,
        })
    }

    /// Simulate AlphaFold prediction
    pub fn simulate_alphafold_prediction(
        &self,
        protein_name: &str,
        length: usize,
    ) -> (ProteinStructure, StructureConfidence) {
        use rand::Rng;

        // Generate simulated pLDDT scores
        let mut rng = rand::thread_rng();
        let plddt_values: Vec<f64> = (0..length)
            .map(|i| {
                // Simulate typical AlphaFold behavior:
                // - High confidence in core
                // - Lower at termini and loops
                let base = if i < 20 || i > length - 20 {
                    60.0 // Termini
                } else if i % 30 < 5 {
                    55.0 // Loops
                } else {
                    85.0 // Core
                };
                (base + rng.gen_range(-10.0..10.0) as f64).clamp(0.0, 100.0)
            })
            .collect();

        let mean_plddt = calculate_mean_plddt(&plddt_values);
        let confident_pct = plddt_values.iter().filter(|&&s| s > 70.0).count() as f64
            / plddt_values.len() as f64;
        let disordered_pct = plddt_values.iter().filter(|&&s| s < 50.0).count() as f64
            / plddt_values.len() as f64;

        let mut structure = create_protein_structure(
            protein_name,
            None,
            PredictionMethod::Alphafold,
            length as u64,
        );

        structure.quality = Some(StructureQuality {
            mean_plddt: Some(mean_plddt),
            confident_residues_pct: Some(confident_pct),
            disordered_residues_pct: Some(disordered_pct),
            resolution_angstrom: None,
        });

        let interpretation = identify_confidence_regions(&plddt_values, 70.0, 50.0);

        let confidence = StructureConfidence {
            schema: Some(
                "https://wia.live/schemas/bio/structure-confidence.schema.json".to_string(),
            ),
            structure_ref: structure.structure_id.clone(),
            plddt: Some(PlddtData {
                description: Some("predicted Local Distance Difference Test".to_string()),
                range: Some((0.0, 100.0)),
                values: plddt_values,
            }),
            pae: None, // PAE matrix would be too large for simulation
            interpretation: Some(interpretation),
        };

        (structure, confidence)
    }
}

impl Default for SimulatorAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl BioProvider for SimulatorAdapter {
    async fn get_sequence(&self, id: &str) -> BioResult<Sequence> {
        let sequences = self.sequences.read().unwrap();
        sequences
            .get(id)
            .cloned()
            .ok_or_else(|| BioError::not_found(format!("Sequence {} not found", id)))
    }

    async fn store_sequence(&self, sequence: Sequence) -> BioResult<String> {
        let id = sequence.sequence_id.clone();
        let mut sequences = self.sequences.write().unwrap();
        sequences.insert(id.clone(), sequence);
        Ok(id)
    }

    async fn get_experiment(&self, id: &str) -> BioResult<CrisprExperiment> {
        let experiments = self.experiments.read().unwrap();
        experiments
            .get(id)
            .cloned()
            .ok_or_else(|| BioError::not_found(format!("Experiment {} not found", id)))
    }

    async fn store_experiment(&self, experiment: CrisprExperiment) -> BioResult<String> {
        let id = experiment.experiment_id.clone();
        let mut experiments = self.experiments.write().unwrap();
        experiments.insert(id.clone(), experiment);
        Ok(id)
    }

    async fn get_structure(&self, id: &str) -> BioResult<ProteinStructure> {
        let structures = self.structures.read().unwrap();
        structures
            .get(id)
            .cloned()
            .ok_or_else(|| BioError::not_found(format!("Structure {} not found", id)))
    }

    async fn store_structure(&self, structure: ProteinStructure) -> BioResult<String> {
        let id = structure.structure_id.clone();
        let mut structures = self.structures.write().unwrap();
        structures.insert(id.clone(), structure);
        Ok(id)
    }

    async fn get_part(&self, id: &str) -> BioResult<BioPart> {
        let parts = self.parts.read().unwrap();
        parts
            .get(id)
            .cloned()
            .ok_or_else(|| BioError::not_found(format!("Part {} not found", id)))
    }

    async fn store_part(&self, part: BioPart) -> BioResult<String> {
        let id = part.part_id.clone();
        let mut parts = self.parts.write().unwrap();
        parts.insert(id.clone(), part);
        Ok(id)
    }

    async fn list_sequences(&self) -> BioResult<Vec<String>> {
        let sequences = self.sequences.read().unwrap();
        Ok(sequences.keys().cloned().collect())
    }

    async fn list_experiments(&self) -> BioResult<Vec<String>> {
        let experiments = self.experiments.read().unwrap();
        Ok(experiments.keys().cloned().collect())
    }

    async fn list_structures(&self) -> BioResult<Vec<String>> {
        let structures = self.structures.read().unwrap();
        Ok(structures.keys().cloned().collect())
    }

    async fn list_parts(&self) -> BioResult<Vec<String>> {
        let parts = self.parts.read().unwrap();
        Ok(parts.keys().cloned().collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_simulator_sequence_operations() {
        let adapter = SimulatorAdapter::new();

        // Create and store a sequence
        let seq = create_sequence("Test Gene", "ATCGATCG", SequenceType::Dna).unwrap();
        let id = adapter.store_sequence(seq.clone()).await.unwrap();

        // Retrieve the sequence
        let retrieved = adapter.get_sequence(&id).await.unwrap();
        assert_eq!(retrieved.sequence_id, seq.sequence_id);
    }

    #[tokio::test]
    async fn test_simulator_with_sample_data() {
        let adapter = SimulatorAdapter::with_sample_data().unwrap();

        let sequences = adapter.list_sequences().await.unwrap();
        assert!(!sequences.is_empty());

        let parts = adapter.list_parts().await.unwrap();
        assert!(!parts.is_empty());
    }

    #[test]
    fn test_alphafold_simulation() {
        let adapter = SimulatorAdapter::new();
        let (structure, confidence) = adapter.simulate_alphafold_prediction("Test Protein", 100);

        assert!(structure.quality.is_some());
        assert!(confidence.plddt.is_some());

        let plddt = confidence.plddt.unwrap();
        assert_eq!(plddt.values.len(), 100);
    }
}
