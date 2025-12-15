//! WIA Biotech Core Implementation
//!
//! This module provides core functionality for biotechnology data processing,
//! including sequence analysis, CRISPR design, and structure handling.

use crate::error::{BioError, BioResult};
use crate::types::*;
use chrono::{NaiveDate, Utc};
use std::collections::HashMap;
use uuid::Uuid;

// ============================================================================
// Sequence Operations
// ============================================================================

/// Calculate GC content of a DNA/RNA sequence
pub fn calculate_gc_content(sequence: &str) -> f64 {
    let sequence = sequence.to_uppercase();
    let total = sequence.len() as f64;
    if total == 0.0 {
        return 0.0;
    }

    let gc_count = sequence
        .chars()
        .filter(|c| *c == 'G' || *c == 'C')
        .count() as f64;

    gc_count / total
}

/// Validate DNA sequence (only A, T, C, G allowed)
pub fn validate_dna_sequence(sequence: &str) -> BioResult<()> {
    let sequence = sequence.to_uppercase();
    for (i, c) in sequence.chars().enumerate() {
        if !matches!(c, 'A' | 'T' | 'C' | 'G') {
            return Err(BioError::sequence(format!(
                "Invalid character '{}' at position {}",
                c, i
            )));
        }
    }
    Ok(())
}

/// Validate RNA sequence (only A, U, C, G allowed)
pub fn validate_rna_sequence(sequence: &str) -> BioResult<()> {
    let sequence = sequence.to_uppercase();
    for (i, c) in sequence.chars().enumerate() {
        if !matches!(c, 'A' | 'U' | 'C' | 'G') {
            return Err(BioError::sequence(format!(
                "Invalid character '{}' at position {}",
                c, i
            )));
        }
    }
    Ok(())
}

/// Get reverse complement of DNA sequence
pub fn reverse_complement(sequence: &str) -> String {
    sequence
        .chars()
        .rev()
        .map(|c| match c.to_ascii_uppercase() {
            'A' => 'T',
            'T' => 'A',
            'C' => 'G',
            'G' => 'C',
            other => other,
        })
        .collect()
}

/// Transcribe DNA to RNA
pub fn transcribe_dna_to_rna(dna: &str) -> String {
    dna.chars()
        .map(|c| match c.to_ascii_uppercase() {
            'T' => 'U',
            other => other.to_ascii_uppercase(),
        })
        .collect()
}

/// Create a new Sequence with generated ID
pub fn create_sequence(
    name: &str,
    sequence_data: &str,
    sequence_type: SequenceType,
) -> BioResult<Sequence> {
    // Validate based on type
    match sequence_type {
        SequenceType::Dna | SequenceType::Synthetic => validate_dna_sequence(sequence_data)?,
        SequenceType::Rna | SequenceType::Mrna => validate_rna_sequence(sequence_data)?,
        SequenceType::Protein => {} // Protein validation could be added
    }

    let gc_content = match sequence_type {
        SequenceType::Dna | SequenceType::Rna | SequenceType::Mrna | SequenceType::Synthetic => {
            Some(calculate_gc_content(sequence_data))
        }
        SequenceType::Protein => None,
    };

    Ok(Sequence {
        schema: Some("https://wia.live/schemas/bio/sequence.schema.json".to_string()),
        sequence_id: format!("seq-{}", Uuid::new_v4().to_string().split('-').next().unwrap()),
        sequence_info: Some(SequenceInfo {
            name: Some(name.to_string()),
            description: None,
            organism: None,
            taxonomy_id: None,
            gene_symbol: None,
            gene_id: None,
        }),
        sequence_type,
        length_bp: sequence_data.len() as u64,
        gc_content,
        source: None,
        annotations: None,
        files: None,
        checksum: None,
    })
}

// ============================================================================
// CRISPR Operations
// ============================================================================

/// Find PAM sites in a sequence for SpCas9 (NGG)
pub fn find_pam_sites_ngg(sequence: &str) -> Vec<(usize, Strand)> {
    let sequence = sequence.to_uppercase();
    let mut sites = Vec::new();

    // Forward strand (NGG)
    for i in 0..sequence.len().saturating_sub(2) {
        if &sequence[i + 1..i + 3] == "GG" {
            sites.push((i, Strand::Plus));
        }
    }

    // Reverse strand (CCN)
    for i in 0..sequence.len().saturating_sub(2) {
        if &sequence[i..i + 2] == "CC" {
            sites.push((i + 2, Strand::Minus));
        }
    }

    sites
}

/// Design guide RNA for a target sequence
pub fn design_guide_rna(
    target_sequence: &str,
    pam_position: usize,
    strand: Strand,
    grna_length: usize,
) -> BioResult<GuideRna> {
    let target_sequence = target_sequence.to_uppercase();

    if grna_length < 17 || grna_length > 25 {
        return Err(BioError::crispr("gRNA length should be 17-25 bp"));
    }

    let (grna_seq, target_start, target_end, pam) = match strand {
        Strand::Plus => {
            if pam_position < grna_length {
                return Err(BioError::crispr("PAM position too close to sequence start"));
            }
            let start = pam_position - grna_length;
            let grna = &target_sequence[start..pam_position];
            let pam = &target_sequence[pam_position..pam_position + 3];
            (grna.to_string(), start, pam_position, pam.to_string())
        }
        Strand::Minus => {
            let end = pam_position + grna_length;
            if end > target_sequence.len() {
                return Err(BioError::crispr("PAM position too close to sequence end"));
            }
            let grna = &target_sequence[pam_position..end];
            let grna = reverse_complement(grna);
            let pam_start = pam_position.saturating_sub(3);
            let pam = reverse_complement(&target_sequence[pam_start..pam_position]);
            (grna, pam_position, end, pam)
        }
    };

    // Convert DNA to RNA (T -> U)
    let grna_rna = transcribe_dna_to_rna(&grna_seq);

    Ok(GuideRna {
        grna_id: format!("grna-{}", Uuid::new_v4().to_string().split('-').next().unwrap()),
        name: None,
        sequence: grna_rna,
        target_start: Some(target_start as u64 + 1), // 1-based
        target_end: Some(target_end as u64),
        strand: Some(strand),
        pam: Some(pam),
        off_target_score: None,
        on_target_score: None,
    })
}

/// Create a new CRISPR experiment
pub fn create_crispr_experiment(
    name: &str,
    gene_symbol: &str,
    system_type: CrisprSystemType,
    guide_rnas: Vec<GuideRna>,
) -> BioResult<CrisprExperiment> {
    if guide_rnas.is_empty() {
        return Err(BioError::crispr("At least one guide RNA is required"));
    }

    Ok(CrisprExperiment {
        schema: Some("https://wia.live/schemas/bio/crispr-experiment.schema.json".to_string()),
        experiment_id: format!(
            "crispr-exp-{}",
            Uuid::new_v4().to_string().split('-').next().unwrap()
        ),
        experiment_info: Some(ExperimentInfo {
            name: Some(name.to_string()),
            description: None,
            date: Some(Utc::now().date_naive()),
            experimenter: None,
        }),
        target: CrisprTarget {
            gene_symbol: gene_symbol.to_string(),
            gene_id: None,
            organism: None,
            cell_line: None,
            cell_type: None,
            sequence_ref: None,
        },
        editing_system: EditingSystem {
            system_type: system_type.clone(),
            cas_variant: None,
            pam_sequence: Some(get_pam_for_system(&system_type)),
            delivery_method: None,
        },
        guide_rnas,
        editing_type: Some(EditingType::Knockout),
        repair_template: None,
        protocol: None,
        results_ref: None,
    })
}

/// Get PAM sequence for a CRISPR system
fn get_pam_for_system(system: &CrisprSystemType) -> String {
    match system {
        CrisprSystemType::CrisprCas9 => "NGG".to_string(),
        CrisprSystemType::CrisprSaCas9 => "NNGRRT".to_string(),
        CrisprSystemType::CrisprCas12a => "TTTV".to_string(),
        CrisprSystemType::CrisprCas13 => "N/A".to_string(),
        CrisprSystemType::BaseEditor => "NGG".to_string(),
        CrisprSystemType::PrimeEditor => "NGG".to_string(),
    }
}

/// Calculate editing efficiency from results
pub fn calculate_editing_efficiency(
    indel_frequency: f64,
    hdr_frequency: Option<f64>,
) -> EditingEfficiency {
    let hdr = hdr_frequency.unwrap_or(0.0);
    let wild_type = 1.0 - indel_frequency - hdr;

    EditingEfficiency {
        overall_efficiency: Some(indel_frequency + hdr),
        indel_frequency: Some(indel_frequency),
        hdr_frequency,
        wild_type_frequency: Some(wild_type.max(0.0)),
    }
}

// ============================================================================
// Protein Structure Operations
// ============================================================================

/// Interpret pLDDT score
pub fn interpret_plddt(score: f64) -> &'static str {
    if score > 90.0 {
        "Very high confidence - suitable for binding site analysis"
    } else if score > 70.0 {
        "High confidence - reliable backbone prediction"
    } else if score > 50.0 {
        "Low confidence - treat with caution"
    } else {
        "Very low confidence - likely disordered region"
    }
}

/// Calculate mean pLDDT from residue scores
pub fn calculate_mean_plddt(scores: &[f64]) -> f64 {
    if scores.is_empty() {
        return 0.0;
    }
    scores.iter().sum::<f64>() / scores.len() as f64
}

/// Identify high/low confidence regions from pLDDT scores
pub fn identify_confidence_regions(
    scores: &[f64],
    high_threshold: f64,
    low_threshold: f64,
) -> ConfidenceInterpretation {
    let mut high_regions = Vec::new();
    let mut low_regions = Vec::new();

    let mut current_high_start: Option<usize> = None;
    let mut current_low_start: Option<usize> = None;

    for (i, &score) in scores.iter().enumerate() {
        // High confidence regions
        if score >= high_threshold {
            if current_high_start.is_none() {
                current_high_start = Some(i);
            }
        } else if let Some(start) = current_high_start {
            let region_scores = &scores[start..i];
            high_regions.push(ConfidenceRegion {
                start: (start + 1) as u64,
                end: i as u64,
                mean_plddt: calculate_mean_plddt(region_scores),
                likely_disordered: Some(false),
            });
            current_high_start = None;
        }

        // Low confidence regions
        if score < low_threshold {
            if current_low_start.is_none() {
                current_low_start = Some(i);
            }
        } else if let Some(start) = current_low_start {
            let region_scores = &scores[start..i];
            low_regions.push(ConfidenceRegion {
                start: (start + 1) as u64,
                end: i as u64,
                mean_plddt: calculate_mean_plddt(region_scores),
                likely_disordered: Some(true),
            });
            current_low_start = None;
        }
    }

    // Close any open regions
    if let Some(start) = current_high_start {
        let region_scores = &scores[start..];
        high_regions.push(ConfidenceRegion {
            start: (start + 1) as u64,
            end: scores.len() as u64,
            mean_plddt: calculate_mean_plddt(region_scores),
            likely_disordered: Some(false),
        });
    }

    if let Some(start) = current_low_start {
        let region_scores = &scores[start..];
        low_regions.push(ConfidenceRegion {
            start: (start + 1) as u64,
            end: scores.len() as u64,
            mean_plddt: calculate_mean_plddt(region_scores),
            likely_disordered: Some(true),
        });
    }

    ConfidenceInterpretation {
        high_confidence_regions: if high_regions.is_empty() {
            None
        } else {
            Some(high_regions)
        },
        low_confidence_regions: if low_regions.is_empty() {
            None
        } else {
            Some(low_regions)
        },
    }
}

/// Create a protein structure record
pub fn create_protein_structure(
    name: &str,
    uniprot_id: Option<&str>,
    method: PredictionMethod,
    length_aa: u64,
) -> ProteinStructure {
    ProteinStructure {
        schema: Some("https://wia.live/schemas/bio/structure.schema.json".to_string()),
        structure_id: format!(
            "struct-{}",
            Uuid::new_v4().to_string().split('-').next().unwrap()
        ),
        structure_info: Some(StructureInfo {
            name: Some(name.to_string()),
            description: None,
            uniprot_id: uniprot_id.map(|s| s.to_string()),
            pdb_id: None,
        }),
        prediction: PredictionInfo {
            method,
            version: None,
            date: Some(Utc::now().date_naive()),
            source: Some(PredictionSource::LocalPrediction),
        },
        protein: ProteinInfo {
            name: name.to_string(),
            organism: None,
            length_aa: Some(length_aa),
            molecular_weight_da: None,
            sequence_ref: None,
        },
        quality: None,
        domains: None,
        ligands: None,
        files: None,
    }
}

// ============================================================================
// Synthetic Biology Part Operations
// ============================================================================

/// Create a BioBrick-compatible part
pub fn create_biopart(
    name: &str,
    part_type: PartType,
    sequence: &str,
) -> BioResult<BioPart> {
    validate_dna_sequence(sequence)?;

    Ok(BioPart {
        schema: Some("https://wia.live/schemas/bio/part.schema.json".to_string()),
        part_id: format!("part-{}", Uuid::new_v4().to_string().split('-').next().unwrap()),
        part_info: PartInfo {
            name: Some(name.to_string()),
            description: None,
            short_name: None,
            part_type,
        },
        registry: None,
        sequence: PartSequence {
            dna: sequence.to_uppercase(),
            length_bp: Some(sequence.len() as u64),
        },
        assembly: Some(AssemblyInfo {
            standard: Some(AssemblyStandard::Biobrick),
            prefix: Some("GAATTCGCGGCCGCTTCTAGAG".to_string()),
            suffix: Some("TACTAGTAGCGGCCGCTGCAG".to_string()),
            compatible_standards: Some(vec![
                "biobrick".to_string(),
                "golden_gate".to_string(),
            ]),
            fusion_sites: None,
        }),
        characterization: None,
        performance: None,
        files: None,
    })
}

/// Add BioBrick prefix and suffix to a sequence
pub fn add_biobrick_scars(sequence: &str) -> String {
    const BIOBRICK_PREFIX: &str = "GAATTCGCGGCCGCTTCTAGAG";
    const BIOBRICK_SUFFIX: &str = "TACTAGTAGCGGCCGCTGCAG";

    format!("{}{}{}", BIOBRICK_PREFIX, sequence.to_uppercase(), BIOBRICK_SUFFIX)
}

/// Create an assembly design
pub fn create_assembly(
    name: &str,
    method: AssemblyStandard,
    parts: Vec<(String, PartType)>,
) -> Assembly {
    let assembly_parts: Vec<AssemblyPart> = parts
        .into_iter()
        .enumerate()
        .map(|(i, (part_ref, part_type))| AssemblyPart {
            position: (i + 1) as u32,
            part_ref: Some(part_ref),
            name: None,
            part_type: Some(part_type),
        })
        .collect();

    Assembly {
        schema: Some("https://wia.live/schemas/bio/assembly.schema.json".to_string()),
        assembly_id: format!(
            "asm-{}",
            Uuid::new_v4().to_string().split('-').next().unwrap()
        ),
        assembly_info: Some(AssemblyDesignInfo {
            name: Some(name.to_string()),
            description: None,
            assembly_date: Some(Utc::now().date_naive()),
        }),
        assembly_method: Some(method),
        backbone: None,
        parts: assembly_parts,
        final_construct: None,
        verification: Some(Verification {
            method: None,
            status: Some(VerificationStatus::Pending),
            date: None,
        }),
    }
}

// ============================================================================
// Project Operations
// ============================================================================

/// Create a new project
pub fn create_project(
    name: &str,
    category: ProjectCategory,
) -> Project {
    Project {
        schema: Some("https://wia.live/schemas/bio/project.schema.json".to_string()),
        wia_version: WIA_VERSION.to_string(),
        format_version: Some(WIA_VERSION.to_string()),
        project_id: format!(
            "proj-{}",
            Uuid::new_v4().to_string().split('-').next().unwrap()
        ),
        project_info: ProjectInfo {
            name: name.to_string(),
            description: None,
            start_date: Some(Utc::now().date_naive()),
            status: Some(ProjectStatus::Active),
            category: Some(category),
        },
        organization: None,
        data_files: Some(DataFiles::default()),
        ethics: None,
        created_at: Some(Utc::now()),
        updated_at: Some(Utc::now()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gc_content() {
        assert!((calculate_gc_content("ATCG") - 0.5).abs() < 0.001);
        assert!((calculate_gc_content("GGCC") - 1.0).abs() < 0.001);
        assert!((calculate_gc_content("ATAT") - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_validate_dna() {
        assert!(validate_dna_sequence("ATCG").is_ok());
        assert!(validate_dna_sequence("ATCGX").is_err());
    }

    #[test]
    fn test_reverse_complement() {
        assert_eq!(reverse_complement("ATCG"), "CGAT");
        assert_eq!(reverse_complement("AAAA"), "TTTT");
    }

    #[test]
    fn test_transcribe() {
        assert_eq!(transcribe_dna_to_rna("ATCG"), "AUCG");
    }

    #[test]
    fn test_find_pam_sites() {
        let sites = find_pam_sites_ngg("ATCGATCGGNGG");
        assert!(!sites.is_empty());
    }

    #[test]
    fn test_interpret_plddt() {
        assert!(interpret_plddt(95.0).contains("Very high"));
        assert!(interpret_plddt(80.0).contains("High"));
        assert!(interpret_plddt(60.0).contains("Low"));
        assert!(interpret_plddt(30.0).contains("Very low"));
    }

    #[test]
    fn test_create_sequence() {
        let seq = create_sequence("Test Gene", "ATCGATCG", SequenceType::Dna);
        assert!(seq.is_ok());
        let seq = seq.unwrap();
        assert_eq!(seq.length_bp, 8);
        assert!(seq.gc_content.is_some());
    }

    #[test]
    fn test_create_biopart() {
        let part = create_biopart("Test Promoter", PartType::Promoter, "ATCGATCG");
        assert!(part.is_ok());
    }
}
