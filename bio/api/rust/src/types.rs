//! WIA Biotech Type Definitions
//!
//! This module contains all type definitions for the WIA Biotechnology Standard,
//! including sequences, CRISPR experiments, protein structures, and synthetic biology parts.

use chrono::{DateTime, NaiveDate, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use validator::Validate;

// ============================================================================
// Common Types
// ============================================================================

/// WIA Biotech format version
pub const WIA_VERSION: &str = "1.0.0";

/// Checksum information for data integrity
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Checksum {
    pub algorithm: ChecksumAlgorithm,
    pub value: String,
}

/// Supported checksum algorithms
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum ChecksumAlgorithm {
    Md5,
    Sha256,
}

// ============================================================================
// Project Types
// ============================================================================

/// Project metadata container
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct Project {
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,

    pub wia_version: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub format_version: Option<String>,

    #[validate(length(min = 1))]
    pub project_id: String,

    pub project_info: ProjectInfo,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub organization: Option<Organization>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub data_files: Option<DataFiles>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub ethics: Option<Ethics>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub created_at: Option<DateTime<Utc>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub updated_at: Option<DateTime<Utc>>,
}

/// Project information
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct ProjectInfo {
    #[validate(length(min = 1))]
    pub name: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub start_date: Option<NaiveDate>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub status: Option<ProjectStatus>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub category: Option<ProjectCategory>,
}

/// Project status
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum ProjectStatus {
    Planning,
    Active,
    Completed,
    Archived,
}

/// Project category
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ProjectCategory {
    GeneTherapy,
    SyntheticBiology,
    ProteinEngineering,
    DrugDiscovery,
    Diagnostics,
    Agriculture,
    Industrial,
    Research,
    Other,
}

/// Organization information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Organization {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub department: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub principal_investigator: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub contact_email: Option<String>,
}

/// Data file references
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct DataFiles {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sequences: Option<Vec<String>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub editing: Option<Vec<String>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub structures: Option<Vec<String>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub parts: Option<Vec<String>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub experiments: Option<Vec<String>>,
}

/// Ethics and regulatory information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Ethics {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub irb_approval: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub biosafety_level: Option<BiosafetyLevel>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub gmo_approval: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub iacuc_approval: Option<String>,
}

/// Biosafety levels
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum BiosafetyLevel {
    #[serde(rename = "BSL-1")]
    Bsl1,
    #[serde(rename = "BSL-2")]
    Bsl2,
    #[serde(rename = "BSL-3")]
    Bsl3,
    #[serde(rename = "BSL-4")]
    Bsl4,
}

// ============================================================================
// Sequence Types
// ============================================================================

/// Biological sequence data
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct Sequence {
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,

    #[validate(length(min = 1))]
    pub sequence_id: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub sequence_info: Option<SequenceInfo>,

    pub sequence_type: SequenceType,

    #[validate(range(min = 1))]
    pub length_bp: u64,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub gc_content: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<SequenceSource>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub annotations: Option<Vec<Annotation>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub files: Option<SequenceFiles>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub checksum: Option<Checksum>,
}

/// Sequence information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SequenceInfo {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub organism: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub taxonomy_id: Option<u64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub gene_symbol: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub gene_id: Option<String>,
}

/// Sequence types
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum SequenceType {
    Dna,
    Rna,
    Mrna,
    Protein,
    Synthetic,
}

/// Sequence source database
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SequenceSource {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub database: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub accession: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub retrieved_date: Option<NaiveDate>,
}

/// Sequence annotation
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct Annotation {
    pub feature: FeatureType,

    #[validate(range(min = 1))]
    pub start: u64,

    #[validate(range(min = 1))]
    pub end: u64,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub strand: Option<Strand>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub qualifiers: Option<HashMap<String, String>>,
}

/// Feature types for annotations
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum FeatureType {
    Gene,
    Exon,
    Intron,
    Cds,
    Utr5,
    Utr3,
    Promoter,
    Terminator,
    Rbs,
    Operator,
    Enhancer,
    Silencer,
    Repeat,
    Misc,
}

/// Strand orientation
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum Strand {
    #[serde(rename = "+")]
    Plus,
    #[serde(rename = "-")]
    Minus,
}

/// Sequence file references
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct SequenceFiles {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fasta: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub genbank: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub embl: Option<String>,
}

// ============================================================================
// CRISPR Types
// ============================================================================

/// CRISPR gene editing experiment
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct CrisprExperiment {
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,

    #[validate(length(min = 1))]
    pub experiment_id: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub experiment_info: Option<ExperimentInfo>,

    pub target: CrisprTarget,

    pub editing_system: EditingSystem,

    #[validate(length(min = 1))]
    pub guide_rnas: Vec<GuideRna>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub editing_type: Option<EditingType>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub repair_template: Option<RepairTemplate>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub protocol: Option<CrisprProtocol>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub results_ref: Option<String>,
}

/// Experiment information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExperimentInfo {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub date: Option<NaiveDate>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub experimenter: Option<String>,
}

/// CRISPR target information
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct CrisprTarget {
    #[validate(length(min = 1))]
    pub gene_symbol: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub gene_id: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub organism: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub cell_line: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub cell_type: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub sequence_ref: Option<String>,
}

/// CRISPR editing system configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EditingSystem {
    #[serde(rename = "type")]
    pub system_type: CrisprSystemType,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub cas_variant: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub pam_sequence: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub delivery_method: Option<DeliveryMethod>,
}

/// CRISPR system types
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum CrisprSystemType {
    CrisprCas9,
    CrisprSaCas9,
    CrisprCas12a,
    CrisprCas13,
    BaseEditor,
    PrimeEditor,
}

/// Delivery methods for CRISPR components
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum DeliveryMethod {
    Lipofection,
    Electroporation,
    ViralAav,
    ViralLentivirus,
    Rnp,
    Microinjection,
    Other,
}

/// Guide RNA definition
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct GuideRna {
    #[validate(length(min = 1))]
    pub grna_id: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,

    #[validate(length(min = 15, max = 25))]
    pub sequence: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub target_start: Option<u64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub target_end: Option<u64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub strand: Option<Strand>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub pam: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub off_target_score: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub on_target_score: Option<f64>,
}

/// Types of gene editing
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum EditingType {
    Knockout,
    Knockin,
    BaseEdit,
    PrimeEdit,
    Crispri,
    Crispra,
    Epigenome,
}

/// HDR repair template
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RepairTemplate {
    #[serde(rename = "type")]
    pub template_type: RepairTemplateType,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub sequence: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub homology_arm_left: Option<u64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub homology_arm_right: Option<u64>,
}

/// Repair template types
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum RepairTemplateType {
    #[serde(rename = "ssODN")]
    SsOdn,
    #[serde(rename = "dsDNA")]
    DsDna,
    Plasmid,
}

/// CRISPR protocol details
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CrisprProtocol {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cas9_concentration_nm: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub grna_concentration_nm: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub transfection_reagent: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub incubation_hours: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub selection_method: Option<String>,
}

// ============================================================================
// CRISPR Results Types
// ============================================================================

/// CRISPR experiment results
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct CrisprResults {
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,

    #[validate(length(min = 1))]
    pub results_id: String,

    pub experiment_ref: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub analysis_date: Option<NaiveDate>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub analysis_method: Option<AnalysisMethod>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub editing_efficiency: Option<EditingEfficiency>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub indel_profile: Option<Vec<IndelVariant>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub off_target_analysis: Option<OffTargetAnalysis>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub quality_metrics: Option<QualityMetrics>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub files: Option<ResultFiles>,
}

/// Analysis methods for CRISPR results
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum AnalysisMethod {
    #[serde(rename = "NGS")]
    Ngs,
    Sanger,
    T7e1,
    Surveyor,
    Tide,
    Ice,
    Other,
}

/// Editing efficiency metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EditingEfficiency {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub overall_efficiency: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub indel_frequency: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub hdr_frequency: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub wild_type_frequency: Option<f64>,
}

/// Indel variant in editing results
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IndelVariant {
    #[serde(rename = "type")]
    pub indel_type: IndelType,

    pub size_bp: i32,

    pub frequency: f64,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub sequence: Option<String>,
}

/// Types of indels
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum IndelType {
    Deletion,
    Insertion,
    Substitution,
    Complex,
}

/// Off-target analysis results
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OffTargetAnalysis {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub method: Option<OffTargetMethod>,

    pub sites_detected: u32,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub sites: Option<Vec<OffTargetSite>>,
}

/// Off-target detection methods
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum OffTargetMethod {
    #[serde(rename = "GUIDE-seq")]
    GuideSeq,
    #[serde(rename = "CIRCLE-seq")]
    CircleSeq,
    #[serde(rename = "DISCOVER-seq")]
    DiscoverSeq,
    #[serde(rename = "SITE-seq")]
    SiteSeq,
    #[serde(rename = "Digenome-seq")]
    DigenomeSeq,
    #[serde(rename = "computational")]
    Computational,
    #[serde(rename = "other")]
    Other,
}

/// Off-target site details
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OffTargetSite {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub chromosome: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub position: Option<u64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub mismatches: Option<u32>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub frequency: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub gene: Option<String>,
}

/// Quality metrics for analysis
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualityMetrics {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sequencing_depth: Option<u64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub mapping_rate: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidence_score: Option<f64>,
}

/// Result file references
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ResultFiles {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub raw_fastq: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub aligned_bam: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub analysis_report: Option<String>,
}

// ============================================================================
// Protein Structure Types
// ============================================================================

/// Protein structure data
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct ProteinStructure {
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,

    #[validate(length(min = 1))]
    pub structure_id: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub structure_info: Option<StructureInfo>,

    pub prediction: PredictionInfo,

    pub protein: ProteinInfo,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub quality: Option<StructureQuality>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub domains: Option<Vec<ProteinDomain>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub ligands: Option<Vec<Ligand>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub files: Option<StructureFiles>,
}

/// Structure metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StructureInfo {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub uniprot_id: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub pdb_id: Option<String>,
}

/// Structure prediction information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PredictionInfo {
    pub method: PredictionMethod,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub date: Option<NaiveDate>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<PredictionSource>,
}

/// Structure prediction methods
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum PredictionMethod {
    Alphafold,
    Alphafold3,
    Rosettafold,
    Esmfold,
    Experimental,
    Homology,
}

/// Prediction data sources
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum PredictionSource {
    AlphafoldDb,
    Pdb,
    LocalPrediction,
    Custom,
}

/// Protein information
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct ProteinInfo {
    #[validate(length(min = 1))]
    pub name: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub organism: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub length_aa: Option<u64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub molecular_weight_da: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub sequence_ref: Option<String>,
}

/// Structure quality metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StructureQuality {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mean_plddt: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub confident_residues_pct: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub disordered_residues_pct: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub resolution_angstrom: Option<f64>,
}

/// Protein domain
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProteinDomain {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,

    pub start: u64,

    pub end: u64,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub mean_plddt: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub pfam_id: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub interpro_id: Option<String>,
}

/// Ligand information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Ligand {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,

    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    pub ligand_type: Option<LigandType>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub binding_site_residues: Option<Vec<u64>>,
}

/// Ligand types
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum LigandType {
    SmallMolecule,
    Ion,
    Cofactor,
    NucleicAcid,
    Peptide,
}

/// Structure file references
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct StructureFiles {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pdb: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub mmcif: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidence: Option<String>,
}

/// AlphaFold confidence metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StructureConfidence {
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,

    pub structure_ref: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub plddt: Option<PlddtData>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub pae: Option<PaeData>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub interpretation: Option<ConfidenceInterpretation>,
}

/// pLDDT (predicted Local Distance Difference Test) data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlddtData {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub range: Option<(f64, f64)>,

    pub values: Vec<f64>,
}

/// PAE (Predicted Aligned Error) data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PaeData {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_value: Option<f64>,

    pub matrix: Vec<Vec<f64>>,
}

/// Confidence interpretation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfidenceInterpretation {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub high_confidence_regions: Option<Vec<ConfidenceRegion>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub low_confidence_regions: Option<Vec<ConfidenceRegion>>,
}

/// Confidence region
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfidenceRegion {
    pub start: u64,
    pub end: u64,
    pub mean_plddt: f64,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub likely_disordered: Option<bool>,
}

// ============================================================================
// Synthetic Biology Part Types
// ============================================================================

/// Synthetic biology genetic part
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct BioPart {
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,

    #[validate(length(min = 1))]
    pub part_id: String,

    pub part_info: PartInfo,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub registry: Option<PartRegistry>,

    pub sequence: PartSequence,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub assembly: Option<AssemblyInfo>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub characterization: Option<Characterization>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub performance: Option<PartPerformance>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub files: Option<PartFiles>,
}

/// Part information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PartInfo {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub short_name: Option<String>,

    pub part_type: PartType,
}

/// Part types
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum PartType {
    Promoter,
    Rbs,
    Cds,
    Terminator,
    Operator,
    Reporter,
    Origin,
    Resistance,
    Tag,
    Insulator,
    Enhancer,
    Composite,
}

/// Part registry information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PartRegistry {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<RegistrySource>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub registry_id: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub url: Option<String>,
}

/// Registry sources
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum RegistrySource {
    #[serde(rename = "iGEM Registry")]
    IgemRegistry,
    #[serde(rename = "SEVA")]
    Seva,
    #[serde(rename = "Addgene")]
    Addgene,
    #[serde(rename = "SynBioHub")]
    SynBioHub,
    #[serde(rename = "custom")]
    Custom,
    #[serde(rename = "other")]
    Other,
}

/// Part sequence data
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct PartSequence {
    #[validate(length(min = 1))]
    pub dna: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub length_bp: Option<u64>,
}

/// Assembly standard information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssemblyInfo {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub standard: Option<AssemblyStandard>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub prefix: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub suffix: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub compatible_standards: Option<Vec<String>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub fusion_sites: Option<Vec<FusionSite>>,
}

/// Assembly standards
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum AssemblyStandard {
    Biobrick,
    BiobrickRfc10,
    GoldenGate,
    Gibson,
    Moclo,
    Seva,
    Basic,
    Other,
}

/// Golden Gate fusion site
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FusionSite {
    pub position: FusionPosition,
    pub overhang: String,
}

/// Fusion site position
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum FusionPosition {
    #[serde(rename = "5prime")]
    FivePrime,
    #[serde(rename = "3prime")]
    ThreePrime,
}

/// Part characterization data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Characterization {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub organism: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub strain: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub relative_strength: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub measurement_method: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub reference_part: Option<String>,
}

/// Part performance metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PartPerformance {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub expression_level: Option<ExpressionLevel>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub reliability: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub tested_conditions: Option<Vec<String>>,
}

/// Expression level categories
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ExpressionLevel {
    VeryLow,
    Low,
    Medium,
    High,
    VeryHigh,
}

/// Part file references
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct PartFiles {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub genbank: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub sbol: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub fasta: Option<String>,
}

/// Assembly design
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct Assembly {
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,

    #[validate(length(min = 1))]
    pub assembly_id: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub assembly_info: Option<AssemblyDesignInfo>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub assembly_method: Option<AssemblyStandard>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub backbone: Option<Backbone>,

    pub parts: Vec<AssemblyPart>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub final_construct: Option<FinalConstruct>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub verification: Option<Verification>,
}

/// Assembly design information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssemblyDesignInfo {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub assembly_date: Option<NaiveDate>,
}

/// Backbone vector
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Backbone {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub registry_id: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub selection_marker: Option<String>,
}

/// Part in assembly
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssemblyPart {
    pub position: u32,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub part_ref: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,

    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    pub part_type: Option<PartType>,
}

/// Final construct information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FinalConstruct {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub length_bp: Option<u64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub sequence_ref: Option<String>,
}

/// Verification status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Verification {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub method: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub status: Option<VerificationStatus>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub date: Option<NaiveDate>,
}

/// Verification status
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum VerificationStatus {
    Pending,
    Verified,
    Failed,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sequence_type_serialization() {
        let seq_type = SequenceType::Dna;
        let json = serde_json::to_string(&seq_type).unwrap();
        assert_eq!(json, "\"dna\"");
    }

    #[test]
    fn test_crispr_system_type_serialization() {
        let system = CrisprSystemType::CrisprCas9;
        let json = serde_json::to_string(&system).unwrap();
        assert_eq!(json, "\"crispr_cas9\"");
    }

    #[test]
    fn test_guide_rna_validation() {
        let grna = GuideRna {
            grna_id: "grna-001".to_string(),
            name: Some("Test gRNA".to_string()),
            sequence: "ATCGATCGATCGATCGATCG".to_string(),
            target_start: Some(100),
            target_end: Some(119),
            strand: Some(Strand::Plus),
            pam: Some("TGG".to_string()),
            off_target_score: Some(0.95),
            on_target_score: Some(0.88),
        };

        assert!(grna.validate().is_ok());
    }
}
