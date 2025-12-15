//! WIA Health Standard Type Definitions
//!
//! Comprehensive types for health and longevity data based on WIA Health Standard v1.0.0

use chrono::{DateTime, NaiveDate, Utc};
use serde::{Deserialize, Serialize};
use uuid::Uuid;
use validator::Validate;

// ============================================================================
// Common Types
// ============================================================================

/// Measurement with value, unit, and optional reference range
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct Measurement {
    pub value: f64,
    pub unit: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reference_range: Option<ReferenceRange>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timestamp: Option<DateTime<Utc>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub method: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub laboratory: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub flags: Option<Vec<MeasurementFlag>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub notes: Option<String>,
}

/// Reference range for a measurement
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReferenceRange {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub low: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub high: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub population: Option<String>,
}

/// Measurement flags
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum MeasurementFlag {
    Normal,
    Abnormal,
    Critical,
    Low,
    High,
}

/// Metadata for tracking data provenance
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Metadata {
    pub version: String,
    pub created_at: DateTime<Utc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub updated_at: Option<DateTime<Utc>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<DataSource>,
}

/// Data source information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataSource {
    pub system: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
}

/// Consent information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Consent {
    pub data_sharing: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub research: Option<bool>,
    pub consent_date: NaiveDate,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub expiration_date: Option<NaiveDate>,
}

/// Biological sex
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum BiologicalSex {
    Male,
    Female,
    Other,
    Unknown,
}

/// Subject information
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct Subject {
    pub id: Uuid,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub anonymized_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 1900, max = 2100))]
    pub birth_year: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub biological_sex: Option<BiologicalSex>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ethnicity: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub consent: Option<Consent>,
}

// ============================================================================
// Biomarker Types
// ============================================================================

/// Inflammatory biomarkers
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct InflammatoryMarkers {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub crp: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub il6: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tnf_alpha: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub il1_beta: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub il10: Option<Measurement>,
}

/// Cholesterol panel
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CholesterolPanel {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ldl: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hdl: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub triglycerides: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vldl: Option<Measurement>,
}

/// Metabolic biomarkers
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MetabolicMarkers {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub igf1: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gdf15: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub glucose: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub insulin: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hba1c: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub homa_ir: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cholesterol: Option<CholesterolPanel>,
}

/// Thyroid panel
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ThyroidPanel {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tsh: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub free_t3: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub free_t4: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reverse_t3: Option<Measurement>,
}

/// Hormonal biomarkers
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct HormonalMarkers {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub testosterone: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub free_testosterone: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub estradiol: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dheas: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cortisol: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub growth_hormone: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub thyroid: Option<ThyroidPanel>,
}

/// Liver function markers
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LiverFunction {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alt: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ast: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ggt: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub albumin: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bilirubin: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alp: Option<Measurement>,
}

/// Kidney function markers
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct KidneyFunction {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub creatinine: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub egfr: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bun: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cystatin_c: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub uric_acid: Option<Measurement>,
}

/// Organ function markers
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct OrganFunction {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub liver: Option<LiverFunction>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub kidney: Option<KidneyFunction>,
}

/// Aging clock types
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum AgingClockType {
    Horvath,
    Hannum,
    GrimAge,
    PhenoAge,
    DunedinPace,
    Levine,
    SkinBlood,
    Custom,
}

/// Aging clock algorithm information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgingClockAlgorithm {
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub provider: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cpg_sites: Option<i32>,
}

/// Aging clocks (biological age calculations)
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct AgingClocks {
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 150.0))]
    pub chronological_age: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 150.0))]
    pub biological_age: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub clock_type: Option<AgingClockType>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub age_delta: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 1.0))]
    pub confidence: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub calculated_at: Option<DateTime<Utc>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub algorithm: Option<AgingClockAlgorithm>,
}

/// Complete biomarker profile
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct BiomarkerProfile {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub inflammatory_markers: Option<InflammatoryMarkers>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metabolic_markers: Option<MetabolicMarkers>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hormonal_markers: Option<HormonalMarkers>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub organ_function: Option<OrganFunction>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub aging_clocks: Option<AgingClocks>,
}

// ============================================================================
// Genomics Types
// ============================================================================

/// Sequencing type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum SequencingType {
    WGS,
    WES,
    TargetedPanel,
    SNPArray,
    RNASeq,
}

/// Reference genome
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum ReferenceGenome {
    GRCh37,
    GRCh38,
    #[serde(rename = "T2T-CHM13")]
    T2TCHM13,
}

/// Sequencing information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SequencingInfo {
    #[serde(rename = "type")]
    pub sequencing_type: SequencingType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub platform: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub coverage: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quality_score: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub date: Option<DateTime<Utc>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub laboratory: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub accession: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reference_genome: Option<ReferenceGenome>,
}

/// Zygosity
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Zygosity {
    Homozygous,
    Heterozygous,
    Hemizygous,
}

/// Clinical significance
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum ClinicalSignificance {
    Pathogenic,
    LikelyPathogenic,
    Uncertain,
    LikelyBenign,
    Benign,
}

/// Inheritance pattern
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum InheritancePattern {
    AutosomalDominant,
    AutosomalRecessive,
    XLinkedDominant,
    XLinkedRecessive,
    Mitochondrial,
    Multifactorial,
}

/// HGVS notation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HgvsNotation {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub c: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub p: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub g: Option<String>,
}

/// Genetic variant
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeneticVariant {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gene: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rs_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub chromosome: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub position: Option<i64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reference: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alternate: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub zygosity: Option<Zygosity>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub clinical_significance: Option<ClinicalSignificance>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub condition: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub inheritance: Option<InheritancePattern>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub clinvar_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hgvs: Option<HgvsNotation>,
}

/// Metabolizer status
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum MetabolizerStatus {
    PoorMetabolizer,
    IntermediateMetabolizer,
    NormalMetabolizer,
    RapidMetabolizer,
    UltrarapidMetabolizer,
}

/// Evidence level (CPIC)
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum EvidenceLevel {
    #[serde(rename = "1A")]
    Level1A,
    #[serde(rename = "1B")]
    Level1B,
    #[serde(rename = "2A")]
    Level2A,
    #[serde(rename = "2B")]
    Level2B,
    #[serde(rename = "3")]
    Level3,
    #[serde(rename = "4")]
    Level4,
}

/// Drug response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrugResponse {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub drug: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gene: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub diplotype: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub phenotype: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub recommendation: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub evidence_level: Option<EvidenceLevel>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<String>,
}

/// Pharmacogenomics profile
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Pharmacogenomics {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metabolizer_status: Option<std::collections::HashMap<String, MetabolizerStatus>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub drug_responses: Option<Vec<DrugResponse>>,
}

/// Risk category
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum RiskCategory {
    Low,
    Average,
    Elevated,
    High,
}

/// Polygenic risk score
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct PolygenicRiskScore {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub condition: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub score: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 100.0))]
    pub percentile: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub risk_category: Option<RiskCategory>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub algorithm: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub snp_count: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub population: Option<String>,
}

/// Ancestry population
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct AncestryPopulation {
    pub population: String,
    #[validate(range(min = 0.0, max = 100.0))]
    pub percentage: f64,
}

/// Haplogroups
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Haplogroups {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub maternal: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub paternal: Option<String>,
}

/// Ancestry information
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Ancestry {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub populations: Option<Vec<AncestryPopulation>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub haplogroups: Option<Haplogroups>,
}

/// Complete genomic profile
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GenomicProfile {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sequencing: Option<SequencingInfo>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub variants: Option<Vec<GeneticVariant>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pharmacogenomics: Option<Pharmacogenomics>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub polygenetic_risk_scores: Option<Vec<PolygenicRiskScore>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ancestry: Option<Ancestry>,
}

// ============================================================================
// Epigenetics Types
// ============================================================================

/// Epigenetic clock type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum EpigeneticClockType {
    Horvath,
    Hannum,
    PhenoAge,
    GrimAge,
    GrimAge2,
    SkinBlood,
    Pace,
    DunedinPace,
    Custom,
}

/// Tissue type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum TissueType {
    Blood,
    Saliva,
    Skin,
    Buccal,
    Adipose,
    Muscle,
    Other,
}

/// Methylation platform
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum MethylationPlatform {
    Illumina450K,
    IlluminaEPIC,
    IlluminaEPICv2,
    WGBS,
    RRBS,
    Custom,
}

/// Quality metrics for methylation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MethylationQualityMetrics {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub detection_p_value: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bisulfite_conversion: Option<f64>,
}

/// Methylation age
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct MethylationAge {
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 150.0))]
    pub age: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub clock_type: Option<EpigeneticClockType>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tissue: Option<TissueType>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cpg_sites: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub platform: Option<MethylationPlatform>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub calculated_at: Option<DateTime<Utc>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub laboratory: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quality_metrics: Option<MethylationQualityMetrics>,
}

/// Expression measurement
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExpressionMeasurement {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub expression: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub method: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reference_gene: Option<String>,
}

/// SA-beta-galactosidase
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct SaBetaGalactosidase {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub positive: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 100.0))]
    pub percentage: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub method: Option<String>,
}

/// SASP markers
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SaspMarkers {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub il6: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub il8: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mcp1: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mmp3: Option<f64>,
}

/// Senescence markers
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SenescenceMarkers {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub p16_ink4a: Option<ExpressionMeasurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub p21_cip1: Option<ExpressionMeasurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub p53: Option<ExpressionMeasurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sa_beta_galactosidase: Option<SaBetaGalactosidase>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sasp: Option<SaspMarkers>,
}

/// Heterochromatin level
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum HeterochromatinLevel {
    Normal,
    Reduced,
    Increased,
}

/// ATAC-seq data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AtacSeqData {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frip: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tss_enrichment: Option<f64>,
}

/// Chromatin state
#[derive(Debug, Clone, Default, Serialize, Deserialize, Validate)]
pub struct ChromatinState {
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 100.0))]
    pub global_methylation: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub line_elements: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub heterochromatin_level: Option<HeterochromatinLevel>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub h3k9me3: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub h3k27me3: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub atac_seq: Option<AtacSeqData>,
}

/// Reprogramming method
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum ReprogrammingMethod {
    YamanakaFactors,
    PartialReprogramming,
    Oksm,
    Oskmln,
    ChemicalReprogramming,
    Other,
}

/// Reprogramming factor
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum ReprogrammingFactor {
    Oct4,
    Sox2,
    Klf4,
    #[serde(rename = "c-Myc")]
    CMyc,
    Lin28,
    Nanog,
    Other,
}

/// Delivery method
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum DeliveryMethod {
    Viral,
    MRNA,
    Plasmid,
    Protein,
    SmallMolecule,
}

/// Reprogramming outcome
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum ReprogrammingOutcome {
    Successful,
    Partial,
    Unsuccessful,
}

/// Reprogramming history entry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReprogrammingEntry {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<Uuid>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub date: Option<DateTime<Utc>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub method: Option<ReprogrammingMethod>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub factors: Option<Vec<ReprogrammingFactor>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub delivery_method: Option<DeliveryMethod>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cycles: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outcome: Option<ReprogrammingOutcome>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub age_reduction: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub side_effects: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub notes: Option<String>,
}

/// Complete epigenetic profile
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EpigeneticProfile {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub methylation_age: Option<MethylationAge>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub senescence_markers: Option<SenescenceMarkers>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub chromatin_state: Option<ChromatinState>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reprogramming_history: Option<Vec<ReprogrammingEntry>>,
}

// ============================================================================
// Telomere Types
// ============================================================================

/// Telomere length unit
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum TelomereLengthUnit {
    Kilobases,
    Kb,
    #[serde(rename = "TRF")]
    Trf,
    #[serde(rename = "T/S_ratio")]
    TsRatio,
}

/// Telomere length value
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TelomereLength {
    pub value: f64,
    pub unit: TelomereLengthUnit,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub percentile: Option<f64>,
}

/// Telomere measurement method
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum TelomereMeasurementMethod {
    #[serde(rename = "qPCR")]
    QPCR,
    TRF,
    FISH,
    FlowFISH,
    STELA,
    TeSLA,
    Nanopore,
}

/// Cell type for telomere measurement
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum TelomereCellType {
    Leukocytes,
    Lymphocytes,
    Granulocytes,
    Pbmc,
    TCells,
    BCells,
    NkCells,
    Fibroblasts,
    Other,
}

/// Quality score
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum QualityScore {
    Excellent,
    Good,
    Acceptable,
    Poor,
}

/// Telomere measurement
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct TelomereMeasurement {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<Uuid>,
    pub average_length: TelomereLength,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub shortest_telomere: Option<TelomereLength>,
    pub method: TelomereMeasurementMethod,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cell_type: Option<TelomereCellType>,
    pub timestamp: DateTime<Utc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub laboratory: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 100.0))]
    pub coefficient_of_variation: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quality_score: Option<QualityScore>,
}

/// Telomerase activity level
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum TelomeraseActivityLevel {
    Undetectable,
    Low,
    Normal,
    Elevated,
    High,
}

/// Telomerase activity assay method
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum TelomeraseAssayMethod {
    TRAP,
    DdTRAP,
    QTRAP,
    ELISA,
}

/// Telomerase activity
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TelomeraseActivity {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub level: Option<TelomeraseActivityLevel>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quantitative: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub method: Option<TelomeraseAssayMethod>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cell_type: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timestamp: Option<DateTime<Utc>>,
}

/// Telomere age category
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum TelomereAgeCategory {
    Shorter,
    Average,
    Longer,
}

/// Telomere age equivalent
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct TelomereAgeEquivalent {
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 150.0))]
    pub years: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 100.0))]
    pub percentile: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub category: Option<TelomereAgeCategory>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reference_population: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub algorithm: Option<String>,
}

/// Attrition rate category
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum AttritionRateCategory {
    Slow,
    Normal,
    Accelerated,
}

/// Telomere attrition rate
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TelomereAttritionRate {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rate: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub category: Option<AttritionRateCategory>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub measurement_period: Option<String>,
}

/// Telomere intervention type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum TelomereInterventionType {
    TelomeraseActivator,
    GeneTherapy,
    Lifestyle,
    Supplement,
    Pharmaceutical,
}

/// Telomere intervention outcome
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TelomereInterventionOutcome {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub length_change: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub percentile_change: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub activity_change: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration: Option<String>,
}

/// Telomere intervention
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TelomereIntervention {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<Uuid>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(rename = "type")]
    pub intervention_type: Option<TelomereInterventionType>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mechanism: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub start_date: Option<NaiveDate>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub end_date: Option<NaiveDate>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dosage: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frequency: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outcome: Option<TelomereInterventionOutcome>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub adverse_events: Option<Vec<String>>,
}

/// Complete telomere profile
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TelomereProfile {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub measurements: Option<Vec<TelomereMeasurement>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub telomerase_activity: Option<TelomeraseActivity>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub age_equivalent: Option<TelomereAgeEquivalent>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub attrition_rate: Option<TelomereAttritionRate>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub interventions: Option<Vec<TelomereIntervention>>,
}

// ============================================================================
// Digital Twin Types
// ============================================================================

/// Digital twin type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum DigitalTwinType {
    WholeBody,
    Cardiac,
    Metabolic,
    Neurological,
    Respiratory,
    Musculoskeletal,
    Custom,
}

/// Digital twin status
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum DigitalTwinStatus {
    Active,
    Archived,
    Updating,
    Calibrating,
    Error,
}

/// Fidelity level
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum FidelityLevel {
    Low,
    Medium,
    High,
    Research,
}

/// Data stream frequency
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum DataStreamFrequency {
    Realtime,
    Continuous,
    Hourly,
    Daily,
    Weekly,
}

/// Privacy level
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum PrivacyLevel {
    Anonymized,
    Pseudonymized,
    Identified,
}

/// Clinical data stream
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClinicalDataStream {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub enabled: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sources: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_sync: Option<DateTime<Utc>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fhir_endpoint: Option<String>,
}

/// Physiological data stream
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhysiologicalDataStream {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub enabled: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sources: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frequency: Option<DataStreamFrequency>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_sync: Option<DateTime<Utc>>,
}

/// Behavioral data stream
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BehavioralDataStream {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub enabled: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sources: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub privacy: Option<PrivacyLevel>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_sync: Option<DateTime<Utc>>,
}

/// Environmental data stream
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvironmentalDataStream {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub enabled: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sources: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub location: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_sync: Option<DateTime<Utc>>,
}

/// Genomic data stream
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GenomicDataStream {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub enabled: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data_types: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_sync: Option<DateTime<Utc>>,
}

/// Data streams
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DataStreams {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub clinical: Option<ClinicalDataStream>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub physiological: Option<PhysiologicalDataStream>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub behavioral: Option<BehavioralDataStream>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub environmental: Option<EnvironmentalDataStream>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub genomic: Option<GenomicDataStream>,
}

/// Organ type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum OrganType {
    Heart,
    Brain,
    Liver,
    Kidney,
    Pancreas,
    Lung,
    Gut,
    Vascular,
    Immune,
    Endocrine,
    Musculoskeletal,
}

/// Model type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum ModelType {
    Physiological,
    Mechanical,
    Metabolic,
    Electrical,
    Hybrid,
}

/// Prediction
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct Prediction {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outcome: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 1.0))]
    pub probability: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeframe: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 1.0))]
    pub confidence: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub calculated_at: Option<DateTime<Utc>>,
}

/// Organ model
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct OrganModel {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<Uuid>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub organ: Option<OrganType>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model_type: Option<ModelType>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub algorithm: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 1.0))]
    pub accuracy: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_calibrated: Option<DateTime<Utc>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub calibration_data: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub predictions: Option<Vec<Prediction>>,
}

/// Simulation type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum SimulationType {
    DrugResponse,
    Surgery,
    Lifestyle,
    Disease,
    Intervention,
    Aging,
    WhatIf,
}

/// Simulation status
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum SimulationStatus {
    Completed,
    Failed,
    Running,
    Queued,
}

/// Simulation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Simulation {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<Uuid>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(rename = "type")]
    pub simulation_type: Option<SimulationType>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parameters: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub results: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timestamp: Option<DateTime<Utc>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub status: Option<SimulationStatus>,
}

/// Health score trend
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum HealthScoreTrend {
    Improving,
    Stable,
    Declining,
}

/// Health score components
#[derive(Debug, Clone, Default, Serialize, Deserialize, Validate)]
pub struct HealthScoreComponents {
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 100.0))]
    pub cardiovascular: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 100.0))]
    pub metabolic: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 100.0))]
    pub cognitive: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 100.0))]
    pub immune: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 100.0))]
    pub musculoskeletal: Option<f64>,
}

/// Health score
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct HealthScore {
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 100.0))]
    pub overall: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub components: Option<HealthScoreComponents>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trend: Option<HealthScoreTrend>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub calculated_at: Option<DateTime<Utc>>,
}

/// Complete digital twin profile
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DigitalTwinProfile {
    pub id: Uuid,
    pub version: String,
    #[serde(rename = "type")]
    pub twin_type: DigitalTwinType,
    pub status: DigitalTwinStatus,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fidelity: Option<FidelityLevel>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data_streams: Option<DataStreams>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub models: Option<Vec<OrganModel>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub simulations: Option<Vec<Simulation>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub health_score: Option<HealthScore>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_updated: Option<DateTime<Utc>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub next_calibration: Option<DateTime<Utc>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<Metadata>,
}

// ============================================================================
// Intervention Types
// ============================================================================

/// Intervention category
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum InterventionCategory {
    Pharmaceutical,
    Nutraceutical,
    Lifestyle,
    Procedure,
    Device,
    Genetic,
    CellTherapy,
    Other,
}

/// Target mechanism
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum TargetMechanism {
    Senolytics,
    Senomorphics,
    TelomereExtension,
    EpigeneticReprogramming,
    Mitochondrial,
    AntiInflammatory,
    Metabolic,
    Hormonal,
    Autophagy,
    Proteostasis,
    StemCell,
    GenomicStability,
    NutrientSensing,
    IntercellularCommunication,
}

/// Administration route
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum AdministrationRoute {
    Oral,
    Sublingual,
    Injection,
    Infusion,
    Topical,
    Inhalation,
    Implant,
    Other,
}

/// Intervention protocol
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterventionProtocol {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dosage: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frequency: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub route: Option<AdministrationRoute>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timing: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cycling: Option<String>,
}

/// Intervention status
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum InterventionStatus {
    Planned,
    Active,
    Paused,
    Completed,
    Discontinued,
}

/// Prescriber information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Prescriber {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub specialty: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub institution: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub identifier: Option<String>,
}

/// Intervention source
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterventionSource {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub manufacturer: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lot_number: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub expiration_date: Option<NaiveDate>,
}

/// Primary endpoint
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrimaryEndpoint {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub measure: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub baseline: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub final_value: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub change: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub change_percent: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub significant: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub p_value: Option<f64>,
}

/// Secondary endpoint
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SecondaryEndpoint {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub measure: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub baseline: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub final_value: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub change: Option<f64>,
}

/// Subjective outcome domain
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum SubjectiveOutcomeDomain {
    Energy,
    Sleep,
    Cognition,
    Mood,
    Pain,
    Mobility,
    Overall,
}

/// Subjective outcome
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubjectiveOutcome {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub domain: Option<SubjectiveOutcomeDomain>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub baseline_score: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub final_score: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub scale: Option<String>,
}

/// Adverse event severity
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum AdverseEventSeverity {
    Mild,
    Moderate,
    Severe,
    LifeThreatening,
}

/// Adverse event action
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum AdverseEventAction {
    None,
    DoseReduced,
    Discontinued,
    Treated,
    Hospitalized,
}

/// Adverse event relatedness
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum AdverseEventRelatedness {
    Unrelated,
    Unlikely,
    Possible,
    Probable,
    Definite,
}

/// Adverse event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdverseEvent {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub event: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub severity: Option<AdverseEventSeverity>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub category: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub onset_date: Option<NaiveDate>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub resolution_date: Option<NaiveDate>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub resolved: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub action_taken: Option<AdverseEventAction>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub relatedness: Option<AdverseEventRelatedness>,
}

/// Intervention outcomes
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct InterventionOutcomes {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub primary_endpoint: Option<PrimaryEndpoint>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub secondary_endpoints: Option<Vec<SecondaryEndpoint>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub subjective_outcomes: Option<Vec<SubjectiveOutcome>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub adverse_events: Option<Vec<AdverseEvent>>,
}

/// Monitoring schedule
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MonitoringSchedule {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub schedule: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub biomarkers: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_assessment: Option<NaiveDate>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub next_assessment: Option<NaiveDate>,
}

/// Evidence level (Oxford CEBM)
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum OxfordEvidenceLevel {
    #[serde(rename = "1a")]
    Level1a,
    #[serde(rename = "1b")]
    Level1b,
    #[serde(rename = "2a")]
    Level2a,
    #[serde(rename = "2b")]
    Level2b,
    #[serde(rename = "3")]
    Level3,
    #[serde(rename = "4")]
    Level4,
    #[serde(rename = "5")]
    Level5,
}

/// Recommendation grade
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum RecommendationGrade {
    A,
    B,
    C,
    D,
}

/// Reference
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Reference {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pmid: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub doi: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub title: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub journal: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub year: Option<i32>,
}

/// Evidence
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Evidence {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub level: Option<OxfordEvidenceLevel>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub grade: Option<RecommendationGrade>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub references: Option<Vec<Reference>>,
}

/// Cost coverage
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum CostCoverage {
    SelfPay,
    PartialCoverage,
    FullCoverage,
    Research,
}

/// Cost information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Cost {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub amount: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub currency: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frequency: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub coverage: Option<CostCoverage>,
}

/// Intervention
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Intervention {
    pub id: Uuid,
    pub category: InterventionCategory,
    pub name: String,
    pub status: InterventionStatus,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target_mechanism: Option<Vec<TargetMechanism>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub protocol: Option<InterventionProtocol>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub start_date: Option<NaiveDate>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub end_date: Option<NaiveDate>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub prescriber: Option<Prescriber>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<InterventionSource>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outcomes: Option<InterventionOutcomes>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub monitoring: Option<MonitoringSchedule>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub evidence: Option<Evidence>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cost: Option<Cost>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub notes: Option<String>,
}

/// Intervention history
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct InterventionHistory {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub interventions: Option<Vec<Intervention>>,
}

// ============================================================================
// Health Profile (Root Type)
// ============================================================================

/// Complete WIA Health Profile
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct HealthProfile {
    pub id: Uuid,
    pub version: String,
    #[validate(nested)]
    pub subject: Subject,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub biomarkers: Option<BiomarkerProfile>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub genomics: Option<GenomicProfile>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub epigenetics: Option<EpigeneticProfile>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub telomeres: Option<TelomereProfile>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub digital_twin: Option<DigitalTwinProfile>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub interventions: Option<InterventionHistory>,
    pub metadata: Metadata,
}

impl HealthProfile {
    /// Create a new health profile
    pub fn new(subject: Subject) -> Self {
        Self {
            id: Uuid::new_v4(),
            version: "1.0.0".to_string(),
            subject,
            biomarkers: None,
            genomics: None,
            epigenetics: None,
            telomeres: None,
            digital_twin: None,
            interventions: None,
            metadata: Metadata {
                version: "1.0.0".to_string(),
                created_at: Utc::now(),
                updated_at: None,
                source: None,
            },
        }
    }

    /// Calculate biological age delta
    pub fn age_delta(&self) -> Option<f64> {
        self.biomarkers
            .as_ref()
            .and_then(|b| b.aging_clocks.as_ref())
            .and_then(|c| c.age_delta)
    }

    /// Get overall health score
    pub fn health_score(&self) -> Option<f64> {
        self.digital_twin
            .as_ref()
            .and_then(|dt| dt.health_score.as_ref())
            .and_then(|hs| hs.overall)
    }
}
