//! Type definitions for the WIA-SOIL-MICROBIOME Standard
//!
//! This module contains all the data structures used in the WIA-SOIL-MICROBIOME API.
//!
//! 弘益人間 (Benefit All Humanity)

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ============================================================================
// Enums
// ============================================================================

/// Soil sample type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum SoilType {
    /// Agricultural soil
    Agricultural,
    /// Forest soil
    Forest,
    /// Grassland soil
    Grassland,
    /// Wetland soil
    Wetland,
    /// Urban soil
    Urban,
    /// Desert soil
    Desert,
    /// Tundra soil
    Tundra,
}

/// Sampling depth category
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum SamplingDepth {
    /// Topsoil 0-15cm
    #[serde(rename = "topsoil-0-15cm")]
    Topsoil0To15Cm,
    /// Subsoil 15-30cm
    #[serde(rename = "subsoil-15-30cm")]
    Subsoil15To30Cm,
    /// Deep soil 30-60cm
    #[serde(rename = "deep-30-60cm")]
    Deep30To60Cm,
    /// Very deep soil 60cm+
    #[serde(rename = "very-deep-60cm+")]
    VeryDeep60CmPlus,
}

/// Analysis methods for microbiome profiling
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum AnalysisMethod {
    /// 16S rRNA sequencing
    #[serde(rename = "16s-rrna-sequencing")]
    SixteenSRrnaSequencing,
    /// Shotgun metagenomics
    ShotgunMetagenomics,
    /// Metatranscriptomics
    Metatranscriptomics,
    /// Amplicon sequencing
    AmpliconSequencing,
    /// qPCR
    Qpcr,
    /// Plate culture
    PlateCulture,
}

/// Microbial domain classification
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum MicrobialDomain {
    /// Bacteria
    Bacteria,
    /// Archaea
    Archaea,
    /// Fungi
    Fungi,
    /// Virus
    Virus,
    /// Protist
    Protist,
}

/// Functional group categories
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum FunctionalGroup {
    /// Nitrogen fixers
    NitrogenFixers,
    /// Nitrifiers
    Nitrifiers,
    /// Denitrifiers
    Denitrifiers,
    /// Decomposers
    Decomposers,
    /// Mycorrhizal fungi
    Mycorrhizal,
    /// Pathogens
    Pathogens,
    /// Phosphate solubilizers
    PhosphateSolubilizers,
    /// Methanotrophs
    Methanotrophs,
    /// Methanogens
    Methanogens,
}

/// Soil health status
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum HealthStatus {
    /// Excellent health
    Excellent,
    /// Good health
    Good,
    /// Fair health
    Fair,
    /// Poor health
    Poor,
    /// Degraded health
    Degraded,
}

/// Carbon cycling process type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum CarbonProcess {
    /// Decomposition
    Decomposition,
    /// Respiration
    Respiration,
    /// Sequestration
    Sequestration,
    /// Mineralization
    Mineralization,
    /// Humification
    Humification,
}

/// Intervention type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum InterventionType {
    /// Compost amendment
    CompostAmendment,
    /// Cover cropping
    CoverCropping,
    /// Reduced tillage
    ReducedTillage,
    /// Crop rotation
    CropRotation,
    /// Biochar application
    BiocharApplication,
    /// Microbial inoculant
    MicrobialInoculant,
    /// Organic fertilizer
    OrganicFertilizer,
    /// Lime application
    LimeApplication,
}

/// API environment
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Environment {
    /// Production environment
    #[default]
    Production,
    /// Sandbox/testing environment
    Sandbox,
}

impl Environment {
    /// Get the base URL for this environment
    pub fn base_url(&self) -> &'static str {
        match self {
            Environment::Production => "https://api.soil-microbiome.wia.org/v1",
            Environment::Sandbox => "https://sandbox.soil-microbiome.wia.org/v1",
        }
    }

    /// Get the WebSocket URL for this environment
    pub fn ws_url(&self) -> &'static str {
        match self {
            Environment::Production => "wss://stream.soil-microbiome.wia.org/v1/ws",
            Environment::Sandbox => "wss://sandbox-stream.soil-microbiome.wia.org/v1/ws",
        }
    }
}

/// Sample status
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum SampleStatus {
    /// Pending analysis
    Pending,
    /// Currently processing
    Processing,
    /// Analysis completed
    Completed,
    /// Analysis failed
    Failed,
}

// ============================================================================
// Core Structs
// ============================================================================

/// Geographic location
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeoLocation {
    /// Latitude
    pub latitude: f64,
    /// Longitude
    pub longitude: f64,
    /// Elevation in meters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub elevation: Option<f64>,
    /// Location description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

/// Soil properties
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SoilProperties {
    /// pH level
    #[serde(skip_serializing_if = "Option::is_none", rename = "pH")]
    pub ph: Option<f64>,
    /// Organic matter percentage
    #[serde(skip_serializing_if = "Option::is_none")]
    pub organic_matter: Option<f64>,
    /// Moisture content percentage
    #[serde(skip_serializing_if = "Option::is_none")]
    pub moisture: Option<f64>,
    /// Temperature in Celsius
    #[serde(skip_serializing_if = "Option::is_none")]
    pub temperature: Option<f64>,
    /// Electrical conductivity (dS/m)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub conductivity: Option<f64>,
    /// Clay percentage
    #[serde(skip_serializing_if = "Option::is_none")]
    pub clay: Option<f64>,
    /// Silt percentage
    #[serde(skip_serializing_if = "Option::is_none")]
    pub silt: Option<f64>,
    /// Sand percentage
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sand: Option<f64>,
    /// Bulk density (g/cm³)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bulk_density: Option<f64>,
}

/// Laboratory information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Laboratory {
    /// Laboratory identifier
    pub id: String,
    /// Laboratory name
    pub name: String,
    /// Certification information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub certification: Option<String>,
    /// Laboratory location
    #[serde(skip_serializing_if = "Option::is_none")]
    pub location: Option<String>,
}

/// Soil sample
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SoilSample {
    /// Unique sample identifier
    pub id: String,
    /// Sample collection date
    pub collection_date: DateTime<Utc>,
    /// Geographic location
    pub location: GeoLocation,
    /// Soil type
    pub soil_type: SoilType,
    /// Sampling depth
    pub depth: SamplingDepth,
    /// Soil properties
    #[serde(skip_serializing_if = "Option::is_none")]
    pub properties: Option<SoilProperties>,
    /// Laboratory that performed analysis
    #[serde(skip_serializing_if = "Option::is_none")]
    pub laboratory: Option<Laboratory>,
    /// Sample metadata
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<HashMap<String, serde_json::Value>>,
}

/// Taxonomic classification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaxonomicUnit {
    /// Taxonomic identifier (e.g., NCBI taxid)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tax_id: Option<String>,
    /// Domain
    #[serde(skip_serializing_if = "Option::is_none")]
    pub domain: Option<MicrobialDomain>,
    /// Phylum
    #[serde(skip_serializing_if = "Option::is_none")]
    pub phylum: Option<String>,
    /// Class
    #[serde(skip_serializing_if = "Option::is_none")]
    pub class: Option<String>,
    /// Order
    #[serde(skip_serializing_if = "Option::is_none")]
    pub order: Option<String>,
    /// Family
    #[serde(skip_serializing_if = "Option::is_none")]
    pub family: Option<String>,
    /// Genus
    #[serde(skip_serializing_if = "Option::is_none")]
    pub genus: Option<String>,
    /// Species
    #[serde(skip_serializing_if = "Option::is_none")]
    pub species: Option<String>,
    /// Relative abundance (0-100%)
    pub abundance: f64,
    /// Number of reads
    #[serde(skip_serializing_if = "Option::is_none")]
    pub read_count: Option<u64>,
}

/// Diversity indices
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiversityIndex {
    /// Shannon diversity index
    #[serde(skip_serializing_if = "Option::is_none")]
    pub shannon: Option<f64>,
    /// Simpson diversity index
    #[serde(skip_serializing_if = "Option::is_none")]
    pub simpson: Option<f64>,
    /// Observed species richness
    #[serde(skip_serializing_if = "Option::is_none")]
    pub observed_species: Option<u32>,
    /// Chao1 richness estimator
    #[serde(skip_serializing_if = "Option::is_none")]
    pub chao1: Option<f64>,
    /// Pielou's evenness
    #[serde(skip_serializing_if = "Option::is_none")]
    pub evenness: Option<f64>,
    /// Faith's phylogenetic diversity
    #[serde(skip_serializing_if = "Option::is_none")]
    pub faiths_pd: Option<f64>,
}

/// Functional group profile
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FunctionalGroupProfile {
    /// Functional group type
    pub group: FunctionalGroup,
    /// Relative abundance percentage
    pub abundance: f64,
    /// Gene markers detected
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub gene_markers: Vec<String>,
    /// Activity level (0-1)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub activity_level: Option<f64>,
}

/// Complete microbiome profile
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MicrobiomeProfile {
    /// Profile identifier
    pub id: String,
    /// Associated sample ID
    pub sample_id: String,
    /// Analysis method used
    pub analysis_method: AnalysisMethod,
    /// Analysis date
    pub analysis_date: DateTime<Utc>,
    /// Taxonomic composition
    pub taxonomy: Vec<TaxonomicUnit>,
    /// Diversity metrics
    pub diversity: DiversityIndex,
    /// Functional groups
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub functional_groups: Vec<FunctionalGroupProfile>,
    /// Total number of sequences
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total_sequences: Option<u64>,
    /// Sequencing depth
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sequencing_depth: Option<u32>,
    /// Quality score
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quality_score: Option<f64>,
}

/// Carbon metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CarbonMetrics {
    /// Total organic carbon (g/kg)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total_organic_carbon: Option<f64>,
    /// Microbial biomass carbon (mg/kg)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub microbial_biomass_carbon: Option<f64>,
    /// Dissolved organic carbon (mg/kg)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dissolved_organic_carbon: Option<f64>,
    /// Carbon to nitrogen ratio
    #[serde(skip_serializing_if = "Option::is_none")]
    pub carbon_nitrogen_ratio: Option<f64>,
    /// Soil respiration rate (mg CO2/kg/day)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub respiration_rate: Option<f64>,
    /// Carbon sequestration potential (tons/ha/year)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sequestration_potential: Option<f64>,
}

/// Nutrient cycling metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NutrientCycling {
    /// Nitrogen mineralization rate (mg/kg/day)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nitrogen_mineralization: Option<f64>,
    /// Nitrification rate (mg/kg/day)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nitrification: Option<f64>,
    /// Denitrification rate (mg/kg/day)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub denitrification: Option<f64>,
    /// Phosphorus availability (mg/kg)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub phosphorus_availability: Option<f64>,
    /// Enzyme activity levels
    #[serde(skip_serializing_if = "Option::is_none")]
    pub enzyme_activities: Option<HashMap<String, f64>>,
}

/// Soil health index
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SoilHealthIndex {
    /// Overall health score (0-100)
    pub score: f64,
    /// Health status classification
    pub status: HealthStatus,
    /// Biological indicator score
    #[serde(skip_serializing_if = "Option::is_none")]
    pub biological_score: Option<f64>,
    /// Chemical indicator score
    #[serde(skip_serializing_if = "Option::is_none")]
    pub chemical_score: Option<f64>,
    /// Physical indicator score
    #[serde(skip_serializing_if = "Option::is_none")]
    pub physical_score: Option<f64>,
    /// Carbon metrics
    #[serde(skip_serializing_if = "Option::is_none")]
    pub carbon_metrics: Option<CarbonMetrics>,
    /// Nutrient cycling metrics
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nutrient_cycling: Option<NutrientCycling>,
    /// Assessment date
    pub assessment_date: DateTime<Utc>,
    /// Recommendations
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub recommendations: Vec<String>,
}

/// Soil intervention record
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Intervention {
    /// Intervention identifier
    pub id: String,
    /// Type of intervention
    #[serde(rename = "type")]
    pub intervention_type: InterventionType,
    /// Intervention name
    pub name: String,
    /// Application rate
    #[serde(skip_serializing_if = "Option::is_none")]
    pub application_rate: Option<String>,
    /// Application date
    pub application_date: String,
    /// Expected duration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration: Option<String>,
    /// Target outcomes
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub target_outcomes: Vec<String>,
    /// Notes
    #[serde(skip_serializing_if = "Option::is_none")]
    pub notes: Option<String>,
    /// Active status
    pub active: bool,
}

/// Carbon sequestration tracking
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CarbonSequestration {
    /// Measurement period start
    pub period_start: DateTime<Utc>,
    /// Measurement period end
    pub period_end: DateTime<Utc>,
    /// Baseline carbon level (tons/ha)
    pub baseline: f64,
    /// Current carbon level (tons/ha)
    pub current: f64,
    /// Net change (tons/ha)
    pub net_change: f64,
    /// Annual sequestration rate (tons/ha/year)
    pub annual_rate: f64,
    /// Confidence level (0-1)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidence: Option<f64>,
    /// Measurement method
    #[serde(skip_serializing_if = "Option::is_none")]
    pub method: Option<String>,
}

/// Metadata for soil reports
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Metadata {
    /// Standard name
    pub standard: String,
    /// Standard version
    pub version: String,
    /// Philosophy
    pub philosophy: String,
    /// Created timestamp
    #[serde(skip_serializing_if = "Option::is_none")]
    pub created_at: Option<DateTime<Utc>>,
    /// Updated timestamp
    #[serde(skip_serializing_if = "Option::is_none")]
    pub updated_at: Option<DateTime<Utc>>,
    /// Additional custom fields
    #[serde(flatten)]
    pub extra: HashMap<String, serde_json::Value>,
}

impl Default for Metadata {
    fn default() -> Self {
        Self {
            standard: "WIA-SOIL-MICROBIOME".to_string(),
            version: "1.0.0".to_string(),
            philosophy: "弘益人間".to_string(),
            created_at: None,
            updated_at: None,
            extra: HashMap::new(),
        }
    }
}

/// Complete soil microbiome report
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SoilMicrobiomeReport {
    /// JSON-LD context
    #[serde(rename = "@context")]
    pub context: Vec<String>,
    /// Report types
    #[serde(rename = "type")]
    pub report_type: Vec<String>,
    /// Unique identifier (URN)
    pub id: String,
    /// Issuer DID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub issuer: Option<String>,
    /// Issuance date
    #[serde(skip_serializing_if = "Option::is_none")]
    pub issuance_date: Option<DateTime<Utc>>,
    /// Soil sample
    pub sample: SoilSample,
    /// Microbiome profile
    pub microbiome_profile: MicrobiomeProfile,
    /// Soil health index
    #[serde(skip_serializing_if = "Option::is_none")]
    pub health_index: Option<SoilHealthIndex>,
    /// Interventions applied
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub interventions: Vec<Intervention>,
    /// Carbon sequestration data
    #[serde(skip_serializing_if = "Option::is_none")]
    pub carbon_sequestration: Option<CarbonSequestration>,
    /// Metadata
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<Metadata>,
}

// ============================================================================
// API Request/Response Types
// ============================================================================

/// Sample submission request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubmitSampleRequest {
    /// Sample information
    pub sample: SoilSample,
    /// Requested analysis methods
    pub analysis_methods: Vec<AnalysisMethod>,
}

/// Sample summary response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SampleSummary {
    /// Sample identifier
    pub id: String,
    /// Collection date
    pub collection_date: DateTime<Utc>,
    /// Soil type
    pub soil_type: SoilType,
    /// Analysis status
    pub status: SampleStatus,
    /// Created timestamp
    pub created_at: DateTime<Utc>,
    /// Last updated timestamp
    pub updated_at: DateTime<Utc>,
}

/// Analysis request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalysisRequest {
    /// Sample ID to analyze
    pub sample_id: String,
    /// Analysis method
    pub method: AnalysisMethod,
    /// Analysis parameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parameters: Option<HashMap<String, serde_json::Value>>,
}

/// Health index calculation request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalculateHealthIndexRequest {
    /// Profile ID
    pub profile_id: String,
    /// Include carbon metrics
    #[serde(skip_serializing_if = "Option::is_none")]
    pub include_carbon: Option<bool>,
    /// Include nutrient cycling
    #[serde(skip_serializing_if = "Option::is_none")]
    pub include_nutrients: Option<bool>,
}

/// Pagination parameters
#[derive(Debug, Clone, Default, Serialize)]
pub struct PaginationParams {
    /// Number of items per page
    #[serde(skip_serializing_if = "Option::is_none")]
    pub limit: Option<u32>,
    /// Offset for pagination
    #[serde(skip_serializing_if = "Option::is_none")]
    pub offset: Option<u32>,
}

/// Pagination response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pagination {
    /// Total number of items
    pub total: u64,
    /// Current limit
    pub limit: u32,
    /// Current offset
    pub offset: u32,
    /// Whether there are more items
    pub has_more: bool,
}

/// Paginated response wrapper
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PaginatedResponse<T> {
    /// Data items
    pub data: Vec<T>,
    /// Pagination info
    pub pagination: Pagination,
}

/// SDK configuration
#[derive(Debug, Clone)]
pub struct Config {
    /// API key or access token
    pub api_key: String,
    /// Environment (production or sandbox)
    pub environment: Environment,
    /// Request timeout in seconds
    pub timeout_secs: u64,
    /// Maximum retry attempts
    pub max_retries: u32,
    /// Base URL override
    pub base_url: Option<String>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            api_key: String::new(),
            environment: Environment::Production,
            timeout_secs: 30,
            max_retries: 3,
            base_url: None,
        }
    }
}
