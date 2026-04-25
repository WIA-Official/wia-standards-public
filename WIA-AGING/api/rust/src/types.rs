//! Type definitions for the WIA-AGING Standard
//!
//! This module contains all the data structures used in the WIA-AGING API.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ============================================================================
// Enums
// ============================================================================

/// Gender options for subject demographics
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum Gender {
    /// Male
    Male,
    /// Female
    Female,
    /// Other
    Other,
    /// Prefer not to say
    PreferNotToSay,
}

/// Biological age calculation methods
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum BiologicalAgeMethod {
    /// Horvath epigenetic clock
    EpigeneticHorvath,
    /// Hannum epigenetic clock
    EpigeneticHannum,
    /// GrimAge epigenetic clock
    EpigeneticGrimage,
    /// PhenoAge epigenetic clock
    EpigeneticPhenoage,
    /// Levine's Phenotypic Age
    PhenotypicLevine,
    /// Telomere length based
    TelomereLength,
    /// Transcriptomic analysis
    Transcriptomic,
    /// Proteomic analysis
    Proteomic,
    /// Composite method
    Composite,
}

/// Biomarker categories
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum BiomarkerCategory {
    /// Inflammatory markers (CRP, IL-6, etc.)
    Inflammatory,
    /// Metabolic markers (glucose, HbA1c, etc.)
    Metabolic,
    /// Organ function markers (creatinine, albumin, etc.)
    OrganFunction,
    /// Hematological markers (lymphocytes, MCV, etc.)
    Hematological,
    /// Epigenetic markers (DNA methylation)
    Epigenetic,
    /// Telomere markers
    Telomere,
    /// Hormonal markers
    Hormonal,
    /// Oxidative stress markers
    OxidativeStress,
}

/// Biomarker status based on reference ranges
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum BiomarkerStatus {
    /// Within normal range
    Normal,
    /// Below normal range
    Low,
    /// Above normal range
    High,
    /// Critically low
    CriticalLow,
    /// Critically high
    CriticalHigh,
}

/// Intervention types for longevity tracking
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum InterventionType {
    /// Supplement
    Supplement,
    /// Medication
    Medication,
    /// Exercise program
    Exercise,
    /// Dietary intervention
    Diet,
    /// Therapy
    Therapy,
    /// Lifestyle change
    Lifestyle,
    /// Medical procedure
    Procedure,
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
            Environment::Production => "https://api.aging.wia.org/v1",
            Environment::Sandbox => "https://sandbox.aging.wia.org/v1",
        }
    }

    /// Get the WebSocket URL for this environment
    pub fn ws_url(&self) -> &'static str {
        match self {
            Environment::Production => "wss://stream.aging.wia.org/v1/ws",
            Environment::Sandbox => "wss://sandbox-stream.aging.wia.org/v1/ws",
        }
    }
}

// ============================================================================
// Core Structs
// ============================================================================

/// Subject (individual being assessed)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Subject {
    /// Decentralized Identifier (DID)
    pub id: String,
    /// Chronological age in years
    pub chronological_age: f64,
    /// Date of birth (optional for privacy)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub date_of_birth: Option<String>,
    /// Gender
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gender: Option<Gender>,
    /// Self-reported ethnicity
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ethnicity: Option<String>,
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

/// Biological age calculation result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BiologicalAge {
    /// Calculated biological age in years
    pub value: f64,
    /// Method used for calculation
    pub method: BiologicalAgeMethod,
    /// Confidence score (0-1)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidence: Option<f64>,
    /// Difference from chronological age
    #[serde(skip_serializing_if = "Option::is_none")]
    pub age_difference: Option<f64>,
    /// Rate of aging (1.0 = normal)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub aging_rate: Option<f64>,
    /// Timestamp of calculation
    pub timestamp: DateTime<Utc>,
    /// Laboratory that performed the analysis
    #[serde(skip_serializing_if = "Option::is_none")]
    pub laboratory: Option<Laboratory>,
}

/// Reference range for biomarkers
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReferenceRange {
    /// Lower bound
    pub low: f64,
    /// Upper bound
    pub high: f64,
    /// Whether range is age-adjusted
    #[serde(default)]
    pub age_adjusted: bool,
}

/// Biomarker measurement
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Biomarker {
    /// LOINC code or WIA biomarker code
    pub code: String,
    /// Human-readable name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Measured value
    pub value: f64,
    /// UCUM unit code
    pub unit: String,
    /// Reference range
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reference_range: Option<ReferenceRange>,
    /// Status based on reference range
    #[serde(skip_serializing_if = "Option::is_none")]
    pub status: Option<BiomarkerStatus>,
    /// Timestamp of measurement
    pub timestamp: DateTime<Utc>,
    /// Biomarker category
    #[serde(skip_serializing_if = "Option::is_none")]
    pub category: Option<BiomarkerCategory>,
    /// Source of the measurement
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<String>,
}

impl Biomarker {
    /// Create a new biomarker with the current timestamp
    pub fn new(code: &str, value: f64, unit: &str) -> Self {
        Self {
            code: code.to_string(),
            name: None,
            value,
            unit: unit.to_string(),
            reference_range: None,
            status: None,
            timestamp: Utc::now(),
            category: None,
            source: None,
        }
    }

    /// Set the biomarker name
    pub fn with_name(mut self, name: &str) -> Self {
        self.name = Some(name.to_string());
        self
    }

    /// Set the biomarker category
    pub fn with_category(mut self, category: BiomarkerCategory) -> Self {
        self.category = Some(category);
        self
    }
}

/// Longevity intervention record
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Intervention {
    /// Intervention identifier
    pub id: String,
    /// Type of intervention
    #[serde(rename = "type")]
    pub intervention_type: InterventionType,
    /// Name of intervention
    pub name: String,
    /// Dosage (if applicable)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dosage: Option<String>,
    /// Frequency
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frequency: Option<String>,
    /// Start date
    pub start_date: String,
    /// End date (if applicable)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub end_date: Option<String>,
    /// Notes
    #[serde(skip_serializing_if = "Option::is_none")]
    pub notes: Option<String>,
    /// Active status
    pub active: bool,
}

/// Metadata for profiles and assessments
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
            standard: "WIA-AGING".to_string(),
            version: "1.0.0".to_string(),
            philosophy: "弘益人間".to_string(),
            created_at: None,
            updated_at: None,
            extra: HashMap::new(),
        }
    }
}

/// Complete aging profile
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgingProfile {
    /// JSON-LD context
    #[serde(rename = "@context")]
    pub context: Vec<String>,
    /// Profile types
    #[serde(rename = "type")]
    pub profile_type: Vec<String>,
    /// Unique identifier (URN)
    pub id: String,
    /// Issuer DID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub issuer: Option<String>,
    /// Issuance date
    #[serde(skip_serializing_if = "Option::is_none")]
    pub issuance_date: Option<DateTime<Utc>>,
    /// Subject information
    pub subject: Subject,
    /// Assessment date
    pub assessment_date: DateTime<Utc>,
    /// Biological age calculation
    #[serde(skip_serializing_if = "Option::is_none")]
    pub biological_age: Option<BiologicalAge>,
    /// Biomarker measurements
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub biomarkers: Vec<Biomarker>,
    /// Interventions
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub interventions: Vec<Intervention>,
    /// Metadata
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<Metadata>,
}

// ============================================================================
// API Request/Response Types
// ============================================================================

/// Profile creation request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CreateProfileRequest {
    /// Chronological age
    pub chronological_age: f64,
    /// Gender
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gender: Option<Gender>,
    /// Date of birth
    #[serde(skip_serializing_if = "Option::is_none")]
    pub date_of_birth: Option<String>,
}

/// Profile summary response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProfileSummary {
    /// Profile identifier
    pub id: String,
    /// Subject identifier
    pub subject_id: String,
    /// Chronological age
    pub chronological_age: f64,
    /// Latest biological age
    #[serde(skip_serializing_if = "Option::is_none")]
    pub biological_age: Option<f64>,
    /// Created timestamp
    pub created_at: DateTime<Utc>,
    /// Last assessment date
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_assessment_at: Option<DateTime<Utc>>,
}

/// Assessment creation request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CreateAssessmentRequest {
    /// Calculation method
    pub method: BiologicalAgeMethod,
    /// Biomarker data
    pub biomarkers: Vec<Biomarker>,
}

/// Assessment result with recommendations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssessmentResult {
    /// Assessment identifier
    pub id: String,
    /// Profile identifier
    pub profile_id: String,
    /// Biological age result
    pub biological_age: BiologicalAge,
    /// Health score (0-100)
    pub health_score: u8,
    /// Interpretation text
    pub interpretation: String,
    /// Recommendations
    pub recommendations: Vec<String>,
    /// Created timestamp
    pub created_at: DateTime<Utc>,
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

// ============================================================================
// Biological Age Calculator
// ============================================================================

/// Calculate biological age using Levine's Phenotypic Age formula (simplified)
///
/// # Arguments
/// * `chronological_age` - Age in years
/// * `albumin` - Albumin level (g/dL)
/// * `creatinine` - Creatinine level (mg/dL)
/// * `glucose` - Fasting glucose (mg/dL)
/// * `crp` - C-reactive protein (mg/L)
/// * `lymphocyte` - Lymphocyte percentage (%)
///
/// # Returns
/// Calculated biological age
pub fn calculate_phenotypic_age(
    chronological_age: f64,
    albumin: Option<f64>,
    creatinine: Option<f64>,
    glucose: Option<f64>,
    crp: Option<f64>,
    lymphocyte: Option<f64>,
) -> BiologicalAge {
    let mut pheno_age = chronological_age;

    if let Some(alb) = albumin {
        pheno_age += (4.5 - alb) * 5.0;
    }
    if let Some(cr) = creatinine {
        pheno_age += (cr - 0.9) * 8.0;
    }
    if let Some(glu) = glucose {
        pheno_age += (glu - 90.0) * 0.05;
    }
    if let Some(c) = crp {
        pheno_age += c * 0.8;
    }
    if let Some(lym) = lymphocyte {
        pheno_age -= (lym - 25.0) * 0.3;
    }

    pheno_age = (pheno_age * 10.0).round() / 10.0;

    BiologicalAge {
        value: pheno_age,
        method: BiologicalAgeMethod::PhenotypicLevine,
        confidence: Some(0.85),
        age_difference: Some(((pheno_age - chronological_age) * 10.0).round() / 10.0),
        aging_rate: Some(((pheno_age / chronological_age) * 100.0).round() / 100.0),
        timestamp: Utc::now(),
        laboratory: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calculate_phenotypic_age() {
        let result = calculate_phenotypic_age(
            55.0,
            Some(4.5),
            Some(0.9),
            Some(92.0),
            Some(1.0),
            Some(28.0),
        );

        assert!(result.value > 0.0);
        assert_eq!(result.method, BiologicalAgeMethod::PhenotypicLevine);
        assert!(result.confidence.unwrap() > 0.0);
    }

    #[test]
    fn test_biomarker_builder() {
        let biomarker = Biomarker::new("WIA-AGE-001", 0.8, "mg/L")
            .with_name("C-Reactive Protein")
            .with_category(BiomarkerCategory::Inflammatory);

        assert_eq!(biomarker.code, "WIA-AGE-001");
        assert_eq!(biomarker.name, Some("C-Reactive Protein".to_string()));
        assert_eq!(biomarker.category, Some(BiomarkerCategory::Inflammatory));
    }
}
