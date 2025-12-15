//! WIA Health Core Processing
//!
//! Core health data processing, calculations, and analysis

use crate::error::{HealthError, Result};
use crate::types::*;
use chrono::{DateTime, Utc};
use uuid::Uuid;

/// Health profile builder for constructing profiles
#[derive(Debug, Default)]
pub struct HealthProfileBuilder {
    subject: Option<Subject>,
    biomarkers: Option<BiomarkerProfile>,
    genomics: Option<GenomicProfile>,
    epigenetics: Option<EpigeneticProfile>,
    telomeres: Option<TelomereProfile>,
    digital_twin: Option<DigitalTwinProfile>,
    interventions: Option<InterventionHistory>,
}

impl HealthProfileBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self::default()
    }

    /// Set subject
    pub fn subject(mut self, subject: Subject) -> Self {
        self.subject = Some(subject);
        self
    }

    /// Set biomarkers
    pub fn biomarkers(mut self, biomarkers: BiomarkerProfile) -> Self {
        self.biomarkers = Some(biomarkers);
        self
    }

    /// Set genomics
    pub fn genomics(mut self, genomics: GenomicProfile) -> Self {
        self.genomics = Some(genomics);
        self
    }

    /// Set epigenetics
    pub fn epigenetics(mut self, epigenetics: EpigeneticProfile) -> Self {
        self.epigenetics = Some(epigenetics);
        self
    }

    /// Set telomeres
    pub fn telomeres(mut self, telomeres: TelomereProfile) -> Self {
        self.telomeres = Some(telomeres);
        self
    }

    /// Set digital twin
    pub fn digital_twin(mut self, digital_twin: DigitalTwinProfile) -> Self {
        self.digital_twin = Some(digital_twin);
        self
    }

    /// Set interventions
    pub fn interventions(mut self, interventions: InterventionHistory) -> Self {
        self.interventions = Some(interventions);
        self
    }

    /// Build the health profile
    pub fn build(self) -> Result<HealthProfile> {
        let subject = self
            .subject
            .ok_or_else(|| HealthError::missing_field("subject"))?;

        Ok(HealthProfile {
            id: Uuid::new_v4(),
            version: "1.0.0".to_string(),
            subject,
            biomarkers: self.biomarkers,
            genomics: self.genomics,
            epigenetics: self.epigenetics,
            telomeres: self.telomeres,
            digital_twin: self.digital_twin,
            interventions: self.interventions,
            metadata: Metadata {
                version: "1.0.0".to_string(),
                created_at: Utc::now(),
                updated_at: None,
                source: Some(DataSource {
                    system: "WIA Health SDK".to_string(),
                    version: Some("1.0.0".to_string()),
                }),
            },
        })
    }
}

/// Aging clock calculator
pub struct AgingClockCalculator;

impl AgingClockCalculator {
    /// Calculate biological age delta
    pub fn calculate_age_delta(chronological: f64, biological: f64) -> f64 {
        biological - chronological
    }

    /// Determine aging status based on delta
    pub fn aging_status(delta: f64) -> AgingStatus {
        if delta < -5.0 {
            AgingStatus::Exceptional
        } else if delta < -2.0 {
            AgingStatus::Younger
        } else if delta <= 2.0 {
            AgingStatus::Average
        } else if delta <= 5.0 {
            AgingStatus::Older
        } else {
            AgingStatus::Accelerated
        }
    }

    /// Calculate pace of aging (rate relative to chronological time)
    pub fn pace_of_aging(delta_change: f64, years_elapsed: f64) -> Result<f64> {
        if years_elapsed <= 0.0 {
            return Err(HealthError::invalid_value(
                "years_elapsed",
                "must be positive",
            ));
        }
        Ok(1.0 + (delta_change / years_elapsed))
    }
}

/// Aging status categories
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AgingStatus {
    Exceptional,
    Younger,
    Average,
    Older,
    Accelerated,
}

impl AgingStatus {
    /// Get status description
    pub fn description(&self) -> &'static str {
        match self {
            Self::Exceptional => "Exceptional: 5+ years younger than chronological age",
            Self::Younger => "Younger: 2-5 years younger than chronological age",
            Self::Average => "Average: Within 2 years of chronological age",
            Self::Older => "Older: 2-5 years older than chronological age",
            Self::Accelerated => "Accelerated: 5+ years older than chronological age",
        }
    }
}

/// Telomere analyzer
pub struct TelomereAnalyzer;

impl TelomereAnalyzer {
    /// Estimate telomere age from length (simplified model)
    pub fn estimate_age_from_length(length_kb: f64) -> f64 {
        // Simplified linear model: newborn ~11kb, loses ~50bp/year
        // Age = (11000 - length_bp) / 50
        // Converting kb to bp: length_bp = length_kb * 1000
        ((11.0 - length_kb) * 1000.0) / 50.0
    }

    /// Calculate telomere attrition rate
    pub fn calculate_attrition_rate(
        initial_length: f64,
        final_length: f64,
        years: f64,
    ) -> Result<f64> {
        if years <= 0.0 {
            return Err(HealthError::invalid_value("years", "must be positive"));
        }
        // Return bp/year (convert kb to bp)
        Ok(((initial_length - final_length) * 1000.0) / years)
    }

    /// Categorize attrition rate
    pub fn categorize_attrition(bp_per_year: f64) -> AttritionRateCategory {
        if bp_per_year < 30.0 {
            AttritionRateCategory::Slow
        } else if bp_per_year <= 70.0 {
            AttritionRateCategory::Normal
        } else {
            AttritionRateCategory::Accelerated
        }
    }
}

/// Biomarker analyzer
pub struct BiomarkerAnalyzer;

impl BiomarkerAnalyzer {
    /// Check if CRP indicates inflammation
    pub fn is_elevated_crp(crp_mg_l: f64) -> bool {
        crp_mg_l > 3.0
    }

    /// Check if glucose is in diabetic range
    pub fn glucose_status(fasting_glucose_mg_dl: f64) -> GlucoseStatus {
        if fasting_glucose_mg_dl < 100.0 {
            GlucoseStatus::Normal
        } else if fasting_glucose_mg_dl < 126.0 {
            GlucoseStatus::Prediabetic
        } else {
            GlucoseStatus::Diabetic
        }
    }

    /// Calculate HOMA-IR (insulin resistance)
    pub fn calculate_homa_ir(fasting_glucose_mg_dl: f64, fasting_insulin_uiu_ml: f64) -> f64 {
        (fasting_glucose_mg_dl * fasting_insulin_uiu_ml) / 405.0
    }

    /// Interpret HOMA-IR
    pub fn interpret_homa_ir(homa_ir: f64) -> InsulinResistanceStatus {
        if homa_ir < 1.0 {
            InsulinResistanceStatus::Optimal
        } else if homa_ir < 2.0 {
            InsulinResistanceStatus::Normal
        } else if homa_ir < 2.9 {
            InsulinResistanceStatus::EarlyResistance
        } else {
            InsulinResistanceStatus::InsulinResistant
        }
    }
}

/// Glucose status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GlucoseStatus {
    Normal,
    Prediabetic,
    Diabetic,
}

/// Insulin resistance status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InsulinResistanceStatus {
    Optimal,
    Normal,
    EarlyResistance,
    InsulinResistant,
}

/// Health score calculator
pub struct HealthScoreCalculator;

impl HealthScoreCalculator {
    /// Calculate overall health score (0-100)
    pub fn calculate_overall_score(profile: &HealthProfile) -> Option<f64> {
        let mut scores: Vec<f64> = Vec::new();
        let mut weights: Vec<f64> = Vec::new();

        // Biomarker score (weight: 0.3)
        if let Some(biomarkers) = &profile.biomarkers {
            if let Some(score) = Self::calculate_biomarker_score(biomarkers) {
                scores.push(score);
                weights.push(0.3);
            }
        }

        // Telomere score (weight: 0.2)
        if let Some(telomeres) = &profile.telomeres {
            if let Some(score) = Self::calculate_telomere_score(telomeres) {
                scores.push(score);
                weights.push(0.2);
            }
        }

        // Epigenetic score (weight: 0.25)
        if let Some(epigenetics) = &profile.epigenetics {
            if let Some(score) = Self::calculate_epigenetic_score(epigenetics) {
                scores.push(score);
                weights.push(0.25);
            }
        }

        // Digital twin score (weight: 0.25)
        if let Some(digital_twin) = &profile.digital_twin {
            if let Some(health_score) = &digital_twin.health_score {
                if let Some(overall) = health_score.overall {
                    scores.push(overall);
                    weights.push(0.25);
                }
            }
        }

        if scores.is_empty() {
            return None;
        }

        // Normalize weights
        let total_weight: f64 = weights.iter().sum();
        let weighted_sum: f64 = scores
            .iter()
            .zip(weights.iter())
            .map(|(s, w)| s * w)
            .sum();

        Some(weighted_sum / total_weight)
    }

    /// Calculate biomarker-based score
    fn calculate_biomarker_score(biomarkers: &BiomarkerProfile) -> Option<f64> {
        if let Some(clocks) = &biomarkers.aging_clocks {
            if let (Some(chrono), Some(bio)) = (clocks.chronological_age, clocks.biological_age) {
                let delta = bio - chrono;
                // Score based on age delta: -10 years = 100, +10 years = 0
                let score = 50.0 - (delta * 5.0);
                return Some(score.clamp(0.0, 100.0));
            }
        }
        None
    }

    /// Calculate telomere-based score
    fn calculate_telomere_score(telomeres: &TelomereProfile) -> Option<f64> {
        if let Some(age_eq) = &telomeres.age_equivalent {
            if let Some(percentile) = age_eq.percentile {
                return Some(percentile);
            }
        }
        None
    }

    /// Calculate epigenetic-based score
    fn calculate_epigenetic_score(epigenetics: &EpigeneticProfile) -> Option<f64> {
        if let Some(meth_age) = &epigenetics.methylation_age {
            if let Some(age) = meth_age.age {
                // Simplified: assume younger methylation age is better
                // This would need actual chronological age for proper calculation
                return Some((100.0 - age).clamp(0.0, 100.0));
            }
        }
        None
    }
}

/// Longevity index calculator
pub struct LongevityIndexCalculator;

impl LongevityIndexCalculator {
    /// Calculate comprehensive longevity index
    pub fn calculate(profile: &HealthProfile) -> Result<LongevityIndex> {
        let mut components = LongevityComponents::default();

        // Biological age component
        if let Some(biomarkers) = &profile.biomarkers {
            if let Some(clocks) = &biomarkers.aging_clocks {
                if let Some(delta) = clocks.age_delta {
                    components.biological_age_score = Some(Self::delta_to_score(delta));
                }
            }
        }

        // Telomere component
        if let Some(telomeres) = &profile.telomeres {
            if let Some(age_eq) = &telomeres.age_equivalent {
                if let Some(percentile) = age_eq.percentile {
                    components.telomere_score = Some(percentile);
                }
            }
        }

        // Metabolic component
        if let Some(biomarkers) = &profile.biomarkers {
            components.metabolic_score = Self::calculate_metabolic_score(biomarkers);
        }

        // Inflammation component
        if let Some(biomarkers) = &profile.biomarkers {
            components.inflammation_score = Self::calculate_inflammation_score(biomarkers);
        }

        // Calculate overall index
        let overall = Self::calculate_overall(&components);

        Ok(LongevityIndex {
            overall,
            components,
            calculated_at: Utc::now(),
        })
    }

    fn delta_to_score(delta: f64) -> f64 {
        // Convert age delta to 0-100 score
        // -10 years = 100, +10 years = 0
        (50.0 - (delta * 5.0)).clamp(0.0, 100.0)
    }

    fn calculate_metabolic_score(biomarkers: &BiomarkerProfile) -> Option<f64> {
        let mut scores: Vec<f64> = Vec::new();

        if let Some(metabolic) = &biomarkers.metabolic_markers {
            // Glucose score
            if let Some(glucose) = &metabolic.glucose {
                let score = if glucose.value < 100.0 {
                    100.0
                } else if glucose.value < 126.0 {
                    50.0 + (126.0 - glucose.value) * 2.0
                } else {
                    (200.0 - glucose.value).max(0.0) / 2.0
                };
                scores.push(score);
            }

            // HbA1c score
            if let Some(hba1c) = &metabolic.hba1c {
                let score = if hba1c.value < 5.7 {
                    100.0
                } else if hba1c.value < 6.5 {
                    50.0 + (6.5 - hba1c.value) * 62.5
                } else {
                    (10.0 - hba1c.value).max(0.0) * 14.3
                };
                scores.push(score);
            }
        }

        if scores.is_empty() {
            None
        } else {
            Some(scores.iter().sum::<f64>() / scores.len() as f64)
        }
    }

    fn calculate_inflammation_score(biomarkers: &BiomarkerProfile) -> Option<f64> {
        if let Some(inflammatory) = &biomarkers.inflammatory_markers {
            if let Some(crp) = &inflammatory.crp {
                // CRP < 1: excellent (100), < 3: good (50-100), > 3: elevated (0-50)
                let score = if crp.value < 1.0 {
                    100.0
                } else if crp.value < 3.0 {
                    50.0 + (3.0 - crp.value) * 25.0
                } else {
                    (10.0 - crp.value).max(0.0) * 7.14
                };
                return Some(score);
            }
        }
        None
    }

    fn calculate_overall(components: &LongevityComponents) -> f64 {
        let mut total = 0.0;
        let mut count = 0;

        if let Some(score) = components.biological_age_score {
            total += score * 1.5; // Higher weight
            count += 1;
        }
        if let Some(score) = components.telomere_score {
            total += score * 1.2;
            count += 1;
        }
        if let Some(score) = components.metabolic_score {
            total += score;
            count += 1;
        }
        if let Some(score) = components.inflammation_score {
            total += score;
            count += 1;
        }

        if count == 0 {
            50.0 // Default score
        } else {
            let weight_sum = if components.biological_age_score.is_some() {
                1.5
            } else {
                0.0
            } + if components.telomere_score.is_some() {
                1.2
            } else {
                0.0
            } + if components.metabolic_score.is_some() {
                1.0
            } else {
                0.0
            } + if components.inflammation_score.is_some() {
                1.0
            } else {
                0.0
            };
            total / weight_sum
        }
    }
}

/// Longevity index result
#[derive(Debug, Clone)]
pub struct LongevityIndex {
    pub overall: f64,
    pub components: LongevityComponents,
    pub calculated_at: DateTime<Utc>,
}

/// Longevity index components
#[derive(Debug, Clone, Default)]
pub struct LongevityComponents {
    pub biological_age_score: Option<f64>,
    pub telomere_score: Option<f64>,
    pub metabolic_score: Option<f64>,
    pub inflammation_score: Option<f64>,
    pub epigenetic_score: Option<f64>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::NaiveDate;

    fn create_test_subject() -> Subject {
        Subject {
            id: Uuid::new_v4(),
            anonymized_id: Some("test-001".to_string()),
            birth_year: Some(1985),
            biological_sex: Some(BiologicalSex::Male),
            ethnicity: None,
            consent: Some(Consent {
                data_sharing: true,
                research: Some(true),
                consent_date: NaiveDate::from_ymd_opt(2025, 1, 1).unwrap(),
                version: Some("1.0".to_string()),
                expiration_date: None,
            }),
        }
    }

    #[test]
    fn test_profile_builder() {
        let subject = create_test_subject();
        let profile = HealthProfileBuilder::new()
            .subject(subject)
            .build()
            .unwrap();

        assert_eq!(profile.version, "1.0.0");
        assert!(profile.biomarkers.is_none());
    }

    #[test]
    fn test_aging_clock_calculator() {
        let delta = AgingClockCalculator::calculate_age_delta(40.0, 36.0);
        assert_eq!(delta, -4.0);

        let status = AgingClockCalculator::aging_status(delta);
        assert_eq!(status, AgingStatus::Younger);
    }

    #[test]
    fn test_telomere_analyzer() {
        let age = TelomereAnalyzer::estimate_age_from_length(7.0);
        assert!((age - 80.0).abs() < 1.0);

        let rate = TelomereAnalyzer::calculate_attrition_rate(8.0, 7.0, 10.0).unwrap();
        assert!((rate - 100.0).abs() < 0.1);
    }

    #[test]
    fn test_biomarker_analyzer() {
        assert!(!BiomarkerAnalyzer::is_elevated_crp(1.0));
        assert!(BiomarkerAnalyzer::is_elevated_crp(5.0));

        let status = BiomarkerAnalyzer::glucose_status(95.0);
        assert_eq!(status, GlucoseStatus::Normal);

        let homa_ir = BiomarkerAnalyzer::calculate_homa_ir(100.0, 10.0);
        assert!((homa_ir - 2.47).abs() < 0.1);
    }
}
