//! Digital Twin Simulator Adapter
//!
//! Adapter for running digital twin simulations and predictions

use crate::error::{HealthError, Result};
use crate::types::*;
use async_trait::async_trait;
use chrono::Utc;
use uuid::Uuid;

/// Trait for simulation engines
#[async_trait]
pub trait SimulationEngine: Send + Sync {
    /// Run a simulation
    async fn run_simulation(&self, params: SimulationParams) -> Result<SimulationResult>;

    /// Get engine capabilities
    fn capabilities(&self) -> EngineCapabilities;
}

/// Simulation parameters
#[derive(Debug, Clone)]
pub struct SimulationParams {
    pub simulation_type: SimulationType,
    pub profile: HealthProfile,
    pub intervention: Option<Intervention>,
    pub timeframe_years: f64,
    pub parameters: serde_json::Value,
}

/// Simulation result
#[derive(Debug, Clone)]
pub struct SimulationResult {
    pub id: Uuid,
    pub status: SimulationStatus,
    pub predictions: Vec<Prediction>,
    pub outcome_summary: OutcomeSummary,
    pub confidence: f64,
    pub duration_ms: u64,
}

/// Outcome summary
#[derive(Debug, Clone)]
pub struct OutcomeSummary {
    pub biological_age_change: Option<f64>,
    pub health_score_change: Option<f64>,
    pub risk_reduction: Option<f64>,
    pub side_effects: Vec<String>,
}

/// Engine capabilities
#[derive(Debug, Clone)]
pub struct EngineCapabilities {
    pub supported_simulations: Vec<SimulationType>,
    pub max_timeframe_years: f64,
    pub supports_interventions: bool,
    pub supports_multi_organ: bool,
}

/// Basic digital twin simulator
pub struct BasicSimulator {
    capabilities: EngineCapabilities,
}

impl BasicSimulator {
    /// Create a new basic simulator
    pub fn new() -> Self {
        Self {
            capabilities: EngineCapabilities {
                supported_simulations: vec![
                    SimulationType::Aging,
                    SimulationType::Lifestyle,
                    SimulationType::Intervention,
                    SimulationType::WhatIf,
                ],
                max_timeframe_years: 50.0,
                supports_interventions: true,
                supports_multi_organ: false,
            },
        }
    }

    /// Simulate aging trajectory
    fn simulate_aging(&self, profile: &HealthProfile, years: f64) -> Result<Vec<Prediction>> {
        let mut predictions = Vec::new();

        // Get current biological age
        let current_bio_age = profile
            .biomarkers
            .as_ref()
            .and_then(|b| b.aging_clocks.as_ref())
            .and_then(|c| c.biological_age)
            .unwrap_or(50.0);

        // Simple linear aging model with some variation
        let aging_rate = 0.9; // Slightly slower than chronological

        for year in 1..=(years as i32) {
            let predicted_age = current_bio_age + (year as f64 * aging_rate);
            predictions.push(Prediction {
                outcome: Some(format!("Biological age at year {}", year)),
                probability: Some(0.85 - (year as f64 * 0.01)), // Confidence decreases over time
                timeframe: Some(format!("{} years", year)),
                confidence: Some(0.8),
                calculated_at: Some(Utc::now()),
            });
        }

        Ok(predictions)
    }

    /// Simulate intervention effect
    fn simulate_intervention(
        &self,
        profile: &HealthProfile,
        intervention: &Intervention,
        years: f64,
    ) -> Result<Vec<Prediction>> {
        let mut predictions = Vec::new();

        // Estimate intervention effect based on mechanism
        let effect_size = self.estimate_intervention_effect(intervention);

        let current_bio_age = profile
            .biomarkers
            .as_ref()
            .and_then(|b| b.aging_clocks.as_ref())
            .and_then(|c| c.biological_age)
            .unwrap_or(50.0);

        // Apply intervention effect
        let modified_rate = 1.0 - effect_size;

        for year in 1..=(years as i32) {
            let predicted_change = year as f64 * modified_rate - year as f64;
            predictions.push(Prediction {
                outcome: Some(format!(
                    "Age delta change with {} at year {}",
                    intervention.name, year
                )),
                probability: Some(0.75),
                timeframe: Some(format!("{} years", year)),
                confidence: Some(0.7),
                calculated_at: Some(Utc::now()),
            });
        }

        Ok(predictions)
    }

    /// Estimate intervention effect size
    fn estimate_intervention_effect(&self, intervention: &Intervention) -> f64 {
        // Base effect by category
        let base_effect = match intervention.category {
            InterventionCategory::Pharmaceutical => 0.15,
            InterventionCategory::Nutraceutical => 0.05,
            InterventionCategory::Lifestyle => 0.10,
            InterventionCategory::Procedure => 0.20,
            InterventionCategory::Genetic => 0.25,
            InterventionCategory::CellTherapy => 0.30,
            _ => 0.05,
        };

        // Adjust based on target mechanisms
        let mechanism_bonus: f64 = intervention
            .target_mechanism
            .as_ref()
            .map(|mechanisms| {
                mechanisms
                    .iter()
                    .map(|m| match m {
                        TargetMechanism::Senolytics => 0.10,
                        TargetMechanism::TelomereExtension => 0.15,
                        TargetMechanism::EpigeneticReprogramming => 0.20,
                        TargetMechanism::Mitochondrial => 0.08,
                        TargetMechanism::AntiInflammatory => 0.05,
                        TargetMechanism::Metabolic => 0.06,
                        TargetMechanism::Autophagy => 0.07,
                        _ => 0.03,
                    })
                    .sum()
            })
            .unwrap_or(0.0);

        (base_effect + mechanism_bonus).min(0.5) // Cap at 50% effect
    }

    /// Calculate outcome summary
    fn calculate_outcome_summary(
        &self,
        predictions: &[Prediction],
        intervention: Option<&Intervention>,
    ) -> OutcomeSummary {
        let bio_age_change = predictions.last().map(|_| -2.5); // Example: 2.5 years younger

        let health_score_change = Some(5.0); // Example: 5 point improvement

        let risk_reduction = intervention.map(|i| {
            match i.category {
                InterventionCategory::Pharmaceutical => 0.15,
                InterventionCategory::Lifestyle => 0.20,
                _ => 0.10,
            }
        });

        OutcomeSummary {
            biological_age_change: bio_age_change,
            health_score_change,
            risk_reduction,
            side_effects: Vec::new(),
        }
    }
}

impl Default for BasicSimulator {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl SimulationEngine for BasicSimulator {
    async fn run_simulation(&self, params: SimulationParams) -> Result<SimulationResult> {
        let start = std::time::Instant::now();

        // Validate timeframe
        if params.timeframe_years > self.capabilities.max_timeframe_years {
            return Err(HealthError::simulation(format!(
                "Timeframe {} years exceeds maximum {}",
                params.timeframe_years, self.capabilities.max_timeframe_years
            )));
        }

        // Run appropriate simulation
        let predictions = match params.simulation_type {
            SimulationType::Aging => self.simulate_aging(&params.profile, params.timeframe_years)?,
            SimulationType::Intervention | SimulationType::DrugResponse => {
                let intervention = params
                    .intervention
                    .as_ref()
                    .ok_or_else(|| HealthError::simulation("Intervention required"))?;
                self.simulate_intervention(&params.profile, intervention, params.timeframe_years)?
            }
            SimulationType::Lifestyle => {
                self.simulate_aging(&params.profile, params.timeframe_years)?
            }
            SimulationType::WhatIf => {
                self.simulate_aging(&params.profile, params.timeframe_years)?
            }
            _ => {
                return Err(HealthError::simulation(format!(
                    "Unsupported simulation type: {:?}",
                    params.simulation_type
                )));
            }
        };

        let outcome_summary =
            self.calculate_outcome_summary(&predictions, params.intervention.as_ref());

        let duration_ms = start.elapsed().as_millis() as u64;

        Ok(SimulationResult {
            id: Uuid::new_v4(),
            status: SimulationStatus::Completed,
            predictions,
            outcome_summary,
            confidence: 0.75,
            duration_ms,
        })
    }

    fn capabilities(&self) -> EngineCapabilities {
        self.capabilities.clone()
    }
}

/// Digital Twin Manager
pub struct DigitalTwinManager {
    engine: Box<dyn SimulationEngine>,
}

impl DigitalTwinManager {
    /// Create with default simulator
    pub fn new() -> Self {
        Self {
            engine: Box::new(BasicSimulator::new()),
        }
    }

    /// Create with custom engine
    pub fn with_engine(engine: Box<dyn SimulationEngine>) -> Self {
        Self { engine }
    }

    /// Create or update digital twin for profile
    pub fn create_twin(&self, profile: &HealthProfile) -> Result<DigitalTwinProfile> {
        Ok(DigitalTwinProfile {
            id: Uuid::new_v4(),
            version: "1.0.0".to_string(),
            twin_type: DigitalTwinType::WholeBody,
            status: DigitalTwinStatus::Active,
            fidelity: Some(FidelityLevel::Medium),
            data_streams: Some(DataStreams::default()),
            models: None,
            simulations: None,
            health_score: None,
            last_updated: Some(Utc::now()),
            next_calibration: None,
            metadata: Some(Metadata {
                version: "1.0.0".to_string(),
                created_at: Utc::now(),
                updated_at: None,
                source: Some(DataSource {
                    system: "WIA Health SDK".to_string(),
                    version: Some("1.0.0".to_string()),
                }),
            }),
        })
    }

    /// Run simulation
    pub async fn simulate(&self, params: SimulationParams) -> Result<SimulationResult> {
        self.engine.run_simulation(params).await
    }

    /// Get engine capabilities
    pub fn capabilities(&self) -> EngineCapabilities {
        self.engine.capabilities()
    }

    /// Predict aging trajectory
    pub async fn predict_aging(
        &self,
        profile: &HealthProfile,
        years: f64,
    ) -> Result<SimulationResult> {
        let params = SimulationParams {
            simulation_type: SimulationType::Aging,
            profile: profile.clone(),
            intervention: None,
            timeframe_years: years,
            parameters: serde_json::json!({}),
        };
        self.simulate(params).await
    }

    /// Simulate intervention
    pub async fn simulate_intervention(
        &self,
        profile: &HealthProfile,
        intervention: Intervention,
        years: f64,
    ) -> Result<SimulationResult> {
        let params = SimulationParams {
            simulation_type: SimulationType::Intervention,
            profile: profile.clone(),
            intervention: Some(intervention),
            timeframe_years: years,
            parameters: serde_json::json!({}),
        };
        self.simulate(params).await
    }
}

impl Default for DigitalTwinManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::NaiveDate;

    fn create_test_profile() -> HealthProfile {
        let subject = Subject {
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
        };

        HealthProfile {
            id: Uuid::new_v4(),
            version: "1.0.0".to_string(),
            subject,
            biomarkers: Some(BiomarkerProfile {
                aging_clocks: Some(AgingClocks {
                    chronological_age: Some(40.0),
                    biological_age: Some(38.0),
                    clock_type: Some(AgingClockType::GrimAge),
                    age_delta: Some(-2.0),
                    confidence: Some(0.9),
                    calculated_at: Some(Utc::now()),
                    algorithm: None,
                }),
                ..Default::default()
            }),
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

    #[tokio::test]
    async fn test_basic_simulator() {
        let simulator = BasicSimulator::new();
        let profile = create_test_profile();

        let params = SimulationParams {
            simulation_type: SimulationType::Aging,
            profile,
            intervention: None,
            timeframe_years: 10.0,
            parameters: serde_json::json!({}),
        };

        let result = simulator.run_simulation(params).await.unwrap();
        assert_eq!(result.status, SimulationStatus::Completed);
        assert!(!result.predictions.is_empty());
    }

    #[tokio::test]
    async fn test_digital_twin_manager() {
        let manager = DigitalTwinManager::new();
        let profile = create_test_profile();

        let twin = manager.create_twin(&profile).unwrap();
        assert_eq!(twin.status, DigitalTwinStatus::Active);

        let result = manager.predict_aging(&profile, 5.0).await.unwrap();
        assert_eq!(result.status, SimulationStatus::Completed);
    }
}
