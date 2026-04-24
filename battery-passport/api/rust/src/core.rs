//! WIA Battery Passport - Core Algorithms
//!
//! Phase 2: SOH, RUL, Carbon, Second-Life, Recycling algorithms
//!
//! 홍익인간 (弘益人間) - Benefit All Humanity

use std::collections::HashMap;
use chrono::{DateTime, Utc, Duration};

use crate::types::*;
use crate::{determine_carbon_class, assess_country_risk, RiskLevel};

// ============================================================================
// Passport Manager
// ============================================================================

/// Battery Passport Manager - CRUD operations
pub struct PassportManager {
    passports: HashMap<String, BatteryPassport>,
}

impl PassportManager {
    pub fn new() -> Self {
        Self {
            passports: HashMap::new(),
        }
    }

    pub fn create(&mut self, passport: BatteryPassport) -> Result<(), PassportError> {
        if self.passports.contains_key(&passport.id) {
            return Err(PassportError::AlreadyExists(passport.id.clone()));
        }
        self.passports.insert(passport.id.clone(), passport);
        Ok(())
    }

    pub fn get(&self, id: &str) -> Option<&BatteryPassport> {
        self.passports.get(id)
    }

    pub fn get_by_qr(&self, qr_code: &str) -> Option<&BatteryPassport> {
        self.passports.values().find(|p| p.qr_code == qr_code)
    }

    pub fn update(&mut self, passport: BatteryPassport) -> Result<(), PassportError> {
        if !self.passports.contains_key(&passport.id) {
            return Err(PassportError::NotFound(passport.id.clone()));
        }
        self.passports.insert(passport.id.clone(), passport);
        Ok(())
    }

    pub fn delete(&mut self, id: &str) -> Result<(), PassportError> {
        self.passports.remove(id)
            .map(|_| ())
            .ok_or_else(|| PassportError::NotFound(id.to_string()))
    }

    pub fn list(&self) -> Vec<&BatteryPassport> {
        self.passports.values().collect()
    }

    pub fn update_health(&mut self, id: &str, health: BatteryHealth) -> Result<(), PassportError> {
        let passport = self.passports.get_mut(id)
            .ok_or_else(|| PassportError::NotFound(id.to_string()))?;
        passport.health = health;
        passport.updated_at = Utc::now();
        Ok(())
    }

    pub fn add_lifecycle_event(&mut self, id: &str, event: LifecycleEvent) -> Result<(), PassportError> {
        let passport = self.passports.get_mut(id)
            .ok_or_else(|| PassportError::NotFound(id.to_string()))?;
        passport.lifecycle.push(event);
        passport.updated_at = Utc::now();
        Ok(())
    }
}

impl Default for PassportManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Passport operation errors
#[derive(Debug, thiserror::Error)]
pub enum PassportError {
    #[error("Passport already exists: {0}")]
    AlreadyExists(String),
    #[error("Passport not found: {0}")]
    NotFound(String),
    #[error("Invalid data: {0}")]
    InvalidData(String),
}

// ============================================================================
// SOH Calculator
// ============================================================================

/// State of Health Calculator
pub struct SohCalculator;

impl SohCalculator {
    pub fn new() -> Self {
        Self
    }

    /// Calculate SOH using multiple indicators
    pub fn calculate(&self, passport: &BatteryPassport) -> SohResult {
        let health = &passport.health;
        let specs = &passport.specifications;

        // 1. Capacity-based SOH (primary metric)
        let capacity_soh = if specs.nominal_capacity_ah > 0.0 {
            (health.current_capacity_ah / specs.nominal_capacity_ah) * 100.0
        } else {
            health.state_of_health_percent
        };

        // 2. Resistance-based SOH
        let resistance_increase = health.resistance_increase_percent;
        let resistance_soh = (100.0 - (resistance_increase * 2.0)).max(0.0);

        // 3. Cycle-based SOH
        let cycle_soh = if specs.expected_cycle_life > 0 {
            (1.0 - (health.full_cycle_equivalents as f64 / specs.expected_cycle_life as f64)) * 100.0
        } else {
            100.0
        }.max(0.0);

        // Weighted combination
        let weights = (0.6, 0.25, 0.15); // capacity, resistance, cycle
        let overall_soh = capacity_soh * weights.0 + resistance_soh * weights.1 + cycle_soh * weights.2;

        // Confidence based on data source and freshness
        let confidence = self.calculate_confidence(health);

        SohResult {
            soh_percent: (overall_soh * 10.0).round() / 10.0,
            capacity_soh: (capacity_soh * 10.0).round() / 10.0,
            resistance_soh: (resistance_soh * 10.0).round() / 10.0,
            cycle_soh: (cycle_soh * 10.0).round() / 10.0,
            confidence,
            measurement_date: Utc::now(),
        }
    }

    fn calculate_confidence(&self, health: &BatteryHealth) -> ConfidenceLevel {
        match health.data_source {
            HealthDataSource::Bms => {
                if let Some(sync) = health.last_bms_sync {
                    let age = Utc::now() - sync;
                    if age < Duration::days(7) {
                        return ConfidenceLevel::High;
                    }
                }
                ConfidenceLevel::Medium
            }
            HealthDataSource::Diagnostic => ConfidenceLevel::High,
            HealthDataSource::Estimated => ConfidenceLevel::Low,
            HealthDataSource::Manual => ConfidenceLevel::Low,
        }
    }
}

impl Default for SohCalculator {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// RUL Predictor
// ============================================================================

/// Remaining Useful Life Predictor
pub struct RulPredictor;

impl RulPredictor {
    pub fn new() -> Self {
        Self
    }

    /// Predict remaining useful life
    pub fn predict(&self, passport: &BatteryPassport, usage_profile: &UsageProfile) -> RulResult {
        let health = &passport.health;
        let specs = &passport.specifications;
        let chemistry = passport.identity.chemistry;

        // Current SOH
        let current_soh = health.state_of_health_percent;

        // SOH to lose until EOL (80%)
        let soh_to_lose = current_soh - 80.0;

        if soh_to_lose <= 0.0 {
            return RulResult {
                remaining_months: 0,
                remaining_cycles: 0,
                expected_eol_date: Utc::now(),
                confidence: ConfidenceLevel::High,
                methodology: "soh_below_threshold".to_string(),
            };
        }

        // Base degradation rate by chemistry (% per 1000 cycles at 25°C)
        let base_rate = self.get_base_degradation_rate(chemistry);

        // Temperature acceleration factor (Arrhenius)
        let avg_temp = usage_profile.avg_operating_temp_c;
        let temp_factor = 2.0_f64.powf((avg_temp - 25.0) / 10.0);

        // Stress factor
        let stress_factor = self.calculate_stress_factor(health, usage_profile);

        // Adjusted degradation rate
        let adjusted_rate = base_rate * temp_factor * stress_factor;

        // Cycles remaining
        let cycles_remaining = ((soh_to_lose / adjusted_rate) * 1000.0) as u32;

        // Convert to months
        let cycles_per_year = usage_profile.annual_cycles.max(1);
        let months_remaining = ((cycles_remaining as f64 / cycles_per_year as f64) * 12.0) as u32;

        // EOL date
        let eol_date = Utc::now() + Duration::days((months_remaining * 30) as i64);

        RulResult {
            remaining_months: months_remaining,
            remaining_cycles: cycles_remaining,
            expected_eol_date: eol_date,
            confidence: ConfidenceLevel::Medium,
            methodology: "arrhenius_adjusted".to_string(),
        }
    }

    fn get_base_degradation_rate(&self, chemistry: BatteryChemistry) -> f64 {
        match chemistry {
            BatteryChemistry::Lfp => 2.0,
            BatteryChemistry::Lto => 1.5,
            BatteryChemistry::Nmc => 3.5,
            BatteryChemistry::Nca => 4.0,
            BatteryChemistry::Lco => 5.0,
            BatteryChemistry::SolidState => 1.8,
            BatteryChemistry::SodiumIon => 3.0,
            _ => 3.5,
        }
    }

    fn calculate_stress_factor(&self, health: &BatteryHealth, profile: &UsageProfile) -> f64 {
        let mut factor = 1.0;

        // Fast charging stress
        let total_charges = health.fast_charge_count + health.slow_charge_count;
        if total_charges > 0 {
            let fast_ratio = health.fast_charge_count as f64 / total_charges as f64;
            if fast_ratio > 0.5 {
                factor *= 1.0 + (fast_ratio - 0.5) * 0.3;
            }
        }

        // High temperature stress
        if health.time_above_45c_hours > 100.0 {
            factor *= 1.0 + (health.time_above_45c_hours / 1000.0).min(0.3);
        }

        // Low temperature stress
        if health.time_below_0c_hours > 50.0 {
            factor *= 1.0 + (health.time_below_0c_hours / 500.0).min(0.2);
        }

        // Deep discharge stress
        if profile.avg_depth_of_discharge > 0.8 {
            factor *= 1.0 + (profile.avg_depth_of_discharge - 0.8) * 0.5;
        }

        factor
    }
}

impl Default for RulPredictor {
    fn default() -> Self {
        Self::new()
    }
}

/// Usage profile for RUL prediction
#[derive(Debug, Clone)]
pub struct UsageProfile {
    pub avg_operating_temp_c: f64,
    pub annual_cycles: u32,
    pub avg_depth_of_discharge: f64,
}

impl Default for UsageProfile {
    fn default() -> Self {
        Self {
            avg_operating_temp_c: 25.0,
            annual_cycles: 300,
            avg_depth_of_discharge: 0.7,
        }
    }
}

// ============================================================================
// Carbon Calculator
// ============================================================================

/// Carbon Footprint Calculator
pub struct CarbonCalculator;

impl CarbonCalculator {
    pub fn new() -> Self {
        Self
    }

    /// Calculate lifecycle carbon footprint
    pub fn calculate(&self, passport: &BatteryPassport) -> CarbonFootprint {
        let specs = &passport.specifications;
        let materials = &passport.materials;

        // 1. Raw material emissions
        let material_emissions = self.calculate_material_emissions(materials);

        // 2. Manufacturing emissions
        let manufacturing_emissions = self.calculate_manufacturing_emissions(
            specs.rated_capacity_wh,
            passport.identity.chemistry,
            &passport.identity.production_country,
        );

        // 3. Transport emissions (simplified)
        let transport_emissions = self.estimate_transport_emissions(
            passport.identity.weight_kg,
            &passport.identity.production_country,
        );

        // Total
        let total = material_emissions + manufacturing_emissions + transport_emissions;
        let capacity_kwh = specs.rated_capacity_wh / 1000.0;
        let per_kwh = if capacity_kwh > 0.0 { total / capacity_kwh } else { 0.0 };

        // Carbon class
        let performance_class = determine_carbon_class(per_kwh);

        CarbonFootprint {
            total_kg_co2e: (total * 100.0).round() / 100.0,
            per_kwh_kg_co2e: (per_kwh * 100.0).round() / 100.0,
            raw_material_acquisition: (material_emissions * 100.0).round() / 100.0,
            manufacturing: (manufacturing_emissions * 100.0).round() / 100.0,
            transport: (transport_emissions * 100.0).round() / 100.0,
            performance_class,
            methodology: "ISO 14067 / EU PEF".to_string(),
            calculation_date: Utc::now(),
            third_party_verified: false,
            verifier: None,
            verification_report_url: None,
            data_quality_rating: DataQuality::Calculated,
        }
    }

    fn calculate_material_emissions(&self, materials: &MaterialComposition) -> f64 {
        // Emission factors (kg CO2e per kg material)
        let factors: HashMap<&str, (f64, f64)> = [
            ("lithium", (15.0, 0.7)),   // (virgin factor, recycled reduction)
            ("cobalt", (35.0, 0.8)),
            ("nickel", (12.0, 0.75)),
            ("manganese", (3.0, 0.6)),
            ("graphite", (4.5, 0.5)),
        ].into_iter().collect();

        let mut total = 0.0;

        for (name, info) in [
            ("lithium", &materials.lithium),
            ("cobalt", &materials.cobalt),
            ("nickel", &materials.nickel),
            ("manganese", &materials.manganese),
            ("graphite", &materials.graphite),
        ] {
            if let Some(&(virgin_factor, recycled_reduction)) = factors.get(name) {
                let recycled_pct = info.recycled_content_percent / 100.0;
                let effective_factor = virgin_factor * (1.0 - recycled_pct)
                    + virgin_factor * (1.0 - recycled_reduction) * recycled_pct;
                total += info.weight_kg * effective_factor;
            }
        }

        total
    }

    fn calculate_manufacturing_emissions(
        &self,
        capacity_wh: f64,
        chemistry: BatteryChemistry,
        country: &str,
    ) -> f64 {
        // Energy consumption per kWh capacity (kWh electricity)
        let energy_intensity = match chemistry {
            BatteryChemistry::Lfp => 45.0,
            BatteryChemistry::Nmc => 55.0,
            BatteryChemistry::Nca => 60.0,
            BatteryChemistry::Lto => 70.0,
            _ => 50.0,
        };

        // Grid emission factors by country (kg CO2e / kWh)
        let grid_factor = match country.to_uppercase().as_str() {
            "CN" => 0.58,
            "KR" => 0.42,
            "JP" => 0.45,
            "DE" => 0.35,
            "US" => 0.40,
            "SE" => 0.02,
            "NO" => 0.01,
            "PL" => 0.65,
            _ => 0.50,
        };

        let capacity_kwh = capacity_wh / 1000.0;
        capacity_kwh * energy_intensity * grid_factor
    }

    fn estimate_transport_emissions(&self, weight_kg: f64, origin_country: &str) -> f64 {
        // Simplified: assume sea transport + road
        let sea_factor = 0.016; // kg CO2e per ton-km
        let road_factor = 0.096;

        // Estimated distances
        let sea_km = match origin_country.to_uppercase().as_str() {
            "CN" => 20000.0,
            "KR" => 18000.0,
            "JP" => 18000.0,
            _ => 5000.0,
        };
        let road_km = 500.0;

        let ton = weight_kg / 1000.0;
        (ton * sea_km * sea_factor) + (ton * road_km * road_factor)
    }
}

impl Default for CarbonCalculator {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Second-Life Assessor
// ============================================================================

/// Second-Life Eligibility Assessor
pub struct SecondLifeAssessor;

impl SecondLifeAssessor {
    pub fn new() -> Self {
        Self
    }

    /// Assess second-life eligibility
    pub fn assess(&self, passport: &BatteryPassport) -> SecondLifeResult {
        let health = &passport.health;
        let specs = &passport.specifications;

        let mut checks = Vec::new();
        let mut score = 0u32;

        // 1. SOH check (sweet spot: 65-85%)
        let soh = health.state_of_health_percent;
        if (65.0..=85.0).contains(&soh) {
            checks.push(Check {
                name: "soh_range".to_string(),
                passed: true,
                message: format!("SOH {}% in acceptable range", soh),
            });
            score += 30;
        } else if (60.0..65.0).contains(&soh) {
            checks.push(Check {
                name: "soh_range".to_string(),
                passed: true,
                message: format!("SOH {}% marginal but acceptable", soh),
            });
            score += 15;
        } else {
            checks.push(Check {
                name: "soh_range".to_string(),
                passed: false,
                message: format!("SOH {}% outside acceptable range", soh),
            });
        }

        // 2. Capacity check (minimum 10 kWh for stationary)
        let remaining_capacity_wh = specs.rated_capacity_wh * (soh / 100.0);
        if remaining_capacity_wh >= 10000.0 {
            checks.push(Check {
                name: "capacity".to_string(),
                passed: true,
                message: format!("{:.1} kWh remaining", remaining_capacity_wh / 1000.0),
            });
            score += 20;
        } else {
            checks.push(Check {
                name: "capacity".to_string(),
                passed: false,
                message: format!("Only {:.1} kWh remaining", remaining_capacity_wh / 1000.0),
            });
        }

        // 3. RUL check (need 3+ years)
        if let Some(rul_months) = health.remaining_useful_life_months {
            if rul_months >= 36 {
                checks.push(Check {
                    name: "rul".to_string(),
                    passed: true,
                    message: format!("{} months remaining", rul_months),
                });
                score += 25;
            } else if rul_months >= 24 {
                checks.push(Check {
                    name: "rul".to_string(),
                    passed: true,
                    message: format!("{} months remaining (marginal)", rul_months),
                });
                score += 15;
            } else {
                checks.push(Check {
                    name: "rul".to_string(),
                    passed: false,
                    message: format!("Only {} months remaining", rul_months),
                });
            }
        }

        // 4. Safety check
        let safety_ok = health.max_temperature_reached_c < 60.0
            && health.resistance_increase_percent < 50.0;

        if safety_ok {
            checks.push(Check {
                name: "safety".to_string(),
                passed: true,
                message: "No safety concerns".to_string(),
            });
            score += 25;
        } else {
            checks.push(Check {
                name: "safety".to_string(),
                passed: false,
                message: "Safety review required".to_string(),
            });
        }

        // Eligibility and applications
        let eligible = score >= 70;
        let applications = self.suggest_applications(score, remaining_capacity_wh / 1000.0);

        // Residual value estimation
        let residual_value = self.estimate_residual_value(specs.rated_capacity_wh, soh, score);

        SecondLifeResult {
            eligible,
            score,
            checks,
            suggested_applications: applications,
            estimated_remaining_value_usd: residual_value,
            recertification_required: true,
        }
    }

    fn suggest_applications(&self, score: u32, capacity_kwh: f64) -> Vec<String> {
        let mut apps = Vec::new();

        if score >= 80 {
            apps.push("Grid-scale energy storage".to_string());
            apps.push("Commercial/industrial backup".to_string());
        }

        if score >= 70 {
            apps.push("Residential energy storage".to_string());
            apps.push("EV charging station buffer".to_string());
        }

        if score >= 60 {
            apps.push("Low-power backup systems".to_string());
            apps.push("Agricultural applications".to_string());
        }

        if capacity_kwh < 20.0 {
            apps.push("Small-scale solar storage".to_string());
            apps.push("Off-grid applications".to_string());
        }

        apps
    }

    fn estimate_residual_value(&self, capacity_wh: f64, soh: f64, score: u32) -> f64 {
        let original_value_per_kwh = 150.0; // USD/kWh assumed original

        // Non-linear depreciation
        let depreciation = 1.0 - ((100.0 - soh) / 100.0).powf(1.5);

        // Score adjustment
        let score_factor = score as f64 / 100.0;

        let capacity_kwh = capacity_wh / 1000.0;
        let original_value = capacity_kwh * original_value_per_kwh;

        (original_value * depreciation * score_factor * 100.0).round() / 100.0
    }
}

impl Default for SecondLifeAssessor {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Supply Chain Verifier
// ============================================================================

/// Supply Chain Verifier
pub struct SupplyChainVerifier;

impl SupplyChainVerifier {
    pub fn new() -> Self {
        Self
    }

    /// Verify supply chain compliance
    pub fn verify(&self, passport: &BatteryPassport) -> SupplyChainVerificationResult {
        let materials = &passport.materials;
        let mut verifications = Vec::new();
        let mut risk_score = 0.0;
        let mut max_risk = 0.0;

        for (name, info) in [
            ("cobalt", &materials.cobalt),
            ("lithium", &materials.lithium),
            ("nickel", &materials.nickel),
        ] {
            max_risk += 100.0;

            // Assess country risk
            let country_risk: f64 = info.source_countries.iter()
                .map(|c| match assess_country_risk(c) {
                    RiskLevel::High => 90.0,
                    RiskLevel::Medium => 40.0,
                    RiskLevel::Low => 10.0,
                })
                .max_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap_or(50.0);

            risk_score += country_risk;

            // Verify certification
            let cert_valid = self.verify_certification(&info.responsible_sourcing);

            verifications.push(MaterialVerification {
                material: name.to_string(),
                source_countries: info.source_countries.clone(),
                country_risk_level: if country_risk >= 70.0 { "HIGH" }
                    else if country_risk >= 40.0 { "MEDIUM" }
                    else { "LOW" }.to_string(),
                certification_valid: cert_valid,
                due_diligence_complete: info.responsible_sourcing.due_diligence_report_url.is_some(),
            });
        }

        let overall_risk = if max_risk > 0.0 { (risk_score / max_risk) * 100.0 } else { 0.0 };
        let compliant = overall_risk < 30.0 && verifications.iter().all(|v| v.certification_valid);

        let recommendations = self.generate_recommendations(&verifications, overall_risk);

        SupplyChainVerificationResult {
            overall_risk_score: (overall_risk * 10.0).round() / 10.0,
            traceability_score: 75, // Simplified
            eu_due_diligence_compliant: compliant,
            material_verifications: verifications,
            recommendations,
        }
    }

    fn verify_certification(&self, sourcing: &ResponsibleSourcing) -> bool {
        if !sourcing.certified {
            return false;
        }

        // Check audit recency (within 2 years)
        if let Some(audit_date) = sourcing.audit_date {
            let age = Utc::now() - audit_date;
            if age > Duration::days(730) {
                return false;
            }
        }

        // Check recognized schemes
        let recognized = ["RMI", "IRMA", "ASI", "LBMA", "RJC"];
        recognized.contains(&sourcing.certification_scheme.as_str())
    }

    fn generate_recommendations(&self, verifications: &[MaterialVerification], risk: f64) -> Vec<String> {
        let mut recs = Vec::new();

        for v in verifications {
            if !v.certification_valid {
                recs.push(format!("Obtain certification for {} sourcing", v.material));
            }
            if v.country_risk_level == "HIGH" {
                recs.push(format!("Diversify {} supply to lower-risk countries", v.material));
            }
        }

        if risk > 50.0 {
            recs.push("Conduct enhanced due diligence audit".to_string());
        }

        recs
    }
}

impl Default for SupplyChainVerifier {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Recycling Efficiency Calculator
// ============================================================================

/// Recycling Efficiency Calculator
pub struct RecyclingCalculator;

impl RecyclingCalculator {
    pub fn new() -> Self {
        Self
    }

    /// Calculate recycling efficiency
    pub fn calculate(
        &self,
        passport: &BatteryPassport,
        recovered: &HashMap<String, f64>,
    ) -> RecyclingEfficiencyResult {
        let materials = &passport.materials;

        // EU 2031 targets
        let targets: HashMap<&str, f64> = [
            ("cobalt", 95.0),
            ("lithium", 80.0),
            ("nickel", 95.0),
            ("copper", 95.0),
        ].into_iter().collect();

        let mut results = HashMap::new();
        let mut compliant = true;

        for (name, original) in [
            ("cobalt", &materials.cobalt),
            ("lithium", &materials.lithium),
            ("nickel", &materials.nickel),
        ] {
            if original.weight_kg == 0.0 {
                continue;
            }

            let recovered_kg = recovered.get(name).copied().unwrap_or(0.0);
            let efficiency = (recovered_kg / original.weight_kg) * 100.0;
            let target = targets.get(name).copied().unwrap_or(80.0);
            let meets_target = efficiency >= target;

            if !meets_target {
                compliant = false;
            }

            results.insert(name.to_string(), MaterialRecoveryResult {
                original_kg: original.weight_kg,
                recovered_kg,
                efficiency_percent: (efficiency * 10.0).round() / 10.0,
                target_percent: target,
                meets_target,
            });
        }

        // Overall efficiency
        let total_original: f64 = [&materials.cobalt, &materials.lithium, &materials.nickel]
            .iter().map(|m| m.weight_kg).sum();
        let total_recovered: f64 = ["cobalt", "lithium", "nickel"]
            .iter().filter_map(|n| recovered.get(*n)).sum();
        let overall = if total_original > 0.0 { (total_recovered / total_original) * 100.0 } else { 0.0 };

        let recommendations = self.generate_recommendations(&results);

        RecyclingEfficiencyResult {
            overall_efficiency_percent: (overall * 10.0).round() / 10.0,
            eu_compliant: compliant,
            material_results: results,
            process_type: "hydrometallurgical".to_string(),
            recommendations,
        }
    }

    fn generate_recommendations(&self, results: &HashMap<String, MaterialRecoveryResult>) -> Vec<String> {
        let mut recs = Vec::new();

        for (material, result) in results {
            if !result.meets_target {
                let gap = result.target_percent - result.efficiency_percent;
                recs.push(format!(
                    "Improve {} recovery by {:.1}% to meet EU target",
                    material, gap
                ));
            }
        }

        if recs.len() > 2 {
            recs.push("Consider upgrading to advanced hydrometallurgical process".to_string());
        }

        recs
    }
}

impl Default for RecyclingCalculator {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_passport_manager() {
        let mut manager = PassportManager::new();

        // Create test passport
        let passport = create_test_passport();
        let id = passport.id.clone();

        // Create
        assert!(manager.create(passport).is_ok());

        // Get
        assert!(manager.get(&id).is_some());

        // List
        assert_eq!(manager.list().len(), 1);

        // Delete
        assert!(manager.delete(&id).is_ok());
        assert!(manager.get(&id).is_none());
    }

    fn create_test_passport() -> BatteryPassport {
        BatteryPassport {
            id: uuid::Uuid::now_v7().to_string(),
            qr_code: "WIABAT:ABC123DEF456GHI7".to_string(),
            version: "1.0.0".to_string(),
            created_at: Utc::now(),
            updated_at: Utc::now(),
            identity: BatteryIdentity {
                unique_identifier: "EU-BAT-2024-0001".to_string(),
                gtin: None,
                category: BatteryCategory::Ev,
                chemistry: BatteryChemistry::Nmc,
                model: "NMC811-75".to_string(),
                serial_number: "SN12345".to_string(),
                batch_number: "BATCH001".to_string(),
                weight_kg: 450.0,
                dimensions: Dimensions {
                    length_mm: 1500.0,
                    width_mm: 1200.0,
                    height_mm: 150.0,
                    form_factor: FormFactor::Pack,
                },
                production_date: Utc::now(),
                production_country: "CN".to_string(),
                production_facility_id: "CATL-001".to_string(),
            },
            manufacturer: ManufacturerInfo {
                name: "CATL".to_string(),
                legal_name: "Contemporary Amperex Technology Co. Limited".to_string(),
                registration_number: "CN123456".to_string(),
                address: Address {
                    street: "1 Battery Road".to_string(),
                    city: "Ningde".to_string(),
                    postal_code: "352100".to_string(),
                    country: "CN".to_string(),
                },
                contact_email: "info@catl.com".to_string(),
                contact_phone: "+86-123-456-7890".to_string(),
                website: "https://catl.com".to_string(),
                eu_representative: None,
                production_facilities: vec![],
            },
            specifications: BatterySpecifications {
                nominal_voltage_v: 400.0,
                min_voltage_v: 300.0,
                max_voltage_v: 450.0,
                nominal_capacity_ah: 200.0,
                rated_capacity_wh: 75000.0,
                energy_density_wh_kg: 167.0,
                power_density_w_kg: 350.0,
                c_rate_charge: 2.0,
                c_rate_discharge: 3.0,
                round_trip_efficiency: 95.0,
                self_discharge_rate: 2.0,
                operating_temp_min_c: -20.0,
                operating_temp_max_c: 45.0,
                storage_temp_min_c: -30.0,
                storage_temp_max_c: 60.0,
                expected_cycle_life: 2000,
                calendar_life_years: 15,
                warranty_years: 8,
                warranty_cycles: 1500,
                ip_rating: "IP67".to_string(),
                un38_3_certified: true,
                thermal_runaway_protection: true,
            },
            materials: MaterialComposition {
                cobalt: MaterialInfo {
                    weight_kg: 15.0,
                    percentage: 3.3,
                    source_countries: vec!["AU".to_string()],
                    recycled_content_percent: 10.0,
                    responsible_sourcing: ResponsibleSourcing {
                        certified: true,
                        certification_scheme: "RMI".to_string(),
                        certificate_id: "RMI-2024-001".to_string(),
                        audit_date: Some(Utc::now()),
                        due_diligence_report_url: Some("https://...".to_string()),
                    },
                },
                lithium: MaterialInfo {
                    weight_kg: 8.0,
                    percentage: 1.8,
                    source_countries: vec!["CL".to_string()],
                    recycled_content_percent: 5.0,
                    responsible_sourcing: ResponsibleSourcing {
                        certified: true,
                        certification_scheme: "IRMA".to_string(),
                        certificate_id: "IRMA-2024-001".to_string(),
                        audit_date: Some(Utc::now()),
                        due_diligence_report_url: None,
                    },
                },
                nickel: MaterialInfo {
                    weight_kg: 50.0,
                    percentage: 11.1,
                    source_countries: vec!["ID".to_string()],
                    recycled_content_percent: 8.0,
                    responsible_sourcing: ResponsibleSourcing {
                        certified: true,
                        certification_scheme: "RMI".to_string(),
                        certificate_id: "RMI-2024-002".to_string(),
                        audit_date: Some(Utc::now()),
                        due_diligence_report_url: None,
                    },
                },
                manganese: MaterialInfo {
                    weight_kg: 10.0,
                    percentage: 2.2,
                    source_countries: vec!["ZA".to_string()],
                    recycled_content_percent: 5.0,
                    responsible_sourcing: ResponsibleSourcing {
                        certified: false,
                        certification_scheme: String::new(),
                        certificate_id: String::new(),
                        audit_date: None,
                        due_diligence_report_url: None,
                    },
                },
                graphite: MaterialInfo {
                    weight_kg: 80.0,
                    percentage: 17.8,
                    source_countries: vec!["CN".to_string()],
                    recycled_content_percent: 0.0,
                    responsible_sourcing: ResponsibleSourcing {
                        certified: false,
                        certification_scheme: String::new(),
                        certificate_id: String::new(),
                        audit_date: None,
                        due_diligence_report_url: None,
                    },
                },
                recycled_cobalt_percent: 10.0,
                recycled_lithium_percent: 5.0,
                recycled_nickel_percent: 8.0,
                recycled_lead_percent: 0.0,
                hazardous_substances: vec![],
                bill_of_materials: vec![],
            },
            carbon_footprint: CarbonFootprint {
                total_kg_co2e: 5250.0,
                per_kwh_kg_co2e: 70.0,
                raw_material_acquisition: 3150.0,
                manufacturing: 1575.0,
                transport: 525.0,
                performance_class: CarbonClass::C,
                methodology: "ISO 14067".to_string(),
                calculation_date: Utc::now(),
                third_party_verified: false,
                verifier: None,
                verification_report_url: None,
                data_quality_rating: DataQuality::Calculated,
            },
            health: BatteryHealth {
                state_of_health_percent: 94.5,
                state_of_charge_percent: 78.0,
                original_capacity_ah: 200.0,
                current_capacity_ah: 189.0,
                capacity_fade_percent: 5.5,
                internal_resistance_mohm: 12.5,
                resistance_increase_percent: 8.0,
                total_energy_throughput_kwh: 45000.0,
                full_cycle_equivalents: 450,
                partial_cycles: 1200,
                max_temperature_reached_c: 42.0,
                min_temperature_reached_c: -5.0,
                time_above_45c_hours: 50.0,
                time_below_0c_hours: 20.0,
                fast_charge_count: 100,
                slow_charge_count: 350,
                avg_charge_rate_c: 0.5,
                remaining_useful_life_months: Some(72),
                expected_eol_date: Some(Utc::now() + Duration::days(2190)),
                last_bms_sync: Some(Utc::now()),
                data_source: HealthDataSource::Bms,
            },
            lifecycle: vec![],
            recycling: RecyclingInfo {
                recyclability_score: 85,
                design_for_recycling: true,
                disassembly_manual_url: None,
                cobalt_recovery_target: 95.0,
                lithium_recovery_target: 80.0,
                nickel_recovery_target: 95.0,
                copper_recovery_target: 95.0,
                hazard_classification: "9".to_string(),
                special_handling_required: true,
                handling_instructions: None,
                take_back_scheme: None,
                collection_points_url: None,
            },
            certifications: vec![],
            signature: None,
        }
    }
}
