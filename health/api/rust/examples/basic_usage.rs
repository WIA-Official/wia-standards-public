//! Basic Usage Example for WIA Health Standard
//!
//! This example demonstrates how to create and work with health profiles.

use chrono::{NaiveDate, Utc};
use uuid::Uuid;
use wia_health::prelude::*;

fn main() -> Result<()> {
    println!("=== WIA Health Standard - Basic Usage Example ===\n");

    // 1. Create a subject
    println!("1. Creating subject...");
    let subject = Subject {
        id: Uuid::new_v4(),
        anonymized_id: Some("patient-001".to_string()),
        birth_year: Some(1985),
        biological_sex: Some(BiologicalSex::Male),
        ethnicity: Some("Asian".to_string()),
        consent: Some(Consent {
            data_sharing: true,
            research: Some(true),
            consent_date: NaiveDate::from_ymd_opt(2025, 1, 1).unwrap(),
            version: Some("1.0".to_string()),
            expiration_date: None,
        }),
    };
    println!("   Subject ID: {}", subject.id);

    // 2. Create biomarker profile with aging clocks
    println!("\n2. Creating biomarker profile...");
    let biomarkers = BiomarkerProfile {
        inflammatory_markers: Some(InflammatoryMarkers {
            crp: Some(Measurement {
                value: 0.8,
                unit: "mg/L".to_string(),
                reference_range: Some(ReferenceRange {
                    low: Some(0.0),
                    high: Some(3.0),
                    unit: Some("mg/L".to_string()),
                    population: None,
                }),
                timestamp: Some(Utc::now()),
                method: Some("high-sensitivity immunoassay".to_string()),
                laboratory: Some("Quest Diagnostics".to_string()),
                flags: Some(vec![MeasurementFlag::Normal]),
                notes: None,
            }),
            il6: Some(Measurement {
                value: 2.1,
                unit: "pg/mL".to_string(),
                reference_range: None,
                timestamp: Some(Utc::now()),
                method: None,
                laboratory: None,
                flags: Some(vec![MeasurementFlag::Normal]),
                notes: None,
            }),
            ..Default::default()
        }),
        metabolic_markers: Some(MetabolicMarkers {
            glucose: Some(Measurement {
                value: 92.0,
                unit: "mg/dL".to_string(),
                reference_range: Some(ReferenceRange {
                    low: Some(70.0),
                    high: Some(100.0),
                    unit: Some("mg/dL".to_string()),
                    population: None,
                }),
                timestamp: Some(Utc::now()),
                method: None,
                laboratory: None,
                flags: Some(vec![MeasurementFlag::Normal]),
                notes: None,
            }),
            insulin: Some(Measurement {
                value: 8.5,
                unit: "μIU/mL".to_string(),
                reference_range: None,
                timestamp: Some(Utc::now()),
                method: None,
                laboratory: None,
                flags: None,
                notes: None,
            }),
            hba1c: Some(Measurement {
                value: 5.2,
                unit: "%".to_string(),
                reference_range: Some(ReferenceRange {
                    low: Some(4.0),
                    high: Some(5.7),
                    unit: Some("%".to_string()),
                    population: None,
                }),
                timestamp: Some(Utc::now()),
                method: None,
                laboratory: None,
                flags: Some(vec![MeasurementFlag::Normal]),
                notes: None,
            }),
            ..Default::default()
        }),
        aging_clocks: Some(AgingClocks {
            chronological_age: Some(40.0),
            biological_age: Some(36.5),
            clock_type: Some(AgingClockType::GrimAge),
            age_delta: Some(-3.5),
            confidence: Some(0.92),
            calculated_at: Some(Utc::now()),
            algorithm: Some(AgingClockAlgorithm {
                name: "GrimAge v2".to_string(),
                version: Some("2.0".to_string()),
                provider: Some("Elysium Health".to_string()),
                cpg_sites: Some(1030),
            }),
        }),
        ..Default::default()
    };
    println!("   CRP: 0.8 mg/L (Normal)");
    println!("   Glucose: 92 mg/dL (Normal)");
    println!("   HbA1c: 5.2% (Normal)");

    // 3. Create telomere profile
    println!("\n3. Creating telomere profile...");
    let telomeres = TelomereProfile {
        measurements: Some(vec![TelomereMeasurement {
            id: Some(Uuid::new_v4()),
            average_length: TelomereLength {
                value: 7.5,
                unit: TelomereLengthUnit::Kilobases,
                percentile: None,
            },
            shortest_telomere: Some(TelomereLength {
                value: 5.2,
                unit: TelomereLengthUnit::Kilobases,
                percentile: Some(15.0),
            }),
            method: TelomereMeasurementMethod::QPCR,
            cell_type: Some(TelomereCellType::Leukocytes),
            timestamp: Utc::now(),
            laboratory: Some("Telomere Diagnostics Inc.".to_string()),
            coefficient_of_variation: Some(4.8),
            quality_score: Some(QualityScore::Excellent),
        }]),
        telomerase_activity: Some(TelomeraseActivity {
            level: Some(TelomeraseActivityLevel::Normal),
            quantitative: Some(1.1),
            unit: Some("relative units".to_string()),
            method: Some(TelomeraseAssayMethod::TRAP),
            cell_type: Some("PBMC".to_string()),
            timestamp: Some(Utc::now()),
        }),
        age_equivalent: Some(TelomereAgeEquivalent {
            years: Some(35.0),
            percentile: Some(75.0),
            category: Some(TelomereAgeCategory::Longer),
            reference_population: Some("US Adults 35-45".to_string()),
            algorithm: Some("TeloYears v3".to_string()),
        }),
        attrition_rate: None,
        interventions: None,
    };
    println!("   Average Telomere Length: 7.5 kb");
    println!("   Telomere Age: 35 years (75th percentile)");

    // 4. Build the complete health profile
    println!("\n4. Building health profile...");
    let profile = HealthProfileBuilder::new()
        .subject(subject)
        .biomarkers(biomarkers)
        .telomeres(telomeres)
        .build()?;

    println!("   Profile ID: {}", profile.id);
    println!("   Version: {}", profile.version);

    // 5. Analyze the profile
    println!("\n5. Analyzing profile...");

    // Age delta analysis
    if let Some(delta) = profile.age_delta() {
        let status = AgingClockCalculator::aging_status(delta);
        println!("   Biological Age Delta: {:.1} years", delta);
        println!("   Status: {}", status.description());
    }

    // CRP analysis
    if let Some(crp) = profile
        .biomarkers
        .as_ref()
        .and_then(|b| b.inflammatory_markers.as_ref())
        .and_then(|i| i.crp.as_ref())
    {
        let elevated = BiomarkerAnalyzer::is_elevated_crp(crp.value);
        println!(
            "   Inflammation Status: {}",
            if elevated { "Elevated" } else { "Normal" }
        );
    }

    // Glucose analysis
    if let Some(glucose) = profile
        .biomarkers
        .as_ref()
        .and_then(|b| b.metabolic_markers.as_ref())
        .and_then(|m| m.glucose.as_ref())
    {
        let status = BiomarkerAnalyzer::glucose_status(glucose.value);
        println!("   Glucose Status: {:?}", status);
    }

    // HOMA-IR calculation
    if let (Some(glucose), Some(insulin)) = (
        profile
            .biomarkers
            .as_ref()
            .and_then(|b| b.metabolic_markers.as_ref())
            .and_then(|m| m.glucose.as_ref()),
        profile
            .biomarkers
            .as_ref()
            .and_then(|b| b.metabolic_markers.as_ref())
            .and_then(|m| m.insulin.as_ref()),
    ) {
        let homa_ir = BiomarkerAnalyzer::calculate_homa_ir(glucose.value, insulin.value);
        let ir_status = BiomarkerAnalyzer::interpret_homa_ir(homa_ir);
        println!("   HOMA-IR: {:.2} ({:?})", homa_ir, ir_status);
    }

    // Telomere analysis
    if let Some(measurements) = &profile.telomeres.as_ref().and_then(|t| t.measurements.as_ref()) {
        if let Some(measurement) = measurements.first() {
            let estimated_age =
                TelomereAnalyzer::estimate_age_from_length(measurement.average_length.value);
            println!(
                "   Telomere-based Age Estimate: {:.1} years",
                estimated_age
            );
        }
    }

    // 6. Calculate health scores
    println!("\n6. Calculating scores...");

    if let Some(score) = HealthScoreCalculator::calculate_overall_score(&profile) {
        println!("   Overall Health Score: {:.1}/100", score);
    }

    let longevity_index =
        LongevityIndexCalculator::calculate(&profile).expect("Failed to calculate longevity index");
    println!("   Longevity Index: {:.1}/100", longevity_index.overall);

    if let Some(bio_score) = longevity_index.components.biological_age_score {
        println!("   - Biological Age Score: {:.1}", bio_score);
    }
    if let Some(telo_score) = longevity_index.components.telomere_score {
        println!("   - Telomere Score: {:.1}", telo_score);
    }
    if let Some(metab_score) = longevity_index.components.metabolic_score {
        println!("   - Metabolic Score: {:.1}", metab_score);
    }
    if let Some(inflam_score) = longevity_index.components.inflammation_score {
        println!("   - Inflammation Score: {:.1}", inflam_score);
    }

    // 7. Serialize to JSON
    println!("\n7. Serializing to JSON...");
    let json = serde_json::to_string_pretty(&profile)?;
    println!("   JSON size: {} bytes", json.len());
    println!("   First 500 characters:\n{}", &json[..json.len().min(500)]);

    println!("\n=== Example Complete ===");
    println!("\n弘益人間 - Benefit All Humanity 🤟");

    Ok(())
}
