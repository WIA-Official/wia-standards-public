//! Digital Twin Simulation Example
//!
//! This example demonstrates how to use the digital twin simulator
//! for aging predictions and intervention simulations.

use chrono::{NaiveDate, Utc};
use uuid::Uuid;
use wia_health::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    println!("=== WIA Health Standard - Digital Twin Example ===\n");

    // 1. Create a health profile
    println!("1. Creating health profile...");
    let subject = Subject {
        id: Uuid::new_v4(),
        anonymized_id: Some("twin-demo-001".to_string()),
        birth_year: Some(1980),
        biological_sex: Some(BiologicalSex::Female),
        ethnicity: None,
        consent: Some(Consent {
            data_sharing: true,
            research: Some(true),
            consent_date: NaiveDate::from_ymd_opt(2025, 1, 1).unwrap(),
            version: Some("1.0".to_string()),
            expiration_date: None,
        }),
    };

    let biomarkers = BiomarkerProfile {
        aging_clocks: Some(AgingClocks {
            chronological_age: Some(45.0),
            biological_age: Some(48.0),
            clock_type: Some(AgingClockType::GrimAge),
            age_delta: Some(3.0), // 3 years older biologically
            confidence: Some(0.88),
            calculated_at: Some(Utc::now()),
            algorithm: None,
        }),
        inflammatory_markers: Some(InflammatoryMarkers {
            crp: Some(Measurement {
                value: 4.2, // Elevated
                unit: "mg/L".to_string(),
                reference_range: None,
                timestamp: Some(Utc::now()),
                method: None,
                laboratory: None,
                flags: Some(vec![MeasurementFlag::High]),
                notes: None,
            }),
            ..Default::default()
        }),
        metabolic_markers: Some(MetabolicMarkers {
            glucose: Some(Measurement {
                value: 108.0, // Prediabetic range
                unit: "mg/dL".to_string(),
                reference_range: None,
                timestamp: Some(Utc::now()),
                method: None,
                laboratory: None,
                flags: Some(vec![MeasurementFlag::Abnormal]),
                notes: None,
            }),
            ..Default::default()
        }),
        ..Default::default()
    };

    let profile = HealthProfileBuilder::new()
        .subject(subject)
        .biomarkers(biomarkers)
        .build()?;

    println!("   Subject: 45-year-old female");
    println!("   Biological Age: 48 years (+3 years)");
    println!("   CRP: 4.2 mg/L (elevated)");
    println!("   Fasting Glucose: 108 mg/dL (prediabetic)");

    // 2. Create Digital Twin Manager
    println!("\n2. Initializing Digital Twin Manager...");
    let manager = DigitalTwinManager::new();

    // Check capabilities
    let capabilities = manager.capabilities();
    println!("   Supported simulations: {:?}", capabilities.supported_simulations);
    println!("   Max timeframe: {} years", capabilities.max_timeframe_years);

    // 3. Create digital twin
    println!("\n3. Creating digital twin...");
    let twin = manager.create_twin(&profile)?;
    println!("   Twin ID: {}", twin.id);
    println!("   Type: {:?}", twin.twin_type);
    println!("   Status: {:?}", twin.status);
    println!("   Fidelity: {:?}", twin.fidelity);

    // 4. Run baseline aging simulation
    println!("\n4. Running baseline aging simulation (10 years)...");
    let baseline_result = manager.predict_aging(&profile, 10.0).await?;

    println!("   Simulation ID: {}", baseline_result.id);
    println!("   Status: {:?}", baseline_result.status);
    println!("   Duration: {} ms", baseline_result.duration_ms);
    println!("   Confidence: {:.1}%", baseline_result.confidence * 100.0);
    println!("   Predictions generated: {}", baseline_result.predictions.len());

    if let Some(bio_change) = baseline_result.outcome_summary.biological_age_change {
        println!("   Predicted biological age change: {:.1} years", bio_change);
    }

    // 5. Create intervention - Lifestyle changes
    println!("\n5. Simulating lifestyle intervention...");
    let lifestyle_intervention = Intervention {
        id: Uuid::new_v4(),
        category: InterventionCategory::Lifestyle,
        name: "Mediterranean Diet + Exercise".to_string(),
        status: InterventionStatus::Active,
        description: Some("Mediterranean diet combined with regular aerobic exercise".to_string()),
        target_mechanism: Some(vec![
            TargetMechanism::AntiInflammatory,
            TargetMechanism::Metabolic,
            TargetMechanism::Mitochondrial,
        ]),
        protocol: Some(InterventionProtocol {
            dosage: None,
            frequency: Some("daily".to_string()),
            duration: Some("ongoing".to_string()),
            route: None,
            timing: None,
            cycling: None,
        }),
        start_date: Some(NaiveDate::from_ymd_opt(2025, 1, 1).unwrap()),
        end_date: None,
        prescriber: None,
        source: None,
        outcomes: None,
        monitoring: None,
        evidence: Some(Evidence {
            level: Some(OxfordEvidenceLevel::Level1a),
            grade: Some(RecommendationGrade::A),
            references: None,
        }),
        cost: None,
        notes: None,
    };

    let lifestyle_result = manager
        .simulate_intervention(&profile, lifestyle_intervention, 10.0)
        .await?;

    println!("   Intervention: Mediterranean Diet + Exercise");
    println!("   Target mechanisms: Anti-inflammatory, Metabolic, Mitochondrial");
    println!("   Simulation confidence: {:.1}%", lifestyle_result.confidence * 100.0);

    if let Some(risk_reduction) = lifestyle_result.outcome_summary.risk_reduction {
        println!("   Predicted risk reduction: {:.1}%", risk_reduction * 100.0);
    }

    // 6. Simulate senolytic intervention
    println!("\n6. Simulating senolytic therapy...");
    let senolytic_intervention = Intervention {
        id: Uuid::new_v4(),
        category: InterventionCategory::Pharmaceutical,
        name: "Dasatinib + Quercetin".to_string(),
        status: InterventionStatus::Planned,
        description: Some("Senolytic combination therapy".to_string()),
        target_mechanism: Some(vec![
            TargetMechanism::Senolytics,
            TargetMechanism::AntiInflammatory,
        ]),
        protocol: Some(InterventionProtocol {
            dosage: Some("D: 100mg + Q: 1000mg".to_string()),
            frequency: Some("3 consecutive days per month".to_string()),
            duration: Some("6 months".to_string()),
            route: Some(AdministrationRoute::Oral),
            timing: Some("morning with food".to_string()),
            cycling: Some("3 days on, 27 days off".to_string()),
        }),
        start_date: None,
        end_date: None,
        prescriber: None,
        source: None,
        outcomes: None,
        monitoring: None,
        evidence: Some(Evidence {
            level: Some(OxfordEvidenceLevel::Level2a),
            grade: Some(RecommendationGrade::B),
            references: None,
        }),
        cost: None,
        notes: None,
    };

    let senolytic_result = manager
        .simulate_intervention(&profile, senolytic_intervention, 5.0)
        .await?;

    println!("   Intervention: Dasatinib + Quercetin");
    println!("   Protocol: 3 days/month for 6 months");
    println!("   Target: Senescent cell clearance");
    println!("   Simulation confidence: {:.1}%", senolytic_result.confidence * 100.0);

    // 7. Simulate epigenetic reprogramming (future therapy)
    println!("\n7. Simulating epigenetic reprogramming (experimental)...");
    let reprogramming_intervention = Intervention {
        id: Uuid::new_v4(),
        category: InterventionCategory::Genetic,
        name: "Partial OSK Reprogramming".to_string(),
        status: InterventionStatus::Planned,
        description: Some("Transient expression of Oct4, Sox2, Klf4 factors".to_string()),
        target_mechanism: Some(vec![
            TargetMechanism::EpigeneticReprogramming,
            TargetMechanism::TelomereExtension,
            TargetMechanism::Mitochondrial,
        ]),
        protocol: Some(InterventionProtocol {
            dosage: None,
            frequency: Some("single treatment".to_string()),
            duration: Some("7 days induction".to_string()),
            route: Some(AdministrationRoute::Injection),
            timing: None,
            cycling: None,
        }),
        start_date: None,
        end_date: None,
        prescriber: None,
        source: None,
        outcomes: None,
        monitoring: None,
        evidence: Some(Evidence {
            level: Some(OxfordEvidenceLevel::Level3),
            grade: Some(RecommendationGrade::C),
            references: None,
        }),
        cost: None,
        notes: Some("Experimental therapy - not yet approved for human use".to_string()),
    };

    let reprogramming_result = manager
        .simulate_intervention(&profile, reprogramming_intervention, 10.0)
        .await?;

    println!("   Intervention: Partial OSK Reprogramming");
    println!("   Target: Epigenetic age reversal");
    println!("   Simulation confidence: {:.1}%", reprogramming_result.confidence * 100.0);
    println!("   Note: This is experimental/theoretical simulation");

    // 8. Compare outcomes
    println!("\n8. Comparing intervention outcomes...");
    println!("   ┌─────────────────────────────┬───────────────┬────────────────┐");
    println!("   │ Intervention                │ Confidence    │ Risk Reduction │");
    println!("   ├─────────────────────────────┼───────────────┼────────────────┤");
    println!(
        "   │ Baseline (no intervention)  │ {:.1}%         │ N/A            │",
        baseline_result.confidence * 100.0
    );
    println!(
        "   │ Lifestyle Changes           │ {:.1}%         │ {:.1}%          │",
        lifestyle_result.confidence * 100.0,
        lifestyle_result.outcome_summary.risk_reduction.unwrap_or(0.0) * 100.0
    );
    println!(
        "   │ Senolytic Therapy           │ {:.1}%         │ {:.1}%          │",
        senolytic_result.confidence * 100.0,
        senolytic_result.outcome_summary.risk_reduction.unwrap_or(0.0) * 100.0
    );
    println!(
        "   │ Epigenetic Reprogramming    │ {:.1}%         │ {:.1}%          │",
        reprogramming_result.confidence * 100.0,
        reprogramming_result.outcome_summary.risk_reduction.unwrap_or(0.0) * 100.0
    );
    println!("   └─────────────────────────────┴───────────────┴────────────────┘");

    // 9. Summary
    println!("\n9. Recommendations based on simulations:");
    println!("   1. Start with lifestyle interventions (highest evidence, lowest risk)");
    println!("   2. Consider senolytic therapy after consulting longevity specialist");
    println!("   3. Monitor epigenetic reprogramming research for future options");
    println!("   4. Regular biomarker monitoring every 3-6 months");

    println!("\n=== Digital Twin Example Complete ===");
    println!("\n弘益人間 - Benefit All Humanity 🤟");

    Ok(())
}
