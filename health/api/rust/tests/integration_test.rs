//! Integration tests for WIA Health Standard

use chrono::{NaiveDate, Utc};
use uuid::Uuid;
use wia_health::prelude::*;

fn create_test_subject() -> Subject {
    Subject {
        id: Uuid::new_v4(),
        anonymized_id: Some("test-integration-001".to_string()),
        birth_year: Some(1980),
        biological_sex: Some(BiologicalSex::Male),
        ethnicity: Some("Caucasian".to_string()),
        consent: Some(Consent {
            data_sharing: true,
            research: Some(true),
            consent_date: NaiveDate::from_ymd_opt(2025, 1, 1).unwrap(),
            version: Some("1.0".to_string()),
            expiration_date: None,
        }),
    }
}

fn create_biomarkers() -> BiomarkerProfile {
    BiomarkerProfile {
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
                method: Some("immunoassay".to_string()),
                laboratory: Some("Test Lab".to_string()),
                flags: Some(vec![MeasurementFlag::Normal]),
                notes: None,
            }),
            il6: Some(Measurement {
                value: 2.5,
                unit: "pg/mL".to_string(),
                reference_range: None,
                timestamp: Some(Utc::now()),
                method: None,
                laboratory: None,
                flags: None,
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
            hba1c: Some(Measurement {
                value: 5.4,
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
            chronological_age: Some(45.0),
            biological_age: Some(41.5),
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
    }
}

fn create_telomere_profile() -> TelomereProfile {
    TelomereProfile {
        measurements: Some(vec![TelomereMeasurement {
            id: Some(Uuid::new_v4()),
            average_length: TelomereLength {
                value: 7.2,
                unit: TelomereLengthUnit::Kilobases,
                percentile: None,
            },
            shortest_telomere: Some(TelomereLength {
                value: 4.5,
                unit: TelomereLengthUnit::Kilobases,
                percentile: Some(10.0),
            }),
            method: TelomereMeasurementMethod::QPCR,
            cell_type: Some(TelomereCellType::Leukocytes),
            timestamp: Utc::now(),
            laboratory: Some("Telomere Labs".to_string()),
            coefficient_of_variation: Some(5.2),
            quality_score: Some(QualityScore::Good),
        }]),
        telomerase_activity: Some(TelomeraseActivity {
            level: Some(TelomeraseActivityLevel::Normal),
            quantitative: Some(1.2),
            unit: Some("relative units".to_string()),
            method: Some(TelomeraseAssayMethod::TRAP),
            cell_type: Some("PBMC".to_string()),
            timestamp: Some(Utc::now()),
        }),
        age_equivalent: Some(TelomereAgeEquivalent {
            years: Some(40.0),
            percentile: Some(70.0),
            category: Some(TelomereAgeCategory::Longer),
            reference_population: Some("US Adults".to_string()),
            algorithm: Some("TeloYears v3".to_string()),
        }),
        attrition_rate: None,
        interventions: None,
    }
}

#[test]
fn test_complete_health_profile() {
    let subject = create_test_subject();
    let biomarkers = create_biomarkers();
    let telomeres = create_telomere_profile();

    let profile = HealthProfileBuilder::new()
        .subject(subject)
        .biomarkers(biomarkers)
        .telomeres(telomeres)
        .build()
        .expect("Failed to build profile");

    // Verify profile structure
    assert_eq!(profile.version, "1.0.0");
    assert!(profile.biomarkers.is_some());
    assert!(profile.telomeres.is_some());

    // Verify age delta
    let delta = profile.age_delta().unwrap();
    assert!((delta - (-3.5)).abs() < 0.01);

    // Verify biomarkers
    let bio = profile.biomarkers.as_ref().unwrap();
    let crp = bio
        .inflammatory_markers
        .as_ref()
        .unwrap()
        .crp
        .as_ref()
        .unwrap();
    assert_eq!(crp.value, 0.8);
    assert!(!BiomarkerAnalyzer::is_elevated_crp(crp.value));

    // Verify telomeres
    let telo = profile.telomeres.as_ref().unwrap();
    let measurement = telo.measurements.as_ref().unwrap().first().unwrap();
    assert_eq!(measurement.average_length.value, 7.2);
}

#[test]
fn test_serialization_roundtrip() {
    let subject = create_test_subject();
    let biomarkers = create_biomarkers();

    let profile = HealthProfileBuilder::new()
        .subject(subject)
        .biomarkers(biomarkers)
        .build()
        .expect("Failed to build profile");

    // Serialize to JSON
    let json = serde_json::to_string_pretty(&profile).expect("Failed to serialize");

    // Deserialize back
    let restored: HealthProfile = serde_json::from_str(&json).expect("Failed to deserialize");

    // Verify key fields
    assert_eq!(profile.id, restored.id);
    assert_eq!(profile.version, restored.version);

    let original_delta = profile.age_delta().unwrap();
    let restored_delta = restored.age_delta().unwrap();
    assert!((original_delta - restored_delta).abs() < 0.01);
}

#[test]
fn test_aging_clock_calculations() {
    let delta = AgingClockCalculator::calculate_age_delta(50.0, 45.0);
    assert_eq!(delta, -5.0);

    let status = AgingClockCalculator::aging_status(delta);
    assert_eq!(status, AgingStatus::Exceptional);

    let pace = AgingClockCalculator::pace_of_aging(-2.0, 5.0).unwrap();
    assert!((pace - 0.6).abs() < 0.01);
}

#[test]
fn test_biomarker_analysis() {
    // Glucose analysis
    assert_eq!(
        BiomarkerAnalyzer::glucose_status(85.0),
        GlucoseStatus::Normal
    );
    assert_eq!(
        BiomarkerAnalyzer::glucose_status(110.0),
        GlucoseStatus::Prediabetic
    );
    assert_eq!(
        BiomarkerAnalyzer::glucose_status(140.0),
        GlucoseStatus::Diabetic
    );

    // HOMA-IR calculation
    let homa_ir = BiomarkerAnalyzer::calculate_homa_ir(90.0, 8.0);
    assert!((homa_ir - 1.78).abs() < 0.1);

    let ir_status = BiomarkerAnalyzer::interpret_homa_ir(homa_ir);
    assert_eq!(ir_status, InsulinResistanceStatus::Normal);
}

#[test]
fn test_telomere_analysis() {
    // Age estimation
    let age = TelomereAnalyzer::estimate_age_from_length(8.0);
    assert!((age - 60.0).abs() < 1.0);

    // Attrition rate
    let rate = TelomereAnalyzer::calculate_attrition_rate(8.5, 7.5, 20.0).unwrap();
    assert!((rate - 50.0).abs() < 0.1);

    let category = TelomereAnalyzer::categorize_attrition(rate);
    assert_eq!(category, AttritionRateCategory::Normal);
}

#[test]
fn test_health_score_calculation() {
    let subject = create_test_subject();
    let biomarkers = create_biomarkers();
    let telomeres = create_telomere_profile();

    let profile = HealthProfileBuilder::new()
        .subject(subject)
        .biomarkers(biomarkers)
        .telomeres(telomeres)
        .build()
        .expect("Failed to build profile");

    let score = HealthScoreCalculator::calculate_overall_score(&profile);
    assert!(score.is_some());

    let score_value = score.unwrap();
    assert!(score_value >= 0.0 && score_value <= 100.0);
}

#[test]
fn test_longevity_index() {
    let subject = create_test_subject();
    let biomarkers = create_biomarkers();
    let telomeres = create_telomere_profile();

    let profile = HealthProfileBuilder::new()
        .subject(subject)
        .biomarkers(biomarkers)
        .telomeres(telomeres)
        .build()
        .expect("Failed to build profile");

    let index = LongevityIndexCalculator::calculate(&profile).expect("Failed to calculate index");

    assert!(index.overall >= 0.0 && index.overall <= 100.0);
    assert!(index.components.biological_age_score.is_some());
    assert!(index.components.telomere_score.is_some());
}

#[tokio::test]
async fn test_digital_twin_simulation() {
    let subject = create_test_subject();
    let biomarkers = create_biomarkers();

    let profile = HealthProfileBuilder::new()
        .subject(subject)
        .biomarkers(biomarkers)
        .build()
        .expect("Failed to build profile");

    let manager = DigitalTwinManager::new();

    // Create digital twin
    let twin = manager.create_twin(&profile).expect("Failed to create twin");
    assert_eq!(twin.status, DigitalTwinStatus::Active);

    // Run aging simulation
    let result = manager.predict_aging(&profile, 10.0).await.unwrap();
    assert_eq!(result.status, SimulationStatus::Completed);
    assert!(!result.predictions.is_empty());
}

#[tokio::test]
async fn test_intervention_simulation() {
    let subject = create_test_subject();
    let biomarkers = create_biomarkers();

    let profile = HealthProfileBuilder::new()
        .subject(subject)
        .biomarkers(biomarkers)
        .build()
        .expect("Failed to build profile");

    let intervention = Intervention {
        id: Uuid::new_v4(),
        category: InterventionCategory::Nutraceutical,
        name: "NMN Supplement".to_string(),
        status: InterventionStatus::Active,
        description: Some("Nicotinamide mononucleotide supplementation".to_string()),
        target_mechanism: Some(vec![TargetMechanism::Metabolic, TargetMechanism::Mitochondrial]),
        protocol: Some(InterventionProtocol {
            dosage: Some("500mg".to_string()),
            frequency: Some("daily".to_string()),
            duration: Some("ongoing".to_string()),
            route: Some(AdministrationRoute::Oral),
            timing: Some("morning".to_string()),
            cycling: None,
        }),
        start_date: Some(NaiveDate::from_ymd_opt(2025, 1, 1).unwrap()),
        end_date: None,
        prescriber: None,
        source: None,
        outcomes: None,
        monitoring: None,
        evidence: None,
        cost: None,
        notes: None,
    };

    let manager = DigitalTwinManager::new();
    let result = manager
        .simulate_intervention(&profile, intervention, 5.0)
        .await
        .unwrap();

    assert_eq!(result.status, SimulationStatus::Completed);
    assert!(result.confidence > 0.0);
}

#[test]
fn test_error_handling() {
    // Test missing subject
    let result = HealthProfileBuilder::new().build();
    assert!(result.is_err());

    let err = result.unwrap_err();
    assert!(err.is_validation() || matches!(err, HealthError::MissingField(_)));

    // Test out of range
    let err = HealthError::out_of_range("age", 200.0, 0.0, 150.0);
    assert!(err.is_recoverable());

    let err_code: ErrorCode = (&err).into();
    assert_eq!(err_code, ErrorCode::ValidationError);
}

#[test]
fn test_json_schema_compliance() {
    let subject = create_test_subject();
    let biomarkers = create_biomarkers();
    let telomeres = create_telomere_profile();

    let profile = HealthProfileBuilder::new()
        .subject(subject)
        .biomarkers(biomarkers)
        .telomeres(telomeres)
        .build()
        .expect("Failed to build profile");

    let json = serde_json::to_value(&profile).expect("Failed to serialize");

    // Verify required fields
    assert!(json.get("id").is_some());
    assert!(json.get("version").is_some());
    assert!(json.get("subject").is_some());
    assert!(json.get("metadata").is_some());

    // Verify subject fields
    let subject_json = json.get("subject").unwrap();
    assert!(subject_json.get("id").is_some());

    // Verify biomarkers structure
    let biomarkers_json = json.get("biomarkers").unwrap();
    assert!(biomarkers_json.get("aging_clocks").is_some());
}
