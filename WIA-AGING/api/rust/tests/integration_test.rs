//! Integration tests for WIA-AGING SDK
//!
//! 弘益人間 (홍익인간) - Benefit All Humanity

use wia_aging_sdk::{
    types::{
        Biomarker, BiomarkerCategory, BiomarkerStatus, BiologicalAgeMethod,
        Environment, Gender, Subject, calculate_phenotypic_age,
    },
    AgingClient,
};

#[test]
fn test_phenotypic_age_calculation() {
    let result = calculate_phenotypic_age(
        50.0,           // chronological age
        Some(4.5),      // albumin
        Some(0.9),      // creatinine
        Some(90.0),     // glucose
        Some(0.5),      // CRP
        Some(30.0),     // lymphocyte
    );

    assert!(result.value > 0.0);
    assert_eq!(result.method, BiologicalAgeMethod::PhenotypicLevine);
    assert!(result.confidence.unwrap() > 0.0);
    assert!(result.age_difference.is_some());
    assert!(result.aging_rate.is_some());
}

#[test]
fn test_phenotypic_age_with_partial_data() {
    let result = calculate_phenotypic_age(
        60.0,
        None,           // missing albumin
        Some(1.0),      // creatinine
        Some(100.0),    // glucose
        None,           // missing CRP
        Some(25.0),     // lymphocyte
    );

    assert!(result.value > 0.0);
    assert!(result.age_difference.is_some());
}

#[test]
fn test_biomarker_creation() {
    let biomarker = Biomarker::new("WIA-AGE-CRP", 1.5, "mg/L")
        .with_name("C-Reactive Protein")
        .with_category(BiomarkerCategory::Inflammatory);

    assert_eq!(biomarker.code, "WIA-AGE-CRP");
    assert_eq!(biomarker.value, 1.5);
    assert_eq!(biomarker.unit, "mg/L");
    assert_eq!(biomarker.name, Some("C-Reactive Protein".to_string()));
    assert_eq!(biomarker.category, Some(BiomarkerCategory::Inflammatory));
}

#[test]
fn test_biomarker_categories() {
    let categories = vec![
        BiomarkerCategory::Inflammatory,
        BiomarkerCategory::Metabolic,
        BiomarkerCategory::OrganFunction,
        BiomarkerCategory::Hematological,
        BiomarkerCategory::Epigenetic,
        BiomarkerCategory::Telomere,
        BiomarkerCategory::Hormonal,
        BiomarkerCategory::OxidativeStress,
    ];

    assert_eq!(categories.len(), 8);
}

#[test]
fn test_biomarker_status() {
    let statuses = vec![
        BiomarkerStatus::Normal,
        BiomarkerStatus::Low,
        BiomarkerStatus::High,
        BiomarkerStatus::CriticalLow,
        BiomarkerStatus::CriticalHigh,
    ];

    assert_eq!(statuses.len(), 5);
}

#[test]
fn test_subject_creation() {
    let subject = Subject {
        id: "did:wia:aging:test-001".to_string(),
        chronological_age: 45.0,
        date_of_birth: Some("1980-01-15".to_string()),
        gender: Some(Gender::Male),
        ethnicity: Some("Asian".to_string()),
    };

    assert_eq!(subject.chronological_age, 45.0);
    assert_eq!(subject.gender, Some(Gender::Male));
}

#[test]
fn test_environment_urls() {
    assert_eq!(
        Environment::Production.base_url(),
        "https://api.aging.wia.org/v1"
    );
    assert_eq!(
        Environment::Sandbox.base_url(),
        "https://sandbox.aging.wia.org/v1"
    );

    assert_eq!(
        Environment::Production.ws_url(),
        "wss://stream.aging.wia.org/v1/ws"
    );
    assert_eq!(
        Environment::Sandbox.ws_url(),
        "wss://sandbox-stream.aging.wia.org/v1/ws"
    );
}

#[test]
fn test_biological_age_methods() {
    let methods = vec![
        BiologicalAgeMethod::EpigeneticHorvath,
        BiologicalAgeMethod::EpigeneticHannum,
        BiologicalAgeMethod::EpigeneticGrimage,
        BiologicalAgeMethod::EpigeneticPhenoage,
        BiologicalAgeMethod::PhenotypicLevine,
        BiologicalAgeMethod::TelomereLength,
        BiologicalAgeMethod::Transcriptomic,
        BiologicalAgeMethod::Proteomic,
        BiologicalAgeMethod::Composite,
    ];

    assert_eq!(methods.len(), 9);
}

#[test]
fn test_client_builder() {
    let result = AgingClient::builder()
        .api_key("test_key")
        .environment(Environment::Sandbox)
        .timeout(60)
        .build();

    assert!(result.is_ok());
    let client = result.unwrap();
    assert_eq!(client.environment(), Environment::Sandbox);
}

#[test]
fn test_aging_rate_interpretation() {
    // Younger biological age (good)
    let young = calculate_phenotypic_age(50.0, Some(4.8), Some(0.8), Some(85.0), Some(0.3), Some(35.0));
    assert!(young.aging_rate.unwrap() < 1.0, "Should indicate slower aging");

    // Older biological age (needs improvement)
    let old = calculate_phenotypic_age(50.0, Some(4.0), Some(1.2), Some(120.0), Some(3.0), Some(18.0));
    assert!(old.aging_rate.unwrap() > 1.0, "Should indicate faster aging");
}

#[tokio::test]
async fn test_client_health_check() {
    let client = AgingClient::builder()
        .api_key("demo_key")
        .environment(Environment::Sandbox)
        .build()
        .unwrap();

    // Note: This will fail without actual API, but tests client creation
    let base_url = client.base_url();
    assert!(base_url.contains("sandbox"));
}
