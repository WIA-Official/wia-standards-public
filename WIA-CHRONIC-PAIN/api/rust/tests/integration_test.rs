//! Integration tests for Chronic Pain Management SDK

use wia_chronic_pain::*;
use uuid::Uuid;
use chrono::Utc;

#[test]
fn test_pain_profile_validation() {
    let profile = PainProfile {
        id: Uuid::new_v4(),
        patient_id: Uuid::new_v4(),
        pain_type: PainType::Neuropathic,
        severity: 7.0,
        location: "Lower back".to_string(),
        duration_months: 12,
        assessment_date: Utc::now(),
    };

    assert!(validate_pain_profile(&profile).is_ok());
}

#[test]
fn test_pain_log_validation() {
    let log = PainLog {
        id: Uuid::new_v4(),
        patient_id: Uuid::new_v4(),
        pain_level: 6.0,
        timestamp: Utc::now(),
        triggers: vec!["stress".to_string()],
        relief_methods: vec!["rest".to_string()],
    };

    assert!(validate_pain_log(&log).is_ok());
}

#[test]
fn test_client_creation() {
    let client = Client::new("https://api.wia.global", "test-key");
    assert!(client.is_ok());
}

#[test]
fn test_intervention_recommendations() {
    let profile = PainProfile {
        id: Uuid::new_v4(),
        patient_id: Uuid::new_v4(),
        pain_type: PainType::Neuropathic,
        severity: 8.0,
        location: "Legs".to_string(),
        duration_months: 6,
        assessment_date: Utc::now(),
    };

    let recommendations = recommend_interventions(&profile);
    assert!(!recommendations.is_empty());
}
