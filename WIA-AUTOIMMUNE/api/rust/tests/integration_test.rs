//! Integration tests for Autoimmune Disease Management SDK

use wia_autoimmune::*;
use uuid::Uuid;
use chrono::Utc;

#[test]
fn test_patient_validation() {
    let patient = Patient {
        id: Uuid::new_v4(),
        diagnosis: AutoimmuneDiseaseType::Lupus,
        symptom_severity: 5.0,
        treatment_plan: TreatmentPlan {
            id: Uuid::new_v4(),
            medications: vec![],
            lifestyle_modifications: vec![],
            monitoring_schedule: "weekly".to_string(),
        },
        last_assessment: Utc::now(),
    };

    assert!(validate_patient(&patient).is_ok());
}

#[test]
fn test_client_creation() {
    let client = Client::new("https://api.wia.global", "test-key");
    assert!(client.is_ok());
}

#[test]
fn test_severity_calculation() {
    let patient = Patient {
        id: Uuid::new_v4(),
        diagnosis: AutoimmuneDiseaseType::RheumatoidArthritis,
        symptom_severity: 8.0,
        treatment_plan: TreatmentPlan {
            id: Uuid::new_v4(),
            medications: vec![],
            lifestyle_modifications: vec![],
            monitoring_schedule: "daily".to_string(),
        },
        last_assessment: Utc::now(),
    };

    let severity = calculate_disease_severity_index(&patient);
    assert_eq!(severity, 8.0);
}
