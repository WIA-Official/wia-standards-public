//! Integration tests for Cancer Metabolism Research SDK

use wia_cancer_metabolism::*;
use uuid::Uuid;
use chrono::Utc;

#[test]
fn test_metabolic_profile_validation() {
    let profile = MetabolicProfile {
        id: Uuid::new_v4(),
        patient_id: Uuid::new_v4(),
        cancer_type: CancerType::Lung,
        metabolic_markers: vec![
            MetabolicMarker {
                name: "Glucose".to_string(),
                value: 120.0,
                unit: "mg/dL".to_string(),
                normal_range_min: 70.0,
                normal_range_max: 100.0,
                significance: MarkerSignificance::Elevated,
            }
        ],
        analysis_date: Utc::now(),
    };

    assert!(validate_metabolic_profile(&profile).is_ok());
}

#[test]
fn test_dysregulation_score() {
    let profile = MetabolicProfile {
        id: Uuid::new_v4(),
        patient_id: Uuid::new_v4(),
        cancer_type: CancerType::Breast,
        metabolic_markers: vec![
            MetabolicMarker {
                name: "Test".to_string(),
                value: 100.0,
                unit: "mg/dL".to_string(),
                normal_range_min: 70.0,
                normal_range_max: 90.0,
                significance: MarkerSignificance::Elevated,
            }
        ],
        analysis_date: Utc::now(),
    };

    let score = calculate_metabolic_dysregulation_score(&profile);
    assert!(score > 0.0);
}

#[test]
fn test_client_creation() {
    let client = Client::new("https://api.wia.global", "test-key");
    assert!(client.is_ok());
}
