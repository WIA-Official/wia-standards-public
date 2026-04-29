//! Integration tests for Dementia Care SDK

use wia_senior_002_dementia_care::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = DementiaCare {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
