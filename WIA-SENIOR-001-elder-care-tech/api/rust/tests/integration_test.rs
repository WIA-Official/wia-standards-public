//! Integration tests for Elder Care Technology SDK

use wia_senior_001_elder_care_tech::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = ElderCare {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
