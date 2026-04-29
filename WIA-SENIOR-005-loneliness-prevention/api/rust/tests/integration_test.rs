//! Integration tests for Loneliness Prevention SDK

use wia_senior_005_loneliness_prevention::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = LonelinessPrevention {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
