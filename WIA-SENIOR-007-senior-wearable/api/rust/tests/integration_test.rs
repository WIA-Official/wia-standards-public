//! Integration tests for Senior Wearable SDK

use wia_senior_007_senior_wearable::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = SeniorWearable {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
