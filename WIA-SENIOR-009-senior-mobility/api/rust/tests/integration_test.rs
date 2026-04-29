//! Integration tests for Senior Mobility SDK

use wia_senior_009_senior_mobility::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = SeniorMobility {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
