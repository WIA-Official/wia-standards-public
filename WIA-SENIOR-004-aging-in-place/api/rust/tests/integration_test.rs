//! Integration tests for Aging In Place SDK

use wia_senior_004_aging_in_place::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = AgingInPlace {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
