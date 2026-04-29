//! Integration tests for Organ Shortage SDK

use wia_organ_shortage::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = OrganShortage {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
