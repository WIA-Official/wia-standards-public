//! Integration tests for Ocean Conservation SDK

use wia_ocean_conservation::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = OceanConservation {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
