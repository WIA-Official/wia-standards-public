//! Integration tests for Ocean Resource SDK

use wia_ocean_010_ocean_resource::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = OceanResource {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
