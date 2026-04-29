//! Integration tests for Submarine Technology SDK

use wia_ocean_006_submarine_tech::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = SubmarineTech {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
