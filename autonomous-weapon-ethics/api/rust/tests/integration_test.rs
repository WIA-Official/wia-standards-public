//! Integration tests for Autonomous Weapon Ethics SDK

use autonomous_weapon_ethics::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let resource = Resource {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_resource(&resource).is_ok());
}
