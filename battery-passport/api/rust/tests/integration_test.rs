//! Integration tests for Battery Passport SDK

use battery_passport::*;
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
