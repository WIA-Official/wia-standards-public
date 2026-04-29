//! Integration tests for Robotics 009 SDK

use wia_rob_009::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = Robotics {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
