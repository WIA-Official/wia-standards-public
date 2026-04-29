//! Integration tests for Robotics 019 SDK

use wia_rob_019::*;
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
