//! Integration tests for Time Management 029 SDK

use wia_time_029::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = TimeManagement {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
