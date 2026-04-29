//! Integration tests for Fall Detection SDK

use wia_senior_003_fall_detection::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = FallDetection {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
