//! Integration tests for Intergenerational SDK

use wia_senior_010_intergenerational::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = Intergenerational {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
