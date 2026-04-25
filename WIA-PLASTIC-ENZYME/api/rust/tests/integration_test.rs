//! Integration tests for Plastic Enzyme SDK

use wia_plastic_enzyme::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = PlasticEnzyme {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
