//! Integration tests for Marine Sensor SDK

use wia_ocean_007_marine_sensor::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = MarineSensor {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
