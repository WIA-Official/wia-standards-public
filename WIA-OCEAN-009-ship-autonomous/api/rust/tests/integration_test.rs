//! Integration tests for Autonomous Ship SDK

use wia_ocean_009_ship_autonomous::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = AutonomousShip {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
