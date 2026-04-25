//! Integration tests for Port Automation SDK

use wia_ocean_008_port_automation::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = PortAutomation {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
