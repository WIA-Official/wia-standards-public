//! Integration tests for WIA Ocean Underwater Communication SDK

use wia_ocean_003_underwater_communication::*;
use chrono::Utc;

#[test]
fn test_message_validation() {
    let message = AcousticMessage {
        id: utils::generate_message_id(),
        sender_id: "AUV-001".to_string(),
        receiver_id: "BASE".to_string(),
        payload: vec![1, 2, 3],
        frequency_khz: 25.0,
        transmission_power_db: 180.0,
        timestamp: Utc::now(),
    };

    assert!(validators::validate_message(&message).is_ok());
}

#[test]
fn test_propagation_delay() {
    let delay = utils::calculate_propagation_delay(1500.0);
    assert_eq!(delay, 1.0);
}
