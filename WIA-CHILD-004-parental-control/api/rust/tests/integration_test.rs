//! Integration tests

use wia_child_parental_control::*;
use uuid::Uuid;

#[test]
fn test_parental_control_validation() {
    let control = ParentalControl {
        id: Uuid::new_v4(),
        child_id: Uuid::new_v4(),
        restrictions: vec![],
        screen_time_limit_minutes: 120,
        content_filters: vec![],
    };
    assert!(validate_parental_control(&control).is_ok());
}

#[test]
fn test_screen_time_calculation() {
    let remaining = calculate_screen_time_remaining(60, 120);
    assert_eq!(remaining, 60);
}

#[test]
fn test_client_creation() {
    let client = Client::new("https://api.wia.global", "test-key");
    assert!(client.is_ok());
}
