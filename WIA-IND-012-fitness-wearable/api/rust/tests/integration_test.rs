//! Integration tests for WIA-IND-012-fitness-wearable

use wia_ind_012_fitness_wearable::*;
use uuid::Uuid;
use chrono::Utc;

#[test]
fn test_status_serialization() {
    let status = Status::Active;
    let json = serde_json::to_string(&status).unwrap();
    assert_eq!(json, "\"active\"");
}

#[test]
fn test_resource_creation() {
    let resource = Resource {
        id: Uuid::new_v4(),
        name: "Test Resource".to_string(),
        status: Status::Active,
        created_at: Utc::now(),
        updated_at: Utc::now(),
        metadata: serde_json::json!({}),
    };
    assert_eq!(resource.name, "Test Resource");
    assert_eq!(resource.status, Status::Active);
}

#[test]
fn test_validators() {
    assert!(validators::validate_url("https://api.example.com").is_ok());
    assert!(validators::validate_url("").is_err());
    assert!(validators::validate_api_key("1234567890abcdef").is_ok());
    assert!(validators::validate_api_key("short").is_err());
}

#[test]
fn test_utils() {
    let id1 = utils::generate_id();
    let id2 = utils::generate_id();
    assert_ne!(id1, id2);
}
