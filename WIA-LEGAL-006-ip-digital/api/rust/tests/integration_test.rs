//! Integration tests for WIA-LEGAL-006-ip-digital

use wia_legal_006_ip_digital::*;
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
}

#[test]
fn test_validators() {
    assert!(validators::validate_url("https://api.example.com").is_ok());
    assert!(validators::validate_api_key("1234567890abcdef").is_ok());
}
