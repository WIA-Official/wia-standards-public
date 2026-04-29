//! Integration tests for the WIA-CHILD-006-child-data-privacy SDK

use wia_child_006_child_data_privacy_sdk::{Child006ChildDataPrivacyClient, Config, CreateChild006ChildDataPrivacyRequest};
use serde_json::json;

fn create_test_client() -> Child006ChildDataPrivacyClient {
    Child006ChildDataPrivacyClient::new(Config {
        api_key: "test-key".to_string(),
        base_url: Some("http://localhost:8080".to_string()),
        timeout_secs: 5,
    })
}

#[tokio::test]
async fn test_validation() {
    let client = create_test_client();

    let data = json!({
        "test": "data"
    });

    let result = client.validate(&data).await;
    assert!(result.is_ok());

    let validation = result.unwrap();
    assert!(validation.valid);
}

#[test]
fn test_config_default() {
    let config = Config::default();
    assert_eq!(config.timeout_secs, 30);
    assert!(config.base_url.is_none());
}

#[test]
fn test_client_creation() {
    let client = create_test_client();
    // Just verify it doesn't panic
    drop(client);
}

#[tokio::test]
async fn test_create_request_serialization() {
    let request = CreateChild006ChildDataPrivacyRequest {
        content: json!({"key": "value"}),
        metadata: Some(json!({"meta": "data"})),
    };

    let serialized = serde_json::to_string(&request).unwrap();
    assert!(serialized.contains("content"));
    assert!(serialized.contains("metadata"));
}
