//! Protocol and Transport Integration Tests
//!
//! Tests for WIA Material Protocol Phase 3 implementation.

use wia_material::prelude::*;
use wia_material::{MaterialType, VERSION};

/// Test protocol message creation
#[test]
fn test_protocol_message_creation() {
    let query = QueryPayload {
        material_type: Some(MaterialType::Superconductor),
        filter: None,
        sort: None,
        pagination: Some(Pagination { offset: 0, limit: 50 }),
        fields: None,
    };

    let message = MessageBuilder::query(query);
    assert_eq!(message.message_type, MessageType::Query);
    assert_eq!(message.protocol, "wia-material");
    assert_eq!(message.version, "1.0.0");
}

/// Test filter builder
#[test]
fn test_filter_builder() {
    let filter = FilterBuilder::new()
        .eq("material_type", "superconductor")
        .gt("properties.superconductor.critical_temperature_k", 77.0)
        .build_and()
        .unwrap();

    let json = serde_json::to_string(&filter).unwrap();
    assert!(json.contains("superconductor"));
    assert!(json.contains("77"));
}

/// Test filter conditions
#[test]
fn test_filter_conditions() {
    let eq = FilterCondition::eq("field", "value");
    assert_eq!(eq.operator, FilterOperator::Eq);

    let gt = FilterCondition::gt("temp", 100.0);
    assert_eq!(gt.operator, FilterOperator::Gt);

    let contains = FilterCondition::contains("name", "YBCO");
    assert_eq!(contains.operator, FilterOperator::Contains);
}

/// Test message serialization
#[test]
fn test_message_serialization() {
    let ping = MessageBuilder::ping();
    let json = ping.to_json().unwrap();

    assert!(json.contains("ping"));
    assert!(json.contains("wia-material"));
    assert!(json.contains("1.0.0"));
}

/// Test transport configuration
#[test]
fn test_transport_config() {
    let config = TransportConfig::new("https://api.example.com")
        .with_api_key("test-api-key")
        .with_max_retries(5);

    assert_eq!(config.base_url, "https://api.example.com");
    assert_eq!(config.api_key, Some("test-api-key".to_string()));
    assert_eq!(config.max_retries, 5);
}

/// Test transport URL generation
#[test]
fn test_transport_urls() {
    let config = TransportConfig::new("https://api.example.com");

    assert_eq!(
        config.materials_url(),
        "https://api.example.com/api/v1/materials"
    );
    assert_eq!(
        config.stream_url(),
        "wss://api.example.com/api/v1/stream"
    );
}

/// Test WebSocket configuration
#[test]
fn test_websocket_config() {
    let config = WebSocketConfig::new("wss://api.example.com")
        .with_api_key("ws-api-key")
        .without_auto_reconnect();

    assert!(!config.auto_reconnect);
    assert_eq!(config.transport.api_key, Some("ws-api-key".to_string()));
}

/// Test HTTP transport connection
#[tokio::test]
async fn test_http_transport_connection() {
    let config = TransportConfig::new("https://api.example.com");
    let mut transport = HttpTransport::new(config);

    assert!(!transport.is_connected());

    transport.connect().await.unwrap();
    assert!(transport.is_connected());

    transport.disconnect().await.unwrap();
    assert!(!transport.is_connected());
}

/// Test HTTP transport CRUD operations
#[tokio::test]
async fn test_http_transport_crud() {
    let config = TransportConfig::new("https://api.example.com");
    let mut transport = HttpTransport::new(config);
    transport.connect().await.unwrap();

    // Create material
    let material = MaterialBuilder::new()
        .material_type(MaterialType::Superconductor)
        .name("Test YBCO")
        .formula("YBa2Cu3O7")
        .build()
        .unwrap();

    let create_result = transport
        .create(wia_material::protocol::CreatePayload {
            material: material.clone(),
        })
        .await
        .unwrap();

    assert_eq!(create_result.data.identity.name, "Test YBCO");

    // Query
    let query_result = transport.query(QueryPayload::default()).await.unwrap();
    assert_eq!(query_result.data.len(), 1);
}

/// Test WebSocket transport
#[tokio::test]
async fn test_websocket_transport() {
    let config = WebSocketConfig::new("wss://api.example.com");
    let mut transport = WebSocketTransport::new(config);

    assert_eq!(transport.state().await, ConnectionState::Disconnected);

    transport.connect().await.unwrap();
    assert_eq!(transport.state().await, ConnectionState::Connected);
    assert!(transport.session_id().await.is_some());

    // Subscribe
    let sub_ack = transport
        .subscribe(wia_material::protocol::SubscribePayload {
            channel: SubscriptionChannel::Measurements,
            material_type: Some(MaterialType::Superconductor),
            filter: None,
        })
        .await
        .unwrap();

    assert_eq!(sub_ack.channel, SubscriptionChannel::Measurements);

    let subs = transport.subscriptions().await;
    assert_eq!(subs.len(), 1);

    // Unsubscribe
    transport
        .unsubscribe(wia_material::protocol::UnsubscribePayload {
            subscription_id: sub_ack.subscription_id,
        })
        .await
        .unwrap();

    let subs = transport.subscriptions().await;
    assert_eq!(subs.len(), 0);

    transport.disconnect().await.unwrap();
}

/// Test data streaming simulation
#[tokio::test]
async fn test_data_streaming() {
    let config = WebSocketConfig::new("wss://api.example.com");
    let mut transport = WebSocketTransport::new(config);
    transport.connect().await.unwrap();

    // Simulate incoming data
    let material = MaterialBuilder::new()
        .material_type(MaterialType::TopologicalInsulator)
        .name("Bi2Se3")
        .formula("Bi2Se3")
        .build()
        .unwrap();

    transport
        .simulate_data(material, SubscriptionChannel::Measurements)
        .await;

    // Receive the message
    let msg = transport.receive().await.unwrap();
    assert!(msg.contains("Bi2Se3"));
    assert!(msg.contains("measurements"));
}

/// Test error handling
#[tokio::test]
async fn test_transport_error_handling() {
    let config = TransportConfig::new("https://api.example.com");
    let transport = HttpTransport::new(config);

    // Should fail when not connected
    let result = transport.query(QueryPayload::default()).await;
    assert!(result.is_err());
}

/// Test sort configuration
#[test]
fn test_sort_config() {
    let sort = SortConfig {
        field: "timestamp".to_string(),
        order: SortOrder::Desc,
    };

    let json = serde_json::to_string(&sort).unwrap();
    assert!(json.contains("timestamp"));
    assert!(json.contains("desc"));
}

/// Test pagination
#[test]
fn test_pagination() {
    let pagination = Pagination {
        offset: 100,
        limit: 50,
    };

    assert_eq!(pagination.offset, 100);
    assert_eq!(pagination.limit, 50);

    let default = Pagination::default();
    assert_eq!(default.offset, 0);
    assert_eq!(default.limit, 100);
}

/// Test version constant
#[test]
fn test_version() {
    assert_eq!(VERSION, "1.0.0");
}
