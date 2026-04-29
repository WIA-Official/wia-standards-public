//! Integration tests

use wia_non_human_intelligence::*;
use uuid::Uuid;
use chrono::Utc;

#[test]
fn test_record_validation() {
    let record = Record {
        id: Uuid::new_v4(),
        timestamp: Utc::now(),
        data: "test".to_string(),
    };
    assert!(validate_record(&record).is_ok());
}

#[test]
fn test_client_creation() {
    let client = Client::new("https://api.wia.global", "test-key");
    assert!(client.is_ok());
}
