//! Integration tests for Memory Assistance SDK

use wia_senior_008_memory_assistance::*;
use chrono::Utc;

#[test]
fn test_validation() {
    let item = MemoryAssistance {
        id: utils::generate_id(),
        name: "Test".to_string(),
        created_at: Utc::now(),
        status: Status::Active,
    };

    assert!(validators::validate_item(&item).is_ok());
}
