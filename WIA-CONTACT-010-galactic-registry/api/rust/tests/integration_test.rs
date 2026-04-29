//! Integration tests for WIA-CONTACT-010 Galactic Registry SDK

use wia_contact_010::*;

#[test]
fn test_technology_level_serialization() {
    let level = TechnologyLevel::Interstellar;
    let json = serde_json::to_string(&level).unwrap();
    assert_eq!(json, "\"INTERSTELLAR\"");
}

#[test]
fn test_diplomatic_status() {
    let status = DiplomaticStatus::Friendly;
    assert_eq!(status, DiplomaticStatus::Friendly);
}

#[test]
fn test_kardashev_classification() {
    use wia_contact_010::utils::classify_kardashev;
    let kardashev = classify_kardashev(1e16);
    assert!((kardashev - 1.0).abs() < 0.1);
}

#[test]
fn test_distance_calculation() {
    use wia_contact_010::utils::calculate_distance_3d;
    let dist = calculate_distance_3d((0.0, 0.0, 0.0), (3.0, 4.0, 0.0));
    assert_eq!(dist, 5.0);
}

#[test]
fn test_client_creation() {
    let client = GalacticRegistryClient::new("test-key".to_string());
    assert_eq!(client.api_key, "test-key");
}
