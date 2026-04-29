//! Integration tests for Art Authentication SDK

use wia_art_authentication::*;
use uuid::Uuid;

#[test]
fn test_artwork_validation() {
    let artwork = Artwork {
        id: Uuid::new_v4(),
        title: "Test Art".to_string(),
        artist: "Test Artist".to_string(),
        year_created: 2025,
        medium: "Digital".to_string(),
        dimensions: "1920x1080".to_string(),
        certificate_hash: "test".to_string(),
        provenance: vec![],
        verified: true,
    };

    assert!(validate_artwork(&artwork).is_ok());
}

#[test]
fn test_client_creation() {
    let client = Client::new("https://api.wia.global", "test-key");
    assert!(client.is_ok());
}

#[test]
fn test_authenticity_score() {
    let artwork = Artwork {
        id: Uuid::new_v4(),
        title: "Test".to_string(),
        artist: "Artist".to_string(),
        year_created: 2025,
        medium: "Oil".to_string(),
        dimensions: "10x10".to_string(),
        certificate_hash: "hash".to_string(),
        provenance: vec![],
        verified: true,
    };

    let score = calculate_authenticity_score(&artwork);
    assert!(score > 0.0);
}
