//! Basic usage example for WIA Art Authentication SDK
//!
//! 弘益人間 (홍익인간) - Protecting art for all humanity

use wia_art_authentication::{Client, Artwork, ProvenanceRecord};
use uuid::Uuid;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🎨 WIA Art Authentication SDK Example");
    println!("弘益人間 - Protecting cultural heritage\n");

    let client = Client::new(
        "https://api.wia.global",
        std::env::var("WIA_API_KEY").unwrap_or_else(|_| "demo-key".to_string()),
    )?;

    let artwork = Artwork {
        id: Uuid::new_v4(),
        title: "Starry Night".to_string(),
        artist: "Vincent van Gogh".to_string(),
        year_created: 1889,
        medium: "Oil on canvas".to_string(),
        dimensions: "73.7 cm × 92.1 cm".to_string(),
        certificate_hash: "abc123".to_string(),
        provenance: vec![],
        verified: false,
    };

    println!("Artwork: {} by {}", artwork.title, artwork.artist);
    println!("Year: {}", artwork.year_created);
    println!("\n弘益人間 - Art authentication for everyone 🌍");

    Ok(())
}
