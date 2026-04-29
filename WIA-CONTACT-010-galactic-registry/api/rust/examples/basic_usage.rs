//! Basic usage example for WIA-CONTACT-010 Galactic Registry SDK
//!
//! 홍익인간 (弘益人間) - Benefit All Humanity through Galactic Cooperation

use wia_contact_010::{GalacticRegistryClient, Civilization, TechnologyLevel, DiplomaticStatus, GalacticLocation, CivilizationMetadata};
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🌌 WIA-CONTACT-010 Galactic Registry Demo");
    println!("==========================================\n");

    let api_key = std::env::var("WIA_API_KEY").unwrap_or_else(|_| "demo-key".to_string());
    let client = GalacticRegistryClient::new(api_key);

    let civ = Civilization {
        civilization_id: "earth-001".to_string(),
        name: "Human Civilization".to_string(),
        species: "Homo sapiens".to_string(),
        home_world: "Earth (Sol III)".to_string(),
        kardashev_scale: 0.73,
        technology_level: TechnologyLevel::Space,
        first_contact_date: Some(Utc::now()),
        diplomatic_status: DiplomaticStatus::Friendly,
        location: GalacticLocation {
            galaxy: "Milky Way".to_string(),
            sector: "Orion Arm".to_string(),
            coordinates: (0.0, 0.0, 0.0),
            distance_from_earth: 0.0,
        },
        population: 8_000_000_000,
        metadata: CivilizationMetadata {
            culture_type: "Diverse".to_string(),
            communication_methods: vec!["Radio".to_string(), "Optical".to_string()],
            languages: vec!["Multiple".to_string()],
            tags: vec!["홍익인간".to_string()],
        },
    };

    println!("📝 Registering civilization: {}", civ.name);
    println!("   Kardashev Scale: {:.2}", civ.kardashev_scale);
    println!("   Technology Level: {:?}", civ.technology_level);

    println!("\n🌟 Demo complete - 홍익인간 (弘益人間)");
    Ok(())
}
