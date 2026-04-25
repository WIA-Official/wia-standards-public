//! Basic usage example for WIA Ocean Marine Biology Data SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_ocean_004_marine_biology_data::*;

#[tokio::main]
async fn main() -> Result<()> {
    println!("WIA Ocean Marine Biology Data SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    let species = MarineSpecies {
        id: utils::generate_species_id(),
        scientific_name: "Megaptera novaeangliae".to_string(),
        common_name: "Humpback Whale".to_string(),
        taxonomy: Taxonomy {
            kingdom: "Animalia".to_string(),
            phylum: "Chordata".to_string(),
            class: "Mammalia".to_string(),
            order: "Cetacea".to_string(),
            family: "Balaenopteridae".to_string(),
            genus: "Megaptera".to_string(),
            species: "M. novaeangliae".to_string(),
        },
        habitat: HabitatType::OpenOcean,
        conservation_status: ConservationStatus::LeastConcern,
    };

    validators::validate_species(&species)?;
    println!("✓ Species validated: {}", utils::format_species_name(&species));

    Ok(())
}
