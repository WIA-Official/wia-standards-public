//! Integration tests for WIA Ocean Marine Biology Data SDK

use wia_ocean_004_marine_biology_data::*;

#[test]
fn test_species_validation() {
    let species = MarineSpecies {
        id: utils::generate_species_id(),
        scientific_name: "Test species".to_string(),
        common_name: "Test".to_string(),
        taxonomy: Taxonomy {
            kingdom: "Animalia".to_string(),
            phylum: "Chordata".to_string(),
            class: "Mammalia".to_string(),
            order: "Cetacea".to_string(),
            family: "Test".to_string(),
            genus: "Test".to_string(),
            species: "T. test".to_string(),
        },
        habitat: HabitatType::OpenOcean,
        conservation_status: ConservationStatus::LeastConcern,
    };

    assert!(validators::validate_species(&species).is_ok());
}
