//! Utility functions for marine biology data

use crate::types::*;
use uuid::Uuid;

pub fn generate_species_id() -> Uuid {
    Uuid::new_v4()
}

pub fn generate_observation_id() -> Uuid {
    Uuid::new_v4()
}

pub fn format_species_name(species: &MarineSpecies) -> String {
    format!("{} ({})", species.scientific_name, species.common_name)
}
