//! Validation functions for marine biology data

use crate::error::{Result, MarineBiologyError};
use crate::types::*;

pub fn validate_species(species: &MarineSpecies) -> Result<()> {
    if species.scientific_name.is_empty() {
        return Err(MarineBiologyError::ValidationError(
            "Scientific name cannot be empty".to_string()
        ));
    }
    Ok(())
}

pub fn validate_observation(obs: &BiologicalObservation) -> Result<()> {
    if obs.location.latitude < -90.0 || obs.location.latitude > 90.0 {
        return Err(MarineBiologyError::ValidationError(
            "Invalid latitude".to_string()
        ));
    }
    Ok(())
}
