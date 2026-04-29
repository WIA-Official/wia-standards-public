//! Validation functions for galactic registry

use crate::{error::{Error, Result}, types::*};

pub fn validate_civilization(civ: &Civilization) -> Result<()> {
    if civ.civilization_id.is_empty() {
        return Err(Error::ValidationError("Civilization ID cannot be empty".to_string()));
    }
    if civ.name.is_empty() {
        return Err(Error::ValidationError("Name cannot be empty".to_string()));
    }
    if civ.kardashev_scale < 0.0 || civ.kardashev_scale > 5.0 {
        return Err(Error::ValidationError("Kardashev scale must be 0-5".to_string()));
    }
    Ok(())
}

pub fn validate_location(loc: &GalacticLocation) -> Result<()> {
    if loc.galaxy.is_empty() {
        return Err(Error::ValidationError("Galaxy name cannot be empty".to_string()));
    }
    if loc.distance_from_earth < 0.0 {
        return Err(Error::ValidationError("Distance must be positive".to_string()));
    }
    Ok(())
}
