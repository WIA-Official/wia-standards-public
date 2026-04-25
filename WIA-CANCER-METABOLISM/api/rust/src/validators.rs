//! Validation functions for cancer metabolism data

use crate::error::{Error, Result};
use crate::types::*;

pub fn validate_metabolic_profile(profile: &MetabolicProfile) -> Result<()> {
    if profile.metabolic_markers.is_empty() {
        return Err(Error::InvalidInput("Must have at least one marker".into()));
    }
    Ok(())
}

pub fn validate_metabolic_marker(marker: &MetabolicMarker) -> Result<()> {
    if marker.name.is_empty() {
        return Err(Error::InvalidInput("Marker name cannot be empty".into()));
    }
    if marker.normal_range_max < marker.normal_range_min {
        return Err(Error::InvalidInput("Invalid normal range".into()));
    }
    Ok(())
}
