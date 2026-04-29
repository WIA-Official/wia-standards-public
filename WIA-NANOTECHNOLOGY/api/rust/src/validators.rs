//! Validation functions for nanotechnology

use crate::error::{Result, NanotechnologyError};
use crate::types::*;

/// Validate nanoparticle specification
pub fn validate_nanoparticle(particle: &Nanoparticle) -> Result<()> {
    if particle.name.is_empty() {
        return Err(NanotechnologyError::ValidationError(
            "Nanoparticle name cannot be empty".to_string()
        ));
    }

    if particle.size_nm <= 0.0 || particle.size_nm > 1000.0 {
        return Err(NanotechnologyError::ValidationError(
            "Nanoparticle size must be between 0 and 1000 nm".to_string()
        ));
    }

    if particle.surface_area_m2 < 0.0 {
        return Err(NanotechnologyError::ValidationError(
            "Surface area cannot be negative".to_string()
        ));
    }

    Ok(())
}

/// Validate synthesis parameters
pub fn validate_synthesis_params(params: &SynthesisParams) -> Result<()> {
    if params.temperature_celsius < -273.15 {
        return Err(NanotechnologyError::ValidationError(
            "Temperature cannot be below absolute zero".to_string()
        ));
    }

    if params.pressure_kpa < 0.0 {
        return Err(NanotechnologyError::ValidationError(
            "Pressure cannot be negative".to_string()
        ));
    }

    if params.duration_minutes <= 0.0 {
        return Err(NanotechnologyError::ValidationError(
            "Duration must be positive".to_string()
        ));
    }

    if params.precursors.is_empty() {
        return Err(NanotechnologyError::ValidationError(
            "At least one precursor is required".to_string()
        ));
    }

    Ok(())
}

/// Validate optical properties
pub fn validate_optical_properties(props: &OpticalProperties) -> Result<()> {
    if props.absorption_wavelength_nm <= 0.0 {
        return Err(NanotechnologyError::ValidationError(
            "Absorption wavelength must be positive".to_string()
        ));
    }

    if props.emission_wavelength_nm <= 0.0 {
        return Err(NanotechnologyError::ValidationError(
            "Emission wavelength must be positive".to_string()
        ));
    }

    if props.quantum_yield < 0.0 || props.quantum_yield > 1.0 {
        return Err(NanotechnologyError::ValidationError(
            "Quantum yield must be between 0 and 1".to_string()
        ));
    }

    Ok(())
}
