//! Validation functions for microplastic detection

use crate::error::{Result, MicroplasticDetectionError};
use crate::types::*;

/// Validate detection configuration
pub fn validate_config(config: &DetectionConfig) -> Result<()> {
    if config.min_size_micrometers <= 0.0 {
        return Err(MicroplasticDetectionError::ValidationError(
            "Minimum size must be positive".to_string()
        ));
    }

    if config.max_depth_meters < 0.0 {
        return Err(MicroplasticDetectionError::ValidationError(
            "Maximum depth cannot be negative".to_string()
        ));
    }

    if config.detection_threshold < 0.0 || config.detection_threshold > 1.0 {
        return Err(MicroplasticDetectionError::ValidationError(
            "Detection threshold must be between 0 and 1".to_string()
        ));
    }

    if config.sampling_rate_hz <= 0.0 {
        return Err(MicroplasticDetectionError::ValidationError(
            "Sampling rate must be positive".to_string()
        ));
    }

    Ok(())
}

/// Validate geographic location
pub fn validate_location(location: &GeoLocation) -> Result<()> {
    if location.latitude < -90.0 || location.latitude > 90.0 {
        return Err(MicroplasticDetectionError::ValidationError(
            "Latitude must be between -90 and 90".to_string()
        ));
    }

    if location.longitude < -180.0 || location.longitude > 180.0 {
        return Err(MicroplasticDetectionError::ValidationError(
            "Longitude must be between -180 and 180".to_string()
        ));
    }

    Ok(())
}

/// Validate microplastic particle
pub fn validate_particle(particle: &MicroplasticParticle) -> Result<()> {
    validate_location(&particle.location)?;

    if particle.size_micrometers <= 0.0 {
        return Err(MicroplasticDetectionError::ValidationError(
            "Particle size must be positive".to_string()
        ));
    }

    if particle.concentration < 0.0 {
        return Err(MicroplasticDetectionError::ValidationError(
            "Concentration cannot be negative".to_string()
        ));
    }

    Ok(())
}
