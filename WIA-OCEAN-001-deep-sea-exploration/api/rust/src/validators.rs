//! Validation functions for deep sea exploration

use crate::error::{Result, DeepSeaExplorationError};
use crate::types::*;

/// Validate ocean location
pub fn validate_location(location: &OceanLocation) -> Result<()> {
    if location.latitude < -90.0 || location.latitude > 90.0 {
        return Err(DeepSeaExplorationError::ValidationError(
            "Latitude must be between -90 and 90".to_string()
        ));
    }

    if location.longitude < -180.0 || location.longitude > 180.0 {
        return Err(DeepSeaExplorationError::ValidationError(
            "Longitude must be between -180 and 180".to_string()
        ));
    }

    if location.ocean_name.is_empty() {
        return Err(DeepSeaExplorationError::ValidationError(
            "Ocean name cannot be empty".to_string()
        ));
    }

    Ok(())
}

/// Validate exploration mission
pub fn validate_mission(mission: &ExplorationMission) -> Result<()> {
    validate_location(&mission.location)?;

    if mission.depth_meters < 0.0 {
        return Err(DeepSeaExplorationError::ValidationError(
            "Depth cannot be negative".to_string()
        ));
    }

    if mission.depth_meters > 11000.0 {
        return Err(DeepSeaExplorationError::DepthLimitExceeded(mission.depth_meters));
    }

    if mission.depth_meters > mission.vessel.max_depth_meters {
        return Err(DeepSeaExplorationError::ValidationError(
            format!("Mission depth ({:.0}m) exceeds vessel limit ({:.0}m)",
                mission.depth_meters, mission.vessel.max_depth_meters)
        ));
    }

    if mission.duration_hours <= 0.0 {
        return Err(DeepSeaExplorationError::ValidationError(
            "Duration must be positive".to_string()
        ));
    }

    if mission.objectives.is_empty() {
        return Err(DeepSeaExplorationError::ValidationError(
            "At least one objective is required".to_string()
        ));
    }

    Ok(())
}

/// Validate vessel
pub fn validate_vessel(vessel: &Vessel) -> Result<()> {
    if vessel.vessel_id.is_empty() {
        return Err(DeepSeaExplorationError::ValidationError(
            "Vessel ID cannot be empty".to_string()
        ));
    }

    if vessel.max_depth_meters <= 0.0 {
        return Err(DeepSeaExplorationError::ValidationError(
            "Max depth must be positive".to_string()
        ));
    }

    Ok(())
}
