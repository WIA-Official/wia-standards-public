//! Validation functions for underwater drone

use crate::error::{Result, UnderwaterDroneError};
use crate::types::*;

/// Validate underwater drone specification
pub fn validate_drone(drone: &UnderwaterDrone) -> Result<()> {
    if drone.name.is_empty() {
        return Err(UnderwaterDroneError::ValidationError(
            "Drone name cannot be empty".to_string()
        ));
    }

    if drone.max_depth_meters <= 0.0 {
        return Err(UnderwaterDroneError::ValidationError(
            "Max depth must be positive".to_string()
        ));
    }

    if drone.battery_capacity_kwh <= 0.0 {
        return Err(UnderwaterDroneError::ValidationError(
            "Battery capacity must be positive".to_string()
        ));
    }

    if drone.max_speed_knots <= 0.0 {
        return Err(UnderwaterDroneError::ValidationError(
            "Max speed must be positive".to_string()
        ));
    }

    Ok(())
}

/// Validate waypoint
pub fn validate_waypoint(waypoint: &Waypoint) -> Result<()> {
    if waypoint.latitude < -90.0 || waypoint.latitude > 90.0 {
        return Err(UnderwaterDroneError::ValidationError(
            "Latitude must be between -90 and 90".to_string()
        ));
    }

    if waypoint.longitude < -180.0 || waypoint.longitude > 180.0 {
        return Err(UnderwaterDroneError::ValidationError(
            "Longitude must be between -180 and 180".to_string()
        ));
    }

    if waypoint.depth_meters < 0.0 {
        return Err(UnderwaterDroneError::ValidationError(
            "Depth cannot be negative".to_string()
        ));
    }

    Ok(())
}

/// Validate navigation command
pub fn validate_command(command: &NavigationCommand, drone: &UnderwaterDrone) -> Result<()> {
    if command.waypoints.is_empty() {
        return Err(UnderwaterDroneError::ValidationError(
            "At least one waypoint is required".to_string()
        ));
    }

    for waypoint in &command.waypoints {
        validate_waypoint(waypoint)?;

        if waypoint.depth_meters > drone.max_depth_meters {
            return Err(UnderwaterDroneError::DepthLimitExceeded(waypoint.depth_meters));
        }
    }

    if command.max_speed_knots > drone.max_speed_knots {
        return Err(UnderwaterDroneError::ValidationError(
            "Command speed exceeds drone capability".to_string()
        ));
    }

    Ok(())
}

/// Validate telemetry
pub fn validate_telemetry(telemetry: &Telemetry) -> Result<()> {
    if telemetry.battery_percent < 20.0 {
        return Err(UnderwaterDroneError::BatteryLow(telemetry.battery_percent));
    }

    if telemetry.heading_degrees < 0.0 || telemetry.heading_degrees >= 360.0 {
        return Err(UnderwaterDroneError::ValidationError(
            "Heading must be between 0 and 360 degrees".to_string()
        ));
    }

    Ok(())
}
