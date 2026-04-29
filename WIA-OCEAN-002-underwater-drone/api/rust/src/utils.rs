//! Utility functions for underwater drone

use crate::types::*;
use uuid::Uuid;

/// Calculate estimated mission time
pub fn calculate_mission_time(waypoints: &[Waypoint], speed_knots: f64) -> f64 {
    let mut total_time = 0.0;

    for i in 0..waypoints.len() {
        // Add waypoint duration
        total_time += waypoints[i].duration_seconds as f64;

        // Calculate travel time to next waypoint
        if i < waypoints.len() - 1 {
            let distance = calculate_distance(
                waypoints[i].latitude,
                waypoints[i].longitude,
                waypoints[i + 1].latitude,
                waypoints[i + 1].longitude,
            );
            total_time += (distance / speed_knots) * 3600.0; // Convert to seconds
        }
    }

    total_time / 60.0 // Return in minutes
}

/// Calculate distance between two points (haversine formula)
pub fn calculate_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let r = 3440.065; // Earth radius in nautical miles
    let dlat = (lat2 - lat1).to_radians();
    let dlon = (lon2 - lon1).to_radians();
    let lat1 = lat1.to_radians();
    let lat2 = lat2.to_radians();

    let a = (dlat / 2.0).sin().powi(2) +
            lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

    r * c
}

/// Estimate battery consumption
pub fn estimate_battery_consumption(
    distance_nm: f64,
    speed_knots: f64,
    battery_capacity_kwh: f64,
) -> f64 {
    let time_hours = distance_nm / speed_knots;
    // Approximate consumption: 10% per hour at max speed
    (time_hours * 10.0 * 100.0) / battery_capacity_kwh
}

/// Generate drone ID
pub fn generate_drone_id() -> Uuid {
    Uuid::new_v4()
}

/// Generate command ID
pub fn generate_command_id() -> Uuid {
    Uuid::new_v4()
}

/// Format drone status
pub fn format_drone_status(drone: &UnderwaterDrone) -> String {
    format!(
        "{} ({:?}) - Max Depth: {:.0}m, Speed: {:.1} knots, Status: {:?}",
        drone.name, drone.drone_type, drone.max_depth_meters,
        drone.max_speed_knots, drone.status
    )
}
