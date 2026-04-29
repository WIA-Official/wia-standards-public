//! Basic usage example for WIA Ocean Underwater Drone SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_ocean_002_underwater_drone::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("WIA Ocean Underwater Drone SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    // Create underwater drone
    let drone = UnderwaterDrone {
        id: utils::generate_drone_id(),
        name: "AUV-Neptune".to_string(),
        drone_type: DroneType::Auv,
        max_depth_meters: 6000.0,
        battery_capacity_kwh: 10.0,
        max_speed_knots: 5.0,
        sensors: vec![
            Sensor::Camera,
            Sensor::Sonar,
            Sensor::CtdSensor,
        ],
        status: DroneStatus::Idle,
    };

    // Validate drone
    validators::validate_drone(&drone)?;
    println!("✓ Drone registered: {}", utils::format_drone_status(&drone));

    // Create waypoints
    let waypoints = vec![
        Waypoint {
            latitude: 36.0,
            longitude: -122.0,
            depth_meters: 100.0,
            duration_seconds: 300,
            actions: vec![WaypointAction::TakePhoto, WaypointAction::ScanSonar],
        },
        Waypoint {
            latitude: 36.1,
            longitude: -122.1,
            depth_meters: 200.0,
            duration_seconds: 600,
            actions: vec![WaypointAction::CollectSample],
        },
    ];

    // Create navigation command
    let command = NavigationCommand {
        command_id: utils::generate_command_id(),
        drone_id: drone.id,
        waypoints: waypoints.clone(),
        max_speed_knots: 3.0,
        emergency_surface_depth: 10.0,
    };

    // Validate command
    validators::validate_command(&command, &drone)?;
    println!("✓ Navigation command validated");

    // Calculate mission time
    let mission_time = utils::calculate_mission_time(&waypoints, command.max_speed_knots);
    println!("  Estimated mission time: {:.1} minutes", mission_time);

    // Calculate distance
    let distance = utils::calculate_distance(
        waypoints[0].latitude,
        waypoints[0].longitude,
        waypoints[1].latitude,
        waypoints[1].longitude,
    );
    println!("  Total distance: {:.2} nautical miles", distance);

    Ok(())
}
