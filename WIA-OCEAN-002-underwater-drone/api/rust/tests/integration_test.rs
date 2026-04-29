//! Integration tests for WIA Ocean Underwater Drone SDK

use wia_ocean_002_underwater_drone::*;

#[test]
fn test_drone_validation() {
    let valid_drone = UnderwaterDrone {
        id: utils::generate_drone_id(),
        name: "AUV-Neptune".to_string(),
        drone_type: DroneType::Auv,
        max_depth_meters: 6000.0,
        battery_capacity_kwh: 10.0,
        max_speed_knots: 5.0,
        sensors: vec![Sensor::Camera],
        status: DroneStatus::Idle,
    };

    assert!(validators::validate_drone(&valid_drone).is_ok());

    let invalid_drone = UnderwaterDrone {
        id: utils::generate_drone_id(),
        name: "".to_string(),
        drone_type: DroneType::Auv,
        max_depth_meters: 6000.0,
        battery_capacity_kwh: 10.0,
        max_speed_knots: 5.0,
        sensors: vec![Sensor::Camera],
        status: DroneStatus::Idle,
    };

    assert!(validators::validate_drone(&invalid_drone).is_err());
}

#[test]
fn test_waypoint_validation() {
    let valid_waypoint = Waypoint {
        latitude: 36.0,
        longitude: -122.0,
        depth_meters: 100.0,
        duration_seconds: 300,
        actions: vec![WaypointAction::TakePhoto],
    };

    assert!(validators::validate_waypoint(&valid_waypoint).is_ok());

    let invalid_waypoint = Waypoint {
        latitude: 100.0,
        longitude: -122.0,
        depth_meters: 100.0,
        duration_seconds: 300,
        actions: vec![WaypointAction::TakePhoto],
    };

    assert!(validators::validate_waypoint(&invalid_waypoint).is_err());
}

#[test]
fn test_distance_calculation() {
    let distance = utils::calculate_distance(36.0, -122.0, 36.1, -122.1);
    assert!(distance > 0.0);
}

#[test]
fn test_mission_time_calculation() {
    let waypoints = vec![
        Waypoint {
            latitude: 36.0,
            longitude: -122.0,
            depth_meters: 100.0,
            duration_seconds: 300,
            actions: vec![WaypointAction::TakePhoto],
        },
    ];

    let time = utils::calculate_mission_time(&waypoints, 5.0);
    assert!(time > 0.0);
}
