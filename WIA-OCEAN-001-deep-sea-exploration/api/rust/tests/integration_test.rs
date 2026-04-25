//! Integration tests for WIA Ocean Deep Sea Exploration SDK

use wia_ocean_001_deep_sea_exploration::*;
use chrono::Utc;

#[test]
fn test_location_validation() {
    let valid_location = OceanLocation {
        latitude: 36.2048,
        longitude: -112.0723,
        ocean_name: "Pacific Ocean".to_string(),
        zone: OceanZone::Hadopelagic,
    };

    assert!(validators::validate_location(&valid_location).is_ok());

    let invalid_location = OceanLocation {
        latitude: 100.0,
        longitude: -112.0723,
        ocean_name: "Pacific Ocean".to_string(),
        zone: OceanZone::Hadopelagic,
    };

    assert!(validators::validate_location(&invalid_location).is_err());
}

#[test]
fn test_zone_determination() {
    assert!(matches!(utils::determine_zone(100.0), OceanZone::Epipelagic));
    assert!(matches!(utils::determine_zone(500.0), OceanZone::Mesopelagic));
    assert!(matches!(utils::determine_zone(2000.0), OceanZone::Bathypelagic));
    assert!(matches!(utils::determine_zone(5000.0), OceanZone::Abyssopelagic));
    assert!(matches!(utils::determine_zone(8000.0), OceanZone::Hadopelagic));
}

#[test]
fn test_pressure_calculation() {
    let pressure = utils::calculate_pressure_bar(1000.0);
    assert_eq!(pressure, 101.0);
}

#[test]
fn test_vessel_recommendation() {
    assert!(matches!(utils::recommend_vessel_type(200.0), VesselType::Rov));
    assert!(matches!(utils::recommend_vessel_type(1000.0), VesselType::Submersible));
    assert!(matches!(utils::recommend_vessel_type(4000.0), VesselType::Auv));
    assert!(matches!(utils::recommend_vessel_type(8000.0), VesselType::DeepSeaDrone));
}

#[test]
fn test_mission_validation() {
    let location = OceanLocation {
        latitude: 36.2048,
        longitude: -112.0723,
        ocean_name: "Pacific Ocean".to_string(),
        zone: OceanZone::Hadopelagic,
    };

    let vessel = Vessel {
        vessel_id: "DSV-001".to_string(),
        vessel_type: VesselType::DeepSeaDrone,
        max_depth_meters: 11000.0,
        crew_capacity: 3,
        equipment: vec!["Camera".to_string()],
    };

    let valid_mission = ExplorationMission {
        id: utils::generate_mission_id(),
        name: "Test Mission".to_string(),
        location,
        depth_meters: 5000.0,
        start_time: Utc::now(),
        duration_hours: 8.0,
        vessel,
        objectives: vec![MissionObjective::BiologicalSurvey],
        status: MissionStatus::Planned,
    };

    assert!(validators::validate_mission(&valid_mission).is_ok());
}
