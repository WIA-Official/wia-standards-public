//! Integration tests for WIA-HYDROPONICS SDK
//!
//! 弘益人間 - Ensuring quality for sustainable agriculture

use wia_hydroponics::*;
use uuid::Uuid;
use chrono::Utc;

#[test]
fn test_system_type_serialization() {
    let system_type = SystemType::NFT;
    let json = serde_json::to_string(&system_type).unwrap();
    assert_eq!(json, "\"nft\"");
}

#[test]
fn test_growth_stage() {
    let stage = GrowthStage::Vegetative;
    assert_eq!(stage, GrowthStage::Vegetative);
    assert_ne!(stage, GrowthStage::Flowering);
}

#[test]
fn test_hydroponic_system_creation() {
    let system = HydroponicSystem {
        id: Uuid::new_v4(),
        name: "Test System".to_string(),
        system_type: SystemType::DWC,
        status: SystemStatus::Active,
        capacity_liters: 100.0,
        plant_count: 10,
        location: Some("Greenhouse A".to_string()),
        created_at: Utc::now(),
        updated_at: Utc::now(),
    };

    assert_eq!(system.name, "Test System");
    assert_eq!(system.system_type, SystemType::DWC);
    assert_eq!(system.capacity_liters, 100.0);
}

#[test]
fn test_plant_creation() {
    let plant = Plant {
        id: Uuid::new_v4(),
        system_id: Uuid::new_v4(),
        species: "Lettuce".to_string(),
        variety: Some("Butterhead".to_string()),
        planted_at: Utc::now(),
        growth_stage: GrowthStage::Seedling,
        expected_harvest: None,
        health_score: 95.0,
    };

    assert_eq!(plant.species, "Lettuce");
    assert_eq!(plant.growth_stage, GrowthStage::Seedling);
    assert_eq!(plant.health_score, 95.0);
}

#[test]
fn test_environment_data() {
    let env_data = EnvironmentData {
        system_id: Uuid::new_v4(),
        timestamp: Utc::now(),
        temperature_celsius: 22.5,
        humidity_percent: 65.0,
        light_intensity_lux: 15000.0,
        co2_ppm: Some(800.0),
    };

    assert_eq!(env_data.temperature_celsius, 22.5);
    assert_eq!(env_data.humidity_percent, 65.0);
}

#[test]
fn test_nutrient_data() {
    let nutrient_data = NutrientData {
        system_id: Uuid::new_v4(),
        timestamp: Utc::now(),
        ph_level: 6.0,
        ec_level: 1.5,
        temperature_celsius: 20.0,
        dissolved_oxygen: Some(8.0),
    };

    assert_eq!(nutrient_data.ph_level, 6.0);
    assert_eq!(nutrient_data.ec_level, 1.5);
}

#[test]
fn test_validators() {
    assert!(validators::validate_ph(6.0).is_ok());
    assert!(validators::validate_ph(15.0).is_err());

    assert!(validators::validate_ec(2.0).is_ok());
    assert!(validators::validate_ec(-1.0).is_err());

    assert!(validators::validate_temperature(25.0).is_ok());
    assert!(validators::validate_temperature(60.0).is_err());

    assert!(validators::validate_humidity(50.0).is_ok());
    assert!(validators::validate_humidity(150.0).is_err());
}

#[test]
fn test_optimal_ranges() {
    let (min_ph, max_ph) = utils::optimal_ph_range("lettuce");
    assert_eq!((min_ph, max_ph), (5.5, 6.5));

    let (min_ec, max_ec) = utils::optimal_ec_range("tomato");
    assert_eq!((min_ec, max_ec), (2.0, 3.5));
}

#[test]
fn test_is_optimal() {
    assert!(utils::is_ph_optimal("lettuce", 6.0));
    assert!(!utils::is_ph_optimal("lettuce", 8.0));

    assert!(utils::is_ec_optimal("lettuce", 1.0));
    assert!(!utils::is_ec_optimal("lettuce", 5.0));
}

#[test]
fn test_filter_plants() {
    let plants = vec![
        Plant {
            id: Uuid::new_v4(),
            system_id: Uuid::new_v4(),
            species: "Lettuce".to_string(),
            variety: None,
            planted_at: Utc::now(),
            growth_stage: GrowthStage::Vegetative,
            expected_harvest: None,
            health_score: 90.0,
        },
        Plant {
            id: Uuid::new_v4(),
            system_id: Uuid::new_v4(),
            species: "Tomato".to_string(),
            variety: None,
            planted_at: Utc::now(),
            growth_stage: GrowthStage::Flowering,
            expected_harvest: None,
            health_score: 85.0,
        },
    ];

    let vegetative = utils::filter_plants_by_stage(&plants, &GrowthStage::Vegetative);
    assert_eq!(vegetative.len(), 1);
    assert_eq!(vegetative[0].species, "Lettuce");
}

#[test]
fn test_alert_filtering() {
    let alerts = vec![
        SystemAlert {
            id: Uuid::new_v4(),
            system_id: Uuid::new_v4(),
            alert_type: AlertType::PhImbalance,
            severity: AlertSeverity::Critical,
            message: "pH too high".to_string(),
            created_at: Utc::now(),
            resolved_at: None,
        },
        SystemAlert {
            id: Uuid::new_v4(),
            system_id: Uuid::new_v4(),
            alert_type: AlertType::LowWaterLevel,
            severity: AlertSeverity::Warning,
            message: "Water level low".to_string(),
            created_at: Utc::now(),
            resolved_at: None,
        },
    ];

    let critical = utils::filter_critical_alerts(&alerts);
    assert_eq!(critical.len(), 1);
    assert_eq!(critical[0].severity, AlertSeverity::Critical);
}

#[test]
fn test_system_health_calculation() {
    let plants = vec![
        Plant {
            id: Uuid::new_v4(),
            system_id: Uuid::new_v4(),
            species: "Test".to_string(),
            variety: None,
            planted_at: Utc::now(),
            growth_stage: GrowthStage::Vegetative,
            expected_harvest: None,
            health_score: 90.0,
        },
    ];

    let alerts = vec![];
    let health = utils::calculate_system_health(&plants, &alerts);
    assert_eq!(health, 90.0);
}
