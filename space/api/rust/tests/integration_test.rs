//! Integration tests for WIA Space API

use wia_space::prelude::*;
use wia_space::adapters::*;

#[test]
fn test_dyson_sphere_creation() {
    let dyson = DysonSphereSpec::sol_swarm("dyson-test-001")
        .with_structure(DysonStructureParameters {
            orbital_radius_au: 1.0,
            total_collectors: Some(1_000_000),
            collector_area_km2: Some(100.0),
            total_collection_area_km2: Some(100_000_000.0),
            coverage_fraction: Some(0.01),
            material_mass_kg: Some(1e18),
        })
        .with_energy(EnergyHarvesting {
            efficiency_percent: 25.0,
            total_power_watts: Some(9.5e23),
            collection_method: Some(EnergyCollectionMethod::Photovoltaic),
            transmission_method: Some(EnergyTransmissionMethod::MicrowaveBeam),
        });

    assert_eq!(dyson.category, TechnologyCategory::DysonSphere);
    assert_eq!(dyson.structure_type, DysonStructureType::DysonSwarm);

    // Calculate power
    let power = dyson.calculate_power();
    assert!(power.is_some());
    assert!(power.unwrap() > 9e23);
}

#[test]
fn test_mars_terraforming() {
    let mars = MarsTerraformingSpec::mars("terraform-mars-001")
        .add_method(InterventionMethod {
            method: TerraformingMethod::SolarMirrors,
            description: Some("Orbital mirrors to increase solar flux".to_string()),
            temperature_effect_kelvin: Some(10.0),
            implementation_time_years: Some(50.0),
        })
        .add_method(InterventionMethod {
            method: TerraformingMethod::GreenhouseGasRelease,
            description: Some("Super greenhouse gas factories".to_string()),
            temperature_effect_kelvin: Some(20.0),
            implementation_time_years: Some(100.0),
        });

    assert_eq!(mars.target_body.name, "Mars");
    assert!(mars.intervention_methods.is_some());
    assert_eq!(mars.intervention_methods.as_ref().unwrap().len(), 2);
}

#[test]
fn test_warp_drive_subluminal() {
    let warp = WarpDriveSpec::subluminal("warp-test-001")
        .with_bubble(BubbleParameters {
            radius_meters: 100.0,
            wall_thickness_meters: Some(1.0),
            shape: Some(BubbleShape::OblateSpheroid),
        })
        .with_performance(WarpPerformance {
            max_velocity_c: Some(0.1),
            cruise_velocity_c: Some(0.05),
            energy_requirement_joules: Some(1e22),
        });

    assert_eq!(warp.drive_type, WarpDriveType::AlcubierreSubluminal);
    assert!(warp.validate().is_ok());

    // Check that exotic matter is not required
    assert_eq!(warp.theoretical_basis.requires_exotic_matter, Some(false));
}

#[test]
fn test_warp_drive_ftl_validation_fails() {
    let warp = WarpDriveSpec::new("warp-ftl", WarpDriveType::AlcubierreSubluminal)
        .with_performance(WarpPerformance {
            max_velocity_c: Some(1.5), // FTL velocity
            cruise_velocity_c: None,
            energy_requirement_joules: None,
        });

    // Subluminal drive should not allow FTL
    assert!(warp.validate().is_err());
}

#[test]
fn test_space_elevator() {
    let elevator = SpaceElevatorSpec::earth_equatorial("elevator-test-001");

    assert_eq!(elevator.location.anchor_latitude, 0.0);
    assert_eq!(elevator.tether.material, TetherMaterial::SingleCrystalGraphene);

    let trip_time = elevator.calculate_geo_trip_time();
    assert!(trip_time.is_some());

    // Should be around 50 hours to GEO at 200 m/s
    let hours = trip_time.unwrap();
    assert!(hours > 45.0 && hours < 55.0);
}

#[test]
fn test_asteroid_mining_psyche() {
    let mining = AsteroidMiningSpec::psyche("mining-psyche-001");

    assert_eq!(mining.target_asteroid.name, "16 Psyche");
    assert_eq!(mining.target_asteroid.asteroid_type, AsteroidType::MType);

    // Check resources
    let resources = mining.resource_assessment.as_ref().unwrap();
    assert!(!resources.resources.is_empty());

    let iron = resources.resources.iter().find(|r| r.element == "Fe");
    assert!(iron.is_some());
    assert_eq!(iron.unwrap().mass_fraction, 0.85);
}

#[test]
fn test_interstellar_starshot() {
    let starshot = InterstellarTravelSpec::alpha_centauri_starshot("starshot-test-001");

    assert_eq!(starshot.mission.target_system, "Alpha Centauri");
    assert_eq!(starshot.mission.distance_ly, 4.246);

    // Validate propulsion
    assert_eq!(starshot.propulsion.propulsion_type, PropulsionType::LaserLightsail);

    // Check trajectory
    let trajectory = starshot.trajectory.as_ref().unwrap();
    assert_eq!(trajectory.cruise_velocity_c, 0.20);
    assert!((trajectory.travel_time_years - 21.2).abs() < 0.1);

    // Validation should pass
    assert!(starshot.validate().is_ok());

    // Calculate travel time
    let calc_time = starshot.calculate_travel_time();
    assert!(calc_time.is_some());
}

#[test]
fn test_orbital_calculations() {
    // Earth to Mars Hohmann transfer
    let delta_v = OrbitalSimulator::hohmann_delta_v(1.0, 1.524);
    assert!(delta_v > 5.0 && delta_v < 6.0); // ~5.6 km/s

    let transfer_time = OrbitalSimulator::hohmann_transfer_time(1.0, 1.524);
    assert!(transfer_time > 250.0 && transfer_time < 270.0); // ~259 days
}

#[test]
fn test_interstellar_calculations() {
    // Alpha Centauri at 20% c
    let travel_time = InterstellarCalculator::travel_time_years(4.246, 0.2);
    assert!((travel_time - 21.23).abs() < 0.1);

    // Lorentz factor at 20% c
    let gamma = InterstellarCalculator::lorentz_factor(0.2);
    assert!(gamma > 1.02 && gamma < 1.03);

    // Ship time (time dilation)
    let ship_time = InterstellarCalculator::ship_time_years(4.246, 0.2);
    assert!(ship_time < travel_time); // Ship experiences less time
}

#[test]
fn test_dyson_calculations() {
    // 1% coverage, 25% efficiency
    let power = DysonCalculator::harvestable_power(1.0, 0.01, 0.25);
    assert!(power > 9e23);

    // Collectors needed
    let num = DysonCalculator::collectors_needed(1.0, 0.01, 100.0);
    assert!(num > 1_000_000_000);
}

#[test]
fn test_json_serialization() {
    let spec = AsteroidMiningSpec::psyche("json-test");
    let json = to_json_str(&spec).unwrap();

    assert!(json.contains("asteroid_mining"));
    assert!(json.contains("16 Psyche"));
    assert!(json.contains("m_type"));

    // Deserialize back
    let restored: AsteroidMiningSpec = from_json_str(&json).unwrap();
    assert_eq!(spec.technology_id, restored.technology_id);
}

#[test]
fn test_spec_type_detection() {
    let json = r#"{
        "technology_id": "test",
        "category": "interstellar_travel",
        "mission": {
            "name": "Test Mission",
            "type": "flyby",
            "target_system": "Alpha Centauri",
            "distance_ly": 4.246
        },
        "propulsion": {
            "type": "laser_lightsail"
        }
    }"#;

    let category = SpecType::detect(json).unwrap();
    assert_eq!(category, TechnologyCategory::InterstellarTravel);
}

#[test]
fn test_project_creation() {
    let project = SpaceProject::new(
        "proj-alpha-001",
        "Alpha Centauri Mission Study",
        TechnologyCategory::InterstellarTravel,
    )
    .with_description("First interstellar probe mission feasibility study")
    .with_status(ProjectStatus::Active)
    .with_organization(Organization::new("WIA Space").with_country("International"));

    assert_eq!(project.project_id, "proj-alpha-001");
    assert_eq!(project.project_info.status, ProjectStatus::Active);
    assert!(project.organization.is_some());
}

#[tokio::test]
async fn test_orbital_simulator() {
    let simulator = OrbitalSimulator::new("test-sim")
        .with_time_step(60.0)
        .with_duration(3600.0);

    let result = simulator.run().await.unwrap();
    assert!(result.success);
    assert_eq!(result.iterations, 60);
}
