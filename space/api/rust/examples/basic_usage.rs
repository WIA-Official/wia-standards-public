//! Basic usage example for WIA Space API
//!
//! This example demonstrates creating specifications for each technology category.

use wia_space::prelude::*;

fn main() {
    println!("=== WIA Space API Examples ===\n");

    // 1. Dyson Sphere
    println!("1. Dyson Sphere Specification:");
    let dyson = DysonSphereSpec::sol_swarm("dyson-001")
        .with_structure(DysonStructureParameters {
            orbital_radius_au: 1.0,
            total_collectors: Some(1_000_000_000),
            collector_area_km2: Some(100.0),
            total_collection_area_km2: Some(1e11),
            coverage_fraction: Some(0.04),
            material_mass_kg: Some(1e21),
        })
        .with_energy(EnergyHarvesting {
            efficiency_percent: 25.0,
            total_power_watts: None,
            collection_method: Some(EnergyCollectionMethod::Photovoltaic),
            transmission_method: Some(EnergyTransmissionMethod::MicrowaveBeam),
        });

    if let Some(power) = dyson.calculate_power() {
        println!("   Harvestable power: {:.2e} W", power);
    }

    // 2. Mars Terraforming
    println!("\n2. Mars Terraforming Specification:");
    let mars = MarsTerraformingSpec::mars("terraform-001")
        .add_method(InterventionMethod {
            method: TerraformingMethod::SolarMirrors,
            description: Some("Orbital mirrors to increase solar flux".to_string()),
            temperature_effect_kelvin: Some(10.0),
            implementation_time_years: Some(50.0),
        })
        .with_target(PlanetaryConditions {
            surface_pressure_mbar: Some(500.0),
            mean_temperature_celsius: Some(10.0),
            atmosphere_composition: Some(AtmosphereComposition::earth_like()),
            surface_gravity_g: Some(0.38),
        });

    println!("   Target: {}", mars.target_body.name);
    println!("   Current temp: {}°C", mars.current_conditions.mean_temperature_celsius.unwrap());

    // 3. Warp Drive
    println!("\n3. Warp Drive Specification (Subluminal):");
    let warp = WarpDriveSpec::subluminal("warp-001")
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

    println!("   Drive type: {:?}", warp.drive_type);
    println!("   Requires exotic matter: {:?}", warp.theoretical_basis.requires_exotic_matter);
    println!("   Validation: {:?}", warp.validate());

    // 4. Space Elevator
    println!("\n4. Space Elevator Specification:");
    let elevator = SpaceElevatorSpec::earth_equatorial("elevator-001");

    println!("   Material: {:?}", elevator.tether.material);
    println!("   Length: {} km", elevator.tether.total_length_km);
    println!("   Tensile strength: {} GPa", elevator.tether.tensile_strength_gpa.unwrap());

    if let Some(trip_time) = elevator.calculate_geo_trip_time() {
        println!("   Trip time to GEO: {:.1} hours", trip_time);
    }

    // 5. Asteroid Mining
    println!("\n5. Asteroid Mining Specification:");
    let mining = AsteroidMiningSpec::psyche("mining-001");

    println!("   Target: {}", mining.target_asteroid.name);
    println!("   Type: {:?}", mining.target_asteroid.asteroid_type);
    println!("   Diameter: {} km", mining.target_asteroid.diameter_km.unwrap());

    if let Some(resources) = &mining.resource_assessment {
        println!("   Resources:");
        for r in &resources.resources {
            println!("     - {}: {:.2}%", r.element, r.mass_fraction * 100.0);
        }
    }

    // 6. Interstellar Travel
    println!("\n6. Interstellar Travel Specification:");
    let starshot = InterstellarTravelSpec::alpha_centauri_starshot("starshot-001");

    println!("   Mission: {}", starshot.mission.name);
    println!("   Target: {} ({} ly)", starshot.mission.target_system, starshot.mission.distance_ly);
    println!("   Propulsion: {:?}", starshot.propulsion.propulsion_type);

    if let Some(traj) = &starshot.trajectory {
        println!("   Cruise velocity: {}% c", traj.cruise_velocity_c * 100.0);
        println!("   Travel time: {:.1} years", traj.travel_time_years);
    }

    // Serialize to JSON
    println!("\n=== JSON Output ===");
    let json = serde_json::to_string_pretty(&mining).unwrap();
    println!("{}", json);
}
