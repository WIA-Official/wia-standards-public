//! Asteroid Mining Example
//!
//! Demonstrates creating a complete asteroid mining mission specification.

use wia_space::prelude::*;
use wia_space::adapters::*;

fn main() {
    println!("=== Asteroid Mining Mission Planner ===\n");

    // Define target asteroid
    let asteroid = TargetAsteroid::new("2024 PT5", AsteroidType::SType)
        .with_diameter_km(10.0)
        .with_mass_kg(5e12)
        .with_location(AsteroidLocation::NearEarth)
        .with_orbit(OrbitalParameters::new(1.05, 0.07, 1.5));

    // Create mining specification
    let mining = AsteroidMiningSpec::new("mining-2024pt5", asteroid)
        .with_resources(ResourceAssessment {
            assessment_date: Some("2025-01-15".to_string()),
            confidence_level: Some(0.6),
            resources: vec![
                Resource::new("Fe", 0.35)
                    .with_name("Iron")
                    .with_mass(1.75e12)
                    .with_value(5e10),
                Resource::new("Ni", 0.15)
                    .with_name("Nickel")
                    .with_mass(7.5e11)
                    .with_value(1.5e11),
                Resource::new("Si", 0.25)
                    .with_name("Silicon")
                    .with_mass(1.25e12)
                    .with_value(2e10),
                Resource::new("H2O", 0.05)
                    .with_name("Water")
                    .with_mass(2.5e11)
                    .with_value(1e11),
            ],
            total_estimated_value_usd: Some(3.1e11),
        })
        .with_mission(MiningMissionParameters {
            launch_window: Some("2027-06-15".to_string()),
            delta_v_km_s: Some(4.5),
            travel_time_days: Some(180.0),
            stay_time_days: Some(90.0),
            extraction_method: Some(ExtractionMethod::OpticalMining),
        });

    // Print mission summary
    println!("Target Asteroid: {}", mining.target_asteroid.name);
    println!("Type: {:?}", mining.target_asteroid.asteroid_type);
    println!("Location: {:?}", mining.target_asteroid.location.unwrap());
    println!();

    // Print orbital info
    if let Some(orbit) = &mining.target_asteroid.orbital_parameters {
        println!("Orbital Parameters:");
        println!("  Semi-major axis: {:.3} AU", orbit.semi_major_axis_au);
        println!("  Eccentricity: {:.3}", orbit.eccentricity);
        println!("  Inclination: {:.1}°", orbit.inclination_deg);
        println!("  Orbital period: {:.2} years", orbit.orbital_period_years);
    }
    println!();

    // Calculate transfer requirements
    println!("Transfer Calculations (Earth to target):");
    let delta_v = OrbitalSimulator::hohmann_delta_v(1.0, 1.05);
    let transfer_time = OrbitalSimulator::hohmann_transfer_time(1.0, 1.05);
    println!("  Delta-v (Hohmann): {:.2} km/s", delta_v);
    println!("  Transfer time: {:.0} days", transfer_time);
    println!();

    // Print resource assessment
    if let Some(resources) = &mining.resource_assessment {
        println!("Resource Assessment (confidence: {:.0}%):", resources.confidence_level.unwrap() * 100.0);
        for r in &resources.resources {
            println!(
                "  {} ({}): {:.1}% - ${:.2e}",
                r.name.as_ref().unwrap_or(&r.element),
                r.element,
                r.mass_fraction * 100.0,
                r.estimated_value_usd.unwrap_or(0.0)
            );
        }
        println!("  Total estimated value: ${:.2e}", resources.total_estimated_value_usd.unwrap());
    }
    println!();

    // Print mission parameters
    if let Some(mission) = &mining.mission_parameters {
        println!("Mission Parameters:");
        println!("  Launch window: {}", mission.launch_window.as_ref().unwrap());
        println!("  Delta-v required: {:.1} km/s", mission.delta_v_km_s.unwrap());
        println!("  Travel time: {:.0} days", mission.travel_time_days.unwrap());
        println!("  Stay time: {:.0} days", mission.stay_time_days.unwrap());
        println!("  Extraction method: {:?}", mission.extraction_method.unwrap());
    }
    println!();

    // Serialize to JSON
    println!("=== JSON Specification ===");
    let json = serde_json::to_string_pretty(&mining).unwrap();
    println!("{}", json);
}
