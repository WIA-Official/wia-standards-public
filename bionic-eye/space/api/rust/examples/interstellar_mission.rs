//! Interstellar Mission Example
//!
//! Demonstrates planning an interstellar probe mission to Alpha Centauri.

use wia_space::prelude::*;
use wia_space::adapters::*;

fn main() {
    println!("=== Interstellar Mission Planner ===\n");

    // Create mission specification
    let mission = InterstellarMission {
        name: "Centauri Pathfinder".to_string(),
        mission_type: MissionType::Flyby,
        target_system: "Alpha Centauri".to_string(),
        target_body: Some("Proxima Centauri b".to_string()),
        distance_ly: 4.246,
    };

    // Define propulsion system
    let propulsion = InterstellarPropulsion {
        propulsion_type: PropulsionType::LaserLightsail,
        sail_area_m2: Some(16.0),
        sail_mass_kg: Some(0.001),
        laser_array_power_gw: Some(100.0),
        acceleration_g: Some(30000.0),
    };

    // Create full specification
    let starshot = InterstellarTravelSpec::new("centauri-pathfinder-001", mission, propulsion)
        .with_spacecraft(InterstellarSpacecraft {
            spacecraft_type: SpacecraftType::Starchip,
            total_mass_kg: Some(0.005), // 5 grams
            payload_mass_kg: Some(0.004),
            instruments: Some(vec![
                "camera_4k".to_string(),
                "spectrometer".to_string(),
                "magnetometer".to_string(),
                "particle_detector".to_string(),
                "laser_comm".to_string(),
            ]),
        })
        .with_trajectory(InterstellarTrajectory {
            cruise_velocity_c: 0.20,
            travel_time_years: 21.23,
            launch_date: Some("2050-07-04".to_string()),
            arrival_date: Some("2071-09-15".to_string()),
        });

    // Validate mission
    match starshot.validate() {
        Ok(_) => println!("Mission validation: PASSED"),
        Err(e) => println!("Mission validation: FAILED - {}", e),
    }
    println!();

    // Print mission summary
    println!("Mission: {}", starshot.mission.name);
    println!("Type: {:?}", starshot.mission.mission_type);
    println!("Target: {}", starshot.mission.target_system);
    if let Some(body) = &starshot.mission.target_body {
        println!("Target body: {}", body);
    }
    println!("Distance: {:.3} light-years", starshot.mission.distance_ly);
    println!();

    // Propulsion details
    println!("Propulsion System:");
    println!("  Type: {:?}", starshot.propulsion.propulsion_type);
    println!("  Sail area: {} m²", starshot.propulsion.sail_area_m2.unwrap());
    println!("  Sail mass: {} g", starshot.propulsion.sail_mass_kg.unwrap() * 1000.0);
    println!("  Laser power: {} GW", starshot.propulsion.laser_array_power_gw.unwrap());
    println!("  Peak acceleration: {} g", starshot.propulsion.acceleration_g.unwrap());
    println!();

    // Spacecraft details
    if let Some(sc) = &starshot.spacecraft {
        println!("Spacecraft:");
        println!("  Type: {:?}", sc.spacecraft_type);
        println!("  Total mass: {} g", sc.total_mass_kg.unwrap() * 1000.0);
        println!("  Payload mass: {} g", sc.payload_mass_kg.unwrap() * 1000.0);
        if let Some(instruments) = &sc.instruments {
            println!("  Instruments: {}", instruments.join(", "));
        }
    }
    println!();

    // Trajectory details
    if let Some(traj) = &starshot.trajectory {
        println!("Trajectory:");
        println!("  Cruise velocity: {}% c ({:.0} km/s)",
            traj.cruise_velocity_c * 100.0,
            traj.cruise_velocity_c * 299792.458
        );
        println!("  Travel time: {:.2} years", traj.travel_time_years);
        if let Some(launch) = &traj.launch_date {
            println!("  Launch date: {}", launch);
        }
        if let Some(arrival) = &traj.arrival_date {
            println!("  Arrival date: {}", arrival);
        }
    }
    println!();

    // Physics calculations
    println!("Physics Calculations:");

    // Lorentz factor
    let velocity_c = starshot.trajectory.as_ref().unwrap().cruise_velocity_c;
    let gamma = InterstellarCalculator::lorentz_factor(velocity_c);
    println!("  Lorentz factor (γ): {:.4}", gamma);

    // Ship time
    let ship_time = InterstellarCalculator::ship_time_years(
        starshot.mission.distance_ly,
        velocity_c
    );
    println!("  Ship time (time dilation): {:.2} years", ship_time);

    // Kinetic energy
    let mass = starshot.spacecraft.as_ref().unwrap().total_mass_kg.unwrap();
    let energy = InterstellarCalculator::kinetic_energy_joules(mass, velocity_c);
    println!("  Kinetic energy: {:.2e} J", energy);
    println!("  Kinetic energy: {:.2e} kg TNT equivalent", energy / 4.184e9);

    // Acceleration time
    let sail_mass = starshot.propulsion.sail_mass_kg.unwrap();
    let sail_area = starshot.propulsion.sail_area_m2.unwrap();
    let laser_power = starshot.propulsion.laser_array_power_gw.unwrap() * 1e9;
    let accel_time = InterstellarCalculator::lightsail_acceleration_time(
        sail_mass + mass,
        sail_area,
        laser_power,
        velocity_c
    );
    println!("  Acceleration phase: {:.1} minutes", accel_time / 60.0);
    println!();

    // Communication
    println!("Communication:");
    let signal_time = starshot.mission.distance_ly; // Light travel time in years
    println!("  Signal travel time: {:.3} years ({:.1} years round-trip)",
        signal_time, signal_time * 2.0
    );
    println!("  Data return: ~1 bit/second (estimated)");
    println!();

    // Serialize to JSON
    println!("=== JSON Specification ===");
    let json = serde_json::to_string_pretty(&starshot).unwrap();
    println!("{}", json);
}
