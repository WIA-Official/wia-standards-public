//! Basic usage example for WIA Ocean Deep Sea Exploration SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_ocean_001_deep_sea_exploration::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("WIA Ocean Deep Sea Exploration SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    // Create ocean location
    let location = OceanLocation {
        latitude: 36.2048,
        longitude: -112.0723,
        ocean_name: "Pacific Ocean".to_string(),
        zone: OceanZone::Hadopelagic,
    };

    // Create vessel
    let vessel = Vessel {
        vessel_id: "DSV-001".to_string(),
        vessel_type: VesselType::DeepSeaDrone,
        max_depth_meters: 11000.0,
        crew_capacity: 3,
        equipment: vec![
            "4K Camera".to_string(),
            "Sonar".to_string(),
            "Sample Collector".to_string(),
        ],
    };

    // Create mission
    let mission = ExplorationMission {
        id: utils::generate_mission_id(),
        name: "Mariana Trench Survey".to_string(),
        location,
        depth_meters: 10994.0,
        start_time: Utc::now(),
        duration_hours: 8.0,
        vessel,
        objectives: vec![
            MissionObjective::BiologicalSurvey,
            MissionObjective::GeologicalMapping,
        ],
        status: MissionStatus::Planned,
    };

    // Validate mission
    validators::validate_mission(&mission)?;
    println!("✓ Mission validated");
    println!("  {}", utils::format_mission_summary(&mission));

    // Calculate pressure at depth
    let pressure = utils::calculate_pressure_bar(mission.depth_meters);
    println!("  Pressure at depth: {:.0} bar", pressure);

    // Determine ocean zone
    let zone = utils::determine_zone(mission.depth_meters);
    println!("  Ocean zone: {:?}", zone);

    // Estimate duration
    let estimated_duration = utils::estimate_mission_duration(
        mission.depth_meters,
        &mission.objectives
    );
    println!("  Estimated duration: {:.1} hours", estimated_duration);

    Ok(())
}
