//! Utility functions for deep sea exploration

use crate::types::*;
use uuid::Uuid;

/// Determine ocean zone based on depth
pub fn determine_zone(depth_meters: f64) -> OceanZone {
    if depth_meters < 200.0 {
        OceanZone::Epipelagic
    } else if depth_meters < 1000.0 {
        OceanZone::Mesopelagic
    } else if depth_meters < 4000.0 {
        OceanZone::Bathypelagic
    } else if depth_meters < 6000.0 {
        OceanZone::Abyssopelagic
    } else {
        OceanZone::Hadopelagic
    }
}

/// Calculate water pressure at depth
pub fn calculate_pressure_bar(depth_meters: f64) -> f64 {
    // Approximate: 1 bar per 10m + 1 bar atmospheric
    1.0 + (depth_meters / 10.0)
}

/// Estimate required vessel type for depth
pub fn recommend_vessel_type(depth_meters: f64) -> VesselType {
    if depth_meters < 300.0 {
        VesselType::Rov
    } else if depth_meters < 2000.0 {
        VesselType::Submersible
    } else if depth_meters < 6000.0 {
        VesselType::Auv
    } else {
        VesselType::DeepSeaDrone
    }
}

/// Generate mission ID
pub fn generate_mission_id() -> Uuid {
    Uuid::new_v4()
}

/// Calculate estimated mission duration
pub fn estimate_mission_duration(depth_meters: f64, objectives: &[MissionObjective]) -> f64 {
    // Base time: descent and ascent (approx 30m/min)
    let travel_time = (depth_meters / 30.0) * 2.0 / 60.0; // hours

    // Time per objective (hours)
    let objective_time = objectives.len() as f64 * 2.0;

    travel_time + objective_time
}

/// Format mission summary
pub fn format_mission_summary(mission: &ExplorationMission) -> String {
    format!(
        "Mission: {} | Depth: {:.0}m | Zone: {:?} | Vessel: {:?}",
        mission.name,
        mission.depth_meters,
        determine_zone(mission.depth_meters),
        mission.vessel.vessel_type
    )
}
