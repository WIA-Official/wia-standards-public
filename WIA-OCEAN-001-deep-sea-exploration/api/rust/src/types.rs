//! Type definitions for deep sea exploration

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};
use uuid::Uuid;

/// Deep sea exploration mission
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExplorationMission {
    pub id: Uuid,
    pub name: String,
    pub location: OceanLocation,
    pub depth_meters: f64,
    pub start_time: DateTime<Utc>,
    pub duration_hours: f64,
    pub vessel: Vessel,
    pub objectives: Vec<MissionObjective>,
    pub status: MissionStatus,
}

/// Ocean location
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OceanLocation {
    pub latitude: f64,
    pub longitude: f64,
    pub ocean_name: String,
    pub zone: OceanZone,
}

/// Ocean depth zones
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum OceanZone {
    Epipelagic,      // 0-200m
    Mesopelagic,     // 200-1000m
    Bathypelagic,    // 1000-4000m
    Abyssopelagic,   // 4000-6000m
    Hadopelagic,     // 6000m+
}

/// Exploration vessel
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vessel {
    pub vessel_id: String,
    pub vessel_type: VesselType,
    pub max_depth_meters: f64,
    pub crew_capacity: u32,
    pub equipment: Vec<String>,
}

/// Types of exploration vessels
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum VesselType {
    Submersible,
    Rov,              // Remotely Operated Vehicle
    Auv,              // Autonomous Underwater Vehicle
    ResearchShip,
    DeepSeaDrone,
}

/// Mission objectives
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum MissionObjective {
    BiologicalSurvey,
    GeologicalMapping,
    ResourceDiscovery,
    WreckExploration,
    EnvironmentalMonitoring,
    ScientificResearch,
}

/// Mission status
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum MissionStatus {
    Planned,
    InProgress,
    Completed,
    Aborted,
    Emergency,
}

/// Exploration data collected
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExplorationData {
    pub mission_id: Uuid,
    pub timestamp: DateTime<Utc>,
    pub depth_meters: f64,
    pub temperature_celsius: f64,
    pub pressure_bar: f64,
    pub salinity_ppt: f64,
    pub visibility_meters: f64,
    pub samples_collected: u32,
}
