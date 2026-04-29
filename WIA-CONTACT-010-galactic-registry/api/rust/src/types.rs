//! Core types for galactic registry

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Civilization {
    pub civilization_id: String,
    pub name: String,
    pub species: String,
    pub home_world: String,
    pub kardashev_scale: f64,
    pub technology_level: TechnologyLevel,
    pub first_contact_date: Option<DateTime<Utc>>,
    pub diplomatic_status: DiplomaticStatus,
    pub location: GalacticLocation,
    pub population: u64,
    pub metadata: CivilizationMetadata,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum TechnologyLevel {
    PreIndustrial,
    Industrial,
    Atomic,
    Information,
    Space,
    Interstellar,
    Galactic,
    Intergalactic,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum DiplomaticStatus {
    Unknown,
    Contacted,
    Friendly,
    Neutral,
    Cautious,
    Hostile,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GalacticLocation {
    pub galaxy: String,
    pub sector: String,
    pub coordinates: (f64, f64, f64),
    pub distance_from_earth: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CivilizationMetadata {
    pub culture_type: String,
    pub communication_methods: Vec<String>,
    pub languages: Vec<String>,
    pub tags: Vec<String>,
}
