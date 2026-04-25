//! Type definitions for marine biology data

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};
use uuid::Uuid;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MarineSpecies {
    pub id: Uuid,
    pub scientific_name: String,
    pub common_name: String,
    pub taxonomy: Taxonomy,
    pub habitat: HabitatType,
    pub conservation_status: ConservationStatus,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Taxonomy {
    pub kingdom: String,
    pub phylum: String,
    pub class: String,
    pub order: String,
    pub family: String,
    pub genus: String,
    pub species: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum HabitatType {
    CoralReef,
    OpenOcean,
    DeepSea,
    CoastalWaters,
    Estuary,
    Mangrove,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ConservationStatus {
    LeastConcern,
    NearThreatened,
    Vulnerable,
    Endangered,
    CriticallyEndangered,
    Extinct,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BiologicalObservation {
    pub id: Uuid,
    pub species_id: Uuid,
    pub location: GeoLocation,
    pub timestamp: DateTime<Utc>,
    pub population_count: u32,
    pub depth_meters: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeoLocation {
    pub latitude: f64,
    pub longitude: f64,
}
