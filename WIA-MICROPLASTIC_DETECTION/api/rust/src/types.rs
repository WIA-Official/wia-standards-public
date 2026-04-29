//! Type definitions for microplastic detection

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};
use uuid::Uuid;

/// Microplastic particle detection data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MicroplasticParticle {
    pub id: Uuid,
    pub detected_at: DateTime<Utc>,
    pub location: GeoLocation,
    pub size_micrometers: f64,
    pub particle_type: PlasticType,
    pub concentration: f64,
    pub depth_meters: f64,
    pub metadata: Option<serde_json::Value>,
}

/// Geographic location
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeoLocation {
    pub latitude: f64,
    pub longitude: f64,
    pub accuracy_meters: Option<f64>,
}

/// Types of plastic materials
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum PlasticType {
    Polyethylene,
    Polypropylene,
    Polystyrene,
    Pvc,
    Pet,
    Unknown,
}

/// Detection configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectionConfig {
    pub min_size_micrometers: f64,
    pub max_depth_meters: f64,
    pub detection_threshold: f64,
    pub sampling_rate_hz: f64,
}

/// Analysis result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalysisResult {
    pub total_particles: u64,
    pub average_concentration: f64,
    pub dominant_type: PlasticType,
    pub analyzed_at: DateTime<Utc>,
}
