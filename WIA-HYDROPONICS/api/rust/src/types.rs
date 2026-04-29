//! Type definitions for WIA-HYDROPONICS standard
//!
//! 弘益人間 - Sustainable agriculture for all humanity

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};
use uuid::Uuid;

/// Hydroponic system type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum SystemType {
    NFT,           // Nutrient Film Technique
    DWC,           // Deep Water Culture
    Ebb,           // Ebb and Flow
    Drip,          // Drip System
    Aeroponic,     // Aeroponics
    Wicking,       // Wick System
}

/// System status
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum SystemStatus {
    Active,
    Inactive,
    Maintenance,
    Error,
}

/// Hydroponic system information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HydroponicSystem {
    pub id: Uuid,
    pub name: String,
    pub system_type: SystemType,
    pub status: SystemStatus,
    pub capacity_liters: f64,
    pub plant_count: u32,
    pub location: Option<String>,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
}

/// Plant information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Plant {
    pub id: Uuid,
    pub system_id: Uuid,
    pub species: String,
    pub variety: Option<String>,
    pub planted_at: DateTime<Utc>,
    pub growth_stage: GrowthStage,
    pub expected_harvest: Option<DateTime<Utc>>,
    pub health_score: f64,
}

/// Plant growth stage
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum GrowthStage {
    Seedling,
    Vegetative,
    Flowering,
    Fruiting,
    Harvest,
}

/// Environmental sensor data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvironmentData {
    pub system_id: Uuid,
    pub timestamp: DateTime<Utc>,
    pub temperature_celsius: f64,
    pub humidity_percent: f64,
    pub light_intensity_lux: f64,
    pub co2_ppm: Option<f64>,
}

/// Nutrient solution data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NutrientData {
    pub system_id: Uuid,
    pub timestamp: DateTime<Utc>,
    pub ph_level: f64,
    pub ec_level: f64,        // Electrical Conductivity (mS/cm)
    pub temperature_celsius: f64,
    pub dissolved_oxygen: Option<f64>,
}

/// Nutrient formulation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NutrientFormula {
    pub id: Uuid,
    pub name: String,
    pub target_ph: f64,
    pub target_ec: f64,
    pub nutrients: Vec<NutrientComponent>,
    pub suitable_for: Vec<String>,
}

/// Nutrient component
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NutrientComponent {
    pub name: String,
    pub concentration_ppm: f64,
    pub element_type: ElementType,
}

/// Nutrient element type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum ElementType {
    Macro,     // N, P, K
    Secondary, // Ca, Mg, S
    Micro,     // Fe, Mn, Zn, Cu, B, Mo, Cl
}

/// Harvest record
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HarvestRecord {
    pub id: Uuid,
    pub plant_id: Uuid,
    pub harvest_date: DateTime<Utc>,
    pub yield_grams: f64,
    pub quality_score: f64,
    pub notes: Option<String>,
}

/// System alert
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemAlert {
    pub id: Uuid,
    pub system_id: Uuid,
    pub alert_type: AlertType,
    pub severity: AlertSeverity,
    pub message: String,
    pub created_at: DateTime<Utc>,
    pub resolved_at: Option<DateTime<Utc>>,
}

/// Alert type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum AlertType {
    PhImbalance,
    HighTemperature,
    LowWaterLevel,
    PumpFailure,
    NutrientDeficiency,
    DiseaseDetected,
}

/// Alert severity
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum AlertSeverity {
    Info,
    Warning,
    Critical,
}

/// API Response wrapper
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApiResponse<T> {
    pub success: bool,
    pub data: Option<T>,
    pub error: Option<String>,
    pub timestamp: DateTime<Utc>,
}
