//! Type definitions for nanotechnology

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};
use uuid::Uuid;

/// Nanoparticle specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Nanoparticle {
    pub id: Uuid,
    pub name: String,
    pub material: NanoMaterial,
    pub size_nm: f64,
    pub shape: NanoShape,
    pub surface_area_m2: f64,
    pub created_at: DateTime<Utc>,
    pub properties: NanoProperties,
}

/// Nanomaterial types
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum NanoMaterial {
    CarbonNanotube,
    Graphene,
    QuantumDot,
    Fullerene,
    MetalOxide,
    GoldNanoparticle,
    SilverNanoparticle,
    Dendrimer,
}

/// Nanoparticle shapes
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum NanoShape {
    Spherical,
    Rod,
    Wire,
    Tube,
    Sheet,
    Irregular,
}

/// Nanomaterial properties
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NanoProperties {
    pub electrical_conductivity: Option<f64>,
    pub thermal_conductivity: Option<f64>,
    pub optical_properties: Option<OpticalProperties>,
    pub magnetic_properties: Option<MagneticProperties>,
    pub toxicity_level: ToxicityLevel,
}

/// Optical properties
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpticalProperties {
    pub absorption_wavelength_nm: f64,
    pub emission_wavelength_nm: f64,
    pub quantum_yield: f64,
}

/// Magnetic properties
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MagneticProperties {
    pub magnetic_moment: f64,
    pub coercivity: f64,
}

/// Toxicity level classification
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ToxicityLevel {
    Low,
    Medium,
    High,
    Unknown,
}

/// Synthesis parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SynthesisParams {
    pub method: SynthesisMethod,
    pub temperature_celsius: f64,
    pub pressure_kpa: f64,
    pub duration_minutes: f64,
    pub precursors: Vec<String>,
}

/// Synthesis methods
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum SynthesisMethod {
    ChemicalVaporDeposition,
    SolGel,
    Hydrothermal,
    Precipitation,
    BallMilling,
    LaserAblation,
}
