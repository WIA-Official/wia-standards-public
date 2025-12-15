//! Type definitions for WIA Space Standard
//!
//! This module contains all the core data types used in the WIA Space API.

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};

// ============================================================================
// Common Types
// ============================================================================

/// Technology category
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TechnologyCategory {
    DysonSphere,
    MarsTerraforming,
    WarpDrive,
    SpaceElevator,
    AsteroidMining,
    InterstellarTravel,
}

impl TechnologyCategory {
    /// Get the schema URL for this category
    pub fn schema_url(&self) -> &'static str {
        match self {
            Self::DysonSphere => "https://wia.live/schemas/space/dyson-sphere.schema.json",
            Self::MarsTerraforming => "https://wia.live/schemas/space/mars-terraforming.schema.json",
            Self::WarpDrive => "https://wia.live/schemas/space/warp-drive.schema.json",
            Self::SpaceElevator => "https://wia.live/schemas/space/space-elevator.schema.json",
            Self::AsteroidMining => "https://wia.live/schemas/space/asteroid-mining.schema.json",
            Self::InterstellarTravel => "https://wia.live/schemas/space/interstellar-travel.schema.json",
        }
    }
}

impl std::fmt::Display for TechnologyCategory {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::DysonSphere => write!(f, "dyson_sphere"),
            Self::MarsTerraforming => write!(f, "mars_terraforming"),
            Self::WarpDrive => write!(f, "warp_drive"),
            Self::SpaceElevator => write!(f, "space_elevator"),
            Self::AsteroidMining => write!(f, "asteroid_mining"),
            Self::InterstellarTravel => write!(f, "interstellar_travel"),
        }
    }
}

/// Technology Readiness Level (TRL)
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct TechnologyReadinessLevel(u8);

impl TechnologyReadinessLevel {
    /// Create a new TRL (1-9)
    pub fn new(level: u8) -> Option<Self> {
        if (1..=9).contains(&level) {
            Some(Self(level))
        } else {
            None
        }
    }

    /// Get the TRL value
    pub fn level(&self) -> u8 {
        self.0
    }

    /// Get description of this TRL
    pub fn description(&self) -> &'static str {
        match self.0 {
            1 => "Basic Principles Observed",
            2 => "Technology Concept Formulated",
            3 => "Experimental Proof of Concept",
            4 => "Laboratory Validation",
            5 => "Relevant Environment Validation",
            6 => "Demonstration in Relevant Environment",
            7 => "System Prototype Demonstration",
            8 => "System Complete and Qualified",
            9 => "Operational System",
            _ => "Unknown",
        }
    }
}

impl Default for TechnologyReadinessLevel {
    fn default() -> Self {
        Self(1)
    }
}

/// 3D coordinate in various reference frames
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Coordinate3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Coordinate3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn origin() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    pub fn magnitude(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
}

/// Velocity vector
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Velocity3D {
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
}

impl Velocity3D {
    pub fn new(vx: f64, vy: f64, vz: f64) -> Self {
        Self { vx, vy, vz }
    }

    pub fn magnitude(&self) -> f64 {
        (self.vx * self.vx + self.vy * self.vy + self.vz * self.vz).sqrt()
    }
}

/// Orbital parameters (Keplerian elements)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OrbitalParameters {
    /// Semi-major axis in AU
    pub semi_major_axis_au: f64,
    /// Eccentricity (0-1 for elliptical)
    pub eccentricity: f64,
    /// Inclination in degrees
    pub inclination_deg: f64,
    /// Orbital period in years
    pub orbital_period_years: f64,
    /// Perihelion distance in AU
    #[serde(skip_serializing_if = "Option::is_none")]
    pub perihelion_au: Option<f64>,
    /// Aphelion distance in AU
    #[serde(skip_serializing_if = "Option::is_none")]
    pub aphelion_au: Option<f64>,
}

impl OrbitalParameters {
    pub fn new(semi_major_axis_au: f64, eccentricity: f64, inclination_deg: f64) -> Self {
        let perihelion = semi_major_axis_au * (1.0 - eccentricity);
        let aphelion = semi_major_axis_au * (1.0 + eccentricity);
        let period = semi_major_axis_au.powf(1.5); // Kepler's third law (in years for AU)

        Self {
            semi_major_axis_au,
            eccentricity,
            inclination_deg,
            orbital_period_years: period,
            perihelion_au: Some(perihelion),
            aphelion_au: Some(aphelion),
        }
    }
}

// ============================================================================
// Star Parameters
// ============================================================================

/// Stellar classification
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StarParameters {
    /// Star name
    pub name: String,
    /// Spectral type (e.g., "G2V", "M5V")
    #[serde(rename = "type")]
    pub spectral_type: String,
    /// Mass in solar masses
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mass_solar: Option<f64>,
    /// Luminosity in solar luminosities
    pub luminosity_solar: f64,
    /// Radius in solar radii
    #[serde(skip_serializing_if = "Option::is_none")]
    pub radius_solar: Option<f64>,
    /// Effective temperature in Kelvin
    #[serde(skip_serializing_if = "Option::is_none")]
    pub temperature_kelvin: Option<f64>,
}

impl StarParameters {
    pub fn sol() -> Self {
        Self {
            name: "Sol".to_string(),
            spectral_type: "G2V".to_string(),
            mass_solar: Some(1.0),
            luminosity_solar: 1.0,
            radius_solar: Some(1.0),
            temperature_kelvin: Some(5778.0),
        }
    }

    pub fn new(name: impl Into<String>, spectral_type: impl Into<String>, luminosity_solar: f64) -> Self {
        Self {
            name: name.into(),
            spectral_type: spectral_type.into(),
            mass_solar: None,
            luminosity_solar,
            radius_solar: None,
            temperature_kelvin: None,
        }
    }
}

// ============================================================================
// Dyson Sphere Types
// ============================================================================

/// Dyson structure type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DysonStructureType {
    DysonSwarm,
    DysonRing,
    DysonBubble,
    PartialSphere,
}

/// Energy collection method
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EnergyCollectionMethod {
    Photovoltaic,
    Thermal,
    Photoelectric,
    Hybrid,
}

/// Energy transmission method
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EnergyTransmissionMethod {
    MicrowaveBeam,
    Laser,
    PhysicalTransport,
    None,
}

// ============================================================================
// Mars Terraforming Types
// ============================================================================

/// Celestial body type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CelestialBodyType {
    Planet,
    Moon,
    DwarfPlanet,
}

/// Atmosphere composition
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AtmosphereComposition {
    #[serde(rename = "CO2_percent", skip_serializing_if = "Option::is_none")]
    pub co2_percent: Option<f64>,
    #[serde(rename = "N2_percent", skip_serializing_if = "Option::is_none")]
    pub n2_percent: Option<f64>,
    #[serde(rename = "O2_percent", skip_serializing_if = "Option::is_none")]
    pub o2_percent: Option<f64>,
    #[serde(rename = "Ar_percent", skip_serializing_if = "Option::is_none")]
    pub ar_percent: Option<f64>,
    #[serde(rename = "H2O_percent", skip_serializing_if = "Option::is_none")]
    pub h2o_percent: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub other_percent: Option<f64>,
}

impl AtmosphereComposition {
    pub fn mars_current() -> Self {
        Self {
            co2_percent: Some(95.32),
            n2_percent: Some(2.7),
            o2_percent: Some(0.13),
            ar_percent: Some(1.6),
            h2o_percent: None,
            other_percent: Some(0.25),
        }
    }

    pub fn earth_like() -> Self {
        Self {
            co2_percent: Some(0.04),
            n2_percent: Some(78.0),
            o2_percent: Some(21.0),
            ar_percent: Some(0.93),
            h2o_percent: None,
            other_percent: Some(0.03),
        }
    }
}

/// Terraforming intervention method
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TerraformingMethod {
    SolarMirrors,
    GreenhouseGasRelease,
    SilicaAerogel,
    AsteroidImpact,
    NuclearHeating,
    IceCapDarkening,
    SyntheticBiology,
}

// ============================================================================
// Warp Drive Types
// ============================================================================

/// Warp drive type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WarpDriveType {
    AlcubierreFtl,
    AlcubierreSubluminal,
    Natario,
    WhiteJuday,
    VanDenBroeck,
    KrasnikovTube,
}

/// Warp bubble shape
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BubbleShape {
    Spherical,
    OblateSpheroid,
    ProlateSpheroid,
    Toroidal,
}

/// Energy source type for propulsion
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PropulsionEnergySource {
    AntimatterAnnihilation,
    Fusion,
    ZeroPoint,
    CasimirEffect,
    ExoticMatter,
    Theoretical,
}

// ============================================================================
// Space Elevator Types
// ============================================================================

/// Tether material
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TetherMaterial {
    CarbonNanotube,
    SingleCrystalGraphene,
    HBoronNitride,
    CarbonFiber,
    DiamondNanothread,
    Theoretical,
}

impl TetherMaterial {
    /// Get tensile strength in GPa
    pub fn tensile_strength_gpa(&self) -> f64 {
        match self {
            Self::CarbonNanotube => 100.0,
            Self::SingleCrystalGraphene => 130.0,
            Self::HBoronNitride => 100.0,
            Self::CarbonFiber => 7.0,
            Self::DiamondNanothread => 120.0,
            Self::Theoretical => 200.0,
        }
    }
}

/// Climber power source
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClimberPowerSource {
    LaserBeamed,
    MicrowaveBeamed,
    Solar,
    Nuclear,
    Battery,
}

/// Counterweight type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CounterweightType {
    CapturedAsteroid,
    ArtificialMass,
    ExtendedTether,
    Station,
}

// ============================================================================
// Asteroid Mining Types
// ============================================================================

/// Asteroid spectral type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AsteroidType {
    CType,
    SType,
    MType,
    VType,
    DType,
    PType,
    XType,
}

impl AsteroidType {
    /// Get primary resources for this asteroid type
    pub fn primary_resources(&self) -> &'static [&'static str] {
        match self {
            Self::CType => &["Water", "Carbon", "Organics"],
            Self::SType => &["Silicates", "Nickel", "Iron"],
            Self::MType => &["Iron", "Nickel", "Platinum Group"],
            Self::VType => &["Basaltic minerals"],
            Self::DType => &["Organic compounds", "Ice"],
            Self::PType => &["Organic compounds", "Silicates"],
            Self::XType => &["Metallic minerals"],
        }
    }
}

/// Asteroid location
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AsteroidLocation {
    MainBelt,
    NearEarth,
    Trojan,
    KuiperBelt,
    Other,
}

/// Resource extraction method
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExtractionMethod {
    OpticalMining,
    MechanicalExcavation,
    MagneticSeparation,
    Electrolysis,
    ChemicalExtraction,
    ThermalProcessing,
}

/// Extracted resource
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Resource {
    /// Element symbol (e.g., "Fe", "Ni", "Au")
    pub element: String,
    /// Element name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Fraction of total mass
    pub mass_fraction: f64,
    /// Estimated mass in kg
    #[serde(skip_serializing_if = "Option::is_none")]
    pub estimated_mass_kg: Option<f64>,
    /// Estimated value in USD
    #[serde(skip_serializing_if = "Option::is_none")]
    pub estimated_value_usd: Option<f64>,
}

impl Resource {
    pub fn new(element: impl Into<String>, mass_fraction: f64) -> Self {
        Self {
            element: element.into(),
            name: None,
            mass_fraction,
            estimated_mass_kg: None,
            estimated_value_usd: None,
        }
    }

    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    pub fn with_mass(mut self, mass_kg: f64) -> Self {
        self.estimated_mass_kg = Some(mass_kg);
        self
    }

    pub fn with_value(mut self, value_usd: f64) -> Self {
        self.estimated_value_usd = Some(value_usd);
        self
    }
}

// ============================================================================
// Interstellar Travel Types
// ============================================================================

/// Mission type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MissionType {
    Flyby,
    Orbit,
    Landing,
    SampleReturn,
    Colony,
}

/// Propulsion type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PropulsionType {
    LaserLightsail,
    SolarSail,
    NuclearThermal,
    NuclearPulse,
    Fusion,
    Antimatter,
    BussardRamjet,
    WarpDrive,
}

impl PropulsionType {
    /// Get theoretical max velocity as fraction of c
    pub fn max_velocity_c(&self) -> f64 {
        match self {
            Self::LaserLightsail => 0.20,
            Self::SolarSail => 0.001,
            Self::NuclearThermal => 0.0001,
            Self::NuclearPulse => 0.05,
            Self::Fusion => 0.10,
            Self::Antimatter => 0.50,
            Self::BussardRamjet => 0.12,
            Self::WarpDrive => 1.0, // FTL theoretical
        }
    }
}

/// Spacecraft type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpacecraftType {
    Starchip,
    Probe,
    CrewedVessel,
    GenerationShip,
    Worldship,
}

// ============================================================================
// Project Metadata
// ============================================================================

/// Project status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProjectStatus {
    Planned,
    Active,
    Completed,
    Cancelled,
    OnHold,
}

/// Project metadata
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProjectMetadata {
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
    pub version: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub license: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub keywords: Option<Vec<String>>,
}

impl Default for ProjectMetadata {
    fn default() -> Self {
        Self {
            created_at: Utc::now(),
            updated_at: Utc::now(),
            version: "1.0.0".to_string(),
            license: Some("MIT".to_string()),
            keywords: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_technology_category() {
        let cat = TechnologyCategory::AsteroidMining;
        assert_eq!(cat.to_string(), "asteroid_mining");
    }

    #[test]
    fn test_trl() {
        let trl = TechnologyReadinessLevel::new(5).unwrap();
        assert_eq!(trl.level(), 5);
        assert_eq!(trl.description(), "Relevant Environment Validation");
    }

    #[test]
    fn test_trl_invalid() {
        assert!(TechnologyReadinessLevel::new(0).is_none());
        assert!(TechnologyReadinessLevel::new(10).is_none());
    }

    #[test]
    fn test_orbital_parameters() {
        let orbit = OrbitalParameters::new(1.0, 0.0167, 0.0);
        assert!((orbit.orbital_period_years - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_asteroid_resources() {
        let m_type = AsteroidType::MType;
        assert!(m_type.primary_resources().contains(&"Platinum Group"));
    }
}
