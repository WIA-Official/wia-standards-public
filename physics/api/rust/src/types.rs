//! Type definitions for WIA Physics Standard
//!
//! This module defines all data types corresponding to the JSON schemas.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use uuid::Uuid;
use validator::Validate;

// ============================================================================
// Common Types
// ============================================================================

/// Measurement uncertainty specification
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Uncertainty {
    /// Statistical (random) uncertainty
    #[serde(skip_serializing_if = "Option::is_none")]
    pub statistical: Option<f64>,
    /// Systematic uncertainty
    #[serde(skip_serializing_if = "Option::is_none")]
    pub systematic: Option<f64>,
    /// Total combined uncertainty (required)
    pub total: f64,
}

impl Uncertainty {
    /// Create uncertainty with only total value
    pub fn total(value: f64) -> Self {
        Self {
            statistical: None,
            systematic: None,
            total: value,
        }
    }

    /// Create uncertainty with statistical and systematic components
    pub fn with_components(statistical: f64, systematic: f64) -> Self {
        let total = (statistical.powi(2) + systematic.powi(2)).sqrt();
        Self {
            statistical: Some(statistical),
            systematic: Some(systematic),
            total,
        }
    }
}

/// Physical measurement with uncertainty
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct Measurement {
    /// Central value of measurement
    pub value: f64,
    /// Measurement uncertainty
    pub uncertainty: Uncertainty,
    /// Physical unit
    pub unit: String,
    /// Confidence level (default 0.68 for 1-sigma)
    #[serde(default = "default_confidence_level")]
    pub confidence_level: f64,
}

fn default_confidence_level() -> f64 {
    0.68
}

impl Measurement {
    /// Create a new measurement
    pub fn new(value: f64, uncertainty: f64, unit: &str) -> Self {
        Self {
            value,
            uncertainty: Uncertainty::total(uncertainty),
            unit: unit.to_string(),
            confidence_level: 0.68,
        }
    }

    /// Create measurement with full uncertainty
    pub fn with_uncertainty(
        value: f64,
        statistical: f64,
        systematic: f64,
        unit: &str,
    ) -> Self {
        Self {
            value,
            uncertainty: Uncertainty::with_components(statistical, systematic),
            unit: unit.to_string(),
            confidence_level: 0.68,
        }
    }

    /// Check if value is within uncertainty of another measurement
    pub fn is_compatible_with(&self, other: &Measurement) -> bool {
        let diff = (self.value - other.value).abs();
        let combined_uncertainty =
            (self.uncertainty.total.powi(2) + other.uncertainty.total.powi(2)).sqrt();
        diff <= 2.0 * combined_uncertainty // 2-sigma compatibility
    }
}

/// 3D vector with uncertainty
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vector3D {
    pub x: Measurement,
    pub y: Measurement,
    pub z: Measurement,
    #[serde(default = "default_coordinate_system")]
    pub coordinate_system: String,
}

fn default_coordinate_system() -> String {
    "cartesian".to_string()
}

/// Numerical range
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Range {
    pub min: f64,
    pub max: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,
}

/// Data quality flag
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum QualityFlag {
    Good,
    Suspect,
    Bad,
    Unchecked,
    Simulated,
}

impl Default for QualityFlag {
    fn default() -> Self {
        QualityFlag::Unchecked
    }
}

/// Creator information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Creator {
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub institution: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub orcid: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub email: Option<String>,
}

/// Experiment information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExperimentInfo {
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub facility: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub collaboration: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub run_period: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub detector: Option<String>,
}

/// Standard metadata block
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Metadata {
    pub id: Uuid,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
    pub created: DateTime<Utc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub modified: Option<DateTime<Utc>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub creator: Option<Creator>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub experiment: Option<ExperimentInfo>,
    #[serde(default = "default_license")]
    pub license: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub doi: Option<String>,
    #[serde(default)]
    pub keywords: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

fn default_license() -> String {
    "CC-BY-4.0".to_string()
}

impl Metadata {
    /// Create new metadata with generated ID
    pub fn new() -> Self {
        Self {
            id: Uuid::new_v4(),
            version: Some("1.0.0".to_string()),
            created: Utc::now(),
            modified: None,
            creator: None,
            experiment: None,
            license: default_license(),
            doi: None,
            keywords: vec![],
            description: None,
        }
    }

    /// Create metadata with experiment info
    pub fn with_experiment(experiment_name: &str) -> Self {
        let mut meta = Self::new();
        meta.experiment = Some(ExperimentInfo {
            name: experiment_name.to_string(),
            facility: None,
            collaboration: None,
            run_period: None,
            detector: None,
        });
        meta
    }
}

impl Default for Metadata {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Fusion Types
// ============================================================================

/// Fusion fuel type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum FuelType {
    #[serde(rename = "D-D")]
    DD,
    #[serde(rename = "D-T")]
    DT,
    #[serde(rename = "D-He3")]
    DHe3,
    #[serde(rename = "p-B11")]
    PB11,
    #[serde(rename = "pure-D")]
    PureD,
    #[serde(rename = "hydrogen")]
    Hydrogen,
}

/// Magnetic confinement type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ConfinementType {
    Tokamak,
    Stellarator,
    SphericalTokamak,
    Frc,
    Mirror,
    LevitatedDipole,
    ZPinch,
    Inertial,
}

/// Heating system type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum HeatingType {
    Nbi,
    Icrh,
    Ecrh,
    Lhcd,
    Ohmic,
}

/// Fuel composition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FuelComposition {
    #[serde(rename = "type")]
    pub fuel_type: FuelType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub deuterium_fraction: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tritium_fraction: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub helium3_fraction: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub impurity_fraction: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub z_effective: Option<Measurement>,
}

/// Plasma parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlasmaParameters {
    pub temperature: Measurement,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub electron_temperature: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ion_temperature: Option<Measurement>,
    pub density: Measurement,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub electron_density: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ion_density: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confinement_time: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub triple_product: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub beta: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fuel_composition: Option<FuelComposition>,
}

/// Magnetic configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MagneticConfiguration {
    #[serde(rename = "type")]
    pub confinement_type: ConfinementType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub toroidal_field: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub poloidal_field: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub plasma_current: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub major_radius: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub minor_radius: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub aspect_ratio: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub plasma_volume: Option<Measurement>,
}

/// Energy balance data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnergyBalance {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub input_power: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fusion_power: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub q_factor: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub neutron_power: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alpha_heating_power: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub radiation_loss: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stored_energy: Option<Measurement>,
}

/// Heating system
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HeatingSystem {
    #[serde(rename = "type")]
    pub heating_type: HeatingType,
    pub power: Measurement,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub efficiency: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frequency: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub beam_energy: Option<Measurement>,
}

/// Complete fusion data structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FusionData {
    pub metadata: Metadata,
    pub plasma: PlasmaParameters,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub magnetic_configuration: Option<MagneticConfiguration>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub energy_balance: Option<EnergyBalance>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub heating_systems: Option<Vec<HeatingSystem>>,
    #[serde(default)]
    pub quality: QualityFlag,
}

// ============================================================================
// Time Crystal Types
// ============================================================================

/// Time crystal type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum TimeCrystalType {
    Discrete,
    Continuous,
    Floquet,
    Prethermal,
    Dissipative,
}

/// Physical system type for time crystals
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum SystemType {
    Magnon,
    SpinChain,
    TrappedIon,
    SuperconductingQubit,
    LiquidCrystal,
    Bec,
    Photonic,
    Phononic,
    NitrogenVacancy,
}

/// Drive type for time crystals
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum DriveType {
    Microwave,
    Laser,
    MagneticPulse,
    ElectricPulse,
    Acoustic,
    AmbientLight,
    None,
}

/// Time crystal parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeCrystalParameters {
    #[serde(rename = "type")]
    pub crystal_type: TimeCrystalType,
    pub oscillation_period: Measurement,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub oscillation_frequency: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub driving_frequency: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub period_ratio: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub subharmonic_order: Option<i32>,
    pub coherence_time: Measurement,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub coherence_cycles: Option<i64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lifetime: Option<Measurement>,
}

/// Material system for time crystals
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MaterialSystem {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub material: Option<String>,
    pub system_type: SystemType,
    pub temperature: Measurement,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub particle_count: Option<i64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub magnetic_field: Option<Measurement>,
}

/// Quantum properties
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuantumProperties {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub entanglement_type: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub qubit_count: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fidelity: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub purity: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error_rate: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub t1_time: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub t2_time: Option<Measurement>,
}

/// Driving parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrivingParameters {
    pub drive_type: DriveType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frequency: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub amplitude: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub power: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wavelength: Option<Measurement>,
}

/// Complete time crystal data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeCrystalData {
    pub metadata: Metadata,
    pub time_crystal: TimeCrystalParameters,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub material_system: Option<MaterialSystem>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quantum_properties: Option<QuantumProperties>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub driving_parameters: Option<DrivingParameters>,
    #[serde(default)]
    pub quality: QualityFlag,
}

// ============================================================================
// Particle Physics Types
// ============================================================================

/// Particle type classification
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ParticleType {
    Fermion,
    Boson,
    Lepton,
    Quark,
    Hadron,
    Meson,
    Baryon,
    GaugeBoson,
    Scalar,
}

/// Particle properties
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParticleProperties {
    pub name: String,
    pub pdg_id: i32,
    pub mass: Measurement,
    pub charge: f64,
    pub spin: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parity: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lifetime: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub width: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub particle_type: Option<ParticleType>,
}

/// Four-momentum vector
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FourMomentum {
    pub e: Measurement,
    pub px: Measurement,
    pub py: Measurement,
    pub pz: Measurement,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pt: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub eta: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub phi: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mass: Option<Measurement>,
}

impl FourMomentum {
    /// Calculate invariant mass
    pub fn invariant_mass(&self) -> f64 {
        let m2 = self.e.value.powi(2)
            - self.px.value.powi(2)
            - self.py.value.powi(2)
            - self.pz.value.powi(2);
        if m2 >= 0.0 {
            m2.sqrt()
        } else {
            0.0
        }
    }

    /// Calculate transverse momentum
    pub fn transverse_momentum(&self) -> f64 {
        (self.px.value.powi(2) + self.py.value.powi(2)).sqrt()
    }

    /// Calculate pseudorapidity
    pub fn pseudorapidity(&self) -> f64 {
        let p = (self.px.value.powi(2) + self.py.value.powi(2) + self.pz.value.powi(2)).sqrt();
        0.5 * ((p + self.pz.value) / (p - self.pz.value)).ln()
    }
}

/// Collision information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CollisionInfo {
    pub sqrt_s: Measurement,
    pub beam1: String,
    pub beam2: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub collision_type: Option<String>,
}

/// Missing transverse energy
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissingET {
    pub met: Measurement,
    pub phi: Measurement,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sum_et: Option<Measurement>,
}

/// Collision event data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CollisionEvent {
    pub event_id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub run_number: Option<i64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub event_number: Option<i64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timestamp: Option<DateTime<Utc>>,
    pub collision: CollisionInfo,
    #[serde(default)]
    pub jets: Vec<FourMomentum>,
    #[serde(default)]
    pub electrons: Vec<FourMomentum>,
    #[serde(default)]
    pub muons: Vec<FourMomentum>,
    #[serde(default)]
    pub photons: Vec<FourMomentum>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub missing_et: Option<MissingET>,
    #[serde(default)]
    pub trigger: Vec<String>,
}

/// Cross-section data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CrossSectionData {
    pub process: String,
    pub sqrt_s: Measurement,
    pub cross_section: Measurement,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub integrated_luminosity: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub significance: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub theory_prediction: Option<Measurement>,
}

/// Complete particle physics data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParticleData {
    pub metadata: Metadata,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub particle: Option<ParticleProperties>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub event: Option<CollisionEvent>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cross_section: Option<CrossSectionData>,
    #[serde(default)]
    pub quality: QualityFlag,
}

// ============================================================================
// Dark Matter Types
// ============================================================================

/// Dark matter candidate type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum DarkMatterCandidate {
    Wimp,
    Axion,
    Alp,
    DarkPhoton,
    SterileNeutrino,
    HiddenSector,
    PrimordialBlackHole,
}

/// Interaction type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum InteractionType {
    SI,
    #[serde(rename = "SD_proton")]
    SDProton,
    #[serde(rename = "SD_neutron")]
    SDNeutron,
    #[serde(rename = "axion_photon")]
    AxionPhoton,
    #[serde(rename = "axion_electron")]
    AxionElectron,
    #[serde(rename = "axion_nucleon")]
    AxionNucleon,
}

/// Signal type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum SignalType {
    NuclearRecoil,
    ElectronRecoil,
    MultipleScatter,
    Unknown,
}

/// Detection event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectionEvent {
    pub event_id: String,
    pub timestamp: DateTime<Utc>,
    pub recoil_energy: Measurement,
    pub signal_type: SignalType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub position: Option<Vector3D>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fiducial: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub passed_cuts: Option<bool>,
}

/// Exclusion limit
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExclusionLimit {
    pub dm_candidate: DarkMatterCandidate,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dm_mass: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mass_range: Option<Range>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cross_section_limit: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub coupling_limit: Option<Measurement>,
    pub confidence_level: f64,
    pub interaction_type: InteractionType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exposure: Option<Measurement>,
}

/// Axion search parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AxionSearch {
    pub search_type: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frequency_range: Option<Range>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub axion_mass_range: Option<Range>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub coupling_type: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gagg_limit: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub magnetic_field: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quality_factor: Option<Measurement>,
}

/// Complete dark matter data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DarkMatterData {
    pub metadata: Metadata,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub detection_event: Option<DetectionEvent>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exclusion_limit: Option<ExclusionLimit>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub axion_search: Option<AxionSearch>,
    #[serde(default)]
    pub quality: QualityFlag,
}

// ============================================================================
// Antimatter Types
// ============================================================================

/// Antiparticle type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum AntiparticleType {
    Antiproton,
    Positron,
    Antihydrogen,
    Antideuteron,
    Antihelium3,
    Antihelium4,
    Other,
}

/// Trap type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum TrapType {
    Penning,
    Paul,
    MagneticMinimum,
    Ioffe,
    Combined,
    Optical,
}

/// Antiparticle properties
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AntiparticleProperties {
    pub particle_type: AntiparticleType,
    pub count: i64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub temperature: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub kinetic_energy: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub storage_time: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quantum_state: Option<String>,
}

/// Trap parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrapParameters {
    pub trap_type: TrapType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trap_name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub magnetic_field: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trap_depth: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vacuum_pressure: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub base_temperature: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub storage_lifetime: Option<Measurement>,
    #[serde(default)]
    pub portable: bool,
}

/// Spectroscopic measurement
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpectroscopicMeasurement {
    pub transition: String,
    pub frequency: Measurement,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wavelength: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub linewidth: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hydrogen_frequency: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frequency_difference: Option<Measurement>,
}

/// CPT test result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CptTestResult {
    pub test_type: String,
    pub antimatter_value: Measurement,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub matter_value: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub relative_difference: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cpt_violation_limit: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidence_level: Option<f64>,
    #[serde(default)]
    pub consistent_with_cpt: bool,
}

/// Complete antimatter data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AntimatterData {
    pub metadata: Metadata,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub antiparticle: Option<AntiparticleProperties>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trap: Option<TrapParameters>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub spectroscopy: Option<SpectroscopicMeasurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cpt_test: Option<CptTestResult>,
    #[serde(default)]
    pub quality: QualityFlag,
}

// ============================================================================
// Quantum Gravity Types
// ============================================================================

/// Quantum gravity theory
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum QuantumGravityTheory {
    Lqg,
    StringTheory,
    AsymptoticSafety,
    Cdt,
    EmergentGravity,
    MassiveGravity,
    HoravaLifshitz,
    EntropicGravity,
    Other,
}

/// String theory type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum StringType {
    TypeI,
    TypeIia,
    TypeIib,
    HeteroticSo32,
    #[serde(rename = "heterotic_E8xE8")]
    HeteroticE8xE8,
    MTheory,
}

/// Theoretical prediction
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TheoreticalPrediction {
    pub theory: QuantumGravityTheory,
    pub prediction_type: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub observable: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub value: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub classical_value: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quantum_correction: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub validity_regime: Option<String>,
}

/// LQG-specific data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LqgData {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub immirzi_parameter: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub area_gap: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cosmological_constant: Option<Measurement>,
}

/// String theory data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StringTheoryData {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub string_type: Option<StringType>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dimensions: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub string_coupling: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub string_length: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub susy_breaking_scale: Option<Measurement>,
}

/// Black hole quantum data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BlackHoleData {
    pub mass: Measurement,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub angular_momentum: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub charge: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub horizon_area: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bekenstein_hawking_entropy: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quantum_entropy: Option<Measurement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hawking_temperature: Option<Measurement>,
}

/// Complete quantum gravity data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuantumGravityData {
    pub metadata: Metadata,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub theoretical_prediction: Option<TheoreticalPrediction>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lqg_data: Option<LqgData>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub string_theory_data: Option<StringTheoryData>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub black_hole: Option<BlackHoleData>,
    #[serde(default)]
    pub quality: QualityFlag,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_measurement_creation() {
        let m = Measurement::new(100.0, 5.0, "GeV");
        assert_eq!(m.value, 100.0);
        assert_eq!(m.uncertainty.total, 5.0);
        assert_eq!(m.unit, "GeV");
    }

    #[test]
    fn test_uncertainty_calculation() {
        let u = Uncertainty::with_components(3.0, 4.0);
        assert!((u.total - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_measurement_compatibility() {
        let m1 = Measurement::new(100.0, 5.0, "GeV");
        let m2 = Measurement::new(102.0, 5.0, "GeV");
        assert!(m1.is_compatible_with(&m2));

        let m3 = Measurement::new(120.0, 5.0, "GeV");
        assert!(!m1.is_compatible_with(&m3));
    }

    #[test]
    fn test_four_momentum() {
        let p = FourMomentum {
            e: Measurement::new(100.0, 1.0, "GeV"),
            px: Measurement::new(30.0, 0.5, "GeV"),
            py: Measurement::new(40.0, 0.5, "GeV"),
            pz: Measurement::new(0.0, 0.5, "GeV"),
            pt: None,
            eta: None,
            phi: None,
            mass: None,
        };
        assert!((p.transverse_momentum() - 50.0).abs() < 1e-10);
    }

    #[test]
    fn test_serialization() {
        let m = Measurement::new(125.25, 0.18, "GeV");
        let json = serde_json::to_string(&m).unwrap();
        let m2: Measurement = serde_json::from_str(&json).unwrap();
        assert_eq!(m.value, m2.value);
    }
}
