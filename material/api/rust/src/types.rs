//! Type definitions for WIA Material Science Standards
//!
//! This module contains all the data types used in the WIA Material SDK,
//! matching the JSON Schema definitions from Phase 1.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use validator::Validate;

/// Material type identifiers
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MaterialType {
    Superconductor,
    Metamaterial,
    ProgrammableMatter,
    HolographicStorage,
    Memristor,
    TopologicalInsulator,
    Custom,
}

impl std::fmt::Display for MaterialType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MaterialType::Superconductor => write!(f, "superconductor"),
            MaterialType::Metamaterial => write!(f, "metamaterial"),
            MaterialType::ProgrammableMatter => write!(f, "programmable_matter"),
            MaterialType::HolographicStorage => write!(f, "holographic_storage"),
            MaterialType::Memristor => write!(f, "memristor"),
            MaterialType::TopologicalInsulator => write!(f, "topological_insulator"),
            MaterialType::Custom => write!(f, "custom"),
        }
    }
}

/// Crystal system enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CrystalSystem {
    Cubic,
    Tetragonal,
    Orthorhombic,
    Hexagonal,
    Trigonal,
    Rhombohedral,
    Monoclinic,
    Triclinic,
    Amorphous,
}

/// Timestamp information
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct Timestamp {
    pub created: DateTime<Utc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub modified: Option<DateTime<Utc>>,
}

impl Default for Timestamp {
    fn default() -> Self {
        Self {
            created: Utc::now(),
            modified: None,
        }
    }
}

/// Material identity information
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct Identity {
    #[validate(length(min = 1))]
    pub name: String,
    #[validate(length(min = 1))]
    pub formula: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub classification: Option<Vec<String>>,
}

/// Lattice parameters for crystal structure
#[derive(Debug, Clone, Serialize, Deserialize, Validate, Default)]
pub struct LatticeParameters {
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0))]
    pub a_angstrom: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0))]
    pub b_angstrom: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0))]
    pub c_angstrom: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 180.0))]
    pub alpha_degree: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 180.0))]
    pub beta_degree: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 180.0))]
    pub gamma_degree: Option<f64>,
}

/// Crystal structure information
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Structure {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub crystal_system: Option<CrystalSystem>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub space_group: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lattice_parameters: Option<LatticeParameters>,
}

/// Electrical properties
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ElectricalProperties {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub resistivity_ohm_m: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub conductivity_s_m: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub carrier_mobility_cm2_vs: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub carrier_concentration_cm3: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub band_gap_ev: Option<f64>,
}

/// Magnetic properties
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct MagneticProperties {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub magnetic_susceptibility: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub permeability: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub saturation_magnetization_a_m: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub coercivity_a_m: Option<f64>,
}

/// Thermal properties
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ThermalProperties {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub thermal_conductivity_w_mk: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub specific_heat_j_kgk: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub melting_point_k: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub thermal_expansion_k: Option<f64>,
}

/// Optical properties
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct OpticalProperties {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub refractive_index: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub absorption_coefficient_cm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub transmittance_percent: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reflectance_percent: Option<f64>,
}

/// Mechanical properties
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct MechanicalProperties {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub density_kg_m3: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub youngs_modulus_gpa: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub poissons_ratio: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hardness_gpa: Option<f64>,
}

/// Material properties container
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Properties {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub electrical: Option<ElectricalProperties>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub magnetic: Option<MagneticProperties>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub thermal: Option<ThermalProperties>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub optical: Option<OpticalProperties>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mechanical: Option<MechanicalProperties>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub domain_specific: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub custom: Option<HashMap<String, serde_json::Value>>,
}

/// Measurement conditions
#[derive(Debug, Clone, Serialize, Deserialize, Default, Validate)]
pub struct Measurement {
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0))]
    pub temperature_k: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0))]
    pub pressure_pa: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub magnetic_field_t: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub electric_field_v_m: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub method: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub instrument: Option<String>,
}

/// Data provenance information
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Provenance {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lab: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub operator: Option<String>,
}

/// External database references
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ExternalReferences {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub materials_project_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub icsd_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cod_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub doi: Option<String>,
}

/// Metadata
#[derive(Debug, Clone, Serialize, Deserialize, Default, Validate)]
pub struct Meta {
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 1.0))]
    pub confidence: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub validated: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub notes: Option<String>,
}

/// Main Material Data structure
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct MaterialData {
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,

    pub version: String,

    pub material_type: MaterialType,

    #[validate(length(min = 16, max = 16))]
    pub material_id: String,

    #[validate(nested)]
    pub timestamp: Timestamp,

    #[validate(nested)]
    pub identity: Identity,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub structure: Option<Structure>,

    pub properties: Properties,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(nested)]
    pub measurement: Option<Measurement>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub provenance: Option<Provenance>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub external_references: Option<ExternalReferences>,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(nested)]
    pub meta: Option<Meta>,
}

lazy_static::lazy_static! {
    pub static ref MATERIAL_ID_REGEX: regex::Regex =
        regex::Regex::new(r"^wia-mat-[a-zA-Z0-9]{8}$").unwrap();
}

// ============================================================================
// Domain-Specific Types
// ============================================================================

/// Superconductor type classification
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SuperconductorType {
    TypeI,
    TypeII,
}

/// Superconductor-specific properties
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct SuperconductorProperties {
    #[validate(range(min = 0.0))]
    pub critical_temperature_k: f64,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0))]
    pub critical_pressure_pa: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0))]
    pub critical_current_density_a_m2: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0))]
    pub critical_magnetic_field_t: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub meissner_effect: Option<bool>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub superconductor_type: Option<SuperconductorType>,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0))]
    pub coherence_length_nm: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0))]
    pub penetration_depth_nm: Option<f64>,
}

/// Metamaterial type classification
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MetamaterialType {
    Electromagnetic,
    Acoustic,
    Mechanical,
    Photonic,
}

/// Unit cell structure for metamaterials
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UnitCell {
    #[serde(rename = "type")]
    pub cell_type: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub period_um: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dimensions: Option<HashMap<String, f64>>,
}

/// Metamaterial-specific properties
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MetamaterialProperties {
    pub metamaterial_type: MetamaterialType,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit_cell: Option<UnitCell>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub permittivity_real: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub permittivity_imag: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub permeability_real: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub permeability_imag: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub refractive_index: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub operating_frequency_hz: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub absorption_percent: Option<f64>,
}

/// Memristor structure type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MemristorStructure {
    MetalInsulatorMetal,
    MetalOxideSemiconductor,
    Organic,
    Hybrid,
}

/// Neuromorphic properties for memristors
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct NeuromorphicProperties {
    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 1.0))]
    pub synaptic_weight: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub plasticity: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 1.0))]
    pub learning_rate: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 2))]
    pub analog_states: Option<i32>,
}

/// Memristor-specific properties
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct MemristorProperties {
    pub material: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub structure: Option<MemristorStructure>,

    #[validate(range(min = 0.0))]
    pub resistance_high_ohm: f64,

    #[validate(range(min = 0.0))]
    pub resistance_low_ohm: f64,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 1.0))]
    pub on_off_ratio: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub set_voltage_v: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub reset_voltage_v: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0))]
    pub endurance_cycles: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(nested)]
    pub neuromorphic: Option<NeuromorphicProperties>,
}

/// Topological surface state properties
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SurfaceState {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fermi_velocity_m_s: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub spin_texture: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub surface_conductivity_s: Option<f64>,
}

/// Topological insulator-specific properties
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct TopologicalInsulatorProperties {
    #[validate(range(min = 0.0))]
    pub band_gap_ev: f64,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub z2_invariant: Option<Vec<i32>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub dirac_point_ev: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub surface_state: Option<SurfaceState>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub spin_hall_angle: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0))]
    pub spin_diffusion_length_nm: Option<f64>,
}

/// Holographic storage medium type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HolographicMediumType {
    PhotorefractiveCrystal,
    Photopolymer,
    Bacteriorhodopsin,
    Other,
}

/// Holographic storage-specific properties
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct HolographicStorageProperties {
    pub medium_type: HolographicMediumType,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub material: Option<String>,

    #[validate(range(min = 0.0))]
    pub capacity_gb: f64,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0))]
    pub wavelength_nm: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub multiplexing_method: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 1))]
    pub hologram_count: Option<i32>,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0))]
    pub read_speed_mbps: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0))]
    pub retention_years: Option<f64>,
}

/// 3D Position
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Position3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Programmable matter module
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProgrammableModule {
    pub module_id: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub module_type: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub position: Option<Position3D>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub bonds: Option<Vec<String>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub power_state: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub energy_level_percent: Option<f64>,
}

/// Programmable matter-specific properties
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct ProgrammableMatterProperties {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub system_type: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub ensemble_id: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub target_shape: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 1))]
    pub module_count: Option<i32>,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate(range(min = 0.0, max = 100.0))]
    pub shape_accuracy_percent: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub modules: Option<Vec<ProgrammableModule>>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_material_type_display() {
        assert_eq!(MaterialType::Superconductor.to_string(), "superconductor");
        assert_eq!(MaterialType::TopologicalInsulator.to_string(), "topological_insulator");
    }

    #[test]
    fn test_material_id_regex() {
        assert!(MATERIAL_ID_REGEX.is_match("wia-mat-00000001"));
        assert!(MATERIAL_ID_REGEX.is_match("wia-mat-abcd1234"));
        assert!(!MATERIAL_ID_REGEX.is_match("wia-mat-123"));
        assert!(!MATERIAL_ID_REGEX.is_match("invalid-id"));
    }

    #[test]
    fn test_timestamp_default() {
        let ts = Timestamp::default();
        assert!(ts.modified.is_none());
    }
}
