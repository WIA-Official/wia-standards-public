//! Core physics calculations and utilities
//!
//! This module provides physics calculations, unit conversions,
//! and validation logic for the WIA Physics Standard.

use crate::error::{PhysicsError, PhysicsResult};
use crate::types::*;

// ============================================================================
// Physical Constants
// ============================================================================

/// Physical constants in SI units
pub mod constants {
    /// Speed of light (m/s)
    pub const SPEED_OF_LIGHT: f64 = 299_792_458.0;

    /// Planck constant (J·s)
    pub const PLANCK_CONSTANT: f64 = 6.626_070_15e-34;

    /// Reduced Planck constant (J·s)
    pub const HBAR: f64 = 1.054_571_817e-34;

    /// Boltzmann constant (J/K)
    pub const BOLTZMANN: f64 = 1.380_649e-23;

    /// Elementary charge (C)
    pub const ELEMENTARY_CHARGE: f64 = 1.602_176_634e-19;

    /// Electron mass (kg)
    pub const ELECTRON_MASS: f64 = 9.109_383_7015e-31;

    /// Proton mass (kg)
    pub const PROTON_MASS: f64 = 1.672_621_923_69e-27;

    /// Gravitational constant (m³/(kg·s²))
    pub const GRAVITATIONAL_CONSTANT: f64 = 6.674_30e-11;

    /// Planck length (m)
    pub const PLANCK_LENGTH: f64 = 1.616_255e-35;

    /// Planck time (s)
    pub const PLANCK_TIME: f64 = 5.391_247e-44;

    /// Planck mass (kg)
    pub const PLANCK_MASS: f64 = 2.176_434e-8;

    /// Planck energy (J)
    pub const PLANCK_ENERGY: f64 = 1.956_1e9;

    /// Avogadro number
    pub const AVOGADRO: f64 = 6.022_140_76e23;

    /// eV to Joules conversion
    pub const EV_TO_JOULES: f64 = 1.602_176_634e-19;

    /// eV to Kelvin conversion (1 eV ≈ 11605 K)
    pub const EV_TO_KELVIN: f64 = 11_604.518;
}

// ============================================================================
// Unit Conversions
// ============================================================================

/// Unit conversion utilities
pub struct UnitConverter;

impl UnitConverter {
    /// Convert energy from eV to Joules
    pub fn ev_to_joules(ev: f64) -> f64 {
        ev * constants::EV_TO_JOULES
    }

    /// Convert energy from Joules to eV
    pub fn joules_to_ev(joules: f64) -> f64 {
        joules / constants::EV_TO_JOULES
    }

    /// Convert energy from keV to MeV
    pub fn kev_to_mev(kev: f64) -> f64 {
        kev / 1000.0
    }

    /// Convert energy from MeV to GeV
    pub fn mev_to_gev(mev: f64) -> f64 {
        mev / 1000.0
    }

    /// Convert temperature from eV to Kelvin
    pub fn ev_to_kelvin(ev: f64) -> f64 {
        ev * constants::EV_TO_KELVIN
    }

    /// Convert temperature from Kelvin to eV
    pub fn kelvin_to_ev(kelvin: f64) -> f64 {
        kelvin / constants::EV_TO_KELVIN
    }

    /// Convert temperature from keV to Kelvin
    pub fn kev_to_kelvin(kev: f64) -> f64 {
        kev * 1000.0 * constants::EV_TO_KELVIN
    }

    /// Convert temperature from Kelvin to keV
    pub fn kelvin_to_kev(kelvin: f64) -> f64 {
        kelvin / (1000.0 * constants::EV_TO_KELVIN)
    }

    /// Convert mass from GeV/c² to kg
    pub fn gev_to_kg(gev: f64) -> f64 {
        gev * 1e9 * constants::EV_TO_JOULES / (constants::SPEED_OF_LIGHT * constants::SPEED_OF_LIGHT)
    }

    /// Convert mass from kg to GeV/c²
    pub fn kg_to_gev(kg: f64) -> f64 {
        kg * constants::SPEED_OF_LIGHT * constants::SPEED_OF_LIGHT / (1e9 * constants::EV_TO_JOULES)
    }

    /// Convert magnetic field from Tesla to Gauss
    pub fn tesla_to_gauss(tesla: f64) -> f64 {
        tesla * 1e4
    }

    /// Convert magnetic field from Gauss to Tesla
    pub fn gauss_to_tesla(gauss: f64) -> f64 {
        gauss / 1e4
    }
}

// ============================================================================
// Fusion Physics Calculations
// ============================================================================

/// Fusion physics calculations
pub struct FusionPhysics;

impl FusionPhysics {
    /// Calculate the fusion triple product (n * T * tau)
    ///
    /// # Arguments
    /// * `density` - Plasma density in m^-3
    /// * `temperature` - Plasma temperature in keV
    /// * `confinement_time` - Energy confinement time in seconds
    ///
    /// # Returns
    /// Triple product in keV·s/m³
    pub fn triple_product(density: f64, temperature: f64, confinement_time: f64) -> f64 {
        density * temperature * confinement_time
    }

    /// Calculate Q-factor (power gain)
    ///
    /// # Arguments
    /// * `fusion_power` - Fusion power output in MW
    /// * `input_power` - Heating input power in MW
    pub fn q_factor(fusion_power: f64, input_power: f64) -> PhysicsResult<f64> {
        if input_power <= 0.0 {
            return Err(PhysicsError::Fusion("Input power must be positive".into()));
        }
        Ok(fusion_power / input_power)
    }

    /// Calculate plasma beta (ratio of plasma pressure to magnetic pressure)
    ///
    /// # Arguments
    /// * `density` - Plasma density in m^-3
    /// * `temperature` - Plasma temperature in keV
    /// * `magnetic_field` - Magnetic field in Tesla
    pub fn beta(density: f64, temperature: f64, magnetic_field: f64) -> PhysicsResult<f64> {
        if magnetic_field <= 0.0 {
            return Err(PhysicsError::Fusion("Magnetic field must be positive".into()));
        }
        // Beta = 2 * μ₀ * n * k * T / B²
        // With T in keV: Beta = 2 * μ₀ * n * T * 1000 * eV_to_J / B²
        let mu_0 = 4.0 * std::f64::consts::PI * 1e-7; // H/m
        let pressure = density * temperature * 1000.0 * constants::EV_TO_JOULES;
        let magnetic_pressure = magnetic_field.powi(2) / (2.0 * mu_0);
        Ok(pressure / magnetic_pressure)
    }

    /// Calculate aspect ratio
    pub fn aspect_ratio(major_radius: f64, minor_radius: f64) -> PhysicsResult<f64> {
        if minor_radius <= 0.0 {
            return Err(PhysicsError::Fusion("Minor radius must be positive".into()));
        }
        Ok(major_radius / minor_radius)
    }

    /// Calculate plasma volume for a tokamak (torus)
    pub fn tokamak_volume(major_radius: f64, minor_radius: f64) -> f64 {
        2.0 * std::f64::consts::PI.powi(2) * major_radius * minor_radius.powi(2)
    }

    /// Estimate Lawson criterion threshold for D-T fusion
    /// Returns minimum triple product required for ignition in keV·s/m³
    pub fn lawson_criterion_dt() -> f64 {
        // Approximately 3 × 10^21 keV·s/m³ for D-T at optimal temperature
        3.0e21
    }

    /// Check if plasma parameters meet ignition conditions
    pub fn is_ignition_possible(triple_product: f64) -> bool {
        triple_product >= Self::lawson_criterion_dt()
    }
}

// ============================================================================
// Particle Physics Calculations
// ============================================================================

/// Particle physics calculations
pub struct ParticlePhysics;

impl ParticlePhysics {
    /// Calculate invariant mass from four-momentum
    pub fn invariant_mass(e: f64, px: f64, py: f64, pz: f64) -> f64 {
        let m2 = e.powi(2) - px.powi(2) - py.powi(2) - pz.powi(2);
        if m2 >= 0.0 {
            m2.sqrt()
        } else {
            0.0
        }
    }

    /// Calculate transverse momentum
    pub fn transverse_momentum(px: f64, py: f64) -> f64 {
        (px.powi(2) + py.powi(2)).sqrt()
    }

    /// Calculate pseudorapidity
    pub fn pseudorapidity(px: f64, py: f64, pz: f64) -> f64 {
        let p = (px.powi(2) + py.powi(2) + pz.powi(2)).sqrt();
        if (p - pz).abs() < 1e-10 {
            return if pz >= 0.0 { f64::INFINITY } else { f64::NEG_INFINITY };
        }
        0.5 * ((p + pz) / (p - pz)).ln()
    }

    /// Calculate rapidity
    pub fn rapidity(e: f64, pz: f64) -> f64 {
        0.5 * ((e + pz) / (e - pz)).ln()
    }

    /// Calculate decay width from lifetime
    /// tau in seconds, returns width in GeV
    pub fn width_from_lifetime(lifetime: f64) -> f64 {
        constants::HBAR / (lifetime * constants::EV_TO_JOULES * 1e9)
    }

    /// Calculate lifetime from decay width
    /// width in GeV, returns lifetime in seconds
    pub fn lifetime_from_width(width: f64) -> f64 {
        constants::HBAR / (width * constants::EV_TO_JOULES * 1e9)
    }

    /// Calculate statistical significance
    pub fn significance(signal: f64, background: f64) -> PhysicsResult<f64> {
        if background < 0.0 {
            return Err(PhysicsError::ParticlePhysics("Background cannot be negative".into()));
        }
        if background == 0.0 {
            return Ok(f64::INFINITY);
        }
        Ok(signal / background.sqrt())
    }

    /// Check if result is a discovery (> 5 sigma)
    pub fn is_discovery(significance: f64) -> bool {
        significance >= 5.0
    }

    /// Check if result is evidence (> 3 sigma)
    pub fn is_evidence(significance: f64) -> bool {
        significance >= 3.0
    }
}

// ============================================================================
// Dark Matter Calculations
// ============================================================================

/// Dark matter physics calculations
pub struct DarkMatterPhysics;

impl DarkMatterPhysics {
    /// Calculate WIMP-nucleon scattering rate
    /// Returns events per kg per day
    pub fn wimp_rate(
        cross_section: f64,  // cm²
        dm_density: f64,     // GeV/cm³
        dm_mass: f64,        // GeV
        target_mass: f64,    // kg
        atomic_mass: f64,    // amu
    ) -> f64 {
        let n_targets = target_mass * constants::AVOGADRO / (atomic_mass * 1e-3);
        let flux = dm_density * 220e5 / dm_mass; // v ~ 220 km/s
        n_targets * cross_section * flux * 86400.0 // per day
    }

    /// Calculate nuclear recoil energy
    pub fn nuclear_recoil_energy(
        dm_mass: f64,      // GeV
        nucleus_mass: f64, // GeV
        dm_velocity: f64,  // m/s
        cos_theta: f64,    // scattering angle cosine
    ) -> f64 {
        let mu = dm_mass * nucleus_mass / (dm_mass + nucleus_mass); // reduced mass
        let e_dm = 0.5 * dm_mass * (dm_velocity / constants::SPEED_OF_LIGHT).powi(2);
        2.0 * mu.powi(2) * e_dm * (1.0 - cos_theta) / nucleus_mass
    }

    /// Calculate axion mass from frequency (for cavity searches)
    /// frequency in Hz, returns mass in eV
    pub fn axion_mass_from_frequency(frequency: f64) -> f64 {
        constants::PLANCK_CONSTANT * frequency / constants::EV_TO_JOULES
    }

    /// Calculate frequency from axion mass (for cavity searches)
    /// mass in eV, returns frequency in Hz
    pub fn frequency_from_axion_mass(mass: f64) -> f64 {
        mass * constants::EV_TO_JOULES / constants::PLANCK_CONSTANT
    }
}

// ============================================================================
// Quantum Gravity Calculations
// ============================================================================

/// Quantum gravity calculations
pub struct QuantumGravityPhysics;

impl QuantumGravityPhysics {
    /// Calculate Schwarzschild radius
    pub fn schwarzschild_radius(mass: f64) -> f64 {
        2.0 * constants::GRAVITATIONAL_CONSTANT * mass / constants::SPEED_OF_LIGHT.powi(2)
    }

    /// Calculate Bekenstein-Hawking entropy
    /// Returns entropy in units of k_B
    pub fn bekenstein_hawking_entropy(horizon_area: f64) -> f64 {
        horizon_area / (4.0 * constants::PLANCK_LENGTH.powi(2))
    }

    /// Calculate Hawking temperature
    /// mass in kg, returns temperature in Kelvin
    pub fn hawking_temperature(mass: f64) -> f64 {
        constants::HBAR * constants::SPEED_OF_LIGHT.powi(3)
            / (8.0 * std::f64::consts::PI * constants::GRAVITATIONAL_CONSTANT * mass * constants::BOLTZMANN)
    }

    /// Calculate black hole evaporation time
    /// mass in kg, returns time in seconds
    pub fn evaporation_time(mass: f64) -> f64 {
        let g = constants::GRAVITATIONAL_CONSTANT;
        let c = constants::SPEED_OF_LIGHT;
        let hbar = constants::HBAR;
        5120.0 * std::f64::consts::PI * g.powi(2) * mass.powi(3) / (hbar * c.powi(4))
    }

    /// LQG area spectrum minimum (area gap)
    /// Returns area in Planck units
    pub fn lqg_area_gap(immirzi_parameter: f64) -> f64 {
        4.0 * std::f64::consts::PI * immirzi_parameter * 3.0_f64.sqrt() * constants::PLANCK_LENGTH.powi(2)
    }

    /// Calculate Planck mass in kg
    pub fn planck_mass() -> f64 {
        (constants::HBAR * constants::SPEED_OF_LIGHT / constants::GRAVITATIONAL_CONSTANT).sqrt()
    }

    /// Calculate Planck energy in GeV
    pub fn planck_energy_gev() -> f64 {
        UnitConverter::kg_to_gev(Self::planck_mass()) * constants::SPEED_OF_LIGHT.powi(2) / 1e9
    }
}

// ============================================================================
// Time Crystal Calculations
// ============================================================================

/// Time crystal physics calculations
pub struct TimeCrystalPhysics;

impl TimeCrystalPhysics {
    /// Calculate coherence quality factor
    pub fn coherence_quality(coherence_time: f64, oscillation_period: f64) -> f64 {
        coherence_time / oscillation_period
    }

    /// Calculate subharmonic response period
    pub fn subharmonic_period(driving_period: f64, order: i32) -> f64 {
        driving_period * order as f64
    }

    /// Check if system shows time crystal behavior
    pub fn is_time_crystal(period_ratio: f64, coherence_cycles: i64) -> bool {
        // Time crystal if period is different from driving and coherent for multiple cycles
        (period_ratio - 1.0).abs() > 0.01 && coherence_cycles >= 10
    }
}

// ============================================================================
// Data Builders
// ============================================================================

/// Builder for FusionData
#[derive(Default)]
pub struct FusionDataBuilder {
    metadata: Option<Metadata>,
    plasma: Option<PlasmaParameters>,
    magnetic_configuration: Option<MagneticConfiguration>,
    energy_balance: Option<EnergyBalance>,
    heating_systems: Vec<HeatingSystem>,
    quality: QualityFlag,
}

impl FusionDataBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn metadata(mut self, metadata: Metadata) -> Self {
        self.metadata = Some(metadata);
        self
    }

    pub fn experiment(mut self, name: &str) -> Self {
        self.metadata = Some(Metadata::with_experiment(name));
        self
    }

    pub fn plasma(mut self, plasma: PlasmaParameters) -> Self {
        self.plasma = Some(plasma);
        self
    }

    pub fn plasma_simple(
        mut self,
        temperature: f64,
        temp_unit: &str,
        density: f64,
        density_unit: &str,
    ) -> Self {
        self.plasma = Some(PlasmaParameters {
            temperature: Measurement::new(temperature, temperature * 0.05, temp_unit),
            electron_temperature: None,
            ion_temperature: None,
            density: Measurement::new(density, density * 0.05, density_unit),
            electron_density: None,
            ion_density: None,
            confinement_time: None,
            triple_product: None,
            beta: None,
            fuel_composition: None,
        });
        self
    }

    pub fn magnetic_configuration(mut self, config: MagneticConfiguration) -> Self {
        self.magnetic_configuration = Some(config);
        self
    }

    pub fn tokamak(mut self, toroidal_field: f64, major_radius: f64, minor_radius: f64) -> Self {
        self.magnetic_configuration = Some(MagneticConfiguration {
            confinement_type: ConfinementType::Tokamak,
            toroidal_field: Some(Measurement::new(toroidal_field, 0.01, "T")),
            poloidal_field: None,
            plasma_current: None,
            major_radius: Some(Measurement::new(major_radius, 0.01, "m")),
            minor_radius: Some(Measurement::new(minor_radius, 0.01, "m")),
            aspect_ratio: Some(Measurement::new(major_radius / minor_radius, 0.1, "")),
            plasma_volume: Some(Measurement::new(
                FusionPhysics::tokamak_volume(major_radius, minor_radius),
                1.0,
                "m^3",
            )),
        });
        self
    }

    pub fn energy_balance(mut self, balance: EnergyBalance) -> Self {
        self.energy_balance = Some(balance);
        self
    }

    pub fn add_heating(mut self, system: HeatingSystem) -> Self {
        self.heating_systems.push(system);
        self
    }

    pub fn quality(mut self, quality: QualityFlag) -> Self {
        self.quality = quality;
        self
    }

    pub fn build(self) -> PhysicsResult<FusionData> {
        let metadata = self.metadata.unwrap_or_default();
        let plasma = self
            .plasma
            .ok_or_else(|| PhysicsError::validation("Plasma parameters are required"))?;

        Ok(FusionData {
            metadata,
            plasma,
            magnetic_configuration: self.magnetic_configuration,
            energy_balance: self.energy_balance,
            heating_systems: if self.heating_systems.is_empty() {
                None
            } else {
                Some(self.heating_systems)
            },
            quality: self.quality,
        })
    }
}

/// Builder for ParticleData
#[derive(Default)]
pub struct ParticleDataBuilder {
    metadata: Option<Metadata>,
    particle: Option<ParticleProperties>,
    event: Option<CollisionEvent>,
    cross_section: Option<CrossSectionData>,
    quality: QualityFlag,
}

impl ParticleDataBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn metadata(mut self, metadata: Metadata) -> Self {
        self.metadata = Some(metadata);
        self
    }

    pub fn experiment(mut self, name: &str) -> Self {
        self.metadata = Some(Metadata::with_experiment(name));
        self
    }

    pub fn particle(mut self, particle: ParticleProperties) -> Self {
        self.particle = Some(particle);
        self
    }

    pub fn event(mut self, event: CollisionEvent) -> Self {
        self.event = Some(event);
        self
    }

    pub fn cross_section(mut self, data: CrossSectionData) -> Self {
        self.cross_section = Some(data);
        self
    }

    pub fn quality(mut self, quality: QualityFlag) -> Self {
        self.quality = quality;
        self
    }

    pub fn build(self) -> PhysicsResult<ParticleData> {
        Ok(ParticleData {
            metadata: self.metadata.unwrap_or_default(),
            particle: self.particle,
            event: self.event,
            cross_section: self.cross_section,
            quality: self.quality,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unit_conversion() {
        let ev = 1.0;
        let joules = UnitConverter::ev_to_joules(ev);
        assert!((joules - constants::EV_TO_JOULES).abs() < 1e-30);

        let kelvin = UnitConverter::ev_to_kelvin(1.0);
        assert!((kelvin - constants::EV_TO_KELVIN).abs() < 1.0);
    }

    #[test]
    fn test_fusion_q_factor() {
        let q = FusionPhysics::q_factor(500.0, 50.0).unwrap();
        assert!((q - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_fusion_triple_product() {
        let tp = FusionPhysics::triple_product(1e20, 15.0, 3.0);
        assert!((tp - 4.5e21).abs() < 1e20);
    }

    #[test]
    fn test_particle_invariant_mass() {
        // Light-like particle (photon)
        let mass = ParticlePhysics::invariant_mass(100.0, 100.0, 0.0, 0.0);
        assert!(mass.abs() < 1e-10);

        // Massive particle at rest
        let mass = ParticlePhysics::invariant_mass(125.0, 0.0, 0.0, 0.0);
        assert!((mass - 125.0).abs() < 1e-10);
    }

    #[test]
    fn test_significance() {
        let sig = ParticlePhysics::significance(100.0, 25.0).unwrap();
        assert!((sig - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_schwarzschild_radius() {
        // Sun's Schwarzschild radius ≈ 3 km
        let r_s = QuantumGravityPhysics::schwarzschild_radius(1.989e30);
        assert!((r_s - 2953.0).abs() < 10.0);
    }

    #[test]
    fn test_hawking_temperature() {
        // Solar mass black hole should have very low temperature
        let t = QuantumGravityPhysics::hawking_temperature(1.989e30);
        assert!(t < 1e-6); // Much less than 1 microkelvin
    }

    #[test]
    fn test_fusion_builder() {
        let data = FusionDataBuilder::new()
            .experiment("ITER")
            .plasma_simple(150e6, "K", 1e20, "m^-3")
            .tokamak(5.3, 6.2, 2.0)
            .quality(QualityFlag::Simulated)
            .build()
            .unwrap();

        assert_eq!(data.plasma.temperature.value, 150e6);
        assert!(data.magnetic_configuration.is_some());
    }
}
