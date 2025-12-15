//! Physics simulators and data generators
//!
//! This module provides simulation capabilities for testing and demonstration.

use async_trait::async_trait;
use chrono::Utc;

use crate::core::*;
use crate::error::{PhysicsError, PhysicsResult};
use crate::types::*;

// ============================================================================
// Simulator Trait
// ============================================================================

/// Trait for physics simulators
#[async_trait]
pub trait PhysicsSimulator: Send + Sync {
    /// Run a simulation step
    async fn step(&mut self) -> PhysicsResult<()>;

    /// Get current simulation time
    fn current_time(&self) -> f64;

    /// Reset the simulator
    fn reset(&mut self);
}

// ============================================================================
// Fusion Simulator
// ============================================================================

/// Configuration for fusion simulation
#[derive(Debug, Clone)]
pub struct FusionSimConfig {
    /// Initial plasma temperature (keV)
    pub initial_temperature: f64,
    /// Initial plasma density (m^-3)
    pub initial_density: f64,
    /// Heating power (MW)
    pub heating_power: f64,
    /// Magnetic field (T)
    pub magnetic_field: f64,
    /// Major radius (m)
    pub major_radius: f64,
    /// Minor radius (m)
    pub minor_radius: f64,
    /// Time step (s)
    pub dt: f64,
    /// Random fluctuation amplitude (fraction)
    pub noise_level: f64,
}

impl Default for FusionSimConfig {
    fn default() -> Self {
        Self {
            initial_temperature: 10.0, // keV
            initial_density: 1e20,     // m^-3
            heating_power: 50.0,       // MW
            magnetic_field: 5.3,       // T
            major_radius: 6.2,         // m
            minor_radius: 2.0,         // m
            dt: 0.001,                 // 1 ms
            noise_level: 0.02,         // 2%
        }
    }
}

/// Fusion plasma simulator
pub struct FusionSimulator {
    config: FusionSimConfig,
    temperature: f64,
    density: f64,
    confinement_time: f64,
    time: f64,
    step_count: u64,
}

impl FusionSimulator {
    /// Create a new fusion simulator
    pub fn new(config: FusionSimConfig) -> Self {
        let initial_temperature = config.initial_temperature;
        let initial_density = config.initial_density;
        Self {
            config,
            temperature: initial_temperature,
            density: initial_density,
            confinement_time: 1.0,
            time: 0.0,
            step_count: 0,
        }
    }

    /// Create simulator with default ITER-like parameters
    pub fn iter_like() -> Self {
        Self::new(FusionSimConfig {
            initial_temperature: 15.0,
            initial_density: 1e20,
            heating_power: 50.0,
            magnetic_field: 5.3,
            major_radius: 6.2,
            minor_radius: 2.0,
            dt: 0.001,
            noise_level: 0.02,
        })
    }

    /// Get current plasma state as FusionData
    pub fn get_state(&self) -> PhysicsResult<FusionData> {
        let temp_kelvin = UnitConverter::kev_to_kelvin(self.temperature);
        let triple_product =
            FusionPhysics::triple_product(self.density, self.temperature, self.confinement_time);

        // Estimate fusion power (simplified D-T model)
        let fusion_power = self.estimate_fusion_power();
        let q_factor = if self.config.heating_power > 0.0 {
            fusion_power / self.config.heating_power
        } else {
            0.0
        };

        FusionDataBuilder::new()
            .experiment("FusionSimulator")
            .plasma(PlasmaParameters {
                temperature: Measurement::new(
                    temp_kelvin,
                    temp_kelvin * self.config.noise_level,
                    "K",
                ),
                electron_temperature: Some(Measurement::new(
                    temp_kelvin,
                    temp_kelvin * self.config.noise_level,
                    "K",
                )),
                ion_temperature: Some(Measurement::new(
                    temp_kelvin * 0.95,
                    temp_kelvin * self.config.noise_level,
                    "K",
                )),
                density: Measurement::new(
                    self.density,
                    self.density * self.config.noise_level,
                    "m^-3",
                ),
                electron_density: Some(Measurement::new(
                    self.density,
                    self.density * self.config.noise_level,
                    "m^-3",
                )),
                ion_density: Some(Measurement::new(
                    self.density * 0.5,
                    self.density * 0.5 * self.config.noise_level,
                    "m^-3",
                )),
                confinement_time: Some(Measurement::new(
                    self.confinement_time,
                    self.confinement_time * 0.1,
                    "s",
                )),
                triple_product: Some(Measurement::new(
                    triple_product,
                    triple_product * 0.15,
                    "keV·s/m³",
                )),
                beta: Some(Measurement::new(
                    FusionPhysics::beta(self.density, self.temperature, self.config.magnetic_field)
                        .unwrap_or(0.0),
                    0.001,
                    "",
                )),
                fuel_composition: Some(FuelComposition {
                    fuel_type: FuelType::DT,
                    deuterium_fraction: Some(0.5),
                    tritium_fraction: Some(0.5),
                    helium3_fraction: None,
                    impurity_fraction: Some(0.02),
                    z_effective: Some(Measurement::new(1.5, 0.1, "")),
                }),
            })
            .tokamak(
                self.config.magnetic_field,
                self.config.major_radius,
                self.config.minor_radius,
            )
            .energy_balance(EnergyBalance {
                input_power: Some(Measurement::new(self.config.heating_power, 1.0, "MW")),
                fusion_power: Some(Measurement::new(fusion_power, fusion_power * 0.1, "MW")),
                q_factor: Some(Measurement::new(q_factor, q_factor * 0.1, "")),
                neutron_power: Some(Measurement::new(fusion_power * 0.8, fusion_power * 0.08, "MW")),
                alpha_heating_power: Some(Measurement::new(
                    fusion_power * 0.2,
                    fusion_power * 0.02,
                    "MW",
                )),
                radiation_loss: Some(Measurement::new(5.0, 1.0, "MW")),
                stored_energy: Some(Measurement::new(300.0, 30.0, "MJ")),
            })
            .quality(QualityFlag::Simulated)
            .build()
    }

    /// Estimate fusion power (simplified model)
    fn estimate_fusion_power(&self) -> f64 {
        // Simplified D-T fusion power estimate
        // P_fusion ∝ n² * <σv> * V
        // Using approximate reactivity for T ~ 10-20 keV
        let reactivity = 1e-22 * self.temperature.powi(2); // Very simplified
        let volume = FusionPhysics::tokamak_volume(
            self.config.major_radius,
            self.config.minor_radius,
        );
        let energy_per_reaction = 17.6e6 * constants::EV_TO_JOULES; // MeV to J

        let power = 0.25 * self.density.powi(2) * reactivity * volume * energy_per_reaction;
        power / 1e6 // Convert to MW
    }

    /// Update simulation state
    fn update(&mut self) {
        // Simplified plasma evolution model
        let dt = self.config.dt;

        // Temperature evolution with heating and losses
        let heating_rate = self.config.heating_power * 1e6 / (self.density *
            FusionPhysics::tokamak_volume(self.config.major_radius, self.config.minor_radius) *
            1.5 * constants::BOLTZMANN);

        let loss_rate = self.temperature / self.confinement_time;

        self.temperature += (heating_rate - loss_rate) * dt / constants::EV_TO_KELVIN / 1000.0;

        // Confinement time evolution (H-mode scaling approximation)
        self.confinement_time = 0.1 * self.config.magnetic_field.powf(0.9) *
            self.density.powf(0.1) * self.config.heating_power.powf(-0.5);

        // Add noise
        let noise = (self.step_count as f64 * 1.23456).sin() * self.config.noise_level;
        self.temperature *= 1.0 + noise;
        self.density *= 1.0 + noise * 0.5;

        // Ensure positive values
        self.temperature = self.temperature.max(0.1);
        self.density = self.density.max(1e18);

        self.time += dt;
        self.step_count += 1;
    }
}

#[async_trait]
impl PhysicsSimulator for FusionSimulator {
    async fn step(&mut self) -> PhysicsResult<()> {
        self.update();
        Ok(())
    }

    fn current_time(&self) -> f64 {
        self.time
    }

    fn reset(&mut self) {
        self.temperature = self.config.initial_temperature;
        self.density = self.config.initial_density;
        self.confinement_time = 1.0;
        self.time = 0.0;
        self.step_count = 0;
    }
}

// ============================================================================
// Particle Event Generator
// ============================================================================

/// Configuration for particle event generation
#[derive(Debug, Clone)]
pub struct EventGenConfig {
    /// Center-of-mass energy (GeV)
    pub sqrt_s: f64,
    /// Beam 1 particle type
    pub beam1: String,
    /// Beam 2 particle type
    pub beam2: String,
    /// Average number of jets
    pub avg_jets: f64,
    /// Average MET (GeV)
    pub avg_met: f64,
}

impl Default for EventGenConfig {
    fn default() -> Self {
        Self {
            sqrt_s: 13600.0, // 13.6 TeV
            beam1: "proton".to_string(),
            beam2: "proton".to_string(),
            avg_jets: 4.0,
            avg_met: 30.0,
        }
    }
}

/// Particle collision event generator
pub struct EventGenerator {
    config: EventGenConfig,
    event_count: u64,
    run_number: i64,
}

impl EventGenerator {
    /// Create a new event generator
    pub fn new(config: EventGenConfig) -> Self {
        Self {
            config,
            event_count: 0,
            run_number: 1,
        }
    }

    /// Create LHC Run 3 style generator
    pub fn lhc_run3() -> Self {
        Self::new(EventGenConfig::default())
    }

    /// Generate a collision event
    pub fn generate_event(&mut self) -> CollisionEvent {
        self.event_count += 1;

        // Generate pseudo-random values based on event count
        let seed = self.event_count as f64;

        // Generate jets
        let n_jets = ((seed * 0.7).sin().abs() * 6.0) as usize + 1;
        let jets: Vec<FourMomentum> = (0..n_jets)
            .map(|i| {
                let pt = 30.0 + (seed * (i as f64 + 1.0) * 0.3).sin().abs() * 200.0;
                let eta = (seed * (i as f64 + 2.0) * 0.5).sin() * 2.5;
                let phi = (seed * (i as f64 + 3.0) * 0.7).sin() * std::f64::consts::PI;
                let mass = 5.0 + (seed * (i as f64 + 4.0)).sin().abs() * 50.0;

                let e = (pt * pt + mass * mass).sqrt() * eta.cosh();
                let px = pt * phi.cos();
                let py = pt * phi.sin();
                let pz = pt * eta.sinh();

                FourMomentum {
                    e: Measurement::new(e, e * 0.03, "GeV"),
                    px: Measurement::new(px, pt * 0.02, "GeV"),
                    py: Measurement::new(py, pt * 0.02, "GeV"),
                    pz: Measurement::new(pz, pt * eta.cosh() * 0.02, "GeV"),
                    pt: Some(Measurement::new(pt, pt * 0.02, "GeV")),
                    eta: Some(Measurement::new(eta, 0.01, "")),
                    phi: Some(Measurement::new(phi, 0.01, "rad")),
                    mass: Some(Measurement::new(mass, mass * 0.1, "GeV")),
                }
            })
            .collect();

        // Generate MET
        let met_value = self.config.avg_met * (1.0 + (seed * 0.9).sin().abs());
        let met_phi = (seed * 1.1).sin() * std::f64::consts::PI;

        CollisionEvent {
            event_id: format!("sim-run{}-evt{}", self.run_number, self.event_count),
            run_number: Some(self.run_number),
            event_number: Some(self.event_count as i64),
            timestamp: Some(Utc::now()),
            collision: CollisionInfo {
                sqrt_s: Measurement::new(self.config.sqrt_s, 1.0, "GeV"),
                beam1: self.config.beam1.clone(),
                beam2: self.config.beam2.clone(),
                collision_type: Some("pp".to_string()),
            },
            jets,
            electrons: vec![],
            muons: vec![],
            photons: vec![],
            missing_et: Some(MissingET {
                met: Measurement::new(met_value, met_value * 0.1, "GeV"),
                phi: Measurement::new(met_phi, 0.1, "rad"),
                sum_et: Some(Measurement::new(met_value * 10.0, met_value, "GeV")),
            }),
            trigger: vec!["HLT_Jet400".to_string(), "HLT_MET100".to_string()],
        }
    }

    /// Generate particle data with event
    pub fn generate_particle_data(&mut self) -> PhysicsResult<ParticleData> {
        let event = self.generate_event();

        ParticleDataBuilder::new()
            .experiment("EventGenerator")
            .event(event)
            .quality(QualityFlag::Simulated)
            .build()
    }

    /// Get event count
    pub fn event_count(&self) -> u64 {
        self.event_count
    }

    /// Start new run
    pub fn new_run(&mut self) {
        self.run_number += 1;
        self.event_count = 0;
    }
}

// ============================================================================
// Dark Matter Signal Generator
// ============================================================================

/// Dark matter signal generator
pub struct DarkMatterGenerator {
    /// Dark matter mass (GeV)
    pub dm_mass: f64,
    /// Cross-section (cm²)
    pub cross_section: f64,
    /// Local DM density (GeV/cm³)
    pub local_density: f64,
    event_count: u64,
}

impl DarkMatterGenerator {
    pub fn new(dm_mass: f64, cross_section: f64) -> Self {
        Self {
            dm_mass,
            cross_section,
            local_density: 0.3, // Standard halo model
            event_count: 0,
        }
    }

    /// Generate a detection event
    pub fn generate_event(&mut self) -> DetectionEvent {
        self.event_count += 1;

        let seed = self.event_count as f64;
        let recoil_energy = (seed * 0.7).sin().abs() * 50.0 + 1.0; // 1-50 keV

        DetectionEvent {
            event_id: format!("dm-evt-{}", self.event_count),
            timestamp: Utc::now(),
            recoil_energy: Measurement::new(recoil_energy, recoil_energy * 0.1, "keV"),
            signal_type: SignalType::NuclearRecoil,
            position: Some(Vector3D {
                x: Measurement::new((seed * 1.1).sin() * 0.5, 0.01, "m"),
                y: Measurement::new((seed * 1.3).sin() * 0.5, 0.01, "m"),
                z: Measurement::new((seed * 1.7).sin() * 0.3, 0.01, "m"),
                coordinate_system: "cartesian".to_string(),
            }),
            fiducial: Some(true),
            passed_cuts: Some(true),
        }
    }

    /// Generate dark matter data
    pub fn generate_data(&mut self) -> PhysicsResult<DarkMatterData> {
        let event = self.generate_event();

        Ok(DarkMatterData {
            metadata: Metadata::with_experiment("DarkMatterGenerator"),
            detection_event: Some(event),
            exclusion_limit: None,
            axion_search: None,
            quality: QualityFlag::Simulated,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_fusion_simulator() {
        let mut sim = FusionSimulator::iter_like();

        // Run several steps
        for _ in 0..100 {
            sim.step().await.unwrap();
        }

        let state = sim.get_state().unwrap();
        assert!(state.plasma.temperature.value > 0.0);
        assert!(state.plasma.density.value > 0.0);
    }

    #[test]
    fn test_event_generator() {
        let mut gen = EventGenerator::lhc_run3();

        let event = gen.generate_event();
        assert!(!event.jets.is_empty());
        assert_eq!(event.collision.sqrt_s.value, 13600.0);
    }

    #[test]
    fn test_dark_matter_generator() {
        let mut gen = DarkMatterGenerator::new(100.0, 1e-45);

        let data = gen.generate_data().unwrap();
        assert!(data.detection_event.is_some());
        assert_eq!(data.quality, QualityFlag::Simulated);
    }
}
