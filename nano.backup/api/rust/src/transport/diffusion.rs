//! Diffusion-based transport for molecular communication
//!
//! Implements Fick's laws of diffusion for nanoscale molecular communication.

use crate::error::{NanoError, NanoResult};
use crate::types::Position3D;
use crate::transport::{Transport, TransportMethod, TransportResult};
use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Boltzmann constant (J/K)
const BOLTZMANN_CONSTANT: f64 = 1.380649e-23;

/// Configuration for diffusion-based transport
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiffusionConfig {
    /// Carrier molecule name
    pub carrier_molecule: String,
    /// Diffusion coefficient in m²/s
    pub diffusion_coefficient_m2_per_s: f64,
    /// Medium viscosity in Pa·s
    pub medium_viscosity_pa_s: f64,
    /// Temperature in Kelvin
    pub temperature_k: f64,
    /// Signal detection threshold (minimum concentration ratio)
    pub detection_threshold: f64,
    /// Maximum communication range in nanometers
    pub max_range_nm: f64,
}

impl Default for DiffusionConfig {
    fn default() -> Self {
        Self {
            carrier_molecule: "AHL".to_string(),
            diffusion_coefficient_m2_per_s: 4.9e-10, // AHL in water
            medium_viscosity_pa_s: 0.001,            // Water at 37°C
            temperature_k: 310.0,                     // Body temperature
            detection_threshold: 0.01,                // 1% of original signal
            max_range_nm: 10000.0,                   // 10 μm
        }
    }
}

impl DiffusionConfig {
    /// Create configuration for blood plasma medium
    pub fn blood_plasma() -> Self {
        Self {
            carrier_molecule: "signaling_peptide".to_string(),
            diffusion_coefficient_m2_per_s: 1e-10,
            medium_viscosity_pa_s: 0.0012, // Blood plasma
            temperature_k: 310.0,
            detection_threshold: 0.01,
            max_range_nm: 5000.0,
        }
    }

    /// Create configuration for cytoplasm medium
    pub fn cytoplasm() -> Self {
        Self {
            carrier_molecule: "ATP".to_string(),
            diffusion_coefficient_m2_per_s: 3e-10,
            medium_viscosity_pa_s: 0.002, // Cytoplasm is more viscous
            temperature_k: 310.0,
            detection_threshold: 0.01,
            max_range_nm: 1000.0,
        }
    }
}

/// Diffusion-based transport implementation
pub struct DiffusionTransport {
    config: DiffusionConfig,
}

impl DiffusionTransport {
    /// Create a new diffusion transport
    pub fn new(config: DiffusionConfig) -> Self {
        Self { config }
    }

    /// Get the configuration
    pub fn config(&self) -> &DiffusionConfig {
        &self.config
    }

    /// Calculate concentration at a distance using 3D Gaussian diffusion
    ///
    /// C(r, t) = N / (4πDt)^(3/2) * exp(-r² / 4Dt)
    ///
    /// Returns concentration as a fraction of initial concentration
    pub fn calculate_concentration(&self, distance_nm: f64, time_s: f64) -> f64 {
        if time_s <= 0.0 {
            return 0.0;
        }

        let d = self.config.diffusion_coefficient_m2_per_s;
        let r_m = distance_nm * 1e-9; // Convert nm to m
        let r_squared = r_m * r_m;

        let denominator = (4.0 * PI * d * time_s).powf(1.5);
        let exponent = -r_squared / (4.0 * d * time_s);

        // Returns normalized concentration (assuming unit initial amount)
        (1.0 / denominator) * exponent.exp()
    }

    /// Calculate the time for signal to reach a certain concentration at distance
    /// Using mean square displacement: <r²> = 6Dt for 3D
    pub fn calculate_diffusion_time(&self, distance_nm: f64) -> f64 {
        let d = self.config.diffusion_coefficient_m2_per_s;
        let r_m = distance_nm * 1e-9;

        // For 3D diffusion, characteristic time when probability peaks at distance r
        // This is when ∂C/∂t = 0, giving t = r²/(6D)
        (r_m * r_m) / (6.0 * d)
    }

    /// Calculate Fick's first law flux
    /// J = -D * (dC/dx)
    pub fn calculate_flux(&self, concentration_gradient_per_m: f64) -> f64 {
        -self.config.diffusion_coefficient_m2_per_s * concentration_gradient_per_m
    }

    /// Calculate Stokes-Einstein diffusion coefficient
    /// D = kT / (6πηr)
    pub fn stokes_einstein_coefficient(
        temperature_k: f64,
        viscosity_pa_s: f64,
        particle_radius_nm: f64,
    ) -> f64 {
        let r_m = particle_radius_nm * 1e-9;
        BOLTZMANN_CONSTANT * temperature_k / (6.0 * PI * viscosity_pa_s * r_m)
    }

    /// Signal reception probability at given distance and time
    pub fn signal_reception_probability(
        &self,
        distance_nm: f64,
        time_s: f64,
        transmitted_molecules: usize,
        threshold_molecules: usize,
    ) -> f64 {
        let concentration = self.calculate_concentration(distance_nm, time_s);
        let received = concentration * transmitted_molecules as f64;

        if received >= threshold_molecules as f64 {
            1.0
        } else {
            (received / threshold_molecules as f64).min(1.0)
        }
    }
}

#[async_trait]
impl Transport for DiffusionTransport {
    fn method(&self) -> TransportMethod {
        TransportMethod::Diffusion
    }

    fn estimate_delivery_time(
        &self,
        source: &Position3D,
        destination: &Position3D,
    ) -> NanoResult<f64> {
        let distance = source.distance_to(destination);

        if distance > self.config.max_range_nm {
            return Err(NanoError::TransportFailed(format!(
                "Distance {} nm exceeds maximum range {} nm",
                distance, self.config.max_range_nm
            )));
        }

        Ok(self.calculate_diffusion_time(distance))
    }

    fn estimate_signal_strength(
        &self,
        source: &Position3D,
        destination: &Position3D,
        time_s: f64,
    ) -> NanoResult<f64> {
        let distance = source.distance_to(destination);

        if distance > self.config.max_range_nm {
            return Ok(0.0);
        }

        // Normalize concentration to a 0-1 scale
        let concentration = self.calculate_concentration(distance, time_s);

        // Convert to signal strength (log scale would be more realistic but we simplify)
        let reference_concentration = self.calculate_concentration(1.0, time_s);
        let signal_strength = (concentration / reference_concentration).min(1.0);

        Ok(signal_strength)
    }

    async fn send(
        &self,
        source: &Position3D,
        destination: &Position3D,
        _payload_size: usize,
    ) -> NanoResult<TransportResult> {
        let distance = source.distance_to(destination);

        if distance > self.config.max_range_nm {
            return Ok(TransportResult::failure(
                NanoError::TransportFailed(format!(
                    "Distance {} nm exceeds maximum range {} nm",
                    distance, self.config.max_range_nm
                ))
            ));
        }

        let delivery_time = self.calculate_diffusion_time(distance);
        let signal_strength = self.estimate_signal_strength(source, destination, delivery_time)?;

        if signal_strength < self.config.detection_threshold {
            return Ok(TransportResult::failure(
                NanoError::TransportFailed(format!(
                    "Signal strength {} below detection threshold {}",
                    signal_strength, self.config.detection_threshold
                ))
            ));
        }

        Ok(TransportResult::success(delivery_time, signal_strength))
    }

    fn is_reachable(&self, source: &Position3D, destination: &Position3D) -> bool {
        let distance = source.distance_to(destination);
        distance <= self.config.max_range_nm
    }
}

/// Simulator for diffusion processes
pub struct DiffusionSimulator {
    config: DiffusionConfig,
    time_step_s: f64,
    current_time_s: f64,
}

impl DiffusionSimulator {
    /// Create a new simulator
    pub fn new(config: DiffusionConfig, time_step_s: f64) -> Self {
        Self {
            config,
            time_step_s,
            current_time_s: 0.0,
        }
    }

    /// Step the simulation forward
    pub fn step(&mut self) {
        self.current_time_s += self.time_step_s;
    }

    /// Get current simulation time
    pub fn current_time(&self) -> f64 {
        self.current_time_s
    }

    /// Reset simulation
    pub fn reset(&mut self) {
        self.current_time_s = 0.0;
    }

    /// Calculate concentration field at current time
    pub fn concentration_at(&self, source: &Position3D, target: &Position3D) -> f64 {
        let transport = DiffusionTransport::new(self.config.clone());
        let distance = source.distance_to(target);
        transport.calculate_concentration(distance, self.current_time_s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_diffusion_time() {
        let config = DiffusionConfig::default();
        let transport = DiffusionTransport::new(config);

        // Test diffusion time increases with distance squared
        let time_100nm = transport.calculate_diffusion_time(100.0);
        let time_200nm = transport.calculate_diffusion_time(200.0);

        // Time should be roughly 4x for 2x distance (quadratic relationship)
        assert!((time_200nm / time_100nm - 4.0).abs() < 0.1);
    }

    #[test]
    fn test_concentration_decreases_with_distance() {
        let config = DiffusionConfig::default();
        let transport = DiffusionTransport::new(config);

        let time = 0.001; // 1 ms
        let conc_100nm = transport.calculate_concentration(100.0, time);
        let conc_200nm = transport.calculate_concentration(200.0, time);

        assert!(conc_100nm > conc_200nm);
    }

    #[test]
    fn test_stokes_einstein() {
        // Test with known values for water at 25°C
        let d = DiffusionTransport::stokes_einstein_coefficient(
            298.15, // 25°C
            0.001,  // Water viscosity
            1.0,    // 1 nm radius
        );

        // Should be on the order of 10^-10 m²/s
        assert!(d > 1e-11 && d < 1e-9);
    }

    #[tokio::test]
    async fn test_send_within_range() {
        let config = DiffusionConfig::default();
        let transport = DiffusionTransport::new(config);

        let source = Position3D::new(0.0, 0.0, 0.0);
        let dest = Position3D::new(100.0, 0.0, 0.0);

        let result = transport.send(&source, &dest, 100).await.unwrap();
        assert!(result.success);
    }

    #[tokio::test]
    async fn test_send_out_of_range() {
        let config = DiffusionConfig::default();
        let transport = DiffusionTransport::new(config);

        let source = Position3D::new(0.0, 0.0, 0.0);
        let dest = Position3D::new(100000.0, 0.0, 0.0); // 100 μm - out of range

        let result = transport.send(&source, &dest, 100).await.unwrap();
        assert!(!result.success);
    }
}
