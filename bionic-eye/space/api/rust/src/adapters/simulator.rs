//! Simulation adapters
//!
//! Provides traits and implementations for running space technology simulations.

use async_trait::async_trait;
use crate::error::SpaceResult;

/// Trait for simulation engines
#[async_trait]
pub trait Simulator: Send + Sync {
    /// Simulator name
    fn name(&self) -> &str;

    /// Run simulation
    async fn run(&self) -> SpaceResult<SimulationResult>;

    /// Get progress (0.0 - 1.0)
    fn progress(&self) -> f64;

    /// Cancel simulation
    async fn cancel(&self) -> SpaceResult<()>;
}

/// Simulation result
#[derive(Debug, Clone)]
pub struct SimulationResult {
    /// Success status
    pub success: bool,
    /// Simulation time in seconds
    pub elapsed_seconds: f64,
    /// Number of iterations
    pub iterations: u64,
    /// Output data as JSON
    pub output: serde_json::Value,
}

/// Orbital mechanics simulator
pub struct OrbitalSimulator {
    name: String,
    time_step_seconds: f64,
    total_time_seconds: f64,
}

impl OrbitalSimulator {
    /// Create a new orbital simulator
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            time_step_seconds: 1.0,
            total_time_seconds: 86400.0, // 1 day default
        }
    }

    /// Set time step
    pub fn with_time_step(mut self, seconds: f64) -> Self {
        self.time_step_seconds = seconds;
        self
    }

    /// Set total simulation time
    pub fn with_duration(mut self, seconds: f64) -> Self {
        self.total_time_seconds = seconds;
        self
    }

    /// Calculate transfer orbit delta-v (Hohmann transfer)
    pub fn hohmann_delta_v(r1_au: f64, r2_au: f64) -> f64 {
        // Simplified Hohmann transfer calculation
        // Returns delta-v in km/s
        let mu = 1.327e11; // km^3/s^2 (Sun's gravitational parameter)
        let r1 = r1_au * 1.496e8; // Convert AU to km
        let r2 = r2_au * 1.496e8;

        let a_transfer = (r1 + r2) / 2.0;

        let v1 = (mu / r1).sqrt();
        let v_transfer_at_r1 = (mu * (2.0 / r1 - 1.0 / a_transfer)).sqrt();
        let delta_v1 = (v_transfer_at_r1 - v1).abs();

        let v2 = (mu / r2).sqrt();
        let v_transfer_at_r2 = (mu * (2.0 / r2 - 1.0 / a_transfer)).sqrt();
        let delta_v2 = (v2 - v_transfer_at_r2).abs();

        delta_v1 + delta_v2
    }

    /// Calculate transfer time (Hohmann)
    pub fn hohmann_transfer_time(r1_au: f64, r2_au: f64) -> f64 {
        // Returns transfer time in days
        let a_transfer = (r1_au + r2_au) / 2.0;
        let period_years = a_transfer.powf(1.5); // Kepler's 3rd law
        period_years * 365.25 / 2.0 // Half the orbital period
    }
}

#[async_trait]
impl Simulator for OrbitalSimulator {
    fn name(&self) -> &str {
        &self.name
    }

    async fn run(&self) -> SpaceResult<SimulationResult> {
        let iterations = (self.total_time_seconds / self.time_step_seconds) as u64;

        Ok(SimulationResult {
            success: true,
            elapsed_seconds: self.total_time_seconds,
            iterations,
            output: serde_json::json!({
                "simulator": self.name,
                "time_step": self.time_step_seconds,
                "total_time": self.total_time_seconds,
                "iterations": iterations
            }),
        })
    }

    fn progress(&self) -> f64 {
        1.0 // Simplified - always complete after run
    }

    async fn cancel(&self) -> SpaceResult<()> {
        Ok(())
    }
}

/// Interstellar trajectory calculator
pub struct InterstellarCalculator;

impl InterstellarCalculator {
    /// Calculate travel time to a star system
    pub fn travel_time_years(distance_ly: f64, velocity_c: f64) -> f64 {
        if velocity_c <= 0.0 {
            return f64::INFINITY;
        }
        distance_ly / velocity_c
    }

    /// Calculate relativistic time dilation factor (Lorentz factor)
    pub fn lorentz_factor(velocity_c: f64) -> f64 {
        if velocity_c >= 1.0 {
            return f64::INFINITY;
        }
        1.0 / (1.0 - velocity_c * velocity_c).sqrt()
    }

    /// Calculate ship time (experienced by crew)
    pub fn ship_time_years(distance_ly: f64, velocity_c: f64) -> f64 {
        let earth_time = Self::travel_time_years(distance_ly, velocity_c);
        let gamma = Self::lorentz_factor(velocity_c);
        earth_time / gamma
    }

    /// Calculate kinetic energy for relativistic spacecraft
    pub fn kinetic_energy_joules(mass_kg: f64, velocity_c: f64) -> f64 {
        let c = 299792458.0; // m/s
        let gamma = Self::lorentz_factor(velocity_c);
        mass_kg * c * c * (gamma - 1.0)
    }

    /// Calculate acceleration time for lightsail
    pub fn lightsail_acceleration_time(
        sail_mass_kg: f64,
        _sail_area_m2: f64,
        laser_power_w: f64,
        target_velocity_c: f64,
    ) -> f64 {
        // Simplified calculation
        let c = 299792458.0;
        let target_velocity = target_velocity_c * c;

        // Radiation pressure force: F = 2P/c (perfect reflection)
        let force = 2.0 * laser_power_w / c;
        let acceleration = force / sail_mass_kg;

        // Time to reach velocity: t = v/a
        target_velocity / acceleration
    }
}

/// Dyson sphere energy calculator
pub struct DysonCalculator;

impl DysonCalculator {
    /// Solar luminosity in watts
    pub const SOLAR_LUMINOSITY: f64 = 3.828e26;

    /// Calculate total power available from a star
    pub fn stellar_power(luminosity_solar: f64) -> f64 {
        luminosity_solar * Self::SOLAR_LUMINOSITY
    }

    /// Calculate harvestable power
    pub fn harvestable_power(
        luminosity_solar: f64,
        coverage_fraction: f64,
        efficiency: f64,
    ) -> f64 {
        Self::stellar_power(luminosity_solar) * coverage_fraction * efficiency
    }

    /// Calculate sphere surface area at given radius
    pub fn sphere_area_km2(radius_au: f64) -> f64 {
        let radius_km = radius_au * 1.496e8;
        4.0 * std::f64::consts::PI * radius_km * radius_km
    }

    /// Calculate number of collectors needed
    pub fn collectors_needed(
        radius_au: f64,
        coverage_fraction: f64,
        collector_area_km2: f64,
    ) -> u64 {
        let total_area = Self::sphere_area_km2(radius_au) * coverage_fraction;
        (total_area / collector_area_km2).ceil() as u64
    }
}

/// Terraforming calculator
pub struct TerraformingCalculator;

impl TerraformingCalculator {
    /// Calculate temperature increase from greenhouse gas release
    pub fn greenhouse_warming(co2_increase_ppm: f64) -> f64 {
        // Simplified logarithmic relationship
        // Each doubling of CO2 ~ 3K warming
        (co2_increase_ppm / 280.0).ln() * 3.0 / 0.693
    }

    /// Calculate oxygen production time from cyanobacteria
    pub fn oxygen_production_years(
        target_partial_pressure_mbar: f64,
        production_rate_kg_per_year: f64,
        _planet_mass_kg: f64,
    ) -> f64 {
        // Very simplified - Mars specific
        let mars_atmosphere_mass = 2.5e16; // kg
        let o2_mass_needed = mars_atmosphere_mass * (target_partial_pressure_mbar / 6.1) * 0.21;
        o2_mass_needed / production_rate_kg_per_year
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hohmann_earth_mars() {
        let delta_v = OrbitalSimulator::hohmann_delta_v(1.0, 1.524);
        // Should be approximately 5.6 km/s
        assert!(delta_v > 5.0 && delta_v < 6.0);
    }

    #[test]
    fn test_transfer_time() {
        let time = OrbitalSimulator::hohmann_transfer_time(1.0, 1.524);
        // Should be approximately 259 days
        assert!(time > 250.0 && time < 270.0);
    }

    #[test]
    fn test_lorentz_factor() {
        let gamma = InterstellarCalculator::lorentz_factor(0.0);
        assert!((gamma - 1.0).abs() < 0.001);

        let gamma_20 = InterstellarCalculator::lorentz_factor(0.2);
        assert!(gamma_20 > 1.02 && gamma_20 < 1.03);
    }

    #[test]
    fn test_alpha_centauri_trip() {
        let time = InterstellarCalculator::travel_time_years(4.246, 0.2);
        assert!((time - 21.23).abs() < 0.1);
    }

    #[test]
    fn test_dyson_power() {
        let power = DysonCalculator::harvestable_power(1.0, 0.01, 0.25);
        // 1% coverage, 25% efficiency
        assert!(power > 9e23 && power < 1e24);
    }
}
