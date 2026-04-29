//! Guided transport mechanisms
//!
//! Implements externally guided transport using magnetic fields, acoustics, or optical methods.

use crate::error::{NanoError, NanoResult};
use crate::types::Position3D;
use crate::transport::{Transport, TransportMethod, TransportResult};
use async_trait::async_trait;
use serde::{Deserialize, Serialize};

/// Configuration for guided transport
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GuidedConfig {
    /// Guidance mechanism type
    pub guidance_type: GuidanceType,
    /// Field strength in Tesla (for magnetic) or appropriate units
    pub field_strength: f64,
    /// Field gradient (T/m for magnetic)
    pub gradient: f64,
    /// Oscillation frequency in Hz
    pub frequency_hz: f64,
    /// Maximum velocity in nm/s
    pub max_velocity_nm_per_s: f64,
    /// Maximum range in nm
    pub max_range_nm: f64,
}

impl Default for GuidedConfig {
    fn default() -> Self {
        Self {
            guidance_type: GuidanceType::MagneticField,
            field_strength: 0.1,           // 0.1 T (100 mT)
            gradient: 0.01,                // 0.01 T/m
            frequency_hz: 10.0,            // 10 Hz rotation
            max_velocity_nm_per_s: 1000.0, // 1 μm/s
            max_range_nm: 100000.0,        // 100 μm
        }
    }
}

impl GuidedConfig {
    /// Create configuration for magnetic propulsion
    pub fn magnetic_propulsion() -> Self {
        Self {
            guidance_type: GuidanceType::MagneticField,
            field_strength: 0.05,
            gradient: 0.1,
            frequency_hz: 20.0,
            max_velocity_nm_per_s: 5000.0, // 5 μm/s
            max_range_nm: 1000000.0,       // 1 mm
        }
    }

    /// Create configuration for acoustic manipulation
    pub fn acoustic_tweezer() -> Self {
        Self {
            guidance_type: GuidanceType::Acoustic,
            field_strength: 1.0, // Pressure in Pa
            gradient: 0.0,
            frequency_hz: 1000000.0, // 1 MHz
            max_velocity_nm_per_s: 10000.0,
            max_range_nm: 10000000.0, // 10 mm
        }
    }

    /// Create configuration for optical tweezers
    pub fn optical_tweezer() -> Self {
        Self {
            guidance_type: GuidanceType::Optical,
            field_strength: 0.001, // Laser power in W
            gradient: 0.0,
            frequency_hz: 0.0,
            max_velocity_nm_per_s: 500.0,
            max_range_nm: 100000.0, // 100 μm
        }
    }
}

/// Type of guidance mechanism
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GuidanceType {
    /// Magnetic field guidance
    MagneticField,
    /// Acoustic/ultrasound guidance
    Acoustic,
    /// Optical tweezers
    Optical,
    /// Chemical gradient (chemotaxis)
    ChemicalGradient,
}

/// Guided transport implementation
pub struct GuidedTransport {
    config: GuidedConfig,
}

impl GuidedTransport {
    /// Create a new guided transport
    pub fn new(config: GuidedConfig) -> Self {
        Self { config }
    }

    /// Get the configuration
    pub fn config(&self) -> &GuidedConfig {
        &self.config
    }

    /// Calculate velocity based on field parameters
    pub fn calculate_velocity(&self) -> f64 {
        match self.config.guidance_type {
            GuidanceType::MagneticField => {
                // Velocity proportional to field gradient
                // Simplified model: v ∝ B * ∇B
                let velocity = self.config.field_strength * self.config.gradient * 1e12;
                velocity.min(self.config.max_velocity_nm_per_s)
            }
            GuidanceType::Acoustic => {
                // Acoustic radiation force
                let velocity = self.config.field_strength * 100.0;
                velocity.min(self.config.max_velocity_nm_per_s)
            }
            GuidanceType::Optical => {
                // Optical trap velocity
                let velocity = self.config.field_strength * 500000.0;
                velocity.min(self.config.max_velocity_nm_per_s)
            }
            GuidanceType::ChemicalGradient => {
                // Chemotaxis velocity
                self.config.max_velocity_nm_per_s * 0.1
            }
        }
    }

    /// Calculate magnetic force on a particle
    /// F = V * χ * (B · ∇)B / μ₀
    pub fn calculate_magnetic_force(
        &self,
        particle_volume_nm3: f64,
        magnetic_susceptibility: f64,
    ) -> f64 {
        if self.config.guidance_type != GuidanceType::MagneticField {
            return 0.0;
        }

        let mu_0 = 1.256637e-6; // Permeability of free space (H/m)
        let v_m3 = particle_volume_nm3 * 1e-27; // Convert nm³ to m³

        // F = V * χ * B * ∇B / μ₀
        v_m3 * magnetic_susceptibility * self.config.field_strength * self.config.gradient / mu_0
    }

    /// Calculate acoustic radiation force
    /// F_acoustic ∝ V * (pressure)² / (ρ * c²)
    pub fn calculate_acoustic_force(
        &self,
        particle_volume_nm3: f64,
        medium_density_kg_m3: f64,
    ) -> f64 {
        if self.config.guidance_type != GuidanceType::Acoustic {
            return 0.0;
        }

        let c = 1500.0; // Speed of sound in water (m/s)
        let v_m3 = particle_volume_nm3 * 1e-27;

        v_m3 * self.config.field_strength.powi(2) / (medium_density_kg_m3 * c * c)
    }
}

#[async_trait]
impl Transport for GuidedTransport {
    fn method(&self) -> TransportMethod {
        TransportMethod::Guided
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

        let velocity = self.calculate_velocity();
        if velocity <= 0.0 {
            return Err(NanoError::TransportFailed(
                "Cannot calculate velocity".into()
            ));
        }

        Ok(distance / velocity)
    }

    fn estimate_signal_strength(
        &self,
        source: &Position3D,
        destination: &Position3D,
        _time_s: f64,
    ) -> NanoResult<f64> {
        let distance = source.distance_to(destination);

        if distance > self.config.max_range_nm {
            return Ok(0.0);
        }

        // Guided transport maintains signal strength better than diffusion
        // Simple linear decay model
        let strength = 1.0 - (distance / self.config.max_range_nm);
        Ok(strength.max(0.0))
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

        let delivery_time = self.estimate_delivery_time(source, destination)?;
        let signal_strength = self.estimate_signal_strength(source, destination, delivery_time)?;

        Ok(TransportResult::success(delivery_time, signal_strength))
    }

    fn is_reachable(&self, source: &Position3D, destination: &Position3D) -> bool {
        source.distance_to(destination) <= self.config.max_range_nm
    }
}

/// Trajectory planner for guided transport
pub struct TrajectoryPlanner {
    config: GuidedConfig,
    waypoints: Vec<Position3D>,
}

impl TrajectoryPlanner {
    /// Create a new trajectory planner
    pub fn new(config: GuidedConfig) -> Self {
        Self {
            config,
            waypoints: Vec::new(),
        }
    }

    /// Add a waypoint
    pub fn add_waypoint(&mut self, point: Position3D) {
        self.waypoints.push(point);
    }

    /// Calculate total trajectory length
    pub fn total_length(&self) -> f64 {
        if self.waypoints.len() < 2 {
            return 0.0;
        }

        let mut total = 0.0;
        for i in 1..self.waypoints.len() {
            total += self.waypoints[i - 1].distance_to(&self.waypoints[i]);
        }
        total
    }

    /// Estimate total travel time
    pub fn estimate_travel_time(&self) -> f64 {
        let transport = GuidedTransport::new(self.config.clone());
        let velocity = transport.calculate_velocity();

        if velocity <= 0.0 {
            return f64::INFINITY;
        }

        self.total_length() / velocity
    }

    /// Get position at time t along trajectory
    pub fn position_at_time(&self, t: f64) -> Option<Position3D> {
        if self.waypoints.len() < 2 || t < 0.0 {
            return self.waypoints.first().cloned();
        }

        let transport = GuidedTransport::new(self.config.clone());
        let velocity = transport.calculate_velocity();
        let target_distance = velocity * t;

        let mut accumulated = 0.0;
        for i in 1..self.waypoints.len() {
            let segment_length = self.waypoints[i - 1].distance_to(&self.waypoints[i]);

            if accumulated + segment_length >= target_distance {
                let segment_fraction = (target_distance - accumulated) / segment_length;
                return Some(Position3D::new(
                    self.waypoints[i - 1].x + segment_fraction * (self.waypoints[i].x - self.waypoints[i - 1].x),
                    self.waypoints[i - 1].y + segment_fraction * (self.waypoints[i].y - self.waypoints[i - 1].y),
                    self.waypoints[i - 1].z + segment_fraction * (self.waypoints[i].z - self.waypoints[i - 1].z),
                ));
            }
            accumulated += segment_length;
        }

        self.waypoints.last().cloned()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_velocity_calculation() {
        let config = GuidedConfig::default();
        let transport = GuidedTransport::new(config);

        let velocity = transport.calculate_velocity();
        assert!(velocity > 0.0);
        assert!(velocity <= transport.config.max_velocity_nm_per_s);
    }

    #[test]
    fn test_magnetic_force() {
        let config = GuidedConfig::magnetic_propulsion();
        let transport = GuidedTransport::new(config);

        let force = transport.calculate_magnetic_force(
            1000.0, // 1000 nm³ particle
            1.0,    // Unit susceptibility
        );

        assert!(force > 0.0);
    }

    #[tokio::test]
    async fn test_guided_send() {
        let config = GuidedConfig::default();
        let transport = GuidedTransport::new(config);

        let source = Position3D::new(0.0, 0.0, 0.0);
        let dest = Position3D::new(1000.0, 0.0, 0.0);

        let result = transport.send(&source, &dest, 100).await.unwrap();
        assert!(result.success);
        assert!(result.delivery_time_s.unwrap() > 0.0);
    }

    #[test]
    fn test_trajectory_planner() {
        let config = GuidedConfig::default();
        let mut planner = TrajectoryPlanner::new(config);

        planner.add_waypoint(Position3D::new(0.0, 0.0, 0.0));
        planner.add_waypoint(Position3D::new(1000.0, 0.0, 0.0));
        planner.add_waypoint(Position3D::new(1000.0, 1000.0, 0.0));

        let length = planner.total_length();
        assert!((length - 2000.0).abs() < 0.1);

        let time = planner.estimate_travel_time();
        assert!(time > 0.0);
    }
}
