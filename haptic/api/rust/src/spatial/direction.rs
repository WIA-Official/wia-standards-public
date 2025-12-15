//! Directional encoding for haptic feedback
//!
//! Encodes azimuth and elevation into actuator activations.

use crate::types::BodyLocation;
use super::{ActuatorActivation, SpatialDirection, normalize_angle, angle_difference, to_radians};

/// Encoding algorithm for directional feedback
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DirectionEncodingAlgorithm {
    /// Only activate nearest actuator
    NearestNeighbor,
    /// Linear interpolation between two nearest
    LinearInterpolation,
    /// Cosine falloff from target direction
    CosineFalloff,
    /// Gaussian distribution around target
    GaussianFalloff,
}

/// Actuator configuration for directional encoding
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActuatorConfiguration {
    /// Single actuator (intensity only)
    Single,
    /// Two actuators (left/right)
    Dual,
    /// Four actuators (front/back/left/right)
    Quad,
    /// Eight actuators (45° spacing)
    Octal,
    /// Twelve or more actuators
    Continuous(u8),
}

impl ActuatorConfiguration {
    /// Get actuator positions in degrees
    pub fn positions(&self) -> &'static [f32] {
        match self {
            Self::Single => &[0.0],
            Self::Dual => &[270.0, 90.0],  // Left, Right
            Self::Quad => &[0.0, 90.0, 180.0, 270.0],  // Front, Right, Back, Left
            Self::Octal => &[0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0],
            Self::Continuous(_) => &[],  // Dynamic, use positions_continuous()
        }
    }

    /// Get positions for continuous configuration
    pub fn positions_continuous(&self, count: u8) -> Vec<f32> {
        let n = count as f32;
        (0..count).map(|i| (i as f32) * 360.0 / n).collect()
    }

    /// Get actuator count
    pub fn count(&self) -> u8 {
        match self {
            Self::Single => 1,
            Self::Dual => 2,
            Self::Quad => 4,
            Self::Octal => 8,
            Self::Continuous(n) => *n,
        }
    }
}

/// Directional encoder configuration
#[derive(Debug, Clone)]
pub struct DirectionEncoderConfig {
    /// Actuator configuration
    pub actuator_config: ActuatorConfiguration,
    /// Encoding algorithm
    pub algorithm: DirectionEncodingAlgorithm,
    /// Falloff angle for cosine/gaussian (degrees)
    pub falloff_angle: f32,
    /// Sigma for gaussian falloff (degrees)
    pub gaussian_sigma: f32,
    /// Minimum intensity threshold
    pub min_intensity: f32,
}

impl Default for DirectionEncoderConfig {
    fn default() -> Self {
        Self {
            actuator_config: ActuatorConfiguration::Quad,
            algorithm: DirectionEncodingAlgorithm::CosineFalloff,
            falloff_angle: 90.0,
            gaussian_sigma: 30.0,
            min_intensity: 0.05,
        }
    }
}

/// Directional encoder
pub struct DirectionEncoder {
    config: DirectionEncoderConfig,
    positions: Vec<f32>,
}

impl DirectionEncoder {
    /// Create a new directional encoder
    pub fn new(config: DirectionEncoderConfig) -> Self {
        let positions = match config.actuator_config {
            ActuatorConfiguration::Continuous(n) => {
                config.actuator_config.positions_continuous(n)
            }
            _ => config.actuator_config.positions().to_vec(),
        };

        Self { config, positions }
    }

    /// Create encoder with quad configuration (default)
    pub fn quad() -> Self {
        Self::new(DirectionEncoderConfig::default())
    }

    /// Create encoder with octal configuration
    pub fn octal() -> Self {
        Self::new(DirectionEncoderConfig {
            actuator_config: ActuatorConfiguration::Octal,
            ..Default::default()
        })
    }

    /// Encode direction to actuator activations
    pub fn encode(&self, direction: &SpatialDirection) -> Vec<ActuatorActivation> {
        let azimuth = normalize_angle(direction.azimuth);

        match self.config.algorithm {
            DirectionEncodingAlgorithm::NearestNeighbor => {
                self.encode_nearest_neighbor(azimuth)
            }
            DirectionEncodingAlgorithm::LinearInterpolation => {
                self.encode_linear_interpolation(azimuth)
            }
            DirectionEncodingAlgorithm::CosineFalloff => {
                self.encode_cosine_falloff(azimuth)
            }
            DirectionEncodingAlgorithm::GaussianFalloff => {
                self.encode_gaussian_falloff(azimuth)
            }
        }
    }

    /// Encode using nearest neighbor algorithm
    fn encode_nearest_neighbor(&self, azimuth: f32) -> Vec<ActuatorActivation> {
        let mut min_delta = f32::MAX;
        let mut nearest_idx = 0;

        for (i, &pos) in self.positions.iter().enumerate() {
            let delta = angle_difference(azimuth, pos).abs();
            if delta < min_delta {
                min_delta = delta;
                nearest_idx = i;
            }
        }

        self.positions
            .iter()
            .enumerate()
            .map(|(i, &pos)| ActuatorActivation {
                location: position_to_location(pos),
                position: pos,
                intensity: if i == nearest_idx { 1.0 } else { 0.0 },
            })
            .collect()
    }

    /// Encode using linear interpolation
    fn encode_linear_interpolation(&self, azimuth: f32) -> Vec<ActuatorActivation> {
        // Find two nearest positions
        let mut sorted: Vec<(usize, f32)> = self.positions
            .iter()
            .enumerate()
            .map(|(i, &pos)| (i, angle_difference(azimuth, pos).abs()))
            .collect();
        sorted.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

        let (idx1, delta1) = sorted[0];
        let (idx2, delta2) = if sorted.len() > 1 { sorted[1] } else { sorted[0] };

        let total_delta = delta1 + delta2;
        let weight1 = if total_delta > 0.0 { 1.0 - delta1 / total_delta } else { 1.0 };
        let weight2 = if total_delta > 0.0 { 1.0 - delta2 / total_delta } else { 0.0 };

        self.positions
            .iter()
            .enumerate()
            .map(|(i, &pos)| {
                let intensity = if i == idx1 {
                    weight1
                } else if i == idx2 {
                    weight2
                } else {
                    0.0
                };
                ActuatorActivation {
                    location: position_to_location(pos),
                    position: pos,
                    intensity,
                }
            })
            .collect()
    }

    /// Encode using cosine falloff
    fn encode_cosine_falloff(&self, azimuth: f32) -> Vec<ActuatorActivation> {
        self.positions
            .iter()
            .map(|&pos| {
                let delta = angle_difference(azimuth, pos);
                let normalized = delta / self.config.falloff_angle;

                let intensity = if normalized.abs() <= 1.0 {
                    (normalized * core::f32::consts::PI).cos() * 0.5 + 0.5
                } else {
                    0.0
                };

                let final_intensity = if intensity < self.config.min_intensity {
                    0.0
                } else {
                    intensity
                };

                ActuatorActivation {
                    location: position_to_location(pos),
                    position: pos,
                    intensity: final_intensity,
                }
            })
            .collect()
    }

    /// Encode using gaussian falloff
    fn encode_gaussian_falloff(&self, azimuth: f32) -> Vec<ActuatorActivation> {
        let sigma = self.config.gaussian_sigma;
        let sigma_sq = sigma * sigma;

        self.positions
            .iter()
            .map(|&pos| {
                let delta = angle_difference(azimuth, pos);
                let intensity = (-delta * delta / (2.0 * sigma_sq)).exp();

                let final_intensity = if intensity < self.config.min_intensity {
                    0.0
                } else {
                    intensity
                };

                ActuatorActivation {
                    location: position_to_location(pos),
                    position: pos,
                    intensity: final_intensity,
                }
            })
            .collect()
    }

    /// Encode with elevation (frequency modulation)
    pub fn encode_with_elevation(
        &self,
        direction: &SpatialDirection,
    ) -> (Vec<ActuatorActivation>, ElevationEncoding) {
        let activations = self.encode(direction);

        let elevation_encoding = if direction.elevation > 30.0 {
            ElevationEncoding::Above { frequency: 200 + (direction.elevation as u16 - 30) * 2 }
        } else if direction.elevation < -30.0 {
            ElevationEncoding::Below { frequency: 100 - ((-direction.elevation) as u16 - 30) }
        } else {
            ElevationEncoding::Level { frequency: 150 }
        };

        (activations, elevation_encoding)
    }
}

/// Elevation encoding result
#[derive(Debug, Clone, Copy)]
pub enum ElevationEncoding {
    Above { frequency: u16 },
    Level { frequency: u16 },
    Below { frequency: u16 },
}

impl ElevationEncoding {
    pub fn frequency(&self) -> u16 {
        match self {
            Self::Above { frequency } => *frequency,
            Self::Level { frequency } => *frequency,
            Self::Below { frequency } => *frequency,
        }
    }
}

/// Map actuator position to body location
fn position_to_location(position: f32) -> BodyLocation {
    let normalized = normalize_angle(position);
    match normalized as u16 {
        0..=22 | 338..=360 => BodyLocation::ChestCenter,
        23..=67 => BodyLocation::ChestRight,
        68..=112 => BodyLocation::WaistRight,
        113..=157 => BodyLocation::BackUpperRight,
        158..=202 => BodyLocation::BackUpperCenter,
        203..=247 => BodyLocation::BackUpperLeft,
        248..=292 => BodyLocation::WaistLeft,
        293..=337 => BodyLocation::ChestLeft,
        _ => BodyLocation::ChestCenter,
    }
}

/// Create a sweep pattern for direction indication
pub fn create_sweep_pattern(
    target_azimuth: f32,
    encoder: &DirectionEncoder,
    sweep_duration_ms: u32,
) -> Vec<(u32, Vec<ActuatorActivation>)> {
    let start_azimuth = normalize_angle(target_azimuth + 180.0);
    let steps = 10;
    let step_duration = sweep_duration_ms / steps;

    (0..steps)
        .map(|i| {
            let progress = i as f32 / steps as f32;
            let current_azimuth = start_azimuth + angle_difference(target_azimuth, start_azimuth) * progress;
            let direction = SpatialDirection::new(current_azimuth, 0.0);
            let activations = encoder.encode(&direction);
            (i * step_duration, activations)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quad_encoder() {
        let encoder = DirectionEncoder::quad();

        // Test forward direction
        let activations = encoder.encode(&SpatialDirection::new(0.0, 0.0));
        let front = activations.iter().find(|a| a.position == 0.0).unwrap();
        assert!(front.intensity > 0.9);

        // Test right direction
        let activations = encoder.encode(&SpatialDirection::new(90.0, 0.0));
        let right = activations.iter().find(|a| a.position == 90.0).unwrap();
        assert!(right.intensity > 0.9);
    }

    #[test]
    fn test_interpolation() {
        let encoder = DirectionEncoder::quad();

        // Test 45° (between front and right)
        let activations = encoder.encode(&SpatialDirection::new(45.0, 0.0));
        let front = activations.iter().find(|a| a.position == 0.0).unwrap();
        let right = activations.iter().find(|a| a.position == 90.0).unwrap();

        // Both should have similar activation
        assert!((front.intensity - right.intensity).abs() < 0.1);
        assert!(front.intensity > 0.4);
    }

    #[test]
    fn test_elevation_encoding() {
        let encoder = DirectionEncoder::quad();

        let (_, elevation) = encoder.encode_with_elevation(&SpatialDirection::new(0.0, 60.0));
        assert!(matches!(elevation, ElevationEncoding::Above { .. }));

        let (_, elevation) = encoder.encode_with_elevation(&SpatialDirection::new(0.0, -60.0));
        assert!(matches!(elevation, ElevationEncoding::Below { .. }));

        let (_, elevation) = encoder.encode_with_elevation(&SpatialDirection::new(0.0, 0.0));
        assert!(matches!(elevation, ElevationEncoding::Level { .. }));
    }
}
