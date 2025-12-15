//! Distance encoding for haptic feedback
//!
//! Encodes distance information into haptic parameters (pulse rate, intensity, etc.)

use crate::types::WaveformType;

/// Distance zone classification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DistanceZone {
    /// 0 - 0.5m: Immediate danger
    Critical,
    /// 0.5 - 2m: High priority warning
    Near,
    /// 2 - 5m: Moderate awareness
    Medium,
    /// 5 - 10m: Low priority info
    Far,
    /// > 10m: Safe, minimal feedback
    Safe,
}

impl DistanceZone {
    /// Get zone from distance in meters
    pub fn from_distance(distance: f32) -> Self {
        match distance {
            d if d < 0.5 => Self::Critical,
            d if d < 2.0 => Self::Near,
            d if d < 5.0 => Self::Medium,
            d if d < 10.0 => Self::Far,
            _ => Self::Safe,
        }
    }

    /// Get zone from distance with custom thresholds
    pub fn from_distance_custom(distance: f32, thresholds: &ZoneThresholds) -> Self {
        match distance {
            d if d < thresholds.critical => Self::Critical,
            d if d < thresholds.near => Self::Near,
            d if d < thresholds.medium => Self::Medium,
            d if d < thresholds.far => Self::Far,
            _ => Self::Safe,
        }
    }

    /// Get priority level (0.0 - 1.0)
    pub fn priority(&self) -> f32 {
        match self {
            Self::Critical => 1.0,
            Self::Near => 0.8,
            Self::Medium => 0.5,
            Self::Far => 0.2,
            Self::Safe => 0.0,
        }
    }
}

/// Custom zone thresholds
#[derive(Debug, Clone, Copy)]
pub struct ZoneThresholds {
    pub critical: f32,
    pub near: f32,
    pub medium: f32,
    pub far: f32,
}

impl Default for ZoneThresholds {
    fn default() -> Self {
        Self {
            critical: 0.5,
            near: 2.0,
            medium: 5.0,
            far: 10.0,
        }
    }
}

impl ZoneThresholds {
    /// Indoor thresholds (smaller scale)
    pub fn indoor() -> Self {
        Self {
            critical: 0.3,
            near: 1.0,
            medium: 2.5,
            far: 5.0,
        }
    }

    /// Outdoor thresholds (larger scale)
    pub fn outdoor() -> Self {
        Self {
            critical: 1.0,
            near: 4.0,
            medium: 10.0,
            far: 20.0,
        }
    }
}

/// Encoding method for distance
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DistanceEncodingMethod {
    /// Closer = faster pulse rate
    Frequency,
    /// Closer = stronger vibration
    Intensity,
    /// Closer = shorter gaps between pulses
    Rhythm,
    /// Combination of all methods
    Combined,
}

/// Distance encoding curve type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodingCurve {
    /// Linear mapping
    Linear,
    /// Exponential (more sensitive at close range)
    Exponential,
    /// Logarithmic (more sensitive at far range)
    Logarithmic,
    /// Quadratic falloff
    Quadratic,
    /// Inverse square (physical falloff)
    InverseSquare,
}

/// Distance encoder configuration
#[derive(Debug, Clone)]
pub struct DistanceEncoderConfig {
    /// Maximum encoding range (meters)
    pub max_range: f32,
    /// Encoding method
    pub method: DistanceEncodingMethod,
    /// Mapping curve
    pub curve: EncodingCurve,
    /// Zone thresholds
    pub thresholds: ZoneThresholds,
    /// Minimum pulse rate (Hz)
    pub min_pulse_rate: f32,
    /// Maximum pulse rate (Hz)
    pub max_pulse_rate: f32,
    /// Minimum intensity
    pub min_intensity: f32,
    /// Maximum intensity
    pub max_intensity: f32,
}

impl Default for DistanceEncoderConfig {
    fn default() -> Self {
        Self {
            max_range: 10.0,
            method: DistanceEncodingMethod::Combined,
            curve: EncodingCurve::Exponential,
            thresholds: ZoneThresholds::default(),
            min_pulse_rate: 0.5,
            max_pulse_rate: 20.0,
            min_intensity: 0.1,
            max_intensity: 1.0,
        }
    }
}

/// Distance encoding result
#[derive(Debug, Clone, Copy)]
pub struct DistanceEncodingResult {
    /// Pulse rate in Hz
    pub pulse_rate: f32,
    /// Intensity (0.0 - 1.0)
    pub intensity: f32,
    /// Suggested waveform
    pub waveform: WaveformType,
    /// Vibration frequency in Hz
    pub frequency: u16,
    /// Distance zone
    pub zone: DistanceZone,
}

/// Distance encoder
pub struct DistanceEncoder {
    config: DistanceEncoderConfig,
}

impl DistanceEncoder {
    /// Create a new distance encoder
    pub fn new(config: DistanceEncoderConfig) -> Self {
        Self { config }
    }

    /// Create encoder with default configuration
    pub fn default_encoder() -> Self {
        Self::new(DistanceEncoderConfig::default())
    }

    /// Encode distance to haptic parameters
    pub fn encode(&self, distance: f32) -> DistanceEncodingResult {
        let zone = DistanceZone::from_distance_custom(distance, &self.config.thresholds);

        // If beyond max range, return safe result
        if distance >= self.config.max_range {
            return DistanceEncodingResult {
                pulse_rate: 0.0,
                intensity: 0.0,
                waveform: WaveformType::Sine,
                frequency: 60,
                zone: DistanceZone::Safe,
            };
        }

        // Calculate normalized distance (0 = close, 1 = far)
        let normalized = (distance / self.config.max_range).clamp(0.0, 1.0);

        // Apply curve transformation
        let curved = self.apply_curve(normalized);

        // Calculate parameters based on method
        let (pulse_rate, intensity) = match self.config.method {
            DistanceEncodingMethod::Frequency => {
                let rate = self.interpolate_pulse_rate(curved);
                (rate, 0.6)
            }
            DistanceEncodingMethod::Intensity => {
                let int = self.interpolate_intensity(curved);
                (5.0, int)
            }
            DistanceEncodingMethod::Rhythm => {
                let rate = self.interpolate_pulse_rate(curved);
                (rate, 0.5)
            }
            DistanceEncodingMethod::Combined => {
                let rate = self.interpolate_pulse_rate(curved);
                let int = self.interpolate_intensity(curved);
                (rate, int)
            }
        };

        // Determine waveform based on zone
        let waveform = match zone {
            DistanceZone::Critical => WaveformType::Square,
            DistanceZone::Near => WaveformType::Square,
            DistanceZone::Medium => WaveformType::Triangle,
            DistanceZone::Far => WaveformType::Sine,
            DistanceZone::Safe => WaveformType::Sine,
        };

        // Determine frequency based on proximity
        let frequency = (60.0 + (1.0 - curved) * 140.0) as u16;

        DistanceEncodingResult {
            pulse_rate,
            intensity,
            waveform,
            frequency,
            zone,
        }
    }

    /// Apply encoding curve transformation
    fn apply_curve(&self, normalized: f32) -> f32 {
        match self.config.curve {
            EncodingCurve::Linear => normalized,
            EncodingCurve::Exponential => {
                // More sensitive at close range
                normalized.powf(2.0)
            }
            EncodingCurve::Logarithmic => {
                // More sensitive at far range
                (1.0 + normalized * 9.0).log10()
            }
            EncodingCurve::Quadratic => {
                normalized * normalized
            }
            EncodingCurve::InverseSquare => {
                // Physical falloff model
                let safe_dist = normalized.max(0.1);
                1.0 / (safe_dist * safe_dist)
            }
        }
    }

    /// Interpolate pulse rate (closer = faster)
    fn interpolate_pulse_rate(&self, curved: f32) -> f32 {
        let range = self.config.max_pulse_rate - self.config.min_pulse_rate;
        self.config.max_pulse_rate - (curved * range)
    }

    /// Interpolate intensity (closer = stronger)
    fn interpolate_intensity(&self, curved: f32) -> f32 {
        let range = self.config.max_intensity - self.config.min_intensity;
        self.config.max_intensity - (curved * range)
    }

    /// Get the distance zone
    pub fn get_zone(&self, distance: f32) -> DistanceZone {
        DistanceZone::from_distance_custom(distance, &self.config.thresholds)
    }

    /// Check if distance is in critical zone
    pub fn is_critical(&self, distance: f32) -> bool {
        self.get_zone(distance) == DistanceZone::Critical
    }

    /// Encode with obstacle type modifier
    pub fn encode_with_type(
        &self,
        distance: f32,
        obstacle_type: ObstacleType,
    ) -> DistanceEncodingResult {
        let modifier = obstacle_type.modifier();
        let effective_distance = distance / modifier.distance_scale;
        let mut result = self.encode(effective_distance);

        // Apply intensity modifier
        result.intensity = (result.intensity * modifier.intensity_scale).clamp(0.0, 1.0);

        // Override waveform if specified
        if let Some(waveform) = modifier.waveform_override {
            result.waveform = waveform;
        }

        result
    }
}

/// Obstacle types for distance encoding
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObstacleType {
    Wall,
    Furniture,
    Person,
    Vehicle,
    Cyclist,
    StairsUp,
    StairsDown,
    Drop,
    Doorway,
    Unknown,
}

impl ObstacleType {
    /// Get encoding modifier for this obstacle type
    pub fn modifier(&self) -> ObstacleModifier {
        match self {
            Self::Wall => ObstacleModifier {
                intensity_scale: 1.0,
                distance_scale: 1.0,
                waveform_override: None,
            },
            Self::Person => ObstacleModifier {
                intensity_scale: 1.2,
                distance_scale: 0.8,  // Treat as closer
                waveform_override: None,
            },
            Self::Vehicle => ObstacleModifier {
                intensity_scale: 1.5,
                distance_scale: 0.5,  // Much closer
                waveform_override: Some(WaveformType::Square),
            },
            Self::Cyclist => ObstacleModifier {
                intensity_scale: 1.3,
                distance_scale: 0.7,
                waveform_override: None,
            },
            Self::Drop => ObstacleModifier {
                intensity_scale: 2.0,
                distance_scale: 0.4,  // Very urgent
                waveform_override: Some(WaveformType::Square),
            },
            Self::StairsDown => ObstacleModifier {
                intensity_scale: 1.3,
                distance_scale: 0.8,
                waveform_override: Some(WaveformType::Sawtooth),
            },
            Self::StairsUp => ObstacleModifier {
                intensity_scale: 1.1,
                distance_scale: 1.0,
                waveform_override: Some(WaveformType::Sawtooth),
            },
            Self::Doorway => ObstacleModifier {
                intensity_scale: 0.5,
                distance_scale: 1.5,  // Treat as farther (opportunity)
                waveform_override: None,
            },
            Self::Furniture => ObstacleModifier {
                intensity_scale: 1.0,
                distance_scale: 1.0,
                waveform_override: None,
            },
            Self::Unknown => ObstacleModifier {
                intensity_scale: 1.1,
                distance_scale: 0.9,
                waveform_override: Some(WaveformType::Noise),
            },
        }
    }

    /// Get priority value (0.0 - 1.0)
    pub fn priority(&self) -> f32 {
        match self {
            Self::Drop => 1.0,
            Self::Vehicle => 0.9,
            Self::Cyclist => 0.8,
            Self::Person => 0.6,
            Self::StairsDown => 0.7,
            Self::StairsUp => 0.5,
            Self::Unknown => 0.5,
            Self::Wall => 0.3,
            Self::Furniture => 0.3,
            Self::Doorway => 0.2,
        }
    }
}

/// Modifier for obstacle type
#[derive(Debug, Clone, Copy)]
pub struct ObstacleModifier {
    pub intensity_scale: f32,
    pub distance_scale: f32,
    pub waveform_override: Option<WaveformType>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_distance_zones() {
        assert_eq!(DistanceZone::from_distance(0.3), DistanceZone::Critical);
        assert_eq!(DistanceZone::from_distance(1.0), DistanceZone::Near);
        assert_eq!(DistanceZone::from_distance(3.0), DistanceZone::Medium);
        assert_eq!(DistanceZone::from_distance(7.0), DistanceZone::Far);
        assert_eq!(DistanceZone::from_distance(15.0), DistanceZone::Safe);
    }

    #[test]
    fn test_distance_encoding() {
        let encoder = DistanceEncoder::default_encoder();

        // Critical distance should have high pulse rate and intensity
        let critical = encoder.encode(0.3);
        assert!(critical.pulse_rate > 10.0);
        assert!(critical.intensity > 0.8);
        assert_eq!(critical.zone, DistanceZone::Critical);

        // Far distance should have low pulse rate
        let far = encoder.encode(8.0);
        assert!(far.pulse_rate < 5.0);
        assert!(far.intensity < 0.4);
    }

    #[test]
    fn test_obstacle_type_modifier() {
        let encoder = DistanceEncoder::default_encoder();

        // Vehicle should be more urgent
        let wall = encoder.encode(2.0);
        let vehicle = encoder.encode_with_type(2.0, ObstacleType::Vehicle);

        assert!(vehicle.intensity > wall.intensity);
    }
}
