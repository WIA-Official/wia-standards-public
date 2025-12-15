//! Spatial encoding module for haptic feedback
//!
//! This module provides algorithms for encoding spatial information
//! (direction, distance, scenes) into haptic patterns.

pub mod direction;
pub mod distance;
pub mod scene;

pub use direction::*;
pub use distance::*;
pub use scene::*;

use crate::types::BodyLocation;

/// Actuator activation with intensity
#[derive(Debug, Clone, Copy)]
pub struct ActuatorActivation {
    /// Body location of the actuator
    pub location: BodyLocation,
    /// Position in degrees (0-360)
    pub position: f32,
    /// Activation intensity (0.0-1.0)
    pub intensity: f32,
}

/// 3D vector for spatial calculations
#[derive(Debug, Clone, Copy, Default)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3 {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn magnitude(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalize(&self) -> Self {
        let mag = self.magnitude();
        if mag > 0.0 {
            Self {
                x: self.x / mag,
                y: self.y / mag,
                z: self.z / mag,
            }
        } else {
            *self
        }
    }

    pub fn dot(&self, other: &Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
}

/// Spatial direction with azimuth and elevation
#[derive(Debug, Clone, Copy, Default)]
pub struct SpatialDirection {
    /// Horizontal angle (0-360°, 0 = forward, clockwise)
    pub azimuth: f32,
    /// Vertical angle (-90 to +90°, 0 = level)
    pub elevation: f32,
}

impl SpatialDirection {
    pub const fn new(azimuth: f32, elevation: f32) -> Self {
        Self { azimuth, elevation }
    }

    /// Convert to unit vector
    pub fn to_unit_vector(&self) -> Vector3 {
        let az_rad = self.azimuth.to_radians();
        let el_rad = self.elevation.to_radians();
        let cos_el = el_rad.cos();

        Vector3 {
            x: az_rad.sin() * cos_el,
            y: el_rad.sin(),
            z: az_rad.cos() * cos_el,
        }
    }

    /// Angular distance to another direction
    pub fn angle_to(&self, other: &Self) -> f32 {
        let v1 = self.to_unit_vector();
        let v2 = other.to_unit_vector();
        let dot = v1.dot(&v2).clamp(-1.0, 1.0);
        dot.acos().to_degrees()
    }

    /// Get cardinal direction
    pub fn to_cardinal(&self) -> CardinalDirection {
        let normalized = normalize_angle(self.azimuth);
        match normalized {
            a if a < 22.5 || a >= 337.5 => CardinalDirection::North,
            a if a < 67.5 => CardinalDirection::NorthEast,
            a if a < 112.5 => CardinalDirection::East,
            a if a < 157.5 => CardinalDirection::SouthEast,
            a if a < 202.5 => CardinalDirection::South,
            a if a < 247.5 => CardinalDirection::SouthWest,
            a if a < 292.5 => CardinalDirection::West,
            _ => CardinalDirection::NorthWest,
        }
    }

    /// Get clock position (1-12)
    pub fn to_clock_position(&self) -> u8 {
        let normalized = normalize_angle(self.azimuth);
        let clock = ((normalized + 15.0) / 30.0) as u8;
        if clock == 0 { 12 } else { clock.min(12) }
    }
}

/// Cardinal directions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CardinalDirection {
    North,
    NorthEast,
    East,
    SouthEast,
    South,
    SouthWest,
    West,
    NorthWest,
}

impl CardinalDirection {
    pub fn to_azimuth(&self) -> f32 {
        match self {
            Self::North => 0.0,
            Self::NorthEast => 45.0,
            Self::East => 90.0,
            Self::SouthEast => 135.0,
            Self::South => 180.0,
            Self::SouthWest => 225.0,
            Self::West => 270.0,
            Self::NorthWest => 315.0,
        }
    }
}

/// Normalize angle to 0-360 range
pub fn normalize_angle(angle: f32) -> f32 {
    let mut normalized = angle % 360.0;
    if normalized < 0.0 {
        normalized += 360.0;
    }
    normalized
}

/// Calculate shortest angular difference between two angles
pub fn angle_difference(a: f32, b: f32) -> f32 {
    let diff = normalize_angle(a - b);
    if diff > 180.0 { diff - 360.0 } else { diff }
}

/// Convert degrees to radians
pub fn to_radians(degrees: f32) -> f32 {
    degrees * core::f32::consts::PI / 180.0
}

/// Convert radians to degrees
pub fn to_degrees(radians: f32) -> f32 {
    radians * 180.0 / core::f32::consts::PI
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 0.001);
        assert!((normalize_angle(360.0) - 0.0).abs() < 0.001);
        assert!((normalize_angle(-90.0) - 270.0).abs() < 0.001);
        assert!((normalize_angle(450.0) - 90.0).abs() < 0.001);
    }

    #[test]
    fn test_angle_difference() {
        assert!((angle_difference(10.0, 350.0) - 20.0).abs() < 0.001);
        assert!((angle_difference(350.0, 10.0) - (-20.0)).abs() < 0.001);
        assert!((angle_difference(90.0, 270.0) - (-180.0)).abs() < 0.001);
    }

    #[test]
    fn test_spatial_direction() {
        let dir = SpatialDirection::new(0.0, 0.0);
        assert_eq!(dir.to_cardinal(), CardinalDirection::North);
        assert_eq!(dir.to_clock_position(), 12);

        let dir_east = SpatialDirection::new(90.0, 0.0);
        assert_eq!(dir_east.to_cardinal(), CardinalDirection::East);
        assert_eq!(dir_east.to_clock_position(), 3);
    }
}
