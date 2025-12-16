//! Position and orientation types for nano systems

use serde::{Deserialize, Serialize};
use std::ops::{Add, Sub, Mul};

/// 3D position in nanometers
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Position3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Position3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn zero() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0 }
    }

    /// Alias for zero() - origin point
    pub fn origin() -> Self {
        Self::zero()
    }

    pub fn magnitude(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn distance_to(&self, other: &Position3D) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
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
            Self::zero()
        }
    }
}

impl Add for Position3D {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl Sub for Position3D {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl Mul<f64> for Position3D {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

impl Default for Position3D {
    fn default() -> Self {
        Self::zero()
    }
}

/// Position in nanometers with explicit unit suffix
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PositionNm {
    pub x_nm: f64,
    pub y_nm: f64,
    pub z_nm: f64,
}

impl PositionNm {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x_nm: x, y_nm: y, z_nm: z }
    }

    pub fn to_position3d(&self) -> Position3D {
        Position3D::new(self.x_nm, self.y_nm, self.z_nm)
    }
}

/// Position in micrometers (for cell/tissue scale)
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PositionMicro {
    pub x_um: f64,
    pub y_um: f64,
    pub z_um: f64,
}

impl PositionMicro {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x_um: x, y_um: y, z_um: z }
    }

    pub fn to_nm(&self) -> PositionNm {
        PositionNm::new(
            self.x_um * 1000.0,
            self.y_um * 1000.0,
            self.z_um * 1000.0,
        )
    }

    pub fn distance_to(&self, other: &PositionMicro) -> f64 {
        let dx = self.x_um - other.x_um;
        let dy = self.y_um - other.y_um;
        let dz = self.z_um - other.z_um;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

impl Default for PositionMicro {
    fn default() -> Self {
        Self { x_um: 0.0, y_um: 0.0, z_um: 0.0 }
    }
}

/// Quaternion for orientation
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quaternion {
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    pub fn identity() -> Self {
        Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }
    }

    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        let (sr, cr) = (roll / 2.0).sin_cos();
        let (sp, cp) = (pitch / 2.0).sin_cos();
        let (sy, cy) = (yaw / 2.0).sin_cos();

        Self {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }

    pub fn magnitude(&self) -> f64 {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalize(&self) -> Self {
        let mag = self.magnitude();
        Self {
            w: self.w / mag,
            x: self.x / mag,
            y: self.y / mag,
            z: self.z / mag,
        }
    }
}

impl Default for Quaternion {
    fn default() -> Self {
        Self::identity()
    }
}

/// Orientation container
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Orientation {
    pub quaternion: Quaternion,
}

impl Orientation {
    pub fn identity() -> Self {
        Self { quaternion: Quaternion::identity() }
    }

    pub fn from_quaternion(q: Quaternion) -> Self {
        Self { quaternion: q }
    }

    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        Self { quaternion: Quaternion::from_euler(roll, pitch, yaw) }
    }
}

impl Default for Orientation {
    fn default() -> Self {
        Self::identity()
    }
}
