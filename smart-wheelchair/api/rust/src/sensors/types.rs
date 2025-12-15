//! Common types for WIA Smart Wheelchair sensors

use std::time::SystemTime;

/// 3D Vector
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Vector3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn zero() -> Self {
        Self::default()
    }

    pub fn magnitude(&self) -> f64 {
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
            Self::zero()
        }
    }
}

/// 2D Vector
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Vector2D {
    pub x: f64,
    pub y: f64,
}

impl Vector2D {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn magnitude(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }
}

/// Quaternion for orientation
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self::identity()
    }
}

impl Quaternion {
    pub fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Self { x, y, z, w }
    }

    pub fn identity() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    }

    pub fn magnitude(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt()
    }

    pub fn normalize(&self) -> Self {
        let mag = self.magnitude();
        if mag > 0.0 {
            Self {
                x: self.x / mag,
                y: self.y / mag,
                z: self.z / mag,
                w: self.w / mag,
            }
        } else {
            Self::identity()
        }
    }

    /// Convert to Euler angles (roll, pitch, yaw) in radians
    pub fn to_euler(&self) -> (f64, f64, f64) {
        // Roll (x-axis rotation)
        let sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z);
        let cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y);
        let roll = sinr_cosp.atan2(cosr_cosp);

        // Pitch (y-axis rotation)
        let sinp = 2.0 * (self.w * self.y - self.z * self.x);
        let pitch = if sinp.abs() >= 1.0 {
            std::f64::consts::FRAC_PI_2.copysign(sinp)
        } else {
            sinp.asin()
        };

        // Yaw (z-axis rotation)
        let siny_cosp = 2.0 * (self.w * self.z + self.x * self.y);
        let cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        let yaw = siny_cosp.atan2(cosy_cosp);

        (roll, pitch, yaw)
    }

    /// Create from Euler angles (roll, pitch, yaw) in radians
    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        let cy = (yaw * 0.5).cos();
        let sy = (yaw * 0.5).sin();
        let cp = (pitch * 0.5).cos();
        let sp = (pitch * 0.5).sin();
        let cr = (roll * 0.5).cos();
        let sr = (roll * 0.5).sin();

        Self {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }
}

/// Message header
#[derive(Debug, Clone)]
pub struct Header {
    pub timestamp: SystemTime,
    pub frame_id: String,
    pub seq: u32,
}

impl Default for Header {
    fn default() -> Self {
        Self {
            timestamp: SystemTime::now(),
            frame_id: String::new(),
            seq: 0,
        }
    }
}

impl Header {
    pub fn new(frame_id: &str) -> Self {
        Self {
            timestamp: SystemTime::now(),
            frame_id: frame_id.to_string(),
            seq: 0,
        }
    }

    pub fn with_seq(mut self, seq: u32) -> Self {
        self.seq = seq;
        self
    }
}

/// 3D Pose
#[derive(Debug, Clone, Copy, Default)]
pub struct Pose {
    pub position: Vector3D,
    pub orientation: Quaternion,
}

/// 2D Pose (for planar navigation)
#[derive(Debug, Clone, Copy, Default)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl Pose2D {
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self { x, y, theta }
    }
}

/// Twist (velocity)
#[derive(Debug, Clone, Copy, Default)]
pub struct Twist {
    pub linear: Vector3D,
    pub angular: Vector3D,
}

/// Sensor status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SensorStatus {
    Disconnected = 0,
    Initializing = 1,
    Ready = 2,
    Running = 3,
    Error = 4,
    Calibrating = 5,
}

impl Default for SensorStatus {
    fn default() -> Self {
        Self::Disconnected
    }
}

/// 6x6 Covariance matrix (row-major)
pub type CovarianceMatrix6x6 = [f64; 36];

/// Create diagonal covariance matrix
pub fn diagonal_covariance(diag: [f64; 6]) -> CovarianceMatrix6x6 {
    let mut cov = [0.0; 36];
    for i in 0..6 {
        cov[i * 6 + i] = diag[i];
    }
    cov
}

/// Normalize angle to [-pi, pi]
pub fn normalize_angle(mut angle: f64) -> f64 {
    use std::f64::consts::PI;
    while angle > PI {
        angle -= 2.0 * PI;
    }
    while angle < -PI {
        angle += 2.0 * PI;
    }
    angle
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vector3d_magnitude() {
        let v = Vector3D::new(3.0, 4.0, 0.0);
        assert!((v.magnitude() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_quaternion_euler_roundtrip() {
        let roll = 0.1;
        let pitch = 0.2;
        let yaw = 0.3;

        let q = Quaternion::from_euler(roll, pitch, yaw);
        let (r, p, y) = q.to_euler();

        assert!((r - roll).abs() < 1e-10);
        assert!((p - pitch).abs() < 1e-10);
        assert!((y - yaw).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_angle() {
        use std::f64::consts::PI;
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 1e-10);
        assert!((normalize_angle(-3.0 * PI) - (-PI)).abs() < 1e-10);
    }
}
