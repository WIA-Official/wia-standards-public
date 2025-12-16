//! Core type definitions for WIA Robot SDK
//!
//! This module contains all shared data types used across the library.

use serde::{Deserialize, Serialize};

/// Robot type enumeration
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum RobotType {
    Exoskeleton,
    Prosthetics,
    Rehabilitation,
    CareRobot,
    Surgical,
    MobilityAid,
}

/// Device operational status
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum DeviceStatus {
    #[default]
    Operational,
    Standby,
    Error,
    Maintenance,
    Charging,
    Calibrating,
}

/// Body side (left or right)
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Side {
    Left,
    Right,
}

/// 3D position vector
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, PartialEq)]
pub struct Position3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Position3D {
    /// Create a new position
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Create a zero position
    pub fn zero() -> Self {
        Self::default()
    }

    /// Calculate Euclidean distance to another point
    pub fn distance_to(&self, other: &Position3D) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Calculate magnitude (length) of the vector
    pub fn magnitude(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Normalize the vector (unit vector)
    pub fn normalize(&self) -> Position3D {
        let mag = self.magnitude();
        if mag == 0.0 {
            return *self;
        }
        Position3D {
            x: self.x / mag,
            y: self.y / mag,
            z: self.z / mag,
        }
    }

    /// Add two vectors
    pub fn add(&self, other: &Position3D) -> Position3D {
        Position3D {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }

    /// Subtract two vectors
    pub fn sub(&self, other: &Position3D) -> Position3D {
        Position3D {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }

    /// Scale vector by a scalar
    pub fn scale(&self, factor: f64) -> Position3D {
        Position3D {
            x: self.x * factor,
            y: self.y * factor,
            z: self.z * factor,
        }
    }

    /// Dot product with another vector
    pub fn dot(&self, other: &Position3D) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Cross product with another vector
    pub fn cross(&self, other: &Position3D) -> Position3D {
        Position3D {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }
}

/// 2D pose (position and orientation in 2D plane)
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    /// Orientation in radians
    pub theta: f64,
}

impl Pose2D {
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self { x, y, theta }
    }

    /// Get theta in degrees
    pub fn theta_deg(&self) -> f64 {
        self.theta.to_degrees()
    }
}

/// Orientation in Roll-Pitch-Yaw (degrees)
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct Orientation {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

impl Orientation {
    pub fn new(roll: f64, pitch: f64, yaw: f64) -> Self {
        Self { roll, pitch, yaw }
    }
}

/// Quaternion for rotation representation
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }
    }
}

/// Joint state information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Joint {
    /// Joint name
    pub name: String,
    /// Current angle in degrees
    pub angle_deg: f64,
    /// Angular velocity in degrees per second
    pub velocity_deg_s: f64,
    /// Torque in Newton-meters
    pub torque_nm: f64,
    /// Target angle in degrees
    pub target_angle_deg: f64,
    /// Minimum angle limit
    pub min_angle_deg: f64,
    /// Maximum angle limit
    pub max_angle_deg: f64,
}

impl Joint {
    /// Create a new joint with name and limits
    pub fn new(name: &str, min_angle: f64, max_angle: f64) -> Self {
        Self {
            name: name.to_string(),
            angle_deg: 0.0,
            velocity_deg_s: 0.0,
            torque_nm: 0.0,
            target_angle_deg: 0.0,
            min_angle_deg: min_angle,
            max_angle_deg: max_angle,
        }
    }

    /// Calculate angle error (target - current)
    pub fn angle_error(&self) -> f64 {
        self.target_angle_deg - self.angle_deg
    }

    /// Check if current angle is within limits
    pub fn is_within_limits(&self) -> bool {
        self.angle_deg >= self.min_angle_deg && self.angle_deg <= self.max_angle_deg
    }

    /// Clamp target angle to joint limits
    pub fn clamp_target(&mut self) {
        self.target_angle_deg = self
            .target_angle_deg
            .clamp(self.min_angle_deg, self.max_angle_deg);
    }

    /// Simple PID control calculation
    pub fn compute_pid(&self, kp: f64, ki: f64, kd: f64, integral: f64) -> f64 {
        let error = self.angle_error();
        let derivative = -self.velocity_deg_s;
        kp * error + ki * integral + kd * derivative
    }
}

/// IMU sensor data
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct ImuData {
    /// Linear acceleration (m/s^2)
    pub acceleration: Position3D,
    /// Angular velocity (deg/s)
    pub gyroscope: Position3D,
    /// Orientation (degrees)
    pub orientation: Orientation,
}

impl ImuData {
    /// Calculate total acceleration magnitude
    pub fn total_acceleration(&self) -> f64 {
        self.acceleration.magnitude()
    }

    /// Check if acceleration indicates potential fall
    pub fn is_free_fall(&self, threshold: f64) -> bool {
        self.total_acceleration() < threshold
    }

    /// Check if orientation indicates tilt beyond threshold
    pub fn is_tilted(&self, pitch_threshold: f64, roll_threshold: f64) -> bool {
        self.orientation.pitch.abs() > pitch_threshold
            || self.orientation.roll.abs() > roll_threshold
    }
}

/// Force sensor data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ForceSensor {
    /// Sensor location
    pub location: String,
    /// Force in Newtons
    pub force_n: f64,
    /// Force vector (optional)
    pub force_vector: Option<Position3D>,
}

/// Velocity (linear and angular)
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct Velocity {
    /// Linear velocity (m/s)
    pub linear: f64,
    /// Angular velocity (rad/s)
    pub angular: f64,
}

/// Device information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceInfo {
    pub id: String,
    pub device_type: RobotType,
    pub name: String,
    pub manufacturer: String,
    pub model: String,
    pub firmware_version: String,
    pub serial_number: Option<String>,
    pub capabilities: Vec<String>,
}

/// Device state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceState {
    pub status: DeviceStatus,
    pub battery_percent: u8,
    pub battery_voltage: Option<f64>,
    pub battery_temp_c: Option<f64>,
    pub uptime_seconds: u64,
    pub last_calibration: Option<String>,
    pub error_codes: Vec<String>,
}

impl Default for DeviceState {
    fn default() -> Self {
        Self {
            status: DeviceStatus::Standby,
            battery_percent: 100,
            battery_voltage: None,
            battery_temp_c: None,
            uptime_seconds: 0,
            last_calibration: None,
            error_codes: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_position3d_distance() {
        let p1 = Position3D::new(0.0, 0.0, 0.0);
        let p2 = Position3D::new(3.0, 4.0, 0.0);
        assert!((p1.distance_to(&p2) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_position3d_normalize() {
        let p = Position3D::new(3.0, 4.0, 0.0);
        let n = p.normalize();
        assert!((n.magnitude() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_position3d_dot() {
        let p1 = Position3D::new(1.0, 0.0, 0.0);
        let p2 = Position3D::new(0.0, 1.0, 0.0);
        assert!((p1.dot(&p2)).abs() < 1e-10);
    }

    #[test]
    fn test_joint_angle_error() {
        let joint = Joint {
            name: "test".to_string(),
            angle_deg: 10.0,
            velocity_deg_s: 0.0,
            torque_nm: 0.0,
            target_angle_deg: 20.0,
            min_angle_deg: -90.0,
            max_angle_deg: 90.0,
        };
        assert!((joint.angle_error() - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_joint_within_limits() {
        let joint = Joint::new("test", -90.0, 90.0);
        assert!(joint.is_within_limits());
    }

    #[test]
    fn test_imu_tilt_detection() {
        let imu = ImuData {
            orientation: Orientation::new(35.0, 10.0, 0.0),
            ..Default::default()
        };
        assert!(imu.is_tilted(30.0, 30.0));
    }
}
