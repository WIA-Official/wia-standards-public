//! Mobility aid robot adapter
//!
//! Provides control for wheelchairs, walkers, and other mobility aids.

use crate::error::{RobotError, RobotResult};
use crate::types::{Pose2D, Position3D, Velocity};
use serde::{Deserialize, Serialize};

/// Mobility aid type
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum MobilityType {
    #[default]
    PoweredWheelchair,
    SmartWalker,
    MobilityScooter,
    StairClimber,
}

/// Autonomous mode
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum AutonomousMode {
    #[default]
    Manual,
    SemiAutonomous,
    FullyAutonomous,
}

/// Path status
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum PathStatus {
    #[default]
    Idle,
    Planning,
    Navigating,
    Arrived,
    Blocked,
}

/// Input method
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum InputMethod {
    #[default]
    Joystick,
    HeadTracker,
    Voice,
    EyeGaze,
    SipPuff,
    Switch,
}

/// Surface type
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum SurfaceType {
    #[default]
    Smooth,
    Carpet,
    Outdoor,
    Gravel,
}

/// Armrest position
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum ArmrestPosition {
    Up,
    #[default]
    Down,
    Adjustable,
}

/// Navigation data
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct MobilityNavigation {
    pub current_pose: Pose2D,
    pub destination: Option<Destination>,
    pub path_status: PathStatus,
    pub distance_to_goal_m: Option<f64>,
    pub eta_seconds: Option<u32>,
    pub map_id: Option<String>,
    pub localization_confidence: f64,
}

/// Destination
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Destination {
    pub x: f64,
    pub y: f64,
    pub name: Option<String>,
}

/// Motion data
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct MotionData {
    pub velocity: Velocity,
    pub max_velocity: Velocity,
    pub acceleration: Velocity,
    pub distance_traveled_m: f64,
    pub heading_deg: f64,
}

/// Sensor data
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct MobilitySensors {
    pub lidar: Option<LidarData>,
    pub ultrasonic: Vec<UltrasonicSensor>,
    pub cameras: Vec<CameraSensor>,
    pub cliff_sensors: Vec<bool>,
    pub bump_sensors: Vec<bool>,
}

/// LiDAR data
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct LidarData {
    pub range_m: f64,
    pub fov_deg: f64,
    pub obstacle_detected: bool,
    pub closest_obstacle_m: Option<f64>,
    pub obstacle_direction_deg: Option<f64>,
}

/// Ultrasonic sensor
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UltrasonicSensor {
    pub id: String,
    pub location: String,
    pub distance_m: f64,
    pub obstacle_detected: bool,
}

/// Camera sensor
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraSensor {
    pub id: String,
    pub location: String,
    pub resolution: String,
    pub frame_rate_fps: u32,
    pub depth_enabled: bool,
    pub objects_detected: u32,
}

/// User interface
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct UserInterface {
    pub input_method: InputMethod,
    pub command: Option<Command>,
    pub override_active: bool,
    pub input_sensitivity: f64,
}

/// Command
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Command {
    pub command_type: String,
    pub value: f64,
}

/// Seating data
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct SeatingData {
    pub tilt_angle_deg: f64,
    pub recline_angle_deg: f64,
    pub seat_elevation_cm: f64,
    pub leg_rest_angle_deg: f64,
    pub armrest_position: ArmrestPosition,
    pub pressure_relief_active: bool,
}

/// Environment data
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct EnvironmentData {
    pub indoor: bool,
    pub surface_type: SurfaceType,
    pub incline_deg: f64,
    pub detected_obstacles: u32,
}

/// Mobility aid specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MobilityAidSpec {
    pub mobility_type: MobilityType,
    pub autonomous_mode: AutonomousMode,
    pub navigation: MobilityNavigation,
    pub motion: MotionData,
    pub sensors: MobilitySensors,
    pub user_interface: UserInterface,
    pub seating: SeatingData,
    pub environment: EnvironmentData,
}

impl Default for MobilityAidSpec {
    fn default() -> Self {
        Self {
            mobility_type: MobilityType::PoweredWheelchair,
            autonomous_mode: AutonomousMode::Manual,
            navigation: MobilityNavigation {
                localization_confidence: 1.0,
                ..Default::default()
            },
            motion: MotionData {
                max_velocity: Velocity {
                    linear: 1.5,
                    angular: 0.8,
                },
                ..Default::default()
            },
            sensors: MobilitySensors::default(),
            user_interface: UserInterface {
                input_sensitivity: 0.7,
                ..Default::default()
            },
            seating: SeatingData::default(),
            environment: EnvironmentData {
                indoor: true,
                ..Default::default()
            },
        }
    }
}

impl MobilityAidSpec {
    /// Create a new powered wheelchair
    pub fn new_wheelchair() -> Self {
        Self::default()
    }

    /// Set destination for navigation
    pub fn set_destination(&mut self, x: f64, y: f64, name: Option<String>) {
        self.navigation.destination = Some(Destination { x, y, name });
        self.navigation.path_status = PathStatus::Planning;
    }

    /// Start navigation to destination
    pub fn start_navigation(&mut self) -> RobotResult<()> {
        if self.navigation.destination.is_none() {
            return Err(RobotError::InvalidParameter(
                "No destination set".into(),
            ));
        }

        if self.autonomous_mode == AutonomousMode::Manual {
            return Err(RobotError::ControlError(
                "Cannot navigate in manual mode".into(),
            ));
        }

        self.navigation.path_status = PathStatus::Navigating;
        self.calculate_distance_to_goal();
        Ok(())
    }

    /// Calculate distance to goal
    fn calculate_distance_to_goal(&mut self) {
        if let Some(dest) = &self.navigation.destination {
            let dx = dest.x - self.navigation.current_pose.x;
            let dy = dest.y - self.navigation.current_pose.y;
            self.navigation.distance_to_goal_m = Some((dx * dx + dy * dy).sqrt());

            // Estimate ETA based on max velocity
            if let Some(distance) = self.navigation.distance_to_goal_m {
                if self.motion.max_velocity.linear > 0.0 {
                    self.navigation.eta_seconds =
                        Some((distance / self.motion.max_velocity.linear) as u32);
                }
            }
        }
    }

    /// Stop navigation
    pub fn stop_navigation(&mut self) {
        self.navigation.path_status = PathStatus::Idle;
        self.motion.velocity = Velocity::default();
    }

    /// Set velocity with safety limits
    pub fn set_velocity(&mut self, linear: f64, angular: f64) -> RobotResult<()> {
        if linear.abs() > self.motion.max_velocity.linear {
            return Err(RobotError::SafetyViolation(format!(
                "Linear velocity {:.2} exceeds max {:.2}",
                linear.abs(),
                self.motion.max_velocity.linear
            )));
        }

        if angular.abs() > self.motion.max_velocity.angular {
            return Err(RobotError::SafetyViolation(format!(
                "Angular velocity {:.2} exceeds max {:.2}",
                angular.abs(),
                self.motion.max_velocity.angular
            )));
        }

        // Check for obstacles
        if self.has_obstacle_ahead() && linear > 0.0 {
            return Err(RobotError::SafetyViolation(
                "Obstacle detected ahead".into(),
            ));
        }

        self.motion.velocity = Velocity { linear, angular };
        Ok(())
    }

    /// Check if there's an obstacle ahead
    pub fn has_obstacle_ahead(&self) -> bool {
        if let Some(lidar) = &self.sensors.lidar {
            if lidar.obstacle_detected {
                if let Some(distance) = lidar.closest_obstacle_m {
                    return distance < 0.5; // 50cm safety margin
                }
            }
        }

        self.sensors.bump_sensors.iter().any(|&b| b)
    }

    /// Check for cliff danger
    pub fn has_cliff_danger(&self) -> bool {
        self.sensors.cliff_sensors.iter().any(|&c| c)
    }

    /// Emergency stop
    pub fn emergency_stop(&mut self) {
        self.motion.velocity = Velocity::default();
        self.navigation.path_status = PathStatus::Blocked;
    }

    /// Adjust seating for pressure relief
    pub fn pressure_relief_cycle(&mut self) {
        self.seating.pressure_relief_active = true;
        self.seating.tilt_angle_deg = 15.0;
    }

    /// Reset seating to default
    pub fn reset_seating(&mut self) {
        self.seating = SeatingData::default();
    }

    /// Calculate total distance traveled
    pub fn update_odometry(&mut self, dt: f64) {
        let distance = self.motion.velocity.linear.abs() * dt;
        self.motion.distance_traveled_m += distance;
        self.motion.heading_deg += self.motion.velocity.angular.to_degrees() * dt;
        self.motion.heading_deg %= 360.0;
    }

    /// Check if arrived at destination
    pub fn check_arrival(&mut self) -> bool {
        if let Some(distance) = self.navigation.distance_to_goal_m {
            if distance < 0.3 {
                // 30cm arrival threshold
                self.navigation.path_status = PathStatus::Arrived;
                return true;
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_wheelchair() {
        let wheelchair = MobilityAidSpec::new_wheelchair();
        assert_eq!(wheelchair.mobility_type, MobilityType::PoweredWheelchair);
        assert_eq!(wheelchair.autonomous_mode, AutonomousMode::Manual);
    }

    #[test]
    fn test_set_destination() {
        let mut wheelchair = MobilityAidSpec::new_wheelchair();
        wheelchair.set_destination(10.0, 5.0, Some("Kitchen".to_string()));
        assert!(wheelchair.navigation.destination.is_some());
        assert_eq!(wheelchair.navigation.path_status, PathStatus::Planning);
    }

    #[test]
    fn test_start_navigation_manual_mode() {
        let mut wheelchair = MobilityAidSpec::new_wheelchair();
        wheelchair.set_destination(10.0, 5.0, None);
        assert!(wheelchair.start_navigation().is_err());
    }

    #[test]
    fn test_set_velocity_safety() {
        let mut wheelchair = MobilityAidSpec::new_wheelchair();
        assert!(wheelchair.set_velocity(1.0, 0.5).is_ok());
        assert!(wheelchair.set_velocity(5.0, 0.5).is_err());
    }

    #[test]
    fn test_emergency_stop() {
        let mut wheelchair = MobilityAidSpec::new_wheelchair();
        wheelchair.motion.velocity = Velocity {
            linear: 1.0,
            angular: 0.5,
        };
        wheelchair.emergency_stop();
        assert_eq!(wheelchair.motion.velocity.linear, 0.0);
        assert_eq!(wheelchair.navigation.path_status, PathStatus::Blocked);
    }

    #[test]
    fn test_update_odometry() {
        let mut wheelchair = MobilityAidSpec::new_wheelchair();
        wheelchair.motion.velocity = Velocity {
            linear: 1.0,
            angular: 0.0,
        };
        wheelchair.update_odometry(1.0);
        assert!((wheelchair.motion.distance_traveled_m - 1.0).abs() < 1e-10);
    }
}
