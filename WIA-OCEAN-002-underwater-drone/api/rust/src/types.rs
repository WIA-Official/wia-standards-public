//! Type definitions for underwater drones

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};
use uuid::Uuid;

/// Underwater drone specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UnderwaterDrone {
    pub id: Uuid,
    pub name: String,
    pub drone_type: DroneType,
    pub max_depth_meters: f64,
    pub battery_capacity_kwh: f64,
    pub max_speed_knots: f64,
    pub sensors: Vec<Sensor>,
    pub status: DroneStatus,
}

/// Types of underwater drones
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum DroneType {
    Auv,  // Autonomous Underwater Vehicle
    Rov,  // Remotely Operated Vehicle
    Glider,
    Hybrid,
}

/// Drone operational status
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum DroneStatus {
    Idle,
    Active,
    Deploying,
    Recovering,
    Maintenance,
    Emergency,
}

/// Sensor types
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum Sensor {
    Camera,
    Sonar,
    Lidar,
    CtdSensor,     // Conductivity, Temperature, Depth
    ChemicalSensor,
    Magnetometer,
    Accelerometer,
}

/// Mission waypoint
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Waypoint {
    pub latitude: f64,
    pub longitude: f64,
    pub depth_meters: f64,
    pub duration_seconds: u64,
    pub actions: Vec<WaypointAction>,
}

/// Actions to perform at waypoint
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum WaypointAction {
    TakePhoto,
    CollectSample,
    RecordVideo,
    ScanSonar,
    MeasureTemperature,
    Hover,
}

/// Drone telemetry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Telemetry {
    pub drone_id: Uuid,
    pub timestamp: DateTime<Utc>,
    pub latitude: f64,
    pub longitude: f64,
    pub depth_meters: f64,
    pub heading_degrees: f64,
    pub speed_knots: f64,
    pub battery_percent: f64,
    pub temperature_celsius: f64,
}

/// Navigation command
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavigationCommand {
    pub command_id: Uuid,
    pub drone_id: Uuid,
    pub waypoints: Vec<Waypoint>,
    pub max_speed_knots: f64,
    pub emergency_surface_depth: f64,
}
