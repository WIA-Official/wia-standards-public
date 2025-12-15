//! Nanorobot trait definitions

use crate::error::NanoResult;
use crate::types::{Position3D, Orientation, SensorReading};
use async_trait::async_trait;

/// Trait for nanorobot navigation and control
#[async_trait]
pub trait Nanorobot: Send + Sync {
    /// Get current position
    fn position(&self) -> Position3D;

    /// Get current orientation
    fn orientation(&self) -> Orientation;

    /// Get current velocity (nm/s)
    fn velocity(&self) -> Velocity3D;

    /// Move to a target position
    async fn move_to(&mut self, target: Position3D) -> NanoResult<()>;

    /// Move by a relative offset
    async fn move_by(&mut self, offset: Position3D) -> NanoResult<()>;

    /// Rotate to a target orientation
    async fn rotate_to(&mut self, target: Orientation) -> NanoResult<()>;

    /// Stop all movement
    async fn stop(&mut self) -> NanoResult<()>;

    /// Get propulsion system info
    fn propulsion(&self) -> &PropulsionSystem;

    /// Get navigation capabilities
    fn navigation(&self) -> &NavigationSystem;

    /// Start a mission
    async fn start_mission(&mut self, mission: Mission) -> NanoResult<String>;

    /// Abort current mission
    async fn abort_mission(&mut self) -> NanoResult<()>;

    /// Get current mission status
    fn mission_status(&self) -> Option<&MissionStatus>;

    /// Get all sensor readings
    fn sensor_readings(&self) -> Vec<SensorReading>;

    /// Execute an action
    async fn execute_action(&mut self, action: RobotAction) -> NanoResult<ActionResult>;
}

/// 3D Velocity vector
#[derive(Debug, Clone, Copy, Default)]
pub struct Velocity3D {
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
}

impl Velocity3D {
    pub fn new(vx: f64, vy: f64, vz: f64) -> Self {
        Self { vx, vy, vz }
    }

    pub fn magnitude(&self) -> f64 {
        (self.vx * self.vx + self.vy * self.vy + self.vz * self.vz).sqrt()
    }

    pub fn zero() -> Self {
        Self::default()
    }
}

/// Propulsion system configuration
#[derive(Debug, Clone)]
pub struct PropulsionSystem {
    pub propulsion_type: PropulsionType,
    pub max_velocity: f64,      // nm/s
    pub max_acceleration: f64,  // nm/s²
    pub power_consumption: f64, // fW
    pub efficiency: f64,        // 0.0-1.0
}

/// Type of propulsion mechanism
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PropulsionType {
    /// Flagella-based propulsion
    Flagellar,
    /// Chemical gradient following
    Chemotaxis,
    /// Magnetic field driven
    Magnetic,
    /// Acoustic wave driven
    Acoustic,
    /// Light-driven (optical tweezers)
    Optical,
    /// Electric field driven
    Electrophoretic,
    /// Catalytic reaction driven
    Catalytic,
    /// Hybrid propulsion
    Hybrid,
}

/// Navigation system capabilities
#[derive(Debug, Clone)]
pub struct NavigationSystem {
    pub positioning_method: PositioningMethod,
    pub accuracy_nm: f64,
    pub update_rate_hz: f64,
    pub obstacle_detection: bool,
    pub path_planning: bool,
}

/// Positioning/localization method
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PositioningMethod {
    /// GPS-like external tracking
    ExternalTracking,
    /// Fluorescence-based localization
    Fluorescence,
    /// Magnetic field mapping
    MagneticMapping,
    /// Chemical gradient sensing
    ChemicalGradient,
    /// Ultrasonic triangulation
    Ultrasonic,
    /// Dead reckoning
    DeadReckoning,
}

/// Mission definition
#[derive(Debug, Clone)]
pub struct Mission {
    pub id: String,
    pub name: String,
    pub mission_type: MissionType,
    pub waypoints: Vec<Waypoint>,
    pub parameters: MissionParameters,
    pub timeout_ms: Option<u64>,
}

/// Type of mission
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissionType {
    /// Navigate to a target
    Navigation,
    /// Search for a target
    Search,
    /// Deliver payload
    Delivery,
    /// Sample collection
    Sampling,
    /// Area surveillance
    Surveillance,
    /// Repair/maintenance task
    Repair,
    /// Custom mission
    Custom,
}

/// Navigation waypoint
#[derive(Debug, Clone)]
pub struct Waypoint {
    pub position: Position3D,
    pub action: Option<WaypointAction>,
    pub tolerance_nm: f64,
    pub dwell_time_ms: Option<u64>,
}

/// Action to perform at waypoint
#[derive(Debug, Clone)]
pub enum WaypointAction {
    Stop,
    Sample,
    Release,
    Scan,
    Signal,
    Custom(String),
}

/// Mission parameters
#[derive(Debug, Clone, Default)]
pub struct MissionParameters {
    pub max_velocity: Option<f64>,
    pub collision_avoidance: bool,
    pub energy_conservation: bool,
    pub logging_enabled: bool,
}

/// Current mission status
#[derive(Debug, Clone)]
pub struct MissionStatus {
    pub mission_id: String,
    pub state: MissionState,
    pub current_waypoint: usize,
    pub total_waypoints: usize,
    pub distance_traveled: f64,
    pub elapsed_time_ms: u64,
    pub errors: Vec<String>,
}

/// Mission execution state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissionState {
    Pending,
    Active,
    Paused,
    Completed,
    Failed,
    Aborted,
}

/// Robot action types
#[derive(Debug, Clone)]
pub enum RobotAction {
    /// Grip/grasp an object
    Grip { force_pn: f64 },
    /// Release gripped object
    Release,
    /// Inject payload
    Inject { volume_fl: f64 },
    /// Collect sample
    Sample { volume_fl: f64 },
    /// Emit signal
    Signal { signal_type: String, intensity: f64 },
    /// Scan surroundings
    Scan { range_nm: f64 },
    /// Custom action
    Custom { name: String, params: serde_json::Value },
}

/// Result of an action
#[derive(Debug, Clone)]
pub struct ActionResult {
    pub success: bool,
    pub action_type: String,
    pub duration_ms: u64,
    pub energy_consumed_fj: f64,
    pub data: Option<serde_json::Value>,
    pub error: Option<String>,
}

/// Swarm coordination trait
#[async_trait]
pub trait SwarmMember: Nanorobot {
    /// Get swarm ID this robot belongs to
    fn swarm_id(&self) -> Option<&str>;

    /// Join a swarm
    async fn join_swarm(&mut self, swarm_id: &str) -> NanoResult<()>;

    /// Leave current swarm
    async fn leave_swarm(&mut self) -> NanoResult<()>;

    /// Broadcast message to swarm
    async fn broadcast(&mut self, message: SwarmMessage) -> NanoResult<()>;

    /// Get nearby swarm members
    fn nearby_members(&self) -> Vec<SwarmMemberInfo>;

    /// Get swarm coordination role
    fn swarm_role(&self) -> SwarmRole;
}

/// Message for swarm communication
#[derive(Debug, Clone)]
pub struct SwarmMessage {
    pub message_type: String,
    pub sender_id: String,
    pub payload: serde_json::Value,
    pub priority: u8,
}

/// Information about a swarm member
#[derive(Debug, Clone)]
pub struct SwarmMemberInfo {
    pub id: String,
    pub position: Position3D,
    pub role: SwarmRole,
    pub status: String,
}

/// Role in swarm coordination
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SwarmRole {
    Leader,
    Worker,
    Scout,
    Relay,
    Standby,
}
