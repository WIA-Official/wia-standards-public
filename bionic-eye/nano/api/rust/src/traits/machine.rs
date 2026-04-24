//! Nano machine trait definitions

use crate::error::NanoResult;
use crate::types::Position3D;
use async_trait::async_trait;

/// Trait for general nano machines
#[async_trait]
pub trait NanoMachine: Send + Sync {
    /// Get machine ID
    fn machine_id(&self) -> &str;

    /// Get machine type
    fn machine_type(&self) -> MachineType;

    /// Get current operational state
    fn state(&self) -> MachineState;

    /// Start the machine
    async fn start(&mut self) -> NanoResult<()>;

    /// Stop the machine
    async fn stop(&mut self) -> NanoResult<()>;

    /// Get operating parameters
    fn parameters(&self) -> &OperatingParameters;

    /// Set operating parameters
    fn set_parameters(&mut self, params: OperatingParameters);

    /// Get kinematics info
    fn kinematics(&self) -> Option<&Kinematics>;

    /// Get diagnostic info
    fn diagnostics(&self) -> MachineDiagnostics;

    /// Execute a machine cycle
    async fn execute_cycle(&mut self) -> NanoResult<CycleResult>;
}

/// Type of nano machine
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MachineType {
    /// Rotary molecular motor
    RotaryMotor,
    /// Linear molecular motor
    LinearMotor,
    /// Molecular pump
    Pump,
    /// Molecular valve
    Valve,
    /// Molecular switch
    Switch,
    /// Molecular gear
    Gear,
    /// Molecular ratchet
    Ratchet,
    /// Generic actuator
    Actuator,
    /// Custom machine
    Custom,
}

/// Operational state of machine
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MachineState {
    Idle,
    Running,
    Paused,
    Error,
    Maintenance,
}

/// Operating parameters for nano machines
#[derive(Debug, Clone)]
pub struct OperatingParameters {
    pub speed: Option<f64>,           // depends on machine type
    pub force: Option<f64>,           // pN
    pub torque: Option<f64>,          // pN·nm
    pub power: Option<f64>,           // fW
    pub duty_cycle: Option<f64>,      // 0.0-1.0
    pub direction: Option<Direction>,
}

impl Default for OperatingParameters {
    fn default() -> Self {
        Self {
            speed: None,
            force: None,
            torque: None,
            power: None,
            duty_cycle: Some(1.0),
            direction: Some(Direction::Forward),
        }
    }
}

/// Direction of operation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Reverse,
    Clockwise,
    CounterClockwise,
}

/// Kinematics information
#[derive(Debug, Clone)]
pub struct Kinematics {
    pub degrees_of_freedom: u8,
    pub range_of_motion: Vec<MotionRange>,
    pub current_position: MachinePosition,
    pub velocity: MachineVelocity,
}

/// Range of motion for a degree of freedom
#[derive(Debug, Clone)]
pub struct MotionRange {
    pub axis: String,
    pub min: f64,
    pub max: f64,
    pub unit: String,
}

/// Machine position (generalized)
#[derive(Debug, Clone)]
pub struct MachinePosition {
    pub linear: Option<Position3D>,    // nm
    pub angular: Option<f64>,          // radians
    pub extension: Option<f64>,        // nm (for linear actuators)
}

/// Machine velocity (generalized)
#[derive(Debug, Clone)]
pub struct MachineVelocity {
    pub linear_speed: Option<f64>,     // nm/s
    pub angular_speed: Option<f64>,    // rad/s
    pub extension_rate: Option<f64>,   // nm/s
}

/// Machine diagnostics
#[derive(Debug, Clone)]
pub struct MachineDiagnostics {
    pub cycles_completed: u64,
    pub total_runtime_ms: u64,
    pub error_count: u32,
    pub last_error: Option<String>,
    pub efficiency: f64,
    pub wear_level: f64,  // 0.0 = new, 1.0 = worn out
}

/// Result of a machine cycle
#[derive(Debug, Clone)]
pub struct CycleResult {
    pub success: bool,
    pub duration_ms: u64,
    pub energy_consumed_fj: f64,
    pub work_done_fj: Option<f64>,
    pub error: Option<String>,
}

/// Rotary motor specific trait
#[async_trait]
pub trait RotaryMotor: NanoMachine {
    /// Get rotation speed in Hz
    fn rotation_speed(&self) -> f64;

    /// Set target rotation speed
    async fn set_rotation_speed(&mut self, hz: f64) -> NanoResult<()>;

    /// Get torque in pN·nm
    fn torque(&self) -> f64;

    /// Get current angle in radians
    fn current_angle(&self) -> f64;

    /// Rotate to specific angle
    async fn rotate_to(&mut self, angle_rad: f64) -> NanoResult<()>;

    /// Rotate by delta angle
    async fn rotate_by(&mut self, delta_rad: f64) -> NanoResult<()>;

    /// Get step size (for stepping motors)
    fn step_size(&self) -> Option<f64>;
}

/// Linear motor specific trait
#[async_trait]
pub trait LinearMotor: NanoMachine {
    /// Get linear speed in nm/s
    fn linear_speed(&self) -> f64;

    /// Set target speed
    async fn set_linear_speed(&mut self, nm_per_s: f64) -> NanoResult<()>;

    /// Get force in pN
    fn force(&self) -> f64;

    /// Get current extension in nm
    fn extension(&self) -> f64;

    /// Move to position
    async fn move_to(&mut self, position_nm: f64) -> NanoResult<()>;

    /// Move by delta
    async fn move_by(&mut self, delta_nm: f64) -> NanoResult<()>;

    /// Get stroke length
    fn stroke_length(&self) -> f64;
}

/// Molecular pump trait
#[async_trait]
pub trait MolecularPump: NanoMachine {
    /// Get flow rate in molecules/s
    fn flow_rate(&self) -> f64;

    /// Set target flow rate
    async fn set_flow_rate(&mut self, rate: f64) -> NanoResult<()>;

    /// Get pressure differential in pN/nm²
    fn pressure_differential(&self) -> f64;

    /// Get transported molecule type
    fn transported_species(&self) -> &str;

    /// Get total molecules transported
    fn molecules_transported(&self) -> u64;
}
