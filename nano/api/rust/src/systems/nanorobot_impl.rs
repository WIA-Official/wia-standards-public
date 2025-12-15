//! Nanorobot implementation

use crate::error::{NanoError, NanoResult};
use crate::types::*;
use crate::traits::*;
use async_trait::async_trait;

/// Standard nanorobot implementation
pub struct StandardNanorobot {
    id: String,
    status: SystemStatus,
    environment: Option<Environment>,
    scale: Option<Scale>,
    position: Position3D,
    orientation: Orientation,
    velocity: Velocity3D,
    propulsion: PropulsionSystem,
    navigation: NavigationSystem,
    mission: Option<Mission>,
    mission_status: Option<MissionStatus>,
    sensor_readings: Vec<SensorReading>,
    energy_available: f64,
    power_consumption: f64,
}

impl StandardNanorobot {
    /// Create a new nanorobot
    pub fn new(id: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            status: SystemStatus::Offline,
            environment: None,
            scale: Some(Scale::default()),
            position: Position3D::origin(),
            orientation: Orientation::identity(),
            velocity: Velocity3D::zero(),
            propulsion: PropulsionSystem {
                propulsion_type: PropulsionType::Flagellar,
                max_velocity: 100.0,     // 100 nm/s
                max_acceleration: 50.0,  // 50 nm/s²
                power_consumption: 100.0, // 100 fW
                efficiency: 0.3,
            },
            navigation: NavigationSystem {
                positioning_method: PositioningMethod::ChemicalGradient,
                accuracy_nm: 10.0,
                update_rate_hz: 100.0,
                obstacle_detection: true,
                path_planning: true,
            },
            mission: None,
            mission_status: None,
            sensor_readings: Vec::new(),
            energy_available: 10_000_000.0, // 10 pJ
            power_consumption: 100.0,       // 100 fW idle
        }
    }

    /// Create with builder
    pub fn builder(id: impl Into<String>) -> NanorobotBuilder {
        NanorobotBuilder::new(id)
    }

    fn consume_energy(&mut self, amount: f64) -> NanoResult<()> {
        if self.energy_available < amount {
            return Err(NanoError::InsufficientEnergy {
                required: amount,
                available: self.energy_available,
            });
        }
        self.energy_available -= amount;
        Ok(())
    }
}

/// Builder for StandardNanorobot
pub struct NanorobotBuilder {
    id: String,
    position: Position3D,
    environment: Option<Environment>,
    propulsion_type: PropulsionType,
    initial_energy: f64,
}

impl NanorobotBuilder {
    pub fn new(id: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            position: Position3D::origin(),
            environment: None,
            propulsion_type: PropulsionType::Flagellar,
            initial_energy: 10_000_000.0,
        }
    }

    pub fn with_position(mut self, pos: Position3D) -> Self {
        self.position = pos;
        self
    }

    pub fn with_environment(mut self, env: Environment) -> Self {
        self.environment = Some(env);
        self
    }

    pub fn with_propulsion(mut self, propulsion_type: PropulsionType) -> Self {
        self.propulsion_type = propulsion_type;
        self
    }

    pub fn with_energy(mut self, energy_fj: f64) -> Self {
        self.initial_energy = energy_fj;
        self
    }

    pub fn build(self) -> StandardNanorobot {
        let mut robot = StandardNanorobot::new(self.id);
        robot.position = self.position;
        robot.environment = self.environment;
        robot.propulsion.propulsion_type = self.propulsion_type;
        robot.energy_available = self.initial_energy;
        robot
    }
}

#[async_trait]
impl NanoSystem for StandardNanorobot {
    fn system_type(&self) -> NanoSystemType {
        NanoSystemType::Nanorobot
    }

    fn id(&self) -> &str {
        &self.id
    }

    fn status(&self) -> SystemStatus {
        self.status
    }

    async fn initialize(&mut self) -> NanoResult<()> {
        self.status = SystemStatus::Initializing;
        self.status = SystemStatus::Idle;
        Ok(())
    }

    async fn shutdown(&mut self) -> NanoResult<()> {
        self.status = SystemStatus::ShuttingDown;
        self.velocity = Velocity3D::zero();
        self.mission = None;
        self.mission_status = None;
        self.status = SystemStatus::Offline;
        Ok(())
    }

    async fn reset(&mut self) -> NanoResult<()> {
        self.position = Position3D::origin();
        self.orientation = Orientation::identity();
        self.velocity = Velocity3D::zero();
        self.mission = None;
        self.mission_status = None;
        self.sensor_readings.clear();
        self.status = SystemStatus::Idle;
        Ok(())
    }

    fn environment(&self) -> Option<&Environment> {
        self.environment.as_ref()
    }

    fn set_environment(&mut self, env: Environment) {
        self.environment = Some(env);
    }

    fn scale(&self) -> Option<&Scale> {
        self.scale.as_ref()
    }

    fn to_message(&self) -> NanoResult<NanoMessage> {
        let payload = serde_json::json!({
            "system_type": "nanorobot",
            "position": {
                "x": self.position.x,
                "y": self.position.y,
                "z": self.position.z
            },
            "velocity": {
                "vx": self.velocity.vx,
                "vy": self.velocity.vy,
                "vz": self.velocity.vz
            },
            "propulsion_type": format!("{:?}", self.propulsion.propulsion_type),
            "energy_available_fj": self.energy_available
        });

        Ok(NanoMessage::builder()
            .device_id(&self.id)
            .system_type(NanoSystemType::Nanorobot)
            .payload(payload)?)
    }

    fn energy_level(&self) -> Option<f64> {
        Some(self.energy_available)
    }
}

#[async_trait]
impl Nanorobot for StandardNanorobot {
    fn position(&self) -> Position3D {
        self.position
    }

    fn orientation(&self) -> Orientation {
        self.orientation.clone()
    }

    fn velocity(&self) -> Velocity3D {
        self.velocity
    }

    async fn move_to(&mut self, target: Position3D) -> NanoResult<()> {
        if !self.is_operational() {
            return Err(NanoError::InvalidParameter("System not operational".into()));
        }

        let distance = self.position.distance_to(&target);
        let travel_time = distance / self.propulsion.max_velocity;
        let energy_cost = travel_time * self.propulsion.power_consumption;

        self.consume_energy(energy_cost)?;

        self.status = SystemStatus::Active;
        self.position = target;
        self.status = SystemStatus::Idle;

        Ok(())
    }

    async fn move_by(&mut self, offset: Position3D) -> NanoResult<()> {
        let target = Position3D::new(
            self.position.x + offset.x,
            self.position.y + offset.y,
            self.position.z + offset.z,
        );
        self.move_to(target).await
    }

    async fn rotate_to(&mut self, target: Orientation) -> NanoResult<()> {
        let energy_cost = 1.0; // Small energy for rotation
        self.consume_energy(energy_cost)?;
        self.orientation = target;
        Ok(())
    }

    async fn stop(&mut self) -> NanoResult<()> {
        self.velocity = Velocity3D::zero();
        Ok(())
    }

    fn propulsion(&self) -> &PropulsionSystem {
        &self.propulsion
    }

    fn navigation(&self) -> &NavigationSystem {
        &self.navigation
    }

    async fn start_mission(&mut self, mission: Mission) -> NanoResult<String> {
        if !self.is_operational() {
            return Err(NanoError::InvalidParameter("System not operational".into()));
        }

        let mission_id = mission.id.clone();

        let status = MissionStatus {
            mission_id: mission_id.clone(),
            state: MissionState::Active,
            current_waypoint: 0,
            total_waypoints: mission.waypoints.len(),
            distance_traveled: 0.0,
            elapsed_time_ms: 0,
            errors: Vec::new(),
        };

        self.mission = Some(mission);
        self.mission_status = Some(status);
        self.status = SystemStatus::Active;

        Ok(mission_id)
    }

    async fn abort_mission(&mut self) -> NanoResult<()> {
        if let Some(ref mut status) = self.mission_status {
            status.state = MissionState::Aborted;
        }
        self.mission = None;
        self.status = SystemStatus::Idle;
        Ok(())
    }

    fn mission_status(&self) -> Option<&MissionStatus> {
        self.mission_status.as_ref()
    }

    fn sensor_readings(&self) -> Vec<SensorReading> {
        self.sensor_readings.clone()
    }

    async fn execute_action(&mut self, action: RobotAction) -> NanoResult<ActionResult> {
        let start = std::time::Instant::now();
        let action_type = format!("{:?}", action);

        let energy_cost = match &action {
            RobotAction::Grip { force_pn } => force_pn * 0.1,
            RobotAction::Release => 0.5,
            RobotAction::Inject { volume_fl } => volume_fl * 0.01,
            RobotAction::Sample { volume_fl } => volume_fl * 0.01,
            RobotAction::Signal { intensity, .. } => intensity * 1.0,
            RobotAction::Scan { range_nm } => range_nm * 0.001,
            RobotAction::Custom { .. } => 10.0,
        };

        self.consume_energy(energy_cost)?;

        Ok(ActionResult {
            success: true,
            action_type,
            duration_ms: start.elapsed().as_millis() as u64,
            energy_consumed_fj: energy_cost,
            data: None,
            error: None,
        })
    }
}

impl PowerManaged for StandardNanorobot {
    fn power_consumption(&self) -> f64 {
        self.power_consumption
    }

    fn remaining_energy(&self) -> f64 {
        self.energy_available
    }

    fn enter_low_power_mode(&mut self) -> NanoResult<()> {
        self.status = SystemStatus::LowPower;
        self.power_consumption = 10.0; // Reduce to 10 fW
        Ok(())
    }

    fn exit_low_power_mode(&mut self) -> NanoResult<()> {
        self.status = SystemStatus::Idle;
        self.power_consumption = 100.0;
        Ok(())
    }
}
