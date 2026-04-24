//! Nano machine implementation

use crate::error::{NanoError, NanoResult};
use crate::types::*;
use crate::traits::*;
use async_trait::async_trait;

/// Standard nano machine implementation
pub struct StandardNanoMachine {
    id: String,
    machine_type: MachineType,
    status: SystemStatus,
    state: MachineState,
    environment: Option<Environment>,
    scale: Option<Scale>,
    parameters: OperatingParameters,
    kinematics: Option<Kinematics>,
    diagnostics: MachineDiagnostics,
    energy_available: f64,
}

impl StandardNanoMachine {
    /// Create a new nano machine
    pub fn new(id: impl Into<String>, machine_type: MachineType) -> Self {
        Self {
            id: id.into(),
            machine_type,
            status: SystemStatus::Offline,
            state: MachineState::Idle,
            environment: None,
            scale: Some(Scale::default()),
            parameters: OperatingParameters::default(),
            kinematics: None,
            diagnostics: MachineDiagnostics {
                cycles_completed: 0,
                total_runtime_ms: 0,
                error_count: 0,
                last_error: None,
                efficiency: 0.85,
                wear_level: 0.0,
            },
            energy_available: 1_000_000.0, // 1 pJ
        }
    }

    /// Create with builder
    pub fn builder(id: impl Into<String>, machine_type: MachineType) -> NanoMachineBuilder {
        NanoMachineBuilder::new(id, machine_type)
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

/// Builder for StandardNanoMachine
pub struct NanoMachineBuilder {
    id: String,
    machine_type: MachineType,
    environment: Option<Environment>,
    parameters: OperatingParameters,
    initial_energy: f64,
}

impl NanoMachineBuilder {
    pub fn new(id: impl Into<String>, machine_type: MachineType) -> Self {
        Self {
            id: id.into(),
            machine_type,
            environment: None,
            parameters: OperatingParameters::default(),
            initial_energy: 1_000_000.0,
        }
    }

    pub fn with_environment(mut self, env: Environment) -> Self {
        self.environment = Some(env);
        self
    }

    pub fn with_parameters(mut self, params: OperatingParameters) -> Self {
        self.parameters = params;
        self
    }

    pub fn with_energy(mut self, energy_fj: f64) -> Self {
        self.initial_energy = energy_fj;
        self
    }

    pub fn build(self) -> StandardNanoMachine {
        let mut machine = StandardNanoMachine::new(self.id, self.machine_type);
        machine.environment = self.environment;
        machine.parameters = self.parameters;
        machine.energy_available = self.initial_energy;
        machine
    }
}

#[async_trait]
impl NanoSystem for StandardNanoMachine {
    fn system_type(&self) -> NanoSystemType {
        NanoSystemType::NanoMachine
    }

    fn id(&self) -> &str {
        &self.id
    }

    fn status(&self) -> SystemStatus {
        self.status
    }

    async fn initialize(&mut self) -> NanoResult<()> {
        self.status = SystemStatus::Initializing;
        self.state = MachineState::Idle;
        self.status = SystemStatus::Idle;
        Ok(())
    }

    async fn shutdown(&mut self) -> NanoResult<()> {
        self.status = SystemStatus::ShuttingDown;
        self.state = MachineState::Idle;
        self.status = SystemStatus::Offline;
        Ok(())
    }

    async fn reset(&mut self) -> NanoResult<()> {
        self.state = MachineState::Idle;
        self.parameters = OperatingParameters::default();
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
            "system_type": "nano_machine",
            "machine_type": format!("{:?}", self.machine_type),
            "state": format!("{:?}", self.state),
            "cycles_completed": self.diagnostics.cycles_completed,
            "efficiency": self.diagnostics.efficiency,
            "energy_available_fj": self.energy_available
        });

        Ok(NanoMessage::builder()
            .device_id(&self.id)
            .system_type(NanoSystemType::NanoMachine)
            .payload(payload)?)
    }

    fn energy_level(&self) -> Option<f64> {
        Some(self.energy_available)
    }
}

#[async_trait]
impl NanoMachine for StandardNanoMachine {
    fn machine_id(&self) -> &str {
        &self.id
    }

    fn machine_type(&self) -> MachineType {
        self.machine_type
    }

    fn state(&self) -> MachineState {
        self.state
    }

    async fn start(&mut self) -> NanoResult<()> {
        if !self.is_operational() {
            return Err(NanoError::InvalidParameter("Machine not operational".into()));
        }

        self.state = MachineState::Running;
        self.status = SystemStatus::Active;
        Ok(())
    }

    async fn stop(&mut self) -> NanoResult<()> {
        self.state = MachineState::Idle;
        self.status = SystemStatus::Idle;
        Ok(())
    }

    fn parameters(&self) -> &OperatingParameters {
        &self.parameters
    }

    fn set_parameters(&mut self, params: OperatingParameters) {
        self.parameters = params;
    }

    fn kinematics(&self) -> Option<&Kinematics> {
        self.kinematics.as_ref()
    }

    fn diagnostics(&self) -> MachineDiagnostics {
        self.diagnostics.clone()
    }

    async fn execute_cycle(&mut self) -> NanoResult<CycleResult> {
        if self.state != MachineState::Running {
            return Err(NanoError::InvalidParameter("Machine not running".into()));
        }

        let start = std::time::Instant::now();

        // Energy consumption per cycle
        let energy_cost = self.parameters.power.unwrap_or(10.0) *
                         self.parameters.duty_cycle.unwrap_or(1.0);
        self.consume_energy(energy_cost)?;

        self.diagnostics.cycles_completed += 1;
        let duration = start.elapsed().as_millis() as u64;
        self.diagnostics.total_runtime_ms += duration;

        // Simulate wear
        self.diagnostics.wear_level += 0.00001;

        Ok(CycleResult {
            success: true,
            duration_ms: duration,
            energy_consumed_fj: energy_cost,
            work_done_fj: Some(energy_cost * self.diagnostics.efficiency),
            error: None,
        })
    }
}

/// Molecular motor implementation
pub struct MolecularMotor {
    inner: StandardNanoMachine,
    rotation_speed_hz: f64,
    current_angle_rad: f64,
    torque_pn_nm: f64,
    step_size_rad: Option<f64>,
}

impl MolecularMotor {
    pub fn new_rotary(id: impl Into<String>) -> Self {
        Self {
            inner: StandardNanoMachine::new(id, MachineType::RotaryMotor),
            rotation_speed_hz: 0.0,
            current_angle_rad: 0.0,
            torque_pn_nm: 40.0, // Typical F1-ATPase torque
            step_size_rad: Some(std::f64::consts::PI * 2.0 / 3.0), // 120° step
        }
    }

    pub fn new_linear(id: impl Into<String>) -> Self {
        Self {
            inner: StandardNanoMachine::new(id, MachineType::LinearMotor),
            rotation_speed_hz: 0.0,
            current_angle_rad: 0.0,
            torque_pn_nm: 0.0,
            step_size_rad: None,
        }
    }
}

#[async_trait]
impl NanoSystem for MolecularMotor {
    fn system_type(&self) -> NanoSystemType {
        self.inner.system_type()
    }

    fn id(&self) -> &str {
        self.inner.id()
    }

    fn status(&self) -> SystemStatus {
        self.inner.status()
    }

    async fn initialize(&mut self) -> NanoResult<()> {
        self.inner.initialize().await
    }

    async fn shutdown(&mut self) -> NanoResult<()> {
        self.rotation_speed_hz = 0.0;
        self.inner.shutdown().await
    }

    async fn reset(&mut self) -> NanoResult<()> {
        self.rotation_speed_hz = 0.0;
        self.current_angle_rad = 0.0;
        self.inner.reset().await
    }

    fn environment(&self) -> Option<&Environment> {
        self.inner.environment()
    }

    fn set_environment(&mut self, env: Environment) {
        self.inner.set_environment(env)
    }

    fn scale(&self) -> Option<&Scale> {
        self.inner.scale()
    }

    fn to_message(&self) -> NanoResult<NanoMessage> {
        self.inner.to_message()
    }

    fn energy_level(&self) -> Option<f64> {
        self.inner.energy_level()
    }
}

#[async_trait]
impl NanoMachine for MolecularMotor {
    fn machine_id(&self) -> &str {
        self.inner.machine_id()
    }

    fn machine_type(&self) -> MachineType {
        self.inner.machine_type()
    }

    fn state(&self) -> MachineState {
        self.inner.state()
    }

    async fn start(&mut self) -> NanoResult<()> {
        self.inner.start().await
    }

    async fn stop(&mut self) -> NanoResult<()> {
        self.rotation_speed_hz = 0.0;
        self.inner.stop().await
    }

    fn parameters(&self) -> &OperatingParameters {
        self.inner.parameters()
    }

    fn set_parameters(&mut self, params: OperatingParameters) {
        self.inner.set_parameters(params)
    }

    fn kinematics(&self) -> Option<&Kinematics> {
        self.inner.kinematics()
    }

    fn diagnostics(&self) -> MachineDiagnostics {
        self.inner.diagnostics()
    }

    async fn execute_cycle(&mut self) -> NanoResult<CycleResult> {
        self.inner.execute_cycle().await
    }
}

#[async_trait]
impl RotaryMotor for MolecularMotor {
    fn rotation_speed(&self) -> f64 {
        self.rotation_speed_hz
    }

    async fn set_rotation_speed(&mut self, hz: f64) -> NanoResult<()> {
        self.rotation_speed_hz = hz;
        Ok(())
    }

    fn torque(&self) -> f64 {
        self.torque_pn_nm
    }

    fn current_angle(&self) -> f64 {
        self.current_angle_rad
    }

    async fn rotate_to(&mut self, angle_rad: f64) -> NanoResult<()> {
        self.current_angle_rad = angle_rad % (2.0 * std::f64::consts::PI);
        Ok(())
    }

    async fn rotate_by(&mut self, delta_rad: f64) -> NanoResult<()> {
        self.current_angle_rad = (self.current_angle_rad + delta_rad) %
                                 (2.0 * std::f64::consts::PI);
        Ok(())
    }

    fn step_size(&self) -> Option<f64> {
        self.step_size_rad
    }
}
