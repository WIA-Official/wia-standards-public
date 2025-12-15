//! Nano simulation engine

use crate::error::{NanoError, NanoResult};
use crate::types::*;
use crate::traits::*;
use crate::systems::*;
use super::world::{SimulationWorld, WorldBounds};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

/// Main simulation engine
pub struct NanoSimulator {
    world: SimulationWorld,
    nanorobots: HashMap<String, Arc<RwLock<StandardNanorobot>>>,
    assemblers: HashMap<String, Arc<RwLock<StandardAssembler>>>,
    sensors: HashMap<String, Arc<RwLock<StandardNanosensor>>>,
    machines: HashMap<String, Arc<RwLock<StandardNanoMachine>>>,
    memories: HashMap<String, Arc<RwLock<DnaMolecularMemory>>>,
    drug_carriers: HashMap<String, Arc<RwLock<StandardDrugDelivery>>>,
    time_ns: f64,
    running: bool,
}

impl NanoSimulator {
    /// Create a new simulator with default world
    pub fn new() -> Self {
        Self {
            world: SimulationWorld::new(),
            nanorobots: HashMap::new(),
            assemblers: HashMap::new(),
            sensors: HashMap::new(),
            machines: HashMap::new(),
            memories: HashMap::new(),
            drug_carriers: HashMap::new(),
            time_ns: 0.0,
            running: false,
        }
    }

    /// Create simulator with custom world
    pub fn with_world(world: SimulationWorld) -> Self {
        let mut sim = Self::new();
        sim.world = world;
        sim
    }

    /// Get current simulation time in nanoseconds
    pub fn time_ns(&self) -> f64 {
        self.time_ns
    }

    /// Get current simulation time in microseconds
    pub fn time_us(&self) -> f64 {
        self.time_ns / 1000.0
    }

    /// Get current simulation time in milliseconds
    pub fn time_ms(&self) -> f64 {
        self.time_ns / 1_000_000.0
    }

    /// Get world reference
    pub fn world(&self) -> &SimulationWorld {
        &self.world
    }

    /// Get mutable world reference
    pub fn world_mut(&mut self) -> &mut SimulationWorld {
        &mut self.world
    }

    // ==================== Nanorobot Management ====================

    /// Create a nanorobot builder
    pub fn create_nanorobot(&self, id: impl Into<String>) -> SimNanorobotBuilder {
        SimNanorobotBuilder::new(id, self.world.environment.clone())
    }

    /// Add a nanorobot to the simulation
    pub async fn add_nanorobot(&mut self, mut robot: StandardNanorobot) -> NanoResult<String> {
        let id = robot.id().to_string();

        // Validate position
        if let Some(scale) = robot.scale() {
            let pos = robot.position();
            if !self.world.bounds.contains(&pos) {
                return Err(NanoError::OutOfBounds {
                    position: [pos.x, pos.y, pos.z],
                    bounds: [
                        self.world.bounds.min.x, self.world.bounds.min.y, self.world.bounds.min.z,
                        self.world.bounds.max.x, self.world.bounds.max.y, self.world.bounds.max.z,
                    ],
                });
            }
        }

        // Initialize the robot
        robot.initialize().await?;

        self.nanorobots.insert(id.clone(), Arc::new(RwLock::new(robot)));
        Ok(id)
    }

    /// Get a nanorobot by ID
    pub fn get_nanorobot(&self, id: &str) -> Option<Arc<RwLock<StandardNanorobot>>> {
        self.nanorobots.get(id).cloned()
    }

    /// Remove a nanorobot from simulation
    pub async fn remove_nanorobot(&mut self, id: &str) -> NanoResult<()> {
        if let Some(robot) = self.nanorobots.remove(id) {
            let mut r = robot.write().await;
            r.shutdown().await?;
        }
        Ok(())
    }

    // ==================== Assembler Management ====================

    /// Create an assembler builder
    pub fn create_assembler(&self, id: impl Into<String>) -> SimAssemblerBuilder {
        SimAssemblerBuilder::new(id, self.world.environment.clone())
    }

    /// Add an assembler to the simulation
    pub async fn add_assembler(&mut self, mut assembler: StandardAssembler) -> NanoResult<String> {
        let id = assembler.id().to_string();
        assembler.initialize().await?;
        self.assemblers.insert(id.clone(), Arc::new(RwLock::new(assembler)));
        Ok(id)
    }

    /// Get an assembler by ID
    pub fn get_assembler(&self, id: &str) -> Option<Arc<RwLock<StandardAssembler>>> {
        self.assemblers.get(id).cloned()
    }

    // ==================== Sensor Management ====================

    /// Create a sensor builder
    pub fn create_sensor(&self, id: impl Into<String>, sensor_type: SensorType) -> SimSensorBuilder {
        SimSensorBuilder::new(id, sensor_type, self.world.environment.clone())
    }

    /// Add a sensor to the simulation
    pub async fn add_sensor(&mut self, mut sensor: StandardNanosensor) -> NanoResult<String> {
        let id = sensor.id().to_string();
        sensor.initialize().await?;
        self.sensors.insert(id.clone(), Arc::new(RwLock::new(sensor)));
        Ok(id)
    }

    /// Get a sensor by ID
    pub fn get_sensor(&self, id: &str) -> Option<Arc<RwLock<StandardNanosensor>>> {
        self.sensors.get(id).cloned()
    }

    // ==================== Drug Carrier Management ====================

    /// Create a drug delivery builder
    pub fn create_drug_carrier(&self, id: impl Into<String>, carrier_type: CarrierType) -> SimDrugDeliveryBuilder {
        SimDrugDeliveryBuilder::new(id, carrier_type, self.world.environment.clone())
    }

    /// Add a drug carrier to the simulation
    pub async fn add_drug_carrier(&mut self, mut carrier: StandardDrugDelivery) -> NanoResult<String> {
        let id = carrier.id().to_string();
        carrier.initialize().await?;
        self.drug_carriers.insert(id.clone(), Arc::new(RwLock::new(carrier)));
        Ok(id)
    }

    /// Get a drug carrier by ID
    pub fn get_drug_carrier(&self, id: &str) -> Option<Arc<RwLock<StandardDrugDelivery>>> {
        self.drug_carriers.get(id).cloned()
    }

    // ==================== Simulation Control ====================

    /// Start the simulation
    pub fn start(&mut self) {
        self.running = true;
    }

    /// Pause the simulation
    pub fn pause(&mut self) {
        self.running = false;
    }

    /// Reset the simulation
    pub async fn reset(&mut self) -> NanoResult<()> {
        self.time_ns = 0.0;
        self.running = false;

        // Reset all systems
        for (_, robot) in &self.nanorobots {
            robot.write().await.reset().await?;
        }
        for (_, assembler) in &self.assemblers {
            assembler.write().await.reset().await?;
        }
        for (_, sensor) in &self.sensors {
            sensor.write().await.reset().await?;
        }
        for (_, carrier) in &self.drug_carriers {
            carrier.write().await.reset().await?;
        }

        Ok(())
    }

    /// Advance simulation by one time step
    pub async fn step(&mut self) -> NanoResult<()> {
        if !self.running {
            return Ok(());
        }

        // Advance time
        self.time_ns += self.world.time_step_ns;

        // Update all systems (simplified - in real sim would be more complex)
        // Physics, collision detection, etc. would go here

        Ok(())
    }

    /// Run simulation for specified duration (in nanoseconds)
    pub async fn run_for(&mut self, duration_ns: f64) -> NanoResult<SimulationResult> {
        let start_time = self.time_ns;
        let end_time = start_time + duration_ns;

        self.start();

        let mut steps = 0u64;
        while self.time_ns < end_time {
            self.step().await?;
            steps += 1;
        }

        self.pause();

        Ok(SimulationResult {
            start_time_ns: start_time,
            end_time_ns: self.time_ns,
            steps_executed: steps,
            nanorobot_count: self.nanorobots.len(),
            assembler_count: self.assemblers.len(),
            sensor_count: self.sensors.len(),
            drug_carrier_count: self.drug_carriers.len(),
        })
    }

    /// Get statistics about the simulation
    pub fn stats(&self) -> SimulationStats {
        SimulationStats {
            time_ns: self.time_ns,
            running: self.running,
            nanorobot_count: self.nanorobots.len(),
            assembler_count: self.assemblers.len(),
            sensor_count: self.sensors.len(),
            machine_count: self.machines.len(),
            memory_count: self.memories.len(),
            drug_carrier_count: self.drug_carriers.len(),
            world_volume_nm3: self.world.bounds.volume(),
        }
    }
}

impl Default for NanoSimulator {
    fn default() -> Self {
        Self::new()
    }
}

/// Simulation result
#[derive(Debug, Clone)]
pub struct SimulationResult {
    pub start_time_ns: f64,
    pub end_time_ns: f64,
    pub steps_executed: u64,
    pub nanorobot_count: usize,
    pub assembler_count: usize,
    pub sensor_count: usize,
    pub drug_carrier_count: usize,
}

/// Simulation statistics
#[derive(Debug, Clone)]
pub struct SimulationStats {
    pub time_ns: f64,
    pub running: bool,
    pub nanorobot_count: usize,
    pub assembler_count: usize,
    pub sensor_count: usize,
    pub machine_count: usize,
    pub memory_count: usize,
    pub drug_carrier_count: usize,
    pub world_volume_nm3: f64,
}

// ==================== Builders ====================

/// Builder for creating nanorobots in simulation
pub struct SimNanorobotBuilder {
    id: String,
    position: Position3D,
    environment: Environment,
    propulsion_type: PropulsionType,
    initial_energy: f64,
}

impl SimNanorobotBuilder {
    pub fn new(id: impl Into<String>, environment: Environment) -> Self {
        Self {
            id: id.into(),
            position: Position3D::origin(),
            environment,
            propulsion_type: PropulsionType::Flagellar,
            initial_energy: 10_000_000.0,
        }
    }

    pub fn with_position(mut self, pos: Position3D) -> Self {
        self.position = pos;
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
        StandardNanorobot::builder(self.id)
            .with_position(self.position)
            .with_environment(self.environment)
            .with_propulsion(self.propulsion_type)
            .with_energy(self.initial_energy)
            .build()
    }
}

/// Builder for creating assemblers in simulation
pub struct SimAssemblerBuilder {
    id: String,
    environment: Environment,
    initial_energy: f64,
}

impl SimAssemblerBuilder {
    pub fn new(id: impl Into<String>, environment: Environment) -> Self {
        Self {
            id: id.into(),
            environment,
            initial_energy: 1_000_000.0,
        }
    }

    pub fn with_energy(mut self, energy_fj: f64) -> Self {
        self.initial_energy = energy_fj;
        self
    }

    pub fn build(self) -> StandardAssembler {
        StandardAssembler::builder(self.id)
            .with_environment(self.environment)
            .with_energy(self.initial_energy)
            .build()
    }
}

/// Builder for creating sensors in simulation
pub struct SimSensorBuilder {
    id: String,
    sensor_type: SensorType,
    environment: Environment,
}

impl SimSensorBuilder {
    pub fn new(id: impl Into<String>, sensor_type: SensorType, environment: Environment) -> Self {
        Self {
            id: id.into(),
            sensor_type,
            environment,
        }
    }

    pub fn build(self) -> StandardNanosensor {
        StandardNanosensor::builder(self.id, self.sensor_type)
            .with_environment(self.environment)
            .build()
    }
}

/// Builder for creating drug carriers in simulation
pub struct SimDrugDeliveryBuilder {
    id: String,
    carrier_type: CarrierType,
    environment: Environment,
    payload: Option<Payload>,
}

impl SimDrugDeliveryBuilder {
    pub fn new(id: impl Into<String>, carrier_type: CarrierType, environment: Environment) -> Self {
        Self {
            id: id.into(),
            carrier_type,
            environment,
            payload: None,
        }
    }

    pub fn with_payload(mut self, payload: Payload) -> Self {
        self.payload = Some(payload);
        self
    }

    pub fn build(self) -> StandardDrugDelivery {
        let mut builder = StandardDrugDelivery::builder(self.id, self.carrier_type)
            .with_environment(self.environment);

        if let Some(payload) = self.payload {
            builder = builder.with_payload(payload);
        }

        builder.build()
    }
}
