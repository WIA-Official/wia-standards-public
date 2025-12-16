//! Molecular assembler implementation

use crate::error::{NanoError, NanoResult};
use crate::types::*;
use crate::traits::*;
use async_trait::async_trait;
use std::sync::atomic::{AtomicU64, Ordering};

/// Standard molecular assembler implementation
pub struct StandardAssembler {
    id: String,
    status: SystemStatus,
    environment: Option<Environment>,
    scale: Option<Scale>,
    tool_position: Position3D,
    work_area: WorkArea,
    precision: AssemblyPrecision,
    building_blocks: Vec<BuildingBlock>,
    current_operation: Option<AssemblyOperation>,
    progress: Option<AssemblyProgress>,
    energy_available: f64,
    atoms_placed: AtomicU64,
}

impl StandardAssembler {
    /// Create a new molecular assembler
    pub fn new(id: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            status: SystemStatus::Offline,
            environment: None,
            scale: Some(Scale::default()),
            tool_position: Position3D::origin(),
            work_area: WorkArea::new(
                Position3D::new(-500.0, -500.0, -500.0),
                Position3D::new(500.0, 500.0, 500.0),
                0.1,
            ),
            precision: AssemblyPrecision::Standard,
            building_blocks: Vec::new(),
            current_operation: None,
            progress: None,
            energy_available: 1_000_000.0, // 1 pJ default
            atoms_placed: AtomicU64::new(0),
        }
    }

    /// Create with a builder pattern
    pub fn builder(id: impl Into<String>) -> AssemblerBuilder {
        AssemblerBuilder::new(id)
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

/// Builder for StandardAssembler
pub struct AssemblerBuilder {
    id: String,
    environment: Option<Environment>,
    work_area: Option<WorkArea>,
    precision: AssemblyPrecision,
    initial_energy: f64,
}

impl AssemblerBuilder {
    pub fn new(id: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            environment: None,
            work_area: None,
            precision: AssemblyPrecision::Standard,
            initial_energy: 1_000_000.0,
        }
    }

    pub fn with_environment(mut self, env: Environment) -> Self {
        self.environment = Some(env);
        self
    }

    pub fn with_work_area(mut self, area: WorkArea) -> Self {
        self.work_area = Some(area);
        self
    }

    pub fn with_precision(mut self, precision: AssemblyPrecision) -> Self {
        self.precision = precision;
        self
    }

    pub fn with_energy(mut self, energy_fj: f64) -> Self {
        self.initial_energy = energy_fj;
        self
    }

    pub fn build(self) -> StandardAssembler {
        let mut assembler = StandardAssembler::new(self.id);
        assembler.environment = self.environment;
        assembler.precision = self.precision;
        assembler.energy_available = self.initial_energy;
        if let Some(area) = self.work_area {
            assembler.work_area = area;
        }
        assembler
    }
}

#[async_trait]
impl NanoSystem for StandardAssembler {
    fn system_type(&self) -> NanoSystemType {
        NanoSystemType::MolecularAssembler
    }

    fn id(&self) -> &str {
        &self.id
    }

    fn status(&self) -> SystemStatus {
        self.status
    }

    async fn initialize(&mut self) -> NanoResult<()> {
        self.status = SystemStatus::Initializing;
        // Simulate initialization
        self.status = SystemStatus::Idle;
        Ok(())
    }

    async fn shutdown(&mut self) -> NanoResult<()> {
        self.status = SystemStatus::ShuttingDown;
        self.current_operation = None;
        self.progress = None;
        self.status = SystemStatus::Offline;
        Ok(())
    }

    async fn reset(&mut self) -> NanoResult<()> {
        self.tool_position = Position3D::origin();
        self.current_operation = None;
        self.progress = None;
        self.atoms_placed.store(0, Ordering::SeqCst);
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
            "system_type": "molecular_assembler",
            "tool_position": {
                "x": self.tool_position.x,
                "y": self.tool_position.y,
                "z": self.tool_position.z
            },
            "precision": format!("{:?}", self.precision),
            "atoms_placed": self.atoms_placed.load(Ordering::SeqCst),
            "energy_available_fj": self.energy_available
        });

        Ok(NanoMessage::builder()
            .device_id(&self.id)
            .system_type(NanoSystemType::MolecularAssembler)
            .payload(payload)?)
    }

    fn energy_level(&self) -> Option<f64> {
        Some(self.energy_available)
    }
}

#[async_trait]
impl MolecularAssembler for StandardAssembler {
    async fn start_assembly(&mut self, target: &Molecule) -> NanoResult<AssemblyOperation> {
        if !self.is_operational() {
            return Err(NanoError::InvalidParameter("System not operational".into()));
        }

        let op_id = format!("asm-{}", uuid::Uuid::new_v4());
        let operation = AssemblyOperation {
            id: op_id.clone(),
            target_molecule: target.name.clone().unwrap_or_else(|| "unknown".to_string()),
            started_at: chrono::Utc::now().to_rfc3339(),
            estimated_completion: None,
            status: AssemblyStatus::InProgress,
        };

        let progress = AssemblyProgress {
            operation_id: op_id,
            atoms_placed: 0,
            atoms_total: target.atoms.len() as u64,
            bonds_created: 0,
            bonds_total: target.bonds.len() as u64,
            energy_consumed: 0.0,
            error_count: 0,
            current_step: "Initializing".to_string(),
        };

        self.current_operation = Some(operation.clone());
        self.progress = Some(progress);
        self.status = SystemStatus::Active;

        Ok(operation)
    }

    async fn pause_assembly(&mut self) -> NanoResult<()> {
        if let Some(ref mut op) = self.current_operation {
            op.status = AssemblyStatus::Paused;
            self.status = SystemStatus::Idle;
        }
        Ok(())
    }

    async fn resume_assembly(&mut self) -> NanoResult<()> {
        if let Some(ref mut op) = self.current_operation {
            if op.status == AssemblyStatus::Paused {
                op.status = AssemblyStatus::InProgress;
                self.status = SystemStatus::Active;
            }
        }
        Ok(())
    }

    async fn cancel_assembly(&mut self) -> NanoResult<()> {
        if let Some(ref mut op) = self.current_operation {
            op.status = AssemblyStatus::Cancelled;
        }
        self.status = SystemStatus::Idle;
        Ok(())
    }

    fn assembly_progress(&self) -> Option<AssemblyProgress> {
        self.progress.clone()
    }

    async fn place_atom(&mut self, element: Element, position: Position3D) -> NanoResult<usize> {
        if !self.work_area.contains(&position) {
            return Err(NanoError::OutOfBounds {
                position: [position.x, position.y, position.z],
                bounds: [
                    self.work_area.min.x, self.work_area.min.y, self.work_area.min.z,
                    self.work_area.max.x, self.work_area.max.y, self.work_area.max.z,
                ],
            });
        }

        // Energy cost per atom placement (simplified model)
        let energy_cost = 10.0; // 10 fJ per atom
        self.consume_energy(energy_cost)?;

        let atom_index = self.atoms_placed.fetch_add(1, Ordering::SeqCst) as usize;

        if let Some(ref mut progress) = self.progress {
            progress.atoms_placed += 1;
            progress.energy_consumed += energy_cost;
            progress.current_step = format!("Placed {} at ({:.2}, {:.2}, {:.2})",
                element.symbol(), position.x, position.y, position.z);
        }

        // Move tool to position
        self.tool_position = position;

        Ok(atom_index)
    }

    async fn create_bond(&mut self, atom1: usize, atom2: usize) -> NanoResult<()> {
        let energy_cost = 5.0; // 5 fJ per bond
        self.consume_energy(energy_cost)?;

        if let Some(ref mut progress) = self.progress {
            progress.bonds_created += 1;
            progress.energy_consumed += energy_cost;
            progress.current_step = format!("Created bond {}-{}", atom1, atom2);
        }

        Ok(())
    }

    fn available_blocks(&self) -> &[BuildingBlock] {
        &self.building_blocks
    }

    fn add_blocks(&mut self, blocks: Vec<BuildingBlock>) {
        self.building_blocks.extend(blocks);
    }

    fn work_area(&self) -> WorkArea {
        self.work_area.clone()
    }

    fn set_precision(&mut self, precision: AssemblyPrecision) {
        self.precision = precision;
    }

    fn tool_position(&self) -> Position3D {
        self.tool_position
    }

    async fn move_tool(&mut self, position: Position3D) -> NanoResult<()> {
        if !self.work_area.contains(&position) {
            return Err(NanoError::OutOfBounds {
                position: [position.x, position.y, position.z],
                bounds: [
                    self.work_area.min.x, self.work_area.min.y, self.work_area.min.z,
                    self.work_area.max.x, self.work_area.max.y, self.work_area.max.z,
                ],
            });
        }

        let distance = self.tool_position.distance_to(&position);
        let energy_cost = distance * 0.01; // 0.01 fJ per nm
        self.consume_energy(energy_cost)?;

        self.tool_position = position;
        Ok(())
    }
}
