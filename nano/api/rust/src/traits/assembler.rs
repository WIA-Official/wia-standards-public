//! Molecular assembler trait definitions

use crate::error::NanoResult;
use crate::types::{Position3D, Molecule, BuildingBlock, Element};
use async_trait::async_trait;

/// Trait for molecular assembly operations
#[async_trait]
pub trait MolecularAssembler: Send + Sync {
    /// Start an assembly operation
    async fn start_assembly(&mut self, target: &Molecule) -> NanoResult<AssemblyOperation>;

    /// Pause the current assembly
    async fn pause_assembly(&mut self) -> NanoResult<()>;

    /// Resume a paused assembly
    async fn resume_assembly(&mut self) -> NanoResult<()>;

    /// Cancel the current assembly
    async fn cancel_assembly(&mut self) -> NanoResult<()>;

    /// Get current assembly progress
    fn assembly_progress(&self) -> Option<AssemblyProgress>;

    /// Place an atom at a specific position
    async fn place_atom(&mut self, element: Element, position: Position3D) -> NanoResult<usize>;

    /// Create a bond between two atoms
    async fn create_bond(&mut self, atom1: usize, atom2: usize) -> NanoResult<()>;

    /// Get available building blocks
    fn available_blocks(&self) -> &[BuildingBlock];

    /// Add building blocks to inventory
    fn add_blocks(&mut self, blocks: Vec<BuildingBlock>);

    /// Get the current work area bounds
    fn work_area(&self) -> WorkArea;

    /// Set precision mode
    fn set_precision(&mut self, precision: AssemblyPrecision);

    /// Get current tool position
    fn tool_position(&self) -> Position3D;

    /// Move tool to position
    async fn move_tool(&mut self, position: Position3D) -> NanoResult<()>;
}

/// Assembly operation information
#[derive(Debug, Clone)]
pub struct AssemblyOperation {
    pub id: String,
    pub target_molecule: String,
    pub started_at: String,
    pub estimated_completion: Option<String>,
    pub status: AssemblyStatus,
}

/// Assembly progress tracking
#[derive(Debug, Clone)]
pub struct AssemblyProgress {
    pub operation_id: String,
    pub atoms_placed: u64,
    pub atoms_total: u64,
    pub bonds_created: u64,
    pub bonds_total: u64,
    pub energy_consumed: f64,
    pub error_count: u32,
    pub current_step: String,
}

impl AssemblyProgress {
    pub fn completion_percentage(&self) -> f64 {
        if self.atoms_total == 0 {
            return 0.0;
        }
        (self.atoms_placed as f64 / self.atoms_total as f64) * 100.0
    }
}

/// Assembly operation status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AssemblyStatus {
    Queued,
    InProgress,
    Paused,
    Completed,
    Failed,
    Cancelled,
}

/// Assembly precision levels
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AssemblyPrecision {
    /// Standard precision (~0.1nm)
    Standard,
    /// High precision (~0.01nm)
    High,
    /// Atomic precision (~0.001nm)
    Atomic,
}

impl AssemblyPrecision {
    /// Get precision in nanometers
    pub fn tolerance_nm(&self) -> f64 {
        match self {
            Self::Standard => 0.1,
            Self::High => 0.01,
            Self::Atomic => 0.001,
        }
    }
}

/// Work area definition
#[derive(Debug, Clone)]
pub struct WorkArea {
    pub min: Position3D,
    pub max: Position3D,
    pub resolution_nm: f64,
}

impl WorkArea {
    pub fn new(min: Position3D, max: Position3D, resolution_nm: f64) -> Self {
        Self { min, max, resolution_nm }
    }

    pub fn contains(&self, pos: &Position3D) -> bool {
        pos.x >= self.min.x && pos.x <= self.max.x &&
        pos.y >= self.min.y && pos.y <= self.max.y &&
        pos.z >= self.min.z && pos.z <= self.max.z
    }

    pub fn volume(&self) -> f64 {
        (self.max.x - self.min.x) *
        (self.max.y - self.min.y) *
        (self.max.z - self.min.z)
    }
}

/// Verification of assembled structure
pub trait AssemblyVerifier {
    /// Verify structural integrity
    fn verify_structure(&self, molecule: &Molecule) -> VerificationResult;

    /// Check bond validity
    fn verify_bonds(&self, molecule: &Molecule) -> Vec<BondVerification>;

    /// Check for steric clashes
    fn check_clashes(&self, molecule: &Molecule) -> Vec<StericClash>;
}

/// Structure verification result
#[derive(Debug, Clone)]
pub struct VerificationResult {
    pub valid: bool,
    pub score: f64,
    pub issues: Vec<String>,
}

/// Bond verification result
#[derive(Debug, Clone)]
pub struct BondVerification {
    pub atom1_idx: usize,
    pub atom2_idx: usize,
    pub expected_length_pm: f64,
    pub actual_length_pm: f64,
    pub valid: bool,
}

/// Steric clash information
#[derive(Debug, Clone)]
pub struct StericClash {
    pub atom1_idx: usize,
    pub atom2_idx: usize,
    pub distance_pm: f64,
    pub severity: ClashSeverity,
}

/// Severity of steric clash
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClashSeverity {
    Minor,
    Moderate,
    Severe,
}
