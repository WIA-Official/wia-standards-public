//! Type definitions for 3D Printing Construction Standard
//!
//! 弘益人間 - Building the future with technology that benefits all

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::{DateTime, Utc};

/// Building design specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BuildingDesign {
    pub id: Uuid,
    pub name: String,
    pub area_sqm: f64,
    pub height_m: f64,
    pub floors: u32,
    pub material_type: MaterialType,
    pub structure_type: StructureType,
    pub design_file_url: String,
    pub created_at: DateTime<Utc>,
}

impl BuildingDesign {
    pub fn new(name: impl Into<String>, area_sqm: f64, material_type: MaterialType) -> Self {
        Self {
            id: Uuid::new_v4(),
            name: name.into(),
            area_sqm,
            height_m: 3.0,
            floors: 1,
            material_type,
            structure_type: StructureType::LoadBearing,
            design_file_url: String::new(),
            created_at: Utc::now(),
        }
    }
}

/// Material types for 3D printing construction
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum MaterialType {
    #[serde(rename = "concrete")]
    Concrete,
    #[serde(rename = "reinforced_concrete")]
    ReinforcedConcrete,
    #[serde(rename = "polymer")]
    Polymer,
    #[serde(rename = "composite")]
    Composite,
    #[serde(rename = "recycled_material")]
    RecycledMaterial,
    #[serde(rename = "bio_material")]
    BioMaterial,
}

/// Building structure types
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum StructureType {
    #[serde(rename = "load_bearing")]
    LoadBearing,
    #[serde(rename = "frame")]
    Frame,
    #[serde(rename = "shell")]
    Shell,
    #[serde(rename = "modular")]
    Modular,
}

/// Print job specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrintJob {
    pub id: Uuid,
    pub design_id: Uuid,
    pub status: PrintStatus,
    pub progress_percent: f64,
    pub estimated_completion: DateTime<Utc>,
    pub material_used_kg: f64,
    pub print_layers: Vec<PrintLayer>,
    pub quality_checks: Vec<QualityCheck>,
    pub started_at: Option<DateTime<Utc>>,
    pub completed_at: Option<DateTime<Utc>>,
}

/// Print job status
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum PrintStatus {
    #[serde(rename = "queued")]
    Queued,
    #[serde(rename = "preparing")]
    Preparing,
    #[serde(rename = "printing")]
    Printing,
    #[serde(rename = "paused")]
    Paused,
    #[serde(rename = "quality_check")]
    QualityCheck,
    #[serde(rename = "completed")]
    Completed,
    #[serde(rename = "failed")]
    Failed,
}

/// Individual print layer
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrintLayer {
    pub layer_number: u32,
    pub height_mm: f64,
    pub thickness_mm: f64,
    pub status: LayerStatus,
    pub print_time_seconds: u64,
    pub material_used_kg: f64,
}

/// Layer status
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum LayerStatus {
    #[serde(rename = "pending")]
    Pending,
    #[serde(rename = "printing")]
    Printing,
    #[serde(rename = "completed")]
    Completed,
    #[serde(rename = "failed")]
    Failed,
}

/// Quality control check
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualityCheck {
    pub id: Uuid,
    pub check_type: QualityCheckType,
    pub status: QualityStatus,
    pub measurements: Vec<Measurement>,
    pub passed: bool,
    pub inspector_id: Option<String>,
    pub checked_at: DateTime<Utc>,
    pub notes: String,
}

/// Quality check types
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum QualityCheckType {
    #[serde(rename = "dimensional_accuracy")]
    DimensionalAccuracy,
    #[serde(rename = "structural_integrity")]
    StructuralIntegrity,
    #[serde(rename = "surface_finish")]
    SurfaceFinish,
    #[serde(rename = "material_strength")]
    MaterialStrength,
    #[serde(rename = "safety_compliance")]
    SafetyCompliance,
}

/// Quality check status
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum QualityStatus {
    #[serde(rename = "pending")]
    Pending,
    #[serde(rename = "in_progress")]
    InProgress,
    #[serde(rename = "passed")]
    Passed,
    #[serde(rename = "failed")]
    Failed,
}

/// Measurement data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Measurement {
    pub parameter: String,
    pub value: f64,
    pub unit: String,
    pub tolerance: f64,
    pub within_spec: bool,
}

/// Printer configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrinterConfig {
    pub printer_id: String,
    pub model: String,
    pub max_build_area_sqm: f64,
    pub max_height_m: f64,
    pub nozzle_size_mm: f64,
    pub print_speed_mm_s: f64,
    pub material_capacity_kg: f64,
    pub status: PrinterStatus,
}

/// Printer status
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum PrinterStatus {
    #[serde(rename = "idle")]
    Idle,
    #[serde(rename = "printing")]
    Printing,
    #[serde(rename = "maintenance")]
    Maintenance,
    #[serde(rename = "offline")]
    Offline,
    #[serde(rename = "error")]
    Error,
}
