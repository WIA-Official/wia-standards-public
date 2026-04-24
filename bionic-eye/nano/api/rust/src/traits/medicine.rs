//! Nanomedicine trait definitions

use crate::error::NanoResult;
use crate::types::Position3D;
use async_trait::async_trait;

/// Trait for nanomedicine drug delivery systems
#[async_trait]
pub trait DrugDeliverySystem: Send + Sync {
    /// Get carrier ID
    fn carrier_id(&self) -> &str;

    /// Get carrier type
    fn carrier_type(&self) -> CarrierType;

    /// Get carrier size in nm
    fn size_nm(&self) -> f64;

    /// Get current position (if trackable)
    fn position(&self) -> Option<Position3D>;

    /// Get payload information
    fn payload(&self) -> &Payload;

    /// Get payload release status
    fn release_status(&self) -> ReleaseStatus;

    /// Get targeting configuration
    fn targeting(&self) -> &TargetingConfig;

    /// Check if at target site
    fn is_at_target(&self) -> bool;

    /// Trigger payload release
    async fn trigger_release(&mut self) -> NanoResult<ReleaseResult>;

    /// Get release triggers
    fn release_triggers(&self) -> &[ReleaseTrigger];

    /// Get pharmacokinetics data
    fn pharmacokinetics(&self) -> &Pharmacokinetics;

    /// Track carrier (update position)
    async fn update_tracking(&mut self) -> NanoResult<TrackingUpdate>;
}

/// Type of drug carrier
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CarrierType {
    /// Lipid-based liposome
    Liposome,
    /// Polymeric nanoparticle
    PolymericNanoparticle,
    /// Dendrimer
    Dendrimer,
    /// Micelle
    Micelle,
    /// Solid lipid nanoparticle
    SolidLipidNanoparticle,
    /// Gold nanoparticle
    GoldNanoparticle,
    /// Magnetic nanoparticle
    MagneticNanoparticle,
    /// Carbon nanotube
    CarbonNanotube,
    /// Quantum dot
    QuantumDot,
    /// Viral vector
    ViralVector,
    /// Exosome
    Exosome,
    /// Custom carrier
    Custom,
}

/// Payload information
#[derive(Debug, Clone)]
pub struct Payload {
    pub drug_name: String,
    pub drug_class: DrugClass,
    pub amount_ng: f64,
    pub loading_efficiency: f64,  // 0.0-1.0
    pub released_amount_ng: f64,
}

impl Payload {
    pub fn remaining_amount(&self) -> f64 {
        self.amount_ng - self.released_amount_ng
    }

    pub fn release_percentage(&self) -> f64 {
        if self.amount_ng > 0.0 {
            (self.released_amount_ng / self.amount_ng) * 100.0
        } else {
            0.0
        }
    }
}

/// Drug classification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DrugClass {
    SmallMolecule,
    Protein,
    Peptide,
    Antibody,
    Nucleic,  // siRNA, mRNA, etc.
    Gene,
    Imaging,
    Radioisotope,
    Other,
}

/// Release status
#[derive(Debug, Clone)]
pub struct ReleaseStatus {
    pub state: ReleaseState,
    pub cumulative_released: f64,  // percentage
    pub release_rate: f64,         // ng/s
    pub time_to_complete: Option<f64>,  // seconds
}

/// Release state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReleaseState {
    /// Payload intact, not releasing
    Intact,
    /// Triggered, actively releasing
    Releasing,
    /// Partial release, paused
    PartialRelease,
    /// Fully released
    Depleted,
    /// Premature/uncontrolled release
    Burst,
}

/// Targeting configuration
#[derive(Debug, Clone)]
pub struct TargetingConfig {
    pub targeting_type: TargetingType,
    pub target_markers: Vec<String>,
    pub target_tissue: Option<String>,
    pub binding_affinity: Option<f64>,  // Kd in nM
    pub specificity: f64,  // 0.0-1.0
}

/// Type of targeting mechanism
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetingType {
    /// No active targeting (passive EPR effect)
    Passive,
    /// Ligand-receptor targeting
    LigandReceptor,
    /// Antibody targeting
    Antibody,
    /// Aptamer targeting
    Aptamer,
    /// Magnetic guidance
    Magnetic,
    /// Ultrasound guidance
    Ultrasound,
    /// pH-responsive
    PhResponsive,
    /// Temperature-responsive
    ThermoResponsive,
}

/// Release trigger mechanism
#[derive(Debug, Clone)]
pub struct ReleaseTrigger {
    pub trigger_type: TriggerType,
    pub threshold: f64,
    pub unit: String,
    pub is_active: bool,
}

/// Type of release trigger
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriggerType {
    /// pH-triggered (acidic environment)
    Ph,
    /// Temperature-triggered
    Temperature,
    /// Enzyme-triggered
    Enzyme,
    /// Light-triggered (photosensitive)
    Light,
    /// Ultrasound-triggered
    Ultrasound,
    /// Magnetic field triggered
    Magnetic,
    /// Redox-triggered
    Redox,
    /// Time-based degradation
    TimeBased,
}

/// Release operation result
#[derive(Debug, Clone)]
pub struct ReleaseResult {
    pub success: bool,
    pub amount_released_ng: f64,
    pub trigger_used: TriggerType,
    pub duration_ms: u64,
    pub side_effects: Vec<String>,
}

/// Pharmacokinetics data
#[derive(Debug, Clone)]
pub struct Pharmacokinetics {
    pub half_life_hours: f64,
    pub clearance_ml_per_hour: f64,
    pub volume_of_distribution_ml: f64,
    pub bioavailability: f64,  // 0.0-1.0
    pub peak_concentration_time_hours: Option<f64>,
}

/// Position tracking update
#[derive(Debug, Clone)]
pub struct TrackingUpdate {
    pub position: Option<Position3D>,
    pub velocity: Option<[f64; 3]>,
    pub tissue_location: Option<String>,
    pub confidence: f64,
    pub timestamp: String,
}

/// Theranostic system (therapy + diagnostics)
#[async_trait]
pub trait Theranostic: DrugDeliverySystem {
    /// Get imaging modality
    fn imaging_modality(&self) -> ImagingModality;

    /// Acquire diagnostic image/signal
    async fn acquire_signal(&mut self) -> NanoResult<DiagnosticSignal>;

    /// Get therapeutic effect
    fn therapeutic_effect(&self) -> &TherapeuticEffect;

    /// Combined treatment and monitoring
    async fn treat_and_monitor(&mut self) -> NanoResult<TheranosticResult>;
}

/// Imaging modality for theranostics
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImagingModality {
    Fluorescence,
    Mri,
    Pet,
    Ct,
    Ultrasound,
    Photoacoustic,
    Raman,
    MultiModal,
}

/// Diagnostic signal from theranostic
#[derive(Debug, Clone)]
pub struct DiagnosticSignal {
    pub modality: ImagingModality,
    pub intensity: f64,
    pub location: Option<Position3D>,
    pub timestamp: String,
    pub metadata: serde_json::Value,
}

/// Therapeutic effect assessment
#[derive(Debug, Clone)]
pub struct TherapeuticEffect {
    pub efficacy: f64,  // 0.0-1.0
    pub toxicity: f64,  // 0.0-1.0
    pub therapeutic_index: f64,
}

/// Combined theranostic result
#[derive(Debug, Clone)]
pub struct TheranosticResult {
    pub diagnostic: DiagnosticSignal,
    pub treatment: ReleaseResult,
    pub effect_assessment: TherapeuticEffect,
}

/// Nanomedicine safety monitoring
pub trait SafetyMonitor {
    /// Check biocompatibility status
    fn biocompatibility(&self) -> BiocompatibilityStatus;

    /// Get toxicity assessment
    fn toxicity_assessment(&self) -> ToxicityAssessment;

    /// Check immune response
    fn immune_response(&self) -> ImmuneResponse;
}

/// Biocompatibility status
#[derive(Debug, Clone)]
pub struct BiocompatibilityStatus {
    pub is_biocompatible: bool,
    pub degradation_products: Vec<String>,
    pub cytotoxicity_level: f64,  // 0.0-1.0
}

/// Toxicity assessment
#[derive(Debug, Clone)]
pub struct ToxicityAssessment {
    pub acute_toxicity: f64,
    pub chronic_toxicity: f64,
    pub genotoxicity: bool,
    pub organ_accumulation: Vec<OrganAccumulation>,
}

/// Organ accumulation data
#[derive(Debug, Clone)]
pub struct OrganAccumulation {
    pub organ: String,
    pub concentration_ng_per_g: f64,
    pub clearance_rate: f64,
}

/// Immune response assessment
#[derive(Debug, Clone)]
pub struct ImmuneResponse {
    pub immunogenic: bool,
    pub complement_activation: bool,
    pub antibody_formation: bool,
    pub inflammation_level: f64,  // 0.0-1.0
}
