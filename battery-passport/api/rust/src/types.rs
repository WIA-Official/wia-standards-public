//! WIA Battery Passport - Type Definitions
//!
//! Data structures for EU Battery Regulation 2027 compliance.
//!
//! 홍익인간 (弘益人間) - Benefit All Humanity

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

// ============================================================================
// Root Types
// ============================================================================

/// Battery Passport - Root structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BatteryPassport {
    pub id: String,
    pub qr_code: String,
    pub version: String,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,

    pub identity: BatteryIdentity,
    pub manufacturer: ManufacturerInfo,
    pub specifications: BatterySpecifications,
    pub materials: MaterialComposition,
    pub carbon_footprint: CarbonFootprint,
    pub health: BatteryHealth,
    pub lifecycle: Vec<LifecycleEvent>,
    pub recycling: RecyclingInfo,
    pub certifications: Vec<Certification>,

    pub signature: Option<DigitalSignature>,
}

// ============================================================================
// Identity Types
// ============================================================================

/// Battery Identity Information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BatteryIdentity {
    /// EU-assigned unique identifier
    pub unique_identifier: String,
    /// Global Trade Item Number (optional)
    pub gtin: Option<String>,

    /// Battery classification
    pub category: BatteryCategory,
    pub chemistry: BatteryChemistry,

    /// Product info
    pub model: String,
    pub serial_number: String,
    pub batch_number: String,

    /// Physical characteristics
    pub weight_kg: f64,
    pub dimensions: Dimensions,

    /// Manufacturing
    pub production_date: DateTime<Utc>,
    pub production_country: String,
    pub production_facility_id: String,
}

/// Battery category per EU regulation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BatteryCategory {
    /// Electric Vehicle battery > 2 kWh
    Ev,
    /// Light Means of Transport (e-bikes, scooters)
    Lmt,
    /// Industrial battery > 2 kWh
    Industrial,
    /// Stationary energy storage
    Stationary,
    /// Portable battery
    Portable,
    /// Starting, Lighting, Ignition
    Sli,
}

/// Battery chemistry type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BatteryChemistry {
    /// Lithium Iron Phosphate
    Lfp,
    /// Nickel Manganese Cobalt
    Nmc,
    /// Nickel Cobalt Aluminum
    Nca,
    /// Lithium Cobalt Oxide
    Lco,
    /// Lithium Manganese Oxide
    Lmo,
    /// Lithium Titanate
    Lto,
    /// Solid State
    SolidState,
    /// Sodium Ion
    SodiumIon,
    /// Lead Acid
    LeadAcid,
    /// AGM
    Agm,
    /// Nickel Metal Hydride
    Nimh,
    /// Other
    Other,
}

/// Physical dimensions
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Dimensions {
    pub length_mm: f64,
    pub width_mm: f64,
    pub height_mm: f64,
    pub form_factor: FormFactor,
}

/// Battery form factor
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FormFactor {
    Cylindrical,
    Prismatic,
    Pouch,
    Pack,
}

// ============================================================================
// Manufacturer Types
// ============================================================================

/// Manufacturer information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ManufacturerInfo {
    pub name: String,
    pub legal_name: String,
    pub registration_number: String,

    pub address: Address,
    pub contact_email: String,
    pub contact_phone: String,
    pub website: String,

    pub eu_representative: Option<EURepresentative>,
    pub production_facilities: Vec<Facility>,
}

/// Address structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Address {
    pub street: String,
    pub city: String,
    pub postal_code: String,
    pub country: String,
}

/// EU Representative for non-EU manufacturers
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EURepresentative {
    pub name: String,
    pub address: Address,
    pub registration_number: String,
}

/// Production facility
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Facility {
    pub id: String,
    pub name: String,
    pub country: String,
    pub address: Address,
    pub certifications: Vec<String>,
}

// ============================================================================
// Specifications Types
// ============================================================================

/// Battery technical specifications
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BatterySpecifications {
    // Electrical
    pub nominal_voltage_v: f64,
    pub min_voltage_v: f64,
    pub max_voltage_v: f64,
    pub nominal_capacity_ah: f64,
    pub rated_capacity_wh: f64,
    pub energy_density_wh_kg: f64,
    pub power_density_w_kg: f64,

    // Performance
    pub c_rate_charge: f64,
    pub c_rate_discharge: f64,
    pub round_trip_efficiency: f64,
    pub self_discharge_rate: f64,

    // Temperature
    pub operating_temp_min_c: f64,
    pub operating_temp_max_c: f64,
    pub storage_temp_min_c: f64,
    pub storage_temp_max_c: f64,

    // Cycle life
    pub expected_cycle_life: u32,
    pub calendar_life_years: u32,
    pub warranty_years: u32,
    pub warranty_cycles: u32,

    // Safety
    pub ip_rating: String,
    pub un38_3_certified: bool,
    pub thermal_runaway_protection: bool,
}

// ============================================================================
// Materials Types
// ============================================================================

/// Material composition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MaterialComposition {
    pub cobalt: MaterialInfo,
    pub lithium: MaterialInfo,
    pub nickel: MaterialInfo,
    pub manganese: MaterialInfo,
    pub graphite: MaterialInfo,

    pub recycled_cobalt_percent: f64,
    pub recycled_lithium_percent: f64,
    pub recycled_nickel_percent: f64,
    pub recycled_lead_percent: f64,

    pub hazardous_substances: Vec<HazardousSubstance>,
    pub bill_of_materials: Vec<BOMEntry>,
}

/// Material information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MaterialInfo {
    pub weight_kg: f64,
    pub percentage: f64,
    pub source_countries: Vec<String>,
    pub recycled_content_percent: f64,
    pub responsible_sourcing: ResponsibleSourcing,
}

/// Responsible sourcing certification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResponsibleSourcing {
    pub certified: bool,
    pub certification_scheme: String,
    pub certificate_id: String,
    pub audit_date: Option<DateTime<Utc>>,
    pub due_diligence_report_url: Option<String>,
}

/// Hazardous substance declaration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HazardousSubstance {
    pub name: String,
    pub cas_number: String,
    pub concentration_ppm: f64,
    pub reach_compliant: bool,
    pub rohs_compliant: bool,
}

/// Bill of materials entry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BOMEntry {
    pub component: String,
    pub material: String,
    pub weight_kg: f64,
    pub supplier: String,
    pub country_of_origin: String,
}

// ============================================================================
// Carbon Footprint Types
// ============================================================================

/// Carbon footprint data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CarbonFootprint {
    pub total_kg_co2e: f64,
    pub per_kwh_kg_co2e: f64,

    pub raw_material_acquisition: f64,
    pub manufacturing: f64,
    pub transport: f64,

    pub performance_class: CarbonClass,

    pub methodology: String,
    pub calculation_date: DateTime<Utc>,
    pub third_party_verified: bool,
    pub verifier: Option<String>,
    pub verification_report_url: Option<String>,

    pub data_quality_rating: DataQuality,
}

/// Carbon performance class (EU requirement)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CarbonClass {
    A,  // < 50 kg CO2e/kWh
    B,  // 50-65 kg CO2e/kWh
    C,  // 65-80 kg CO2e/kWh
    D,  // 80-95 kg CO2e/kWh
    E,  // > 95 kg CO2e/kWh
}

/// Data quality rating
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DataQuality {
    Measured,
    Calculated,
    Estimated,
    Mixed,
}

// ============================================================================
// Health Types
// ============================================================================

/// Battery health data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BatteryHealth {
    pub state_of_health_percent: f64,
    pub state_of_charge_percent: f64,

    pub original_capacity_ah: f64,
    pub current_capacity_ah: f64,
    pub capacity_fade_percent: f64,

    pub internal_resistance_mohm: f64,
    pub resistance_increase_percent: f64,

    pub total_energy_throughput_kwh: f64,
    pub full_cycle_equivalents: u32,
    pub partial_cycles: u32,

    pub max_temperature_reached_c: f64,
    pub min_temperature_reached_c: f64,
    pub time_above_45c_hours: f64,
    pub time_below_0c_hours: f64,

    pub fast_charge_count: u32,
    pub slow_charge_count: u32,
    pub avg_charge_rate_c: f64,

    pub remaining_useful_life_months: Option<u32>,
    pub expected_eol_date: Option<DateTime<Utc>>,

    pub last_bms_sync: Option<DateTime<Utc>>,
    pub data_source: HealthDataSource,
}

/// Health data source
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HealthDataSource {
    Bms,
    Diagnostic,
    Estimated,
    Manual,
}

// ============================================================================
// Lifecycle Types
// ============================================================================

/// Lifecycle event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LifecycleEvent {
    pub id: String,
    pub event_type: LifecycleEventType,
    pub timestamp: DateTime<Utc>,

    pub facility_id: Option<String>,
    pub country: String,

    pub actor: ActorInfo,
    pub data: serde_json::Value,

    pub verified: bool,
    pub verifier: Option<String>,

    pub documents: Vec<Document>,
}

/// Lifecycle event type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LifecycleEventType {
    // Production
    Manufactured,
    QualityTested,

    // Distribution
    Shipped,
    Received,
    Sold,

    // First use
    Installed,
    Activated,

    // Service
    Maintained,
    Repaired,
    Diagnosed,
    Updated,

    // Ownership
    OwnershipTransferred,
    Leased,
    Returned,

    // Second life
    Repurposed,
    Recertified,

    // End of life
    Decommissioned,
    Collected,
    Recycled,
    Disposed,
}

/// Actor information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActorInfo {
    pub actor_type: ActorType,
    pub name: String,
    pub id: String,
    pub country: String,
}

/// Actor type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ActorType {
    Manufacturer,
    Distributor,
    Dealer,
    Owner,
    Operator,
    ServiceCenter,
    Recycler,
    Regulator,
}

/// Document reference
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Document {
    pub doc_type: String,
    pub url: String,
    pub hash: String,
}

// ============================================================================
// Recycling Types
// ============================================================================

/// Recycling information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecyclingInfo {
    pub recyclability_score: u32,
    pub design_for_recycling: bool,
    pub disassembly_manual_url: Option<String>,

    pub cobalt_recovery_target: f64,
    pub lithium_recovery_target: f64,
    pub nickel_recovery_target: f64,
    pub copper_recovery_target: f64,

    pub hazard_classification: String,
    pub special_handling_required: bool,
    pub handling_instructions: Option<String>,

    pub take_back_scheme: Option<String>,
    pub collection_points_url: Option<String>,
}

// ============================================================================
// Certification Types
// ============================================================================

/// Certification record
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Certification {
    pub cert_type: CertificationType,
    pub name: String,
    pub issuer: String,
    pub certificate_id: String,
    pub issue_date: DateTime<Utc>,
    pub expiry_date: DateTime<Utc>,
    pub verification_url: Option<String>,
    pub status: CertificationStatus,
}

/// Certification type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CertificationType {
    // Safety
    Un383,
    Iec62619,
    Iec62660,
    Ul2580,

    // Quality
    Iso9001,
    Iatf16949,

    // Environmental
    Iso14001,
    Iso14067,

    // Responsible sourcing
    Rmi,
    Irma,
    Asi,

    // EU specific
    CeMarking,
    EuTypeApproval,
}

/// Certification status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CertificationStatus {
    Valid,
    Expired,
    Revoked,
    Pending,
}

// ============================================================================
// Security Types
// ============================================================================

/// Digital signature
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DigitalSignature {
    pub algorithm: SignatureAlgorithm,
    pub public_key: String,
    pub signature: String,
    pub signed_at: DateTime<Utc>,
    pub signer_did: String,
}

/// Signature algorithm
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SignatureAlgorithm {
    Ed25519,
    #[serde(rename = "ECDSA-P256")]
    EcdsaP256,
}

// ============================================================================
// Result Types
// ============================================================================

/// SOH calculation result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SohResult {
    pub soh_percent: f64,
    pub capacity_soh: f64,
    pub resistance_soh: f64,
    pub cycle_soh: f64,
    pub confidence: ConfidenceLevel,
    pub measurement_date: DateTime<Utc>,
}

/// RUL prediction result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RulResult {
    pub remaining_months: u32,
    pub remaining_cycles: u32,
    pub expected_eol_date: DateTime<Utc>,
    pub confidence: ConfidenceLevel,
    pub methodology: String,
}

/// Second-life assessment result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SecondLifeResult {
    pub eligible: bool,
    pub score: u32,
    pub checks: Vec<Check>,
    pub suggested_applications: Vec<String>,
    pub estimated_remaining_value_usd: f64,
    pub recertification_required: bool,
}

/// Compliance check
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Check {
    pub name: String,
    pub passed: bool,
    pub message: String,
}

/// Confidence level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConfidenceLevel {
    High,
    Medium,
    Low,
}

/// Supply chain verification result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SupplyChainVerificationResult {
    pub overall_risk_score: f64,
    pub traceability_score: u32,
    pub eu_due_diligence_compliant: bool,
    pub material_verifications: Vec<MaterialVerification>,
    pub recommendations: Vec<String>,
}

/// Material verification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MaterialVerification {
    pub material: String,
    pub source_countries: Vec<String>,
    pub country_risk_level: String,
    pub certification_valid: bool,
    pub due_diligence_complete: bool,
}

/// Recycling efficiency result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecyclingEfficiencyResult {
    pub overall_efficiency_percent: f64,
    pub eu_compliant: bool,
    pub material_results: std::collections::HashMap<String, MaterialRecoveryResult>,
    pub process_type: String,
    pub recommendations: Vec<String>,
}

/// Material recovery result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MaterialRecoveryResult {
    pub original_kg: f64,
    pub recovered_kg: f64,
    pub efficiency_percent: f64,
    pub target_percent: f64,
    pub meets_target: bool,
}
