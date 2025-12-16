//! Phase 1 기반 데이터 타입 정의
//!
//! WIA-PET-HEALTH-PASSPORT 데이터 구조

use chrono::{DateTime, NaiveDate, Utc};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

// ==================== Pet Identity ====================

/// 반려동물 종
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PetSpecies {
    Dog,
    Cat,
    Rabbit,
    Hamster,
    Bird,
    Reptile,
    Fish,
    Other,
}

impl Default for PetSpecies {
    fn default() -> Self {
        Self::Dog
    }
}

/// 반려동물 성별
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PetSex {
    Male,
    Female,
    NeuteredMale,
    SpayedFemale,
}

impl Default for PetSex {
    fn default() -> Self {
        Self::Male
    }
}

/// 반려동물 사진
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PetPhoto {
    pub photo_hash: String,
    pub captured_at: DateTime<Utc>,
    pub photo_type: String,
    pub storage_uri: Option<String>,
}

/// 체중 정보
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Weight {
    pub value: f32,
    pub unit: String,
    pub measured_at: DateTime<Utc>,
}

/// 반려동물 신원 정보
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PetIdentity {
    pub passport_id: Uuid,
    pub microchip_id: Option<String>,
    pub tattoo_id: Option<String>,

    pub species: PetSpecies,
    pub breed: Option<String>,
    pub breed_verified: bool,

    pub name: String,
    pub date_of_birth: Option<NaiveDate>,
    pub date_of_birth_estimated: bool,
    pub sex: PetSex,

    pub color: Option<String>,
    pub markings: Vec<String>,
    pub weight: Option<Weight>,

    pub photos: Vec<PetPhoto>,
}

impl Default for PetIdentity {
    fn default() -> Self {
        Self {
            passport_id: Uuid::now_v7(),
            microchip_id: None,
            tattoo_id: None,
            species: PetSpecies::default(),
            breed: None,
            breed_verified: false,
            name: String::new(),
            date_of_birth: None,
            date_of_birth_estimated: false,
            sex: PetSex::default(),
            color: None,
            markings: Vec::new(),
            weight: None,
            photos: Vec::new(),
        }
    }
}

// ==================== Microchip ====================

/// 마이크로칩 상태
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MicrochipStatus {
    Active,
    Replaced,
    Removed,
    Lost,
}

/// 마이크로칩 정보
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MicrochipInfo {
    pub chip_number: String,
    pub manufacturer_code: Option<String>,
    pub registered_at: DateTime<Utc>,
    pub registry_name: String,
    pub registry_country: String,
    pub status: MicrochipStatus,
    pub implant_location: String,
    pub implanted_at: DateTime<Utc>,
}

// ==================== Guardian ====================

/// 비상 연락처
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmergencyContact {
    pub name: String,
    pub relationship: String,
    pub phone: String,
    pub email: Option<String>,
}

/// 주소
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Address {
    pub country: String,
    pub postal_code: String,
    pub city: String,
    pub address_line1: String,
    pub address_line2: Option<String>,
}

/// 보호자 정보
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GuardianInfo {
    pub guardian_id: String,
    pub name: String,
    pub contact_phone: String,
    pub contact_email: String,
    pub address: Address,
    pub ownership_type: String,
    pub ownership_start_date: NaiveDate,
    pub emergency_contacts: Vec<EmergencyContact>,
}

// ==================== Medical Records ====================

/// 백신 대상 질병
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum VaccineDisease {
    // 개
    Rabies,
    Distemper,
    Parvovirus,
    Hepatitis,
    Parainfluenza,
    Leptospirosis,
    Bordetella,
    Lyme,
    CanineInfluenza,

    // 고양이
    FelinePanleukopenia,
    FelineHerpesvirus,
    FelineCalicivirus,
    FelineLeukemia,
    FelineImmunodeficiency,

    Other,
}

/// 부작용 기록
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdverseReaction {
    pub reaction_type: String,
    pub symptoms: Vec<String>,
    pub onset: DateTime<Utc>,
    pub duration: String,
    pub treatment: Option<String>,
}

/// 백신 정보
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VaccineInfo {
    pub name: String,
    pub manufacturer: Option<String>,
    pub lot_number: Option<String>,
    pub serial_number: Option<String>,
}

/// 수의사 참조
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VeterinarianRef {
    pub name: String,
    pub license_number: String,
    pub clinic: Option<String>,
}

/// 병원 참조
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClinicRef {
    pub name: String,
    pub license_number: Option<String>,
    pub address: Option<String>,
    pub phone: Option<String>,
}

/// 백신 접종 기록
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VaccinationRecord {
    pub record_id: Uuid,
    pub vaccine: VaccineInfo,
    pub target_diseases: Vec<VaccineDisease>,

    pub administered_at: DateTime<Utc>,
    pub administered_by: Option<VeterinarianRef>,
    pub clinic: Option<ClinicRef>,

    pub valid_from: DateTime<Utc>,
    pub valid_until: DateTime<Utc>,

    pub dose_number: u32,
    pub total_doses: u32,

    pub adverse_reactions: Vec<AdverseReaction>,

    pub verified: bool,
    pub verification_method: Option<String>,
    pub digital_signature: Option<String>,
}

/// 수술/시술 유형
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProcedureType {
    Spay,
    Neuter,
    Dental,
    Orthopedic,
    TumorRemoval,
    Emergency,
    Diagnostic,
    Cosmetic,
    Other,
}

/// 마취 정보
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnesthesiaInfo {
    pub anesthesia_type: String,
    pub agents: Vec<String>,
    pub duration: u32,
    pub complications: Vec<String>,
}

/// 수술 기록
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SurgeryRecord {
    pub record_id: Uuid,
    pub procedure_type: ProcedureType,
    pub procedure_name: String,
    pub description: Option<String>,

    pub performed_at: DateTime<Utc>,
    pub duration: u32,

    pub surgeon: VeterinarianRef,
    pub assistants: Vec<VeterinarianRef>,
    pub clinic: ClinicRef,

    pub anesthesia: Option<AnesthesiaInfo>,

    pub outcome: String,
    pub complications: Vec<String>,
    pub follow_up_required: bool,
    pub follow_up_date: Option<NaiveDate>,

    pub recovery_instructions: Option<String>,
    pub restrictions: Vec<String>,
    pub restriction_end_date: Option<NaiveDate>,
}

/// 진단 검사 유형
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DiagnosticTestType {
    BloodPanel,
    Urinalysis,
    Fecal,
    Xray,
    Ultrasound,
    Mri,
    CtScan,
    Biopsy,
    AllergyTest,
    GeneticTest,
    Heartworm,
    TickBorne,
    Other,
}

/// 진단 결과
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiagnosticResult {
    pub parameter: String,
    pub value: String,
    pub unit: Option<String>,
    pub reference_min: Option<f32>,
    pub reference_max: Option<f32>,
    pub status: String,
    pub notes: Option<String>,
}

/// 진단 기록
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiagnosticRecord {
    pub record_id: Uuid,
    pub test_type: DiagnosticTestType,
    pub test_name: String,

    pub performed_at: DateTime<Utc>,
    pub performed_by: Option<VeterinarianRef>,
    pub clinic: Option<ClinicRef>,
    pub laboratory: Option<String>,

    pub results: Vec<DiagnosticResult>,
    pub interpretation: Option<String>,
}

/// 알레르기 유형
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AllergenType {
    Food,
    Environmental,
    Medication,
    Insect,
    Contact,
    Other,
}

/// 알레르기 기록
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AllergyRecord {
    pub record_id: Uuid,
    pub allergen: String,
    pub allergen_type: AllergenType,
    pub reaction_severity: String,
    pub symptoms: Vec<String>,

    pub diagnosed_at: Option<DateTime<Utc>>,
    pub diagnosed_by: Option<VeterinarianRef>,
    pub diagnosis_method: Option<String>,

    pub avoidance_instructions: Option<String>,
    pub emergency_treatment: Option<String>,

    pub status: String,
}

/// 만성 질환 기록
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChronicCondition {
    pub condition_id: Uuid,
    pub condition_name: String,
    pub icd_code: Option<String>,

    pub diagnosed_at: DateTime<Utc>,
    pub diagnosed_by: Option<VeterinarianRef>,

    pub severity: String,
    pub status: String,

    pub management_plan: Option<String>,
    pub monitoring_schedule: Option<String>,
    pub prognosis: Option<String>,
}

/// 투약 용법
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Dosage {
    pub amount: f32,
    pub unit: String,
    pub frequency: String,
    pub route: String,
}

/// 약물 정보
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MedicationInfo {
    pub name: String,
    pub generic_name: Option<String>,
    pub manufacturer: Option<String>,
    pub ndc: Option<String>,
}

/// 투약 기록
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MedicationRecord {
    pub record_id: Uuid,
    pub medication: MedicationInfo,

    pub prescribed_at: DateTime<Utc>,
    pub prescribed_by: Option<VeterinarianRef>,

    pub dosage: Dosage,

    pub start_date: NaiveDate,
    pub end_date: Option<NaiveDate>,
    pub as_needed: bool,

    pub indication: String,
    pub related_condition: Option<String>,

    pub warnings: Vec<String>,
    pub interactions: Vec<String>,
}

// ==================== Genetic ====================

/// 품종 구성
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BreedComponent {
    pub breed: String,
    pub percentage: f32,
    pub confidence: f32,
}

/// 유전 건강 마커
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeneticHealthMarker {
    pub marker: String,
    pub gene: String,
    pub condition: String,
    pub genotype: String,
    pub risk_level: String,
    pub description: String,
    pub inheritance: String,
    pub action_required: Option<String>,
}

/// 유전 형질
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeneticTrait {
    pub trait_name: String,
    pub gene: String,
    pub genotype: String,
    pub phenotype: String,
    pub confidence: f32,
}

/// 유전자 프로필
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeneticProfile {
    pub profile_id: Uuid,
    pub test_date: NaiveDate,
    pub laboratory: String,
    pub test_kit: Option<String>,

    pub breed_composition: Vec<BreedComponent>,
    pub health_markers: Vec<GeneticHealthMarker>,
    pub traits: Vec<GeneticTrait>,

    pub maternal_haplogroup: Option<String>,
    pub paternal_haplogroup: Option<String>,

    pub raw_data_hash: Option<String>,
    pub raw_data_uri: Option<String>,
}

// ==================== Travel ====================

/// 검역 기록
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuarantineRecord {
    pub facility_name: String,
    pub facility_country: String,
    pub start_date: NaiveDate,
    pub end_date: NaiveDate,
    pub duration_days: u32,
    pub outcome: String,
    pub notes: Option<String>,
}

/// 여행 서류
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TravelDocument {
    pub document_type: String,
    pub document_number: Option<String>,
    pub issued_at: DateTime<Utc>,
    pub valid_until: DateTime<Utc>,
    pub issuing_authority: String,
    pub status: String,
}

/// 여행 기록
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TravelRecord {
    pub record_id: Uuid,
    pub departure_country: String,
    pub arrival_country: String,
    pub departure_date: NaiveDate,
    pub arrival_date: NaiveDate,
    pub transport_method: String,
    pub carrier: Option<String>,
    pub quarantine: Option<QuarantineRecord>,
    pub required_documents: Vec<TravelDocument>,
    pub approval_status: String,
    pub approval_authority: Option<String>,
}

/// 광견병 백신 요구사항
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RabiesRequirement {
    pub required: bool,
    pub minimum_age: u32,
    pub wait_period: u32,
    pub validity_period: u32,
    pub titer_test_required: bool,
    pub minimum_titer: f32,
}

/// 기생충 치료 요구사항
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParasiteTreatmentRequirement {
    pub treatment_type: String,
    pub required: bool,
    pub timing_before_entry: u32,
}

/// 검역 요구사항
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuarantineRequirement {
    pub required: bool,
    pub duration: u32,
    pub home_quarantine_allowed: bool,
}

/// 국가별 검역 요구사항
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CountryRequirements {
    pub country: String,
    pub last_updated: DateTime<Utc>,

    pub microchip_required: bool,
    pub microchip_standard: String,

    pub rabies_vaccination: RabiesRequirement,
    pub other_vaccinations: Vec<String>,

    pub parasite_treatment: Option<ParasiteTreatmentRequirement>,
    pub quarantine: Option<QuarantineRequirement>,

    pub banned_breeds: Vec<String>,
    pub import_permit_required: bool,
    pub health_certificate_required: bool,
    pub health_certificate_valid_days: u32,

    pub exempt_countries: Vec<String>,
}

// ==================== Medical Records ====================

/// 의료 기록 컬렉션
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MedicalRecords {
    pub vaccinations: Vec<VaccinationRecord>,
    pub surgeries: Vec<SurgeryRecord>,
    pub diagnostics: Vec<DiagnosticRecord>,
    pub allergies: Vec<AllergyRecord>,
    pub conditions: Vec<ChronicCondition>,
    pub medications: Vec<MedicationRecord>,
}

// ==================== Verification ====================

/// 검증 정보
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Verification {
    pub issuing_authority: String,
    pub digital_signature: Option<String>,
    pub certificate_chain: Vec<String>,
}

// ==================== Passport Header ====================

/// 여권 헤더
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PassportHeader {
    pub magic: String,
    pub version: [u32; 3],
    pub standard: String,
    pub created_at: DateTime<Utc>,
    pub last_updated: DateTime<Utc>,
}

impl Default for PassportHeader {
    fn default() -> Self {
        Self {
            magic: "WIAPET".to_string(),
            version: [1, 0, 0],
            standard: "WIA-PET-HEALTH-PASSPORT".to_string(),
            created_at: Utc::now(),
            last_updated: Utc::now(),
        }
    }
}

// ==================== Pet Health Passport ====================

/// 반려동물 건강 여권 (메인 구조)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PetHealthPassport {
    pub header: PassportHeader,
    pub identity: PetIdentity,
    pub microchip: Option<MicrochipInfo>,
    pub guardian: Option<GuardianInfo>,
    pub medical_records: MedicalRecords,
    pub genetics: Option<GeneticProfile>,
    pub travel_history: Vec<TravelRecord>,
    pub current_documents: Vec<TravelDocument>,
    pub verification: Option<Verification>,
}

impl PetHealthPassport {
    /// 새 여권 생성
    pub fn new(identity: PetIdentity) -> Self {
        Self {
            header: PassportHeader::default(),
            identity,
            microchip: None,
            guardian: None,
            medical_records: MedicalRecords::default(),
            genetics: None,
            travel_history: Vec::new(),
            current_documents: Vec::new(),
            verification: None,
        }
    }

    /// 여권 ID 조회
    pub fn passport_id(&self) -> Uuid {
        self.identity.passport_id
    }

    /// 마지막 업데이트 시간 갱신
    pub fn touch(&mut self) {
        self.header.last_updated = Utc::now();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_passport() {
        let identity = PetIdentity {
            name: "Max".to_string(),
            species: PetSpecies::Dog,
            breed: Some("Golden Retriever".to_string()),
            sex: PetSex::NeuteredMale,
            ..Default::default()
        };

        let passport = PetHealthPassport::new(identity);

        assert_eq!(passport.identity.name, "Max");
        assert_eq!(passport.identity.species, PetSpecies::Dog);
        assert_eq!(passport.header.magic, "WIAPET");
    }

    #[test]
    fn test_passport_serialization() {
        let passport = PetHealthPassport::new(PetIdentity::default());
        let json = serde_json::to_string(&passport);
        assert!(json.is_ok());
    }
}
