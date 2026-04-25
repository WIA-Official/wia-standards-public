//! WIA Refugee Credential - Type Definitions
//!
//! 국가가 무너져도, 사람의 가치는 무너지지 않습니다.
//!
//! 홍익인간 (弘益人間) - Benefit All Humanity

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

// ============================================================================
// Root Types
// ============================================================================

/// Refugee Credential - Root structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RefugeeCredential {
    pub id: String,
    pub version: String,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,

    pub identity: HolderIdentity,
    pub education: Vec<EducationRecord>,
    pub competencies: Vec<CompetencyRecord>,
    pub languages: Vec<LanguageAbility>,
    pub experience: Vec<WorkExperience>,

    pub verification_level: u8,
    pub verifications: Vec<Verification>,

    pub signature: Option<QuantumResistantSignature>,
}

// ============================================================================
// Identity Types
// ============================================================================

/// Holder Identity (privacy-preserving)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HolderIdentity {
    pub holder_did: String,
    pub name_hash: String,
    pub display_name: Option<String>,
    pub photo_hash: Option<String>,

    pub birth_year: u32,
    pub gender: Option<Gender>,
    pub nationality_claimed: String,

    pub refugee_status: RefugeeStatus,
    pub displacement_date: Option<DateTime<Utc>>,
    pub displacement_reason: Option<DisplacementReason>,
    pub current_country: String,

    pub anonymous_mode: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Gender {
    Male,
    Female,
    Other,
    PreferNotToSay,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RefugeeStatus {
    UNHCRRecognized,
    AsylumSeeker,
    InternallyDisplaced,
    Stateless,
    SelfDeclared,
    Returnee,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisplacementReason {
    War,
    Persecution,
    NaturalDisaster,
    Climate,
    Famine,
    PoliticalInstability,
    Other,
}

// ============================================================================
// Education Types
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EducationRecord {
    pub id: String,
    pub level: EducationLevel,
    pub field_of_study: String,
    pub field_code: Option<String>,

    pub institution_name: String,
    pub institution_name_local: Option<String>,
    pub institution_type: InstitutionType,
    pub institution_country: String,
    pub institution_city: Option<String>,

    pub year_start: u32,
    pub year_end: Option<u32>,
    pub status: EducationStatus,

    pub qualification_name: Option<String>,
    pub grade: Option<String>,

    pub verification: VerificationInfo,
    pub evidence: Vec<Evidence>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EducationLevel {
    Primary,
    Secondary,
    HighSchool,
    Vocational,
    Associate,
    Bachelors,
    Masters,
    Doctorate,
    PostDoctorate,
    Professional,
    Certificate,
    Other,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InstitutionType {
    University,
    College,
    TechnicalSchool,
    HighSchool,
    PrimarySchool,
    MedicalSchool,
    LawSchool,
    Seminary,
    Military,
    Online,
    Other,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EducationStatus {
    Completed,
    Incomplete,
    InProgress,
    Unknown,
}

// ============================================================================
// Competency Types
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CompetencyRecord {
    pub id: String,
    pub domain: CompetencyDomain,
    pub domain_specific: Option<String>,
    pub skill_name: String,
    pub skill_description: String,

    pub level: ProficiencyLevel,
    pub years_of_practice: u32,

    pub assessment: Option<CompetencyAssessment>,
    pub verification: VerificationInfo,
    pub evidence: Vec<Evidence>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CompetencyDomain {
    Medicine,
    Nursing,
    Pharmacy,
    Dentistry,
    CivilEngineering,
    MechanicalEngineering,
    ElectricalEngineering,
    SoftwareEngineering,
    Teaching,
    Law,
    Accounting,
    Finance,
    Biology,
    Chemistry,
    Physics,
    Mathematics,
    Construction,
    Automotive,
    Agriculture,
    IT,
    SocialWork,
    Other,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProficiencyLevel {
    Novice,
    Beginner,
    Competent,
    Proficient,
    Expert,
    Master,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CompetencyAssessment {
    pub assessment_type: AssessmentType,
    pub assessment_date: DateTime<Utc>,
    pub assessor: String,
    pub assessor_type: AssessorType,
    pub score: Option<f64>,
    pub score_interpretation: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssessmentType {
    WrittenTest,
    PracticalTest,
    OralExam,
    Simulation,
    PortfolioReview,
    WorkSampleReview,
    TechnicalInterview,
    BehavioralInterview,
    OnTheJobObservation,
    SelfAssessment,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssessorType {
    WIACertified,
    UniversityPartner,
    ProfessionalBody,
    EmployerPartner,
    PeerGroup,
    Self_,
}

// ============================================================================
// Language Types
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LanguageAbility {
    pub language_code: String,
    pub language_name: String,

    pub speaking: CEFRLevel,
    pub listening: CEFRLevel,
    pub reading: CEFRLevel,
    pub writing: CEFRLevel,

    pub is_native: bool,
    pub certification: Option<LanguageCertification>,
    pub verification: VerificationInfo,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CEFRLevel {
    A1,
    A2,
    B1,
    B2,
    C1,
    C2,
    Native,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LanguageCertification {
    pub certification_name: String,
    pub score: Option<String>,
    pub date: DateTime<Utc>,
    pub issuer: String,
}

// ============================================================================
// Work Experience Types
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorkExperience {
    pub id: String,
    pub job_title: String,
    pub job_title_local: Option<String>,
    pub job_category: JobCategory,

    pub employer_name: String,
    pub employer_type: EmployerType,
    pub employer_country: String,
    pub employer_city: Option<String>,
    pub industry: String,

    pub start_date: DateTime<Utc>,
    pub end_date: Option<DateTime<Utc>>,
    pub is_current: bool,
    pub total_months: u32,

    pub description: String,
    pub key_responsibilities: Vec<String>,
    pub achievements: Vec<String>,
    pub skills_used: Vec<String>,

    pub verification: VerificationInfo,
    pub evidence: Vec<Evidence>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JobCategory {
    Executive,
    Management,
    Professional,
    Technical,
    Administrative,
    Skilled,
    SemiSkilled,
    Unskilled,
    SelfEmployed,
    Academic,
    Medical,
    Legal,
    Military,
    Government,
    NGO,
    Other,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EmployerType {
    PublicSector,
    PrivateSector,
    NGO,
    InternationalOrg,
    SelfEmployed,
    Military,
    Academic,
    Hospital,
    Other,
}

// ============================================================================
// Verification Types
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VerificationInfo {
    pub level: u8,
    pub confidence_score: f64,
    pub method: VerificationMethod,
    pub method_details: Option<String>,

    pub verifier_name: String,
    pub verifier_type: VerifierType,
    pub verifier_id: Option<String>,

    pub verification_date: DateTime<Utc>,
    pub valid_until: Option<DateTime<Utc>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Verification {
    pub id: String,
    pub verification_type: VerificationType,
    pub level: u8,
    pub confidence: f64,
    pub date: DateTime<Utc>,
    pub verifier: String,
    pub fields_verified: Vec<String>,
    pub notes: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VerificationType {
    SelfDeclaration,
    PeerVerification,
    Assessment,
    DocumentVerification,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VerificationMethod {
    SelfDeclaration,
    PeerAttestation,
    ColleagueConfirmation,
    AlumniNetwork,
    OnlineAssessment,
    PracticalExam,
    PortfolioReview,
    TechnicalInterview,
    Simulation,
    OriginalDocument,
    CertifiedCopy,
    InstitutionVerification,
    DatabaseLookup,
    AIDocumentVerification,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VerifierType {
    Self_,
    Peer,
    WIAAssessor,
    UniversityPartner,
    ProfessionalBody,
    Government,
    UNHCR,
    NGO,
}

// ============================================================================
// Evidence Types
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Evidence {
    pub id: String,
    pub evidence_type: EvidenceType,
    pub title: String,
    pub description: String,

    pub file_hash: Option<String>,
    pub file_type: Option<String>,
    pub file_size_bytes: Option<u64>,

    pub date_submitted: DateTime<Utc>,
    pub authenticity_score: Option<f64>,
    pub is_encrypted: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceType {
    Diploma,
    Transcript,
    Certificate,
    License,
    IDCard,
    Passport,
    EmploymentLetter,
    PayStub,
    BusinessCard,
    Portfolio,
    Publication,
    Patent,
    PhotoColleagues,
    PhotoWorkplace,
    PhotoCertificate,
    LinkedInProfile,
    OnlinePortfolio,
    GithubProfile,
    ResearchProfile,
    WrittenStatement,
    VideoStatement,
    PeerAttestation,
    EmployerReference,
    Other,
}

// ============================================================================
// Security Types
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuantumResistantSignature {
    pub algorithm: SignatureAlgorithm,
    pub public_key: String,
    pub signature: String,
    pub signed_fields: Vec<String>,
    pub signed_at: DateTime<Utc>,
    pub signer_did: String,
    pub valid_until: DateTime<Utc>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SignatureAlgorithm {
    #[serde(rename = "CRYSTALS-Dilithium")]
    CrystalsDilithium,
    #[serde(rename = "SPHINCS+")]
    SphincsPlus,
    Falcon,
}

// ============================================================================
// Result Types
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfidenceResult {
    pub overall_confidence: f64,
    pub base_confidence: f64,
    pub education_confidence: f64,
    pub competency_confidence: f64,
    pub experience_confidence: f64,
    pub classification: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PeerMatch {
    pub peer_id: String,
    pub match_type: String,
    pub match_score: f64,
    pub shared_context: PeerContext,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PeerContext {
    pub institution: Option<String>,
    pub employer: Option<String>,
    pub years: Option<String>,
    pub field: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssessmentResult {
    pub raw_score: f64,
    pub normalized_score: f64,
    pub claimed_level: ProficiencyLevel,
    pub achieved_level: ProficiencyLevel,
    pub verification_status: String,
    pub confidence: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DocumentVerificationResult {
    pub document_id: String,
    pub authenticity_score: f64,
    pub tampering_detected: bool,
    pub text_verified: bool,
    pub confidence: f64,
}
