# WIA-REFUGEE-CREDENTIAL: Phase 1 - Data Format

**난민 자격증명 데이터 형식**
*Universal Credential Schema for Displaced Persons*

> "국가가 무너져도, 사람의 가치는 무너지지 않습니다."

홍익인간 (弘益人間) - Benefit All Humanity

---

## 1. Overview

### 1.1 The Problem

```
시리아 의사 → 10년 공부 → 전쟁 → 서류 없음 → 독일에서 청소부
우크라이나 교수 → 30년 경력 → 전쟁 → 서류 없음 → 폴란드에서 공장 노동자
아프간 교사 → 20년 경력 → 탈레반 → 서류 없음 → 파키스탄에서 일용직
```

국가가 무너지면, 그 사람의 역사도 같이 사라집니다.

### 1.2 The Solution

WIA-REFUGEE-CREDENTIAL은 국가 독립적인, 영구 보존되는, 역량 기반의 자격증명 시스템입니다.

### 1.3 Core Principles

| 원칙 | 설명 |
|------|------|
| 국가 독립성 | 특정 국가에 의존하지 않음 |
| 영구 보존 | WIA-PQ-CRYPTO 양자내성 암호로 저장 |
| 역량 기반 | 서류 없이도 실력으로 증명 가능 |
| 무료 | 난민에게 모든 서비스 무료 |
| 호환성 | 전 세계 교육기관과 호환 |

---

## 2. Core Data Structures

### 2.1 RefugeeCredential (Root)

```typescript
interface RefugeeCredential {
  // Identifier
  id: UUID;                              // WIA credential ID
  version: string;                       // "1.0.0"

  // Timestamps
  created_at: ISO8601;
  updated_at: ISO8601;

  // Core data
  identity: HolderIdentity;
  education: EducationRecord[];
  competencies: CompetencyRecord[];
  languages: LanguageAbility[];
  experience: WorkExperience[];

  // Verification
  verification_level: VerificationLevel;
  verifications: Verification[];

  // Storage
  storage: StorageInfo;

  // Cryptographic proof
  signature: QuantumResistantSignature;
}
```

### 2.2 HolderIdentity

```typescript
interface HolderIdentity {
  // Decentralized Identifier (DID)
  holder_did: string;                    // "did:wia:..."

  // Basic info (privacy-preserving)
  name_hash: string;                     // SHA-256 of full name
  display_name: string;                  // Optional public name
  photo_hash?: string;                   // Optional biometric

  // Demographics
  birth_year: number;                    // Year only for privacy
  gender?: Gender;
  nationality_claimed: string;           // ISO 3166-1 alpha-2

  // Refugee status
  refugee_status: RefugeeStatus;
  displacement_date?: ISO8601;           // When displaced
  displacement_reason?: DisplacementReason;
  current_country: string;               // Where currently residing

  // Contact (optional, encrypted)
  contact_encrypted?: EncryptedContact;

  // Anonymous verification option
  anonymous_mode: boolean;
}

enum Gender {
  Male = "male",
  Female = "female",
  Other = "other",
  PreferNotToSay = "prefer_not_to_say"
}

enum RefugeeStatus {
  // UNHCR recognized
  UNHCRRecognized = "unhcr_recognized",
  // Asylum seeker
  AsylumSeeker = "asylum_seeker",
  // Internally displaced
  InternallyDisplaced = "internally_displaced",
  // Stateless
  Stateless = "stateless",
  // Self-declared (awaiting verification)
  SelfDeclared = "self_declared",
  // Returnee
  Returnee = "returnee"
}

enum DisplacementReason {
  War = "war",
  Persecution = "persecution",
  NaturalDisaster = "natural_disaster",
  Climate = "climate",
  Famine = "famine",
  PoliticalInstability = "political_instability",
  Other = "other"
}

interface EncryptedContact {
  email_encrypted: string;
  phone_encrypted: string;
  address_encrypted: string;
  encryption_algorithm: string;          // "AES-256-GCM"
  public_key_id: string;                 // Key used for encryption
}
```

### 2.3 EducationRecord

```typescript
interface EducationRecord {
  id: UUID;

  // Education level
  level: EducationLevel;
  field_of_study: string;                // Major/specialization
  field_code?: string;                   // ISCED-F 2013 code

  // Institution (may be from memory)
  institution_name: string;
  institution_name_local?: string;       // In local language
  institution_type: InstitutionType;
  institution_country: string;           // ISO 3166-1
  institution_city?: string;

  // Duration
  year_start: number;
  year_end?: number;                     // Null if incomplete
  status: EducationStatus;

  // Qualification
  qualification_name?: string;           // e.g., "Bachelor of Medicine"
  qualification_name_local?: string;
  grade?: string;                        // GPA or grade
  thesis_title?: string;

  // Verification
  verification: VerificationInfo;

  // Supporting evidence (what they have)
  evidence: Evidence[];

  // Notes
  notes?: string;                        // Additional context
}

enum EducationLevel {
  Primary = "primary",
  Secondary = "secondary",
  HighSchool = "high_school",
  VocationalTraining = "vocational",
  AssociateDegree = "associate",
  BachelorsDegree = "bachelors",
  MastersDegree = "masters",
  Doctorate = "doctorate",
  PostDoctorate = "post_doctorate",
  ProfessionalDegree = "professional",   // MD, JD, etc.
  Certificate = "certificate",
  Other = "other"
}

enum InstitutionType {
  University = "university",
  College = "college",
  TechnicalSchool = "technical_school",
  HighSchool = "high_school",
  PrimarySchool = "primary_school",
  MedicalSchool = "medical_school",
  LawSchool = "law_school",
  Seminary = "seminary",
  Military = "military",
  Online = "online",
  Other = "other"
}

enum EducationStatus {
  Completed = "completed",
  Incomplete = "incomplete",             // Left before completion
  InProgress = "in_progress",            // Still studying (unlikely for refugees)
  Unknown = "unknown"                    // Cannot remember/verify
}
```

### 2.4 CompetencyRecord

```typescript
interface CompetencyRecord {
  id: UUID;

  // Competency details
  domain: CompetencyDomain;
  domain_specific?: string;              // e.g., "Cardiology" for Medicine
  skill_name: string;
  skill_description: string;

  // Proficiency level
  level: ProficiencyLevel;
  years_of_practice: number;

  // Assessment
  assessment: CompetencyAssessment;

  // Evidence
  evidence: Evidence[];

  // Verification
  verification: VerificationInfo;
}

enum CompetencyDomain {
  // Healthcare
  Medicine = "medicine",
  Nursing = "nursing",
  Pharmacy = "pharmacy",
  Dentistry = "dentistry",

  // Engineering
  CivilEngineering = "civil_engineering",
  MechanicalEngineering = "mechanical_engineering",
  ElectricalEngineering = "electrical_engineering",
  SoftwareEngineering = "software_engineering",
  ChemicalEngineering = "chemical_engineering",

  // Education
  Teaching = "teaching",
  EarlyChildhood = "early_childhood",
  SpecialEducation = "special_education",

  // Law & Governance
  Law = "law",
  PublicAdministration = "public_administration",

  // Business
  Accounting = "accounting",
  Finance = "finance",
  Management = "management",
  Marketing = "marketing",

  // Sciences
  Biology = "biology",
  Chemistry = "chemistry",
  Physics = "physics",
  Mathematics = "mathematics",

  // Arts & Humanities
  Literature = "literature",
  History = "history",
  Philosophy = "philosophy",
  Music = "music",
  Art = "art",

  // Trades
  Construction = "construction",
  Automotive = "automotive",
  Electrical = "electrical",
  Plumbing = "plumbing",
  Agriculture = "agriculture",

  // Other
  IT = "it",
  Healthcare = "healthcare",
  SocialWork = "social_work",
  Other = "other"
}

enum ProficiencyLevel {
  Novice = "novice",                     // Basic awareness
  Beginner = "beginner",                 // Limited experience
  Competent = "competent",               // Independent work
  Proficient = "proficient",             // Complex work
  Expert = "expert",                     // Can teach others
  Master = "master"                      // Industry leader
}

interface CompetencyAssessment {
  assessment_type: AssessmentType;
  assessment_date: ISO8601;
  assessor: string;                      // Organization/person
  assessor_type: AssessorType;
  score?: number;                        // 0-100
  score_interpretation?: string;
  duration_hours?: number;
  assessment_id?: string;
  certificate_url?: string;
}

enum AssessmentType {
  // Formal
  WrittenTest = "written_test",
  PracticalTest = "practical_test",
  OralExam = "oral_exam",
  Simulation = "simulation",

  // Portfolio
  PortfolioReview = "portfolio_review",
  WorkSampleReview = "work_sample_review",

  // Interview
  TechnicalInterview = "technical_interview",
  BehavioralInterview = "behavioral_interview",

  // Observation
  OnTheJobObservation = "on_job_observation",
  CaseStudy = "case_study",

  // Self
  SelfAssessment = "self_assessment"
}

enum AssessorType {
  WIACertified = "wia_certified",
  UniversityPartner = "university_partner",
  ProfessionalBody = "professional_body",
  EmployerPartner = "employer_partner",
  PeerGroup = "peer_group",
  Self = "self"
}
```

### 2.5 LanguageAbility

```typescript
interface LanguageAbility {
  language_code: string;                 // ISO 639-1
  language_name: string;                 // e.g., "Arabic"

  // CEFR levels (A1-C2)
  speaking: CEFRLevel;
  listening: CEFRLevel;
  reading: CEFRLevel;
  writing: CEFRLevel;

  // Native language
  is_native: boolean;

  // Certification
  certification?: LanguageCertification;

  // Verification
  verification: VerificationInfo;
}

enum CEFRLevel {
  A1 = "A1",     // Beginner
  A2 = "A2",     // Elementary
  B1 = "B1",     // Intermediate
  B2 = "B2",     // Upper Intermediate
  C1 = "C1",     // Advanced
  C2 = "C2",     // Proficient
  Native = "native"
}

interface LanguageCertification {
  certification_name: string;            // e.g., "IELTS", "TOEFL", "DELE"
  score?: string;
  date: ISO8601;
  issuer: string;
  verification_url?: string;
}
```

### 2.6 WorkExperience

```typescript
interface WorkExperience {
  id: UUID;

  // Position
  job_title: string;
  job_title_local?: string;
  job_category: JobCategory;

  // Employer (may be from memory)
  employer_name: string;
  employer_type: EmployerType;
  employer_country: string;
  employer_city?: string;
  industry: string;

  // Duration
  start_date: ISO8601;                   // Year-month at minimum
  end_date?: ISO8601;
  is_current: boolean;
  total_months: number;

  // Responsibilities
  description: string;
  key_responsibilities: string[];
  achievements: string[];

  // Skills used
  skills_used: string[];

  // Supervision
  supervised_others: boolean;
  team_size?: number;

  // Verification
  verification: VerificationInfo;

  // Evidence
  evidence: Evidence[];
}

enum JobCategory {
  Executive = "executive",
  Management = "management",
  Professional = "professional",
  Technical = "technical",
  Administrative = "administrative",
  Skilled = "skilled",
  SemiSkilled = "semi_skilled",
  Unskilled = "unskilled",
  SelfEmployed = "self_employed",
  Academic = "academic",
  Medical = "medical",
  Legal = "legal",
  Military = "military",
  Government = "government",
  NGO = "ngo",
  Other = "other"
}

enum EmployerType {
  PublicSector = "public_sector",
  PrivateSector = "private_sector",
  NGO = "ngo",
  International = "international_org",
  SelfEmployed = "self_employed",
  Military = "military",
  Academic = "academic",
  Hospital = "hospital",
  Other = "other"
}
```

### 2.7 VerificationInfo

```typescript
interface VerificationInfo {
  // Verification level achieved
  level: VerificationLevel;
  confidence_score: number;              // 0.0 - 1.0

  // Method used
  method: VerificationMethod;
  method_details?: string;

  // Verifier
  verifier_name: string;
  verifier_type: VerifierType;
  verifier_id?: string;
  verifier_country?: string;

  // Timing
  verification_date: ISO8601;
  valid_until?: ISO8601;

  // Evidence of verification
  verification_proof?: string;           // Hash or URL
}

enum VerificationLevel {
  // Level 1: Self-declaration
  Level1_SelfDeclaration = 1,            // Confidence: 0.3

  // Level 2: Peer verification
  Level2_PeerVerification = 2,           // Confidence: 0.6

  // Level 3: Assessment
  Level3_Assessment = 3,                 // Confidence: 0.8

  // Level 4: Document verification
  Level4_DocumentVerification = 4        // Confidence: 0.95
}

enum VerificationMethod {
  // Level 1
  SelfDeclaration = "self_declaration",

  // Level 2
  PeerAttestation = "peer_attestation",
  ColleagueConfirmation = "colleague_confirmation",
  AlumniNetworkVerification = "alumni_network",

  // Level 3
  OnlineAssessment = "online_assessment",
  PracticalExam = "practical_exam",
  PortfolioReview = "portfolio_review",
  TechnicalInterview = "technical_interview",
  SimulationExercise = "simulation",

  // Level 4
  OriginalDocument = "original_document",
  CertifiedCopy = "certified_copy",
  InstitutionDirectVerification = "institution_verification",
  DatabaseLookup = "database_lookup",
  AIDocumentVerification = "ai_document_verification"
}

enum VerifierType {
  Self = "self",
  Peer = "peer",
  WIAAssessor = "wia_assessor",
  UniversityPartner = "university_partner",
  ProfessionalBody = "professional_body",
  Government = "government",
  UNHCR = "unhcr",
  NGO = "ngo"
}
```

### 2.8 Evidence

```typescript
interface Evidence {
  id: UUID;
  evidence_type: EvidenceType;

  // Content
  title: string;
  description: string;

  // File info (if applicable)
  file_hash?: string;                    // SHA-256
  file_type?: string;                    // MIME type
  file_size_bytes?: number;
  storage_location?: StorageLocation;

  // Metadata
  date_created?: ISO8601;
  date_submitted: ISO8601;

  // Authenticity
  authenticity_score?: number;           // AI-assessed 0-1
  ai_verification_result?: AIVerificationResult;

  // Encryption
  is_encrypted: boolean;
  access_control: AccessControl;
}

enum EvidenceType {
  // Documents
  Diploma = "diploma",
  Transcript = "transcript",
  Certificate = "certificate",
  License = "license",
  IDCard = "id_card",
  Passport = "passport",

  // Work evidence
  EmploymentLetter = "employment_letter",
  PayStub = "pay_stub",
  BusinessCard = "business_card",
  Portfolio = "portfolio",
  Publication = "publication",
  Patent = "patent",

  // Photos
  PhotoWithColleagues = "photo_colleagues",
  PhotoAtWorkplace = "photo_workplace",
  PhotoOfCertificate = "photo_certificate",

  // Digital
  LinkedInProfile = "linkedin",
  OnlinePortfolio = "online_portfolio",
  GithubProfile = "github",
  ResearchProfile = "research_profile",

  // Statements
  WrittenStatement = "written_statement",
  VideoStatement = "video_statement",

  // Third party
  PeerAttestation = "peer_attestation",
  EmployerReference = "employer_reference",

  Other = "other"
}

interface AIVerificationResult {
  model_version: string;
  verification_date: ISO8601;
  authenticity_probability: number;
  tampering_detected: boolean;
  confidence_level: string;
  flags: string[];
}

interface AccessControl {
  owner_only: boolean;
  share_with_verifiers: boolean;
  share_with_institutions: boolean;
  share_with_employers: boolean;
  public: boolean;
  specific_dids?: string[];              // Specific DIDs that can access
}
```

### 2.9 StorageInfo

```typescript
interface StorageInfo {
  // Primary storage (WIA servers)
  primary: {
    server_region: string[];             // Multiple regions
    storage_date: ISO8601;
    replication_count: number;
  };

  // Distributed backup
  distributed: {
    ipfs_cid?: string;                   // IPFS Content ID
    arweave_tx?: string;                 // Arweave transaction
    backup_date?: ISO8601;
  };

  // Local backup (user's device)
  local_backup_advised: boolean;
  local_backup_hash?: string;

  // Integrity
  merkle_root: string;
  last_integrity_check: ISO8601;
}
```

### 2.10 QuantumResistantSignature

```typescript
interface QuantumResistantSignature {
  algorithm: "CRYSTALS-Dilithium" | "SPHINCS+" | "FALCON";
  public_key: string;
  signature: string;
  signed_fields: string[];               // Which fields are signed
  signed_at: ISO8601;
  signer_did: string;                    // WIA's DID
  valid_until: ISO8601;                  // Long validity (50+ years)
}
```

---

## 3. JSON Schema

### 3.1 Complete Credential Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.org/schemas/refugee-credential/v1.0.0",
  "title": "WIA Refugee Credential",
  "type": "object",
  "required": [
    "id",
    "version",
    "identity",
    "verification_level",
    "signature"
  ],
  "properties": {
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "identity": {
      "$ref": "#/definitions/HolderIdentity"
    },
    "education": {
      "type": "array",
      "items": { "$ref": "#/definitions/EducationRecord" }
    },
    "competencies": {
      "type": "array",
      "items": { "$ref": "#/definitions/CompetencyRecord" }
    },
    "languages": {
      "type": "array",
      "items": { "$ref": "#/definitions/LanguageAbility" }
    },
    "experience": {
      "type": "array",
      "items": { "$ref": "#/definitions/WorkExperience" }
    },
    "verification_level": {
      "type": "integer",
      "minimum": 1,
      "maximum": 4
    },
    "signature": {
      "$ref": "#/definitions/QuantumResistantSignature"
    }
  }
}
```

---

## 4. Verification Levels

### 4.1 Level 1: Self-Declaration (자기 선언)

```yaml
level: 1
confidence: 0.3
time_to_issue: Immediate
requirements:
  - Basic identity information
  - Education/experience claims
  - Signed declaration of truth

use_cases:
  - Initial credential creation
  - Emergency situations
  - Starting point for further verification
```

### 4.2 Level 2: Peer Verification (동료 검증)

```yaml
level: 2
confidence: 0.6
time_to_issue: Days to weeks
requirements:
  - 2+ peers from same institution/employer
  - Peer identity verification
  - Consistent testimony

peer_network:
  - Alumni networks
  - Professional associations (in exile)
  - Colleague matching system
```

### 4.3 Level 3: Assessment (역량 평가)

```yaml
level: 3
confidence: 0.8
time_to_issue: Days to weeks
requirements:
  - Standardized assessment
  - WIA-certified assessor
  - Practical demonstration

assessment_types:
  - Online tests
  - Portfolio review
  - Technical interview
  - Simulation exercises
  - Practical examinations
```

### 4.4 Level 4: Document Verification (문서 검증)

```yaml
level: 4
confidence: 0.95
time_to_issue: Days to months
requirements:
  - Original or certified documents
  - AI authenticity verification
  - Institution confirmation (if possible)

documents_accepted:
  - Original diploma/certificate
  - Certified copies
  - Digital records from institution
  - Partial documents + corroborating evidence
```

---

## 5. Binary Format (.wiaref)

### 5.1 File Structure

```
┌─────────────────────────────────────┐
│ Magic Number (4 bytes): "WREF"      │
├─────────────────────────────────────┤
│ Version (2 bytes): 0x0100           │
├─────────────────────────────────────┤
│ Header Length (4 bytes)             │
├─────────────────────────────────────┤
│ Header                              │
│  - Credential ID (16 bytes UUID)    │
│  - Holder DID hash (32 bytes)       │
│  - Created timestamp (8 bytes)      │
│  - Verification level (1 byte)      │
│  - Checksum (32 bytes SHA-256)      │
├─────────────────────────────────────┤
│ Encrypted Payload (variable)        │
│  - Uses holder's public key         │
├─────────────────────────────────────┤
│ PQ Signature (variable)             │
│  - CRYSTALS-Dilithium signature     │
└─────────────────────────────────────┘
```

---

## 6. QR Code Format

### 6.1 URL Format (Online Verification)

```
https://credential.wia.org/v/{credential_id}
```

### 6.2 Compact Format (Offline)

```
WIAREF:1.0:{base64url_encoded_essential_data}
```

Essential data includes:
- Credential ID
- Holder DID (truncated)
- Verification level
- Education summary (highest level)
- Key competencies
- Signature

---

## 7. Multi-Language Support

### 7.1 Supported Languages

| Code | Language | Script |
|------|----------|--------|
| ar | العربية (Arabic) | RTL |
| uk | Українська (Ukrainian) | Cyrillic |
| fa | فارسی (Persian/Dari) | RTL |
| ps | پښتو (Pashto) | RTL |
| ti | ትግርኛ (Tigrinya) | Ge'ez |
| so | Soomaali (Somali) | Latin |
| fr | Français (French) | Latin |
| es | Español (Spanish) | Latin |
| en | English | Latin |
| de | Deutsch (German) | Latin |
| tr | Türkçe (Turkish) | Latin |
| ku | Kurdî (Kurdish) | Latin/Arabic |

### 7.2 Localized Fields

All user-facing text fields support multiple languages:

```typescript
interface LocalizedText {
  default: string;                       // Primary language
  translations: {
    [languageCode: string]: string;
  };
}
```

---

## 8. Privacy & Security

### 8.1 Privacy Principles

1. **Minimal Disclosure**: Share only what's needed
2. **Holder Control**: Credential holder controls access
3. **Anonymity Option**: Can verify competency without revealing identity
4. **Right to be Forgotten**: Can request deletion (except immutable blockchain records)

### 8.2 Encryption

```typescript
interface EncryptionScheme {
  // At rest
  storage_encryption: "AES-256-GCM";
  key_derivation: "Argon2id";

  // In transit
  transport_encryption: "TLS 1.3";

  // Long-term
  quantum_resistant: "CRYSTALS-Kyber";
}
```

---

## 9. Use Case Scenarios

### 9.1 Syrian Doctor Scenario

```yaml
persona:
  name: "Dr. Ahmad"
  original_country: "Syria"
  profession: "Cardiologist"
  years_experience: 15
  situation: |
    Medical school records destroyed in bombing.
    Fled to Germany in 2015.
    Working as hospital cleaner.

credential_journey:
  1. Self-Declaration:
     - Enters medical education details from memory
     - Lists hospitals worked at
     - Confidence: 0.3

  2. Peer Verification:
     - Found 3 colleagues from Damascus University Hospital
     - They verify his role and skills
     - Confidence: 0.6

  3. Assessment:
     - Takes WIA medical competency assessment
     - Passes with 85%
     - Confidence: 0.8

  4. Document:
     - Found partial transcript in email
     - AI verification confirms authenticity
     - Confidence: 0.95

outcome:
  - German hospital accepts Level 3+ credential
  - Starts supervised practice
  - Path to full license
```

### 9.2 Ukrainian Professor Scenario

```yaml
persona:
  name: "Prof. Olena"
  original_country: "Ukraine"
  profession: "Computer Science Professor"
  years_experience: 25
  situation: |
    University database destroyed in attack.
    Fled to Poland in 2022.
    Working in warehouse.

credential_journey:
  1. Self-Declaration:
     - Lists PhD, publications, courses taught
     - Confidence: 0.3

  2. Peer Verification:
     - Former students now in EU verify
     - Colleagues in diaspora confirm
     - Confidence: 0.6

  3. Assessment:
     - Submits code portfolio (GitHub)
     - Technical interview passed
     - Confidence: 0.8

  4. Document:
     - Found publications online (Google Scholar)
     - Wayback Machine has university page
     - Confidence: 0.95

outcome:
  - Polish university accepts as visiting researcher
  - Path to professor position
```

---

**Document ID**: WIA-REFUGEE-CREDENTIAL-PHASE-1
**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft

*"시리아 의사는 여전히 의사입니다. 우크라이나 교수는 여전히 교수입니다."*
