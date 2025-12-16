# Phase 1: Pet Health Passport Data Format Specification

## WIA-PET-HEALTH-PASSPORT Data Format Standard

**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft
**Standard ID**: WIA-PET-HEALTH-PASSPORT-PHASE1-001

---

## 1. 개요

WIA-PET-HEALTH-PASSPORT는 전 세계 어디서든 인정되는 반려동물 건강 기록 표준입니다.
백신, 수술, 알레르기, 유전자 정보를 포함하며, 국경 통과 시 자동 검역 인증을 지원합니다.

### 1.1 목적

- 글로벌 반려동물 건강 기록 호환성 확보
- 국경 통과 시 검역 절차 간소화
- 응급 상황 시 수의사 즉시 접근 가능
- 반려동물 복지 향상 및 학대 이력 추적

### 1.2 적용 범위

- 개, 고양이, 토끼, 햄스터 등 모든 반려동물
- 수의사, 보호소, 검역소, 반려동물 보호자
- 193개국 검역 규정 호환

### 1.3 철학

**홍익인간 (弘益人間)** - 인류와 동물 모두를 이롭게 하라

---

## 2. Pet Identity Format

### 2.1 기본 신원 정보

```typescript
interface PetIdentity {
  // 고유 식별자
  passportId: string;           // UUID v7 (시간순 정렬)
  microchipId?: string;         // ISO 11784/11785 마이크로칩 ID (15자리)
  tattooId?: string;            // 문신 ID (일부 국가)

  // 기본 정보
  species: PetSpecies;          // 종
  breed: string;                // 품종
  breedVerified: boolean;       // DNA 검증 여부

  // 개체 정보
  name: string;                 // 이름
  dateOfBirth: ISO8601;         // 생년월일
  dateOfBirthEstimated: boolean; // 추정 여부
  sex: 'male' | 'female' | 'neutered_male' | 'spayed_female';

  // 외형
  color: string;                // 모색
  markings: string[];           // 특징적 무늬
  weight: {
    value: number;              // kg
    measuredAt: ISO8601;
  };

  // 사진 (해시로 무결성 검증)
  photos: PetPhoto[];
}

enum PetSpecies {
  DOG = 'dog',
  CAT = 'cat',
  RABBIT = 'rabbit',
  HAMSTER = 'hamster',
  BIRD = 'bird',
  REPTILE = 'reptile',
  FISH = 'fish',
  OTHER = 'other'
}

interface PetPhoto {
  photoHash: string;            // SHA-256
  capturedAt: ISO8601;
  photoType: 'face' | 'full_body' | 'marking' | 'other';
  storageUri?: string;          // 분산 저장소 URI
}
```

### 2.2 마이크로칩 정보

```typescript
interface MicrochipInfo {
  // ISO 11784/11785 표준
  chipNumber: string;           // 15자리 숫자

  // 제조사 코드 (첫 3자리)
  manufacturerCode: string;     // 국가/제조사

  // 등록 정보
  registeredAt: ISO8601;
  registryName: string;         // 등록 기관
  registryCountry: ISO3166Alpha2;

  // 칩 상태
  status: 'active' | 'replaced' | 'removed' | 'lost';

  // 위치
  implantLocation: 'left_neck' | 'right_neck' | 'between_shoulders' | 'other';
  implantedBy: VeterinarianRef;
  implantedAt: ISO8601;
}
```

### 2.3 보호자 정보

```typescript
interface GuardianInfo {
  guardianId: string;           // WIA-DID

  // 기본 정보
  name: string;
  contactPhone: string;
  contactEmail: string;

  // 주소
  address: {
    country: ISO3166Alpha2;
    postalCode: string;
    city: string;
    addressLine1: string;
    addressLine2?: string;
  };

  // 소유권
  ownershipType: 'owner' | 'guardian' | 'foster' | 'shelter';
  ownershipStartDate: ISO8601;

  // 비상 연락처
  emergencyContacts: EmergencyContact[];

  // 이전 보호자 이력 (프라이버시 보호)
  previousGuardians?: {
    count: number;
    transferDates: ISO8601[];
  };
}

interface EmergencyContact {
  name: string;
  relationship: string;
  phone: string;
  email?: string;
}
```

---

## 3. Medical Records Format

### 3.1 백신 기록

```typescript
interface VaccinationRecord {
  recordId: string;             // UUID

  // 백신 정보
  vaccine: {
    name: string;               // 백신명
    manufacturer: string;       // 제조사
    lotNumber: string;          // 로트 번호
    serialNumber?: string;      // 시리얼 번호
  };

  // 접종 대상 질병
  targetDiseases: VaccineDisease[];

  // 접종 정보
  administeredAt: ISO8601;
  administeredBy: VeterinarianRef;
  clinic: ClinicRef;

  // 유효 기간
  validFrom: ISO8601;
  validUntil: ISO8601;

  // 접종 차수
  doseNumber: number;           // 1, 2, 3...
  totalDoses: number;           // 총 필요 차수

  // 부작용 기록
  adverseReactions?: AdverseReaction[];

  // 검증
  verified: boolean;
  verificationMethod: 'clinic' | 'blockchain' | 'government';
  digitalSignature?: string;
}

enum VaccineDisease {
  // 개
  RABIES = 'rabies',
  DISTEMPER = 'distemper',
  PARVOVIRUS = 'parvovirus',
  HEPATITIS = 'hepatitis',
  PARAINFLUENZA = 'parainfluenza',
  LEPTOSPIROSIS = 'leptospirosis',
  BORDETELLA = 'bordetella',
  LYME = 'lyme',
  CANINE_INFLUENZA = 'canine_influenza',

  // 고양이
  FELINE_PANLEUKOPENIA = 'feline_panleukopenia',
  FELINE_HERPESVIRUS = 'feline_herpesvirus',
  FELINE_CALICIVIRUS = 'feline_calicivirus',
  FELINE_LEUKEMIA = 'feline_leukemia',
  FELINE_IMMUNODEFICIENCY = 'feline_immunodeficiency',

  // 기타
  OTHER = 'other'
}

interface AdverseReaction {
  reactionType: 'mild' | 'moderate' | 'severe';
  symptoms: string[];
  onset: ISO8601;
  duration: string;             // ISO 8601 duration
  treatment?: string;
}
```

### 3.2 수술 및 시술 기록

```typescript
interface SurgeryRecord {
  recordId: string;

  // 수술 정보
  procedureType: ProcedureType;
  procedureName: string;
  description: string;

  // 일시
  performedAt: ISO8601;
  duration: number;             // 분

  // 수의사/병원
  surgeon: VeterinarianRef;
  assistants?: VeterinarianRef[];
  clinic: ClinicRef;

  // 마취 정보
  anesthesia?: {
    type: 'general' | 'local' | 'sedation' | 'none';
    agents: string[];
    duration: number;           // 분
    complications?: string[];
  };

  // 결과
  outcome: 'success' | 'partial' | 'complications' | 'failed';
  complications?: string[];
  followUpRequired: boolean;
  followUpDate?: ISO8601;

  // 회복 지침
  recoveryInstructions?: string;
  restrictions?: string[];
  restrictionEndDate?: ISO8601;
}

enum ProcedureType {
  SPAY = 'spay',
  NEUTER = 'neuter',
  DENTAL = 'dental',
  ORTHOPEDIC = 'orthopedic',
  TUMOR_REMOVAL = 'tumor_removal',
  EMERGENCY = 'emergency',
  DIAGNOSTIC = 'diagnostic',
  COSMETIC = 'cosmetic',
  OTHER = 'other'
}
```

### 3.3 진단 및 검사 기록

```typescript
interface DiagnosticRecord {
  recordId: string;

  // 검사 유형
  testType: DiagnosticTestType;
  testName: string;

  // 일시/장소
  performedAt: ISO8601;
  performedBy: VeterinarianRef;
  clinic: ClinicRef;
  laboratory?: string;

  // 결과
  results: DiagnosticResult[];
  interpretation: string;

  // 첨부 파일
  attachments?: {
    type: 'image' | 'pdf' | 'raw_data';
    hash: string;
    uri: string;
  }[];
}

enum DiagnosticTestType {
  BLOOD_PANEL = 'blood_panel',
  URINALYSIS = 'urinalysis',
  FECAL = 'fecal',
  XRAY = 'xray',
  ULTRASOUND = 'ultrasound',
  MRI = 'mri',
  CT_SCAN = 'ct_scan',
  BIOPSY = 'biopsy',
  ALLERGY_TEST = 'allergy_test',
  GENETIC_TEST = 'genetic_test',
  HEARTWORM = 'heartworm',
  TICK_BORNE = 'tick_borne',
  OTHER = 'other'
}

interface DiagnosticResult {
  parameter: string;            // 검사 항목
  value: string | number;       // 결과값
  unit?: string;                // 단위
  referenceRange?: {
    min: number;
    max: number;
  };
  status: 'normal' | 'low' | 'high' | 'abnormal';
  notes?: string;
}
```

### 3.4 알레르기 및 질환 기록

```typescript
interface AllergyRecord {
  recordId: string;

  // 알레르기 정보
  allergen: string;
  allergenType: AllergenType;

  // 반응
  reactionSeverity: 'mild' | 'moderate' | 'severe' | 'life_threatening';
  symptoms: string[];

  // 진단
  diagnosedAt: ISO8601;
  diagnosedBy: VeterinarianRef;
  diagnosisMethod: 'clinical' | 'test' | 'elimination_diet' | 'owner_report';

  // 관리
  avoidanceInstructions: string;
  emergencyTreatment?: string;

  // 상태
  status: 'active' | 'resolved' | 'suspected';
}

enum AllergenType {
  FOOD = 'food',
  ENVIRONMENTAL = 'environmental',
  MEDICATION = 'medication',
  INSECT = 'insect',
  CONTACT = 'contact',
  OTHER = 'other'
}

interface ChronicCondition {
  conditionId: string;

  // 질환 정보
  conditionName: string;
  icdCode?: string;             // ICD-11 코드

  // 진단
  diagnosedAt: ISO8601;
  diagnosedBy: VeterinarianRef;

  // 상태
  severity: 'mild' | 'moderate' | 'severe';
  status: 'active' | 'managed' | 'remission' | 'resolved';

  // 관리
  managementPlan: string;
  medications: MedicationRecord[];
  monitoringSchedule?: string;

  // 예후
  prognosis?: string;
}
```

### 3.5 투약 기록

```typescript
interface MedicationRecord {
  recordId: string;

  // 약물 정보
  medication: {
    name: string;
    genericName: string;
    manufacturer?: string;
    ndc?: string;               // National Drug Code
  };

  // 처방
  prescribedAt: ISO8601;
  prescribedBy: VeterinarianRef;

  // 용법
  dosage: {
    amount: number;
    unit: string;               // mg, ml, etc.
    frequency: string;          // BID, TID, etc.
    route: 'oral' | 'injection' | 'topical' | 'inhalation' | 'other';
  };

  // 기간
  startDate: ISO8601;
  endDate?: ISO8601;
  asNeeded: boolean;

  // 목적
  indication: string;           // 투약 이유
  relatedCondition?: string;

  // 주의사항
  warnings?: string[];
  interactions?: string[];
}
```

---

## 4. Genetic Information Format

### 4.1 유전자 검사 결과

```typescript
interface GeneticProfile {
  profileId: string;

  // 검사 정보
  testDate: ISO8601;
  laboratory: string;
  testKit?: string;

  // 품종 분석
  breedComposition: BreedComponent[];

  // 유전 질환 위험도
  healthMarkers: GeneticHealthMarker[];

  // 형질
  traits: GeneticTrait[];

  // 혈통
  ancestry?: {
    maternalHaplogroup?: string;
    paternalHaplogroup?: string;
  };

  // 원시 데이터 (선택적)
  rawDataHash?: string;
  rawDataUri?: string;
}

interface BreedComponent {
  breed: string;
  percentage: number;           // 0-100
  confidence: number;           // 0-1
}

interface GeneticHealthMarker {
  marker: string;               // 유전자 마커명
  gene: string;                 // 유전자
  condition: string;            // 관련 질환

  // 결과
  genotype: string;             // AA, Aa, aa 등
  riskLevel: 'clear' | 'carrier' | 'at_risk';

  // 상세
  description: string;
  inheritance: 'autosomal_dominant' | 'autosomal_recessive' | 'x_linked' | 'complex';
  actionRequired?: string;
}

interface GeneticTrait {
  trait: string;                // 형질명
  gene: string;
  genotype: string;
  phenotype: string;            // 예상 표현형
  confidence: number;
}
```

---

## 5. Travel & Quarantine Format

### 5.1 여행 기록

```typescript
interface TravelRecord {
  recordId: string;

  // 여행 정보
  departureCountry: ISO3166Alpha2;
  arrivalCountry: ISO3166Alpha2;

  departureDate: ISO8601;
  arrivalDate: ISO8601;

  // 운송
  transportMethod: 'air' | 'land' | 'sea';
  carrier?: string;             // 항공사/선사 등

  // 검역
  quarantine?: QuarantineRecord;

  // 필요 서류 상태
  requiredDocuments: TravelDocument[];

  // 승인
  approvalStatus: 'pending' | 'approved' | 'rejected';
  approvalAuthority?: string;
}

interface QuarantineRecord {
  facilityName: string;
  facilityCountry: ISO3166Alpha2;

  startDate: ISO8601;
  endDate: ISO8601;
  durationDays: number;

  // 검역 중 검사
  testsPerformed: DiagnosticRecord[];

  // 결과
  outcome: 'released' | 'extended' | 'denied_entry';
  notes?: string;
}

interface TravelDocument {
  documentType: 'health_certificate' | 'import_permit' | 'rabies_titer' | 'parasite_treatment' | 'other';
  documentNumber?: string;
  issuedAt: ISO8601;
  validUntil: ISO8601;
  issuingAuthority: string;
  status: 'valid' | 'expired' | 'pending';
}
```

### 5.2 검역 요구사항 (국가별)

```typescript
interface CountryRequirements {
  country: ISO3166Alpha2;
  lastUpdated: ISO8601;

  // 기본 요구사항
  requirements: {
    // 마이크로칩
    microchipRequired: boolean;
    microchipStandard: 'ISO11784' | 'ISO11785' | 'AVID' | 'any';

    // 광견병
    rabiesVaccination: {
      required: boolean;
      minimumAge: number;        // 개월
      waitPeriod: number;        // 일 (접종 후 대기)
      validityPeriod: number;    // 개월
      titerTestRequired: boolean;
      minimumTiter: number;      // IU/ml
    };

    // 기타 백신
    otherVaccinations: {
      disease: VaccineDisease;
      required: boolean;
      waitPeriod?: number;
    }[];

    // 기생충 치료
    parasiteTreatment?: {
      type: 'tapeworm' | 'tick' | 'other';
      required: boolean;
      timingBeforeEntry: number; // 시간
    };

    // 검역
    quarantine?: {
      required: boolean;
      duration: number;         // 일
      homeQuarantineAllowed: boolean;
    };

    // 금지 품종
    bannedBreeds?: string[];

    // 수입 허가
    importPermitRequired: boolean;

    // 건강 증명서
    healthCertificateRequired: boolean;
    healthCertificateValidDays: number;
  };

  // 예외 국가 (간소화된 요구사항)
  exemptCountries?: ISO3166Alpha2[];
}
```

---

## 6. Passport Document Format

### 6.1 여권 메인 구조

```typescript
interface PetHealthPassport {
  // 헤더
  header: {
    magic: 'WIAPET';
    version: [1, 0, 0];
    standard: 'WIA-PET-HEALTH-PASSPORT';
    createdAt: ISO8601;
    lastUpdated: ISO8601;
  };

  // 신원
  identity: PetIdentity;
  microchip?: MicrochipInfo;
  guardian: GuardianInfo;

  // 의료 기록
  medicalRecords: {
    vaccinations: VaccinationRecord[];
    surgeries: SurgeryRecord[];
    diagnostics: DiagnosticRecord[];
    allergies: AllergyRecord[];
    conditions: ChronicCondition[];
    medications: MedicationRecord[];
  };

  // 유전자 정보
  genetics?: GeneticProfile;

  // 여행 기록
  travel: {
    history: TravelRecord[];
    currentDocuments: TravelDocument[];
  };

  // 검증
  verification: {
    issuingAuthority: string;
    digitalSignature: string;
    certificateChain: string[];
  };

  // 메타데이터
  metadata: {
    qrCodeData: string;         // 빠른 조회용
    lastSyncedAt?: ISO8601;
    syncStatus: 'synced' | 'pending' | 'conflict';
  };
}
```

### 6.2 QR 코드 데이터 형식

```typescript
interface QRCodePayload {
  version: number;

  // 최소 정보 (빠른 스캔용)
  mini: {
    passportId: string;
    microchipId?: string;
    petName: string;
    species: PetSpecies;

    // 광견병 백신 상태
    rabiesValid: boolean;
    rabiesExpiry?: ISO8601;

    // 긴급 연락처
    emergencyPhone: string;
  };

  // 검증
  signature: string;            // 전자 서명
  timestamp: ISO8601;

  // 전체 조회 URL
  fullDataUrl: string;
}
```

---

## 7. Binary Format

### 7.1 컴팩트 바이너리 형식 (.wiapet)

```
┌────────────────────────────────────────────────────┐
│ Header (32 bytes)                                  │
├────────────────────────────────────────────────────┤
│ Magic       │ Version │ Flags  │ Timestamp        │
│ 6 bytes     │ 3 bytes │ 1 byte │ 8 bytes          │
│ 'WIAPET'    │ 1.0.0   │        │ Unix timestamp   │
├────────────────────────────────────────────────────┤
│ DataLength  │ Reserved                             │
│ 4 bytes     │ 10 bytes                             │
├────────────────────────────────────────────────────┤
│ Identity Section (variable)                        │
├────────────────────────────────────────────────────┤
│ Medical Records Section (variable)                 │
├────────────────────────────────────────────────────┤
│ Genetics Section (variable, optional)              │
├────────────────────────────────────────────────────┤
│ Travel Section (variable)                          │
├────────────────────────────────────────────────────┤
│ Signature (64 bytes)                               │
├────────────────────────────────────────────────────┤
│ Checksum (4 bytes) - CRC32                         │
└────────────────────────────────────────────────────┘
```

### 7.2 섹션 인코딩

```typescript
interface BinarySectionHeader {
  sectionType: u8;              // 섹션 유형
  sectionLength: u32;           // 바이트 길이
  compression: u8;              // 0: none, 1: lz4, 2: zstd
  encrypted: u8;                // 0: no, 1: AES-256-GCM
}

enum SectionType {
  IDENTITY = 0x01,
  MICROCHIP = 0x02,
  GUARDIAN = 0x03,
  VACCINATION = 0x10,
  SURGERY = 0x11,
  DIAGNOSTIC = 0x12,
  ALLERGY = 0x13,
  CONDITION = 0x14,
  MEDICATION = 0x15,
  GENETICS = 0x20,
  TRAVEL = 0x30,
  SIGNATURE = 0xFF
}
```

---

## 8. JSON Schema

### 8.1 메인 스키마

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wiastandards.com/schemas/pet-health-passport/v1.json",
  "title": "WIA Pet Health Passport",
  "type": "object",
  "required": ["header", "identity", "guardian", "medicalRecords", "verification"],
  "properties": {
    "header": {
      "type": "object",
      "required": ["magic", "version", "standard", "createdAt"],
      "properties": {
        "magic": { "const": "WIAPET" },
        "version": {
          "type": "array",
          "items": { "type": "integer" },
          "minItems": 3,
          "maxItems": 3
        },
        "standard": { "const": "WIA-PET-HEALTH-PASSPORT" },
        "createdAt": { "type": "string", "format": "date-time" }
      }
    },
    "identity": {
      "$ref": "#/definitions/PetIdentity"
    },
    "medicalRecords": {
      "$ref": "#/definitions/MedicalRecords"
    }
  },
  "definitions": {
    "PetIdentity": {
      "type": "object",
      "required": ["passportId", "species", "name", "sex"],
      "properties": {
        "passportId": { "type": "string", "format": "uuid" },
        "microchipId": {
          "type": "string",
          "pattern": "^[0-9]{15}$"
        },
        "species": {
          "enum": ["dog", "cat", "rabbit", "hamster", "bird", "reptile", "fish", "other"]
        },
        "breed": { "type": "string" },
        "name": { "type": "string", "minLength": 1 },
        "dateOfBirth": { "type": "string", "format": "date" },
        "sex": {
          "enum": ["male", "female", "neutered_male", "spayed_female"]
        }
      }
    },
    "MedicalRecords": {
      "type": "object",
      "properties": {
        "vaccinations": {
          "type": "array",
          "items": { "$ref": "#/definitions/VaccinationRecord" }
        }
      }
    },
    "VaccinationRecord": {
      "type": "object",
      "required": ["recordId", "vaccine", "targetDiseases", "administeredAt"],
      "properties": {
        "recordId": { "type": "string", "format": "uuid" },
        "vaccine": {
          "type": "object",
          "required": ["name"],
          "properties": {
            "name": { "type": "string" },
            "manufacturer": { "type": "string" },
            "lotNumber": { "type": "string" }
          }
        },
        "administeredAt": { "type": "string", "format": "date-time" },
        "validUntil": { "type": "string", "format": "date-time" }
      }
    }
  }
}
```

---

## 9. 호환성

### 9.1 기존 표준과의 매핑

| 기존 표준 | WIA 매핑 |
|-----------|----------|
| EU Pet Passport | 완전 호환 |
| USDA APHIS Form 7001 | 변환 가능 |
| ISO 11784/11785 (마이크로칩) | 직접 호환 |
| IATA Live Animal Regulations | 여행 섹션 매핑 |
| OIE Terrestrial Animal Health Code | 질병 코드 매핑 |

### 9.2 버전 호환성

| 버전 | 호환 |
|------|------|
| 1.0.x | 완전 호환 |
| 1.x.x | 상위 호환 |
| 2.x.x | 마이그레이션 필요 |

---

## 10. 성능 요구사항

| 지표 | 요구사항 | 목표 |
|------|----------|------|
| QR 코드 스캔 | < 500ms | < 200ms |
| 전체 여권 로드 | < 2s | < 500ms |
| 검역 검증 | < 1s | < 300ms |
| 오프라인 검증 | 지원 | 지원 |
| 최대 파일 크기 | 5MB | 1MB |

---

**Document ID**: WIA-PET-HEALTH-PASSPORT-PHASE1-001
**Version**: 1.0.0
**Last Updated**: 2025-12-16
**Copyright**: © 2025 WIA - MIT License

**홍익인간 (弘益人間)** - 인류와 동물 모두를 이롭게 하라
