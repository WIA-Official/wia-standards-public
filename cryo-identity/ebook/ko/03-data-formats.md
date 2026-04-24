# 제3장: 데이터 형식

## 개요

WIA-CRYO-IDENTITY 표준은 냉동보존 시설에서 대상자 신원을 관리하기 위한 포괄적인 데이터 스키마를 정의합니다. 이 장에서는 핵심 데이터 구조, 유효성 검사 규칙, 그리고 상호운용성을 위한 형식 지침을 다룹니다.

## 핵심 스키마

### 대상자 스키마

```typescript
import { z } from 'zod';

// 대상자 유형 정의
const SubjectTypeSchema = z.enum([
  'individual',      // 성인 개인
  'minor',          // 미성년자
  'incapacitated',  // 의사결정 무능력자
  'posthumous'      // 사후 대상자
]);

// 대상자 상태 정의
const SubjectStatusSchema = z.enum([
  'pending',    // 등록 대기
  'active',     // 활성
  'suspended',  // 일시 중지
  'inactive',   // 비활성
  'deceased'    // 사망
]);

// 식별자 유형
const IdentifierTypeSchema = z.enum([
  'internal',       // 내부 ID
  'national-id',    // 주민등록번호/국가 ID
  'passport',       // 여권
  'medical-record', // 의료 기록 번호
  'donor-id',       // 기증자 ID
  'anonymous'       // 익명 식별자
]);

// 대상자 식별자 스키마
const SubjectIdentifierSchema = z.object({
  type: IdentifierTypeSchema,
  value: z.string().min(1).max(255),
  issuingAuthority: z.string().optional(),
  issueDate: z.coerce.date().optional(),
  expirationDate: z.coerce.date().optional(),
  verified: z.boolean().default(false),
  verifiedAt: z.coerce.date().optional(),
  verifiedBy: z.string().optional(),
  metadata: z.record(z.unknown()).optional()
}).refine(
  (data) => {
    if (data.issueDate && data.expirationDate) {
      return data.expirationDate > data.issueDate;
    }
    return true;
  },
  { message: '만료일은 발급일 이후여야 합니다' }
);

// 법적 이름 스키마
const LegalNameSchema = z.object({
  given: z.string().min(1).max(100),
  family: z.string().min(1).max(100),
  middle: z.string().max(100).optional(),
  prefix: z.string().max(20).optional(),
  suffix: z.string().max(20).optional(),
  preferredName: z.string().max(100).optional(),
  maidenName: z.string().max(100).optional(),
  aliases: z.array(z.string().max(100)).optional(),
  nativeScript: z.string().max(200).optional(), // 원어 이름 (예: 한글)
  romanization: z.string().max(200).optional()  // 로마자 표기
});

// 주소 스키마
const AddressSchema = z.object({
  type: z.enum(['home', 'work', 'mailing', 'legal']),
  line1: z.string().max(255),
  line2: z.string().max(255).optional(),
  city: z.string().max(100),
  state: z.string().max(100).optional(),
  postalCode: z.string().max(20),
  country: z.string().length(2), // ISO 3166-1 alpha-2
  validFrom: z.coerce.date().optional(),
  validUntil: z.coerce.date().optional(),
  primary: z.boolean().default(false)
});

// 연락처 스키마
const ContactInfoSchema = z.object({
  phones: z.array(z.object({
    type: z.enum(['mobile', 'home', 'work', 'fax']),
    number: z.string().regex(/^\+?[1-9]\d{1,14}$/), // E.164 형식
    primary: z.boolean().default(false)
  })).optional(),
  emails: z.array(z.object({
    type: z.enum(['personal', 'work']),
    address: z.string().email(),
    verified: z.boolean().default(false),
    primary: z.boolean().default(false)
  })).optional(),
  addresses: z.array(AddressSchema).optional(),
  preferredLanguage: z.string().length(2).default('ko'), // ISO 639-1
  preferredContactMethod: z.enum(['phone', 'email', 'mail']).default('email')
});

// 비상 연락처 스키마
const EmergencyContactSchema = z.object({
  name: LegalNameSchema,
  relationship: z.string().max(50),
  phone: z.string().regex(/^\+?[1-9]\d{1,14}$/),
  email: z.string().email().optional(),
  priority: z.number().int().min(1).max(5),
  canMakeDecisions: z.boolean().default(false),
  legalAuthority: z.string().optional()
});

// 완전한 대상자 스키마
const SubjectSchema = z.object({
  id: z.string().uuid(),
  externalId: z.string().max(255).optional(),
  type: SubjectTypeSchema,
  status: SubjectStatusSchema.default('pending'),
  identifiers: z.array(SubjectIdentifierSchema).min(1),
  profile: z.object({
    legalName: LegalNameSchema,
    dateOfBirth: z.coerce.date().optional(),
    placeOfBirth: z.string().max(255).optional(),
    nationality: z.string().length(2).optional(), // ISO 3166-1 alpha-2
    gender: z.enum(['male', 'female', 'other', 'unknown']).optional(),
    contactInfo: ContactInfoSchema.optional(),
    emergencyContacts: z.array(EmergencyContactSchema).optional()
  }),
  consent: z.object({
    obtainedAt: z.coerce.date(),
    type: z.enum(['written', 'electronic', 'verbal']),
    documentRef: z.string().optional(),
    scope: z.array(z.string()),
    expiresAt: z.coerce.date().optional()
  }).optional(),
  createdAt: z.coerce.date(),
  updatedAt: z.coerce.date(),
  createdBy: z.string(),
  facilityId: z.string().uuid(),
  metadata: z.record(z.unknown()).optional()
});

type Subject = z.infer<typeof SubjectSchema>;
type SubjectIdentifier = z.infer<typeof SubjectIdentifierSchema>;
type LegalName = z.infer<typeof LegalNameSchema>;
```

### 생체정보 스키마

```typescript
// 생체정보 양식
const BiometricModalitySchema = z.enum([
  'fingerprint',  // 지문
  'facial',       // 얼굴
  'iris',         // 홍채
  'voice',        // 음성
  'palm',         // 손바닥
  'dna'           // DNA
]);

// 생체정보 품질 등급
const BiometricQualitySchema = z.object({
  score: z.number().min(0).max(100),
  grade: z.enum(['excellent', 'good', 'acceptable', 'poor', 'unacceptable']),
  factors: z.array(z.object({
    name: z.string(),
    value: z.number(),
    threshold: z.number(),
    passed: z.boolean()
  })).optional()
});

// 생체정보 템플릿 스키마
const BiometricTemplateSchema = z.object({
  id: z.string().uuid(),
  subjectId: z.string().uuid(),
  modality: BiometricModalitySchema,
  template: z.object({
    format: z.string(), // 예: 'ISO-19794-2', 'ANSI-378'
    version: z.string(),
    data: z.string(), // Base64 인코딩된 템플릿
    encryptionAlgorithm: z.string().optional(),
    keyVersion: z.string().optional()
  }),
  quality: BiometricQualitySchema,
  capture: z.object({
    deviceId: z.string(),
    deviceType: z.string(),
    capturedAt: z.coerce.date(),
    capturedBy: z.string(),
    environment: z.object({
      lighting: z.number().optional(),
      temperature: z.number().optional(),
      humidity: z.number().optional()
    }).optional()
  }),
  liveness: z.object({
    performed: z.boolean(),
    passed: z.boolean().optional(),
    score: z.number().optional(),
    method: z.string().optional()
  }).optional(),
  isActive: z.boolean().default(true),
  expiresAt: z.coerce.date().optional(),
  createdAt: z.coerce.date(),
  metadata: z.record(z.unknown()).optional()
});

type BiometricTemplate = z.infer<typeof BiometricTemplateSchema>;

// 생체정보 비교 결과
const BiometricMatchResultSchema = z.object({
  matchId: z.string().uuid(),
  templateId: z.string().uuid(),
  score: z.number().min(0).max(100),
  threshold: z.number(),
  decision: z.enum(['match', 'no-match', 'inconclusive']),
  confidence: z.number().min(0).max(1),
  algorithm: z.object({
    name: z.string(),
    version: z.string(),
    vendor: z.string()
  }),
  matchedFeatures: z.number().optional(),
  totalFeatures: z.number().optional(),
  processingTimeMs: z.number(),
  timestamp: z.coerce.date()
});

type BiometricMatchResult = z.infer<typeof BiometricMatchResultSchema>;

// 지문 특정 스키마
const FingerprintSchema = z.object({
  position: z.enum([
    'right-thumb', 'right-index', 'right-middle', 'right-ring', 'right-little',
    'left-thumb', 'left-index', 'left-middle', 'left-ring', 'left-little'
  ]),
  impressionType: z.enum(['live-scan', 'ink-rolled', 'ink-flat', 'latent']),
  patternClass: z.enum(['arch', 'tented-arch', 'left-loop', 'right-loop', 'whorl']).optional(),
  minutiaeCount: z.number().optional(),
  nfiqScore: z.number().min(1).max(5).optional() // NIST 지문 이미지 품질
});

// 얼굴 특정 스키마
const FacialSchema = z.object({
  poseAngle: z.object({
    yaw: z.number().min(-90).max(90),
    pitch: z.number().min(-90).max(90),
    roll: z.number().min(-180).max(180)
  }).optional(),
  expression: z.enum(['neutral', 'smile', 'other']).optional(),
  eyesOpen: z.boolean().optional(),
  glasses: z.enum(['none', 'regular', 'sunglasses']).optional(),
  faceOcclusion: z.number().min(0).max(100).optional(),
  icaoCompliance: z.boolean().optional()
});

// 홍채 특정 스키마
const IrisSchema = z.object({
  eye: z.enum(['left', 'right']),
  pupilDilation: z.number().min(0).max(1).optional(),
  irisRadius: z.number().optional(),
  occlusionScore: z.number().min(0).max(100).optional()
});
```

### 인증 스키마

```typescript
// 인증 레벨
const VerificationLevelSchema = z.enum([
  'basic',     // 기본: 문서 확인
  'standard',  // 표준: 문서 + 지식 기반
  'enhanced',  // 향상: 문서 + 생체인식
  'maximum'    // 최대: 다중 생체인식 + 제3자
]);

// 인증 방법
const VerificationMethodSchema = z.enum([
  'document',    // 문서 인증
  'biometric',   // 생체인식
  'knowledge',   // 지식 기반 (질문)
  'possession',  // 소유 기반 (OTP 등)
  'third-party'  // 제3자 인증
]);

// 인증 상태
const VerificationStatusSchema = z.enum([
  'pending',      // 대기 중
  'in-progress',  // 진행 중
  'completed',    // 완료
  'failed',       // 실패
  'expired'       // 만료
]);

// 인증 세션 스키마
const VerificationSessionSchema = z.object({
  id: z.string().uuid(),
  subjectId: z.string().uuid(),
  requestedLevel: VerificationLevelSchema,
  requiredMethods: z.array(VerificationMethodSchema),
  completedMethods: z.array(VerificationMethodSchema).default([]),
  status: VerificationStatusSchema.default('pending'),
  context: z.object({
    purpose: z.string(),
    initiator: z.string(),
    facility: z.string(),
    specimenIds: z.array(z.string()).optional(),
    operationType: z.string().optional(),
    riskLevel: z.enum(['low', 'medium', 'high', 'critical'])
  }),
  createdAt: z.coerce.date(),
  updatedAt: z.coerce.date(),
  expiresAt: z.coerce.date(),
  completedAt: z.coerce.date().optional()
});

type VerificationSession = z.infer<typeof VerificationSessionSchema>;

// 인증 결과 스키마
const VerificationResultSchema = z.object({
  id: z.string().uuid(),
  sessionId: z.string().uuid(),
  subjectId: z.string().uuid(),
  method: VerificationMethodSchema,
  status: VerificationStatusSchema,
  confidence: z.number().min(0).max(1),
  evidence: z.array(z.object({
    type: z.string(),
    source: z.string(),
    capturedAt: z.coerce.date(),
    hash: z.string(),
    encryptedData: z.string().optional()
  })),
  reviewer: z.object({
    id: z.string(),
    name: z.string(),
    role: z.string(),
    reviewedAt: z.coerce.date(),
    decision: z.enum(['approved', 'rejected', 'escalated']),
    notes: z.string().optional()
  }).optional(),
  timestamp: z.coerce.date(),
  expiresAt: z.coerce.date(),
  metadata: z.record(z.unknown()).optional()
});

type VerificationResult = z.infer<typeof VerificationResultSchema>;

// 문서 인증 상세
const DocumentVerificationSchema = z.object({
  documentType: z.enum([
    'passport', 'national-id', 'drivers-license',
    'birth-certificate', 'medical-id', 'donor-card'
  ]),
  documentNumber: z.string(),
  issuingCountry: z.string().length(2),
  issuingAuthority: z.string().optional(),
  issueDate: z.coerce.date(),
  expirationDate: z.coerce.date().optional(),
  holderName: LegalNameSchema,
  dateOfBirth: z.coerce.date().optional(),
  verificationChecks: z.array(z.object({
    checkType: z.string(),
    passed: z.boolean(),
    confidence: z.number(),
    details: z.string().optional()
  })),
  fraudIndicators: z.array(z.object({
    type: z.string(),
    severity: z.enum(['low', 'medium', 'high']),
    description: z.string()
  })).optional(),
  ocrConfidence: z.number().optional(),
  imageQuality: z.number().optional()
});

type DocumentVerification = z.infer<typeof DocumentVerificationSchema>;
```

### 관계 스키마

```typescript
// 관계 유형
const RelationshipTypeSchema = z.enum([
  'spouse',               // 배우자
  'parent',               // 부모
  'child',                // 자녀
  'sibling',              // 형제자매
  'guardian',             // 보호자
  'legal-representative', // 법정 대리인
  'emergency-contact',    // 비상 연락처
  'next-of-kin',          // 친족
  'authorized-agent',     // 권한 대리인
  'healthcare-proxy',     // 의료 대리인
  'executor'              // 유언 집행자
]);

// 관계 스키마
const RelationshipSchema = z.object({
  id: z.string().uuid(),
  subjectId: z.string().uuid(),
  relatedSubjectId: z.string().uuid(),
  relationshipType: RelationshipTypeSchema,
  isPrimary: z.boolean().default(false),
  bidirectional: z.boolean().default(true),
  legalAuthority: z.object({
    type: z.string(),
    documentRef: z.string(),
    grantedAt: z.coerce.date(),
    expiresAt: z.coerce.date().optional(),
    scope: z.array(z.string()),
    restrictions: z.array(z.string()).optional()
  }).optional(),
  validFrom: z.coerce.date(),
  validUntil: z.coerce.date().optional(),
  verified: z.boolean().default(false),
  verifiedAt: z.coerce.date().optional(),
  verifiedBy: z.string().optional(),
  createdAt: z.coerce.date(),
  updatedAt: z.coerce.date(),
  metadata: z.record(z.unknown()).optional()
});

type Relationship = z.infer<typeof RelationshipSchema>;

// 법적 지시서 유형
const DirectiveTypeSchema = z.enum([
  'advance-directive',       // 사전 의료 지시서
  'power-of-attorney',       // 위임장
  'specimen-disposition',    // 표본 처분 지시
  'research-consent',        // 연구 동의
  'transfer-authorization',  // 이전 승인
  'posthumous-instructions'  // 사후 지시
]);

// 법적 지시서 스키마
const LegalDirectiveSchema = z.object({
  id: z.string().uuid(),
  subjectId: z.string().uuid(),
  type: DirectiveTypeSchema,
  documentRef: z.string(),
  documentHash: z.string(),
  title: z.string(),
  description: z.string().optional(),
  effectiveDate: z.coerce.date(),
  expirationDate: z.coerce.date().optional(),
  instructions: z.string(),
  scope: z.object({
    specimenTypes: z.array(z.string()).optional(),
    actions: z.array(z.string()),
    conditions: z.array(z.string()).optional()
  }),
  witnesses: z.array(z.object({
    name: z.string(),
    role: z.string(),
    signedAt: z.coerce.date(),
    signatureRef: z.string()
  })).optional(),
  notarized: z.boolean().default(false),
  notaryInfo: z.object({
    name: z.string(),
    commission: z.string(),
    notarizedAt: z.coerce.date()
  }).optional(),
  status: z.enum(['draft', 'active', 'superseded', 'revoked']),
  createdAt: z.coerce.date(),
  updatedAt: z.coerce.date()
});

type LegalDirective = z.infer<typeof LegalDirectiveSchema>;
```

### 동의 스키마

```typescript
// 동의 유형
const ConsentTypeSchema = z.enum([
  'storage',          // 보관 동의
  'research',         // 연구 동의
  'transfer',         // 이전 동의
  'disposal',         // 폐기 동의
  'data-sharing',     // 데이터 공유 동의
  'third-party-access'// 제3자 접근 동의
]);

// 동의 상태
const ConsentStatusSchema = z.enum([
  'pending',   // 대기
  'granted',   // 허가
  'denied',    // 거부
  'withdrawn', // 철회
  'expired'    // 만료
]);

// 동의 기록 스키마
const ConsentRecordSchema = z.object({
  id: z.string().uuid(),
  subjectId: z.string().uuid(),
  consentType: ConsentTypeSchema,
  status: ConsentStatusSchema,
  scope: z.array(z.string()),
  purpose: z.string(),
  legalBasis: z.enum([
    'consent',
    'contract',
    'legal-obligation',
    'vital-interests',
    'public-interest',
    'legitimate-interests'
  ]),
  grantedAt: z.coerce.date().optional(),
  grantedBy: z.object({
    subjectId: z.string().uuid(),
    relationship: z.string().optional(),
    legalAuthority: z.string().optional()
  }).optional(),
  expiresAt: z.coerce.date().optional(),
  withdrawnAt: z.coerce.date().optional(),
  withdrawnBy: z.object({
    subjectId: z.string().uuid(),
    reason: z.string().optional()
  }).optional(),
  documentRef: z.string().optional(),
  documentHash: z.string().optional(),
  witnessedBy: z.array(z.object({
    name: z.string(),
    role: z.string(),
    witnessedAt: z.coerce.date()
  })).optional(),
  verificationSession: z.string().uuid().optional(),
  conditions: z.array(z.object({
    type: z.string(),
    description: z.string(),
    value: z.unknown()
  })).optional(),
  createdAt: z.coerce.date(),
  updatedAt: z.coerce.date(),
  metadata: z.record(z.unknown()).optional()
});

type ConsentRecord = z.infer<typeof ConsentRecordSchema>;

// 동의 검증
class ConsentValidator {
  validateConsent(consent: ConsentRecord): ValidationResult {
    const errors: string[] = [];

    // 상태 검증
    if (consent.status === 'granted') {
      if (!consent.grantedAt) {
        errors.push('허가된 동의에는 허가 일시가 필요합니다');
      }
      if (consent.expiresAt && consent.expiresAt < new Date()) {
        errors.push('동의가 만료되었습니다');
      }
    }

    if (consent.status === 'withdrawn') {
      if (!consent.withdrawnAt) {
        errors.push('철회된 동의에는 철회 일시가 필요합니다');
      }
    }

    // 범위 검증
    if (consent.scope.length === 0) {
      errors.push('동의 범위가 비어있습니다');
    }

    return {
      valid: errors.length === 0,
      errors
    };
  }

  isConsentActive(consent: ConsentRecord): boolean {
    if (consent.status !== 'granted') {
      return false;
    }

    if (consent.expiresAt && consent.expiresAt < new Date()) {
      return false;
    }

    return true;
  }

  checkConsentScope(consent: ConsentRecord, requiredScope: string[]): boolean {
    if (!this.isConsentActive(consent)) {
      return false;
    }

    return requiredScope.every(s => consent.scope.includes(s));
  }
}

interface ValidationResult {
  valid: boolean;
  errors: string[];
}
```

## 데이터 유효성 검사

### 유효성 검사 유틸리티

```typescript
// 중앙 유효성 검사 서비스
class SchemaValidationService {
  private validators: Map<string, z.ZodSchema> = new Map();

  constructor() {
    this.registerSchemas();
  }

  private registerSchemas(): void {
    this.validators.set('subject', SubjectSchema);
    this.validators.set('biometric-template', BiometricTemplateSchema);
    this.validators.set('verification-session', VerificationSessionSchema);
    this.validators.set('verification-result', VerificationResultSchema);
    this.validators.set('relationship', RelationshipSchema);
    this.validators.set('consent', ConsentRecordSchema);
    this.validators.set('legal-directive', LegalDirectiveSchema);
  }

  validate<T>(schemaName: string, data: unknown): SchemaValidationResult<T> {
    const schema = this.validators.get(schemaName);
    if (!schema) {
      return {
        success: false,
        errors: [`알 수 없는 스키마: ${schemaName}`]
      };
    }

    const result = schema.safeParse(data);

    if (result.success) {
      return {
        success: true,
        data: result.data as T
      };
    }

    return {
      success: false,
      errors: result.error.errors.map(e =>
        `${e.path.join('.')}: ${e.message}`
      )
    };
  }

  validatePartial<T>(schemaName: string, data: unknown): SchemaValidationResult<Partial<T>> {
    const schema = this.validators.get(schemaName);
    if (!schema) {
      return {
        success: false,
        errors: [`알 수 없는 스키마: ${schemaName}`]
      };
    }

    // 부분 검증을 위한 스키마 생성
    const partialSchema = (schema as z.ZodObject<any>).partial();
    const result = partialSchema.safeParse(data);

    if (result.success) {
      return {
        success: true,
        data: result.data as Partial<T>
      };
    }

    return {
      success: false,
      errors: result.error.errors.map(e =>
        `${e.path.join('.')}: ${e.message}`
      )
    };
  }

  getSchema(schemaName: string): z.ZodSchema | undefined {
    return this.validators.get(schemaName);
  }

  listSchemas(): string[] {
    return Array.from(this.validators.keys());
  }
}

interface SchemaValidationResult<T> {
  success: boolean;
  data?: T;
  errors?: string[];
}

// 사용 예시
const validationService = new SchemaValidationService();

// 대상자 유효성 검사
const subjectData = {
  id: '550e8400-e29b-41d4-a716-446655440000',
  type: 'individual',
  status: 'active',
  identifiers: [{
    type: 'national-id',
    value: '123456-1234567',
    verified: true
  }],
  profile: {
    legalName: {
      given: '길동',
      family: '홍',
      nativeScript: '홍길동'
    },
    dateOfBirth: new Date('1990-01-15'),
    nationality: 'KR'
  },
  createdAt: new Date(),
  updatedAt: new Date(),
  createdBy: 'admin',
  facilityId: '550e8400-e29b-41d4-a716-446655440001'
};

const result = validationService.validate<Subject>('subject', subjectData);
if (result.success) {
  console.log('유효한 대상자:', result.data);
} else {
  console.log('유효성 검사 실패:', result.errors);
}
```

## 데이터 변환

### 형식 변환 유틸리티

```typescript
// 데이터 변환 서비스
class DataTransformationService {
  // JSON에서 HL7 FHIR Patient로 변환
  toFHIRPatient(subject: Subject): FHIRPatient {
    return {
      resourceType: 'Patient',
      id: subject.id,
      identifier: subject.identifiers.map(id => ({
        system: this.getIdentifierSystem(id.type),
        value: id.value,
        period: {
          start: id.issueDate?.toISOString(),
          end: id.expirationDate?.toISOString()
        }
      })),
      name: [{
        use: 'official',
        family: subject.profile.legalName.family,
        given: [
          subject.profile.legalName.given,
          subject.profile.legalName.middle
        ].filter(Boolean) as string[],
        prefix: subject.profile.legalName.prefix
          ? [subject.profile.legalName.prefix]
          : undefined,
        suffix: subject.profile.legalName.suffix
          ? [subject.profile.legalName.suffix]
          : undefined
      }],
      birthDate: subject.profile.dateOfBirth?.toISOString().split('T')[0],
      gender: this.mapGender(subject.profile.gender)
    };
  }

  // HL7 FHIR Patient에서 Subject로 변환
  fromFHIRPatient(patient: FHIRPatient): Partial<Subject> {
    const officialName = patient.name?.find(n => n.use === 'official') || patient.name?.[0];

    return {
      id: patient.id,
      identifiers: patient.identifier?.map(id => ({
        type: this.getIdentifierType(id.system || ''),
        value: id.value || '',
        verified: false
      })) || [],
      profile: {
        legalName: {
          given: officialName?.given?.[0] || '',
          family: officialName?.family || '',
          middle: officialName?.given?.[1],
          prefix: officialName?.prefix?.[0],
          suffix: officialName?.suffix?.[0]
        },
        dateOfBirth: patient.birthDate
          ? new Date(patient.birthDate)
          : undefined,
        gender: this.reverseMapGender(patient.gender)
      }
    };
  }

  // 개인정보 마스킹
  maskSensitiveData(subject: Subject, level: MaskingLevel): MaskedSubject {
    const masked = JSON.parse(JSON.stringify(subject)) as MaskedSubject;

    switch (level) {
      case 'full':
        // 모든 PII 마스킹
        masked.profile.legalName = {
          given: '***',
          family: '***'
        };
        masked.identifiers = masked.identifiers.map(id => ({
          ...id,
          value: this.maskValue(id.value, id.type)
        }));
        if (masked.profile.dateOfBirth) {
          masked.profile.dateOfBirth = undefined;
        }
        break;

      case 'partial':
        // 부분 마스킹
        masked.identifiers = masked.identifiers.map(id => ({
          ...id,
          value: this.partialMask(id.value)
        }));
        break;

      case 'pseudonymized':
        // 가명화
        masked.id = this.generatePseudonym(masked.id);
        masked.identifiers = masked.identifiers.map(id => ({
          ...id,
          value: this.generatePseudonym(id.value)
        }));
        break;
    }

    return masked;
  }

  private getIdentifierSystem(type: string): string {
    const systems: Record<string, string> = {
      'national-id': 'urn:oid:2.16.840.1.113883.2.19.2.0',
      'passport': 'urn:iso:std:iso:3166',
      'medical-record': 'urn:oid:2.16.840.1.113883.2.19.1.0',
      'internal': 'urn:cryo-identity:internal'
    };
    return systems[type] || 'urn:cryo-identity:unknown';
  }

  private getIdentifierType(system: string): string {
    const types: Record<string, string> = {
      'urn:oid:2.16.840.1.113883.2.19.2.0': 'national-id',
      'urn:iso:std:iso:3166': 'passport',
      'urn:oid:2.16.840.1.113883.2.19.1.0': 'medical-record',
      'urn:cryo-identity:internal': 'internal'
    };
    return types[system] || 'internal';
  }

  private mapGender(gender?: string): string {
    const genderMap: Record<string, string> = {
      'male': 'male',
      'female': 'female',
      'other': 'other',
      'unknown': 'unknown'
    };
    return genderMap[gender || 'unknown'];
  }

  private reverseMapGender(gender?: string): 'male' | 'female' | 'other' | 'unknown' {
    if (gender === 'male' || gender === 'female' || gender === 'other') {
      return gender;
    }
    return 'unknown';
  }

  private maskValue(value: string, type: string): string {
    if (type === 'national-id') {
      // 주민등록번호: 앞 6자리만 표시
      return value.substring(0, 6) + '-*******';
    }
    return '****' + value.slice(-4);
  }

  private partialMask(value: string): string {
    if (value.length <= 4) return '****';
    return value.substring(0, 2) + '****' + value.slice(-2);
  }

  private generatePseudonym(value: string): string {
    // 실제로는 일관된 해싱 사용
    const hash = require('crypto')
      .createHash('sha256')
      .update(value)
      .digest('hex');
    return 'PSN-' + hash.substring(0, 16);
  }
}

type MaskingLevel = 'full' | 'partial' | 'pseudonymized';
type MaskedSubject = Subject;

interface FHIRPatient {
  resourceType: 'Patient';
  id?: string;
  identifier?: {
    system?: string;
    value?: string;
    period?: {
      start?: string;
      end?: string;
    };
  }[];
  name?: {
    use?: string;
    family?: string;
    given?: string[];
    prefix?: string[];
    suffix?: string[];
  }[];
  birthDate?: string;
  gender?: string;
}
```

## 요약

이 장에서 다룬 내용:

1. **핵심 스키마**: 대상자, 식별자, 프로필의 완전한 타입 정의
2. **생체정보 스키마**: 지문, 얼굴, 홍채 등 다양한 양식 지원
3. **인증 스키마**: 인증 세션, 결과, 문서 검증 구조
4. **관계 스키마**: 가족 관계, 법정 대리인, 법적 지시서
5. **동의 스키마**: 다양한 동의 유형과 범위 관리
6. **유효성 검사**: Zod 기반 스키마 검증 유틸리티
7. **데이터 변환**: FHIR 변환 및 개인정보 마스킹

다음 장에서는 REST API 및 GraphQL 인터페이스를 상세히 다룹니다.
