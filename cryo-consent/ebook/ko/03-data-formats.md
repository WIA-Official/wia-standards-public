# 제3장: 데이터 형식

## 동의 데이터 구조

본 장에서는 냉동보존 맥락에서 사전 동의를 표현하기 위한 포괄적인 데이터 형식을 정의합니다. 이러한 구조는 무한한 시간 범위, 복잡한 결정 트리, 진화하는 해석을 지원해야 합니다.

---

## 3.1 핵심 동의 스키마

```typescript
// 핵심 동의 레코드 구조
interface ConsentRecord {
  // 식별
  id: string;                          // 고유 동의 식별자
  patientId: string;                   // 환자 참조
  organizationId: string;              // 관리 조직

  // 동의 분류
  consentType: ConsentType;
  category: ConsentCategory;
  subcategories: string[];

  // 범위 정의
  scope: ConsentScope;

  // 결정 내용
  decisions: ConsentDecision[];

  // 권한 체인
  authority: ConsentAuthority;

  // 유효성 매개변수
  validity: ConsentValidity;

  // 문서 참조
  documents: ConsentDocument[];

  // 메타데이터
  metadata: ConsentMetadata;
}

// 동의 유형
enum ConsentType {
  INITIAL = 'INITIAL',                 // 절차에 대한 최초 동의
  UPDATE = 'UPDATE',                   // 기존 동의 수정
  RENEWAL = 'RENEWAL',                 // 정기적 재확인
  REVOCATION = 'REVOCATION',           // 동의 철회
  PROXY = 'PROXY',                     // 위임 동의
  EMERGENCY = 'EMERGENCY',             // 비상 상황 동의
}

// 동의 카테고리
enum ConsentCategory {
  PRESERVATION = 'PRESERVATION',       // 초기 보존 절차
  CARE = 'CARE',                       // 지속적 케어 결정
  REVIVAL = 'REVIVAL',                 // 미래 회복 시나리오
  RESEARCH = 'RESEARCH',               // 연구 참여
  ASSET = 'ASSET',                     // 자산 관리
  COMMUNICATION = 'COMMUNICATION',     // 정보 공유
  PROXY_AUTHORITY = 'PROXY_AUTHORITY', // 권한 위임
}

// 동의 범위 정의
interface ConsentScope {
  // 대상 절차
  procedures: ProcedureScope[];

  // 시간 매개변수
  timeframe: TimeframeScope;

  // 조건 및 제한
  conditions: ScopeCondition[];
  limitations: ScopeLimitation[];

  // 지리적 범위
  jurisdictions: string[];
  facilityTypes: string[];

  // 연구 범위 (해당되는 경우)
  researchScope?: ResearchScope;
}

interface ProcedureScope {
  procedureType: string;
  specificProcedures: string[];
  exclusions: string[];
  conditions: string[];
}

interface TimeframeScope {
  type: 'DEFINITE' | 'INDEFINITE' | 'CONDITIONAL';
  startDate?: Date;
  endDate?: Date;
  duration?: string;           // ISO 8601 기간
  conditionForExpiry?: string;
  renewalRequired?: boolean;
  renewalInterval?: string;
}

interface ScopeCondition {
  conditionId: string;
  conditionType: string;
  description: string;
  evaluationCriteria: string;
  defaultIfUnmet: 'PROCEED' | 'HALT' | 'ESCALATE';
}

interface ScopeLimitation {
  limitationType: string;
  description: string;
  hardLimit: boolean;          // 재정의 불가
  exceptions: string[];
}
```

---

## 3.2 결정 구조

```typescript
// 동의 결정 구조
interface ConsentDecision {
  decisionId: string;
  decisionType: DecisionType;

  // 결정 내용
  question: string;            // 결정되는 내용
  answer: DecisionAnswer;      // 내린 결정
  reasoning?: string;          // 이 결정을 내린 이유

  // 맥락
  context: DecisionContext;

  // 조건
  conditions: DecisionCondition[];

  // 고려된 대안
  alternatives: Alternative[];

  // 메타데이터
  metadata: DecisionMetadata;
}

enum DecisionType {
  BINARY = 'BINARY',                   // 예/아니오
  CHOICE = 'CHOICE',                   // 옵션에서 선택
  THRESHOLD = 'THRESHOLD',             // 숫자 임계값
  PREFERENCE = 'PREFERENCE',           // 순위별 선호도
  CONDITIONAL = 'CONDITIONAL',         // 조건부 결정
  DELEGATION = 'DELEGATION',           // 타인에게 위임
}

interface DecisionAnswer {
  type: DecisionType;

  // 이진 결정용
  binaryValue?: boolean;

  // 선택 결정용
  selectedOption?: string;
  selectedOptions?: string[];  // 다중 선택

  // 임계값 결정용
  thresholdValue?: number;
  thresholdUnit?: string;
  thresholdOperator?: 'GT' | 'GTE' | 'LT' | 'LTE' | 'EQ';

  // 선호도 결정용
  rankedOptions?: RankedOption[];

  // 조건부 결정용
  conditionalRules?: ConditionalRule[];

  // 위임 결정용
  delegateTo?: DelegationTarget;
}

interface RankedOption {
  option: string;
  rank: number;
  weight?: number;
  conditions?: string[];
}

interface ConditionalRule {
  ruleId: string;
  condition: string;
  thenAction: string;
  elseAction?: string;
  priority: number;
}

interface DelegationTarget {
  targetType: 'INDIVIDUAL' | 'ROLE' | 'COMMITTEE' | 'ORGANIZATION';
  targetId: string;
  targetName: string;
  scopeLimitations: string[];
  requiresConfirmation: boolean;
}

interface DecisionContext {
  scenarioDescription: string;
  relevantFactors: string[];
  assumedConditions: string[];
  uncertainties: string[];
  timestamp: Date;
}

interface DecisionCondition {
  conditionId: string;
  conditionExpression: string;
  conditionDescription: string;
  evaluationType: 'AUTOMATIC' | 'MANUAL' | 'HYBRID';
  fallbackBehavior: string;
}

interface Alternative {
  alternativeId: string;
  description: string;
  wasConsidered: boolean;
  reasonNotChosen?: string;
}

interface DecisionMetadata {
  version: number;
  createdAt: Date;
  createdBy: string;
  lastReviewedAt?: Date;
  lastReviewedBy?: string;
  confidenceLevel: number;     // 0-1
  requiresPeriodicReview: boolean;
  reviewInterval?: string;
}
```

---

## 3.3 권한 체인 구조

```typescript
// 동의 권한 구조
interface ConsentAuthority {
  // 주요 부여자
  grantor: AuthorityEntity;

  // 증인 정보
  witnesses: Witness[];

  // 공증
  notarization?: NotarizationRecord;

  // 법률 검토
  legalReview?: LegalReview;

  // 능력 평가
  capacityAssessment?: CapacityAssessment;

  // 대리인 체인
  proxyChain?: ProxyChainEntry[];
}

interface AuthorityEntity {
  entityId: string;
  entityType: 'PATIENT' | 'PROXY' | 'GUARDIAN' | 'ORGANIZATION';

  // 신원 확인
  identityVerification: IdentityVerification;

  // 능력 상태
  capacityConfirmed: boolean;
  capacityDate: Date;
  capacityAssessor?: string;

  // 연락처 정보
  contactInfo: ContactInfo;
}

interface IdentityVerification {
  verificationMethod: string;
  verificationDate: Date;
  verifiedBy: string;
  documentTypes: string[];
  documentReferences: string[];
  biometricData?: BiometricReference;
  verificationScore: number;
}

interface BiometricReference {
  biometricType: string;
  referenceId: string;
  captureDate: Date;
  storageLocation: string;
  encryptionMethod: string;
}

interface Witness {
  witnessId: string;
  name: string;
  role: string;
  relationship: string;
  qualifications?: string[];

  // 증인 확인
  identityVerified: boolean;
  verificationMethod: string;

  // 증인 증명
  attestation: WitnessAttestation;

  contactInfo: ContactInfo;
}

interface WitnessAttestation {
  attestationDate: Date;
  attestationLocation: string;
  attestationType: 'IN_PERSON' | 'REMOTE_VERIFIED' | 'NOTARIZED';

  // 증인이 증명하는 내용
  attestsToVoluntariness: boolean;
  attestsToCapacity: boolean;
  attestsToIdentity: boolean;
  attestsToUnderstanding: boolean;

  // 서명
  signature: SignatureRecord;

  notes?: string;
}

interface NotarizationRecord {
  notaryId: string;
  notaryName: string;
  notaryCommission: string;
  jurisdiction: string;

  notarizationDate: Date;
  notarizationLocation: string;

  documentId: string;

  seal: NotarySeal;
  signature: SignatureRecord;

  expirationDate?: Date;
}

interface NotarySeal {
  sealType: string;
  sealImageReference: string;
  sealNumber: string;
}

interface SignatureRecord {
  signatureType: 'WET' | 'ELECTRONIC' | 'DIGITAL';
  signatureData?: string;      // 전자/디지털용
  signatureImageRef?: string;  // 육안 스캔용

  // 디지털 서명 상세
  digitalSignature?: {
    algorithm: string;
    publicKeyId: string;
    signatureValue: string;
    certificateChain: string[];
    timestamp: string;
    timestampAuthority?: string;
  };

  signatureDate: Date;
  signatureLocation: string;
}

interface CapacityAssessment {
  assessmentId: string;
  assessmentDate: Date;
  assessor: CapacityAssessor;

  // 평가 방법
  method: string;
  tools: string[];

  // 결과
  hasCapacity: boolean;
  capacityScore?: number;
  domains: CapacityDomain[];

  // 소견
  findings: string;
  limitations?: string[];
  recommendations?: string[];

  // 유효성
  validUntil?: Date;
  reviewRequired: boolean;
  reviewTriggers: string[];
}

interface CapacityAssessor {
  assessorId: string;
  name: string;
  credentials: string[];
  licenseNumber: string;
  jurisdiction: string;
  specialty: string;
}

interface CapacityDomain {
  domain: string;
  score: number;
  maxScore: number;
  assessment: string;
  concerns?: string[];
}

interface ProxyChainEntry {
  order: number;
  proxy: ProxyDesignation;
  activationConditions: string[];
  deactivationConditions: string[];
  limitations: string[];
}

interface ProxyDesignation {
  proxyId: string;
  proxyType: 'INDIVIDUAL' | 'ORGANIZATION' | 'ROLE';

  // 개인용
  individual?: {
    name: string;
    relationship: string;
    contactInfo: ContactInfo;
    identityVerification: IdentityVerification;
  };

  // 조직용
  organization?: {
    name: string;
    registrationNumber: string;
    contactInfo: ContactInfo;
    authorizedRepresentatives: string[];
  };

  // 역할 기반용
  role?: {
    roleName: string;
    roleDefinition: string;
    qualificationRequirements: string[];
    appointmentProcess: string;
  };

  // 권한 범위
  authorityScope: ProxyAuthorityScope;

  // 수락
  acceptanceStatus: 'PENDING' | 'ACCEPTED' | 'DECLINED';
  acceptanceDate?: Date;
  acceptanceSignature?: SignatureRecord;
}

interface ProxyAuthorityScope {
  categories: ConsentCategory[];
  specificDecisions: string[];
  exclusions: string[];

  // 제한
  financialLimit?: number;
  requiresConsultation: string[];
  requiresCommitteeApproval: string[];

  // 타이밍
  effectiveFrom?: Date;
  effectiveUntil?: Date;
  activationCondition?: string;
}
```

---

## 3.4 유효성 및 생명주기 구조

```typescript
// 동의 유효성 구조
interface ConsentValidity {
  // 상태
  status: ConsentStatus;
  statusHistory: StatusChange[];

  // 유효 기간
  effectiveDate: Date;
  expirationDate?: Date;       // 무기한의 경우 Null

  // 조건
  conditions: ValidityCondition[];

  // 검토 일정
  reviewSchedule?: ReviewSchedule;

  // 철회 정보
  revocation?: RevocationInfo;

  // 대체
  supersedes?: string[];       // 대체된 동의 ID
  supersededBy?: string;       // 대체하는 동의 ID
}

enum ConsentStatus {
  DRAFT = 'DRAFT',
  PENDING_WITNESS = 'PENDING_WITNESS',
  PENDING_NOTARIZATION = 'PENDING_NOTARIZATION',
  PENDING_REVIEW = 'PENDING_REVIEW',
  ACTIVE = 'ACTIVE',
  SUSPENDED = 'SUSPENDED',
  REVOKED = 'REVOKED',
  EXPIRED = 'EXPIRED',
  SUPERSEDED = 'SUPERSEDED',
}

interface StatusChange {
  fromStatus: ConsentStatus;
  toStatus: ConsentStatus;
  changeDate: Date;
  changedBy: string;
  reason: string;
  documentReference?: string;
}

interface ValidityCondition {
  conditionId: string;
  conditionType: 'PREREQUISITE' | 'MAINTENANCE' | 'TERMINATION';

  description: string;
  evaluationExpression: string;

  // 전제 조건용
  mustBeTrueFor: 'ACTIVATION' | 'CONTINUATION';

  // 평가
  lastEvaluated?: Date;
  lastResult?: boolean;
  evaluationMethod: 'AUTOMATIC' | 'MANUAL' | 'PERIODIC';
  evaluationFrequency?: string;

  // 조건 실패 시
  failureAction: 'SUSPEND' | 'NOTIFY' | 'ESCALATE' | 'TERMINATE';
  notifyParties?: string[];
}

interface ReviewSchedule {
  reviewRequired: boolean;
  reviewFrequency: string;     // ISO 8601 기간
  nextReviewDate: Date;

  reviewType: 'REAFFIRMATION' | 'UPDATE' | 'CAPACITY_CHECK';

  reviewProcess: {
    initiatedBy: string;
    notificationDays: number;
    reminderSchedule: number[];
    escalationProcess: string;
    defaultIfNoResponse: 'MAINTAIN' | 'SUSPEND' | 'EXPIRE';
  };

  reviewHistory: ReviewRecord[];
}

interface ReviewRecord {
  reviewId: string;
  scheduledDate: Date;
  completedDate?: Date;

  reviewType: string;
  reviewedBy: string;

  outcome: 'REAFFIRMED' | 'MODIFIED' | 'REVOKED' | 'DEFERRED' | 'NO_RESPONSE';

  modifications?: string[];
  notes?: string;

  nextReviewScheduled?: Date;
}

interface RevocationInfo {
  revokedAt: Date;
  revokedBy: string;
  revocationType: 'FULL' | 'PARTIAL';

  reason: string;

  // 부분 철회용
  revokedDecisions?: string[];
  remainingScope?: ConsentScope;

  // 효과
  effectiveDate: Date;
  retroactive: boolean;

  // 절차
  verificationMethod: string;
  witnessedBy?: string[];
  documentReference: string;
}
```

---

## 3.5 문서 참조 구조

```typescript
// 동의 문서 구조
interface ConsentDocument {
  documentId: string;
  documentType: DocumentType;

  // 문서 내용
  title: string;
  description: string;
  language: string;

  // 저장
  storage: DocumentStorage;

  // 버전
  version: DocumentVersion;

  // 무결성
  integrity: DocumentIntegrity;

  // 액세스
  accessControl: DocumentAccess;

  // 법적 상태
  legalStatus: LegalDocumentStatus;
}

enum DocumentType {
  CONSENT_FORM = 'CONSENT_FORM',
  INFORMATION_SHEET = 'INFORMATION_SHEET',
  CAPACITY_ASSESSMENT = 'CAPACITY_ASSESSMENT',
  WITNESS_STATEMENT = 'WITNESS_STATEMENT',
  NOTARIZATION_CERTIFICATE = 'NOTARIZATION_CERTIFICATE',
  PROXY_DESIGNATION = 'PROXY_DESIGNATION',
  VALUE_STATEMENT = 'VALUE_STATEMENT',
  ADVANCE_DIRECTIVE = 'ADVANCE_DIRECTIVE',
  AMENDMENT = 'AMENDMENT',
  REVOCATION = 'REVOCATION',
  SUPPORTING_EVIDENCE = 'SUPPORTING_EVIDENCE',
}

interface DocumentStorage {
  storageType: 'LOCAL' | 'DISTRIBUTED' | 'BLOCKCHAIN' | 'HYBRID';

  primaryLocation: StorageLocation;
  backupLocations: StorageLocation[];

  encryption: {
    encrypted: boolean;
    algorithm: string;
    keyReference: string;
  };

  retentionPolicy: {
    retentionPeriod: string;
    retentionType: 'INDEFINITE' | 'FIXED' | 'CONDITIONAL';
    disposalMethod?: string;
  };
}

interface StorageLocation {
  locationType: string;
  locationUri: string;
  provider?: string;
  region?: string;
  replicationFactor?: number;
}

interface DocumentVersion {
  versionNumber: string;
  versionDate: Date;

  createdBy: string;
  changeDescription?: string;

  previousVersion?: string;

  isCurrent: boolean;
  isLatest: boolean;
}

interface DocumentIntegrity {
  hashAlgorithm: string;
  hashValue: string;

  // 미래 보장을 위한 다중 해시 알고리즘
  additionalHashes?: {
    algorithm: string;
    value: string;
  }[];

  // 타임스탬프
  timestamp: {
    timestampDate: Date;
    timestampAuthority?: string;
    timestampToken?: string;
  };

  // 블록체인 앵커링
  blockchainAnchor?: {
    network: string;
    transactionId: string;
    blockNumber: number;
    timestamp: Date;
  };
}

interface DocumentAccess {
  accessLevel: 'PUBLIC' | 'RESTRICTED' | 'CONFIDENTIAL' | 'SECRET';

  authorizedRoles: string[];
  authorizedIndividuals: string[];

  accessConditions: string[];

  accessLog: AccessLogEntry[];
}

interface AccessLogEntry {
  accessDate: Date;
  accessedBy: string;
  accessType: 'VIEW' | 'DOWNLOAD' | 'PRINT' | 'SHARE';
  purpose: string;
  ipAddress?: string;
  authorized: boolean;
}

interface LegalDocumentStatus {
  isLegallyBinding: boolean;
  jurisdictions: string[];

  legalReview?: {
    reviewDate: Date;
    reviewedBy: string;
    opinion: string;
    concerns?: string[];
  };

  courtRecognition?: {
    jurisdiction: string;
    caseReference?: string;
    recognitionDate?: Date;
    status: string;
  }[];
}
```

---

## 3.6 환자 가치 문서화

```typescript
// 환자 가치 구조
interface PatientValueDocument {
  documentId: string;
  patientId: string;

  // 가치 카테고리
  coreValues: CoreValue[];
  lifePhilosophy: LifePhilosophy;
  qualityOfLifePreferences: QualityOfLifePreferences;
  identityPreferences: IdentityPreferences;
  socialPreferences: SocialPreferences;
  spiritualBeliefs: SpiritualBeliefs;

  // 특정 시나리오
  scenarioPreferences: ScenarioPreference[];

  // 해석 지침
  interpretationGuidance: InterpretationGuidance;

  // 메타데이터
  metadata: ValueDocumentMetadata;
}

interface CoreValue {
  valueId: string;
  valueName: string;
  description: string;
  importance: number;          // 1-10
  rank: number;

  // 맥락
  relevantScenarios: string[];
  tradeoffs: ValueTradeoff[];

  // 증거
  supportingStatements: string[];
  exampleDecisions: string[];
}

interface ValueTradeoff {
  conflictingValue: string;
  resolution: string;
  priority: 'THIS' | 'OTHER' | 'CONTEXT_DEPENDENT';
  contextRules?: string[];
}

interface LifePhilosophy {
  philosophyStatement: string;
  meaningOfLife: string;
  attitudeTowardDeath: string;
  attitudeTowardRisk: string;
  attitudeTowardChange: string;
  attitudeTowardTechnology: string;

  religiousViews?: {
    religion: string;
    denomination?: string;
    practiceLevel: string;
    relevantBeliefs: string[];
    restrictions: string[];
  };
}

interface QualityOfLifePreferences {
  minimumAcceptableQuality: {
    physicalFunction: QualityThreshold;
    cognitiveFunction: QualityThreshold;
    socialFunction: QualityThreshold;
    emotionalWellbeing: QualityThreshold;
    independence: QualityThreshold;
  };

  unacceptableConditions: string[];
  conditionsPreferDeathOver: string[];

  willingnessToTryExperimental: number; // 1-10
  riskTolerance: number;               // 1-10
}

interface QualityThreshold {
  minimumLevel: number;        // 1-10
  description: string;
  absoluteMinimum?: boolean;
  contextualFactors?: string[];
}

interface IdentityPreferences {
  whatMakesYouYou: string;
  acceptableChanges: string[];
  unacceptableChanges: string[];

  memoryPreferences: {
    minimumMemoryRetention: string;
    acceptableLoss: string[];
    unacceptableLoss: string[];
  };

  bodyPreferences: {
    originalBodyPreference: number; // 1-10
    acceptableAlternatives: string[];
    unacceptableAlternatives: string[];
  };

  continuityDefinition: string;
}

interface SocialPreferences {
  relationshipPriorities: {
    family: number;
    friends: number;
    community: number;
    humanity: number;
  };

  whoToReviveWith: string[];
  whoNotToReviveWith: string[];

  socialIntegrationPreferences: {
    preferFamiliarSociety: number;
    willingnessToAdapt: number;
    communityRequirements: string[];
  };
}

interface SpiritualBeliefs {
  beliefSystem: string;
  relevanceToRevival: string;
  spiritualConcerns: string[];
  spiritualRequirements: string[];

  afterlifeBeliefs?: {
    belief: string;
    howAffectsDecisions: string;
  };
}

interface ScenarioPreference {
  scenarioId: string;
  scenarioDescription: string;

  decision: string;
  reasoning: string;

  conditions: string[];
  flexibility: 'FIRM' | 'FLEXIBLE' | 'OPEN_TO_PROXY';

  dateRecorded: Date;
  reviewRequired: boolean;
}

interface InterpretationGuidance {
  generalPrinciples: string[];

  whenUnclear: {
    defaultApproach: string;
    escalationPath: string;
    consultationRequired: string[];
  };

  importantContext: string[];
  commonMisinterpretations: string[];

  trustedInterpreters: {
    individuals: string[];
    roles: string[];
    qualifications: string[];
  };
}

interface ValueDocumentMetadata {
  version: number;
  createdAt: Date;
  updatedAt: Date;

  facilitator?: string;
  process: string;

  verificationStatus: 'UNVERIFIED' | 'SELF_VERIFIED' | 'WITNESS_VERIFIED' | 'PROFESSIONAL_VERIFIED';
  verifiedBy?: string;
  verificationDate?: Date;

  nextReviewDate?: Date;
  reviewHistory: {
    date: Date;
    type: string;
    changes: string[];
  }[];
}
```

---

## 3.7 직렬화 및 저장

```typescript
// 직렬화 서비스
class ConsentSerializationService {
  // 동의를 저장 형식으로 직렬화
  serializeConsent(consent: ConsentRecord): SerializedConsent {
    return {
      format: 'WIA-CONSENT-V1',
      encoding: 'JSON',
      compression: 'GZIP',

      // 핵심 데이터
      data: this.compressAndEncode(JSON.stringify(consent)),

      // 무결성
      integrity: {
        algorithm: 'SHA-256',
        hash: this.calculateHash(consent),
        timestamp: new Date().toISOString(),
      },

      // 스키마 참조
      schema: {
        version: '1.0.0',
        uri: 'https://wia.org/schemas/consent/v1.0.0',
      },

      // 마이그레이션 지원
      migration: {
        minimumReaderVersion: '1.0.0',
        backwardsCompatible: true,
      },
    };
  }

  // 저장소에서 역직렬화
  deserializeConsent(serialized: SerializedConsent): ConsentRecord {
    // 무결성 확인
    const data = this.decodeAndDecompress(serialized.data);
    const consent = JSON.parse(data) as ConsentRecord;

    const calculatedHash = this.calculateHash(consent);
    if (calculatedHash !== serialized.integrity.hash) {
      throw new IntegrityError('동의 데이터 무결성 검사 실패');
    }

    // 필요시 스키마 마이그레이션 처리
    if (this.needsMigration(serialized.schema.version)) {
      return this.migrateConsent(consent, serialized.schema.version);
    }

    return consent;
  }

  // 무결성을 위한 해시 계산
  private calculateHash(consent: ConsentRecord): string {
    const canonicalized = this.canonicalize(consent);
    return crypto.createHash('sha256').update(canonicalized).digest('hex');
  }

  // 일관된 해싱을 위한 정규화
  private canonicalize(consent: ConsentRecord): string {
    // 비필수 메타데이터 제거
    const essential = {
      id: consent.id,
      patientId: consent.patientId,
      consentType: consent.consentType,
      category: consent.category,
      scope: consent.scope,
      decisions: consent.decisions,
      authority: consent.authority,
      validity: consent.validity,
    };

    // 일관성을 위한 키 정렬
    return JSON.stringify(essential, Object.keys(essential).sort());
  }

  // 스키마 마이그레이션
  private migrateConsent(consent: any, fromVersion: string): ConsentRecord {
    const migrations: Record<string, (c: any) => any> = {
      '0.9.0': this.migrate_0_9_to_1_0,
      '0.8.0': (c) => this.migrate_0_9_to_1_0(this.migrate_0_8_to_0_9(c)),
    };

    if (migrations[fromVersion]) {
      return migrations[fromVersion](consent);
    }

    throw new MigrationError(`버전 ${fromVersion}에서의 마이그레이션 경로가 없습니다`);
  }

  private migrate_0_9_to_1_0(consent: any): ConsentRecord {
    return {
      ...consent,
      // 기본값과 함께 새 필수 필드 추가
      subcategories: consent.subcategories || [],
      metadata: {
        ...consent.metadata,
        documentFormat: consent.metadata?.documentFormat || 'WIA-CONSENT-V1',
      },
    };
  }

  private migrate_0_8_to_0_9(consent: any): any {
    // 이전 형식 변환 처리
    return {
      ...consent,
      authority: {
        grantor: consent.grantedBy,
        witnesses: consent.witnesses || [],
        notarization: consent.notary,
      },
    };
  }
}

interface SerializedConsent {
  format: string;
  encoding: string;
  compression: string;
  data: string;
  integrity: {
    algorithm: string;
    hash: string;
    timestamp: string;
  };
  schema: {
    version: string;
    uri: string;
  };
  migration: {
    minimumReaderVersion: string;
    backwardsCompatible: boolean;
  };
}
```

---

## 3.8 검증 스키마

```typescript
// 동의 검증을 위한 JSON 스키마
const consentSchema = {
  $schema: 'http://json-schema.org/draft-07/schema#',
  $id: 'https://wia.org/schemas/consent/v1.0.0',
  title: 'WIA 냉동보존 동의 레코드',
  type: 'object',
  required: [
    'id',
    'patientId',
    'consentType',
    'category',
    'scope',
    'decisions',
    'authority',
    'validity',
    'metadata',
  ],
  properties: {
    id: {
      type: 'string',
      pattern: '^CONSENT-[A-Z0-9]{8}-[A-Z0-9]{4}$',
    },
    patientId: {
      type: 'string',
      pattern: '^PATIENT-[A-Z0-9]{8}$',
    },
    consentType: {
      type: 'string',
      enum: ['INITIAL', 'UPDATE', 'RENEWAL', 'REVOCATION', 'PROXY', 'EMERGENCY'],
    },
    category: {
      type: 'string',
      enum: ['PRESERVATION', 'CARE', 'REVIVAL', 'RESEARCH', 'ASSET', 'COMMUNICATION', 'PROXY_AUTHORITY'],
    },
    // ... 추가 속성 정의
  },
};

// 검증 서비스
class ConsentValidationService {
  private ajv: Ajv;
  private validator: ValidateFunction;

  constructor() {
    this.ajv = new Ajv({ allErrors: true, strict: false });
    addFormats(this.ajv);
    this.validator = this.ajv.compile(consentSchema);
  }

  validateConsent(consent: unknown): ValidationResult {
    const valid = this.validator(consent);

    if (!valid) {
      return {
        valid: false,
        errors: this.formatErrors(this.validator.errors || []),
      };
    }

    // 추가 비즈니스 규칙 검증
    const businessErrors = this.validateBusinessRules(consent as ConsentRecord);
    if (businessErrors.length > 0) {
      return {
        valid: false,
        errors: businessErrors,
      };
    }

    return { valid: true, errors: [] };
  }

  private validateBusinessRules(consent: ConsentRecord): ValidationError[] {
    const errors: ValidationError[] = [];

    // 규칙: 활성 동의는 부여자 능력 확인 필요
    if (consent.validity.status === 'ACTIVE' && !consent.authority.grantor.capacityConfirmed) {
      errors.push({
        path: 'authority.grantor.capacityConfirmed',
        message: '활성 동의에는 확인된 부여자 능력이 필요합니다',
        code: 'CAPACITY_NOT_CONFIRMED',
      });
    }

    // 규칙: 무기한 시간 범위에는 종료일이 없어야 함
    if (consent.scope.timeframe.type === 'INDEFINITE' && consent.scope.timeframe.endDate) {
      errors.push({
        path: 'scope.timeframe',
        message: '무기한 시간 범위에는 종료일이 없어야 합니다',
        code: 'INVALID_TIMEFRAME',
      });
    }

    // 규칙: 최소 하나의 결정 필요
    if (!consent.decisions || consent.decisions.length === 0) {
      errors.push({
        path: 'decisions',
        message: '최소 하나의 결정이 필요합니다',
        code: 'NO_DECISIONS',
      });
    }

    return errors;
  }
}

interface ValidationResult {
  valid: boolean;
  errors: ValidationError[];
}

interface ValidationError {
  path: string;
  message: string;
  code: string;
  params?: Record<string, any>;
  severity?: 'ERROR' | 'WARNING';
}
```

---

*다음 장: API 인터페이스 - 동의 관리를 위한 RESTful 및 GraphQL API*
