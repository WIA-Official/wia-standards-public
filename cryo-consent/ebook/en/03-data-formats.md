# Chapter 3: Data Formats

## Consent Data Structures

This chapter defines the comprehensive data formats for representing informed consent in cryonics preservation contexts. These structures must support indefinite time horizons, complex decision trees, and evolving interpretations.

---

## 3.1 Core Consent Schema

```typescript
// Core consent record structure
interface ConsentRecord {
  // Identification
  id: string;                          // Unique consent identifier
  patientId: string;                   // Reference to patient
  organizationId: string;              // Managing organization

  // Consent classification
  consentType: ConsentType;
  category: ConsentCategory;
  subcategories: string[];

  // Scope definition
  scope: ConsentScope;

  // Decision content
  decisions: ConsentDecision[];

  // Authority chain
  authority: ConsentAuthority;

  // Validity parameters
  validity: ConsentValidity;

  // Document references
  documents: ConsentDocument[];

  // Metadata
  metadata: ConsentMetadata;
}

// Consent types
enum ConsentType {
  INITIAL = 'INITIAL',                 // First consent for procedure
  UPDATE = 'UPDATE',                   // Modification to existing
  RENEWAL = 'RENEWAL',                 // Periodic reaffirmation
  REVOCATION = 'REVOCATION',           // Withdrawal of consent
  PROXY = 'PROXY',                     // Delegated consent
  EMERGENCY = 'EMERGENCY',             // Emergency situation consent
}

// Consent categories
enum ConsentCategory {
  PRESERVATION = 'PRESERVATION',       // Initial preservation procedures
  CARE = 'CARE',                       // Ongoing care decisions
  REVIVAL = 'REVIVAL',                 // Future revival scenarios
  RESEARCH = 'RESEARCH',               // Research participation
  ASSET = 'ASSET',                     // Asset management
  COMMUNICATION = 'COMMUNICATION',     // Information sharing
  PROXY_AUTHORITY = 'PROXY_AUTHORITY', // Delegation of authority
}

// Consent scope definition
interface ConsentScope {
  // Covered procedures
  procedures: ProcedureScope[];

  // Time parameters
  timeframe: TimeframeScope;

  // Conditions and limitations
  conditions: ScopeCondition[];
  limitations: ScopeLimitation[];

  // Geographic scope
  jurisdictions: string[];
  facilityTypes: string[];

  // Research scope (if applicable)
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
  duration?: string;           // ISO 8601 duration
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
  hardLimit: boolean;          // Cannot be overridden
  exceptions: string[];
}
```

---

## 3.2 Decision Structures

```typescript
// Consent decision structure
interface ConsentDecision {
  decisionId: string;
  decisionType: DecisionType;

  // Decision content
  question: string;            // What is being decided
  answer: DecisionAnswer;      // The decision made
  reasoning?: string;          // Why this decision was made

  // Context
  context: DecisionContext;

  // Conditions
  conditions: DecisionCondition[];

  // Alternatives considered
  alternatives: Alternative[];

  // Metadata
  metadata: DecisionMetadata;
}

enum DecisionType {
  BINARY = 'BINARY',                   // Yes/No
  CHOICE = 'CHOICE',                   // Select from options
  THRESHOLD = 'THRESHOLD',             // Numeric threshold
  PREFERENCE = 'PREFERENCE',           // Ranked preferences
  CONDITIONAL = 'CONDITIONAL',         // If-then decision
  DELEGATION = 'DELEGATION',           // Delegate to others
}

interface DecisionAnswer {
  type: DecisionType;

  // For binary decisions
  binaryValue?: boolean;

  // For choice decisions
  selectedOption?: string;
  selectedOptions?: string[];  // Multiple selection

  // For threshold decisions
  thresholdValue?: number;
  thresholdUnit?: string;
  thresholdOperator?: 'GT' | 'GTE' | 'LT' | 'LTE' | 'EQ';

  // For preference decisions
  rankedOptions?: RankedOption[];

  // For conditional decisions
  conditionalRules?: ConditionalRule[];

  // For delegation decisions
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

## 3.3 Authority Chain Structures

```typescript
// Consent authority structure
interface ConsentAuthority {
  // Primary grantor
  grantor: AuthorityEntity;

  // Witness information
  witnesses: Witness[];

  // Notarization
  notarization?: NotarizationRecord;

  // Legal review
  legalReview?: LegalReview;

  // Capacity assessment
  capacityAssessment?: CapacityAssessment;

  // Proxy chain
  proxyChain?: ProxyChainEntry[];
}

interface AuthorityEntity {
  entityId: string;
  entityType: 'PATIENT' | 'PROXY' | 'GUARDIAN' | 'ORGANIZATION';

  // Identity verification
  identityVerification: IdentityVerification;

  // Capacity status
  capacityConfirmed: boolean;
  capacityDate: Date;
  capacityAssessor?: string;

  // Contact information
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

  // Witness verification
  identityVerified: boolean;
  verificationMethod: string;

  // Witness attestation
  attestation: WitnessAttestation;

  contactInfo: ContactInfo;
}

interface WitnessAttestation {
  attestationDate: Date;
  attestationLocation: string;
  attestationType: 'IN_PERSON' | 'REMOTE_VERIFIED' | 'NOTARIZED';

  // What the witness attests to
  attestsToVoluntariness: boolean;
  attestsToCapacity: boolean;
  attestsToIdentity: boolean;
  attestsToUnderstanding: boolean;

  // Signature
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
  signatureData?: string;      // For electronic/digital
  signatureImageRef?: string;  // For wet ink scan

  // Digital signature details
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

  // Assessment method
  method: string;
  tools: string[];

  // Results
  hasCapacity: boolean;
  capacityScore?: number;
  domains: CapacityDomain[];

  // Findings
  findings: string;
  limitations?: string[];
  recommendations?: string[];

  // Validity
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

  // For individual
  individual?: {
    name: string;
    relationship: string;
    contactInfo: ContactInfo;
    identityVerification: IdentityVerification;
  };

  // For organization
  organization?: {
    name: string;
    registrationNumber: string;
    contactInfo: ContactInfo;
    authorizedRepresentatives: string[];
  };

  // For role-based
  role?: {
    roleName: string;
    roleDefinition: string;
    qualificationRequirements: string[];
    appointmentProcess: string;
  };

  // Authority scope
  authorityScope: ProxyAuthorityScope;

  // Acceptance
  acceptanceStatus: 'PENDING' | 'ACCEPTED' | 'DECLINED';
  acceptanceDate?: Date;
  acceptanceSignature?: SignatureRecord;
}

interface ProxyAuthorityScope {
  categories: ConsentCategory[];
  specificDecisions: string[];
  exclusions: string[];

  // Limitations
  financialLimit?: number;
  requiresConsultation: string[];
  requiresCommitteeApproval: string[];

  // Timing
  effectiveFrom?: Date;
  effectiveUntil?: Date;
  activationCondition?: string;
}
```

---

## 3.4 Validity and Lifecycle Structures

```typescript
// Consent validity structure
interface ConsentValidity {
  // Status
  status: ConsentStatus;
  statusHistory: StatusChange[];

  // Effective period
  effectiveDate: Date;
  expirationDate?: Date;       // Null for indefinite

  // Conditions
  conditions: ValidityCondition[];

  // Review schedule
  reviewSchedule?: ReviewSchedule;

  // Revocation info
  revocation?: RevocationInfo;

  // Supersession
  supersedes?: string[];       // IDs of superseded consents
  supersededBy?: string;       // ID of superseding consent
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

  // For prerequisite conditions
  mustBeTrueFor: 'ACTIVATION' | 'CONTINUATION';

  // Evaluation
  lastEvaluated?: Date;
  lastResult?: boolean;
  evaluationMethod: 'AUTOMATIC' | 'MANUAL' | 'PERIODIC';
  evaluationFrequency?: string;

  // If condition fails
  failureAction: 'SUSPEND' | 'NOTIFY' | 'ESCALATE' | 'TERMINATE';
  notifyParties?: string[];
}

interface ReviewSchedule {
  reviewRequired: boolean;
  reviewFrequency: string;     // ISO 8601 duration
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

  // For partial revocation
  revokedDecisions?: string[];
  remainingScope?: ConsentScope;

  // Effects
  effectiveDate: Date;
  retroactive: boolean;

  // Process
  verificationMethod: string;
  witnessedBy?: string[];
  documentReference: string;
}
```

---

## 3.5 Document Reference Structures

```typescript
// Consent document structure
interface ConsentDocument {
  documentId: string;
  documentType: DocumentType;

  // Document content
  title: string;
  description: string;
  language: string;

  // Storage
  storage: DocumentStorage;

  // Versions
  version: DocumentVersion;

  // Integrity
  integrity: DocumentIntegrity;

  // Access
  accessControl: DocumentAccess;

  // Legal status
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

  // Multiple hash algorithms for future-proofing
  additionalHashes?: {
    algorithm: string;
    value: string;
  }[];

  // Timestamp
  timestamp: {
    timestampDate: Date;
    timestampAuthority?: string;
    timestampToken?: string;
  };

  // Blockchain anchoring
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

## 3.6 Patient Values Documentation

```typescript
// Patient values structure
interface PatientValueDocument {
  documentId: string;
  patientId: string;

  // Value categories
  coreValues: CoreValue[];
  lifePhilosophy: LifePhilosophy;
  qualityOfLifePreferences: QualityOfLifePreferences;
  identityPreferences: IdentityPreferences;
  socialPreferences: SocialPreferences;
  spiritualBeliefs: SpiritualBeliefs;

  // Specific scenarios
  scenarioPreferences: ScenarioPreference[];

  // Interpretation guidance
  interpretationGuidance: InterpretationGuidance;

  // Metadata
  metadata: ValueDocumentMetadata;
}

interface CoreValue {
  valueId: string;
  valueName: string;
  description: string;
  importance: number;          // 1-10
  rank: number;

  // Context
  relevantScenarios: string[];
  tradeoffs: ValueTradeoff[];

  // Evidence
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

## 3.7 Serialization and Storage

```typescript
// Serialization service
class ConsentSerializationService {
  // Serialize consent to storage format
  serializeConsent(consent: ConsentRecord): SerializedConsent {
    return {
      format: 'WIA-CONSENT-V1',
      encoding: 'JSON',
      compression: 'GZIP',

      // Core data
      data: this.compressAndEncode(JSON.stringify(consent)),

      // Integrity
      integrity: {
        algorithm: 'SHA-256',
        hash: this.calculateHash(consent),
        timestamp: new Date().toISOString(),
      },

      // Schema reference
      schema: {
        version: '1.0.0',
        uri: 'https://wia.org/schemas/consent/v1.0.0',
      },

      // Migration support
      migration: {
        minimumReaderVersion: '1.0.0',
        backwardsCompatible: true,
      },
    };
  }

  // Deserialize from storage
  deserializeConsent(serialized: SerializedConsent): ConsentRecord {
    // Verify integrity
    const data = this.decodeAndDecompress(serialized.data);
    const consent = JSON.parse(data) as ConsentRecord;

    const calculatedHash = this.calculateHash(consent);
    if (calculatedHash !== serialized.integrity.hash) {
      throw new IntegrityError('Consent data integrity check failed');
    }

    // Handle schema migration if needed
    if (this.needsMigration(serialized.schema.version)) {
      return this.migrateConsent(consent, serialized.schema.version);
    }

    return consent;
  }

  // Calculate hash for integrity
  private calculateHash(consent: ConsentRecord): string {
    const canonicalized = this.canonicalize(consent);
    return crypto.createHash('sha256').update(canonicalized).digest('hex');
  }

  // Canonicalize for consistent hashing
  private canonicalize(consent: ConsentRecord): string {
    // Remove non-essential metadata
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

    // Sort keys for consistency
    return JSON.stringify(essential, Object.keys(essential).sort());
  }

  // Schema migration
  private migrateConsent(consent: any, fromVersion: string): ConsentRecord {
    const migrations: Record<string, (c: any) => any> = {
      '0.9.0': this.migrate_0_9_to_1_0,
      '0.8.0': (c) => this.migrate_0_9_to_1_0(this.migrate_0_8_to_0_9(c)),
    };

    if (migrations[fromVersion]) {
      return migrations[fromVersion](consent);
    }

    throw new MigrationError(`No migration path from version ${fromVersion}`);
  }

  private migrate_0_9_to_1_0(consent: any): ConsentRecord {
    return {
      ...consent,
      // Add new required fields with defaults
      subcategories: consent.subcategories || [],
      metadata: {
        ...consent.metadata,
        documentFormat: consent.metadata?.documentFormat || 'WIA-CONSENT-V1',
      },
    };
  }

  private migrate_0_8_to_0_9(consent: any): any {
    // Handle older format transformations
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

## 3.8 Query and Filter Structures

```typescript
// Consent query structures
interface ConsentQuery {
  // Entity filters
  patientId?: string;
  organizationId?: string;

  // Type filters
  consentTypes?: ConsentType[];
  categories?: ConsentCategory[];

  // Status filters
  statuses?: ConsentStatus[];
  includeExpired?: boolean;
  includeRevoked?: boolean;

  // Date filters
  effectiveAsOf?: Date;
  createdAfter?: Date;
  createdBefore?: Date;
  expiresAfter?: Date;
  expiresBefore?: Date;

  // Decision filters
  includesDecisionType?: string;
  includesDecisionAnswer?: string;

  // Scope filters
  jurisdiction?: string;
  facilityType?: string;
  procedureType?: string;

  // Authority filters
  grantorId?: string;
  proxyId?: string;

  // Pagination
  offset?: number;
  limit?: number;

  // Sorting
  sortBy?: ConsentSortField;
  sortOrder?: 'ASC' | 'DESC';

  // Include options
  include?: ConsentIncludeOptions;
}

enum ConsentSortField {
  CREATED_AT = 'CREATED_AT',
  UPDATED_AT = 'UPDATED_AT',
  EFFECTIVE_DATE = 'EFFECTIVE_DATE',
  EXPIRATION_DATE = 'EXPIRATION_DATE',
  PATIENT_NAME = 'PATIENT_NAME',
  STATUS = 'STATUS',
}

interface ConsentIncludeOptions {
  documents?: boolean;
  fullAuthority?: boolean;
  statusHistory?: boolean;
  reviewHistory?: boolean;
  relatedConsents?: boolean;
}

interface ConsentQueryResult {
  consents: ConsentRecord[];
  total: number;
  offset: number;
  limit: number;
  hasMore: boolean;

  // Aggregations
  aggregations?: {
    byStatus: Record<ConsentStatus, number>;
    byCategory: Record<ConsentCategory, number>;
    byYear: Record<number, number>;
  };
}

// Decision query
interface DecisionQuery {
  patientId: string;
  decisionType: string;
  context: DecisionContext;

  // Options
  includeSuperseded?: boolean;
  asOfDate?: Date;
  interpretationMode?: 'STRICT' | 'FLEXIBLE';
}

interface DecisionQueryResult {
  found: boolean;
  decision?: ConsentDecision;
  consent?: ConsentRecord;

  confidence: number;
  interpretation?: string;
  alternatives?: ConsentDecision[];

  warnings?: string[];
  recommendations?: string[];
}
```

---

## 3.9 Event and Audit Structures

```typescript
// Consent event structures
interface ConsentEvent {
  eventId: string;
  eventType: ConsentEventType;

  // Subject
  consentId: string;
  patientId: string;

  // Actor
  actor: EventActor;

  // Timing
  timestamp: Date;

  // Details
  details: Record<string, any>;

  // Context
  context: EventContext;

  // Related
  relatedEvents?: string[];
  causedBy?: string;
}

enum ConsentEventType {
  CREATED = 'CREATED',
  UPDATED = 'UPDATED',
  ACTIVATED = 'ACTIVATED',
  SUSPENDED = 'SUSPENDED',
  REVOKED = 'REVOKED',
  EXPIRED = 'EXPIRED',

  DECISION_QUERIED = 'DECISION_QUERIED',
  DECISION_APPLIED = 'DECISION_APPLIED',
  DECISION_OVERRIDDEN = 'DECISION_OVERRIDDEN',

  PROXY_ACTIVATED = 'PROXY_ACTIVATED',
  PROXY_DEACTIVATED = 'PROXY_DEACTIVATED',

  REVIEW_SCHEDULED = 'REVIEW_SCHEDULED',
  REVIEW_COMPLETED = 'REVIEW_COMPLETED',
  REVIEW_MISSED = 'REVIEW_MISSED',

  DOCUMENT_ADDED = 'DOCUMENT_ADDED',
  DOCUMENT_ACCESSED = 'DOCUMENT_ACCESSED',

  INTEGRITY_VERIFIED = 'INTEGRITY_VERIFIED',
  INTEGRITY_FAILED = 'INTEGRITY_FAILED',

  ESCALATION_TRIGGERED = 'ESCALATION_TRIGGERED',
  ESCALATION_RESOLVED = 'ESCALATION_RESOLVED',
}

interface EventActor {
  actorType: 'PATIENT' | 'PROXY' | 'STAFF' | 'SYSTEM' | 'EXTERNAL';
  actorId: string;
  actorName: string;
  role?: string;
  organization?: string;
}

interface EventContext {
  source: string;
  ipAddress?: string;
  userAgent?: string;
  sessionId?: string;
  requestId?: string;

  location?: {
    facility: string;
    jurisdiction: string;
  };

  reason?: string;
  notes?: string;
}

// Audit trail
interface ConsentAuditTrail {
  consentId: string;
  events: ConsentEvent[];

  // Summary
  summary: {
    createdAt: Date;
    createdBy: string;
    lastModifiedAt: Date;
    lastModifiedBy: string;
    totalModifications: number;
    currentStatus: ConsentStatus;
    statusChanges: number;
  };

  // Compliance
  compliance: {
    allAccessesLogged: boolean;
    allChangesAuthorized: boolean;
    integrityVerified: boolean;
    lastVerificationDate: Date;
  };
}
```

---

## 3.10 Validation Schemas

```typescript
// JSON Schema for consent validation
const consentSchema = {
  $schema: 'http://json-schema.org/draft-07/schema#',
  $id: 'https://wia.org/schemas/consent/v1.0.0',
  title: 'WIA Cryonics Consent Record',
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
    scope: {
      type: 'object',
      required: ['procedures', 'timeframe'],
      properties: {
        procedures: {
          type: 'array',
          items: { $ref: '#/definitions/procedureScope' },
          minItems: 1,
        },
        timeframe: { $ref: '#/definitions/timeframeScope' },
        conditions: {
          type: 'array',
          items: { $ref: '#/definitions/scopeCondition' },
        },
        limitations: {
          type: 'array',
          items: { $ref: '#/definitions/scopeLimitation' },
        },
      },
    },
    decisions: {
      type: 'array',
      items: { $ref: '#/definitions/consentDecision' },
      minItems: 1,
    },
    authority: { $ref: '#/definitions/consentAuthority' },
    validity: { $ref: '#/definitions/consentValidity' },
    documents: {
      type: 'array',
      items: { $ref: '#/definitions/consentDocument' },
    },
    metadata: { $ref: '#/definitions/consentMetadata' },
  },
  definitions: {
    procedureScope: {
      type: 'object',
      required: ['procedureType'],
      properties: {
        procedureType: { type: 'string' },
        specificProcedures: {
          type: 'array',
          items: { type: 'string' },
        },
        exclusions: {
          type: 'array',
          items: { type: 'string' },
        },
        conditions: {
          type: 'array',
          items: { type: 'string' },
        },
      },
    },
    timeframeScope: {
      type: 'object',
      required: ['type'],
      properties: {
        type: {
          type: 'string',
          enum: ['DEFINITE', 'INDEFINITE', 'CONDITIONAL'],
        },
        startDate: { type: 'string', format: 'date-time' },
        endDate: { type: 'string', format: 'date-time' },
        duration: { type: 'string' },
        conditionForExpiry: { type: 'string' },
      },
    },
    consentDecision: {
      type: 'object',
      required: ['decisionId', 'decisionType', 'question', 'answer'],
      properties: {
        decisionId: { type: 'string' },
        decisionType: {
          type: 'string',
          enum: ['BINARY', 'CHOICE', 'THRESHOLD', 'PREFERENCE', 'CONDITIONAL', 'DELEGATION'],
        },
        question: { type: 'string', minLength: 10 },
        answer: { $ref: '#/definitions/decisionAnswer' },
        reasoning: { type: 'string' },
      },
    },
    decisionAnswer: {
      type: 'object',
      required: ['type'],
      properties: {
        type: { type: 'string' },
        binaryValue: { type: 'boolean' },
        selectedOption: { type: 'string' },
        selectedOptions: {
          type: 'array',
          items: { type: 'string' },
        },
        thresholdValue: { type: 'number' },
        thresholdUnit: { type: 'string' },
        rankedOptions: {
          type: 'array',
          items: { $ref: '#/definitions/rankedOption' },
        },
      },
    },
    rankedOption: {
      type: 'object',
      required: ['option', 'rank'],
      properties: {
        option: { type: 'string' },
        rank: { type: 'integer', minimum: 1 },
        weight: { type: 'number', minimum: 0, maximum: 1 },
      },
    },
    consentAuthority: {
      type: 'object',
      required: ['grantor'],
      properties: {
        grantor: { $ref: '#/definitions/authorityEntity' },
        witnesses: {
          type: 'array',
          items: { $ref: '#/definitions/witness' },
        },
      },
    },
    authorityEntity: {
      type: 'object',
      required: ['entityId', 'entityType', 'capacityConfirmed'],
      properties: {
        entityId: { type: 'string' },
        entityType: {
          type: 'string',
          enum: ['PATIENT', 'PROXY', 'GUARDIAN', 'ORGANIZATION'],
        },
        capacityConfirmed: { type: 'boolean' },
        capacityDate: { type: 'string', format: 'date-time' },
      },
    },
    witness: {
      type: 'object',
      required: ['witnessId', 'name', 'attestation'],
      properties: {
        witnessId: { type: 'string' },
        name: { type: 'string' },
        role: { type: 'string' },
        relationship: { type: 'string' },
        attestation: { $ref: '#/definitions/witnessAttestation' },
      },
    },
    witnessAttestation: {
      type: 'object',
      required: ['attestationDate'],
      properties: {
        attestationDate: { type: 'string', format: 'date-time' },
        attestsToVoluntariness: { type: 'boolean' },
        attestsToCapacity: { type: 'boolean' },
        attestsToIdentity: { type: 'boolean' },
      },
    },
    consentValidity: {
      type: 'object',
      required: ['status', 'effectiveDate'],
      properties: {
        status: {
          type: 'string',
          enum: ['DRAFT', 'PENDING_WITNESS', 'PENDING_NOTARIZATION', 'PENDING_REVIEW', 'ACTIVE', 'SUSPENDED', 'REVOKED', 'EXPIRED', 'SUPERSEDED'],
        },
        effectiveDate: { type: 'string', format: 'date-time' },
        expirationDate: { type: 'string', format: 'date-time' },
      },
    },
    consentDocument: {
      type: 'object',
      required: ['documentId', 'documentType', 'title'],
      properties: {
        documentId: { type: 'string' },
        documentType: { type: 'string' },
        title: { type: 'string' },
        language: { type: 'string' },
      },
    },
    consentMetadata: {
      type: 'object',
      required: ['version', 'createdAt'],
      properties: {
        version: { type: 'integer', minimum: 1 },
        createdAt: { type: 'string', format: 'date-time' },
        updatedAt: { type: 'string', format: 'date-time' },
        createdBy: { type: 'string' },
        documentFormat: { type: 'string' },
      },
    },
  },
};

// Validation service
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

    // Additional business rule validations
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

    // Rule: Active consent must have grantor capacity confirmed
    if (consent.validity.status === 'ACTIVE' && !consent.authority.grantor.capacityConfirmed) {
      errors.push({
        path: 'authority.grantor.capacityConfirmed',
        message: 'Active consent requires confirmed grantor capacity',
        code: 'CAPACITY_NOT_CONFIRMED',
      });
    }

    // Rule: Indefinite timeframe must not have end date
    if (consent.scope.timeframe.type === 'INDEFINITE' && consent.scope.timeframe.endDate) {
      errors.push({
        path: 'scope.timeframe',
        message: 'Indefinite timeframe should not have end date',
        code: 'INVALID_TIMEFRAME',
      });
    }

    // Rule: At least one decision required
    if (!consent.decisions || consent.decisions.length === 0) {
      errors.push({
        path: 'decisions',
        message: 'At least one decision is required',
        code: 'NO_DECISIONS',
      });
    }

    // Rule: Revival consent requires quality of life preferences
    if (consent.category === 'REVIVAL') {
      const hasQoLDecision = consent.decisions.some(
        d => d.decisionType === 'THRESHOLD' && d.question.toLowerCase().includes('quality')
      );
      if (!hasQoLDecision) {
        errors.push({
          path: 'decisions',
          message: 'Revival consent should include quality of life preferences',
          code: 'MISSING_QOL_PREFERENCE',
          severity: 'WARNING',
        });
      }
    }

    return errors;
  }

  private formatErrors(ajvErrors: ErrorObject[]): ValidationError[] {
    return ajvErrors.map(err => ({
      path: err.instancePath || err.schemaPath,
      message: err.message || 'Validation failed',
      code: err.keyword.toUpperCase(),
      params: err.params,
    }));
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

*Next Chapter: API Interface - RESTful and GraphQL APIs for consent management*
