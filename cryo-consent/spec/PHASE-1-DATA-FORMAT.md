# CRYO-CONSENT Phase 1: Data Format Specification

## Overview

This document defines the data structures for cryopreservation consent management, including consent documents, revival conditions, family agreements, and amendment tracking.

## Core Interfaces

### Consent Document

```typescript
interface CryoConsentDocument {
  documentId: string;                    // "CONSENT-2025-001"
  version: string;                       // "1.0.0"
  status: ConsentStatus;

  subject: SubjectInformation;
  consentDeclarations: ConsentDeclarations;
  revivalConditions: RevivalConditions;
  familyAgreements: FamilyAgreements;
  financialArrangements: FinancialArrangements;
  signatures: SignatureBlock;
  amendments: Amendment[];

  createdAt: string;                     // ISO 8601
  lastModified: string;
  effectiveDate: string;
  expirationDate?: string;               // Optional, if time-limited
}

type ConsentStatus =
  | 'DRAFT'
  | 'PENDING_SIGNATURES'
  | 'PENDING_NOTARIZATION'
  | 'ACTIVE'
  | 'SUSPENDED'
  | 'WITHDRAWN'
  | 'EXECUTED'                           // Preservation began
  | 'COMPLETED';                         // Revival completed
```

### Subject Information

```typescript
interface SubjectInformation {
  subjectId: string;
  identityRef: string;                   // Reference to CRYO-IDENTITY

  personalInfo: {
    fullLegalName: string;
    dateOfBirth: string;
    placeOfBirth: string;
    nationality: string[];
    currentAddress: EncryptedData;
    contactInfo: EncryptedData;
  };

  legalCapacity: LegalCapacity;
  medicalSummary: MedicalSummary;
  mentalCapacityAssessment: MentalCapacityAssessment;
}

interface LegalCapacity {
  assessmentDate: string;
  assessedBy: string;
  capacity: 'FULL' | 'LIMITED' | 'ASSISTED';
  limitations?: string[];
  legalRepresentative?: LegalRepresentative;
  courtOrders?: CourtOrder[];
}

interface MentalCapacityAssessment {
  assessmentId: string;
  assessmentDate: string;
  assessor: {
    name: string;
    credentials: string;
    licenseNumber: string;
  };

  understanding: {
    preservationProcess: boolean;
    risksAndLimitations: boolean;
    noGuarantees: boolean;
    financialImplications: boolean;
    familyImpact: boolean;
  };

  decisionMaking: {
    voluntary: boolean;
    noCoercion: boolean;
    understoodAlternatives: boolean;
  };

  overallCapacity: 'CAPABLE' | 'INCAPABLE' | 'QUESTIONABLE';
  notes: string;
  signature: string;
}

interface MedicalSummary {
  terminalConditions: TerminalCondition[];
  currentTreatments: Treatment[];
  lifeSupportStatus?: LifeSupportStatus;
  physiciansStatement: PhysicianStatement;
}

interface TerminalCondition {
  condition: string;
  diagnosisDate: string;
  prognosis: string;
  treatingPhysician: string;
  documentation: string[];              // Vault references
}
```

### Consent Declarations

```typescript
interface ConsentDeclarations {
  preservationConsent: PreservationConsent;
  revivalConsent: RevivalConsent;
  researchConsent: ResearchConsent;
  alternativeDirectives: AlternativeDirectives;
}

interface PreservationConsent {
  consentType: ConsentType;

  acknowledgedRisks: AcknowledgedRisk[];
  acknowledgedLimitations: string[];
  acknowledgedUncertainties: string[];

  preferredMethod: PreservationMethod;
  facilityPreferences: FacilityPreference[];

  bodyDisposition: {
    wholeBody: boolean;
    neuroOnly: boolean;                  // Head/brain only
  };

  emergencyPreservation: {
    authorized: boolean;
    conditions: string[];
    maximumDelay: string;                // Duration
  };
}

type ConsentType =
  | 'FULL'                               // All options enabled
  | 'CONDITIONAL'                        // Specific conditions
  | 'LIMITED'                            // Restricted scenarios
  | 'RESEARCH_ONLY';                     // No revival intended

type PreservationMethod =
  | 'VITRIFICATION'
  | 'SLOW_FREEZE'
  | 'PROVIDER_DISCRETION';

interface AcknowledgedRisk {
  riskId: string;
  riskCategory: RiskCategory;
  description: string;
  acknowledged: boolean;
  acknowledgedAt: string;
}

type RiskCategory =
  | 'TECHNICAL'                          // Equipment failure
  | 'BIOLOGICAL'                         // Cell damage
  | 'ORGANIZATIONAL'                     // Company failure
  | 'LEGAL'                              // Law changes
  | 'FINANCIAL'                          // Funding issues
  | 'UNKNOWN';                           // Future unknowns

interface RevivalConsent {
  revivalAuthorized: boolean;
  revivalConditions: RevivalCondition[];
  revivalRefusalConditions: RefusalCondition[];

  postRevivalDirectives: {
    medicalCare: MedicalCareDirective;
    identityRestoration: boolean;
    familyReunification: boolean;
    financialAccess: boolean;
  };
}

interface ResearchConsent {
  authorized: boolean;
  scope: ResearchScope;

  allowedResearch: {
    tissuesampling: boolean;
    imagingStudies: boolean;
    geneticResearch: boolean;
    procedureObservation: boolean;
  };

  dataSharing: {
    anonymizedData: boolean;
    identifiableData: boolean;
    commercialUse: boolean;
  };

  exclusions: string[];
}

type ResearchScope =
  | 'NONE'
  | 'PRESERVATION_RESEARCH'
  | 'REVIVAL_RESEARCH'
  | 'GENERAL_MEDICAL'
  | 'UNRESTRICTED';
```

### Revival Conditions

```typescript
interface RevivalConditions {
  conditionLogic: 'ALL' | 'ANY' | 'CUSTOM';
  customLogic?: string;                  // Boolean expression

  conditions: RevivalCondition[];

  minimumPreservationDuration?: string;  // Duration, e.g., "P50Y"
  maximumPreservationDuration?: string;

  emergencyRevivalAuthorized: boolean;
  emergencyConditions?: EmergencyCondition[];
}

interface RevivalCondition {
  conditionId: string;
  conditionType: ConditionType;
  description: string;

  required: boolean;
  weight?: number;                       // For weighted evaluation

  parameters: ConditionParameters;
  verificationMethod: VerificationMethod;

  status: 'PENDING' | 'MET' | 'FAILED' | 'WAIVED';
  statusHistory: ConditionStatusChange[];
}

type ConditionType =
  | 'MEDICAL'                            // Disease cure
  | 'TECHNOLOGY'                         // Revival tech
  | 'TIME'                               // Duration
  | 'SOCIAL'                             // Family/guardian
  | 'LEGAL'                              // Legal status
  | 'FINANCIAL'                          // Funding available
  | 'CUSTOM';

interface MedicalConditionParams {
  conditionType: 'MEDICAL';
  targetCondition: string;               // e.g., "cancer"
  requiredOutcome: 'CURABLE' | 'TREATABLE' | 'MANAGEABLE';
  evidenceRequired: string[];
  verifyingAuthority: string;
}

interface TechnologyConditionParams {
  conditionType: 'TECHNOLOGY';
  requiredCapability: string;            // e.g., "successful_human_revival"
  minimumSuccessRate?: number;           // Percentage
  minimumCases?: number;
  verifyingAuthority: string;
}

interface TimeConditionParams {
  conditionType: 'TIME';
  minimumDuration: string;               // Duration
  maximumDuration?: string;
  referencePoint: 'PRESERVATION_START' | 'LEGAL_DEATH' | 'FACILITY_REGISTRATION';
}

interface SocialConditionParams {
  conditionType: 'SOCIAL';
  requiredApprovals: GuardianApproval[];
  familyNotification: boolean;
  publicAnnouncement?: boolean;
}

type ConditionParameters =
  | MedicalConditionParams
  | TechnologyConditionParams
  | TimeConditionParams
  | SocialConditionParams
  | CustomConditionParams;

interface VerificationMethod {
  method: 'AUTOMATIC' | 'GUARDIAN_VOTE' | 'AUTHORITY_CERTIFICATION' | 'COURT_ORDER';
  authority?: string;
  threshold?: number;
  documentation: string[];
}

interface RefusalCondition {
  conditionId: string;
  description: string;

  refusalTriggers: {
    specificConditions: string[];        // If these exist, refuse revival
    qualityOfLife: QualityOfLifeThreshold;
    socialConditions: string[];
  };

  overrideAllowed: boolean;
  overrideRequirements?: string[];
}

interface QualityOfLifeThreshold {
  minimumMobility: 'FULL' | 'ASSISTED' | 'WHEELCHAIR' | 'BEDRIDDEN' | 'ANY';
  minimumCognition: 'FULL' | 'MILD_IMPAIRMENT' | 'MODERATE' | 'SEVERE' | 'ANY';
  minimumCommunication: 'VERBAL' | 'WRITTEN' | 'ASSISTED' | 'ANY';
}
```

### Family Agreements

```typescript
interface FamilyAgreements {
  spouseAgreement?: SpouseAgreement;
  childrenAgreements: ChildAgreement[];
  parentAgreements: ParentAgreement[];
  guardianDesignations: GuardianDesignation[];

  conflictResolution: ConflictResolutionPlan;
  familyNotificationPlan: NotificationPlan;
}

interface SpouseAgreement {
  spouseId: string;
  spouseName: string;
  marriageStatus: 'MARRIED' | 'SEPARATED' | 'DIVORCED' | 'WIDOWED';

  consentGiven: boolean;
  consentScope: SpouseConsentScope;

  rightsAfterPreservation: {
    remarriageAllowed: boolean;
    inheritanceRights: string;
    decisionMakingRole: 'PRIMARY' | 'SECONDARY' | 'NONE';
    revivalVoteWeight: number;
  };

  jointPreservation?: {
    requested: boolean;
    conditions: string[];
  };

  signature: SignatureRecord;
}

interface SpouseConsentScope {
  preservationApproved: boolean;
  financialArrangementApproved: boolean;
  revivalConditionsApproved: boolean;

  objections: string[];
  negotiatedTerms: string[];
}

interface ChildAgreement {
  childId: string;
  childName: string;
  dateOfBirth: string;
  isMinor: boolean;

  notification: {
    notified: boolean;
    notificationDate: string;
    method: string;
  };

  acknowledgment?: {
    acknowledged: boolean;
    acknowledgedDate: string;
    concerns: string[];
  };

  guardianRole?: {
    designated: boolean;
    responsibilities: string[];
    voteWeight: number;
  };

  signature?: SignatureRecord;           // If adult
  legalGuardianSignature?: SignatureRecord; // If minor
}

interface GuardianDesignation {
  guardianId: string;
  guardianType: 'PERSON' | 'ORGANIZATION' | 'SMART_CONTRACT';

  personalInfo?: {
    name: string;
    relationship: string;
    contactInfo: EncryptedData;
  };

  organizationInfo?: {
    name: string;
    type: string;
    registrationNumber: string;
    contactInfo: EncryptedData;
  };

  responsibilities: GuardianResponsibility[];
  voteWeight: number;
  succession: SuccessionPlan;

  acceptanceRecord: {
    accepted: boolean;
    acceptedDate: string;
    signature: SignatureRecord;
  };
}

type GuardianResponsibility =
  | 'CONSENT_DECISIONS'
  | 'FINANCIAL_OVERSIGHT'
  | 'FACILITY_LIAISON'
  | 'REVIVAL_DECISIONS'
  | 'FAMILY_COMMUNICATION'
  | 'LEGAL_REPRESENTATION';

interface ConflictResolutionPlan {
  primaryMethod: 'MEDIATION' | 'ARBITRATION' | 'COURT';
  mediator?: string;
  arbitrationProvider?: string;
  governingLaw: string;
  venue: string;

  escalationPath: EscalationStep[];
  deadlockResolution: string;
}
```

### Financial Arrangements

```typescript
interface FinancialArrangements {
  fundingMethod: FundingMethod;
  totalCommitment: MonetaryAmount;

  paymentSchedule: PaymentSchedule;
  trustArrangement?: TrustArrangement;
  insuranceCoverage?: InsuranceCoverage[];

  refundPolicy: RefundPolicy;
  defaultProcedure: DefaultProcedure;
}

type FundingMethod =
  | 'LUMP_SUM'
  | 'INSTALLMENTS'
  | 'TRUST_FUNDED'
  | 'INSURANCE_FUNDED'
  | 'HYBRID';

interface PaymentSchedule {
  initialPayment: Payment;
  recurringPayments?: RecurringPayment[];
  finalPayment?: Payment;

  adjustmentClauses: AdjustmentClause[];
  currencyStabilization: CurrencyStabilization;
}

interface TrustArrangement {
  trustId: string;
  trustType: 'REVOCABLE' | 'IRREVOCABLE' | 'CHARITABLE';
  trustee: TrusteeInfo;

  fundingLevel: MonetaryAmount;
  investmentStrategy: string;
  disbursementRules: DisbursementRule[];

  crossReference: string;                // Reference to CRYO-ASSET
}

interface RefundPolicy {
  prePreservationWithdrawal: {
    fullRefund: boolean;
    refundPercentage: number;
    deductions: string[];
    processingTime: string;
  };

  duringPreservation: {
    refundAvailable: boolean;
    conditions: string[];
    calculation: string;
  };

  revivalRefusal: {
    refundAvailable: boolean;
    beneficiaries: string[];
  };
}
```

### Signatures

```typescript
interface SignatureBlock {
  subjectSignature: SignatureRecord;
  witnessSignatures: SignatureRecord[];
  notarization?: NotarizationRecord;

  legalCounselReview?: LegalReview;
  medicalAdvisorReview?: MedicalReview;
}

interface SignatureRecord {
  signerId: string;
  signerName: string;
  signerRole: SignerRole;

  signatureType: 'WET' | 'DIGITAL' | 'BIOMETRIC';
  signatureData: string;                 // Encrypted

  timestamp: string;
  location: string;
  ipAddress?: string;
  deviceInfo?: string;

  verificationStatus: 'PENDING' | 'VERIFIED' | 'INVALID';
  verificationMethod: string;
  verificationDate?: string;
}

type SignerRole =
  | 'SUBJECT'
  | 'SPOUSE'
  | 'CHILD'
  | 'GUARDIAN'
  | 'WITNESS'
  | 'NOTARY'
  | 'LEGAL_COUNSEL'
  | 'MEDICAL_ADVISOR'
  | 'FACILITY_REPRESENTATIVE';

interface NotarizationRecord {
  notaryId: string;
  notaryName: string;
  notaryLicense: string;
  jurisdiction: string;

  notarizationDate: string;
  notarizationLocation: string;
  sealNumber: string;

  expirationDate: string;
  verificationUrl?: string;
}
```

### Amendments

```typescript
interface Amendment {
  amendmentId: string;
  amendmentNumber: number;

  effectiveDate: string;
  createdAt: string;

  amendmentType: AmendmentType;
  description: string;

  changes: AmendmentChange[];

  initiator: string;
  reason: string;

  approvals: AmendmentApproval[];
  requiredApprovals: number;

  status: 'PENDING' | 'APPROVED' | 'REJECTED' | 'SUPERSEDED';

  signatures: SignatureRecord[];
  previousVersion: string;
}

type AmendmentType =
  | 'REVIVAL_CONDITIONS'
  | 'GUARDIAN_CHANGE'
  | 'FINANCIAL_ADJUSTMENT'
  | 'FACILITY_TRANSFER'
  | 'RESEARCH_CONSENT'
  | 'COMPLETE_WITHDRAWAL'
  | 'OTHER';

interface AmendmentChange {
  section: string;
  field: string;
  previousValue: any;
  newValue: any;
  justification: string;
}

interface AmendmentApproval {
  approverId: string;
  approverRole: string;
  decision: 'APPROVE' | 'REJECT' | 'ABSTAIN';
  timestamp: string;
  notes?: string;
  signature: SignatureRecord;
}
```

## Alternative Directives

```typescript
interface AlternativeDirectives {
  ifRevivalImpossible: ImpossibleRevivalDirective;
  ifFacilityFails: FacilityFailureDirective;
  ifFundingFails: FundingFailureDirective;
  ifLawChanges: LegalChangeDirective;
}

interface ImpossibleRevivalDirective {
  waitDuration: string;                  // How long to wait
  afterWaiting: 'CONTINUE' | 'TERMINATE' | 'RESEARCH_DONATION';

  researchDonation?: {
    authorized: boolean;
    institutions: string[];
    restrictions: string[];
  };

  termination?: {
    method: 'CREMATION' | 'BURIAL' | 'FACILITY_DISCRETION';
    remainsDisposition: string;
    ceremonyPreferences: string;
  };
}

interface FacilityFailureDirective {
  transferPriority: string[];            // Preferred facilities
  transferFunding: string;

  ifNoTransferPossible: 'TERMINATE' | 'GOVERNMENT_CUSTODY' | 'RESEARCH';

  insuranceClaim: {
    authorized: boolean;
    beneficiaries: string[];
  };
}

interface FundingFailureDirective {
  gracePeriod: string;

  fundingRecoveryAttempts: {
    familyAppeal: boolean;
    crowdfunding: boolean;
    charitableFunds: boolean;
    governmentAssistance: boolean;
  };

  ifFundingNotRestored: 'TERMINATE' | 'REDUCED_SERVICE' | 'RESEARCH';
}

interface LegalChangeDirective {
  ifPreservationBanned: 'TRANSFER_JURISDICTION' | 'TERMINATE' | 'CHALLENGE';
  preferredJurisdictions: string[];

  ifRevivalBanned: 'WAIT' | 'TRANSFER' | 'TERMINATE';
  waitDuration: string;

  legalDefenseFund: {
    authorized: boolean;
    maximumAmount: MonetaryAmount;
  };
}
```

## JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.org/schemas/cryo-consent/v1",
  "title": "CryoConsentDocument",
  "type": "object",
  "required": [
    "documentId",
    "version",
    "status",
    "subject",
    "consentDeclarations",
    "signatures"
  ],
  "properties": {
    "documentId": {
      "type": "string",
      "pattern": "^CONSENT-[0-9]{4}-[0-9]{3}$"
    },
    "version": {
      "type": "string",
      "pattern": "^[0-9]+\\.[0-9]+\\.[0-9]+$"
    },
    "status": {
      "type": "string",
      "enum": [
        "DRAFT",
        "PENDING_SIGNATURES",
        "PENDING_NOTARIZATION",
        "ACTIVE",
        "SUSPENDED",
        "WITHDRAWN",
        "EXECUTED",
        "COMPLETED"
      ]
    }
  }
}
```

## Validation Rules

1. **Mental Capacity**: Subject must have documented mental capacity assessment
2. **Witness Requirements**: Minimum 2 witnesses, not family members
3. **Notarization**: Required for execution
4. **Amendment Thresholds**: Critical changes require guardian majority
5. **Financial Verification**: Funding must be confirmed before activation

---

*WIA Technical Committee - Cryopreservation Working Group*
