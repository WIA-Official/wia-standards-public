/**
 * WIA-TIME-031: Temporal Law - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Legal Committee
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Legal Types
// ============================================================================

/**
 * Time crime classifications
 */
export enum TimeCrimeClass {
  CLASS_A_CATASTROPHIC = 'class-a-catastrophic',
  CLASS_B_MAJOR = 'class-b-major',
  CLASS_C_SERIOUS = 'class-c-serious',
  CLASS_D_MODERATE = 'class-d-moderate',
  CLASS_E_MINOR = 'class-e-minor',
}

/**
 * Legal status types
 */
export enum LegalStatus {
  CITIZEN = 'citizen',
  RESIDENT = 'resident',
  VISITOR = 'visitor',
  REFUGEE = 'refugee',
  STATELESS = 'stateless',
  CRIMINAL = 'criminal',
}

/**
 * Jurisdiction levels
 */
export enum JurisdictionLevel {
  ORIGIN_TIMELINE = 'origin-timeline',
  DESTINATION_TIMELINE = 'destination-timeline',
  INTERNATIONAL_TEMPORAL_COURT = 'international-temporal-court',
  SUPREME_TEMPORAL_TRIBUNAL = 'supreme-temporal-tribunal',
}

/**
 * Court levels
 */
export enum CourtLevel {
  LOCAL_TEMPORAL = 'local-temporal',
  REGIONAL_APPEALS = 'regional-appeals',
  NATIONAL_SUPREME = 'national-supreme',
  INTERNATIONAL = 'international',
  SUPREME_TRIBUNAL = 'supreme-tribunal',
}

// ============================================================================
// Temporal Jurisdiction
// ============================================================================

/**
 * Geographic location
 */
export interface GeoLocation {
  /** Latitude in degrees */
  lat: number;

  /** Longitude in degrees */
  lon: number;

  /** Altitude in meters (optional) */
  alt?: number;

  /** Location description */
  description?: string;
}

/**
 * Temporal jurisdiction determination
 */
export interface TemporalJurisdiction {
  /** Primary jurisdiction */
  primary: JurisdictionLevel;

  /** Secondary jurisdictions */
  secondary: JurisdictionLevel[];

  /** Applicable laws */
  applicableLaws: string[];

  /** Jurisdiction rationale */
  rationale: string;

  /** Conflicts present */
  conflicts?: JurisdictionConflict[];

  /** Resolution mechanism */
  resolutionMechanism?: string;
}

/**
 * Jurisdiction conflict
 */
export interface JurisdictionConflict {
  /** Conflict type */
  type: 'territorial' | 'temporal' | 'substantive' | 'procedural';

  /** Conflicting jurisdictions */
  jurisdictions: JurisdictionLevel[];

  /** Description of conflict */
  description: string;

  /** Proposed resolution */
  resolution?: string;
}

/**
 * Jurisdiction check request
 */
export interface JurisdictionCheckRequest {
  /** Traveler identifier */
  traveler: string;

  /** Origin timeline */
  originTimeline: string | Date;

  /** Destination timeline */
  destinationTimeline: string | Date;

  /** Location */
  location: GeoLocation;

  /** Activity type */
  activityType?: string;

  /** Purpose */
  purpose?: string;
}

// ============================================================================
// Time Crimes
// ============================================================================

/**
 * Time crime types
 */
export enum TimeCrimeType {
  TIMELINE_DESTRUCTION = 'timeline-destruction',
  TEMPORAL_GENOCIDE = 'temporal-genocide',
  PARADOX_WARFARE = 'paradox-warfare',
  TEMPORAL_TERRORISM = 'temporal-terrorism',
  HISTORICAL_ALTERATION = 'historical-alteration',
  TEMPORAL_ASSASSINATION = 'temporal-assassination',
  UNAUTHORIZED_TIMELINE_CREATION = 'unauthorized-timeline-creation',
  TIME_WAR_INSTIGATION = 'time-war-instigation',
  TECHNOLOGY_TRANSFER = 'technology-transfer',
  TEMPORAL_EXPLOITATION = 'temporal-exploitation',
  HISTORICAL_MANIPULATION = 'historical-manipulation',
  UNAUTHORIZED_INTERVENTION = 'unauthorized-intervention',
  OBSERVER_PROTOCOL_VIOLATION = 'observer-protocol-violation',
  MINOR_HISTORICAL_INTERFERENCE = 'minor-historical-interference',
  TEMPORAL_TRESPASSING = 'temporal-trespassing',
  DOCUMENT_FALSIFICATION = 'document-falsification',
  CERTIFICATION_VIOLATION = 'certification-violation',
  REPORTING_FAILURE = 'reporting-failure',
  ADMINISTRATIVE_OFFENSE = 'administrative-offense',
  TECHNICAL_VIOLATION = 'technical-violation',
}

/**
 * Time crime definition
 */
export interface TimeCrime {
  /** Crime identifier */
  id: string;

  /** Crime type */
  type: TimeCrimeType;

  /** Classification */
  class: TimeCrimeClass;

  /** Description */
  description: string;

  /** Statutory reference */
  statute: string;

  /** Penalty range */
  penaltyRange: {
    minimum: string;
    maximum: string;
  };

  /** Elements of crime */
  elements: string[];

  /** Defenses available */
  defenses?: string[];

  /** Statute of limitations (years, -1 = none) */
  statuteOfLimitations: number;
}

/**
 * Crime report
 */
export interface CrimeReport {
  /** Report identifier */
  id: string;

  /** Reporter information */
  reporter: {
    id: string;
    name?: string;
    contact?: string;
    timeline: string;
  };

  /** Crime details */
  crime: {
    type: TimeCrimeType;
    class: TimeCrimeClass;
    date: Date;
    timeline: string | Date;
    location: GeoLocation;
    description: string;
  };

  /** Suspect information */
  suspect?: {
    name?: string;
    description?: string;
    originTimeline?: string;
    id?: string;
  };

  /** Evidence */
  evidence: {
    timelineVerification?: boolean;
    witnessStatements?: string[];
    physicalEvidence?: string[];
    digitalRecords?: string[];
    other?: string[];
  };

  /** Timeline impact assessment */
  timelineImpact: {
    detected: boolean;
    severity: 'none' | 'minimal' | 'moderate' | 'severe' | 'critical';
    description: string;
  };

  /** Report status */
  status: 'filed' | 'under-review' | 'investigating' | 'prosecuting' | 'closed';

  /** Filing date */
  filedDate: Date;

  /** Last updated */
  lastUpdated: Date;
}

// ============================================================================
// Traveler Rights
// ============================================================================

/**
 * Temporal traveler rights
 */
export interface TravelerRights {
  /** Right to legal counsel */
  legalCounsel: boolean;

  /** Right to fair trial */
  fairTrial: boolean;

  /** Right to timeline integrity (non-discrimination) */
  timelineIntegrity: boolean;

  /** Right to property */
  property: boolean;

  /** Right to contracts */
  contracts: boolean;

  /** Right to appeal */
  appeal: boolean;

  /** Right to refuge/asylum */
  refuge: boolean;

  /** Right to return to origin timeline */
  returnRight: boolean;

  /** Additional rights */
  additionalRights?: string[];

  /** Restrictions */
  restrictions?: string[];
}

/**
 * Legal rights invocation
 */
export interface RightsInvocation {
  /** Right being invoked */
  right: keyof TravelerRights | string;

  /** Basis for invocation */
  basis: string;

  /** Timeline of invocation */
  timeline: string | Date;

  /** Supporting evidence */
  evidence?: string[];

  /** Status */
  status: 'pending' | 'granted' | 'denied' | 'under-review';

  /** Decision rationale */
  rationale?: string;
}

// ============================================================================
// Legal Status
// ============================================================================

/**
 * Traveler registration
 */
export interface TravelerRegistration {
  /** Traveler identifier */
  travelerId: string;

  /** Full name */
  name: string;

  /** Citizenship */
  citizenship: string;

  /** Origin timeline */
  originTimeline: string | Date;

  /** Destination timeline */
  destinationTimeline: string | Date;

  /** Purpose of travel */
  purpose: string;

  /** Duration (seconds) */
  duration: number;

  /** Registration date */
  registrationDate: Date;

  /** Status */
  status: 'pending' | 'approved' | 'denied' | 'active' | 'expired';

  /** Legal protections granted */
  protections: string[];

  /** Restrictions */
  restrictions?: string[];

  /** Emergency contact */
  emergencyContact?: {
    name: string;
    relationship: string;
    contact: string;
  };
}

/**
 * Legal status check
 */
export interface LegalStatusCheck {
  /** Traveler identifier */
  travelerId: string;

  /** Timeline being checked */
  timeline: string | Date;

  /** Current status */
  status: LegalStatus;

  /** Citizenship status */
  citizenship: {
    primary: string;
    dual?: string[];
    stateless: boolean;
  };

  /** Residency */
  residency?: {
    timeline: string;
    type: 'permanent' | 'temporary' | 'visitor';
    expires?: Date;
  };

  /** Criminal record */
  criminalRecord?: {
    hasRecord: boolean;
    convictions?: string[];
    pending?: string[];
  };

  /** Active warrants */
  warrants?: {
    count: number;
    jurisdictions: string[];
    types: string[];
  };

  /** Legal restrictions */
  restrictions?: string[];

  /** Rights available */
  availableRights: TravelerRights;

  /** Last updated */
  lastUpdated: Date;
}

// ============================================================================
// Property Rights
// ============================================================================

/**
 * Property types
 */
export enum PropertyType {
  REAL_PROPERTY = 'real-property',
  PERSONAL_PROPERTY = 'personal-property',
  INTELLECTUAL_PROPERTY = 'intellectual-property',
  CULTURAL_ARTIFACT = 'cultural-artifact',
  NATURAL_RESOURCE = 'natural-resource',
  FINANCIAL_ASSET = 'financial-asset',
}

/**
 * Property ownership types
 */
export enum OwnershipType {
  SOLE = 'sole',
  JOINT = 'joint',
  TENANTS_IN_COMMON = 'tenants-in-common',
  COMMUNITY = 'community',
  TRUST = 'trust',
  CORPORATE = 'corporate',
}

/**
 * Property claim
 */
export interface PropertyClaim {
  /** Claim identifier */
  id: string;

  /** Claimant */
  claimant: string;

  /** Property type */
  propertyType: PropertyType;

  /** Description */
  description: string;

  /** Timeline of property */
  timeline: string | Date;

  /** Location */
  location?: GeoLocation;

  /** Basis for claim */
  basis: 'original-ownership' | 'discovery' | 'purchase' | 'inheritance' | 'creation' | 'other';

  /** Supporting evidence */
  evidence: {
    documents?: string[];
    witnesses?: string[];
    physicalEvidence?: string[];
    timelineVerification?: boolean;
  };

  /** Competing claims */
  competingClaims?: string[];

  /** Value (if applicable) */
  value?: {
    amount: number;
    currency: string;
    timeline: string;
  };

  /** Status */
  status: 'filed' | 'under-review' | 'approved' | 'denied' | 'contested';

  /** Filing date */
  filedDate: Date;

  /** Decision */
  decision?: {
    outcome: 'granted' | 'denied' | 'partial';
    rationale: string;
    conditions?: string[];
    date: Date;
  };
}

/**
 * Property transfer
 */
export interface PropertyTransfer {
  /** Transfer identifier */
  id: string;

  /** Property being transferred */
  propertyId: string;

  /** Transferor (seller/grantor) */
  transferor: string;

  /** Transferee (buyer/grantee) */
  transferee: string;

  /** Transfer type */
  type: 'sale' | 'gift' | 'inheritance' | 'exchange' | 'lease';

  /** Consideration */
  consideration?: {
    type: 'monetary' | 'property' | 'services' | 'other';
    value?: number;
    description?: string;
  };

  /** Timeline of transfer */
  transferTimeline: string | Date;

  /** Effective date */
  effectiveDate: Date;

  /** Conditions */
  conditions?: string[];

  /** Recording */
  recording?: {
    recorded: boolean;
    registryId?: string;
    date?: Date;
  };

  /** Legal validity */
  validity: {
    valid: boolean;
    issues?: string[];
  };
}

// ============================================================================
// Temporal Contracts
// ============================================================================

/**
 * Contract types
 */
export enum ContractType {
  RESEARCH_COLLABORATION = 'research-collaboration',
  EMPLOYMENT = 'employment',
  TRADE_AGREEMENT = 'trade-agreement',
  SERVICE_CONTRACT = 'service-contract',
  LICENSING = 'licensing',
  PROPERTY_TRANSFER = 'property-transfer',
  PARTNERSHIP = 'partnership',
  NONDISCLOSURE = 'nondisclosure',
  OTHER = 'other',
}

/**
 * Temporal contract
 */
export interface TemporalContract {
  /** Contract identifier */
  id: string;

  /** Contract type */
  type: ContractType;

  /** Parties to contract */
  parties: string[];

  /** Contract terms */
  terms: string;

  /** Origin timeline (where contract created) */
  originTimeline: Date | string;

  /** Execution timeline (where contract performed) */
  executionTimeline: Date | string;

  /** Duration (seconds) */
  duration: number;

  /** Consideration */
  consideration: string;

  /** Governing law */
  governingLaw?: string;

  /** Dispute resolution */
  disputeResolution?: {
    method: 'litigation' | 'arbitration' | 'mediation';
    forum?: string;
    rules?: string;
  };

  /** Signatures */
  signatures: {
    party: string;
    signed: boolean;
    signatureDate?: Date;
    timeline?: string;
  }[];

  /** Effective date */
  effectiveDate?: Date;

  /** Termination date */
  terminationDate?: Date;

  /** Status */
  status: 'draft' | 'executed' | 'active' | 'completed' | 'breached' | 'terminated';

  /** Amendments */
  amendments?: {
    date: Date;
    description: string;
    parties: string[];
  }[];
}

/**
 * Contract validation result
 */
export interface ContractValidation {
  /** Is contract valid? */
  valid: boolean;

  /** Validation issues */
  issues: string[];

  /** Enforceability */
  enforceability: 'fully-enforceable' | 'partially-enforceable' | 'unenforceable';

  /** Enforceability rationale */
  rationale: string;

  /** Required modifications */
  requiredModifications?: string[];

  /** Timeline compatibility */
  timelineCompatibility: {
    compatible: boolean;
    conflicts?: string[];
  };

  /** Legal compliance */
  legalCompliance: {
    compliant: boolean;
    violations?: string[];
  };

  /** Recommendations */
  recommendations?: string[];
}

/**
 * Contract dispute
 */
export interface ContractDispute {
  /** Dispute identifier */
  id: string;

  /** Contract ID */
  contractId: string;

  /** Disputing parties */
  parties: {
    claimant: string;
    respondent: string;
  };

  /** Dispute type */
  type: 'breach' | 'interpretation' | 'performance' | 'termination' | 'other';

  /** Description */
  description: string;

  /** Claimed damages */
  damages?: {
    amount: number;
    type: 'compensatory' | 'punitive' | 'consequential';
    timeline: string;
  };

  /** Relief sought */
  reliefSought: string[];

  /** Status */
  status: 'filed' | 'mediation' | 'arbitration' | 'litigation' | 'settled' | 'decided';

  /** Resolution */
  resolution?: {
    outcome: string;
    date: Date;
    terms?: string;
  };

  /** Filing date */
  filedDate: Date;
}

// ============================================================================
// Court Proceedings
// ============================================================================

/**
 * Legal case types
 */
export enum CaseType {
  CRIMINAL = 'criminal',
  CIVIL = 'civil',
  ADMINISTRATIVE = 'administrative',
  CONSTITUTIONAL = 'constitutional',
  APPELLATE = 'appellate',
}

/**
 * Case status
 */
export enum CaseStatus {
  FILED = 'filed',
  PENDING = 'pending',
  DISCOVERY = 'discovery',
  PRETRIAL = 'pretrial',
  TRIAL = 'trial',
  POST_TRIAL = 'post-trial',
  APPEAL = 'appeal',
  SETTLED = 'settled',
  DISMISSED = 'dismissed',
  DECIDED = 'decided',
  CLOSED = 'closed',
}

/**
 * Court case
 */
export interface CourtCase {
  /** Case identifier */
  caseId: string;

  /** Case number */
  caseNumber: string;

  /** Court */
  court: CourtLevel;

  /** Case type */
  type: CaseType;

  /** Parties */
  parties: {
    plaintiff?: string;
    defendant?: string;
    appellant?: string;
    appellee?: string;
    prosecutor?: string;
    accused?: string;
  };

  /** Case caption */
  caption: string;

  /** Jurisdiction */
  jurisdiction: TemporalJurisdiction;

  /** Charges/claims */
  chargesOrClaims: string[];

  /** Timeline of alleged violation */
  violationTimeline?: Date | string;

  /** Filing date */
  filingDate: Date;

  /** Status */
  status: CaseStatus;

  /** Next hearing */
  nextHearing?: {
    date: Date;
    type: string;
    location: string;
  };

  /** Assigned judge */
  assignedJudge?: string;

  /** Attorneys */
  attorneys?: {
    party: string;
    attorney: string;
    contact?: string;
  }[];

  /** Documents */
  documents?: {
    id: string;
    type: string;
    filedDate: Date;
    filedBy: string;
  }[];

  /** Hearings */
  hearings?: {
    date: Date;
    type: string;
    outcome?: string;
  }[];

  /** Verdict/judgment */
  decision?: {
    date: Date;
    outcome: string;
    details: string;
    judge?: string;
  };

  /** Sentencing (criminal) */
  sentencing?: {
    date: Date;
    sentence: string;
    duration?: number;
    fines?: number;
    conditions?: string[];
  };

  /** Appeal filed */
  appeal?: {
    filed: boolean;
    filingDate?: Date;
    appealCourt?: CourtLevel;
    status?: string;
  };
}

/**
 * Legal proceeding
 */
export interface LegalProceeding {
  /** Proceeding identifier */
  id: string;

  /** Case identifier */
  caseId: string;

  /** Proceeding type */
  type: 'arraignment' | 'hearing' | 'trial' | 'sentencing' | 'appeal' | 'other';

  /** Date and time */
  dateTime: Date;

  /** Location */
  location: string;

  /** Presiding judge */
  judge: string;

  /** Parties present */
  partiesPresent: string[];

  /** Transcript */
  transcript?: {
    available: boolean;
    url?: string;
  };

  /** Outcome */
  outcome?: string;

  /** Next steps */
  nextSteps?: string[];
}

// ============================================================================
// Sentencing and Penalties
// ============================================================================

/**
 * Sentencing types
 */
export enum SentencingType {
  IMPRISONMENT = 'imprisonment',
  TEMPORAL_INCARCERATION = 'temporal-incarceration',
  TIMELINE_EXILE = 'timeline-exile',
  PROBATION = 'probation',
  COMMUNITY_SERVICE = 'community-service',
  FINES = 'fines',
  RESTITUTION = 'restitution',
  TEMPORAL_ERASURE = 'temporal-erasure',
}

/**
 * Sentencing decision
 */
export interface Sentencing {
  /** Sentencing identifier */
  id: string;

  /** Case identifier */
  caseId: string;

  /** Defendant */
  defendant: string;

  /** Conviction */
  conviction: {
    crime: TimeCrimeType;
    class: TimeCrimeClass;
    counts: number;
  }[];

  /** Sentence type */
  sentenceType: SentencingType[];

  /** Imprisonment details */
  imprisonment?: {
    duration: number; // months
    facility: string;
    paroleEligibility?: number; // months
  };

  /** Temporal incarceration */
  temporalIncarceration?: {
    duration: number;
    type: 'isolation' | 'timeline-locked' | 'stasis';
    conditions?: string[];
  };

  /** Timeline exile */
  timelineExile?: {
    excludedTimelines: string[];
    duration: number; // months, -1 = permanent
  };

  /** Probation */
  probation?: {
    duration: number; // months
    conditions: string[];
    supervisionLevel: 'minimal' | 'standard' | 'intensive';
  };

  /** Fines */
  fines?: {
    amount: number;
    currency: string;
    timeline: string;
    paymentSchedule?: string;
  };

  /** Restitution */
  restitution?: {
    amount: number;
    payees: string[];
    timeline: string;
  };

  /** Community service */
  communityService?: {
    hours: number;
    type: string;
    timeline?: string;
  };

  /** Sentencing date */
  sentencingDate: Date;

  /** Judge */
  judge: string;

  /** Aggravating factors */
  aggravatingFactors?: string[];

  /** Mitigating factors */
  mitigatingFactors?: string[];

  /** Appeal filed */
  appeal?: {
    filed: boolean;
    filingDate?: Date;
    status?: string;
  };
}

// ============================================================================
// Appeals
// ============================================================================

/**
 * Appeal grounds
 */
export enum AppealGrounds {
  LEGAL_ERROR = 'legal-error',
  FACTUAL_ERROR = 'factual-error',
  PROCEDURAL_ERROR = 'procedural-error',
  CONSTITUTIONAL_VIOLATION = 'constitutional-violation',
  EXCESSIVE_SENTENCE = 'excessive-sentence',
  JURISDICTION_ERROR = 'jurisdiction-error',
  EVIDENCE_ERROR = 'evidence-error',
  OTHER = 'other',
}

/**
 * Appeal
 */
export interface Appeal {
  /** Appeal identifier */
  id: string;

  /** Original case identifier */
  originalCaseId: string;

  /** Appellant */
  appellant: string;

  /** Appellee */
  appellee: string;

  /** Appeal court */
  court: CourtLevel;

  /** Grounds for appeal */
  grounds: {
    type: AppealGrounds;
    description: string;
  }[];

  /** Relief sought */
  reliefSought: string[];

  /** Filing date */
  filingDate: Date;

  /** Briefs */
  briefs?: {
    party: string;
    type: 'opening' | 'answering' | 'reply';
    filedDate: Date;
  }[];

  /** Oral argument */
  oralArgument?: {
    scheduled: boolean;
    date?: Date;
    location?: string;
  };

  /** Status */
  status: 'filed' | 'briefing' | 'submitted' | 'argued' | 'decided';

  /** Decision */
  decision?: {
    outcome: 'affirmed' | 'reversed' | 'remanded' | 'modified';
    date: Date;
    opinion?: string;
    dissentingOpinions?: string[];
  };

  /** Further appeal available */
  furtherAppeal?: {
    available: boolean;
    deadline?: Date;
    court?: CourtLevel;
  };
}

// ============================================================================
// Law Enforcement
// ============================================================================

/**
 * Law enforcement actions
 */
export enum EnforcementAction {
  INVESTIGATION = 'investigation',
  ARREST = 'arrest',
  DETENTION = 'detention',
  SEARCH = 'search',
  SEIZURE = 'seizure',
  SURVEILLANCE = 'surveillance',
  EXTRADITION = 'extradition',
  ASSET_FREEZE = 'asset-freeze',
}

/**
 * Law enforcement operation
 */
export interface LawEnforcement {
  /** Operation identifier */
  id: string;

  /** Action type */
  action: EnforcementAction;

  /** Target */
  target: string;

  /** Timeline of operation */
  timeline: string | Date;

  /** Jurisdiction */
  jurisdiction: JurisdictionLevel;

  /** Authorizing authority */
  authority: {
    type: 'warrant' | 'court-order' | 'emergency' | 'consent';
    issuedBy?: string;
    date?: Date;
    expirationDate?: Date;
  };

  /** Executing agency */
  agency: string;

  /** Officers involved */
  officers?: string[];

  /** Date of operation */
  operationDate: Date;

  /** Outcome */
  outcome?: {
    successful: boolean;
    detailsI: string;
    evidenceCollected?: string[];
    arrestsMade?: string[];
  };

  /** Legal challenges */
  challenges?: {
    filed: boolean;
    type?: string;
    status?: string;
  }[];
}

// ============================================================================
// Treaties and International Law
// ============================================================================

/**
 * Treaty types
 */
export enum TreatyType {
  BILATERAL = 'bilateral',
  MULTILATERAL = 'multilateral',
  REGIONAL = 'regional',
  UNIVERSAL = 'universal',
}

/**
 * International treaty
 */
export interface InternationalTreaty {
  /** Treaty identifier */
  id: string;

  /** Treaty name */
  name: string;

  /** Type */
  type: TreatyType;

  /** Signatories */
  signatories: string[];

  /** Subject matter */
  subject: string;

  /** Key provisions */
  provisions: string[];

  /** Adoption date */
  adoptionDate: Date;

  /** Entry into force */
  entryIntoForce?: Date;

  /** Status */
  status: 'negotiating' | 'signed' | 'ratified' | 'in-force' | 'terminated';

  /** Enforcement mechanism */
  enforcement?: {
    body: string;
    procedures: string[];
    sanctions?: string[];
  };

  /** Amendments */
  amendments?: {
    date: Date;
    description: string;
    status: string;
  }[];

  /** Reservations */
  reservations?: {
    country: string;
    provision: string;
    text: string;
  }[];
}

// ============================================================================
// Legal Remedies
// ============================================================================

/**
 * Remedy types
 */
export enum RemedyType {
  COMPENSATORY_DAMAGES = 'compensatory-damages',
  PUNITIVE_DAMAGES = 'punitive-damages',
  RESTITUTION = 'restitution',
  SPECIFIC_PERFORMANCE = 'specific-performance',
  INJUNCTION = 'injunction',
  DECLARATORY_JUDGMENT = 'declaratory-judgment',
  REFORMATION = 'reformation',
  RESCISSION = 'rescission',
  TIMELINE_REMEDIATION = 'timeline-remediation',
}

/**
 * Legal remedy
 */
export interface LegalRemedy {
  /** Remedy type */
  type: RemedyType;

  /** Amount (if monetary) */
  amount?: {
    value: number;
    currency: string;
    timeline: string;
  };

  /** Description */
  description: string;

  /** Timeline for compliance */
  complianceDeadline?: Date;

  /** Enforcement mechanism */
  enforcement?: string;

  /** Conditions */
  conditions?: string[];
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Legal constants
 */
export const LEGAL_CONSTANTS = {
  /** Statute of limitations (years) by crime class */
  STATUTE_OF_LIMITATIONS: {
    CLASS_A: -1, // No limit
    CLASS_B: -1, // No limit
    CLASS_C: 20,
    CLASS_D: 10,
    CLASS_E: 5,
  },

  /** Minimum penalties (months imprisonment) */
  MINIMUM_PENALTIES: {
    CLASS_A: -1, // Life
    CLASS_B: 300, // 25 years
    CLASS_C: 60, // 5 years
    CLASS_D: 12, // 1 year
    CLASS_E: 0, // No minimum
  },

  /** Maximum penalties (months imprisonment) */
  MAXIMUM_PENALTIES: {
    CLASS_A: -1, // Life/temporal erasure
    CLASS_B: -1, // Life
    CLASS_C: 300, // 25 years
    CLASS_D: 60, // 5 years
    CLASS_E: 0, // Fines only
  },

  /** Court filing fees (USD baseline) */
  FILING_FEES: {
    LOCAL: 500,
    REGIONAL: 1000,
    NATIONAL: 2500,
    INTERNATIONAL: 5000,
    SUPREME: 10000,
  },

  /** Appeal deadlines (days) */
  APPEAL_DEADLINES: {
    CRIMINAL: 30,
    CIVIL: 30,
    ADMINISTRATIVE: 60,
  },

  /** Discovery period (days) */
  DISCOVERY_PERIOD: 180,

  /** Trial preparation time (days) */
  TRIAL_PREPARATION: 90,
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-TIME-031 error codes
 */
export enum LegalErrorCode {
  JURISDICTION_ERROR = 'L001',
  INVALID_CONTRACT = 'L002',
  PROPERTY_CLAIM_DENIED = 'L003',
  STATUTE_OF_LIMITATIONS = 'L004',
  LACK_OF_STANDING = 'L005',
  PROCEDURAL_ERROR = 'L006',
  EVIDENCE_INADMISSIBLE = 'L007',
  UNAUTHORIZED_ACTION = 'L008',
  TREATY_VIOLATION = 'L009',
  ENFORCEMENT_FAILURE = 'L010',
}

/**
 * Temporal law error
 */
export class TemporalLawError extends Error {
  constructor(
    public code: LegalErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'TemporalLawError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  GeoLocation,
  TemporalJurisdiction,
  JurisdictionConflict,
  JurisdictionCheckRequest,
  TimeCrime,
  CrimeReport,
  TravelerRights,
  RightsInvocation,
  TravelerRegistration,
  LegalStatusCheck,
  PropertyClaim,
  PropertyTransfer,
  TemporalContract,
  ContractValidation,
  ContractDispute,
  CourtCase,
  LegalProceeding,
  Sentencing,
  Appeal,
  LawEnforcement,
  InternationalTreaty,
  LegalRemedy,
};

export {
  TimeCrimeClass,
  TimeCrimeType,
  LegalStatus,
  JurisdictionLevel,
  CourtLevel,
  PropertyType,
  OwnershipType,
  ContractType,
  CaseType,
  CaseStatus,
  SentencingType,
  AppealGrounds,
  EnforcementAction,
  TreatyType,
  RemedyType,
  LEGAL_CONSTANTS,
  LegalErrorCode,
  TemporalLawError,
};
