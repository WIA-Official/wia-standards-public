/**
 * WIA Cryo Identity Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Cryo Identity Types
// ============================================================================

export interface WIACryoIdentityProject {
  standard: 'WIA-CRYO-IDENTITY';
  version: string;
  metadata: ProjectMetadata;
  identityManagement: IdentityManagement;
  subjects: Subject[];
  verification: VerificationSystem;
  privacy: PrivacyFramework;
  continuity: ContinuityPlanning;
  audit: AuditSystem;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  organization: Organization;
  jurisdiction: string[];
  createdAt: string;
  updatedAt?: string;
  status: ProjectStatus;
}

export interface Organization {
  name: string;
  type: 'facility' | 'registry' | 'authority' | 'provider';
  country: string;
  registrationNumber?: string;
  contact: ContactInfo;
}

export interface ContactInfo {
  name: string;
  email: string;
  phone?: string;
}

export type ProjectStatus = 'active' | 'pending' | 'suspended' | 'archived';

// ============================================================================
// Identity Management
// ============================================================================

export interface IdentityManagement {
  schema: IdentitySchema;
  lifecycle: IdentityLifecycle;
  linking: LinkingRules;
  resolution: ResolutionPolicy;
}

export interface IdentitySchema {
  version: string;
  attributes: IdentityAttribute[];
  required: string[];
  unique: string[];
  immutable: string[];
}

export interface IdentityAttribute {
  id: string;
  name: string;
  type: AttributeType;
  description: string;
  format?: string;
  validation?: string;
  sensitivity: SensitivityLevel;
}

export type AttributeType = 'string' | 'date' | 'number' | 'boolean' | 'biometric' | 'document';
export type SensitivityLevel = 'public' | 'internal' | 'confidential' | 'restricted';

export interface IdentityLifecycle {
  states: IdentityState[];
  transitions: StateTransition[];
  retention: RetentionPolicy;
}

export interface IdentityState {
  name: string;
  description: string;
  allowedOperations: string[];
}

export interface StateTransition {
  from: string;
  to: string;
  trigger: string;
  conditions?: string[];
  authorization: string;
}

export interface RetentionPolicy {
  active: string;
  suspended: string;
  deceased: string;
  archival: string;
}

export interface LinkingRules {
  specimenToIdentity: LinkingRule;
  crossFacility: LinkingRule;
  familial: LinkingRule;
}

export interface LinkingRule {
  enabled: boolean;
  method: string;
  verification: string;
  reversible: boolean;
}

export interface ResolutionPolicy {
  priority: string[];
  conflicts: string;
  fallback: string;
}

// ============================================================================
// Subject
// ============================================================================

export interface Subject {
  id: string;
  type: SubjectType;
  identifiers: SubjectIdentifier[];
  profile: SubjectProfile;
  biometrics?: BiometricData[];
  specimens: SpecimenLink[];
  relationships?: Relationship[];
  directives?: IdentityDirective[];
  status: SubjectStatus;
  history: SubjectEvent[];
}

export type SubjectType = 'individual' | 'minor' | 'incapacitated' | 'posthumous';

export interface SubjectIdentifier {
  type: IdentifierType;
  value: string;
  issuer?: string;
  validFrom?: string;
  validTo?: string;
  verified: boolean;
  primary: boolean;
}

export type IdentifierType = 'internal' | 'national-id' | 'passport' | 'medical-record' | 'donor-id' | 'anonymous';

export interface SubjectProfile {
  legalName?: LegalName;
  dateOfBirth?: string;
  placeOfBirth?: string;
  nationality?: string[];
  gender?: string;
  contactInfo?: ContactDetails;
  nextOfKin?: NextOfKin[];
}

export interface LegalName {
  given: string;
  middle?: string;
  family: string;
  suffix?: string;
  previous?: string[];
}

export interface ContactDetails {
  email?: string;
  phone?: string;
  address?: Address;
  preferred: string;
}

export interface Address {
  street: string;
  city: string;
  state?: string;
  postalCode: string;
  country: string;
}

export interface NextOfKin {
  name: string;
  relationship: string;
  contact: ContactDetails;
  priority: number;
  authorized: boolean;
}

export interface BiometricData {
  type: BiometricType;
  template: string;
  capturedAt: string;
  capturedBy: string;
  quality: number;
  equipment?: string;
}

export type BiometricType = 'fingerprint' | 'facial' | 'iris' | 'dna' | 'voice';

export interface SpecimenLink {
  specimenId: string;
  type: string;
  facility: string;
  linkedAt: string;
  linkedBy: string;
  status: 'active' | 'inactive' | 'disposed';
}

export interface Relationship {
  relatedSubjectId: string;
  type: RelationshipType;
  verified: boolean;
  verifiedAt?: string;
  permissions?: string[];
}

export type RelationshipType = 'parent' | 'child' | 'sibling' | 'spouse' | 'donor-recipient' | 'genetic';

export interface IdentityDirective {
  type: DirectiveType;
  content: string;
  effectiveFrom: string;
  effectiveTo?: string;
  witnesses?: string[];
  legalBasis?: string;
}

export type DirectiveType = 'name-change' | 'anonymization' | 'access-restriction' | 'posthumous' | 'research-consent';

export type SubjectStatus = 'active' | 'suspended' | 'deceased' | 'anonymized' | 'merged';

export interface SubjectEvent {
  id: string;
  type: EventType;
  timestamp: string;
  actor: string;
  description: string;
  changes?: Record<string, { before: unknown; after: unknown }>;
}

export type EventType = 'created' | 'updated' | 'verified' | 'suspended' | 'merged' | 'anonymized' | 'deceased';

// ============================================================================
// Verification System
// ============================================================================

export interface VerificationSystem {
  methods: VerificationMethod[];
  workflows: VerificationWorkflow[];
  thresholds: VerificationThreshold[];
  reVerification: ReVerificationPolicy;
}

export interface VerificationMethod {
  id: string;
  name: string;
  type: VerificationType;
  strength: 'low' | 'medium' | 'high';
  requirements: string[];
  automated: boolean;
}

export type VerificationType = 'document' | 'biometric' | 'knowledge' | 'possession' | 'third-party';

export interface VerificationWorkflow {
  name: string;
  purpose: string;
  steps: VerificationStep[];
  timeout: number;
  fallback?: string;
}

export interface VerificationStep {
  order: number;
  method: string;
  required: boolean;
  retries: number;
}

export interface VerificationThreshold {
  operation: string;
  minimumStrength: string;
  methods: string[];
  multiFactorRequired: boolean;
}

export interface ReVerificationPolicy {
  triggers: string[];
  frequency: string;
  gracePeriod: string;
}

// ============================================================================
// Privacy Framework
// ============================================================================

export interface PrivacyFramework {
  principles: PrivacyPrinciple[];
  dataMinimization: DataMinimization;
  anonymization: AnonymizationRules;
  accessControl: AccessControl;
  breachProtocol: BreachProtocol;
}

export interface PrivacyPrinciple {
  name: string;
  description: string;
  implementation: string;
}

export interface DataMinimization {
  collection: string;
  retention: string;
  disclosure: string;
}

export interface AnonymizationRules {
  methods: AnonymizationMethod[];
  triggers: string[];
  verification: string;
  reversibility: boolean;
}

export interface AnonymizationMethod {
  type: 'pseudonymization' | 'k-anonymity' | 'differential-privacy' | 'redaction';
  parameters: Record<string, unknown>;
  scope: string[];
}

export interface AccessControl {
  model: 'rbac' | 'abac' | 'hybrid';
  roles: AccessRole[];
  policies: AccessPolicy[];
  audit: boolean;
}

export interface AccessRole {
  name: string;
  permissions: string[];
  restrictions?: string[];
}

export interface AccessPolicy {
  resource: string;
  actions: string[];
  conditions?: string[];
  effect: 'allow' | 'deny';
}

export interface BreachProtocol {
  detection: string[];
  notification: NotificationPolicy;
  remediation: string[];
  reporting: string;
}

export interface NotificationPolicy {
  authorities: { name: string; timeframe: string }[];
  subjects: { method: string; timeframe: string };
  internal: { recipients: string[]; timeframe: string };
}

// ============================================================================
// Continuity Planning
// ============================================================================

export interface ContinuityPlanning {
  succession: SuccessionPlan;
  disasterRecovery: DisasterRecovery;
  portability: DataPortability;
  termination: TerminationProcedures;
}

export interface SuccessionPlan {
  designatedSuccessor?: string;
  transferProtocol: string;
  dataHandling: string;
  notificationRequired: boolean;
}

export interface DisasterRecovery {
  rto: number;
  rpo: number;
  backupSite?: string;
  testingFrequency: string;
  lastTest?: string;
}

export interface DataPortability {
  formats: string[];
  standards: string[];
  exportProcess: string;
  timeline: string;
}

export interface TerminationProcedures {
  notice: string;
  dataTransfer: string;
  destruction: string;
  certification: boolean;
}

// ============================================================================
// Audit System
// ============================================================================

export interface AuditSystem {
  logging: AuditLogging;
  reporting: AuditReporting;
  retention: string;
  integrity: IntegrityProtection;
}

export interface AuditLogging {
  events: string[];
  details: 'minimal' | 'standard' | 'detailed';
  realtime: boolean;
}

export interface AuditReporting {
  automated: { frequency: string; recipients: string[] }[];
  onDemand: string[];
  compliance: string[];
}

export interface IntegrityProtection {
  method: 'hash' | 'blockchain' | 'signature';
  verification: string;
  alerting: boolean;
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig { baseURL: string; apiKey?: string; timeout?: number; }
export interface ProjectResponse { id: string; name: string; status: ProjectStatus; createdAt: string; updatedAt?: string; }
export interface ValidationResult { valid: boolean; errors?: ValidationError[]; }
export interface ValidationError { path: string; message: string; value?: unknown; }
export interface PaginatedResponse<T> { data: T[]; pagination: { total: number; limit: number; offset: number; hasMore: boolean }; }
