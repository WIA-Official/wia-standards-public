/**
 * WIA Cryo Consent Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Cryo Consent Types
// ============================================================================

export interface WIACryoConsentProject {
  standard: 'WIA-CRYO-CONSENT';
  version: string;
  metadata: ProjectMetadata;
  consentFramework: ConsentFramework;
  subjects: ConsentSubject[];
  documents: ConsentDocument[];
  verification: VerificationSystem;
  compliance: ComplianceFramework;
  audit: AuditConfiguration;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  organization: Organization;
  jurisdiction: Jurisdiction;
  createdAt: string;
  updatedAt?: string;
  status: ProjectStatus;
}

export interface Organization {
  name: string;
  type: 'facility' | 'hospital' | 'research' | 'commercial';
  country: string;
  registrationNumber?: string;
  contact?: ContactInfo;
}

export interface ContactInfo {
  name: string;
  email: string;
  phone?: string;
}

export interface Jurisdiction {
  country: string;
  state?: string;
  applicableLaws: string[];
  regulatoryBody?: string;
}

export type ProjectStatus = 'active' | 'pending' | 'suspended' | 'archived';

// ============================================================================
// Consent Framework
// ============================================================================

export interface ConsentFramework {
  types: ConsentType[];
  requirements: ConsentRequirement[];
  process: ConsentProcess;
  retention: RetentionPolicy;
  withdrawal: WithdrawalPolicy;
}

export interface ConsentType {
  id: string;
  name: string;
  category: ConsentCategory;
  description: string;
  scope: ConsentScope[];
  mandatory: boolean;
  duration: ConsentDuration;
  renewal?: RenewalPolicy;
}

export type ConsentCategory =
  | 'preservation'
  | 'storage'
  | 'research'
  | 'donation'
  | 'disposal'
  | 'commercial-use'
  | 'genetic-testing'
  | 'transplantation';

export type ConsentScope =
  | 'personal-use'
  | 'family-use'
  | 'research-specific'
  | 'research-broad'
  | 'commercial'
  | 'educational'
  | 'therapeutic';

export interface ConsentDuration {
  type: 'indefinite' | 'fixed' | 'until-event';
  period?: string;
  event?: string;
  reviewInterval?: string;
}

export interface RenewalPolicy {
  required: boolean;
  interval: string;
  method: 'active' | 'passive' | 'notification';
  noticeperiod: string;
}

export interface ConsentRequirement {
  id: string;
  type: RequirementType;
  description: string;
  mandatory: boolean;
  verification: VerificationMethod;
  documentation: string[];
}

export type RequirementType =
  | 'age'
  | 'capacity'
  | 'information'
  | 'voluntary'
  | 'specific'
  | 'witness'
  | 'cooling-off';

export type VerificationMethod =
  | 'document'
  | 'signature'
  | 'witness'
  | 'video'
  | 'biometric'
  | 'digital-signature';

export interface ConsentProcess {
  steps: ProcessStep[];
  timeline: string;
  counseling?: CounselingRequirement;
  languages: string[];
  accessibility: AccessibilityOptions;
}

export interface ProcessStep {
  order: number;
  name: string;
  description: string;
  responsible: string;
  documentation: string[];
  timeframe?: string;
}

export interface CounselingRequirement {
  required: boolean;
  type: 'individual' | 'group' | 'both';
  provider: string;
  duration: string;
  topics: string[];
}

export interface AccessibilityOptions {
  formats: ('print' | 'large-print' | 'braille' | 'audio' | 'video' | 'digital')[];
  languages: string[];
  interpreter: boolean;
  cognitiveSupport: boolean;
}

export interface RetentionPolicy {
  consentForms: string;
  supportingDocuments: string;
  auditTrails: string;
  format: 'physical' | 'digital' | 'both';
  storage: string;
}

export interface WithdrawalPolicy {
  allowed: boolean;
  restrictions?: string[];
  process: WithdrawalProcess;
  consequences: string[];
  partialWithdrawal: boolean;
}

export interface WithdrawalProcess {
  steps: string[];
  timeframe: string;
  confirmation: string;
  documentation: string[];
}

// ============================================================================
// Consent Subject
// ============================================================================

export interface ConsentSubject {
  id: string;
  type: SubjectType;
  identifier: SubjectIdentifier;
  demographics: Demographics;
  capacity: CapacityAssessment;
  consents: ConsentRecord[];
  representatives?: LegalRepresentative[];
  status: SubjectStatus;
}

export type SubjectType = 'individual' | 'minor' | 'incapacitated' | 'deceased';

export interface SubjectIdentifier {
  internalId: string;
  externalId?: string;
  anonymized?: boolean;
}

export interface Demographics {
  dateOfBirth?: string;
  age?: number;
  gender?: string;
  nationality?: string;
  language: string;
}

export interface CapacityAssessment {
  hasCapacity: boolean;
  assessmentDate: string;
  assessor: string;
  method: string;
  documentation?: string;
  reviewDate?: string;
}

export interface ConsentRecord {
  id: string;
  consentTypeId: string;
  status: ConsentStatus;
  givenAt: string;
  givenBy: string;
  method: ConsentMethod;
  scope: ConsentScope[];
  restrictions?: string[];
  validFrom: string;
  validTo?: string;
  documents: string[];
  witnesses?: Witness[];
  history: ConsentEvent[];
}

export type ConsentStatus =
  | 'pending'
  | 'active'
  | 'withdrawn'
  | 'expired'
  | 'superseded'
  | 'void';

export type ConsentMethod =
  | 'written'
  | 'electronic'
  | 'verbal-recorded'
  | 'video'
  | 'biometric';

export interface Witness {
  name: string;
  role: string;
  signature?: string;
  date: string;
  relationship?: string;
}

export interface ConsentEvent {
  id: string;
  type: ConsentEventType;
  timestamp: string;
  actor: string;
  description: string;
  documentation?: string;
}

export type ConsentEventType =
  | 'created'
  | 'signed'
  | 'witnessed'
  | 'verified'
  | 'modified'
  | 'renewed'
  | 'withdrawn'
  | 'expired'
  | 'voided';

export interface LegalRepresentative {
  id: string;
  name: string;
  relationship: string;
  authority: AuthorityType;
  authorityDocument?: string;
  contact: ContactInfo;
  validFrom: string;
  validTo?: string;
}

export type AuthorityType =
  | 'parent'
  | 'guardian'
  | 'power-of-attorney'
  | 'court-appointed'
  | 'next-of-kin';

export type SubjectStatus = 'active' | 'inactive' | 'deceased' | 'withdrawn';

// ============================================================================
// Consent Documents
// ============================================================================

export interface ConsentDocument {
  id: string;
  type: DocumentType;
  version: string;
  title: string;
  language: string;
  content: DocumentContent;
  approval: DocumentApproval;
  effectiveDate: string;
  expiryDate?: string;
  status: DocumentStatus;
}

export type DocumentType =
  | 'consent-form'
  | 'information-sheet'
  | 'summary'
  | 'authorization'
  | 'directive'
  | 'amendment';

export interface DocumentContent {
  format: 'pdf' | 'html' | 'markdown' | 'docx';
  url?: string;
  checksum?: string;
  sections: DocumentSection[];
}

export interface DocumentSection {
  id: string;
  title: string;
  content: string;
  mandatory: boolean;
  signature?: boolean;
  initial?: boolean;
}

export interface DocumentApproval {
  approvedBy: string;
  approvedAt: string;
  ethicsReview?: EthicsReview;
  legalReview?: LegalReview;
}

export interface EthicsReview {
  committee: string;
  referenceNumber: string;
  approvalDate: string;
  expiryDate?: string;
}

export interface LegalReview {
  reviewer: string;
  reviewDate: string;
  notes?: string;
}

export type DocumentStatus = 'draft' | 'approved' | 'active' | 'superseded' | 'archived';

// ============================================================================
// Verification & Compliance
// ============================================================================

export interface VerificationSystem {
  methods: VerificationConfig[];
  identityVerification: IdentityVerification;
  signatureVerification: SignatureVerification;
  auditTrail: AuditTrailConfig;
}

export interface VerificationConfig {
  type: string;
  required: boolean;
  method: string;
  frequency: string;
}

export interface IdentityVerification {
  required: boolean;
  methods: IdentityMethod[];
  documentation: string[];
}

export type IdentityMethod =
  | 'government-id'
  | 'biometric'
  | 'two-factor'
  | 'in-person'
  | 'video-call';

export interface SignatureVerification {
  electronic: ElectronicSignature;
  physical: PhysicalSignature;
  witnessing: WitnessingRequirements;
}

export interface ElectronicSignature {
  allowed: boolean;
  standards: string[];
  provider?: string;
  certification?: string;
}

export interface PhysicalSignature {
  required: boolean;
  retention: string;
  scanning: boolean;
}

export interface WitnessingRequirements {
  required: boolean;
  count: number;
  qualifications: string[];
  independentRequired: boolean;
}

export interface AuditTrailConfig {
  enabled: boolean;
  events: string[];
  retention: string;
  immutable: boolean;
  blockchain?: boolean;
}

export interface ComplianceFramework {
  regulations: Regulation[];
  policies: Policy[];
  audits: AuditSchedule;
  breachProtocol: BreachProtocol;
}

export interface Regulation {
  name: string;
  jurisdiction: string;
  requirements: string[];
  lastReview: string;
  nextReview: string;
}

export interface Policy {
  id: string;
  name: string;
  version: string;
  effectiveDate: string;
  owner: string;
  reviewCycle: string;
}

export interface AuditSchedule {
  internal: { frequency: string; scope: string[] };
  external: { frequency: string; auditor: string };
  random: { enabled: boolean; frequency: string };
}

export interface BreachProtocol {
  definition: string[];
  reportingTimeframe: string;
  notificationRequired: boolean;
  remediation: string[];
}

export interface AuditConfiguration {
  enabled: boolean;
  scope: string[];
  retention: string;
  reporting: AuditReporting;
}

export interface AuditReporting {
  automated: boolean;
  frequency: string;
  recipients: string[];
  format: string;
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface ProjectResponse {
  id: string;
  name: string;
  status: ProjectStatus;
  createdAt: string;
  updatedAt?: string;
}

export interface ValidationResult {
  valid: boolean;
  errors?: ValidationError[];
}

export interface ValidationError {
  path: string;
  message: string;
  value?: unknown;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    total: number;
    limit: number;
    offset: number;
    hasMore: boolean;
  };
}
