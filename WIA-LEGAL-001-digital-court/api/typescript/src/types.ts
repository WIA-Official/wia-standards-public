/**
 * WIA-LEGAL-001: Digital Court Standard - TypeScript Type Definitions
 * Version: 1.0.0
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

// ============================================================================
// Core Types
// ============================================================================

export type CaseType = 'civil' | 'criminal' | 'family' | 'administrative' | 'appellate';
export type CaseStatus = 'filed' | 'active' | 'pending' | 'closed' | 'archived' | 'appealed';
export type CourtLevel = 'district' | 'appellate' | 'supreme' | 'specialized';
export type PartyType = 'individual' | 'corporation' | 'government' | 'ngo';
export type PartyRole = 'plaintiff' | 'defendant' | 'third-party' | 'intervenor';

// ============================================================================
// Court & Jurisdiction
// ============================================================================

export interface Court {
  courtId: string;
  courtName: string;
  jurisdiction: string;
  level: CourtLevel;
  location?: Address;
  contact?: ContactInfo;
}

export interface Address {
  street?: string;
  city: string;
  state?: string;
  postalCode?: string;
  country: string;
}

export interface ContactInfo {
  email?: string;
  phone?: string;
  fax?: string;
  website?: string;
}

// ============================================================================
// Case Data
// ============================================================================

export interface Case {
  standard: 'WIA-LEGAL-001';
  version: string;
  caseId: string;
  caseNumber: string;
  caseType: CaseType;
  court: Court;
  parties: Parties;
  filingDate: string; // ISO 8601
  status: CaseStatus;
  description?: string;
  metadata?: CaseMetadata;
  timestamp: string; // ISO 8601
}

export interface Parties {
  plaintiffs: Party[];
  defendants: Party[];
  attorneys?: Attorney[];
  judges?: Judge[];
}

export interface Party {
  partyId: string;
  type: PartyType;
  name: string;
  legalName?: string;
  did?: string; // Decentralized Identifier
  contact?: ContactInfo;
  representative?: Attorney;
  role: PartyRole;
}

export interface Attorney {
  attorneyId: string;
  name: string;
  did?: string;
  licenseNumber: string;
  barAssociation: string;
  firm?: string;
  contact: ContactInfo;
  representingParty: string; // partyId
}

export interface Judge {
  judgeId: string;
  name: string;
  did?: string;
  title: string;
  court: string;
  appointed?: string; // ISO 8601
}

export interface CaseMetadata {
  tags?: string[];
  estimatedDuration?: string;
  estimatedValue?: MonetaryAmount;
  complexity?: 'low' | 'medium' | 'high';
  priority?: 'normal' | 'high' | 'urgent';
  confidential?: boolean;
  relatedCases?: string[]; // caseIds
}

export interface MonetaryAmount {
  amount: number;
  currency: string; // ISO 4217
}

// ============================================================================
// Documents
// ============================================================================

export type DocumentType = 'pleading' | 'motion' | 'evidence' | 'order' | 'transcript' | 'brief' | 'exhibit';
export type ConfidentialityLevel = 'public' | 'sealed' | 'confidential' | 'restricted';

export interface Document {
  documentId: string;
  caseId: string;
  type: DocumentType;
  title: string;
  filedBy: string; // partyId
  filedDate: string; // ISO 8601
  confidentiality: ConfidentialityLevel;
  format: string; // MIME type
  hash: string; // SHA-256
  signature: DigitalSignature;
  metadata: DocumentMetadata;
  storageLocation: string; // URI
  timestamp: string; // ISO 8601
}

export interface DocumentMetadata {
  pageCount?: number;
  wordCount?: number;
  language: string; // ISO 639-1
  tags?: string[];
  version?: number;
}

export interface DigitalSignature {
  algorithm: string; // e.g., "Ed25519", "RSA-2048"
  publicKey: string;
  signature: string;
  signedBy: string; // DID or identifier
  signedDate: string; // ISO 8601
}

// ============================================================================
// Evidence
// ============================================================================

export type EvidenceType = 'document' | 'photo' | 'video' | 'audio' | 'digital' | 'physical';
export type AdmissibilityStatus = 'pending' | 'admitted' | 'excluded' | 'conditional';

export interface Evidence {
  evidenceId: string;
  caseId: string;
  type: EvidenceType;
  description: string;
  submittedBy: string; // partyId
  submittedDate: string; // ISO 8601
  chainOfCustody: CustodyRecord[];
  authenticity: AuthenticityVerification;
  forensics?: ForensicAnalysis;
  admissibility: Admissibility;
}

export interface CustodyRecord {
  custodian: string; // DID or identifier
  action: string;
  location: string;
  timestamp: string; // ISO 8601
  blockchainTx?: string;
  signature: string;
}

export interface AuthenticityVerification {
  verified: boolean;
  method: string;
  verifiedBy: string;
  verifiedDate: string; // ISO 8601
  confidence?: number; // 0-1
}

export interface ForensicAnalysis {
  hash: string;
  metadata: Record<string, any>;
  analysisReport?: string; // URI
  analyst?: string;
  analysisDate?: string; // ISO 8601
}

export interface Admissibility {
  status: AdmissibilityStatus;
  rulingDate?: string; // ISO 8601
  rulingJudge?: string;
  reason?: string;
}

// ============================================================================
// Hearings
// ============================================================================

export type HearingType = 'preliminary' | 'trial' | 'motion' | 'sentencing' | 'appeal' | 'settlement';
export type HearingMode = 'in-person' | 'virtual' | 'hybrid';

export interface Hearing {
  hearingId: string;
  caseId: string;
  type: HearingType;
  scheduledDate: string; // ISO 8601
  duration: number; // minutes
  mode: HearingMode;
  participants: HearingParticipants;
  recording?: RecordingInfo;
  proceedings?: Proceedings;
  outcome?: HearingOutcome;
}

export interface HearingParticipants {
  judge: Judge;
  parties: Party[];
  attorneys: Attorney[];
  witnesses?: Witness[];
  observers?: Observer[];
}

export interface Witness {
  witnessId: string;
  name: string;
  type: 'expert' | 'fact' | 'character';
  qualifications?: string;
}

export interface Observer {
  observerId: string;
  name?: string;
  type: 'public' | 'press' | 'family';
}

export interface RecordingInfo {
  enabled: boolean;
  videoUrl?: string; // URI
  audioUrl?: string; // URI
  transcriptUrl?: string; // URI
  encryption: string;
}

export interface Proceedings {
  startTime: string; // ISO 8601
  endTime?: string; // ISO 8601
  events: HearingEvent[];
}

export interface HearingEvent {
  eventType: string;
  timestamp: string; // ISO 8601
  description: string;
  speaker?: string;
}

export interface HearingOutcome {
  ruling?: string;
  orders?: CourtOrder[];
  nextHearing?: string; // ISO 8601
}

// ============================================================================
// Court Orders
// ============================================================================

export type OrderType = 'procedural' | 'substantive' | 'temporary' | 'final' | 'injunction';
export type EnforcementStatus = 'pending' | 'active' | 'completed' | 'appealed' | 'vacated';

export interface CourtOrder {
  orderId: string;
  caseId: string;
  type: OrderType;
  title: string;
  issuedBy: string; // judgeId
  issuedDate: string; // ISO 8601
  effectiveDate: string; // ISO 8601
  expirationDate?: string; // ISO 8601
  content: string;
  conditions?: string[];
  signature: DigitalSignature;
  enforcement: EnforcementInfo;
}

export interface EnforcementInfo {
  status: EnforcementStatus;
  enforcedBy?: string;
  complianceDeadline?: string; // ISO 8601
  violations?: Violation[];
}

export interface Violation {
  violationId: string;
  date: string; // ISO 8601
  description: string;
  reportedBy: string;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

export interface CreateCaseRequest {
  caseNumber: string;
  caseType: CaseType;
  court: Partial<Court>;
  parties: {
    plaintiffs: Omit<Party, 'partyId'>[];
    defendants: Omit<Party, 'partyId'>[];
  };
  description?: string;
}

export interface CreateCaseResponse {
  success: boolean;
  data: {
    caseId: string;
    caseNumber: string;
    status: CaseStatus;
    filingDate: string;
    nextSteps?: string[];
  };
  timestamp: string;
}

export interface FileDocumentRequest {
  caseId: string;
  type: DocumentType;
  title: string;
  filedBy: string;
  confidentiality: ConfidentialityLevel;
  file: File | Buffer;
}

export interface FileDocumentResponse {
  success: boolean;
  data: {
    documentId: string;
    caseId: string;
    title: string;
    hash: string;
    filedDate: string;
    status: string;
    serveRequired?: boolean;
    serveDeadline?: string;
  };
  timestamp: string;
}

export interface ScheduleHearingRequest {
  caseId: string;
  type: HearingType;
  scheduledDate: string;
  duration: number;
  mode: HearingMode;
  participants?: Partial<HearingParticipants>;
  recording?: boolean;
}

export interface ScheduleHearingResponse {
  success: boolean;
  data: {
    hearingId: string;
    caseId: string;
    scheduledDate: string;
    virtualRoomUrl?: string;
    accessCode?: string;
    calendar?: {
      icsUrl: string;
      googleCalendar?: string;
      outlookCalendar?: string;
    };
    notifications?: {
      emailSent: boolean;
      smsSent: boolean;
      reminder24h: boolean;
      reminder1h: boolean;
    };
  };
  timestamp: string;
}

export interface ApiError {
  success: false;
  error: {
    code: string;
    message: string;
    details?: Record<string, any>;
    timestamp: string;
    requestId: string;
  };
}

// ============================================================================
// Client Configuration
// ============================================================================

export interface DigitalCourtClientConfig {
  apiKey: string;
  environment: 'production' | 'staging' | 'development';
  baseUrl?: string;
  timeout?: number;
  retries?: number;
}

// ============================================================================
// Webhook Events
// ============================================================================

export type WebhookEventType =
  | 'case.created'
  | 'case.updated'
  | 'case.closed'
  | 'document.filed'
  | 'document.served'
  | 'hearing.scheduled'
  | 'hearing.completed'
  | 'evidence.submitted'
  | 'order.issued';

export interface WebhookPayload {
  event: WebhookEventType;
  eventId: string;
  timestamp: string;
  data: Record<string, any>;
}

// ============================================================================
// Export all types
// ============================================================================

export default {
  // Version info
  STANDARD: 'WIA-LEGAL-001',
  VERSION: '1.0.0',
  PHILOSOPHY: '弘益人間 (Hongik Ingan) - Benefit All Humanity'
};
