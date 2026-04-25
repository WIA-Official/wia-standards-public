/**
 * WIA Cryo Legal Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Cryo Legal Types
// ============================================================================

export interface WIACryoLegalProject {
  standard: 'WIA-CRYO-LEGAL';
  version: string;
  metadata: ProjectMetadata;
  legalFramework: LegalFramework;
  contracts: ContractManagement;
  disputes: DisputeResolution;
  compliance: ComplianceManagement;
  reporting: LegalReporting;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  organization: Organization;
  jurisdictions: Jurisdiction[];
  createdAt: string;
  updatedAt?: string;
  status: ProjectStatus;
}

export interface Organization {
  name: string;
  type: 'facility' | 'law-firm' | 'authority' | 'association';
  country: string;
  registrationNumber?: string;
  contact: ContactInfo;
  legalCounsel?: LegalCounsel;
}

export interface ContactInfo { name: string; email: string; phone?: string; }
export interface LegalCounsel { name: string; firm?: string; bar: string; contact: ContactInfo; }
export interface Jurisdiction { country: string; state?: string; type: 'primary' | 'secondary'; applicableLaws: string[]; }
export type ProjectStatus = 'active' | 'pending' | 'suspended' | 'archived';

// ============================================================================
// Legal Framework
// ============================================================================

export interface LegalFramework {
  regulations: Regulation[];
  rights: LegalRight[];
  obligations: LegalObligation[];
  liabilities: Liability[];
  precedents: LegalPrecedent[];
}

export interface Regulation {
  id: string;
  name: string;
  jurisdiction: string;
  category: RegulationCategory;
  requirements: string[];
  penalties: string[];
  effectiveDate: string;
  lastReview: string;
  status: 'current' | 'amended' | 'superseded';
}

export type RegulationCategory = 'tissue-banking' | 'reproductive' | 'research' | 'transplantation' | 'privacy' | 'consumer-protection';

export interface LegalRight {
  id: string;
  name: string;
  holder: 'donor' | 'patient' | 'facility' | 'beneficiary';
  description: string;
  basis: string[];
  limitations?: string[];
  enforcement: string;
}

export interface LegalObligation {
  id: string;
  name: string;
  obligor: string;
  description: string;
  basis: string;
  deadline?: string;
  penalty?: string;
}

export interface Liability {
  id: string;
  type: LiabilityType;
  description: string;
  coverage: string;
  limit?: number;
  insurance?: string;
  mitigation: string[];
}

export type LiabilityType = 'strict' | 'negligence' | 'contractual' | 'statutory' | 'product';

export interface LegalPrecedent {
  id: string;
  case: string;
  court: string;
  date: string;
  jurisdiction: string;
  ruling: string;
  relevance: string;
  citation: string;
}

// ============================================================================
// Contract Management
// ============================================================================

export interface ContractManagement {
  templates: ContractTemplate[];
  contracts: Contract[];
  terms: StandardTerms;
  amendments: AmendmentProcess;
  termination: TerminationProcedures;
}

export interface ContractTemplate {
  id: string;
  name: string;
  type: ContractType;
  version: string;
  jurisdiction: string;
  clauses: Clause[];
  approvedBy: string;
  approvedAt: string;
  status: 'draft' | 'approved' | 'active' | 'deprecated';
}

export type ContractType = 'storage-agreement' | 'consent-form' | 'service-agreement' | 'transfer-agreement' | 'research-agreement' | 'donation-agreement';

export interface Clause {
  id: string;
  title: string;
  text: string;
  type: 'standard' | 'optional' | 'negotiable';
  mandatory: boolean;
  category: string;
}

export interface Contract {
  id: string;
  templateId?: string;
  type: ContractType;
  parties: Party[];
  subject?: ContractSubject;
  terms: ContractTerms;
  signatures: Signature[];
  effectiveDate: string;
  expiryDate?: string;
  status: ContractStatus;
  history: ContractEvent[];
}

export interface Party {
  id: string;
  type: 'individual' | 'organization';
  name: string;
  role: 'primary' | 'counterparty' | 'guarantor' | 'witness';
  representative?: string;
  contact: ContactInfo;
}

export interface ContractSubject {
  type: 'specimen' | 'service' | 'facility' | 'research';
  identifier: string;
  description: string;
}

export interface ContractTerms {
  duration: string;
  renewal: 'automatic' | 'manual' | 'none';
  fees?: Fee[];
  obligations: { party: string; obligation: string }[];
  warranties: string[];
  limitations: string[];
  governing: string;
  venue: string;
}

export interface Fee {
  type: string;
  amount: number;
  currency: string;
  frequency: 'one-time' | 'monthly' | 'annual';
  due: string;
}

export interface Signature {
  party: string;
  signatory: string;
  date: string;
  method: 'wet' | 'electronic' | 'digital';
  witness?: string;
  notarized?: boolean;
}

export type ContractStatus = 'draft' | 'pending' | 'active' | 'expired' | 'terminated' | 'disputed';

export interface ContractEvent {
  id: string;
  type: string;
  date: string;
  actor: string;
  description: string;
  documents?: string[];
}

export interface StandardTerms {
  storage: string[];
  liability: string[];
  termination: string[];
  dispute: string[];
  privacy: string[];
}

export interface AmendmentProcess {
  allowed: boolean;
  procedure: string;
  approval: string[];
  documentation: string[];
}

export interface TerminationProcedures {
  notice: string;
  grounds: string[];
  procedure: string[];
  consequences: string[];
}

// ============================================================================
// Dispute Resolution
// ============================================================================

export interface DisputeResolution {
  mechanisms: ResolutionMechanism[];
  disputes: Dispute[];
  escalation: EscalationPath;
  documentation: DisputeDocumentation;
}

export interface ResolutionMechanism {
  type: ResolutionType;
  order: number;
  mandatory: boolean;
  provider?: string;
  rules?: string;
  timeframe: string;
  costs: string;
}

export type ResolutionType = 'negotiation' | 'mediation' | 'arbitration' | 'litigation';

export interface Dispute {
  id: string;
  type: DisputeType;
  parties: string[];
  subject: string;
  description: string;
  filedDate: string;
  status: DisputeStatus;
  currentMechanism?: string;
  resolution?: Resolution;
  history: DisputeEvent[];
}

export type DisputeType = 'contract' | 'consent' | 'ownership' | 'negligence' | 'regulatory';
export type DisputeStatus = 'filed' | 'under-review' | 'negotiation' | 'mediation' | 'arbitration' | 'litigation' | 'resolved' | 'dismissed';

export interface Resolution {
  date: string;
  mechanism: string;
  outcome: string;
  terms: string[];
  binding: boolean;
  enforcement?: string;
}

export interface DisputeEvent {
  date: string;
  type: string;
  description: string;
  actor: string;
  documents?: string[];
}

export interface EscalationPath {
  levels: { level: number; mechanism: string; trigger: string; timeframe: string }[];
  finalResort: string;
}

export interface DisputeDocumentation {
  required: string[];
  retention: string;
  confidentiality: string;
}

// ============================================================================
// Compliance Management
// ============================================================================

export interface ComplianceManagement {
  requirements: ComplianceRequirement[];
  audits: AuditProgram;
  violations: ViolationManagement;
  training: ComplianceTraining;
}

export interface ComplianceRequirement {
  id: string;
  regulation: string;
  requirement: string;
  responsible: string;
  evidence: string[];
  frequency: string;
  lastCheck: string;
  status: 'compliant' | 'non-compliant' | 'partial' | 'pending';
}

export interface AuditProgram {
  internal: { frequency: string; scope: string[]; team: string };
  external: { frequency: string; auditor: string; scope: string[] };
  regulatory: { agencies: string[]; schedule: string };
}

export interface ViolationManagement {
  reporting: string;
  investigation: string;
  remediation: string;
  disclosure: string;
  tracking: boolean;
}

export interface ComplianceTraining {
  topics: string[];
  frequency: string;
  mandatory: boolean;
  records: boolean;
}

// ============================================================================
// Legal Reporting
// ============================================================================

export interface LegalReporting {
  internal: InternalReporting;
  regulatory: RegulatoryReporting;
  incidents: IncidentReporting;
}

export interface InternalReporting {
  schedule: string;
  recipients: string[];
  content: string[];
}

export interface RegulatoryReporting {
  agencies: { name: string; reports: { type: string; frequency: string }[] }[];
  format: string;
  retention: string;
}

export interface IncidentReporting {
  types: string[];
  timeframe: string;
  authorities: string[];
  procedure: string;
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig { baseURL: string; apiKey?: string; timeout?: number; }
export interface ProjectResponse { id: string; name: string; status: ProjectStatus; createdAt: string; updatedAt?: string; }
export interface ValidationResult { valid: boolean; errors?: ValidationError[]; }
export interface ValidationError { path: string; message: string; value?: unknown; }
export interface PaginatedResponse<T> { data: T[]; pagination: { total: number; limit: number; offset: number; hasMore: boolean }; }
