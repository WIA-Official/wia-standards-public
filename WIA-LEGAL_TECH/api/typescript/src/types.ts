/**
 * WIA-LEGAL_TECH TypeScript Type Definitions
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Hongik Ingan - Benefit All Humanity)
 */

// ============================================================================
// Base Types
// ============================================================================

export type DocumentStatus = 'draft' | 'review' | 'approved' | 'executed' | 'archived' | 'void';
export type DocumentType = 'contract' | 'brief' | 'motion' | 'pleading' | 'evidence' | 'memo' | 'opinion' | 'agreement';
export type Classification = 'public' | 'internal' | 'confidential' | 'privileged';
export type PartyRole = 'plaintiff' | 'defendant' | 'client' | 'opposing' | 'witness' | 'expert';
export type EntityType = 'individual' | 'corporation' | 'government' | 'organization';
export type ContractType = 'nda' | 'msa' | 'sow' | 'employment' | 'lease' | 'purchase' | 'license' | 'partnership';
export type RiskLevel = 'low' | 'medium' | 'high' | 'critical';
export type Favorability = 'favorable' | 'neutral' | 'unfavorable';
export type CaseStatus = 'filed' | 'active' | 'settled' | 'dismissed' | 'judgment' | 'appeal';
export type CaseType = 'civil' | 'criminal' | 'administrative' | 'appellate';
export type ComplianceFramework = 'gdpr' | 'ccpa' | 'sox' | 'hipaa' | 'fcpa' | 'aml' | 'kyc';

// ============================================================================
// Document Interfaces
// ============================================================================

export interface Party {
  partyId: string;
  role: PartyRole;
  entityType: EntityType;
  name: string;
  contact?: ContactInfo;
}

export interface ContactInfo {
  email?: string;
  phone?: string;
  address?: Address;
}

export interface Address {
  street?: string;
  city?: string;
  state?: string;
  postalCode?: string;
  country: string;
}

export interface Author {
  authorId: string;
  name: string;
  role: 'attorney' | 'paralegal' | 'clerk' | 'associate' | 'partner';
  barNumber?: string;
  jurisdiction?: string;
}

export interface RelatedDocument {
  documentId: string;
  relationship: 'supersedes' | 'amends' | 'references' | 'exhibits' | 'incorporates';
}

export interface Checksum {
  sha256: string;
  md5?: string;
}

export interface DigitalSignature {
  signerId: string;
  algorithm: 'RSA' | 'ECDSA';
  timestamp: string;
  signature: string;
}

export interface DocumentMetadata {
  documentId: string;
  documentType: DocumentType;
  title: string;
  jurisdiction: string;
  language: string;
  version: string;
  status: DocumentStatus;
  classification: Classification;
  practiceArea: string[];
  createdAt: string;
  modifiedAt: string;
  effectiveDate?: string;
  expirationDate?: string;
  parties: Party[];
  authors: Author[];
  relatedDocuments?: RelatedDocument[];
  checksums: Checksum;
  digitalSignatures?: DigitalSignature[];
}

// ============================================================================
// Contract Interfaces
// ============================================================================

export interface ContractParty {
  partyId: string;
  role: 'buyer' | 'seller' | 'licensor' | 'licensee' | 'employer' | 'employee' | 'landlord' | 'tenant';
  entityName: string;
  legalName: string;
  jurisdiction: string;
  address: Address;
  representative: Representative;
}

export interface Representative {
  name: string;
  title: string;
  authority: 'signatory' | 'witness' | 'advisor';
}

export interface ContractTerms {
  effectiveDate: string;
  expirationDate?: string;
  autoRenewal: boolean;
  renewalTerm?: string;
  terminationNotice?: string;
  governingLaw: string;
  venue?: string;
}

export interface FinancialTerm {
  type: 'payment' | 'fee' | 'penalty' | 'escrow' | 'deposit';
  amount: number;
  currency: string;
  schedule: 'one_time' | 'recurring' | 'milestone_based';
  dueDate?: string;
  frequency?: 'daily' | 'weekly' | 'monthly' | 'quarterly' | 'annually';
}

export interface AIClauseAnalysis {
  favorability: Favorability;
  riskScore: number;
  suggestions: string[];
  precedents: string[];
}

export interface Clause {
  clauseId: string;
  type: 'confidentiality' | 'indemnification' | 'limitation_of_liability' | 'warranty' | 'ip_rights' | 'termination' | 'force_majeure' | 'dispute_resolution';
  title: string;
  text: string;
  risk: RiskLevel;
  standardDeviation?: number;
  aiAnalysis?: AIClauseAnalysis;
}

export interface Obligation {
  obligationId: string;
  partyId: string;
  type: 'deliverable' | 'action' | 'payment' | 'reporting';
  description: string;
  deadline?: string;
  status: 'pending' | 'in_progress' | 'completed' | 'overdue' | 'waived';
  dependencies?: string[];
}

export interface Milestone {
  milestoneId: string;
  name: string;
  dueDate: string;
  completionCriteria: string;
  status: 'pending' | 'achieved' | 'missed';
}

export interface Amendment {
  amendmentId: string;
  date: string;
  description: string;
  modifiedClauses: string[];
}

export interface Attachment {
  attachmentId: string;
  type: 'exhibit' | 'schedule' | 'addendum';
  title: string;
  fileUrl: string;
}

export interface Contract {
  contractId: string;
  contractType: ContractType;
  title: string;
  parties: ContractParty[];
  terms: ContractTerms;
  financialTerms: FinancialTerm[];
  clauses: Clause[];
  obligations: Obligation[];
  milestones?: Milestone[];
  amendments?: Amendment[];
  attachments?: Attachment[];
}

// ============================================================================
// Contract Analysis Interfaces
// ============================================================================

export interface ContractAnalysisRequest {
  file?: Buffer | Blob;
  contractId?: string;
  analysisType: 'full' | 'quick' | 'clause_extraction' | 'risk_analysis';
  metadata?: Partial<DocumentMetadata>;
}

export interface DocumentInfo {
  title: string;
  parties: string[];
  effectiveDate?: string;
  expirationDate?: string;
  jurisdiction?: string;
}

export interface RiskScore {
  overall: number;
  categories: {
    liability: number;
    compliance: number;
    financial: number;
    termination: number;
  };
}

export interface KeyDate {
  type: string;
  date: string;
  description: string;
}

export interface RedFlag {
  severity: 'critical' | 'high' | 'medium' | 'low';
  description: string;
  location: string;
  recommendation: string;
}

export interface Comparison {
  industryStandard: number;
  yourTemplates: number;
  marketTerms: number;
}

export interface AIInsights {
  executiveSummary: string;
  negotiationPoints: string[];
  missingClauses: string[];
  unusualTerms: string[];
}

export interface ContractAnalysis {
  contractId: string;
  status: 'processing' | 'completed' | 'failed';
  analysis?: {
    documentInfo: DocumentInfo;
    clauses: Clause[];
    financialTerms: FinancialTerm[];
    obligations: Obligation[];
    riskScore: RiskScore;
    keyDates: KeyDate[];
    redFlags: RedFlag[];
    comparisons: Comparison;
  };
  aiInsights?: AIInsights;
}

// ============================================================================
// Case Management Interfaces
// ============================================================================

export interface CaseClaim {
  claimId: string;
  type: string;
  description: string;
  damagesRequested: {
    amount: number;
    currency: string;
    type: 'compensatory' | 'punitive' | 'statutory';
  };
}

export interface DocketEntry {
  entryId: string;
  date: string;
  type: 'filing' | 'motion' | 'order' | 'hearing' | 'trial';
  description: string;
  documentId?: string;
  filedBy: string;
}

export interface Deadline {
  deadlineId: string;
  type: 'discovery' | 'motion' | 'response' | 'trial';
  dueDate: string;
  description: string;
  status: 'upcoming' | 'met' | 'missed' | 'extended';
}

export interface Deposition {
  deponentName: string;
  date: string;
  transcriptId: string;
}

export interface Discovery {
  interrogatories: number;
  depositions: Deposition[];
  documentsProduced: number;
  requestsForAdmission: number;
}

export interface Evidence {
  evidenceId: string;
  type: 'document' | 'physical' | 'digital' | 'testimonial';
  description: string;
  custodian: string;
  chainOfCustody: Array<{
    handler: string;
    timestamp: string;
    action: string;
  }>;
  admissible: boolean;
  exhibits: string[];
}

export interface CaseRecord {
  caseId: string;
  caseNumber: string;
  court: string;
  jurisdiction: string;
  caseType: CaseType;
  status: CaseStatus;
  filingDate: string;
  trialDate?: string;
  parties: Party[];
  claims: CaseClaim[];
  docket: DocketEntry[];
  deadlines: Deadline[];
  discovery: Discovery;
  evidence: Evidence[];
}

// ============================================================================
// E-Discovery Interfaces
// ============================================================================

export interface ReviewerInfo {
  reviewerId: string;
  name: string;
  role: 'attorney' | 'paralegal' | 'contract_reviewer';
  timestamp: string;
}

export interface DocumentCoding {
  responsive: boolean;
  privileged: boolean;
  confidential: boolean;
  hotDocument: boolean;
  relevance: number;
  issues: string[];
  custodian: string;
  dateRange?: {
    start: string;
    end: string;
  };
}

export interface Redaction {
  reason: 'privileged' | 'pii' | 'confidential' | 'irrelevant';
  location: 'page' | 'paragraph' | 'line';
  coordinates?: {
    x: number;
    y: number;
    width: number;
    height: number;
  };
}

export interface AIAssistance {
  predictiveRelevance: number;
  suggestedTags: string[];
  similarDocuments: string[];
  keyPhrases: string[];
}

export interface DocumentReview {
  reviewId: string;
  documentId: string;
  reviewerInfo: ReviewerInfo;
  coding: DocumentCoding;
  tags: string[];
  notes: string;
  redactions: Redaction[];
  aiAssistance?: AIAssistance;
}

// ============================================================================
// Legal Research Interfaces
// ============================================================================

export interface KeyPassage {
  text: string;
  page: number;
  context: string;
}

export interface LegalResult {
  resultId: string;
  type: 'case_law' | 'statute' | 'regulation' | 'article' | 'treatise';
  citation: string;
  title: string;
  court?: string;
  date: string;
  jurisdiction: string;
  relevanceScore: number;
  keyPassages: KeyPassage[];
  citedBy: number;
  treatment?: 'positive' | 'negative' | 'distinguished' | 'overruled' | 'followed';
  shepardSignal?: 'red' | 'yellow' | 'green';
}

export interface ResearchFilters {
  jurisdiction?: string[];
  dateRange?: {
    start: string;
    end: string;
  };
  court?: string[];
  practiceArea?: string[];
}

export interface ResearchResult {
  researchId: string;
  query: string;
  timestamp: string;
  results: LegalResult[];
  relatedSearches: string[];
  filters: ResearchFilters;
}

// ============================================================================
// Compliance Interfaces
// ============================================================================

export interface ComplianceFinding {
  findingId: string;
  severity: 'critical' | 'high' | 'medium' | 'low';
  category: 'missing_clause' | 'non_standard_language' | 'prohibited_term' | 'regulatory_violation';
  description: string;
  location: string;
  recommendation: string;
  reference: string;
}

export interface RequiredClause {
  clauseType: string;
  status: 'present' | 'missing' | 'incomplete';
  requirement: string;
}

export interface ProhibitedTerm {
  term: string;
  location: string;
  severity: 'critical' | 'high' | 'medium' | 'low';
}

export interface ComplianceCheck {
  checkId: string;
  documentId: string;
  framework: ComplianceFramework;
  timestamp: string;
  status: 'compliant' | 'non_compliant' | 'needs_review';
  score: number;
  findings: ComplianceFinding[];
  requiredClauses: RequiredClause[];
  prohibitedTerms: ProhibitedTerm[];
}

// ============================================================================
// Document Generation Interfaces
// ============================================================================

export interface Variable {
  name: string;
  type: 'string' | 'number' | 'date' | 'boolean' | 'enum';
  required: boolean;
  default?: any;
  validation?: string;
}

export interface Template {
  templateId: string;
  name: string;
  category: 'contract' | 'pleading' | 'letter' | 'memo';
  content: string;
  variables: Variable[];
  clauses: Array<{
    id: string;
    optional: boolean;
    conditions?: string;
  }>;
}

export interface GenerateDocumentRequest {
  templateId: string;
  variables: Record<string, any>;
  format: 'pdf' | 'docx' | 'html';
  includeSignatureBlocks: boolean;
}

export interface GeneratedDocument {
  documentId: string;
  downloadUrl: string;
  expiresAt: string;
  previewUrl: string;
}

// ============================================================================
// Workflow Interfaces
// ============================================================================

export interface WorkflowStep {
  id: string;
  type: 'review' | 'approval' | 'signature' | 'notification';
  assignee: string;
  deadline: number;
  conditions?: string;
  actions: Array<{
    type: 'email' | 'webhook' | 'task';
    config: Record<string, any>;
  }>;
}

export interface Workflow {
  workflowId: string;
  name: string;
  type: 'contract_review' | 'document_approval' | 'matter_intake';
  steps: WorkflowStep[];
  autoStart: boolean;
  status: 'active' | 'inactive' | 'archived';
  version: string;
}

export interface WorkflowInstance {
  instanceId: string;
  workflowId: string;
  documentId: string;
  status: 'in_progress' | 'completed' | 'cancelled';
  currentStep: string;
  assignedTo: string;
  dueDate: string;
  metadata?: Record<string, any>;
}

// ============================================================================
// API Response Types
// ============================================================================

export interface APIError {
  code: string;
  message: string;
  details?: any;
  requestId: string;
  timestamp: string;
}

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
}

export interface PaginatedResponse<T> {
  total: number;
  page: number;
  pageSize: number;
  data: T[];
}

export interface UploadResponse {
  contractId: string;
  status: 'processing' | 'completed' | 'failed';
  estimatedTime?: number;
  trackingUrl: string;
}

// ============================================================================
// Client Configuration
// ============================================================================

export interface WIALegalTechConfig {
  apiKey?: string;
  baseUrl?: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

export interface AuthTokenResponse {
  access_token: string;
  token_type: string;
  expires_in: number;
  scope: string;
}

// ============================================================================
// Search and Filter Types
// ============================================================================

export interface SearchFilters {
  dateRange?: {
    start: string;
    end: string;
  };
  custodian?: string[];
  fileType?: string[];
  tags?: string[];
  reviewStatus?: 'pending' | 'reviewed' | 'exported';
}

export interface SearchRequest {
  query: string;
  filters?: SearchFilters;
  sort?: {
    field: 'date' | 'relevance' | 'custodian';
    order: 'asc' | 'desc';
  };
  pagination?: {
    page: number;
    pageSize: number;
  };
}

export interface DocumentSearchResult {
  documentId: string;
  filename: string;
  custodian: string;
  date: string;
  fileType: string;
  size: number;
  tags: string[];
  aiPredictions: {
    responsive: number;
    privileged: number;
    relevance: number;
  };
  snippet: string;
  reviewStatus: string;
}

// ============================================================================
// Export Types
// ============================================================================

export type {
  DocumentMetadata as LegalDocument,
  Contract as LegalContract,
  CaseRecord as LegalCase,
  DocumentReview as EDiscoveryReview,
  ComplianceCheck as ComplianceReport,
  ResearchResult as LegalResearch,
  Workflow as LegalWorkflow,
};
