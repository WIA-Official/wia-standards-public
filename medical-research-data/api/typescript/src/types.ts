/**
 * WIA-MED-019: Medical Research Data Standard - TypeScript Types
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Enums
// ============================================================================

export enum StudyStatus {
  PLANNING = 'planning',
  RECRUITING = 'recruiting',
  ACTIVE = 'active',
  COMPLETED = 'completed',
  SUSPENDED = 'suspended',
  TERMINATED = 'terminated',
}

export enum StudyPhase {
  PHASE_1 = 'phase_1',
  PHASE_2 = 'phase_2',
  PHASE_3 = 'phase_3',
  PHASE_4 = 'phase_4',
  OBSERVATIONAL = 'observational',
}

export enum ConsentStatus {
  PENDING = 'pending',
  CONSENTED = 'consented',
  WITHDRAWN = 'withdrawn',
  EXPIRED = 'expired',
}

export enum DataAccessLevel {
  PUBLIC = 'public',
  REGISTERED = 'registered',
  CONTROLLED = 'controlled',
  RESTRICTED = 'restricted',
}

// ============================================================================
// Study Types
// ============================================================================

export interface ResearchStudy {
  studyId: string;
  title: string;
  description: string;
  principalInvestigator: Researcher;
  institution: string;
  phase: StudyPhase;
  status: StudyStatus;
  startDate: string;
  endDate?: string;
  enrollmentTarget: number;
  enrollmentCurrent: number;
  irbApproval: IRBApproval;
}

export interface Researcher {
  researcherId: string;
  name: string;
  email: string;
  institution: string;
  orcid?: string;
  role: 'principal' | 'co_investigator' | 'coordinator';
}

export interface IRBApproval {
  approvalId: string;
  irbName: string;
  approvalDate: string;
  expirationDate: string;
  protocolVersion: string;
}

// ============================================================================
// Participant Types
// ============================================================================

export interface Participant {
  participantId: string;
  studyId: string;
  enrollmentDate: string;
  status: 'enrolled' | 'active' | 'completed' | 'withdrawn';
  consentStatus: ConsentStatus;
  demographics: Demographics;
}

export interface Demographics {
  ageRange: string;
  gender: string;
  ethnicity?: string;
  region?: string;
}

// ============================================================================
// Dataset Types
// ============================================================================

export interface Dataset {
  datasetId: string;
  studyId: string;
  name: string;
  description: string;
  version: string;
  accessLevel: DataAccessLevel;
  format: string;
  sizeBytes: number;
  recordCount: number;
  createdAt: string;
  updatedAt: string;
  variables: DataVariable[];
}

export interface DataVariable {
  name: string;
  type: 'string' | 'number' | 'boolean' | 'date' | 'categorical';
  description: string;
  unit?: string;
  categories?: string[];
}

export interface DataAccessRequest {
  requestId: string;
  datasetId: string;
  requesterId: string;
  purpose: string;
  status: 'pending' | 'approved' | 'denied';
  submittedAt: string;
  reviewedAt?: string;
}

// ============================================================================
// API Types
// ============================================================================

export interface WIAConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  debug?: boolean;
}

export interface APIResponse<T = unknown> {
  success: boolean;
  data?: T;
  error?: APIError;
  timestamp: string;
}

export interface APIError {
  code: string;
  message: string;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    page: number;
    pageSize: number;
    totalPages: number;
    totalCount: number;
  };
}
