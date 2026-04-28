/**
 * WIA-EDU-010 Student Data Standard - TypeScript Type Definitions
 * Version: 2.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export type EnrollmentStatus =
  | 'active'
  | 'leave_of_absence'
  | 'withdrawn'
  | 'graduated'
  | 'suspended'
  | 'expelled';

export type AcademicLevel =
  | 'undergraduate'
  | 'graduate'
  | 'doctoral'
  | 'certificate';

export type GradeScale =
  | 'letter'
  | 'pass_fail'
  | 'numerical'
  | 'percentage';

export type CourseStatus =
  | 'in_progress'
  | 'completed'
  | 'withdrawn'
  | 'incomplete'
  | 'failed';

export type AttendanceStatus =
  | 'present'
  | 'absent_excused'
  | 'absent_unexcused'
  | 'tardy'
  | 'early_departure';

export type Term =
  | 'Fall'
  | 'Spring'
  | 'Summer'
  | 'Winter';

export type AcademicStanding =
  | 'good_standing'
  | 'probation'
  | 'suspension'
  | 'honors'
  | 'deans_list';

// ============================================================================
// Student Profile
// ============================================================================

export interface Address {
  street: string;
  city: string;
  state: string;
  zipCode: string;
  country: string;
}

export interface EmergencyContact {
  name: string;
  relationship: string;
  phone: string;
  email?: string;
}

export interface PersonalInfo {
  firstName: string;
  lastName: string;
  middleName?: string;
  preferredName?: string;
  dateOfBirth: string;
  email: string;
  phone?: string;
  address?: Address;
  emergencyContact?: EmergencyContact;
}

export interface Enrollment {
  status: EnrollmentStatus;
  enrollmentDate: string;
  expectedGraduation?: string;
  program: string;
  major: string;
  minor?: string;
  academicLevel: AcademicLevel;
  currentYear: number;
  fullTime: boolean;
}

export interface PrivacySettings {
  directoryInformation: boolean;
  parentAccess: boolean;
  thirdPartySharing: boolean;
  researchParticipation: boolean;
  marketingCommunications: boolean;
}

export interface Metadata {
  createdAt: string;
  updatedAt: string;
  createdBy?: string;
  updatedBy?: string;
  version: string;
  standard: string;
}

export interface StudentProfile {
  studentId: string;
  personalInfo: PersonalInfo;
  enrollment: Enrollment;
  privacy: PrivacySettings;
  metadata: Metadata;
}

// ============================================================================
// Academic Records
// ============================================================================

export interface Semester {
  term: Term;
  year: number;
  startDate: string;
  endDate: string;
}

export interface Course {
  courseId: string;
  courseCode: string;
  courseName: string;
  credits: number;
  creditsAttempted?: number;
  creditsEarned?: number;
  gradeScale: GradeScale;
  grade?: string;
  gradePoints?: number;
  status: CourseStatus;
  instructor?: string;
  department?: string;
}

export interface GPA {
  semester: number;
  cumulative: number;
  major?: number;
  creditsAttempted: number;
  creditsEarned: number;
  qualityPoints?: number;
}

export interface AcademicRecord {
  recordId: string;
  studentId: string;
  semester: Semester;
  courses: Course[];
  gpa: GPA;
  academicStanding: AcademicStanding;
}

// ============================================================================
// Attendance
// ============================================================================

export interface AttendanceEntry {
  date: string;
  status: AttendanceStatus;
  arrivalTime?: string;
  departureTime?: string;
  reason?: string;
  documentation?: string;
  notes?: string;
}

export interface AttendanceSummary {
  totalClasses: number;
  present: number;
  excusedAbsences: number;
  unexcusedAbsences: number;
  tardies: number;
  attendanceRate: number;
}

export interface AttendanceRecord {
  attendanceId: string;
  studentId: string;
  courseId: string;
  attendanceRecords: AttendanceEntry[];
  summary: AttendanceSummary;
}

// ============================================================================
// Privacy & Consent
// ============================================================================

export type LegalBasis =
  | 'consent'
  | 'contract'
  | 'legal_obligation'
  | 'legitimate_interest';

export interface ConsentMetadata {
  ipAddress?: string;
  userAgent?: string;
  version?: string;
}

export interface Consent {
  consentId: string;
  studentId: string;
  purpose: string;
  granted: boolean;
  grantedAt: string;
  expiresAt?: string;
  withdrawnAt?: string;
  legalBasis: LegalBasis;
  metadata?: ConsentMetadata;
}

// ============================================================================
// Data Transfer
// ============================================================================

export interface TransferRequest {
  transferId?: string;
  sourceInstitution: string;
  destinationInstitution: string;
  studentId: string;
  dataTypes: string[];
  authorizationCode: string;
  studentConsent: boolean;
  consentTimestamp: string;
}

export interface TransferResponse {
  transferId: string;
  status: string;
  estimatedCompletion?: string;
  trackingUrl?: string;
}

// ============================================================================
// Export
// ============================================================================

export interface ExportRequest {
  studentId: string;
  format: 'json' | 'pdf' | 'xml' | 'csv';
  dataTypes?: string[];
  includeMetadata?: boolean;
  encryptionKey?: string;
}

export interface ExportResponse {
  exportId: string;
  status: 'pending' | 'processing' | 'completed' | 'failed';
  downloadUrl?: string;
  expiresAt?: string;
  fileSize?: number;
}

// ============================================================================
// API Client Configuration
// ============================================================================

export interface ClientConfig {
  baseURL: string;
  accessToken?: string;
  apiKey?: string;
  timeout?: number;
  enableSync?: boolean;
  syncURL?: string;
}

// ============================================================================
// API Responses
// ============================================================================

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    requestId: string;
    timestamp: string;
    version: string;
  };
}

export interface PaginatedResponse<T> {
  success: boolean;
  data: T[];
  pagination: {
    page: number;
    limit: number;
    total: number;
    totalPages: number;
    hasNext: boolean;
    hasPrevious: boolean;
    links?: {
      self: string;
      next?: string;
      previous?: string;
      first: string;
      last: string;
    };
  };
}

// ============================================================================
// Verifiable Credentials
// ============================================================================

export interface VerifiableCredential {
  '@context': string[];
  id: string;
  type: string[];
  issuer: {
    id: string;
    name: string;
  };
  issuanceDate: string;
  expirationDate?: string | null;
  credentialSubject: any;
  proof: {
    type: string;
    created: string;
    proofPurpose: string;
    verificationMethod: string;
    jws: string;
  };
}

export interface Transcript extends VerifiableCredential {
  type: ['VerifiableCredential', 'AcademicTranscript'];
  credentialSubject: {
    id: string;
    name: string;
    program: string;
    gpa: number;
    graduationDate?: string;
    courses: Course[];
  };
}

export interface DegreeCertificate extends VerifiableCredential {
  type: ['VerifiableCredential', 'DegreeCertificate'];
  credentialSubject: {
    id: string;
    name: string;
    degree: string;
    major: string;
    graduationDate: string;
    honors?: string;
    gpa: number;
  };
}

// ============================================================================
// Synchronization
// ============================================================================

export interface SyncMessage {
  action: string;
  entity?: string;
  operation?: string;
  entityId?: string;
  timestamp?: string;
  changes?: any;
  version?: string;
  author?: string;
}

export interface SyncConfig {
  clientId: string;
  deviceId: string;
  syncFrom?: string;
}

// ============================================================================
// Error Types
// ============================================================================

export class StudentDataError extends Error {
  code: string;
  details?: any;

  constructor(message: string, code: string, details?: any) {
    super(message);
    this.name = 'StudentDataError';
    this.code = code;
    this.details = details;
  }
}
