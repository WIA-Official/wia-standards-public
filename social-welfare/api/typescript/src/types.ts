/**
 * WIA-SOC-017: Social Welfare Standard - TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Basic Types
// ============================================================================

/**
 * ISO 8601 timestamp string
 */
export type Timestamp = string;

/**
 * UUID v4 identifier
 */
export type UUID = string;

// ============================================================================
// Program Types
// ============================================================================

/**
 * Welfare program types
 */
export enum ProgramType {
  FOOD_ASSISTANCE = 'FOOD_ASSISTANCE',           // 식량 지원
  HOUSING_SUPPORT = 'HOUSING_SUPPORT',           // 주택 지원
  MEDICAL_AID = 'MEDICAL_AID',                   // 의료 지원
  CHILDCARE = 'CHILDCARE',                       // 육아 지원
  DISABILITY = 'DISABILITY',                     // 장애인 지원
  UNEMPLOYMENT = 'UNEMPLOYMENT',                 // 실업 급여
  ELDERLY_CARE = 'ELDERLY_CARE',                 // 노인 돌봄
  EDUCATION = 'EDUCATION',                       // 교육 지원
  EMERGENCY_AID = 'EMERGENCY_AID',               // 긴급 지원
}

/**
 * Welfare program definition
 */
export interface WelfareProgram {
  programId: string;
  name: string;
  type: ProgramType;
  description: string;
  eligibility: EligibilityCriteria;
  benefits: BenefitStructure;
  status: ProgramStatus;
  createdAt: Timestamp;
  updatedAt: Timestamp;
}

export enum ProgramStatus {
  ACTIVE = 'ACTIVE',
  INACTIVE = 'INACTIVE',
  SUSPENDED = 'SUSPENDED',
}

// ============================================================================
// Beneficiary Types
// ============================================================================

/**
 * Beneficiary profile
 */
export interface BeneficiaryProfile {
  beneficiaryId: string;
  personalInfo: PersonalInformation;
  household: HouseholdComposition;
  financial: FinancialStatus;
  employment: EmploymentStatus;
  health?: HealthInformation;
  contact: ContactInformation;
  privacy: PrivacySettings;
  metadata: RecordMetadata;
}

/**
 * Personal information
 */
export interface PersonalInformation {
  firstName: string;
  middleName?: string;
  lastName: string;
  dateOfBirth: string; // YYYY-MM-DD
  ssn?: string; // Encrypted
  citizenship: CitizenshipStatus;
  preferredLanguage: string;
}

export enum CitizenshipStatus {
  CITIZEN = 'CITIZEN',
  PERMANENT_RESIDENT = 'PERMANENT_RESIDENT',
  TEMPORARY_RESIDENT = 'TEMPORARY_RESIDENT',
  REFUGEE = 'REFUGEE',
}

/**
 * Household composition
 */
export interface HouseholdComposition {
  householdId: string;
  size: number;
  members: HouseholdMember[];
  address: Address;
}

export interface HouseholdMember {
  memberId: string;
  relationship: RelationshipType;
  name: string;
  age: number;
  disability: boolean;
}

export enum RelationshipType {
  APPLICANT = 'APPLICANT',
  SPOUSE = 'SPOUSE',
  CHILD = 'CHILD',
  PARENT = 'PARENT',
  SIBLING = 'SIBLING',
  OTHER = 'OTHER',
}

export interface Address {
  street1: string;
  street2?: string;
  city: string;
  state: string;
  zipCode: string;
  country: string;
}

/**
 * Financial status
 */
export interface FinancialStatus {
  monthlyIncome: number;
  incomeSource: IncomeSource[];
  assets: Assets;
  expenses: Expenses;
  lastVerified: Timestamp;
}

export interface IncomeSource {
  source: IncomeType;
  amount: number;
  frequency: PaymentFrequency;
  employer?: string;
}

export enum IncomeType {
  EMPLOYMENT = 'EMPLOYMENT',
  SELF_EMPLOYMENT = 'SELF_EMPLOYMENT',
  SOCIAL_SECURITY = 'SOCIAL_SECURITY',
  PENSION = 'PENSION',
  INVESTMENTS = 'INVESTMENTS',
  CHILD_SUPPORT = 'CHILD_SUPPORT',
  OTHER = 'OTHER',
}

export enum PaymentFrequency {
  WEEKLY = 'WEEKLY',
  BIWEEKLY = 'BIWEEKLY',
  MONTHLY = 'MONTHLY',
  ANNUALLY = 'ANNUALLY',
}

export interface Assets {
  savings: number;
  realEstate: number;
  vehicles: number;
  investments: number;
  otherAssets: number;
  total: number;
}

export interface Expenses {
  rent: number;
  utilities: number;
  childcare: number;
  medical: number;
  other: number;
  total: number;
}

/**
 * Employment status
 */
export interface EmploymentStatus {
  status: EmploymentType;
  employer?: string;
  occupation?: string;
  hoursPerWeek?: number;
  startDate?: string;
}

export enum EmploymentType {
  EMPLOYED_FULL_TIME = 'EMPLOYED_FULL_TIME',
  EMPLOYED_PART_TIME = 'EMPLOYED_PART_TIME',
  SELF_EMPLOYED = 'SELF_EMPLOYED',
  UNEMPLOYED = 'UNEMPLOYED',
  DISABLED = 'DISABLED',
  RETIRED = 'RETIRED',
  STUDENT = 'STUDENT',
}

/**
 * Health information
 */
export interface HealthInformation {
  disabilities: Disability[];
  chronicConditions: string[];
  medicationNeeds: boolean;
}

export interface Disability {
  type: DisabilityType;
  severity: DisabilitySeverity;
  documentedBy: string;
  documentedAt: Timestamp;
}

export enum DisabilityType {
  PHYSICAL = 'PHYSICAL',
  MENTAL = 'MENTAL',
  INTELLECTUAL = 'INTELLECTUAL',
  SENSORY = 'SENSORY',
}

export enum DisabilitySeverity {
  MILD = 'MILD',
  MODERATE = 'MODERATE',
  SEVERE = 'SEVERE',
}

/**
 * Contact information
 */
export interface ContactInformation {
  phone: string;
  email?: string;
  preferredMethod: ContactMethod;
  emergencyContact?: EmergencyContact;
}

export enum ContactMethod {
  PHONE = 'PHONE',
  EMAIL = 'EMAIL',
  SMS = 'SMS',
  MAIL = 'MAIL',
}

export interface EmergencyContact {
  name: string;
  relationship: string;
  phone: string;
}

// ============================================================================
// Application Types
// ============================================================================

/**
 * Benefit application
 */
export interface BenefitApplication {
  applicationId: string;
  beneficiaryId: string;
  programId: string;
  submittedAt: Timestamp;
  data: ApplicationData;
  documents: Document[];
  status: ApplicationStatus;
  decision?: EligibilityDecision;
  history: ProcessingHistory[];
}

export interface ApplicationData {
  household: HouseholdComposition;
  financial: FinancialStatus;
  employment: EmploymentStatus;
  additionalInfo: Record<string, any>;
}

export enum ApplicationStatus {
  SUBMITTED = 'SUBMITTED',
  UNDER_REVIEW = 'UNDER_REVIEW',
  PENDING_VERIFICATION = 'PENDING_VERIFICATION',
  PENDING_DOCUMENTS = 'PENDING_DOCUMENTS',
  APPROVED = 'APPROVED',
  DENIED = 'DENIED',
  WITHDRAWN = 'WITHDRAWN',
}

export interface Document {
  documentId: string;
  type: DocumentType;
  fileName: string;
  uploadedAt: Timestamp;
  verifiedAt?: Timestamp;
  verificationStatus: VerificationStatus;
}

export enum DocumentType {
  ID_PROOF = 'ID_PROOF',
  INCOME_PROOF = 'INCOME_PROOF',
  ADDRESS_PROOF = 'ADDRESS_PROOF',
  MEDICAL_RECORDS = 'MEDICAL_RECORDS',
  OTHER = 'OTHER',
}

export enum VerificationStatus {
  PENDING = 'PENDING',
  VERIFIED = 'VERIFIED',
  REJECTED = 'REJECTED',
}

// ============================================================================
// Eligibility Types
// ============================================================================

/**
 * Eligibility criteria
 */
export interface EligibilityCriteria {
  incomeLimit: IncomeLimitRule;
  assetLimit?: AssetLimitRule;
  categorical: CategoricalRule[];
  geographic?: GeographicRule;
  temporal?: TemporalRule;
}

export interface IncomeLimitRule {
  limit: number;
  type: IncomeLimitType;
  householdSizeAdjustment: boolean;
}

export enum IncomeLimitType {
  GROSS_INCOME = 'GROSS_INCOME',
  NET_INCOME = 'NET_INCOME',
  PERCENT_OF_POVERTY = 'PERCENT_OF_POVERTY',
}

export interface AssetLimitRule {
  limit: number;
  exemptions: string[];
}

export interface CategoricalRule {
  criterion: string;
  required: boolean;
  values: string[];
}

export interface GeographicRule {
  allowedStates?: string[];
  allowedCounties?: string[];
  allowedZipCodes?: string[];
}

export interface TemporalRule {
  applicationPeriod?: DateRange;
  benefitPeriod?: DateRange;
  timeLimit?: number; // months
  waitingPeriod?: number; // days
}

export interface DateRange {
  start: string;
  end: string;
}

/**
 * Eligibility decision
 */
export interface EligibilityDecision {
  eligible: boolean;
  reason: string;
  benefitAmount?: number;
  effectiveDate?: string;
  expirationDate?: string;
  conditions?: string[];
  decidedBy: string;
  decidedAt: Timestamp;
}

// ============================================================================
// Benefit Types
// ============================================================================

/**
 * Benefit structure
 */
export interface BenefitStructure {
  type: BenefitType;
  calculation: BenefitCalculation;
  maximumBenefit: number;
  minimumBenefit: number;
  paymentSchedule: PaymentSchedule;
}

export enum BenefitType {
  CASH = 'CASH',
  EBT_CARD = 'EBT_CARD',
  VOUCHER = 'VOUCHER',
  DIRECT_SERVICE = 'DIRECT_SERVICE',
}

export interface BenefitCalculation {
  method: CalculationMethod;
  formula: string;
  parameters: Record<string, number>;
}

export enum CalculationMethod {
  FLAT_BENEFIT = 'FLAT_BENEFIT',
  GRADUATED_BENEFIT = 'GRADUATED_BENEFIT',
  GAP_FILLING = 'GAP_FILLING',
}

export interface PaymentSchedule {
  frequency: PaymentFrequency;
  dayOfMonth?: number;
  dayOfWeek?: number;
}

/**
 * Benefit payment
 */
export interface BenefitPayment {
  paymentId: string;
  beneficiaryId: string;
  programId: string;
  amount: number;
  paymentDate: string;
  paymentMethod: PaymentMethod;
  status: PaymentStatus;
  transactionId?: string;
}

export enum PaymentMethod {
  DIRECT_DEPOSIT = 'DIRECT_DEPOSIT',
  CHECK = 'CHECK',
  EBT_CARD = 'EBT_CARD',
  CASH = 'CASH',
}

export enum PaymentStatus {
  SCHEDULED = 'SCHEDULED',
  PROCESSED = 'PROCESSED',
  COMPLETED = 'COMPLETED',
  FAILED = 'FAILED',
  CANCELLED = 'CANCELLED',
}

// ============================================================================
// Case Management Types
// ============================================================================

/**
 * Case record
 */
export interface CaseRecord {
  caseId: string;
  beneficiaryId: string;
  caseManager: StaffReference;
  programs: ProgramEnrollment[];
  notes: CaseNote[];
  actionItems: ActionItem[];
  status: CaseStatus;
  openedAt: Timestamp;
  closedAt?: Timestamp;
}

export interface StaffReference {
  staffId: string;
  name: string;
  role: string;
}

export interface ProgramEnrollment {
  programId: string;
  enrolledAt: Timestamp;
  status: EnrollmentStatus;
  benefitAmount: number;
  nextRecertification?: string;
}

export enum EnrollmentStatus {
  ACTIVE = 'ACTIVE',
  PENDING_RECERTIFICATION = 'PENDING_RECERTIFICATION',
  SUSPENDED = 'SUSPENDED',
  TERMINATED = 'TERMINATED',
}

export interface CaseNote {
  noteId: string;
  author: StaffReference;
  timestamp: Timestamp;
  category: NoteCategory;
  content: string;
  confidential: boolean;
}

export enum NoteCategory {
  CONTACT = 'CONTACT',
  ASSESSMENT = 'ASSESSMENT',
  REFERRAL = 'REFERRAL',
  FOLLOW_UP = 'FOLLOW_UP',
  INCIDENT = 'INCIDENT',
  OTHER = 'OTHER',
}

export interface ActionItem {
  actionId: string;
  description: string;
  dueDate: string;
  assignedTo: StaffReference;
  status: ActionStatus;
  completedAt?: Timestamp;
}

export enum ActionStatus {
  PENDING = 'PENDING',
  IN_PROGRESS = 'IN_PROGRESS',
  COMPLETED = 'COMPLETED',
  CANCELLED = 'CANCELLED',
}

export enum CaseStatus {
  OPEN = 'OPEN',
  ACTIVE = 'ACTIVE',
  ON_HOLD = 'ON_HOLD',
  CLOSED = 'CLOSED',
}

// ============================================================================
// Fraud Detection Types
// ============================================================================

/**
 * Fraud alert
 */
export interface FraudAlert {
  alertId: string;
  type: FraudType;
  severity: AlertSeverity;
  beneficiaryId: string;
  programId?: string;
  description: string;
  detectedAt: Timestamp;
  status: AlertStatus;
  assignedTo?: StaffReference;
  resolution?: AlertResolution;
}

export enum FraudType {
  DUPLICATE_SSN = 'DUPLICATE_SSN',
  INCOME_INCONSISTENCY = 'INCOME_INCONSISTENCY',
  ADDRESS_MISMATCH = 'ADDRESS_MISMATCH',
  IDENTITY_FRAUD = 'IDENTITY_FRAUD',
  BENEFIT_TRAFFICKING = 'BENEFIT_TRAFFICKING',
  UNREPORTED_INCOME = 'UNREPORTED_INCOME',
}

export enum AlertSeverity {
  LOW = 'LOW',
  MEDIUM = 'MEDIUM',
  HIGH = 'HIGH',
  CRITICAL = 'CRITICAL',
}

export enum AlertStatus {
  NEW = 'NEW',
  UNDER_INVESTIGATION = 'UNDER_INVESTIGATION',
  RESOLVED = 'RESOLVED',
  FALSE_POSITIVE = 'FALSE_POSITIVE',
}

export interface AlertResolution {
  resolvedAt: Timestamp;
  resolvedBy: StaffReference;
  outcome: ResolutionOutcome;
  notes: string;
}

export enum ResolutionOutcome {
  NO_FRAUD = 'NO_FRAUD',
  FRAUD_CONFIRMED = 'FRAUD_CONFIRMED',
  FURTHER_REVIEW_NEEDED = 'FURTHER_REVIEW_NEEDED',
}

// ============================================================================
// Common Types
// ============================================================================

export interface RecordMetadata {
  createdAt: Timestamp;
  createdBy: string;
  updatedAt: Timestamp;
  updatedBy: string;
  version: number;
}

export interface PrivacySettings {
  consentToDataSharing: boolean;
  consentToResearch: boolean;
  communicationPreferences: string[];
}

export interface ProcessingHistory {
  timestamp: Timestamp;
  action: string;
  actor: string;
  details: string;
}

// ============================================================================
// API Response Types
// ============================================================================

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: ApiError;
  metadata?: ResponseMetadata;
}

export interface ApiError {
  code: string;
  message: string;
  details?: any;
}

export interface ResponseMetadata {
  timestamp: Timestamp;
  requestId: string;
  pagination?: PaginationInfo;
}

export interface PaginationInfo {
  page: number;
  pageSize: number;
  totalPages: number;
  totalItems: number;
}

export interface PaginationParams {
  page?: number;
  pageSize?: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}
