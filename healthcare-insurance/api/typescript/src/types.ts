/**
 * WIA-SOC-019 Healthcare Insurance Standard - TypeScript Type Definitions
 * @version 1.0.0
 * @license MIT
 */

// ===========================
// Enumerations
// ===========================

export enum Gender {
  MALE = 'MALE',
  FEMALE = 'FEMALE',
  OTHER = 'OTHER',
  UNKNOWN = 'UNKNOWN',
}

export enum MaritalStatus {
  SINGLE = 'SINGLE',
  MARRIED = 'MARRIED',
  DIVORCED = 'DIVORCED',
  WIDOWED = 'WIDOWED',
  SEPARATED = 'SEPARATED',
  DOMESTIC_PARTNER = 'DOMESTIC_PARTNER',
}

export enum CoverageStatus {
  ACTIVE = 'ACTIVE',
  SUSPENDED = 'SUSPENDED',
  TERMINATED = 'TERMINATED',
  PENDING = 'PENDING',
}

export enum PlanType {
  INDIVIDUAL = 'INDIVIDUAL',
  FAMILY = 'FAMILY',
  GROUP = 'GROUP',
  GOVERNMENT = 'GOVERNMENT',
  MEDICARE = 'MEDICARE',
  MEDICAID = 'MEDICAID',
}

export enum CoverageLevel {
  BRONZE = 'BRONZE',
  SILVER = 'SILVER',
  GOLD = 'GOLD',
  PLATINUM = 'PLATINUM',
  CATASTROPHIC = 'CATASTROPHIC',
  BASIC = 'BASIC',
  COMPREHENSIVE = 'COMPREHENSIVE',
}

export enum Relationship {
  SPOUSE = 'SPOUSE',
  CHILD = 'CHILD',
  PARENT = 'PARENT',
  DOMESTIC_PARTNER = 'DOMESTIC_PARTNER',
  OTHER = 'OTHER',
}

export enum ClaimType {
  PROFESSIONAL = 'PROFESSIONAL',
  INSTITUTIONAL = 'INSTITUTIONAL',
  DENTAL = 'DENTAL',
  PHARMACY = 'PHARMACY',
  VISION = 'VISION',
}

export enum ClaimStatus {
  SUBMITTED = 'SUBMITTED',
  IN_REVIEW = 'IN_REVIEW',
  APPROVED = 'APPROVED',
  DENIED = 'DENIED',
  PARTIAL = 'PARTIAL',
  APPEALED = 'APPEALED',
  PAID = 'PAID',
}

export enum ServiceType {
  OUTPATIENT = 'OUTPATIENT',
  INPATIENT = 'INPATIENT',
  EMERGENCY = 'EMERGENCY',
  SURGERY = 'SURGERY',
  DIAGNOSTIC = 'DIAGNOSTIC',
  PHARMACY = 'PHARMACY',
  DENTAL = 'DENTAL',
  VISION = 'VISION',
  PRIMARY_CARE = 'PRIMARY_CARE',
  SPECIALIST = 'SPECIALIST',
  MENTAL_HEALTH = 'MENTAL_HEALTH',
  PHYSICAL_THERAPY = 'PHYSICAL_THERAPY',
}

export enum ProviderType {
  PHYSICIAN = 'PHYSICIAN',
  HOSPITAL = 'HOSPITAL',
  CLINIC = 'CLINIC',
  PHARMACY = 'PHARMACY',
  LAB = 'LAB',
  IMAGING = 'IMAGING',
  DME = 'DME',
  MENTAL_HEALTH = 'MENTAL_HEALTH',
  DENTAL = 'DENTAL',
  VISION = 'VISION',
}

export enum NetworkStatus {
  IN_NETWORK = 'IN_NETWORK',
  OUT_OF_NETWORK = 'OUT_OF_NETWORK',
  PENDING = 'PENDING',
  TERMINATED = 'TERMINATED',
}

// ===========================
// Core Data Types
// ===========================

export interface Address {
  street: string;
  city: string;
  state: string;
  postalCode: string;
  country: string;
}

export interface ContactInfo {
  address: Address;
  phone: string;
  email: string;
  preferredLanguage?: string;
}

export interface MoneyAmount {
  amount: number;
  currency: string;
}

export interface DateRange {
  from: string; // ISO 8601 date
  to?: string;
}

// ===========================
// Member & Enrollment
// ===========================

export interface PersonalInfo {
  firstName: string;
  middleName?: string;
  lastName: string;
  dateOfBirth: string; // ISO 8601 date
  gender: Gender;
  maritalStatus?: MaritalStatus;
}

export interface Dependent {
  dependentId: string;
  relationship: Relationship;
  firstName: string;
  lastName: string;
  dateOfBirth: string;
}

export interface Deductible {
  individual: number;
  family: number;
  currency: string;
}

export interface OutOfPocketMax {
  individual: number;
  family: number;
  currency: string;
}

export interface Coverage {
  coverageId: string;
  memberId: string;
  policyNumber: string;
  groupNumber?: string;
  planType: PlanType;
  coverageLevel: CoverageLevel;
  effectiveDate: string;
  terminationDate?: string;
  status: CoverageStatus;
  deductible: Deductible;
  outOfPocketMax: OutOfPocketMax;
  dependents: Dependent[];
}

export interface Member {
  memberId: string;
  nationalId?: string;
  personalInfo: PersonalInfo;
  contact: ContactInfo;
  coverage: Coverage;
}

export interface EnrollmentRequest {
  personalInfo: PersonalInfo;
  contact: ContactInfo;
  coverageType: PlanType;
  planId: string;
  effectiveDate: string;
  dependents?: Omit<Dependent, 'dependentId'>[];
}

export interface EnrollmentResponse {
  memberId: string;
  enrollmentId: string;
  status: CoverageStatus;
  effectiveDate: string;
  policyNumber: string;
}

// ===========================
// Claims
// ===========================

export interface DiagnosisCode {
  code: string; // ICD-10/11
  type: 'PRIMARY' | 'SECONDARY' | 'ADMITTING';
  description: string;
}

export interface ProcedureCode {
  code: string; // CPT, HCPCS
  modifiers?: string[];
  description: string;
  serviceDate: string;
  quantity: number;
  chargedAmount: MoneyAmount;
  allowedAmount?: MoneyAmount;
}

export interface PatientResponsibility {
  copay: number;
  coinsurance: number;
  deductible: number;
  total: number;
}

export interface DenialReason {
  code: string;
  description: string;
}

export interface ClaimAdjudication {
  adjudicatedDate: string;
  approvedAmount: number;
  deniedAmount: number;
  patientResponsibility: PatientResponsibility;
  insurancePayment: number;
  denialReasons?: DenialReason[];
}

export interface Claim {
  claimId: string;
  claimNumber: string;
  memberId: string;
  providerId: string;
  claimType: ClaimType;
  submissionDate: string;
  serviceDate: DateRange;
  diagnosis: DiagnosisCode[];
  procedures: ProcedureCode[];
  totalCharged: MoneyAmount;
  status: ClaimStatus;
  adjudication?: ClaimAdjudication;
}

export interface ClaimSubmission {
  memberId: string;
  providerId: string;
  serviceDate: string;
  diagnosis: Omit<DiagnosisCode, 'description'>[];
  procedures: Omit<ProcedureCode, 'allowedAmount'>[];
  totalCharged: number;
  currency: string;
}

export interface ClaimSubmissionResponse {
  claimId: string;
  claimNumber: string;
  status: ClaimStatus;
  submittedDate: string;
  estimatedProcessingDays: number;
}

// ===========================
// Eligibility
// ===========================

export interface DeductibleInfo {
  annual: number;
  met: number;
  remaining: number;
}

export interface OutOfPocketInfo {
  annual: number;
  met: number;
  remaining: number;
}

export interface EligibilityRequest {
  memberId: string;
  providerId?: string;
  serviceType: ServiceType;
  serviceDate: string;
}

export interface EligibilityResponse {
  eligible: boolean;
  coverageStatus: CoverageStatus;
  copay?: number;
  coinsurance?: number;
  deductible: DeductibleInfo;
  outOfPocketMax: OutOfPocketInfo;
  preAuthRequired: boolean;
  message?: string;
}

export interface BenefitsRequest {
  memberId: string;
  serviceType: ServiceType;
}

export interface BenefitsResponse {
  serviceType: ServiceType;
  covered: boolean;
  coveragePercentage: number;
  requiresPriorAuth: boolean;
  networkStatus: NetworkStatus;
  annualLimit?: number;
  lifetimeLimit?: number;
}

// ===========================
// Providers
// ===========================

export interface ProviderCredentials {
  licenseNumber: string;
  licenseState: string;
  licenseExpiration: string;
  boardCertifications?: BoardCertification[];
  deaNumber?: string;
  medicareNumber?: string;
  medicaidNumber?: string;
}

export interface BoardCertification {
  specialty: string;
  boardName: string;
  certificationDate: string;
  expirationDate?: string;
}

export interface Specialty {
  specialtyCode: string;
  specialtyName: string;
  isPrimary: boolean;
}

export interface ProviderNetworkInfo {
  status: NetworkStatus;
  effectiveDate: string;
  terminationDate?: string;
  contractId?: string;
  paymentTier?: string;
}

export interface QualityMetrics {
  patientSatisfactionScore?: number;
  numberOfPatients?: number;
  numberOfReviews?: number;
  hedisScores?: Record<string, number>;
  accreditations?: Accreditation[];
}

export interface Accreditation {
  accreditingBody: string;
  accreditationType: string;
  effectiveDate: string;
  expirationDate?: string;
}

export interface Provider {
  providerId: string;
  npi: string;
  providerType: ProviderType;
  name: string;
  organizationName?: string;
  taxId?: string;
  address: Address;
  phone: string;
  fax?: string;
  email?: string;
  website?: string;
  credentials?: ProviderCredentials;
  specialties: Specialty[];
  networkStatus: ProviderNetworkInfo;
  qualityMetrics?: QualityMetrics;
}

export interface ProviderSearchRequest {
  specialty?: string;
  city?: string;
  state?: string;
  zipCode?: string;
  inNetwork?: boolean;
  acceptingNewPatients?: boolean;
  page?: number;
  pageSize?: number;
}

export interface ProviderSearchResponse {
  results: Provider[];
  total: number;
  page: number;
  pageSize: number;
}

// ===========================
// Premiums
// ===========================

export interface RiskAdjustments {
  ageFactor: number;
  genderFactor: number;
  geographicFactor: number;
  healthStatusFactor: number;
  tobaccoSurcharge: number;
}

export interface Subsidies {
  governmentSubsidy: number;
  employerContribution: number;
  otherSubsidies: number;
}

export interface Premium {
  premiumId: string;
  memberId: string;
  billingPeriod: DateRange;
  basePremium: number;
  riskAdjustments: RiskAdjustments;
  subsidies: Subsidies;
  totalPremium: number;
  memberResponsibility: number;
  paymentFrequency: 'MONTHLY' | 'QUARTERLY' | 'SEMI_ANNUAL' | 'ANNUAL';
  currency: string;
}

export interface PremiumCalculationRequest {
  age: number;
  gender: Gender;
  zipCode: string;
  planId: string;
  dependents: number;
  smoker: boolean;
}

export interface PremiumCalculationResponse {
  monthlyPremium: number;
  annualPremium: number;
  breakdown: {
    basePremium: number;
    dependentCost: number;
    geographicAdjustment: number;
    subsidyEligible: boolean;
    estimatedSubsidy: number;
    netPremium: number;
  };
}

// ===========================
// Cross-Border Healthcare
// ===========================

export interface ReciprocalAgreement {
  partnerCountry: string;
  agreementType: 'FULL_RECIPROCITY' | 'LIMITED' | 'EMERGENCY_ONLY';
  effectiveDate: string;
  expirationDate?: string;
}

export interface TravelAssistance {
  emergencyEvacuation: boolean;
  medicalRepatriation: boolean;
  translationServices: boolean;
  legalAssistance: boolean;
}

export interface CrossBorderCoverage {
  coverageId: string;
  memberId: string;
  homeCountry: string;
  coveredCountries: string[];
  emergencyCoverageOnly: boolean;
  prePlannedCareCoverage: boolean;
  coverageLimits: {
    annualLimit: number;
    perVisitLimit: number;
    daysPerYear: number;
  };
  reciprocalAgreements: ReciprocalAgreement[];
  travelAssistance: TravelAssistance;
}

export interface CrossBorderRequest {
  memberId: string;
  destinationCountry: string;
  serviceType: ServiceType;
  estimatedCost: number;
}

export interface CrossBorderResponse {
  covered: boolean;
  coveragePercentage: number;
  estimatedPayment: number;
  patientResponsibility: number;
  requiresPreAuth: boolean;
  networkProviders: Provider[];
}

// ===========================
// API Configuration
// ===========================

export interface APIConfig {
  apiKey: string;
  baseURL?: string;
  timeout?: number;
  retryAttempts?: number;
}

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, any>;
  requestId?: string;
}

export interface PaginationParams {
  page?: number;
  pageSize?: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

export interface PaginatedResponse<T> {
  data: T[];
  total: number;
  page: number;
  pageSize: number;
  totalPages: number;
}
