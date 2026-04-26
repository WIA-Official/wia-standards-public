/**
 * WIA-FIN-018: Health Insurance Data Standard - TypeScript Types
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Enums
// ============================================================================

export enum ClaimType {
  INPATIENT = 'inpatient',
  OUTPATIENT = 'outpatient',
  PHARMACY = 'pharmacy',
  EMERGENCY = 'emergency',
  PREVENTIVE = 'preventive',
  MENTAL_HEALTH = 'mental_health',
  DENTAL = 'dental',
  VISION = 'vision',
}

export enum ClaimStatus {
  SUBMITTED = 'submitted',
  PENDING = 'pending',
  IN_REVIEW = 'in_review',
  APPROVED = 'approved',
  DENIED = 'denied',
  PAID = 'paid',
  APPEALED = 'appealed',
}

export enum MemberStatus {
  ACTIVE = 'active',
  INACTIVE = 'inactive',
  SUSPENDED = 'suspended',
  TERMINATED = 'terminated',
  PENDING = 'pending',
}

export enum PlanType {
  HMO = 'hmo',
  PPO = 'ppo',
  EPO = 'epo',
  POS = 'pos',
  HDHP = 'hdhp',
}

// ============================================================================
// Member Types
// ============================================================================

export interface InsuranceMember {
  memberId: string;
  firstName: string;
  lastName: string;
  dateOfBirth: string;
  gender: 'male' | 'female' | 'other';
  policyNumber: string;
  groupNumber?: string;
  status: MemberStatus;
  effectiveDate: string;
  terminationDate?: string;
  dependents?: Dependent[];
  contact: ContactInfo;
}

export interface Dependent {
  dependentId: string;
  relationship: 'spouse' | 'child' | 'domestic_partner';
  firstName: string;
  lastName: string;
  dateOfBirth: string;
  effectiveDate: string;
}

export interface ContactInfo {
  phone?: string;
  email?: string;
  address: Address;
}

export interface Address {
  street1: string;
  street2?: string;
  city: string;
  state: string;
  zipCode: string;
  country: string;
}

// ============================================================================
// Claim Types
// ============================================================================

export interface InsuranceClaim {
  claimId: string;
  memberId: string;
  providerId: string;
  type: ClaimType;
  serviceDate: string;
  diagnosisCodes: string[];
  procedureCodes: string[];
  charges: ClaimCharges;
  status: ClaimStatus;
  submissionDate: string;
  processedDate?: string;
  denialReason?: string;
}

export interface ClaimCharges {
  totalAmount: number;
  allowedAmount: number;
  deductible: number;
  coinsurance: number;
  copay: number;
  patientResponsibility: number;
  insurancePays: number;
}

// ============================================================================
// Coverage Types
// ============================================================================

export interface CoverageDetails {
  planId: string;
  planName: string;
  planType: PlanType;
  annualDeductible: DeductibleInfo;
  outOfPocketMax: OutOfPocketInfo;
  coinsurancePercentage: number;
  copays: CopaySchedule;
}

export interface DeductibleInfo {
  individual: number;
  family: number;
  individualMet: number;
  familyMet: number;
}

export interface OutOfPocketInfo {
  individual: number;
  family: number;
  individualMet: number;
  familyMet: number;
}

export interface CopaySchedule {
  primaryCare: number;
  specialist: number;
  urgentCare: number;
  emergency: number;
  genericRx: number;
  brandRx: number;
}

// ============================================================================
// Provider Types
// ============================================================================

export interface Provider {
  providerId: string;
  npi: string;
  name: string;
  specialty: string;
  networkStatus: 'in_network' | 'out_of_network';
  address: Address;
  phone: string;
}

// ============================================================================
// API Types
// ============================================================================

export interface WIAInsuranceConfig {
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
