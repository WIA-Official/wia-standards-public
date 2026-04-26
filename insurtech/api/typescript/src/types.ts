/**
 * WIA-FIN-005 InsurTech Standard - Type Definitions
 *
 * @package @wia/insurtech
 * @version 1.0.0
 * @license MIT
 * @author WIA Standards Committee
 */

// ============================================================================
// Core Types
// ============================================================================

export type InsuranceType = 'life' | 'auto' | 'health' | 'property' | 'travel' | 'cyber';
export type PolicyStatus = 'pending' | 'active' | 'suspended' | 'cancelled' | 'expired';
export type ClaimStatus = 'submitted' | 'reviewing' | 'approved' | 'denied' | 'paid';
export type RiskLevel = 'low' | 'medium' | 'high';
export type PaymentFrequency = 'monthly' | 'quarterly' | 'annually';
export type PaymentMethod = 'credit_card' | 'bank_transfer' | 'crypto';

// ============================================================================
// Policy Types
// ============================================================================

export interface Policy {
  policyId: string;
  type: InsuranceType;
  status: PolicyStatus;
  holder: PolicyHolder;
  coverage: Coverage;
  premium: Premium;
  underwriting: Underwriting;
  beneficiaries?: Beneficiary[];
  riders?: Rider[];
  metadata: Metadata;
}

export interface PolicyHolder {
  customerId: string;
  name: string;
  dateOfBirth: string;
  contact: ContactInfo;
}

export interface ContactInfo {
  email: string;
  phone: string;
  address: Address;
}

export interface Address {
  street: string;
  city: string;
  state: string;
  zipCode: string;
  country: string;
}

export interface Coverage {
  amount: number;
  currency: string;
  deductible: number;
  effectiveDate: string;
  expirationDate: string;
  limits: {
    annual: number;
    perIncident: number;
  };
}

export interface Premium {
  annual: number;
  monthly: number;
  paymentFrequency: PaymentFrequency;
  paymentMethod: PaymentMethod;
  nextPaymentDue: string;
}

export interface Underwriting {
  riskScore: number;
  riskLevel: RiskLevel;
  factors: string[];
  assessedBy: 'human' | 'ai' | 'hybrid';
  assessmentDate: string;
}

export interface Beneficiary {
  name: string;
  relationship: string;
  percentage: number;
}

export interface Rider {
  type: string;
  description: string;
  premium: number;
}

// ============================================================================
// Claim Types
// ============================================================================

export interface Claim {
  claimId: string;
  policyId: string;
  type: 'accident' | 'health' | 'property' | 'life' | 'cyber';
  status: ClaimStatus;
  incidentDate: string;
  reportedDate: string;
  description: string;
  amount: ClaimAmount;
  assessment: ClaimAssessment;
  documents: Document[];
  payment?: PaymentInfo;
  metadata: Metadata;
}

export interface ClaimAmount {
  claimed: number;
  approved: number;
  deductible: number;
  payout: number;
}

export interface ClaimAssessment {
  fraudScore: number;
  fraudRisk: RiskLevel;
  autoApproved: boolean;
  assessor: string;
  reviewedDate: string;
  notes?: string;
}

export interface Document {
  type: 'photo' | 'receipt' | 'report' | 'medical';
  url: string;
  uploadedAt: string;
}

export interface PaymentInfo {
  method: PaymentMethod;
  status: 'pending' | 'processing' | 'completed' | 'failed';
  transactionId: string;
  paidAt?: string;
}

// ============================================================================
// Customer Types
// ============================================================================

export interface Customer {
  customerId: string;
  profile: CustomerProfile;
  riskProfile: RiskProfile;
  policies: string[];
  claims: string[];
  preferences: CustomerPreferences;
  consent: Consent;
  metadata: Metadata;
}

export interface CustomerProfile {
  name: string;
  dateOfBirth: string;
  gender: 'male' | 'female' | 'other' | 'prefer_not_to_say';
  occupation: string;
  contact: ContactInfo;
}

export interface RiskProfile {
  overallScore: number;
  healthRisk: number;
  drivingRisk: number;
  lifestyleRisk: number;
  lastAssessed: string;
}

export interface CustomerPreferences {
  communicationChannel: 'email' | 'sms' | 'app';
  language: string;
}

export interface Consent {
  dataProcessing: boolean;
  marketingCommunications: boolean;
  iotDataCollection: boolean;
  consentDate: string;
}

// ============================================================================
// Quote Types
// ============================================================================

export interface QuoteRequest {
  type: InsuranceType;
  applicant: {
    age: number;
    location: string;
    occupation?: string;
    [key: string]: any;
  };
  coverage: {
    amount: number;
    deductible: number;
  };
  term: number;
}

export interface Quote {
  quoteId: string;
  premium: {
    annual: number;
    monthly: number;
  };
  riskScore: number;
  riskLevel: RiskLevel;
  validUntil: string;
}

// ============================================================================
// Underwriting Types
// ============================================================================

export interface UnderwritingRequest {
  applicationId: string;
  applicant: CustomerProfile;
  insuranceType: InsuranceType;
  requestedCoverage: number;
}

export interface UnderwritingAssessment {
  assessmentId: string;
  applicationId: string;
  riskFactors: RiskFactor[];
  mlModelResults: MLModelResults;
  decision: UnderwritingDecision;
}

export interface RiskFactor {
  category: string;
  factor: string;
  impact: 'positive' | 'neutral' | 'negative';
  score: number;
}

export interface MLModelResults {
  modelVersion: string;
  prediction: string;
  confidence: number;
  features: Record<string, any>;
}

export interface UnderwritingDecision {
  approved: boolean;
  riskScore: number;
  recommendedPremium: number;
  conditions: string[];
  decisionDate: string;
}

// ============================================================================
// API Response Types
// ============================================================================

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  metadata?: {
    requestId: string;
    timestamp: string;
  };
}

export interface APIError {
  code: string;
  message: string;
  details?: any;
}

// ============================================================================
// Utility Types
// ============================================================================

export interface Metadata {
  createdAt: string;
  updatedAt: string;
  version?: number;
}

export interface PaginationParams {
  page: number;
  limit: number;
  sortBy?: string;
  order?: 'asc' | 'desc';
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    currentPage: number;
    totalPages: number;
    totalItems: number;
    itemsPerPage: number;
  };
}

// ============================================================================
// Client Configuration
// ============================================================================

export interface InsurTechClientConfig {
  apiKey: string;
  environment?: 'production' | 'sandbox';
  baseURL?: string;
  timeout?: number;
  retryAttempts?: number;
}

export interface WebSocketEventData {
  type: string;
  data: any;
  timestamp: string;
}
