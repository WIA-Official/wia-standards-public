/**
 * WIA-SOC-018 Pension System Standard - TypeScript Type Definitions
 *
 * @packageDocumentation
 * @module wia-soc-018
 * @version 1.0.0
 * @license MIT
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

/**
 * Member status enumeration
 */
export enum MemberStatus {
  ACTIVE = 'active',
  INACTIVE = 'inactive',
  RETIRED = 'retired',
  DECEASED = 'deceased'
}

/**
 * Pension scheme type enumeration
 */
export enum SchemeType {
  DEFINED_BENEFIT = 'defined_benefit',
  DEFINED_CONTRIBUTION = 'defined_contribution',
  HYBRID = 'hybrid'
}

/**
 * Asset class enumeration
 */
export enum AssetClass {
  DOMESTIC_EQUITY = 'domestic_equity',
  INTERNATIONAL_EQUITY = 'international_equity',
  FIXED_INCOME = 'fixed_income',
  REAL_ESTATE = 'real_estate',
  ALTERNATIVES = 'alternatives'
}

/**
 * Member personal information
 */
export interface PersonalInfo {
  firstName: string;
  lastName: string;
  dateOfBirth: string; // ISO8601 date
  citizenship: string; // ISO 3166-1 alpha-2
  taxId: string;
  socialSecurityNumber?: string; // Encrypted
}

/**
 * Contact information
 */
export interface ContactInfo {
  email: string;
  phone: string; // E.164 format
  address: {
    street: string;
    city: string;
    state: string;
    postalCode: string;
    country: string; // ISO 3166-1 alpha-2
  };
}

/**
 * Member preferences
 */
export interface MemberPreferences {
  language: string; // ISO 639-1
  communicationMethod: 'email' | 'sms' | 'mail' | 'app';
  investmentRiskProfile: 'conservative' | 'moderate' | 'aggressive';
}

/**
 * Pension member entity
 */
export interface PensionMember {
  '@context'?: string;
  '@type'?: string;
  memberId: string; // WIA-PENSION-GLOBAL-UUID
  personalInfo: PersonalInfo;
  contactInfo: ContactInfo;
  enrollmentDate: string; // ISO8601 datetime
  status: MemberStatus;
  preferences: MemberPreferences;
}

/**
 * Employer information
 */
export interface EmployerInfo {
  employerId: string;
  employerName: string;
  employerTaxId: string;
  jurisdiction: string; // ISO 3166-1 alpha-2
}

/**
 * Earnings information
 */
export interface Earnings {
  grossEarnings: number;
  pensionableEarnings: number;
  currency: string; // ISO 4217
}

/**
 * Contribution amounts
 */
export interface Contributions {
  employeeAmount: number;
  employeeRate: number; // Percentage
  employerAmount: number;
  employerRate: number; // Percentage
  totalContribution: number;
}

/**
 * Pension scheme information
 */
export interface PensionScheme {
  schemeId: string;
  schemeName: string;
  schemeType: SchemeType;
}

/**
 * Payment information
 */
export interface PaymentInfo {
  paymentMethod: 'ach' | 'wire' | 'crypto' | 'other';
  paymentDate: string; // ISO8601 date
  paymentReference: string;
}

/**
 * Validation status
 */
export interface ValidationStatus {
  status: 'pending' | 'validated' | 'rejected' | 'adjusted';
  validatedAt?: string; // ISO8601 datetime
  validatedBy?: string;
  notes?: string;
}

/**
 * Blockchain anchor information
 */
export interface BlockchainAnchor {
  txHash: string;
  blockNumber: number;
  network: string;
}

/**
 * Pension contribution record
 */
export interface PensionContribution {
  '@context'?: string;
  '@type'?: string;
  contributionId: string;
  memberId: string;
  timestamp: string; // ISO8601 datetime
  contributionPeriod: {
    startDate: string; // ISO8601 date
    endDate: string; // ISO8601 date
  };
  employerInfo: EmployerInfo;
  earnings: Earnings;
  contributions: Contributions;
  pensionScheme: PensionScheme;
  paymentInfo: PaymentInfo;
  validationStatus: ValidationStatus;
  blockchainAnchor?: BlockchainAnchor;
}

/**
 * Benefit formula parameters
 */
export interface BenefitFormula {
  accrualRate: number; // Percentage
  multiplier: number;
  integrationOffset: number;
  colaAdjustment: number; // Percentage
}

/**
 * Calculated benefits
 */
export interface CalculatedBenefits {
  monthlyBenefit: number;
  annualBenefit: number;
  lumpsumOption: number;
  replacementRate: number; // Percentage
}

/**
 * Benefit adjustments
 */
export interface BenefitAdjustments {
  earlyRetirementReduction: number; // Percentage
  delayedRetirementCredit: number; // Percentage
  survivorBenefitReduction: number; // Percentage
  socialSecurityOffset: number;
}

/**
 * Benefit projection
 */
export interface BenefitProjection {
  age: number;
  monthlyBenefit: number;
  cumulativeBenefit: number;
  adjustmentFactors: Record<string, any>;
}

/**
 * Pension benefit calculation
 */
export interface PensionBenefitCalculation {
  '@context'?: string;
  '@type'?: string;
  calculationId: string;
  memberId: string;
  calculationDate: string; // ISO8601 datetime
  retirementAge: number;
  serviceCreditYears: number;
  earningsHistory: {
    finalAverageSalary: number;
    careerAverageEarnings: number;
    averagePeriodYears: number;
    currency: string; // ISO 4217
  };
  benefitFormula: BenefitFormula;
  calculatedBenefits: CalculatedBenefits;
  adjustments: BenefitAdjustments;
  projections: BenefitProjection[];
}

/**
 * Asset allocation entry
 */
export interface AssetAllocation {
  assetClass: AssetClass;
  allocationPercentage: number; // Percentage
  currentValue: number;
  numberOfUnits: number;
  unitPrice: number;
  returnYTD: number; // Percentage
  returnAnnualized: number; // Percentage
}

/**
 * Risk metrics
 */
export interface RiskMetrics {
  standardDeviation: number;
  sharpeRatio: number;
  beta: number;
  var95: number;
}

/**
 * Rebalancing rules
 */
export interface RebalancingRules {
  method: 'calendar' | 'threshold' | 'tactical';
  frequency: 'monthly' | 'quarterly' | 'annually';
  thresholdPercentage: number;
}

/**
 * Pension fund allocation
 */
export interface PensionFundAllocation {
  '@context'?: string;
  '@type'?: string;
  allocationId: string;
  memberId: string;
  effectiveDate: string; // ISO8601 date
  totalBalance: {
    amount: number;
    currency: string; // ISO 4217
  };
  assetAllocation: AssetAllocation[];
  riskMetrics: RiskMetrics;
  rebalancingRules: RebalancingRules;
}

/**
 * API Response wrapper
 */
export interface ApiResponse<T> {
  status: 'success' | 'error';
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: Record<string, any>;
  };
  meta: {
    requestId: string;
    timestamp: string; // ISO8601
  };
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    page: number;
    limit: number;
    totalPages: number;
    totalItems: number;
  };
}

/**
 * Cross-border transfer request
 */
export interface CrossBorderTransferRequest {
  memberId: string;
  fromJurisdiction: string; // ISO 3166-1 alpha-2
  toJurisdiction: string; // ISO 3166-1 alpha-2
  transferAmount: number;
  transferType: 'full' | 'partial';
  receivingPensionScheme: string;
}

/**
 * Transfer status
 */
export interface TransferStatus {
  transferId: string;
  status: 'initiated' | 'validated' | 'in_transit' | 'completed' | 'failed';
  timeline: {
    initiated: string; // ISO8601
    expectedCompletion: string; // ISO8601
    actualCompletion?: string; // ISO8601
  };
  details?: Record<string, any>;
}
