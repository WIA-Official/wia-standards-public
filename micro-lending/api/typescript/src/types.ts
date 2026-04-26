/**
 * WIA-FIN-003 Micro-Lending Standard - Type Definitions
 *
 * 弘益人間 · Benefit All Humanity
 *
 * @module @wia/micro-lending
 * @version 1.0.0
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Address information
 */
export interface Address {
  street: string;
  city: string;
  state?: string;
  country: string; // ISO 3166-1 alpha-2
  postalCode: string;
}

/**
 * Borrower types
 */
export enum BorrowerType {
  INDIVIDUAL = 'individual',
  BUSINESS = 'business'
}

/**
 * Lender types
 */
export enum LenderType {
  INDIVIDUAL = 'individual',
  INSTITUTIONAL = 'institutional'
}

/**
 * KYC verification status
 */
export enum KYCStatus {
  PENDING = 'pending',
  VERIFIED = 'verified',
  REJECTED = 'rejected'
}

// ============================================================================
// Borrower
// ============================================================================

/**
 * Borrower profile information
 */
export interface BorrowerProfile {
  firstName: string;
  lastName: string;
  email: string;
  phone: string; // E.164 format
  dateOfBirth: string; // ISO 8601 date
  nationalId?: string; // Encrypted
  address: Address;
}

/**
 * Borrower entity
 */
export interface Borrower {
  id: string; // UUID
  type: BorrowerType;
  profile: BorrowerProfile;
  creditScore: number; // 0-1000
  kycStatus: KYCStatus;
  kycVerifiedAt: string | null; // ISO 8601 datetime
  createdAt: string; // ISO 8601 datetime
  updatedAt: string; // ISO 8601 datetime
}

/**
 * Request to create a borrower
 */
export interface CreateBorrowerRequest {
  type: BorrowerType;
  profile: BorrowerProfile;
}

// ============================================================================
// Lender
// ============================================================================

/**
 * Risk tolerance levels
 */
export enum RiskTolerance {
  CONSERVATIVE = 'conservative',
  MODERATE = 'moderate',
  AGGRESSIVE = 'aggressive'
}

/**
 * Lender investment preferences
 */
export interface LenderPreferences {
  minCreditScore: number; // 0-1000
  maxLoanAmount: number;
  preferredSectors: string[];
  preferredRegions: string[]; // ISO 3166-1 alpha-2
}

/**
 * Lender investment profile
 */
export interface InvestmentProfile {
  totalCapital: number;
  availableCapital: number;
  investedCapital: number;
  riskTolerance: RiskTolerance;
  preferences: LenderPreferences;
}

/**
 * Lender profile information
 */
export interface LenderProfile {
  name: string;
  email: string;
  phone: string; // E.164 format
  address: Address;
}

/**
 * Lender entity
 */
export interface Lender {
  id: string; // UUID
  type: LenderType;
  profile: LenderProfile;
  investmentProfile: InvestmentProfile;
  kycStatus: KYCStatus;
  createdAt: string; // ISO 8601 datetime
  updatedAt: string; // ISO 8601 datetime
}

/**
 * Request to create a lender
 */
export interface CreateLenderRequest {
  type: LenderType;
  profile: LenderProfile;
  investmentProfile: InvestmentProfile;
}

/**
 * Lender portfolio statistics
 */
export interface LenderPortfolio {
  totalInvested: number;
  availableCapital: number;
  activeLoans: number;
  completedLoans: number;
  averageROI: number; // Percentage
  defaultRate: number; // Percentage
  loans: LoanSummary[];
}

// ============================================================================
// Loan
// ============================================================================

/**
 * Loan purposes
 */
export enum LoanPurpose {
  BUSINESS = 'business',
  EDUCATION = 'education',
  AGRICULTURE = 'agriculture',
  HEALTHCARE = 'healthcare',
  EMERGENCY = 'emergency',
  OTHER = 'other'
}

/**
 * Loan status
 */
export enum LoanStatus {
  DRAFT = 'draft',
  PENDING = 'pending',
  APPROVED = 'approved',
  ACTIVE = 'active',
  COMPLETED = 'completed',
  DEFAULTED = 'defaulted',
  CANCELLED = 'cancelled'
}

/**
 * Repayment frequency
 */
export enum RepaymentFrequency {
  WEEKLY = 'weekly',
  BI_WEEKLY = 'bi-weekly',
  MONTHLY = 'monthly'
}

/**
 * Installment status
 */
export enum InstallmentStatus {
  PENDING = 'pending',
  PAID = 'paid',
  OVERDUE = 'overdue',
  DEFAULTED = 'defaulted'
}

/**
 * Loan installment
 */
export interface Installment {
  number: number;
  dueDate: string; // ISO 8601 date
  principalAmount: number;
  interestAmount: number;
  totalAmount: number;
  status: InstallmentStatus;
  paidAt: string | null; // ISO 8601 datetime
}

/**
 * Repayment schedule
 */
export interface RepaymentSchedule {
  frequency: RepaymentFrequency;
  installments: Installment[];
}

/**
 * Funding progress
 */
export interface FundingProgress {
  targetAmount: number;
  fundedAmount: number;
  percentFunded: number; // 0-100
}

/**
 * Lender participation in a loan
 */
export interface LoanLender {
  lenderId: string; // UUID
  amount: number;
  percentage: number; // 0-100
}

/**
 * Loan metadata
 */
export interface LoanMetadata {
  tags: string[];
  customFields: Record<string, any>;
}

/**
 * Loan entity
 */
export interface Loan {
  id: string; // UUID
  borrowerId: string; // UUID
  amount: number;
  currency: string; // ISO 4217
  purpose: LoanPurpose;
  description: string;
  term: number; // months
  interestRate: number; // decimal, e.g., 0.12 for 12%
  status: LoanStatus;
  requestedAt: string; // ISO 8601 datetime
  approvedAt: string | null; // ISO 8601 datetime
  disbursedAt: string | null; // ISO 8601 datetime
  completedAt: string | null; // ISO 8601 datetime
  fundingProgress: FundingProgress;
  repaymentSchedule: RepaymentSchedule;
  lenders: LoanLender[];
  metadata: LoanMetadata;
  createdAt: string; // ISO 8601 datetime
  updatedAt: string; // ISO 8601 datetime
}

/**
 * Request to create a loan
 */
export interface CreateLoanRequest {
  borrowerId: string;
  amount: number;
  currency: string; // ISO 4217
  purpose: LoanPurpose;
  description: string;
  term: number; // months
  interestRate: number; // decimal
  metadata?: LoanMetadata;
}

/**
 * Loan summary (for lists)
 */
export interface LoanSummary {
  id: string;
  borrowerId: string;
  amount: number;
  currency: string;
  purpose: LoanPurpose;
  status: LoanStatus;
  fundingProgress: FundingProgress;
  createdAt: string;
}

/**
 * Request to fund a loan
 */
export interface FundLoanRequest {
  lenderId: string;
  amount: number;
}

// ============================================================================
// Credit Score
// ============================================================================

/**
 * Credit score rating
 */
export enum CreditRating {
  EXCELLENT = 'excellent',
  GOOD = 'good',
  FAIR = 'fair',
  POOR = 'poor',
  VERY_POOR = 'very_poor'
}

/**
 * Credit score factor
 */
export interface CreditFactor {
  value: number; // 0-100
  weight: number; // decimal
  score: number; // calculated
}

/**
 * Credit score factors
 */
export interface CreditFactors {
  paymentHistory: CreditFactor;
  financialStability: CreditFactor;
  networkTrust: CreditFactor;
  incomeVerification: CreditFactor;
  educationSkills: CreditFactor;
}

/**
 * Credit data source types
 */
export enum DataSourceType {
  MOBILE_USAGE = 'mobile_usage',
  UTILITY_PAYMENTS = 'utility_payments',
  SOCIAL_NETWORK = 'social_network',
  TRANSACTION_HISTORY = 'transaction_history',
  EMPLOYMENT = 'employment'
}

/**
 * Credit data source
 */
export interface DataSource {
  type: DataSourceType;
  verified: boolean;
  lastUpdated: string; // ISO 8601 datetime
}

/**
 * Credit score entity
 */
export interface CreditScore {
  borrowerId: string; // UUID
  score: number; // 0-1000
  rating: CreditRating;
  factors: CreditFactors;
  dataSources: DataSource[];
  calculatedAt: string; // ISO 8601 datetime
  expiresAt: string; // ISO 8601 datetime
}

// ============================================================================
// Payment
// ============================================================================

/**
 * Payment methods
 */
export enum PaymentMethod {
  BANK_TRANSFER = 'bank_transfer',
  MOBILE_WALLET = 'mobile_wallet',
  CARD = 'card',
  CRYPTOCURRENCY = 'cryptocurrency'
}

/**
 * Payment status
 */
export enum PaymentStatus {
  PENDING = 'pending',
  PROCESSING = 'processing',
  COMPLETED = 'completed',
  FAILED = 'failed',
  REFUNDED = 'refunded'
}

/**
 * Payment metadata
 */
export interface PaymentMetadata {
  gateway: string;
  reference: string;
  [key: string]: any;
}

/**
 * Payment entity
 */
export interface Payment {
  id: string; // UUID
  loanId: string; // UUID
  payerId: string; // UUID
  amount: number;
  currency: string; // ISO 4217
  method: PaymentMethod;
  status: PaymentStatus;
  transactionId: string;
  processedAt: string | null; // ISO 8601 datetime
  metadata: PaymentMetadata;
  createdAt: string; // ISO 8601 datetime
}

/**
 * Request to make a payment
 */
export interface MakePaymentRequest {
  loanId: string;
  amount: number;
  method: PaymentMethod;
  accountId?: string; // Bank account or wallet ID
  metadata?: PaymentMetadata;
}

// ============================================================================
// Matching
// ============================================================================

/**
 * Loan match result
 */
export interface LoanMatch {
  lenderId: string;
  matchScore: number; // 0-1
  recommendedAmount: number;
  reason: string;
}

/**
 * Investment opportunity
 */
export interface InvestmentOpportunity {
  loanId: string;
  matchScore: number; // 0-1
  recommendedAmount: number;
  borrowerCreditScore: number;
  loanAmount: number;
  purpose: LoanPurpose;
  interestRate: number;
  term: number;
}

// ============================================================================
// API Response Types
// ============================================================================

/**
 * Pagination information
 */
export interface Pagination {
  total: number;
  limit: number;
  offset: number;
}

/**
 * Paginated list response
 */
export interface PaginatedResponse<T> {
  data: T[];
  pagination: Pagination;
}

/**
 * API error details
 */
export interface ErrorDetails {
  [key: string]: any;
}

/**
 * API error response
 */
export interface APIError {
  code: string;
  message: string;
  details?: ErrorDetails;
  timestamp: string;
  requestId: string;
}

/**
 * Error response wrapper
 */
export interface ErrorResponse {
  error: APIError;
}

// ============================================================================
// SDK Configuration
// ============================================================================

/**
 * SDK environment
 */
export enum Environment {
  DEVELOPMENT = 'development',
  SANDBOX = 'sandbox',
  PRODUCTION = 'production'
}

/**
 * SDK configuration options
 */
export interface SDKConfig {
  apiKey: string;
  environment?: Environment;
  apiEndpoint?: string;
  timeout?: number; // milliseconds
  maxRetries?: number;
}

// ============================================================================
// Webhook Events
// ============================================================================

/**
 * Webhook event types
 */
export enum WebhookEventType {
  PAYMENT_INITIATED = 'payment.initiated',
  PAYMENT_PROCESSING = 'payment.processing',
  PAYMENT_COMPLETED = 'payment.completed',
  PAYMENT_FAILED = 'payment.failed',
  LOAN_CREATED = 'loan.created',
  LOAN_FUNDED = 'loan.funded',
  LOAN_COMPLETED = 'loan.completed',
  LOAN_DEFAULTED = 'loan.defaulted'
}

/**
 * Webhook event
 */
export interface WebhookEvent<T = any> {
  event: WebhookEventType;
  data: T;
  timestamp: string; // ISO 8601 datetime
}

// ============================================================================
// Auto-Investment
// ============================================================================

/**
 * Diversification rules
 */
export interface DiversificationRules {
  maxPerBorrower: number;
  maxPerSector: number;
  maxPerRegion?: number;
}

/**
 * Auto-investment settings
 */
export interface AutoInvestSettings {
  amountPerLoan: number;
  maxLoansPerDay: number;
  diversificationRules: DiversificationRules;
}

// ============================================================================
// Auto-Repayment
// ============================================================================

/**
 * Auto-repayment settings
 */
export interface AutoRepaymentSettings {
  method: PaymentMethod;
  accountId: string; // Bank account or wallet ID
  frequency: RepaymentFrequency;
  dayOfMonth?: number; // For monthly payments (1-31)
  dayOfWeek?: number; // For weekly payments (0-6, 0 = Sunday)
}

// ============================================================================
// Exports
// ============================================================================

export default {
  BorrowerType,
  LenderType,
  KYCStatus,
  RiskTolerance,
  LoanPurpose,
  LoanStatus,
  RepaymentFrequency,
  InstallmentStatus,
  CreditRating,
  DataSourceType,
  PaymentMethod,
  PaymentStatus,
  Environment,
  WebhookEventType
};
