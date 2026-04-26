/**
 * WIA-FIN-023 Financial Inclusion Standard
 * TypeScript Type Definitions
 *
 * @version 2.0.0
 * @license MIT
 */

// ============================================================================
// Core Types
// ============================================================================

export type Currency = string; // ISO 4217 currency code
export type PhoneNumber = string; // E.164 format
export type AccountId = string;
export type TransactionId = string;
export type UserId = string;

// ============================================================================
// User Segments
// ============================================================================

export enum UserSegment {
  UNBANKED_RURAL = 'unbanked-rural',
  WOMEN_ENTREPRENEURS = 'women-entrepreneurs',
  MIGRANT_WORKERS = 'migrant-workers',
  STUDENTS = 'students',
  ELDERLY = 'elderly',
  REFUGEES = 'refugees',
  SMALLHOLDER_FARMERS = 'smallholder-farmers',
  INFORMAL_WORKERS = 'informal-workers'
}

// ============================================================================
// KYC Levels
// ============================================================================

export enum KYCLevel {
  TIER_1 = 'tier-1',        // Basic: Name, phone, ID photo. Limit: $500/month
  TIER_2 = 'tier-2',        // Intermediate: + Address verification. Limit: $2,500/month
  TIER_3 = 'tier-3',        // Full: Complete documentation. No limit
  E_KYC = 'e-kyc'           // Biometric digital KYC
}

// ============================================================================
// Account Types
// ============================================================================

export enum AccountType {
  BASIC = 'basic',
  STANDARD = 'standard',
  PREMIUM = 'premium',
  BUSINESS = 'business',
  SAVINGS = 'savings',
  INVESTMENT = 'investment'
}

export enum AccountStatus {
  ACTIVE = 'active',
  INACTIVE = 'inactive',
  SUSPENDED = 'suspended',
  CLOSED = 'closed',
  PENDING_VERIFICATION = 'pending-verification'
}

// ============================================================================
// User & Account Interfaces
// ============================================================================

export interface User {
  id: UserId;
  phoneNumber: PhoneNumber;
  firstName: string;
  lastName: string;
  email?: string;
  dateOfBirth?: string;
  gender?: 'male' | 'female' | 'other' | 'prefer-not-to-say';
  address?: Address;
  kycLevel: KYCLevel;
  userSegment?: UserSegment;
  language: string;
  createdAt: Date;
  updatedAt: Date;
}

export interface Address {
  street?: string;
  city: string;
  state?: string;
  country: string;
  postalCode?: string;
}

export interface Account {
  id: AccountId;
  userId: UserId;
  accountNumber: string;
  accountType: AccountType;
  currency: Currency;
  balance: number;
  availableBalance: number;
  status: AccountStatus;
  kycLevel: KYCLevel;
  monthlyTransactionLimit: number;
  createdAt: Date;
  updatedAt: Date;
}

// ============================================================================
// Transaction Types
// ============================================================================

export enum TransactionType {
  P2P_TRANSFER = 'p2p-transfer',
  CASH_IN = 'cash-in',
  CASH_OUT = 'cash-out',
  BILL_PAYMENT = 'bill-payment',
  MERCHANT_PAYMENT = 'merchant-payment',
  LOAN_DISBURSEMENT = 'loan-disbursement',
  LOAN_REPAYMENT = 'loan-repayment',
  SAVINGS_DEPOSIT = 'savings-deposit',
  SAVINGS_WITHDRAWAL = 'savings-withdrawal',
  INTERNATIONAL_REMITTANCE = 'international-remittance'
}

export enum TransactionStatus {
  PENDING = 'pending',
  PROCESSING = 'processing',
  COMPLETED = 'completed',
  FAILED = 'failed',
  REVERSED = 'reversed'
}

export interface Transaction {
  id: TransactionId;
  type: TransactionType;
  fromAccountId: AccountId;
  toAccountId?: AccountId;
  toPhoneNumber?: PhoneNumber;
  amount: number;
  currency: Currency;
  fee: number;
  status: TransactionStatus;
  description?: string;
  reference?: string;
  metadata?: Record<string, any>;
  createdAt: Date;
  completedAt?: Date;
}

export interface TransactionRequest {
  type: TransactionType;
  fromAccountId: AccountId;
  toAccountId?: AccountId;
  toPhoneNumber?: PhoneNumber;
  amount: number;
  currency: Currency;
  description?: string;
  reference?: string;
  pin: string;
}

// ============================================================================
// Payment Services
// ============================================================================

export interface P2PTransferRequest {
  fromAccountId: AccountId;
  toPhoneNumber: PhoneNumber;
  amount: number;
  currency: Currency;
  description?: string;
  pin: string;
}

export interface BillPayment {
  billerId: string;
  accountNumber: string;
  amount: number;
  currency: Currency;
}

export interface MerchantPayment {
  merchantId: string;
  amount: number;
  currency: Currency;
  reference?: string;
}

// ============================================================================
// Credit & Lending
// ============================================================================

export enum LoanStatus {
  PENDING_APPROVAL = 'pending-approval',
  APPROVED = 'approved',
  DISBURSED = 'disbursed',
  ACTIVE = 'active',
  PAID_OFF = 'paid-off',
  DEFAULTED = 'defaulted',
  REJECTED = 'rejected'
}

export interface LoanApplication {
  userId: UserId;
  amount: number;
  currency: Currency;
  purpose: string;
  term: number; // months
  requestedAt: Date;
}

export interface Loan {
  id: string;
  userId: UserId;
  principalAmount: number;
  interestRate: number;
  term: number;
  currency: Currency;
  status: LoanStatus;
  disbursementDate?: Date;
  repaymentSchedule: RepaymentSchedule[];
  totalRepaid: number;
  outstandingBalance: number;
}

export interface RepaymentSchedule {
  dueDate: Date;
  amount: number;
  principal: number;
  interest: number;
  status: 'pending' | 'paid' | 'overdue';
}

export interface CreditScore {
  userId: UserId;
  score: number; // 0-1000
  factors: {
    transactionHistory: number;
    repaymentHistory: number;
    savingsPattern: number;
    alternativeData: number;
  };
  risk: 'low' | 'medium' | 'high';
  calculatedAt: Date;
}

// ============================================================================
// Savings & Investment
// ============================================================================

export interface SavingsGoal {
  id: string;
  userId: UserId;
  name: string;
  targetAmount: number;
  currentAmount: number;
  currency: Currency;
  targetDate: Date;
  autoSaveAmount?: number; // automatic periodic savings
  autoSaveFrequency?: 'daily' | 'weekly' | 'monthly';
  status: 'active' | 'completed' | 'cancelled';
}

export interface Investment {
  id: string;
  userId: UserId;
  type: 'mutual-fund' | 'index-fund' | 'green-bond' | 'p2p-lending';
  amount: number;
  currency: Currency;
  returns: number;
  purchaseDate: Date;
  maturityDate?: Date;
}

// ============================================================================
// Insurance
// ============================================================================

export interface InsurancePolicy {
  id: string;
  userId: UserId;
  type: 'life' | 'health' | 'crop' | 'livestock' | 'device' | 'unemployment';
  coverageAmount: number;
  premium: number;
  premiumFrequency: 'daily' | 'weekly' | 'monthly' | 'annually';
  currency: Currency;
  status: 'active' | 'lapsed' | 'claimed' | 'expired';
  startDate: Date;
  endDate: Date;
  beneficiaries?: Beneficiary[];
}

export interface Beneficiary {
  name: string;
  relationship: string;
  phoneNumber: PhoneNumber;
  percentage: number; // 0-100
}

// ============================================================================
// Agent Network
// ============================================================================

export interface Agent {
  id: string;
  businessName: string;
  ownerName: string;
  phoneNumber: PhoneNumber;
  address: Address;
  location: {
    latitude: number;
    longitude: number;
  };
  status: 'active' | 'inactive' | 'suspended';
  services: AgentService[];
  commissionRate: number;
  eMoneyBalance: number;
  cashBalance: number;
  registeredAt: Date;
}

export enum AgentService {
  CASH_IN = 'cash-in',
  CASH_OUT = 'cash-out',
  ACCOUNT_REGISTRATION = 'account-registration',
  PIN_RESET = 'pin-reset',
  BALANCE_INQUIRY = 'balance-inquiry'
}

export interface AgentTransaction {
  id: TransactionId;
  agentId: string;
  customerId: UserId;
  service: AgentService;
  amount: number;
  commission: number;
  currency: Currency;
  timestamp: Date;
}

// ============================================================================
// SDK Configuration
// ============================================================================

export interface SDKConfig {
  apiKey: string;
  environment: 'sandbox' | 'production';
  baseURL?: string;
  timeout?: number;
  retryAttempts?: number;
  locale?: string;
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
    timestamp: Date;
  };
}

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, any>;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    page: number;
    pageSize: number;
    totalPages: number;
    totalItems: number;
  };
}

// ============================================================================
// Analytics & Reporting
// ============================================================================

export interface UsageMetrics {
  period: {
    start: Date;
    end: Date;
  };
  activeUsers: number;
  newUsers: number;
  transactionVolume: number;
  transactionValue: number;
  averageTransactionSize: number;
  successRate: number;
}

export interface ImpactMetrics {
  unbankedServed: number;
  womenPercentage: number;
  ruralPercentage: number;
  averageSavings: number;
  loansIssued: number;
  insuranceCoverage: number;
  financialHealthScore: number;
}

// ============================================================================
// Webhook Events
// ============================================================================

export enum WebhookEvent {
  ACCOUNT_CREATED = 'account.created',
  ACCOUNT_VERIFIED = 'account.verified',
  TRANSACTION_COMPLETED = 'transaction.completed',
  TRANSACTION_FAILED = 'transaction.failed',
  LOAN_APPROVED = 'loan.approved',
  LOAN_DISBURSED = 'loan.disbursed',
  PAYMENT_RECEIVED = 'payment.received',
  KYC_VERIFIED = 'kyc.verified',
  FRAUD_DETECTED = 'fraud.detected'
}

export interface WebhookPayload {
  event: WebhookEvent;
  timestamp: Date;
  data: Record<string, any>;
  signature: string;
}
