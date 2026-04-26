/**
 * WIA-FIN-014 Cross-Border Payment SDK - Type Definitions
 *
 * @module @wia/cross-border-payment-sdk/types
 * @version 1.0.0
 */

// ============================================================================
// Core Types
// ============================================================================

export type Currency = string; // ISO 4217 currency code
export type Country = string;  // ISO 3166-1 alpha-2 country code
export type Timestamp = string; // ISO 8601 timestamp

export type PaymentMethod = 'SWIFT' | 'SEPA' | 'BLOCKCHAIN' | 'RTP' | 'INSTANT';

export type PaymentStatus =
  | 'PENDING'
  | 'PROCESSING'
  | 'COMPLETED'
  | 'FAILED'
  | 'REVERSED';

export type RiskLevel = 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';

export type ChargeBearer = 'DEBT' | 'CRED' | 'SHAR';

// ============================================================================
// Configuration
// ============================================================================

export interface ClientConfig {
  apiKey: string;
  environment: 'production' | 'sandbox';
  baseURL?: string;
  timeout?: number;
  retries?: number;
}

// ============================================================================
// Money & Amount
// ============================================================================

export interface Money {
  amount: number;
  currency: Currency;
}

export interface ExchangeRate {
  from: Currency;
  to: Currency;
  rate: number;
  timestamp: Timestamp;
  expiresAt: Timestamp;
}

// ============================================================================
// Party Information
// ============================================================================

export interface Address {
  street: string;
  city: string;
  postalCode: string;
  country: Country;
}

export interface Identification {
  type: 'PASSPORT' | 'NATIONAL_ID' | 'TAX_ID' | 'DRIVERS_LICENSE';
  number: string;
  country: Country;
}

export interface Party {
  name: string;
  identification?: Identification;
  address?: Address;
  country: Country;
}

// ============================================================================
// Beneficiary
// ============================================================================

export interface CreateBeneficiaryRequest {
  name: string;
  country: Country;
  currency: Currency;
  accountNumber: string;
  bankCode: string;
  accountType: 'SAVINGS' | 'CHECKING';
  address: Address;
}

export interface Beneficiary {
  id: string;
  name: string;
  country: Country;
  currency: Currency;
  accountNumber: string;
  bankCode: string;
  accountType: 'SAVINGS' | 'CHECKING';
  address: Address;
  status: 'VERIFIED' | 'PENDING_VERIFICATION';
  createdAt: Timestamp;
  updatedAt: Timestamp;
}

// ============================================================================
// Payment
// ============================================================================

export interface CreatePaymentRequest {
  beneficiaryId: string;
  amount: number;
  currency: Currency;
  purpose: string;
  reference?: string;
  method?: PaymentMethod;
  rateLockId?: string;
  metadata?: Record<string, any>;
}

export interface Fee {
  type: 'TRANSACTION_FEE' | 'FX_MARKUP' | 'NETWORK_FEE';
  amount: number;
  currency: Currency;
}

export interface Payment {
  id: string;
  beneficiaryId: string;
  status: PaymentStatus;
  sender: Party;
  recipient: Party;
  sourceAmount: Money;
  destinationAmount: Money;
  exchangeRate: number;
  fees: Fee[];
  method: PaymentMethod;
  purpose: string;
  reference?: string;
  transactionReference?: string;
  estimatedDelivery?: Timestamp;
  createdAt: Timestamp;
  updatedAt: Timestamp;
  completedAt?: Timestamp;
  failureReason?: string;
  metadata?: Record<string, any>;
}

// ============================================================================
// Quote
// ============================================================================

export interface QuoteRequest {
  from: Currency;
  to: Currency;
  amount: number;
  method?: PaymentMethod;
}

export interface Quote {
  from: Currency;
  to: Currency;
  sourceAmount: number;
  destinationAmount: number;
  rate: number;
  fees: Fee[];
  totalCost: number;
  estimatedDelivery: Timestamp;
  method: PaymentMethod;
  expiresAt: Timestamp;
}

// ============================================================================
// Batch Payment
// ============================================================================

export interface BatchPaymentRequest {
  payments: CreatePaymentRequest[];
  method: PaymentMethod;
  scheduledDate?: string; // ISO 8601 date
}

export interface BatchPayment {
  batchId: string;
  status: PaymentStatus;
  paymentCount: number;
  totalAmount: number;
  currency: Currency;
  successCount: number;
  failedCount: number;
  payments: Array<{
    id: string;
    status: PaymentStatus;
  }>;
  createdAt: Timestamp;
}

// ============================================================================
// FX Rate Lock
// ============================================================================

export interface RateLockRequest {
  from: Currency;
  to: Currency;
  amount: number;
  duration: number; // seconds
}

export interface RateLock {
  rateLockId: string;
  from: Currency;
  to: Currency;
  rate: number;
  amount: number;
  expiresAt: Timestamp;
}

// ============================================================================
// Compliance
// ============================================================================

export interface ComplianceCheckRequest {
  sender: Party;
  recipient: Party;
  amount: number;
  currency: Currency;
}

export interface RiskFactor {
  type: 'COUNTRY_RISK' | 'AMOUNT_RISK' | 'VELOCITY_RISK' | 'PATTERN_RISK';
  score: number;
  description: string;
}

export interface ComplianceCheck {
  approved: boolean;
  riskScore: number;
  riskLevel: RiskLevel;
  factors: RiskFactor[];
  recommendedAction: 'APPROVE' | 'REVIEW' | 'REJECT';
  reasons?: string[];
}

// ============================================================================
// Webhook
// ============================================================================

export interface WebhookConfig {
  url: string;
  events: string[];
  secret: string;
}

export interface WebhookEvent {
  id: string;
  type: string;
  timestamp: Timestamp;
  data: any;
}

// ============================================================================
// Corridors & Routes
// ============================================================================

export interface PaymentCorridor {
  from: Country;
  to: Country;
  supportedMethods: PaymentMethod[];
  averageTime: number; // seconds
  averageFee: number;
  successRate: number;
}

export interface PaymentRoute {
  method: PaymentMethod;
  estimatedTime: number; // seconds
  fee: number;
  successProbability: number;
  score: number;
}

// ============================================================================
// Error Types
// ============================================================================

export class WiaApiError extends Error {
  constructor(
    public code: string,
    public statusCode: number,
    message: string,
    public details?: any
  ) {
    super(message);
    this.name = 'WiaApiError';
  }
}

// ============================================================================
// API Response Types
// ============================================================================

export interface ApiResponse<T> {
  data: T;
  meta?: {
    requestId: string;
    timestamp: Timestamp;
  };
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
