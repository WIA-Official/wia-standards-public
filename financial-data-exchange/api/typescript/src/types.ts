/**
 * WIA-FIN-021 Financial Data Exchange Types
 * Version 2.0
 */

// ============================================================================
// Core Types
// ============================================================================

export interface WIAConfig {
  /** API base URL */
  baseUrl: string;
  /** API version */
  version?: '1.0' | '1.1' | '1.2' | '2.0';
  /** Authentication configuration */
  auth: AuthConfig;
  /** Optional timeout in milliseconds */
  timeout?: number;
  /** Optional custom headers */
  headers?: Record<string, string>;
  /** Enable debug logging */
  debug?: boolean;
}

export interface AuthConfig {
  /** Authentication type */
  type: 'oauth2' | 'jwt' | 'mtls' | 'api-key';
  /** OAuth 2.0 specific configuration */
  oauth2?: OAuth2Config;
  /** JWT specific configuration */
  jwt?: JWTConfig;
  /** mTLS specific configuration */
  mtls?: MTLSConfig;
  /** API Key */
  apiKey?: string;
}

export interface OAuth2Config {
  /** Client ID */
  clientId: string;
  /** Client secret */
  clientSecret: string;
  /** Token endpoint */
  tokenEndpoint: string;
  /** Authorization endpoint */
  authEndpoint: string;
  /** Redirect URI */
  redirectUri: string;
  /** Requested scopes */
  scopes: string[];
}

export interface JWTConfig {
  /** JWT token */
  token: string;
  /** Token refresh callback */
  refreshToken?: () => Promise<string>;
}

export interface MTLSConfig {
  /** Client certificate */
  cert: string;
  /** Client private key */
  key: string;
  /** CA certificate */
  ca: string;
}

// ============================================================================
// Account Types
// ============================================================================

export interface Account {
  /** Unique account identifier */
  id: string;
  /** Customer ID */
  customerId: string;
  /** Account type */
  type: AccountType;
  /** Account currency (ISO 4217) */
  currency: string;
  /** Account balance */
  balance: Balance;
  /** Account status */
  status: AccountStatus;
  /** Account open date (ISO 8601) */
  openDate: string;
  /** Optional account nickname */
  nickname?: string;
  /** Additional metadata */
  metadata?: Record<string, any>;
}

export enum AccountType {
  CHECKING = 'CHECKING',
  SAVINGS = 'SAVINGS',
  INVESTMENT = 'INVESTMENT',
  CREDIT = 'CREDIT',
  LOAN = 'LOAN',
}

export enum AccountStatus {
  ACTIVE = 'ACTIVE',
  SUSPENDED = 'SUSPENDED',
  CLOSED = 'CLOSED',
  PENDING = 'PENDING',
}

export interface Balance {
  /** Available balance */
  available: number;
  /** Current balance */
  current: number;
  /** Pending balance */
  pending: number;
  /** Currency (ISO 4217) */
  currency: string;
}

// ============================================================================
// Transaction Types
// ============================================================================

export interface Transaction {
  /** Unique transaction identifier */
  id: string;
  /** Account ID */
  accountId: string;
  /** Transaction amount */
  amount: number;
  /** Currency (ISO 4217) */
  currency: string;
  /** Transaction type */
  type: TransactionType;
  /** Credit or debit */
  indicator: CreditDebitIndicator;
  /** Transaction status */
  status: TransactionStatus;
  /** Transaction date (ISO 8601) */
  date: string;
  /** Booking date (ISO 8601) */
  bookingDate?: string;
  /** Value date (ISO 8601) */
  valueDate?: string;
  /** Transaction description */
  description?: string;
  /** Counterparty information */
  counterparty?: Counterparty;
  /** Merchant details */
  merchant?: MerchantDetails;
  /** Additional metadata */
  metadata?: Record<string, any>;
}

export enum TransactionType {
  PAYMENT = 'PAYMENT',
  TRANSFER = 'TRANSFER',
  WITHDRAWAL = 'WITHDRAWAL',
  DEPOSIT = 'DEPOSIT',
  FEE = 'FEE',
  INTEREST = 'INTEREST',
  DIVIDEND = 'DIVIDEND',
  OTHER = 'OTHER',
}

export enum CreditDebitIndicator {
  CREDIT = 'CREDIT',
  DEBIT = 'DEBIT',
}

export enum TransactionStatus {
  PENDING = 'PENDING',
  BOOKED = 'BOOKED',
  CANCELLED = 'CANCELLED',
  FAILED = 'FAILED',
}

export interface Counterparty {
  /** Counterparty name */
  name: string;
  /** Counterparty account */
  account?: string;
  /** Counterparty bank */
  bank?: string;
}

export interface MerchantDetails {
  /** Merchant name */
  name: string;
  /** Merchant category code */
  categoryCode?: string;
  /** Merchant location */
  location?: string;
}

// ============================================================================
// Payment Types
// ============================================================================

export interface PaymentRequest {
  /** Debtor account */
  debtorAccount: string;
  /** Creditor account */
  creditorAccount: string;
  /** Creditor name */
  creditorName: string;
  /** Payment amount */
  amount: number;
  /** Currency (ISO 4217) */
  currency: string;
  /** Payment reference */
  reference?: string;
  /** Remittance information */
  remittanceInformation?: string;
  /** Requested execution date (ISO 8601) */
  executionDate?: string;
  /** Idempotency key */
  idempotencyKey?: string;
}

export interface Payment {
  /** Payment ID */
  id: string;
  /** Payment status */
  status: PaymentStatus;
  /** Debtor account */
  debtorAccount: string;
  /** Creditor account */
  creditorAccount: string;
  /** Payment amount */
  amount: number;
  /** Currency (ISO 4217) */
  currency: string;
  /** Creation timestamp (ISO 8601) */
  createdAt: string;
  /** Execution timestamp (ISO 8601) */
  executedAt?: string;
  /** SCA redirect URL */
  scaRedirectUrl?: string;
  /** SCA status */
  scaStatus?: SCAStatus;
}

export enum PaymentStatus {
  INITIATED = 'INITIATED',
  PENDING = 'PENDING',
  ACCEPTED = 'ACCEPTED',
  REJECTED = 'REJECTED',
  COMPLETED = 'COMPLETED',
  CANCELLED = 'CANCELLED',
  FAILED = 'FAILED',
}

export enum SCAStatus {
  REQUIRED = 'REQUIRED',
  COMPLETED = 'COMPLETED',
  EXEMPTED = 'EXEMPTED',
  FAILED = 'FAILED',
}

// ============================================================================
// Consent Types
// ============================================================================

export interface Consent {
  /** Consent ID */
  id: string;
  /** Customer ID */
  customerId: string;
  /** Third-party provider ID */
  tppId: string;
  /** Permissions granted */
  permissions: Permission[];
  /** Consent status */
  status: ConsentStatus;
  /** Creation date (ISO 8601) */
  createdAt: string;
  /** Expiration date (ISO 8601) */
  expiresAt: string;
  /** Last accessed date (ISO 8601) */
  lastAccessedAt?: string;
}

export enum Permission {
  ACCOUNTS_BASIC_READ = 'accounts:basic:read',
  ACCOUNTS_DETAIL_READ = 'accounts:detail:read',
  ACCOUNTS_BALANCE_READ = 'accounts:balance:read',
  TRANSACTIONS_RECENT_READ = 'transactions:recent:read',
  TRANSACTIONS_HISTORICAL_READ = 'transactions:historical:read',
  PAYMENTS_INITIATE = 'payments:initiate',
}

export enum ConsentStatus {
  ACTIVE = 'ACTIVE',
  EXPIRED = 'EXPIRED',
  REVOKED = 'REVOKED',
}

// ============================================================================
// API Response Types
// ============================================================================

export interface APIResponse<T> {
  /** Response data */
  data: T;
  /** Pagination information */
  pagination?: Pagination;
  /** Response metadata */
  metadata?: ResponseMetadata;
}

export interface Pagination {
  /** Next cursor */
  nextCursor?: string;
  /** Previous cursor */
  prevCursor?: string;
  /** Has more results */
  hasMore: boolean;
  /** Total count (optional, expensive) */
  totalCount?: number;
}

export interface ResponseMetadata {
  /** Request ID */
  requestId: string;
  /** Response timestamp (ISO 8601) */
  timestamp: string;
  /** API version */
  version: string;
}

export interface APIError {
  /** Error code */
  code: string;
  /** Error message */
  message: string;
  /** Error details */
  details?: Record<string, any>;
  /** Is retryable */
  retryable?: boolean;
  /** Retry after (seconds) */
  retryAfter?: number;
}

// ============================================================================
// Webhook Types
// ============================================================================

export interface Webhook {
  /** Webhook ID */
  id: string;
  /** Webhook URL */
  url: string;
  /** Event types */
  events: WebhookEvent[];
  /** Webhook status */
  status: WebhookStatus;
  /** Webhook secret */
  secret: string;
  /** Creation date (ISO 8601) */
  createdAt: string;
}

export enum WebhookEvent {
  TRANSACTION_CREATED = 'transaction.created',
  TRANSACTION_UPDATED = 'transaction.updated',
  PAYMENT_INITIATED = 'payment.initiated',
  PAYMENT_COMPLETED = 'payment.completed',
  PAYMENT_FAILED = 'payment.failed',
  ACCOUNT_UPDATED = 'account.updated',
  BALANCE_UPDATED = 'balance.updated',
  CONSENT_GRANTED = 'consent.granted',
  CONSENT_REVOKED = 'consent.revoked',
}

export enum WebhookStatus {
  ACTIVE = 'ACTIVE',
  INACTIVE = 'INACTIVE',
  FAILED = 'FAILED',
}

export interface WebhookPayload {
  /** Event ID */
  id: string;
  /** Event type */
  type: WebhookEvent;
  /** Event timestamp (ISO 8601) */
  created: string;
  /** Event data */
  data: any;
}

// ============================================================================
// Validation Types
// ============================================================================

export interface ValidationResult {
  /** Is valid */
  valid: boolean;
  /** Validation errors */
  errors?: ValidationError[];
  /** Validation warnings */
  warnings?: ValidationWarning[];
}

export interface ValidationError {
  /** Error code */
  code: string;
  /** Error message */
  message: string;
  /** Field path */
  field?: string;
}

export interface ValidationWarning {
  /** Warning code */
  code: string;
  /** Warning message */
  message: string;
  /** Field path */
  field?: string;
}

// ============================================================================
// AI/ML Types (v2.0+)
// ============================================================================

export interface IntelligentValidationRequest {
  /** Data to validate */
  data: any;
  /** Validation type */
  type: 'transaction' | 'payment' | 'account';
}

export interface IntelligentValidationResult {
  /** Is valid */
  valid: boolean;
  /** AI confidence score (0-1) */
  confidence: number;
  /** Suggestions */
  suggestions?: FieldSuggestion[];
  /** Detected anomalies */
  anomalies?: Anomaly[];
}

export interface FieldSuggestion {
  /** Field name */
  field: string;
  /** Predicted value */
  predictedValue: any;
  /** Confidence (0-1) */
  confidence: number;
}

export interface Anomaly {
  /** Anomaly type */
  type: string;
  /** Severity */
  severity: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  /** Description */
  description: string;
}

export interface FraudScore {
  /** Fraud score (0-1) */
  score: number;
  /** Risk level */
  riskLevel: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  /** Contributing factors */
  factors: FraudFactor[];
  /** Recommended action */
  recommendedAction: 'ALLOW' | 'REVIEW' | 'BLOCK';
}

export interface FraudFactor {
  /** Factor name */
  factor: string;
  /** Factor score (0-1) */
  score: number;
  /** Factor weight */
  weight: number;
  /** Description */
  description?: string;
}

// ============================================================================
// Export all types
// ============================================================================

export * from './types';
