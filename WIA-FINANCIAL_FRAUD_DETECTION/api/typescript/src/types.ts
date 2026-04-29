/**
 * WIA-FINANCIAL_FRAUD_DETECTION - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @standard WIA-FINANCIAL_FRAUD_DETECTION
 * @organization WIA (World Certification Industry Association)
 */

// ============================================================================
// Transaction Types
// ============================================================================

/**
 * Transaction type enumeration
 */
export enum TransactionType {
  PURCHASE = 'purchase',
  AUTHORIZATION = 'authorization',
  CAPTURE = 'capture',
  REFUND = 'refund',
  CHARGEBACK = 'chargeback',
  WITHDRAWAL = 'withdrawal',
  DEPOSIT = 'deposit',
}

/**
 * Card brand enumeration
 */
export enum CardBrand {
  VISA = 'visa',
  MASTERCARD = 'mastercard',
  AMEX = 'amex',
  DISCOVER = 'discover',
  OTHER = 'other',
}

/**
 * Card funding type
 */
export enum CardFunding {
  CREDIT = 'credit',
  DEBIT = 'debit',
  PREPAID = 'prepaid',
  UNKNOWN = 'unknown',
}

/**
 * Wallet provider enumeration
 */
export enum WalletProvider {
  PAYPAL = 'paypal',
  APPLE_PAY = 'apple_pay',
  GOOGLE_PAY = 'google_pay',
  VENMO = 'venmo',
  OTHER = 'other',
}

// ============================================================================
// Address Interface
// ============================================================================

/**
 * Physical address
 */
export interface Address {
  /** Street address line 1 */
  line1: string;
  /** Street address line 2 (optional) */
  line2?: string;
  /** City */
  city: string;
  /** State/Province/Region (optional) */
  state?: string;
  /** Postal code */
  postal_code: string;
  /** ISO 3166-1 alpha-2 country code */
  country: string;
  /** Latitude (optional) */
  latitude?: number;
  /** Longitude (optional) */
  longitude?: number;
}

// ============================================================================
// Payment Method Interfaces
// ============================================================================

/**
 * 3D Secure authentication details
 */
export interface ThreeDSecure {
  /** 3DS version (1.0 or 2.0) */
  version: '1.0' | '2.0';
  /** Authentication successful */
  authenticated: boolean;
  /** Electronic Commerce Indicator */
  eci: string;
  /** Cardholder Authentication Verification Value (optional) */
  cavv?: string;
}

/**
 * Card payment method
 */
export interface CardPayment {
  type: 'card';
  /** First 6 digits of card (BIN) */
  card_bin: string;
  /** Last 4 digits of card */
  card_last4: string;
  /** Card brand */
  card_brand: CardBrand;
  /** Card funding type */
  card_funding: CardFunding;
  /** ISO 3166-1 alpha-2 country code */
  card_country: string;
  /** Cardholder name (optional) */
  cardholder_name?: string;
  /** Expiry month (1-12) */
  expiry_month?: number;
  /** Expiry year (4-digit) */
  expiry_year?: number;
  /** CVV was provided (never store actual CVV) */
  cvv_provided: boolean;
  /** 3D Secure authentication (optional) */
  three_d_secure?: ThreeDSecure;
}

/**
 * Bank transfer payment method
 */
export interface BankPayment {
  type: 'bank_transfer' | 'ach' | 'wire';
  /** Account type */
  account_type: 'checking' | 'savings';
  /** Routing number (optional, masked) */
  routing_number?: string;
  /** Last 4 digits of account number */
  account_last4?: string;
  /** Bank name (optional) */
  bank_name?: string;
  /** ISO 3166-1 alpha-2 country code */
  bank_country: string;
}

/**
 * Digital wallet payment method
 */
export interface WalletPayment {
  type: 'wallet';
  /** Wallet provider */
  wallet_provider: WalletProvider;
  /** Wallet identifier */
  wallet_id: string;
  /** Email associated with wallet (optional) */
  wallet_email?: string;
}

/**
 * Union type for all payment methods
 */
export type PaymentMethod = CardPayment | BankPayment | WalletPayment;

// ============================================================================
// Device & Session Interfaces
// ============================================================================

/**
 * IP geolocation information
 */
export interface IpGeolocation {
  /** ISO 3166-1 alpha-2 country code */
  country: string;
  /** Region/State */
  region: string;
  /** City */
  city: string;
  /** Latitude */
  latitude: number;
  /** Longitude */
  longitude: number;
  /** Internet Service Provider */
  isp: string;
}

/**
 * Device information
 */
export interface DeviceInfo {
  /** Unique device fingerprint */
  fingerprint: string;
  /** IP address */
  ip_address: string;
  /** IP geolocation (optional) */
  ip_geolocation?: IpGeolocation;
  /** User agent string */
  user_agent: string;
  /** Accept-Language header */
  accept_language: string;
  /** Screen resolution (e.g., "1920x1080") */
  screen_resolution?: string;
  /** Timezone offset in minutes from UTC */
  timezone_offset: number;
  /** Platform (iOS, Android, Windows, macOS, etc.) */
  platform?: string;
  /** Browser (Chrome, Safari, Firefox, etc.) */
  browser?: string;
  /** VPN detected */
  is_vpn: boolean;
  /** Proxy detected */
  is_proxy: boolean;
  /** Tor network detected */
  is_tor: boolean;
}

/**
 * Session information
 */
export interface SessionInfo {
  /** Session identifier */
  id: string;
  /** Session start timestamp (ISO 8601) */
  started_at: string;
  /** Session duration in seconds */
  duration_seconds: number;
  /** Number of pages viewed */
  pages_viewed: number;
  /** Referrer URL (optional) */
  referrer?: string;
  /** UTM source (optional) */
  utm_source?: string;
  /** UTM medium (optional) */
  utm_medium?: string;
  /** UTM campaign (optional) */
  utm_campaign?: string;
}

// ============================================================================
// Customer & Merchant Interfaces
// ============================================================================

/**
 * Customer information
 */
export interface Customer {
  /** Unique customer identifier */
  id: string;
  /** Email address */
  email: string;
  /** Phone number (optional, E.164 format) */
  phone?: string;
  /** Full name (optional) */
  name?: string;
  /** Account creation timestamp (ISO 8601) */
  account_created_at: string;
  /** Account age in days */
  account_age_days: number;
  /** Loyalty tier (optional) */
  loyalty_tier?: 'bronze' | 'silver' | 'gold' | 'platinum';
  /** Total lifetime value in USD (optional) */
  total_lifetime_value?: number;
}

/**
 * Merchant information
 */
export interface Merchant {
  /** Unique merchant identifier */
  id: string;
  /** Merchant name */
  name: string;
  /** Merchant Category Code (4-digit) */
  mcc: string;
  /** ISO 3166-1 alpha-2 country code */
  country: string;
  /** Merchant website URL (optional) */
  url?: string;
  /** Merchant phone (optional) */
  phone?: string;
}

// ============================================================================
// Transaction Interface
// ============================================================================

/**
 * Complete transaction object for fraud analysis
 */
export interface Transaction {
  /** Unique transaction identifier */
  id: string;
  /** Merchant identifier */
  merchant_id: string;
  /** Customer identifier */
  customer_id: string;

  // Financial details
  /** Transaction amount */
  amount: number;
  /** ISO 4217 currency code (USD, EUR, etc.) */
  currency: string;
  /** Transaction timestamp (ISO 8601) */
  timestamp: string;
  /** Transaction type */
  type: TransactionType;

  // Related entities
  /** Merchant information */
  merchant: Merchant;
  /** Customer information */
  customer: Customer;
  /** Payment method */
  payment_method: PaymentMethod;

  // Addresses
  /** Shipping address (optional) */
  shipping_address?: Address;
  /** Billing address (optional) */
  billing_address?: Address;

  // Device & session
  /** Device information */
  device: DeviceInfo;
  /** Session information (optional) */
  session?: SessionInfo;

  // Additional metadata
  /** Custom metadata (optional) */
  metadata?: Record<string, any>;
}

// ============================================================================
// Fraud Assessment Types
// ============================================================================

/**
 * Risk level enumeration
 */
export enum RiskLevel {
  VERY_LOW = 'very_low',
  LOW = 'low',
  MEDIUM = 'medium',
  HIGH = 'high',
  CRITICAL = 'critical',
}

/**
 * Decision enumeration
 */
export enum Decision {
  APPROVE = 'approve',
  CHALLENGE = 'challenge',
  REVIEW = 'review',
  BLOCK = 'block',
}

/**
 * Rule evaluation result
 */
export interface RuleEvaluation {
  /** Rule identifier */
  rule_id: string;
  /** Rule name */
  rule_name: string;
  /** Rule was triggered */
  triggered: boolean;
  /** Evaluation details */
  details: string;
  /** Confidence score (optional, 0-1) */
  confidence?: number;
}

/**
 * Feature importance for explainability
 */
export interface FeatureImportance {
  /** Feature name */
  feature: string;
  /** Importance score (0-1) */
  importance: number;
  /** Actual feature value */
  value: any;
  /** Contribution direction */
  contribution: 'increases_risk' | 'decreases_risk';
}

/**
 * Recommended action
 */
export interface RecommendedAction {
  /** Recommended action */
  action: Decision;
  /** Confidence in recommendation (0-1) */
  confidence: number;
  /** Reasoning for recommendation */
  reasoning: string;
  /** Alternative actions (optional) */
  alternative_actions?: {
    action: Decision;
    confidence: number;
  }[];
}

/**
 * Model scores from ensemble
 */
export interface ModelScores {
  /** XGBoost model score */
  xgboost: number;
  /** Random Forest model score */
  random_forest: number;
  /** Deep Learning model score */
  deep_learning: number;
  /** Isolation Forest anomaly score */
  isolation_forest: number;
  /** Autoencoder reconstruction error */
  autoencoder: number;
}

/**
 * Fraud assessment result
 */
export interface FraudAssessment {
  /** Transaction identifier */
  transaction_id: string;
  /** Risk score (0-1, higher = more risky) */
  risk_score: number;
  /** Risk level category */
  risk_level: RiskLevel;
  /** Final decision */
  decision: Decision;
  /** Reasons for decision */
  decision_reasons: string[];

  /** Individual model scores */
  model_scores: ModelScores;
  /** Rule evaluations */
  rule_evaluations: RuleEvaluation[];
  /** Top feature importance (for explainability) */
  feature_importance: FeatureImportance[];
  /** Recommended actions */
  recommended_actions: RecommendedAction[];

  /** Analysis timestamp (ISO 8601) */
  analyzed_at: string;
  /** Model version used */
  model_version: string;
}

// ============================================================================
// Feedback Types
// ============================================================================

/**
 * Feedback type enumeration
 */
export enum FeedbackType {
  CONFIRMED_FRAUD = 'confirmed_fraud',
  FALSE_POSITIVE = 'false_positive',
  TRUE_NEGATIVE = 'true_negative',
  FALSE_NEGATIVE = 'false_negative',
}

/**
 * Fraud type enumeration
 */
export enum FraudType {
  STOLEN_CARD = 'stolen_card',
  ACCOUNT_TAKEOVER = 'account_takeover',
  SYNTHETIC_IDENTITY = 'synthetic_identity',
  FRIENDLY_FRAUD = 'friendly_fraud',
  REFUND_FRAUD = 'refund_fraud',
  CARD_TESTING = 'card_testing',
  MONEY_LAUNDERING = 'money_laundering',
  OTHER = 'other',
}

/**
 * Fraud feedback evidence
 */
export interface FraudFeedbackEvidence {
  /** Chargeback identifier (optional) */
  chargeback_id?: string;
  /** Chargeback reason code (optional) */
  chargeback_reason?: string;
  /** Customer confirmation (optional) */
  customer_confirmation?: boolean;
  /** Law enforcement report (optional) */
  law_enforcement_report?: string;
}

/**
 * Fraud feedback submission
 */
export interface FraudFeedback {
  /** Feedback identifier (generated) */
  feedback_id?: string;
  /** Transaction identifier */
  transaction_id: string;
  /** Feedback type */
  feedback_type: FeedbackType;
  /** Fraud type (optional, required if confirmed_fraud) */
  fraud_type?: FraudType;
  /** Additional notes (optional) */
  notes?: string;
  /** Submitted by (user ID or agent ID) */
  submitted_by: string;
  /** Submission timestamp (ISO 8601, generated) */
  submitted_at?: string;
  /** Supporting evidence (optional) */
  evidence?: FraudFeedbackEvidence;
}

// ============================================================================
// API Response Types
// ============================================================================

/**
 * API response metadata
 */
export interface ApiMeta {
  /** Request identifier for tracing */
  request_id: string;
  /** Response timestamp (ISO 8601) */
  timestamp: string;
  /** Processing time in milliseconds */
  processing_time_ms: number;
  /** Model version (optional) */
  model_version?: string;
}

/**
 * API error object
 */
export interface ApiError {
  /** Error code */
  code: string;
  /** Human-readable error message */
  message: string;
  /** Field that caused the error (optional) */
  field?: string;
  /** Documentation URL (optional) */
  docs_url?: string;
}

/**
 * Generic API response wrapper
 */
export interface ApiResponse<T> {
  /** Response status */
  status: 'success' | 'error';
  /** Response data (only present if status=success) */
  data?: T;
  /** Response metadata */
  meta: ApiMeta;
  /** Errors (only present if status=error) */
  errors?: ApiError[];
}

/**
 * Fraud analysis response
 */
export interface FraudAnalysisResponse {
  /** Fraud assessment */
  fraud_assessment: FraudAssessment;
}

/**
 * Feedback submission response
 */
export interface FeedbackSubmissionResponse {
  /** Feedback identifier */
  feedback_id: string;
  /** Transaction identifier */
  transaction_id: string;
  /** Feedback accepted */
  feedback_accepted: boolean;
  /** Model update scheduled */
  model_update_scheduled: boolean;
  /** Estimated retrain date (ISO 8601) */
  estimated_retrain_date: string;
}

/**
 * Batch analysis result
 */
export interface BatchAnalysisResult {
  /** Transaction identifier */
  transaction_id: string;
  /** Fraud assessment */
  fraud_assessment: FraudAssessment;
}

/**
 * Batch analysis response
 */
export interface BatchAnalysisResponse {
  /** Individual results */
  results: BatchAnalysisResult[];
  /** Summary statistics */
  summary: {
    /** Total transactions analyzed */
    total_transactions: number;
    /** Number of approved transactions */
    approved: number;
    /** Number of challenged transactions */
    challenged: number;
    /** Number of transactions for review */
    reviewed: number;
    /** Number of blocked transactions */
    blocked: number;
  };
}

// ============================================================================
// Statistics Types
// ============================================================================

/**
 * Date range
 */
export interface DateRange {
  /** Start date (ISO 8601) */
  start: string;
  /** End date (ISO 8601) */
  end: string;
}

/**
 * Country fraud statistics
 */
export interface CountryFraudStats {
  /** ISO 3166-1 alpha-2 country code */
  country: string;
  /** Fraud count */
  fraud_count: number;
}

/**
 * Fraud statistics
 */
export interface FraudStatistics {
  /** Date range */
  date_range: DateRange;
  /** Total transactions */
  total_transactions: number;
  /** Fraud detected */
  fraud_detected: number;
  /** Fraud rate (0-1) */
  fraud_rate: number;
  /** False positive rate (0-1) */
  false_positive_rate: number;
  /** False negative rate (0-1) */
  false_negative_rate: number;
  /** Chargebacks */
  chargebacks: number;
  /** Chargeback rate (0-1) */
  chargeback_rate: number;
  /** Amount saved in USD */
  amount_saved_usd: number;

  /** Breakdown by decision */
  by_decision: {
    approved: number;
    challenged: number;
    reviewed: number;
    blocked: number;
  };

  /** Breakdown by fraud type */
  by_fraud_type: Record<string, number>;

  /** Top countries by fraud count */
  top_countries_fraud: CountryFraudStats[];
}

/**
 * Statistics response
 */
export interface StatisticsResponse {
  /** Statistics */
  statistics: FraudStatistics;
}

// ============================================================================
// Configuration Types
// ============================================================================

/**
 * Client configuration
 */
export interface ClientConfig {
  /** API key for authentication */
  apiKey: string;
  /** Base URL for API (default: https://api.wia-fraud.io/v1) */
  baseUrl?: string;
  /** Request timeout in milliseconds (default: 10000) */
  timeout?: number;
  /** Maximum retry attempts (default: 3) */
  maxRetries?: number;
  /** Enable debug logging (default: false) */
  debug?: boolean;
}

// ============================================================================
// Exports
// ============================================================================

export default {
  // Enums
  TransactionType,
  CardBrand,
  CardFunding,
  WalletProvider,
  RiskLevel,
  Decision,
  FeedbackType,
  FraudType,
};
