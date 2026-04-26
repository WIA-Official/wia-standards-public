/**
 * WIA-FIN-012 Payment System Standard - TypeScript Type Definitions
 * Version: 1.0.0
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

// ============================================================================
// Card Types
// ============================================================================

export type CardNetwork =
  | 'visa'
  | 'mastercard'
  | 'amex'
  | 'discover'
  | 'diners'
  | 'jcb'
  | 'unionpay'
  | 'maestro'
  | 'unknown';

export type CardFunding = 'credit' | 'debit' | 'prepaid' | 'charge' | 'unknown';

export type CardTier = 'standard' | 'gold' | 'platinum' | 'world-elite' | 'infinite';

export interface Card {
  /** Card network (Visa, Mastercard, etc.) */
  brand: CardNetwork;

  /** Last 4 digits of PAN */
  last4: string;

  /** First 6 digits (BIN/IIN) */
  bin?: string;

  /** Expiration month (1-12) */
  exp_month: number;

  /** Expiration year (4 digits) */
  exp_year: number;

  /** Card fingerprint for deduplication */
  fingerprint: string;

  /** Funding type */
  funding: CardFunding;

  /** Issuing country (ISO 3166-1 alpha-2) */
  country?: string;

  /** Issuing bank name */
  issuer?: string;

  /** Card tier/level */
  tier?: CardTier;
}

export interface CardInput {
  /** Full card number (PAN) */
  number: string;

  /** Expiration month (1-12) */
  exp_month: number;

  /** Expiration year (2 or 4 digits) */
  exp_year: number;

  /** Card verification code (CVV/CVC) */
  cvc: string;

  /** Cardholder name */
  name?: string;
}

// ============================================================================
// Payment Types
// ============================================================================

export type PaymentStatus =
  | 'pending'
  | 'processing'
  | 'requires_action'
  | 'succeeded'
  | 'failed'
  | 'canceled';

export type PaymentMethod = 'card' | 'token' | 'wallet';

export type Currency = 'USD' | 'EUR' | 'GBP' | 'JPY' | 'CNY' | 'KRW' | string;

export interface PaymentIntent {
  /** Unique identifier */
  id: string;

  /** Object type */
  object: 'payment_intent';

  /** Amount in minor units (cents) */
  amount: number;

  /** Amount captured */
  amount_capturable?: number;

  /** Amount captured */
  amount_captured?: number;

  /** Amount refunded */
  amount_refunded?: number;

  /** Currency code (ISO 4217) */
  currency: Currency;

  /** Payment status */
  status: PaymentStatus;

  /** Authorization code (if approved) */
  auth_code?: string;

  /** Payment method ID */
  payment_method_id?: string;

  /** Card details */
  card?: Card;

  /** Merchant ID */
  merchant_id: string;

  /** Order/invoice ID */
  order_id?: string;

  /** Description */
  description?: string;

  /** Statement descriptor (appears on statement) */
  statement_descriptor?: string;

  /** Receipt email */
  receipt_email?: string;

  /** 3D Secure information */
  three_d_secure?: ThreeDSecure;

  /** Risk score (0-100) */
  risk_score?: number;

  /** Network transaction ID */
  network_transaction_id?: string;

  /** Created timestamp */
  created: string;

  /** Last updated timestamp */
  updated?: string;

  /** Expiration timestamp */
  expires_at?: string;

  /** Metadata */
  metadata?: Record<string, any>;
}

export interface CreatePaymentIntentParams {
  /** Amount in minor units */
  amount: number;

  /** Currency code */
  currency: Currency;

  /** Payment method details */
  payment_method?: PaymentMethodInput;

  /** Saved payment method ID */
  payment_method_id?: string;

  /** Merchant ID */
  merchant_id: string;

  /** Order ID */
  order_id?: string;

  /** Description */
  description?: string;

  /** Statement descriptor */
  statement_descriptor?: string;

  /** Receipt email */
  receipt_email?: string;

  /** Billing details */
  billing_details?: BillingDetails;

  /** 3D Secure configuration */
  three_d_secure?: ThreeDSecureConfig;

  /** Metadata */
  metadata?: Record<string, any>;

  /** Auto-capture (default: false) */
  capture?: boolean;
}

export interface PaymentMethodInput {
  /** Payment method type */
  type: PaymentMethod;

  /** Card details (if type = 'card') */
  card?: CardInput;

  /** Token ID (if type = 'token') */
  token?: string;

  /** Wallet details (if type = 'wallet') */
  wallet?: WalletInput;
}

export interface WalletInput {
  /** Wallet type */
  type: 'apple_pay' | 'google_pay' | 'samsung_pay';

  /** Wallet token/cryptogram */
  token: string;
}

// ============================================================================
// Address & Billing Types
// ============================================================================

export interface Address {
  /** Street address line 1 */
  line1?: string;

  /** Street address line 2 */
  line2?: string;

  /** City */
  city?: string;

  /** State/Province */
  state?: string;

  /** Postal/ZIP code */
  postal_code?: string;

  /** Country (ISO 3166-1 alpha-2) */
  country?: string;
}

export interface BillingDetails {
  /** Billing address */
  address?: Address;

  /** Billing email */
  email?: string;

  /** Billing phone */
  phone?: string;

  /** Billing name */
  name?: string;
}

// ============================================================================
// 3D Secure Types
// ============================================================================

export interface ThreeDSecure {
  /** Authentication status */
  authenticated: boolean;

  /** 3DS version (1.0, 2.1, 2.2, etc.) */
  version: string;

  /** Transaction ID */
  transaction_id: string;

  /** Electronic Commerce Indicator */
  eci: string;

  /** CAVV (Cardholder Authentication Verification Value) */
  cavv?: string;

  /** XID (Transaction Identifier) */
  xid?: string;

  /** Enrollment status */
  enrollment?: 'Y' | 'N' | 'U';

  /** Authentication status */
  authentication_status?: 'Y' | 'N' | 'U' | 'A' | 'R';
}

export interface ThreeDSecureConfig {
  /** Enable 3DS */
  enabled: boolean;

  /** Challenge preference */
  challenge_preference?: 'no_preference' | 'no_challenge' | 'challenge_required';

  /** Browser information (for 3DS 2.0) */
  browser_info?: BrowserInfo;
}

export interface BrowserInfo {
  /** User agent string */
  user_agent: string;

  /** Accept header */
  accept_header: string;

  /** Browser language */
  language: string;

  /** Screen width */
  screen_width: number;

  /** Screen height */
  screen_height: number;

  /** Color depth */
  color_depth: number;

  /** Timezone offset (minutes) */
  timezone_offset: number;

  /** JavaScript enabled */
  javascript_enabled: boolean;

  /** Java enabled */
  java_enabled: boolean;
}

// ============================================================================
// Transaction Types
// ============================================================================

export type TransactionType =
  | 'authorization'
  | 'capture'
  | 'charge'
  | 'refund'
  | 'void';

export interface Authorization {
  /** Authorization ID */
  id: string;

  /** Object type */
  object: 'authorization';

  /** Amount authorized */
  amount: number;

  /** Currency */
  currency: Currency;

  /** Status */
  status: PaymentStatus;

  /** Authorization code */
  auth_code: string;

  /** Created timestamp */
  created: string;

  /** Expiration timestamp */
  expires_at: string;

  /** Captured */
  captured: boolean;

  /** Void */
  voided: boolean;
}

export interface Capture {
  /** Capture ID */
  id: string;

  /** Object type */
  object: 'capture';

  /** Authorization ID */
  authorization_id: string;

  /** Amount captured */
  amount: number;

  /** Currency */
  currency: Currency;

  /** Status */
  status: PaymentStatus;

  /** Created timestamp */
  created: string;

  /** Settlement date */
  settlement_date?: string;
}

export interface Refund {
  /** Refund ID */
  id: string;

  /** Object type */
  object: 'refund';

  /** Payment ID */
  payment_id: string;

  /** Amount refunded */
  amount: number;

  /** Currency */
  currency: Currency;

  /** Status */
  status: PaymentStatus;

  /** Reason */
  reason?: 'duplicate' | 'fraudulent' | 'requested_by_customer' | 'other';

  /** Created timestamp */
  created: string;

  /** Estimated arrival */
  estimated_arrival?: string;

  /** Metadata */
  metadata?: Record<string, any>;
}

// ============================================================================
// Token Types
// ============================================================================

export interface Token {
  /** Token ID */
  id: string;

  /** Object type */
  object: 'token';

  /** Token type */
  type: 'card';

  /** Card details */
  card: Card;

  /** Created timestamp */
  created: string;

  /** Live mode */
  livemode: boolean;

  /** Used flag */
  used: boolean;
}

// ============================================================================
// Subscription Types
// ============================================================================

export type SubscriptionStatus =
  | 'active'
  | 'past_due'
  | 'unpaid'
  | 'canceled'
  | 'incomplete'
  | 'incomplete_expired'
  | 'trialing';

export interface Subscription {
  /** Subscription ID */
  id: string;

  /** Object type */
  object: 'subscription';

  /** Customer ID */
  customer_id: string;

  /** Payment method ID */
  payment_method_id: string;

  /** Plan details */
  plan: SubscriptionPlan;

  /** Status */
  status: SubscriptionStatus;

  /** Current period start */
  current_period_start: string;

  /** Current period end */
  current_period_end: string;

  /** Trial end */
  trial_end?: string;

  /** Next billing date */
  next_billing_date?: string;

  /** Canceled at */
  canceled_at?: string;

  /** Created timestamp */
  created: string;
}

export interface SubscriptionPlan {
  /** Amount in minor units */
  amount: number;

  /** Currency */
  currency: Currency;

  /** Billing interval */
  interval: 'day' | 'week' | 'month' | 'year';

  /** Interval count */
  interval_count?: number;
}

// ============================================================================
// Webhook Types
// ============================================================================

export type WebhookEventType =
  | 'payment.succeeded'
  | 'payment.failed'
  | 'payment.refunded'
  | 'payment.disputed'
  | 'subscription.created'
  | 'subscription.updated'
  | 'subscription.deleted'
  | 'invoice.payment_succeeded'
  | 'invoice.payment_failed';

export interface WebhookEvent {
  /** Event ID */
  id: string;

  /** Event type */
  type: WebhookEventType;

  /** Created timestamp */
  created: string;

  /** Event data */
  data: {
    object: any;
  };

  /** Live mode */
  livemode: boolean;
}

// ============================================================================
// API Response Types
// ============================================================================

export interface APIError {
  /** Error object */
  error: {
    /** Error type */
    type: 'api_error' | 'card_error' | 'invalid_request_error' | 'authentication_error';

    /** Error code */
    code?: string;

    /** Error message */
    message: string;

    /** Decline code (for card errors) */
    decline_code?: string;

    /** Parameter that caused error */
    param?: string;
  };
}

export interface ListResponse<T> {
  /** Object type */
  object: 'list';

  /** Data items */
  data: T[];

  /** Has more items */
  has_more: boolean;

  /** Total count */
  total_count?: number;

  /** URL for next page */
  next_page?: string;
}

// ============================================================================
// Configuration Types
// ============================================================================

export interface PaymentConfig {
  /** API key */
  apiKey: string;

  /** Environment */
  environment?: 'production' | 'sandbox';

  /** API version */
  apiVersion?: string;

  /** Timeout (ms) */
  timeout?: number;

  /** Max retries */
  maxRetries?: number;
}

/**
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 *
 * These type definitions enable secure, type-safe payment processing
 * that benefits developers and end-users worldwide.
 */
