// WIA-FINANCIAL_INCLUSION TypeScript SDK type definitions
// Version: 1.0.0
// 弘益人間 (Benefit All Humanity)
//
// All references conform to the WIA Citation & Veracity Policy v1.0 §2.1 ALLOW.
// Aligned with: ISO 20022 (financial-services messaging), ISO 4217 (currency codes),
//              ISO 13616 (IBAN), ISO 8601:2019 (date/time), RFC 9562 (UUID v4),
//              W3C DID Core 1.0, W3C VC Data Model 2.0.

export type CurrencyCode = string;       // ISO 4217 currency code (3-letter)
export type IBAN = string;               // ISO 13616 IBAN
export type ISO8601 = string;            // ISO 8601:2019 date-time
export type UUID = string;               // RFC 9562 UUID
export type DID = string;                // W3C DID Core 1.0 DID URL

export interface AccountIdentifier {
  scheme: 'iban' | 'wallet-address' | 'mobile-money' | 'wia-id';
  value: string;
  bic?: string;                          // Bank Identifier Code per ISO 9362 (when scheme is iban)
}

export interface MoneyAmount {
  currency: CurrencyCode;                // ISO 4217
  amount_minor_units: bigint;            // amount in minor units (e.g., cents)
  decimal_places: number;                // ISO 4217 minor-unit count (e.g., 2 for USD, 0 for JPY)
}

export interface Party {
  did?: DID;                             // W3C DID Core 1.0 identifier
  display_name?: string;
  jurisdiction_iso3166?: string;         // ISO 3166-1 alpha-2 / alpha-3 country code
  verifiable_credentials?: VerifiableCredentialReference[];
}

export interface VerifiableCredentialReference {
  // W3C VC Data Model 2.0
  vc_uri: string;
  vc_hash_sha256?: string;
}

export interface TransactionRecord {
  type: 'WIA-FINANCIAL_INCLUSIONRecord';
  version: string;                       // semantic version
  id: UUID;                              // RFC 9562
  timestamp_iso8601: ISO8601;
  origin: Party;
  destination: Party;
  origin_account?: AccountIdentifier;
  destination_account?: AccountIdentifier;
  amount: MoneyAmount;
  category: TransactionCategory;
  metadata?: Record<string, unknown>;
  signature?: Signature;
}

export type TransactionCategory =
  | 'remittance'
  | 'micropayment'
  | 'savings-deposit'
  | 'savings-withdrawal'
  | 'loan-disbursement'
  | 'loan-repayment'
  | 'merchant-payment'
  | 'salary'
  | 'aid-distribution'
  | 'other';

export interface Signature {
  alg: 'EdDSA' | 'ES256' | 'ES384' | 'PS256';   // RFC 8032, NIST FIPS 186-5
  kid: string;                                   // key identifier (DID URL or X.509 SKI)
  sig_base64: string;
}

export interface ValidationResult {
  ok: boolean;
  errors: ValidationError[];
  warnings: ValidationWarning[];
}

export interface ValidationError {
  code: string;
  field?: string;
  message: string;
}

export interface ValidationWarning {
  code: string;
  field?: string;
  message: string;
}

export interface ListOptions {
  cursor?: string;
  limit?: number;                        // 1..1000
  filter?: Record<string, string>;
}

export interface RecordList {
  items: TransactionRecord[];
  next_cursor?: string;
  total_estimate?: number;
}

export interface HealthStatus {
  status: 'ok' | 'degraded' | 'down';
  version: string;
  uptime_seconds: number;
  checks: Record<string, 'ok' | 'fail'>;
}
