/**
 * WIA-FIN-013 Mobile Payment Standard - TypeScript Type Definitions
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Core Types
// ============================================================================

export interface Amount {
  value: string;
  currency: string;
  minorUnits: number;
}

export interface Timestamp {
  timestamp: string;
  timezone: string;
}

export interface Address {
  line1: string;
  line2?: string;
  city: string;
  state?: string;
  postalCode: string;
  country: string;
}

export interface DeviceInfo {
  deviceId: string;
  platform: 'iOS' | 'Android' | 'Web' | 'Other';
  osVersion: string;
  appVersion: string;
  manufacturer?: string;
  model?: string;
}

// ============================================================================
// Transaction Types
// ============================================================================

export type TransactionStatus =
  | 'initiated'
  | 'authorizing'
  | 'authorized'
  | 'completed'
  | 'failed'
  | 'declined'
  | 'cancelled'
  | 'refunded'
  | 'partially_refunded';

export type PaymentMethod =
  | 'nfc'
  | 'qr_code'
  | 'card'
  | 'wallet'
  | 'bank_transfer'
  | 'crypto';

export interface Transaction {
  version: string;
  standard: 'WIA-FIN-013';
  transactionId: string;
  type: 'payment';
  method: PaymentMethod;
  status: TransactionStatus;
  amount: Amount;
  merchant?: MerchantInfo;
  customer?: CustomerInfo;
  paymentMethod?: PaymentMethodDetails;
  authentication?: Authentication;
  device?: DeviceInfo;
  network?: NetworkInfo;
  cryptogram?: Cryptogram;
  timestamps: {
    initiated: string;
    authorized?: string;
    completed?: string;
  };
  fees?: {
    processing?: Amount;
    service?: Amount;
  };
  metadata?: Record<string, any>;
}

export interface MerchantInfo {
  id: string;
  name: string;
  category?: string;
  location?: {
    latitude: number;
    longitude: number;
    address?: Address;
  };
}

export interface CustomerInfo {
  customerId: string;
  email?: string;
  phone?: string;
}

export interface PaymentMethodDetails {
  type: 'card' | 'bank_account' | 'wallet';
  brand?: string;
  last4?: string;
  expiryMonth?: number;
  expiryYear?: number;
  tokenId?: string;
}

export interface Authentication {
  method: 'biometric' | 'pin' | 'password' | 'otp';
  type?: 'fingerprint' | 'face_id' | 'iris' | 'voice';
  timestamp: string;
  deviceCVM?: boolean;
}

export interface NetworkInfo {
  acquirer?: string;
  processor?: string;
  cardNetwork?: string;
}

export interface Cryptogram {
  type: string;
  value: string;
  transactionCounter?: number;
}

// ============================================================================
// Wallet Types
// ============================================================================

export type WalletStatus = 'active' | 'suspended' | 'closed';

export interface Wallet {
  version: string;
  standard: 'WIA-FIN-013';
  walletId: string;
  userId: string;
  type: 'mobile_wallet' | 'web_wallet' | 'hardware_wallet';
  status: WalletStatus;
  balance: {
    available: Amount;
    pending: Amount;
    reserved: Amount;
  };
  paymentMethods?: PaymentMethodInfo[];
  settings?: WalletSettings;
  limits?: WalletLimits;
  created: string;
  updated: string;
}

export interface PaymentMethodInfo {
  id: string;
  type: 'card' | 'bank_account';
  brand?: string;
  last4: string;
  expiryMonth?: number;
  expiryYear?: number;
  isDefault: boolean;
  tokenId?: string;
  billingAddress?: Address;
}

export interface WalletSettings {
  defaultCurrency: string;
  allowNFC: boolean;
  allowQRCode: boolean;
  biometricEnabled: boolean;
  notificationsEnabled: boolean;
  autoReload?: {
    enabled: boolean;
    threshold: string;
    amount: string;
    sourcePaymentMethodId: string;
  };
}

export interface WalletLimits {
  daily?: {
    transaction?: Amount;
    withdrawal?: Amount;
  };
  monthly?: {
    transaction?: Amount;
  };
}

// ============================================================================
// Token Types
// ============================================================================

export type TokenStatus = 'active' | 'suspended' | 'expired' | 'deleted';
export type TokenType = 'payment_token' | 'network_token';

export interface Token {
  version: string;
  standard: 'WIA-FIN-013';
  tokenId: string;
  type: TokenType;
  status: TokenStatus;
  deviceBinding: {
    deviceId: string;
    deviceFingerprint: string;
    bindingMethod: string;
  };
  cardDetails: {
    tokenPAN: string;
    brand: string;
    last4DigitsRealPAN: string;
    expiryMonth: number;
    expiryYear: number;
    cardholderName?: string;
  };
  tokenization: {
    requestor: string;
    requestorId: string;
    tokenProvider: string;
    tokenReferenceId: string;
  };
  securityCodes?: {
    cvv?: string;
    dynamicCVV?: boolean;
  };
  restrictions?: {
    singleUse?: boolean;
    maxAmount?: Amount;
    allowedMerchantCategories?: string[];
    allowedCountries?: string[];
    expiresAt?: string;
  };
  created: string;
  lastUsed?: string;
}

// ============================================================================
// QR Code Types
// ============================================================================

export type QRCodeType = 'static' | 'dynamic';

export interface QRCode {
  version: string;
  standard: 'WIA-FIN-013';
  qrType: QRCodeType;
  merchantId: string;
  merchantName?: string;
  qrCodeId: string;
  paymentDetails: {
    amount: Amount & { editable: boolean };
    description?: string;
    reference?: string;
    items?: Array<{
      name: string;
      quantity: number;
      unitPrice: string;
      total: string;
    }>;
    tax?: string;
    tip?: string;
    total?: string;
  };
  expiresAt?: string | null;
  created: string;
  maxScans?: number;
  scannedCount?: number;
}

// ============================================================================
// Biometric Types
// ============================================================================

export type BiometricMethod =
  | 'fingerprint'
  | 'face_recognition'
  | 'iris_scan'
  | 'voice_recognition'
  | 'palm_print';

export type BiometricResult = 'success' | 'failure' | 'retry';

export interface BiometricAuthentication {
  version: string;
  standard: 'WIA-FIN-013';
  authenticationId: string;
  type: 'biometric';
  method: BiometricMethod;
  result: BiometricResult;
  confidence?: number;
  deviceInfo: {
    deviceId: string;
    biometricType: string;
    sensorVersion?: string;
  };
  biometricData: {
    templateHash: string;
    encryptedData: string;
    algorithm: string;
  };
  liveness?: {
    detected: boolean;
    score: number;
  };
  timestamp: string;
  expiresAt?: string;
}

// ============================================================================
// P2P Transfer Types
// ============================================================================

export interface P2PTransfer {
  version: string;
  standard: 'WIA-FIN-013';
  transferId: string;
  type: 'p2p_transfer';
  status: TransactionStatus;
  sender: {
    userId: string;
    walletId: string;
    name?: string;
    email?: string;
    phone?: string;
  };
  recipient: {
    userId: string;
    walletId: string;
    name?: string;
    email?: string;
    phone?: string;
    verificationStatus?: string;
  };
  amount: Amount;
  fees?: {
    sender?: Amount;
    recipient?: Amount;
  };
  message?: string;
  memo?: string;
  timestamps: {
    initiated: string;
    completed?: string;
  };
  metadata?: Record<string, any>;
}

// ============================================================================
// Error Types
// ============================================================================

export type ErrorType =
  | 'validation_error'
  | 'authentication_error'
  | 'payment_error'
  | 'network_error'
  | 'system_error'
  | 'security_error';

export type ErrorCode =
  | 'invalid_amount'
  | 'invalid_currency'
  | 'insufficient_funds'
  | 'card_declined'
  | 'expired_token'
  | 'biometric_failed'
  | 'rate_limit_exceeded'
  | 'unknown_error';

export interface WIAError {
  version: string;
  standard: 'WIA-FIN-013';
  error: {
    code: ErrorCode | string;
    message: string;
    type: ErrorType;
    details?: Record<string, any>;
    timestamp: string;
    requestId?: string;
    documentation?: string;
  };
}

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIAConfig {
  apiKey: string;
  environment?: 'production' | 'test' | 'sandbox';
  baseUrl?: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// API Response Types
// ============================================================================

export interface WIAResponse<T> {
  success: boolean;
  data?: T;
  error?: WIAError['error'];
  metadata?: {
    requestId: string;
    timestamp: string;
    version: string;
  };
}

// ============================================================================
// Request Types
// ============================================================================

export interface ProcessPaymentRequest {
  amount: number | Amount;
  currency?: string;
  method: PaymentMethod;
  merchantId: string;
  customerId?: string;
  deviceId?: string;
  tokenId?: string;
  metadata?: Record<string, any>;
}

export interface CreateWalletRequest {
  userId: string;
  type?: 'mobile_wallet' | 'web_wallet';
  currency?: string;
  initialBalance?: Amount;
}

export interface AddPaymentMethodRequest {
  walletId: string;
  type: 'card' | 'bank_account';
  cardNumber?: string;
  expiryMonth?: number;
  expiryYear?: number;
  cvv?: string;
  accountNumber?: string;
  routingNumber?: string;
  billingAddress?: Address;
}

export interface GenerateQRCodeRequest {
  merchantId: string;
  amount?: number | Amount;
  currency?: string;
  type?: QRCodeType;
  description?: string;
  expiresIn?: number; // seconds
}

export interface BiometricAuthRequest {
  method?: BiometricMethod;
  allowFallback?: boolean;
}
