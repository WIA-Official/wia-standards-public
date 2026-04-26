/**
 * WIA-UNI-013 Currency Integration SDK - Type Definitions
 *
 * @module @wia/currency-integration-sdk/types
 * @version 1.0.0
 */

// ============================================================================
// Core Types
// ============================================================================

export type Currency = 'KRW' | 'KPW' | 'USD' | 'EUR' | 'CNY' | 'JPY';
export type Country = 'KR' | 'KP' | string;
export type Timestamp = string; // ISO 8601 timestamp

export type RateType = 'OFFICIAL' | 'MARKET' | 'MANAGED_FLOAT' | 'PEGGED';
export type TransactionType = 'INDIVIDUAL_TRANSFER' | 'BUSINESS_PAYMENT' | 'TRADE_SETTLEMENT' | 'REMITTANCE';
export type Priority = 'INSTANT' | 'STANDARD' | 'BULK' | 'SCHEDULED';
export type SettlementMethod = 'RTGS' | 'DNS' | 'BLOCKCHAIN';
export type ConversionStatus = 'COMPLETED' | 'PENDING' | 'PROCESSING' | 'FAILED';

export type WalletType = 'PERSONAL' | 'BUSINESS' | 'GOVERNMENT';
export type CBDCTransactionType = 'TRANSFER' | 'PAYMENT' | 'ISSUANCE' | 'REDEMPTION';

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

// ============================================================================
// Exchange Rate Types
// ============================================================================

export interface ExchangeRate {
  rateId: string;
  effectiveDate: Timestamp;
  expiryDate?: Timestamp;
  baseCurrency: Currency;
  targetCurrency: Currency;
  rate: number;
  rateType: RateType;
  source: string;
  confidence?: 'HIGH' | 'MEDIUM' | 'LOW';
  volatilityIndex?: number;
}

export interface GetExchangeRateRequest {
  baseCurrency: Currency;
  targetCurrency: Currency;
}

export interface GetHistoricalRatesRequest {
  baseCurrency: Currency;
  targetCurrency: Currency;
  startDate: string; // YYYY-MM-DD
  endDate: string;   // YYYY-MM-DD
}

export interface HistoricalRate {
  date: string;
  rate: number;
  rateId: string;
}

// ============================================================================
// Party Information
// ============================================================================

export interface Party {
  id: string;
  name: string;
  country: Country;
  accountNumber: string;
  email?: string;
  phone?: string;
}

// ============================================================================
// Currency Conversion Types
// ============================================================================

export interface ConvertCurrencyRequest {
  fromCurrency: Currency;
  toCurrency: Currency;
  amount: number;
  transactionType: TransactionType;
  priority: Priority;
  sender: Party;
  recipient: Party;
  metadata?: Record<string, any>;
}

export interface ConversionFees {
  conversionFee: number;
  currency: Currency;
  feePercentage: number;
}

export interface Settlement {
  method: SettlementMethod;
  settlementTime: string;
  confirmationNumber: string;
}

export interface CurrencyConversion {
  conversionId: string;
  status: ConversionStatus;
  processedAt?: Timestamp;
  exchangeRate: number;
  convertedAmount: number;
  fees: ConversionFees;
  settlement?: Settlement;
}

export interface GetConversionStatusRequest {
  conversionId: string;
}

// ============================================================================
// CBDC Types
// ============================================================================

export interface CreateWalletRequest {
  walletType: WalletType;
  ownerInfo: {
    name: string;
    idNumber: string;
    country: Country;
  };
  initialBalance?: number;
}

export interface CBDCWallet {
  walletId: string;
  walletType: WalletType;
  balance: number;
  createdAt: Timestamp;
  ownerInfo: {
    name: string;
    country: Country;
  };
}

export interface TransferCBDCRequest {
  fromWalletId: string;
  toWalletId: string;
  amount: number;
  purpose?: string;
  metadata?: Record<string, any>;
}

export interface CBDCTransaction {
  transactionId: string;
  timestamp: Timestamp;
  cbdcType: string;
  transactionType: CBDCTransactionType;
  amount: number;
  from: {
    walletId: string;
    walletType: WalletType;
    balance?: number;
  };
  to: {
    walletId: string;
    walletType: WalletType;
  };
  metadata?: Record<string, any>;
  signature?: string;
  blockchainHash?: string;
  status: 'COMPLETED' | 'PENDING' | 'FAILED';
}

// ============================================================================
// Analytics Types
// ============================================================================

export interface TransactionSummaryRequest {
  startDate: string; // YYYY-MM-DD
  endDate: string;   // YYYY-MM-DD
  currency?: Currency | 'ALL';
}

export interface TransactionSummary {
  totalVolume: number;
  totalTransactions: number;
  averageTransactionSize: number;
  conversionsByType: Record<TransactionType, number>;
  dailyBreakdown: Array<{
    date: string;
    volume: number;
    transactions: number;
  }>;
}

// ============================================================================
// API Response
// ============================================================================

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  timestamp: Timestamp;
}

export interface PaginatedResponse<T> {
  success: boolean;
  data: T[];
  pagination: {
    page: number;
    pageSize: number;
    totalPages: number;
    totalItems: number;
  };
  timestamp: Timestamp;
}

// ============================================================================
// Error Types
// ============================================================================

export class CurrencyIntegrationError extends Error {
  constructor(
    message: string,
    public code: string,
    public details?: any
  ) {
    super(message);
    this.name = 'CurrencyIntegrationError';
  }
}

export class AuthenticationError extends CurrencyIntegrationError {
  constructor(message: string = 'Authentication failed', details?: any) {
    super(message, 'AUTHENTICATION_ERROR', details);
    this.name = 'AuthenticationError';
  }
}

export class ValidationError extends CurrencyIntegrationError {
  constructor(message: string = 'Validation failed', details?: any) {
    super(message, 'VALIDATION_ERROR', details);
    this.name = 'ValidationError';
  }
}

export class NetworkError extends CurrencyIntegrationError {
  constructor(message: string = 'Network error occurred', details?: any) {
    super(message, 'NETWORK_ERROR', details);
    this.name = 'NetworkError';
  }
}
