/**
 * WIA Wealth Management Standard - Type Definitions
 *
 * @module @wia/wealth-management/types
 * @version 1.0.0
 * @license MIT
 *
 * 弘益人間 · Benefit All Humanity
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * User account information
 */
export interface User {
  /** Unique user identifier */
  id: string;
  /** User's email address */
  email: string;
  /** User's full name */
  fullName: string;
  /** Date of birth (ISO 8601) */
  dateOfBirth: Date;
  /** Country of residence (ISO 3166-1 alpha-2) */
  country: string;
  /** Tax residency countries */
  taxResidency: string[];
  /** Investment risk tolerance */
  riskTolerance: RiskTolerance;
  /** Account creation timestamp */
  createdAt: Date;
  /** Last update timestamp */
  updatedAt: Date;
}

/**
 * Risk tolerance levels
 */
export enum RiskTolerance {
  CONSERVATIVE = 'conservative',
  MODERATE = 'moderate',
  AGGRESSIVE = 'aggressive'
}

/**
 * Asset types supported by the platform
 */
export enum AssetType {
  EQUITY = 'equity',
  BOND = 'bond',
  MUTUAL_FUND = 'mutual_fund',
  ETF = 'etf',
  REAL_ESTATE = 'real_estate',
  CRYPTOCURRENCY = 'cryptocurrency',
  COMMODITY = 'commodity',
  CASH = 'cash',
  OTHER = 'other'
}

/**
 * Individual asset in a portfolio
 */
export interface Asset {
  /** Unique asset identifier */
  id: string;
  /** Owner user ID */
  userId: string;
  /** Portfolio ID this asset belongs to */
  portfolioId: string;
  /** Asset type */
  type: AssetType;
  /** Ticker symbol (if applicable) */
  symbol?: string;
  /** Asset name */
  name: string;
  /** Number of units owned */
  quantity: number;
  /** Total cost basis */
  costBasis: number;
  /** Current market value */
  currentValue: number;
  /** Currency code (ISO 4217) */
  currency: string;
  /** Acquisition date */
  acquiredDate: Date;
  /** Additional metadata */
  metadata: Record<string, any>;
  /** Creation timestamp */
  createdAt: Date;
  /** Last update timestamp */
  updatedAt: Date;
}

/**
 * Portfolio containing multiple assets
 */
export interface Portfolio {
  /** Unique portfolio identifier */
  id: string;
  /** Owner user ID */
  userId: string;
  /** Portfolio name */
  name: string;
  /** Portfolio description */
  description?: string;
  /** Assets in this portfolio */
  assets: Asset[];
  /** Total portfolio value */
  totalValue: number;
  /** Base currency (ISO 4217) */
  currency: string;
  /** Target allocation strategy */
  targetAllocation?: AllocationTarget[];
  /** Portfolio performance metrics */
  performance?: PerformanceMetrics;
  /** Creation timestamp */
  createdAt: Date;
  /** Last update timestamp */
  updatedAt: Date;
}

/**
 * Target allocation for an asset class
 */
export interface AllocationTarget {
  /** Asset type */
  assetType: AssetType;
  /** Target percentage (0-100) */
  targetPercentage: number;
  /** Current percentage (0-100) */
  currentPercentage: number;
  /** Drift from target */
  drift: number;
}

/**
 * Portfolio performance metrics
 */
export interface PerformanceMetrics {
  /** Year-to-date return */
  ytd: number;
  /** 1-month return */
  oneMonth: number;
  /** 3-month return */
  threeMonth: number;
  /** 1-year return */
  oneYear: number;
  /** 3-year annualized return */
  threeYear?: number;
  /** 5-year annualized return */
  fiveYear?: number;
  /** Since inception return */
  sinceInception?: number;
}

// ============================================================================
// Transaction Types
// ============================================================================

/**
 * Transaction types
 */
export enum TransactionType {
  BUY = 'buy',
  SELL = 'sell',
  DIVIDEND = 'dividend',
  INTEREST = 'interest',
  TRANSFER_IN = 'transfer_in',
  TRANSFER_OUT = 'transfer_out',
  FEE = 'fee',
  TAX = 'tax'
}

/**
 * Financial transaction record
 */
export interface Transaction {
  /** Unique transaction identifier */
  id: string;
  /** User ID */
  userId: string;
  /** Portfolio ID */
  portfolioId: string;
  /** Asset ID */
  assetId: string;
  /** Transaction type */
  type: TransactionType;
  /** Quantity of assets */
  quantity: number;
  /** Price per unit */
  price: number;
  /** Total transaction amount */
  totalAmount: number;
  /** Transaction fees */
  fees: number;
  /** Currency code (ISO 4217) */
  currency: string;
  /** Execution timestamp */
  executedAt: Date;
  /** Additional metadata */
  metadata: Record<string, any>;
}

// ============================================================================
// Analytics Types
// ============================================================================

/**
 * Risk metrics for portfolio analysis
 */
export interface RiskMetrics {
  /** Portfolio beta */
  beta: number;
  /** Standard deviation (volatility) */
  volatility: number;
  /** Sharpe ratio */
  sharpeRatio: number;
  /** Maximum drawdown */
  maxDrawdown: number;
  /** Value at Risk (95% confidence) */
  valueAtRisk95: number;
  /** Conditional Value at Risk */
  conditionalVaR: number;
}

/**
 * Performance attribution analysis
 */
export interface PerformanceAttribution {
  /** Total return */
  totalReturn: number;
  /** Asset allocation effect */
  assetAllocation: number;
  /** Security selection effect */
  securitySelection: number;
  /** Currency effect */
  currencyEffect: number;
  /** Fees and expenses */
  feesExpenses: number;
}

/**
 * Portfolio analytics
 */
export interface Analytics {
  /** Portfolio ID */
  portfolioId: string;
  /** Analysis period */
  period: string;
  /** Performance metrics */
  performance: PerformanceMetrics;
  /** Risk metrics */
  risk: RiskMetrics;
  /** Performance attribution */
  attribution?: PerformanceAttribution;
  /** Generated timestamp */
  generatedAt: Date;
}

// ============================================================================
// Tax Types
// ============================================================================

/**
 * Capital gains breakdown
 */
export interface CapitalGains {
  /** Short-term capital gains */
  shortTerm: number;
  /** Long-term capital gains */
  longTerm: number;
  /** Total capital gains */
  total: number;
}

/**
 * Tax-loss harvesting opportunity
 */
export interface TaxLossOpportunity {
  /** Opportunity ID */
  id: string;
  /** Asset ID */
  assetId: string;
  /** Asset symbol */
  symbol: string;
  /** Asset name */
  name: string;
  /** Cost basis */
  costBasis: number;
  /** Current market value */
  currentValue: number;
  /** Unrealized loss */
  unrealizedLoss: number;
  /** Potential tax savings */
  potentialTaxSavings: number;
  /** Wash sale risk indicator */
  washSaleRisk: 'low' | 'medium' | 'high';
  /** Recommendation */
  recommendation: string;
}

/**
 * Annual tax report
 */
export interface TaxReport {
  /** Unique report identifier */
  id: string;
  /** User ID */
  userId: string;
  /** Tax year */
  taxYear: number;
  /** Total income */
  totalIncome: number;
  /** Capital gains */
  capitalGains: CapitalGains;
  /** Dividend income */
  dividendIncome: number;
  /** Interest income */
  interestIncome: number;
  /** Estimated tax liability */
  estimatedTaxLiability: number;
  /** Tax-loss harvesting opportunities */
  taxLossHarvestingOpportunities: TaxLossOpportunity[];
  /** Report generation timestamp */
  generatedAt: Date;
}

// ============================================================================
// Planning Types
// ============================================================================

/**
 * Financial goal
 */
export interface FinancialGoal {
  /** Unique goal identifier */
  id: string;
  /** User ID */
  userId: string;
  /** Goal name */
  name: string;
  /** Goal description */
  description?: string;
  /** Target amount */
  targetAmount: number;
  /** Current amount */
  currentAmount: number;
  /** Target date */
  targetDate: Date;
  /** Monthly contribution needed */
  monthlyContributionNeeded: number;
  /** Progress percentage */
  progress: number;
  /** Goal status */
  status: 'on_track' | 'behind' | 'ahead' | 'completed';
  /** Creation timestamp */
  createdAt: Date;
  /** Last update timestamp */
  updatedAt: Date;
}

/**
 * Retirement planning parameters
 */
export interface RetirementPlan {
  /** Current age */
  currentAge: number;
  /** Target retirement age */
  retirementAge: number;
  /** Years to retirement */
  yearsToRetirement: number;
  /** Current savings */
  currentSavings: number;
  /** Target retirement amount */
  targetAmount: number;
  /** Monthly contribution */
  monthlyContribution: number;
  /** Expected annual return */
  expectedReturn: number;
  /** Inflation rate */
  inflationRate: number;
  /** Projected retirement amount */
  projectedAmount: number;
  /** Shortfall or surplus */
  gap: number;
}

// ============================================================================
// API Types
// ============================================================================

/**
 * SDK configuration
 */
export interface SDKConfig {
  /** API key for authentication */
  apiKey: string;
  /** Environment (development, staging, production) */
  environment?: 'development' | 'staging' | 'production';
  /** Base API URL (optional override) */
  baseUrl?: string;
  /** Request timeout in milliseconds */
  timeout?: number;
}

/**
 * API response wrapper
 */
export interface APIResponse<T> {
  /** Response data */
  data: T;
  /** Success status */
  success: boolean;
  /** Error message (if any) */
  error?: string;
  /** Metadata */
  meta?: {
    /** Request ID */
    requestId: string;
    /** Timestamp */
    timestamp: Date;
    /** Rate limit info */
    rateLimit?: {
      limit: number;
      remaining: number;
      reset: Date;
    };
  };
}

/**
 * Pagination parameters
 */
export interface PaginationParams {
  /** Page number (1-indexed) */
  page?: number;
  /** Items per page */
  limit?: number;
  /** Sort field */
  sortBy?: string;
  /** Sort order */
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  /** Items in current page */
  items: T[];
  /** Total number of items */
  total: number;
  /** Current page */
  page: number;
  /** Items per page */
  limit: number;
  /** Total pages */
  totalPages: number;
  /** Has next page */
  hasNext: boolean;
  /** Has previous page */
  hasPrevious: boolean;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * API error codes
 */
export enum ErrorCode {
  UNAUTHORIZED = 'UNAUTHORIZED',
  FORBIDDEN = 'FORBIDDEN',
  NOT_FOUND = 'NOT_FOUND',
  VALIDATION_ERROR = 'VALIDATION_ERROR',
  RATE_LIMIT_EXCEEDED = 'RATE_LIMIT_EXCEEDED',
  INTERNAL_ERROR = 'INTERNAL_ERROR',
  SERVICE_UNAVAILABLE = 'SERVICE_UNAVAILABLE'
}

/**
 * API error
 */
export class WIAError extends Error {
  constructor(
    public code: ErrorCode,
    message: string,
    public statusCode?: number,
    public details?: any
  ) {
    super(message);
    this.name = 'WIAError';
  }
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Date range filter
 */
export interface DateRange {
  /** Start date */
  start: Date;
  /** End date */
  end: Date;
}

/**
 * Money amount with currency
 */
export interface MoneyAmount {
  /** Amount */
  amount: number;
  /** Currency code (ISO 4217) */
  currency: string;
}

/**
 * Market quote
 */
export interface Quote {
  /** Symbol */
  symbol: string;
  /** Current price */
  price: number;
  /** Price change */
  change: number;
  /** Percentage change */
  changePercent: number;
  /** Volume */
  volume: number;
  /** Market cap */
  marketCap?: number;
  /** Quote timestamp */
  timestamp: Date;
}
