/**
 * WIA-FIN-016 Robo-Advisor Standard - TypeScript Type Definitions
 * Version: 1.0.0
 *
 * © 2025 WIA (World Certification Industry Association)
 * 弘益人間 · Benefit All Humanity
 */

// Enums
export enum AccountType {
  TAXABLE = 'taxable',
  TRADITIONAL_IRA = 'traditional_ira',
  ROTH_IRA = 'roth_ira',
  K401 = '401k'
}

export enum PortfolioStatus {
  ACTIVE = 'active',
  INACTIVE = 'inactive',
  CLOSED = 'closed'
}

export enum RiskLevel {
  CONSERVATIVE = 'conservative',
  MODERATE_CONSERVATIVE = 'moderate_conservative',
  MODERATE = 'moderate',
  MODERATE_AGGRESSIVE = 'moderate_aggressive',
  AGGRESSIVE = 'aggressive'
}

export enum AssetClass {
  US_STOCKS = 'us_stocks',
  INTL_STOCKS = 'intl_stocks',
  EMERGING_MARKETS = 'emerging_markets',
  BONDS = 'bonds',
  CASH = 'cash',
  ALTERNATIVES = 'alternatives'
}

export enum RebalanceTrigger {
  THRESHOLD = 'threshold',
  CALENDAR = 'calendar',
  MANUAL = 'manual',
  TAX_LOSS_HARVEST = 'tax_loss_harvest'
}

export enum RebalanceStatus {
  PENDING = 'pending',
  EXECUTING = 'executing',
  COMPLETED = 'completed',
  FAILED = 'failed',
  CANCELLED = 'cancelled'
}

export enum TransactionAction {
  BUY = 'buy',
  SELL = 'sell'
}

// Interfaces
export interface RiskQuestionnaire {
  version: string;
  completedAt: string;
  responses: RiskQuestionResponse[];
}

export interface RiskQuestionResponse {
  questionId: string;
  question: string;
  answer: string;
  score: number;
}

export interface RiskProfile {
  score: number; // 0-100
  level: RiskLevel;
  questionnaire: RiskQuestionnaire;
}

export interface Allocation {
  stocks: number;
  bonds: number;
  cash: number;
  alternatives: number;
}

export interface Holding {
  symbol: string;
  name: string;
  assetClass: AssetClass;
  shares: number;
  costBasis: number;
  currentPrice: number;
  marketValue: number;
  unrealizedGain: number;
  weight: number;
}

export interface Performance {
  inception: number;
  ytd: number;
  oneMonth: number;
  threeMonth: number;
  oneYear: number;
  threeYear: number;
  fiveYear: number;
}

export interface Portfolio {
  id: string;
  userId: string;
  accountType: AccountType;
  createdAt: string;
  updatedAt: string;
  status: PortfolioStatus;
  totalValue: number;
  currency: string;
  riskProfile: RiskProfile;
  targetAllocation: Allocation;
  currentAllocation: Allocation;
  holdings: Holding[];
  performance: Performance;
  lastRebalance: string;
  nextRebalanceCheck: string;
}

export interface RiskAssessment {
  id: string;
  userId: string;
  version: string;
  completedAt: string;
  expiresAt: string;
  questions: RiskQuestionResponse[];
  totalScore: number;
  riskLevel: RiskLevel;
  recommendedAllocation: {
    stocks: number;
    bonds: number;
  };
  constraints: {
    timeHorizon: number;
    liquidityNeeds: 'high' | 'medium' | 'low';
    taxSensitivity: 'high' | 'medium' | 'low';
  };
}

export interface Transaction {
  action: TransactionAction;
  symbol: string;
  shares: number;
  price: number;
  value: number;
  fees: number;
}

export interface RebalanceAnalysis {
  maxDrift: number;
  driftedAssets: string[];
  rebalanceNeeded: boolean;
  estimatedCost: number;
  estimatedTaxImpact: number;
}

export interface RebalanceImpact {
  transactionCosts: number;
  taxImpact: number;
  portfolioRiskChange: number;
}

export interface RebalanceEvent {
  id: string;
  portfolioId: string;
  triggeredAt: string;
  executedAt: string;
  status: RebalanceStatus;
  trigger: RebalanceTrigger;
  analysis: RebalanceAnalysis;
  transactions: Transaction[];
  impact: RebalanceImpact;
}

// API Request/Response Types
export interface CreatePortfolioRequest {
  userId: string;
  accountType: AccountType;
  initialDeposit: number;
  riskScore: number;
}

export interface CreatePortfolioResponse {
  portfolioId: string;
  status: PortfolioStatus;
  createdAt: string;
}

export interface UpdateAllocationRequest {
  stocks: number;
  bonds: number;
  cash?: number;
  alternatives?: number;
}

export interface SubmitRiskAssessmentRequest {
  userId: string;
  responses: Array<{
    questionId: string;
    answer: string;
    score: number;
  }>;
}

export interface RiskAssessmentResponse {
  riskScore: number;
  riskLevel: RiskLevel;
  recommendedAllocation: {
    stocks: number;
    bonds: number;
  };
}

export interface RebalancePreview {
  rebalanceNeeded: boolean;
  maxDrift: number;
  proposedTransactions: Transaction[];
  estimatedCost: number;
  estimatedTaxImpact: number;
}

export interface RebalanceExecuteResponse {
  rebalanceId: string;
  status: RebalanceStatus;
}

// Configuration Types
export interface RoboAdvisorConfig {
  apiKey: string;
  apiSecret?: string;
  baseURL?: string;
  timeout?: number;
  retries?: number;
}

// Error Types
export class RoboAdvisorError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number,
    public details?: any
  ) {
    super(message);
    this.name = 'RoboAdvisorError';
  }
}

// Utility Types
export type PartialPortfolio = Partial<Portfolio>;
export type PortfolioId = string;
export type UserId = string;

// API Response Wrapper
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
}
