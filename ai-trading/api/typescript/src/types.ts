/**
 * WIA-FIN-017 AI Trading Standard - TypeScript Type Definitions
 * @module @wia/ai-trading/types
 */

// ============================================================================
// Market Data Types
// ============================================================================

export interface Quote {
  symbol: string;
  timestamp: number;
  bid: number;
  ask: number;
  bidSize: number;
  askSize: number;
  exchange: string;
}

export interface Trade {
  symbol: string;
  timestamp: number;
  price: number;
  size: number;
  side: 'buy' | 'sell';
  exchange: string;
}

export interface OHLCV {
  symbol: string;
  timestamp: number;
  open: number;
  high: number;
  low: number;
  close: number;
  volume: number;
  timeframe: string;
}

export interface OrderBook {
  symbol: string;
  timestamp: number;
  bids: [number, number][]; // [price, size][]
  asks: [number, number][]; // [price, size][]
  exchange: string;
}

// ============================================================================
// AI Model Types
// ============================================================================

export interface ModelMetadata {
  modelId: string;
  version: string;
  type: 'lstm' | 'gru' | 'transformer' | 'xgboost' | 'random_forest' | 'reinforcement_learning';
  inputFeatures: number;
  outputType: 'classification' | 'regression';
  trainingData: {
    startDate: string;
    endDate: string;
    samples: number;
  };
  performance: PerformanceMetrics;
}

export interface Prediction {
  timestamp: number;
  symbol: string;
  prediction: number | string;
  confidence: number;
  modelId: string;
}

export interface FeatureImportance {
  [featureName: string]: number;
}

export interface ModelExplanation {
  prediction: 'buy' | 'sell' | 'hold';
  confidence: number;
  featureImportance: FeatureImportance;
  shapValues?: Record<string, number>;
  counterfactual?: string;
  decisionPath?: string[];
}

// ============================================================================
// Trading Signal & Strategy Types
// ============================================================================

export interface TradingSignal {
  timestamp: number;
  symbol: string;
  signal: 'buy' | 'sell' | 'hold';
  strength: number; // 0-1
  confidence: number; // 0-1
  price: number;
  size: number;
  strategyId: string;
  explanation?: ModelExplanation;
}

export interface Strategy {
  strategyId: string;
  name: string;
  description: string;
  type: 'momentum' | 'mean_reversion' | 'arbitrage' | 'market_making' | 'ml_based';
  parameters: Record<string, any>;
  riskProfile: RiskProfile;
  performance: PerformanceMetrics;
}

export interface RiskProfile {
  maxPositionSize: number; // percentage of portfolio
  maxLeverage: number;
  stopLoss: number; // percentage
  takeProfit?: number; // percentage
  maxDrawdown: number; // percentage
  maxDailyLoss: number; // percentage
}

// ============================================================================
// Order Types
// ============================================================================

export type OrderSide = 'buy' | 'sell';
export type OrderType = 'market' | 'limit' | 'stop' | 'stop_limit' | 'trailing_stop' | 'iceberg';
export type TimeInForce = 'GTC' | 'IOC' | 'FOK' | 'GTD';
export type OrderStatus = 'pending' | 'open' | 'partially_filled' | 'filled' | 'cancelled' | 'rejected' | 'expired';

export interface Order {
  orderId: string;
  symbol: string;
  side: OrderSide;
  type: OrderType;
  quantity: number;
  price?: number;
  stopPrice?: number;
  timeInForce: TimeInForce;
  reduceOnly?: boolean;
  postOnly?: boolean;
  status: OrderStatus;
  filledQuantity?: number;
  averagePrice?: number;
  commission?: number;
  timestamp: number;
  strategyId?: string;
}

export interface IcebergOrder extends Order {
  totalQuantity: number;
  visibleQuantity: number;
  variancePct?: number;
}

// ============================================================================
// Position & Portfolio Types
// ============================================================================

export interface Position {
  symbol: string;
  side: 'long' | 'short';
  quantity: number;
  entryPrice: number;
  currentPrice: number;
  unrealizedPnL: number;
  realizedPnL: number;
  leverage?: number;
  marginUsed?: number;
  liquidationPrice?: number;
  timestamp: number;
}

export interface Portfolio {
  totalValue: number;
  cash: number;
  equity: number;
  margin?: number;
  positions: Position[];
  performance: PerformanceMetrics;
  riskMetrics: RiskMetrics;
}

// ============================================================================
// Performance & Risk Metrics
// ============================================================================

export interface PerformanceMetrics {
  totalReturn: number;
  cagr?: number;
  sharpeRatio: number;
  sortinoRatio?: number;
  calmarRatio?: number;
  maxDrawdown: number;
  winRate: number;
  profitFactor: number;
  totalTrades: number;
  avgWin?: number;
  avgLoss?: number;
  bestTrade?: number;
  worstTrade?: number;
}

export interface RiskMetrics {
  volatility: number;
  beta?: number;
  var95: number; // Value at Risk (95% confidence)
  cvar95: number; // Conditional VaR
  currentDrawdown: number;
  maxLeverage: number;
  correlationMatrix?: Record<string, Record<string, number>>;
}

export interface TransactionCostAnalysis {
  orderId: string;
  arrivalPrice: number;
  executionPrice: number;
  slippageBps: number;
  commissionUsd: number;
  marketImpactBps: number;
  timingCostBps: number;
  totalCostBps: number;
  benchmark: 'arrival_price' | 'vwap' | 'twap';
}

// ============================================================================
// Backtesting Types
// ============================================================================

export interface BacktestConfig {
  strategyId: string;
  startDate: string;
  endDate: string;
  initialCapital: number;
  commission: number;
  slippage: number;
  symbols: string[];
  timeframe: string;
  walkForward?: {
    trainDays: number;
    testDays: number;
    stepDays: number;
  };
}

export interface BacktestResult {
  config: BacktestConfig;
  performance: PerformanceMetrics;
  equityCurve: { timestamp: number; value: number }[];
  trades: Order[];
  drawdowns: { timestamp: number; drawdown: number }[];
}

// ============================================================================
// Risk Management Types
// ============================================================================

export interface CircuitBreaker {
  name: string;
  enabled: boolean;
  type: 'daily_loss' | 'drawdown' | 'volatility' | 'correlation';
  threshold: number;
  action: 'halt' | 'reduce_exposure' | 'alert';
  triggered: boolean;
  triggeredAt?: number;
}

export interface RiskLimit {
  limitType: 'position_size' | 'leverage' | 'daily_loss' | 'total_exposure';
  value: number;
  unit: 'usd' | 'percentage' | 'ratio';
  currentValue: number;
  breached: boolean;
}

// ============================================================================
// Monitoring & Alerts
// ============================================================================

export type AlertLevel = 'info' | 'warning' | 'error' | 'critical';

export interface Alert {
  alertId: string;
  timestamp: number;
  level: AlertLevel;
  type: string;
  message: string;
  data?: Record<string, any>;
  acknowledged: boolean;
}

export interface SystemHealth {
  status: 'healthy' | 'degraded' | 'down';
  latencyP50Ms: number;
  latencyP99Ms: number;
  orderFillRate: number;
  uptime: number;
  lastCheck: number;
  errors: Alert[];
}

// ============================================================================
// Client Configuration
// ============================================================================

export interface ClientConfig {
  apiKey: string;
  apiSecret?: string;
  baseURL?: string;
  timeout?: number;
  retryAttempts?: number;
  webSocketURL?: string;
  environment?: 'production' | 'sandbox' | 'backtest';
}

// ============================================================================
// WebSocket Event Types
// ============================================================================

export interface WebSocketMessage {
  type: 'quote' | 'trade' | 'orderbook' | 'signal' | 'order' | 'position' | 'alert';
  data: any;
  timestamp: number;
}

export type WebSocketCallback<T = any> = (data: T) => void;
export type WebSocketErrorCallback = (error: Error) => void;

// ============================================================================
// Alternative Data Types
// ============================================================================

export interface SentimentData {
  symbol: string;
  timestamp: number;
  source: 'twitter' | 'reddit' | 'news' | 'telegram';
  score: number; // -1 to 1
  confidence: number; // 0 to 1
  sampleSize: number;
  trendingTopics?: string[];
}

export interface OnChainMetrics {
  symbol: string;
  timestamp: number;
  activeAddresses24h: number;
  transactionVolumeUsd: number;
  exchangeInflow: number;
  exchangeOutflow: number;
  whaleTransactions24h: number;
  networkHashRate?: number;
}

// ============================================================================
// Ensemble & Multi-Model Types
// ============================================================================

export interface EnsembleConfig {
  method: 'weighted_average' | 'voting' | 'stacking';
  models: Array<{
    modelId: string;
    weight: number;
    performance?: PerformanceMetrics;
  }>;
}

export interface EnsemblePrediction {
  timestamp: number;
  symbol: string;
  prediction: number | string;
  confidence: number;
  modelPredictions: Array<{
    modelId: string;
    prediction: number | string;
    confidence: number;
    weight: number;
  }>;
}

// ============================================================================
// Export all types
// ============================================================================

export * from './types';
