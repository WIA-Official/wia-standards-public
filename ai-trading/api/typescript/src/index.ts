/**
 * WIA-FIN-017 AI Trading Standard - TypeScript SDK
 * @module @wia/ai-trading
 */

import axios, { AxiosInstance } from 'axios';
import WebSocket from 'ws';
import EventEmitter from 'eventemitter3';
import * as types from './types';

export * from './types';

/**
 * Main AI Trading Client
 */
export class AITradingClient extends EventEmitter {
  private api: AxiosInstance;
  private ws: WebSocket | null = null;
  private config: types.ClientConfig;
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 5;

  constructor(config: types.ClientConfig) {
    super();
    this.config = {
      baseURL: 'https://api.wia-trading.io/v1',
      timeout: 30000,
      retryAttempts: 3,
      webSocketURL: 'wss://ws.wia-trading.io/v1/stream',
      environment: 'production',
      ...config,
    };

    this.api = axios.create({
      baseURL: this.config.baseURL,
      timeout: this.config.timeout,
      headers: {
        'X-API-Key': this.config.apiKey,
        'Content-Type': 'application/json',
      },
    });
  }

  // ========================================================================
  // Market Data Methods
  // ========================================================================

  /**
   * Get real-time quote for symbol
   */
  async getQuote(symbol: string): Promise<types.Quote> {
    const response = await this.api.get(`/market-data/quote/${symbol}`);
    return response.data;
  }

  /**
   * Get OHLCV data for symbol
   */
  async getOHLCV(
    symbol: string,
    timeframe: string,
    startDate?: string,
    endDate?: string
  ): Promise<types.OHLCV[]> {
    const response = await this.api.get(`/market-data/ohlcv/${symbol}`, {
      params: { timeframe, startDate, endDate },
    });
    return response.data;
  }

  /**
   * Get order book snapshot
   */
  async getOrderBook(symbol: string, depth: number = 20): Promise<types.OrderBook> {
    const response = await this.api.get(`/market-data/orderbook/${symbol}`, {
      params: { depth },
    });
    return response.data;
  }

  // ========================================================================
  // Strategy & Signal Methods
  // ========================================================================

  /**
   * Create a new trading strategy
   */
  async createStrategy(strategy: Partial<types.Strategy>): Promise<types.Strategy> {
    const response = await this.api.post('/strategies', strategy);
    return response.data;
  }

  /**
   * Get strategy by ID
   */
  async getStrategy(strategyId: string): Promise<types.Strategy> {
    const response = await this.api.get(`/strategies/${strategyId}`);
    return response.data;
  }

  /**
   * Update strategy parameters
   */
  async updateStrategy(
    strategyId: string,
    updates: Partial<types.Strategy>
  ): Promise<types.Strategy> {
    const response = await this.api.patch(`/strategies/${strategyId}`, updates);
    return response.data;
  }

  /**
   * Get trading signals from strategy
   */
  async getSignals(strategyId: string, symbol?: string): Promise<types.TradingSignal[]> {
    const response = await this.api.get(`/strategies/${strategyId}/signals`, {
      params: { symbol },
    });
    return response.data;
  }

  /**
   * Generate signal for specific symbol
   */
  async generateSignal(
    strategyId: string,
    symbol: string
  ): Promise<types.TradingSignal> {
    const response = await this.api.post(`/strategies/${strategyId}/signals`, {
      symbol,
    });
    return response.data;
  }

  // ========================================================================
  // Order Management Methods
  // ========================================================================

  /**
   * Place a new order
   */
  async placeOrder(order: Partial<types.Order>): Promise<types.Order> {
    const response = await this.api.post('/orders', order);
    this.emit('order:placed', response.data);
    return response.data;
  }

  /**
   * Get order by ID
   */
  async getOrder(orderId: string): Promise<types.Order> {
    const response = await this.api.get(`/orders/${orderId}`);
    return response.data;
  }

  /**
   * Cancel an order
   */
  async cancelOrder(orderId: string): Promise<types.Order> {
    const response = await this.api.delete(`/orders/${orderId}`);
    this.emit('order:cancelled', response.data);
    return response.data;
  }

  /**
   * Get all open orders
   */
  async getOpenOrders(symbol?: string): Promise<types.Order[]> {
    const response = await this.api.get('/orders', {
      params: { status: 'open', symbol },
    });
    return response.data;
  }

  /**
   * Get order history
   */
  async getOrderHistory(
    startDate?: string,
    endDate?: string,
    symbol?: string
  ): Promise<types.Order[]> {
    const response = await this.api.get('/orders/history', {
      params: { startDate, endDate, symbol },
    });
    return response.data;
  }

  // ========================================================================
  // Position & Portfolio Methods
  // ========================================================================

  /**
   * Get all current positions
   */
  async getPositions(): Promise<types.Position[]> {
    const response = await this.api.get('/positions');
    return response.data;
  }

  /**
   * Get position for specific symbol
   */
  async getPosition(symbol: string): Promise<types.Position> {
    const response = await this.api.get(`/positions/${symbol}`);
    return response.data;
  }

  /**
   * Close position
   */
  async closePosition(symbol: string, quantity?: number): Promise<types.Order> {
    const response = await this.api.post(`/positions/${symbol}/close`, { quantity });
    this.emit('position:closed', response.data);
    return response.data;
  }

  /**
   * Get portfolio summary
   */
  async getPortfolio(): Promise<types.Portfolio> {
    const response = await this.api.get('/portfolio');
    return response.data;
  }

  /**
   * Get portfolio performance metrics
   */
  async getPerformanceMetrics(
    startDate?: string,
    endDate?: string
  ): Promise<types.PerformanceMetrics> {
    const response = await this.api.get('/portfolio/performance', {
      params: { startDate, endDate },
    });
    return response.data;
  }

  // ========================================================================
  // Backtesting Methods
  // ========================================================================

  /**
   * Run backtest for strategy
   */
  async runBacktest(config: types.BacktestConfig): Promise<types.BacktestResult> {
    const response = await this.api.post('/backtest', config);
    return response.data;
  }

  /**
   * Get backtest results
   */
  async getBacktestResults(backtestId: string): Promise<types.BacktestResult> {
    const response = await this.api.get(`/backtest/${backtestId}`);
    return response.data;
  }

  // ========================================================================
  // ML Model Methods
  // ========================================================================

  /**
   * Get model metadata
   */
  async getModel(modelId: string): Promise<types.ModelMetadata> {
    const response = await this.api.get(`/models/${modelId}`);
    return response.data;
  }

  /**
   * Get model prediction
   */
  async getPrediction(
    modelId: string,
    features: Record<string, number>
  ): Promise<types.Prediction> {
    const response = await this.api.post(`/models/${modelId}/predict`, { features });
    return response.data;
  }

  /**
   * Get model explanation (XAI)
   */
  async getModelExplanation(
    modelId: string,
    features: Record<string, number>
  ): Promise<types.ModelExplanation> {
    const response = await this.api.post(`/models/${modelId}/explain`, { features });
    return response.data;
  }

  // ========================================================================
  // Risk Management Methods
  // ========================================================================

  /**
   * Get current risk metrics
   */
  async getRiskMetrics(): Promise<types.RiskMetrics> {
    const response = await this.api.get('/risk/metrics');
    return response.data;
  }

  /**
   * Get circuit breaker status
   */
  async getCircuitBreakers(): Promise<types.CircuitBreaker[]> {
    const response = await this.api.get('/risk/circuit-breakers');
    return response.data;
  }

  /**
   * Update circuit breaker
   */
  async updateCircuitBreaker(
    name: string,
    updates: Partial<types.CircuitBreaker>
  ): Promise<types.CircuitBreaker> {
    const response = await this.api.patch(`/risk/circuit-breakers/${name}`, updates);
    return response.data;
  }

  /**
   * Get risk limits
   */
  async getRiskLimits(): Promise<types.RiskLimit[]> {
    const response = await this.api.get('/risk/limits');
    return response.data;
  }

  // ========================================================================
  // Alternative Data Methods
  // ========================================================================

  /**
   * Get sentiment data
   */
  async getSentimentData(
    symbol: string,
    source?: string
  ): Promise<types.SentimentData[]> {
    const response = await this.api.get(`/alt-data/sentiment/${symbol}`, {
      params: { source },
    });
    return response.data;
  }

  /**
   * Get on-chain metrics (for crypto)
   */
  async getOnChainMetrics(symbol: string): Promise<types.OnChainMetrics> {
    const response = await this.api.get(`/alt-data/onchain/${symbol}`);
    return response.data;
  }

  // ========================================================================
  // WebSocket Methods
  // ========================================================================

  /**
   * Connect to WebSocket stream
   */
  connectWebSocket(): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      return;
    }

    this.ws = new WebSocket(this.config.webSocketURL!, {
      headers: {
        'X-API-Key': this.config.apiKey,
      },
    });

    this.ws.on('open', () => {
      this.emit('ws:connected');
      this.reconnectAttempts = 0;
    });

    this.ws.on('message', (data: WebSocket.Data) => {
      try {
        const message: types.WebSocketMessage = JSON.parse(data.toString());
        this.emit(`ws:${message.type}`, message.data);
        this.emit('ws:message', message);
      } catch (error) {
        this.emit('ws:error', error);
      }
    });

    this.ws.on('error', (error) => {
      this.emit('ws:error', error);
    });

    this.ws.on('close', () => {
      this.emit('ws:disconnected');
      this.handleReconnect();
    });
  }

  /**
   * Disconnect WebSocket
   */
  disconnectWebSocket(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }

  /**
   * Subscribe to WebSocket channel
   */
  subscribe(channel: string, params?: Record<string, any>): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error('WebSocket not connected');
    }

    this.ws.send(
      JSON.stringify({
        action: 'subscribe',
        channel,
        ...params,
      })
    );
  }

  /**
   * Unsubscribe from WebSocket channel
   */
  unsubscribe(channel: string): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      return;
    }

    this.ws.send(
      JSON.stringify({
        action: 'unsubscribe',
        channel,
      })
    );
  }

  /**
   * Handle WebSocket reconnection
   */
  private handleReconnect(): void {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++;
      const delay = Math.min(1000 * Math.pow(2, this.reconnectAttempts), 30000);

      setTimeout(() => {
        this.connectWebSocket();
      }, delay);
    } else {
      this.emit('ws:max_reconnect_attempts');
    }
  }

  // ========================================================================
  // System Health Methods
  // ========================================================================

  /**
   * Get system health status
   */
  async getSystemHealth(): Promise<types.SystemHealth> {
    const response = await this.api.get('/system/health');
    return response.data;
  }

  /**
   * Get alerts
   */
  async getAlerts(
    level?: types.AlertLevel,
    acknowledged?: boolean
  ): Promise<types.Alert[]> {
    const response = await this.api.get('/system/alerts', {
      params: { level, acknowledged },
    });
    return response.data;
  }

  /**
   * Acknowledge alert
   */
  async acknowledgeAlert(alertId: string): Promise<types.Alert> {
    const response = await this.api.patch(`/system/alerts/${alertId}/acknowledge`);
    return response.data;
  }
}

/**
 * Utility function to calculate common indicators
 */
export class Indicators {
  /**
   * Calculate Simple Moving Average
   */
  static sma(data: number[], period: number): number[] {
    const result: number[] = [];
    for (let i = period - 1; i < data.length; i++) {
      const sum = data.slice(i - period + 1, i + 1).reduce((a, b) => a + b, 0);
      result.push(sum / period);
    }
    return result;
  }

  /**
   * Calculate Exponential Moving Average
   */
  static ema(data: number[], period: number): number[] {
    const multiplier = 2 / (period + 1);
    const result: number[] = [data[0]];

    for (let i = 1; i < data.length; i++) {
      result.push((data[i] - result[i - 1]) * multiplier + result[i - 1]);
    }

    return result;
  }

  /**
   * Calculate RSI (Relative Strength Index)
   */
  static rsi(data: number[], period: number = 14): number[] {
    const changes = data.slice(1).map((val, i) => val - data[i]);
    const gains = changes.map((c) => (c > 0 ? c : 0));
    const losses = changes.map((c) => (c < 0 ? -c : 0));

    const avgGains = this.sma(gains, period);
    const avgLosses = this.sma(losses, period);

    return avgGains.map((gain, i) => {
      const rs = avgLosses[i] === 0 ? 100 : gain / avgLosses[i];
      return 100 - 100 / (1 + rs);
    });
  }
}

/**
 * Export default client
 */
export default AITradingClient;
