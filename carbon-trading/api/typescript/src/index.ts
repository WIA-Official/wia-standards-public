/**
 * WIA-FIN-025 Carbon Trading Standard - TypeScript SDK
 * @version 2.0.0
 */

import axios, { AxiosInstance } from 'axios';
import EventEmitter from 'eventemitter3';
import WebSocket from 'ws';
import type * as Types from './types';

export * from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

export class CarbonTradingSDK extends EventEmitter {
  private api: AxiosInstance;
  private ws?: WebSocket;
  private config: Types.SDKConfig;

  // Sub-modules
  public projects: ProjectService;
  public credits: CreditService;
  public market: MarketService;
  public verification: VerificationService;

  constructor(config: Types.SDKConfig) {
    super();
    this.config = config;

    // Determine base URLs
    const baseUrl = config.baseUrl || this.getDefaultBaseUrl(config.environment);
    const wsUrl = config.wsUrl || this.getDefaultWsUrl(config.environment);

    // Initialize HTTP client
    this.api = axios.create({
      baseURL: baseUrl,
      timeout: config.timeout || 30000,
      headers: {
        'Authorization': `Bearer ${config.apiKey}`,
        'Content-Type': 'application/json',
        'User-Agent': 'WIA-Carbon-Trading-SDK/2.0.0'
      }
    });

    // Add response interceptor for error handling
    this.api.interceptors.response.use(
      response => response,
      error => {
        const apiError = this.handleAPIError(error);
        throw apiError;
      }
    );

    // Initialize sub-modules
    this.projects = new ProjectService(this.api);
    this.credits = new CreditService(this.api);
    this.market = new MarketService(this.api);
    this.verification = new VerificationService(this.api);
  }

  /**
   * Connect to WebSocket for real-time updates
   */
  async connectWebSocket(channels: string[]): Promise<void> {
    return new Promise((resolve, reject) => {
      const wsUrl = this.config.wsUrl || this.getDefaultWsUrl(this.config.environment);
      this.ws = new WebSocket(wsUrl, {
        headers: {
          'Authorization': `Bearer ${this.config.apiKey}`
        }
      });

      this.ws.on('open', () => {
        // Subscribe to channels
        const subscribeMsg: Types.WSSubscribeMessage = {
          action: 'subscribe',
          channels
        };
        this.ws!.send(JSON.stringify(subscribeMsg));
        this.emit('connected');
        resolve();
      });

      this.ws.on('message', (data: WebSocket.Data) => {
        try {
          const message: Types.WSMessage = JSON.parse(data.toString());
          this.emit('message', message);
          this.emit(message.channel, message.data);
        } catch (error) {
          this.emit('error', new Types.CarbonTradingError('Failed to parse WebSocket message', 'WS_PARSE_ERROR'));
        }
      });

      this.ws.on('error', (error) => {
        this.emit('error', error);
        reject(error);
      });

      this.ws.on('close', () => {
        this.emit('disconnected');
      });
    });
  }

  /**
   * Disconnect WebSocket
   */
  disconnectWebSocket(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = undefined;
    }
  }

  private getDefaultBaseUrl(environment: string): string {
    const urls = {
      production: 'https://api.wia.org/carbon/v2',
      staging: 'https://staging-api.wia.org/carbon/v2',
      development: 'http://localhost:3000/api/v2'
    };
    return urls[environment as keyof typeof urls] || urls.production;
  }

  private getDefaultWsUrl(environment: string): string {
    const urls = {
      production: 'wss://api.wia.org/carbon/v2/stream',
      staging: 'wss://staging-api.wia.org/carbon/v2/stream',
      development: 'ws://localhost:3000/api/v2/stream'
    };
    return urls[environment as keyof typeof urls] || urls.production;
  }

  private handleAPIError(error: any): Types.CarbonTradingError {
    if (error.response) {
      const apiError: Types.APIErrorResponse = error.response.data;
      return new Types.CarbonTradingError(
        apiError.error.message,
        apiError.error.code,
        error.response.status,
        apiError.error.details
      );
    } else if (error.request) {
      return new Types.CarbonTradingError(
        'No response from server',
        'NETWORK_ERROR'
      );
    } else {
      return new Types.CarbonTradingError(
        error.message,
        'UNKNOWN_ERROR'
      );
    }
  }
}

// ============================================================================
// Project Service
// ============================================================================

class ProjectService {
  constructor(private api: AxiosInstance) {}

  /**
   * Create a new carbon project
   */
  async create(request: Types.ProjectCreateRequest): Promise<Types.ProjectCreateResponse> {
    const response = await this.api.post('/projects', request);
    return response.data;
  }

  /**
   * Get project by ID
   */
  async get(projectId: string): Promise<Types.CarbonProject> {
    const response = await this.api.get(`/projects/${projectId}`);
    return response.data;
  }

  /**
   * List projects with filters
   */
  async list(params?: {
    type?: Types.ProjectType;
    status?: Types.ProjectStatus;
    country?: string;
    limit?: number;
    offset?: number;
  }): Promise<{ total: number; projects: Types.CarbonProject[] }> {
    const response = await this.api.get('/projects', { params });
    return response.data;
  }

  /**
   * Get real-time project metrics
   */
  async getMetrics(projectId: string): Promise<any> {
    const response = await this.api.get(`/projects/${projectId}/metrics/realtime`);
    return response.data;
  }

  /**
   * Update project
   */
  async update(projectId: string, updates: Partial<Types.CarbonProject>): Promise<Types.CarbonProject> {
    const response = await this.api.patch(`/projects/${projectId}`, updates);
    return response.data;
  }
}

// ============================================================================
// Credit Service
// ============================================================================

class CreditService {
  constructor(private api: AxiosInstance) {}

  /**
   * Issue carbon credits
   */
  async issue(request: Types.CreditIssueRequest): Promise<Types.CreditIssueResponse> {
    const response = await this.api.post('/credits/issue', request);
    return response.data;
  }

  /**
   * Get credit by ID
   */
  async get(creditId: string): Promise<Types.CarbonCredit> {
    const response = await this.api.get(`/credits/${creditId}`);
    return response.data;
  }

  /**
   * Search credits
   */
  async search(params: Types.CreditSearchParams): Promise<Types.CreditSearchResponse> {
    const response = await this.api.get('/credits/search', { params });
    return response.data;
  }

  /**
   * Transfer credits
   */
  async transfer(creditId: string, to: string, quantity: number, price?: number): Promise<any> {
    const response = await this.api.post('/credits/transfer', {
      creditId,
      to,
      quantity,
      price
    });
    return response.data;
  }

  /**
   * Retire credits
   */
  async retire(
    creditId: string,
    quantity: number,
    beneficiary: string,
    reason: string
  ): Promise<Types.RetirementInfo> {
    const response = await this.api.post('/credits/retire', {
      creditId,
      quantity,
      beneficiary,
      reason
    });
    return response.data;
  }

  /**
   * Get credit transactions
   */
  async getTransactions(creditId: string): Promise<Types.TransactionRecord[]> {
    const response = await this.api.get(`/credits/${creditId}/transactions`);
    return response.data;
  }
}

// ============================================================================
// Market Service
// ============================================================================

class MarketService {
  constructor(private api: AxiosInstance) {}

  /**
   * Create a trade order
   */
  async createOrder(request: Types.OrderCreateRequest): Promise<Types.OrderCreateResponse> {
    const response = await this.api.post('/trading/orders', request);
    return response.data;
  }

  /**
   * Get order by ID
   */
  async getOrder(orderId: string): Promise<Types.TradeOrder> {
    const response = await this.api.get(`/trading/orders/${orderId}`);
    return response.data;
  }

  /**
   * Cancel order
   */
  async cancelOrder(orderId: string): Promise<void> {
    await this.api.delete(`/trading/orders/${orderId}`);
  }

  /**
   * Get order book
   */
  async getOrderBook(creditType?: Types.CreditType, vintage?: number): Promise<any> {
    const response = await this.api.get('/trading/orderbook', {
      params: { creditType, vintage }
    });
    return response.data;
  }

  /**
   * Get current market prices
   */
  async getPrices(): Promise<Types.MarketPrice[]> {
    const response = await this.api.get('/market/prices');
    return response.data;
  }

  /**
   * Get market statistics
   */
  async getStatistics(period: '24h' | '7d' | '30d' = '24h'): Promise<any> {
    const response = await this.api.get('/market/statistics', {
      params: { period }
    });
    return response.data;
  }

  /**
   * Get trade history
   */
  async getTradeHistory(params?: {
    limit?: number;
    offset?: number;
    from?: string;
    to?: string;
  }): Promise<any> {
    const response = await this.api.get('/trading/history', { params });
    return response.data;
  }
}

// ============================================================================
// Verification Service
// ============================================================================

class VerificationService {
  constructor(private api: AxiosInstance) {}

  /**
   * Submit monitoring data
   */
  async submitMonitoringData(request: Types.VerificationRequest): Promise<any> {
    const response = await this.api.post('/verification/monitoring-data', request);
    return response.data;
  }

  /**
   * Request AI verification
   */
  async aiVerify(projectId: string, monitoringDataId: string): Promise<Types.VerificationResult> {
    const response = await this.api.post('/verification/ai-verify', {
      projectId,
      monitoringDataId
    });
    return response.data;
  }

  /**
   * Get verification status
   */
  async getStatus(verificationId: string): Promise<any> {
    const response = await this.api.get(`/verification/${verificationId}`);
    return response.data;
  }

  /**
   * Get verification history for project
   */
  async getHistory(projectId: string): Promise<Types.VerificationResult[]> {
    const response = await this.api.get(`/projects/${projectId}/verifications`);
    return response.data;
  }
}

// ============================================================================
// Exports
// ============================================================================

export default CarbonTradingSDK;

// Example usage:
/*
import CarbonTradingSDK from '@wia/carbon-trading';

const sdk = new CarbonTradingSDK({
  apiKey: 'your-api-key',
  environment: 'production'
});

// Create a project
const project = await sdk.projects.create({
  name: 'Solar Farm India',
  type: 'renewable_energy',
  location: {
    country: 'India',
    region: 'Gujarat'
  },
  methodology: 'ACM0002',
  developer: {
    name: 'Green Energy Corp',
    contact: 'info@greenenergy.com'
  },
  estimatedAnnualReduction: 50000
});

// Issue credits
const credits = await sdk.credits.issue({
  projectId: project.projectId,
  quantity: 50000,
  vintage: 2024,
  creditType: 'reduction',
  verificationReport: '...',
  calculateQuality: true
});

// Create market order
const order = await sdk.market.createOrder({
  type: 'market',
  side: 'buy',
  quantity: 10000,
  minQualityScore: 85
});

// Connect to real-time feed
await sdk.connectWebSocket(['prices.all', 'trades.VCS']);
sdk.on('prices.all', (data) => {
  console.log('Price update:', data);
});

// Retire credits
await sdk.credits.retire(
  credits.credits[0].id,
  5000,
  'Acme Corp',
  'Corporate net-zero commitment 2024'
);
*/
