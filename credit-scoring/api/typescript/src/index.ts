/**
 * WIA-FIN-020 Credit Scoring Standard - TypeScript SDK
 * Version: 2.0.0
 * 
 * © 2025 SmileStory Inc. / WIA
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

import axios, { AxiosInstance } from 'axios';
import {
  CreditScoringConfig,
  Application,
  ScoreOptions,
  ScoreResult,
  AdverseActionNotice,
  MonitoringAlert,
  BatchScoreRequest,
  BatchScoreStatus,
  WebhookRegistration,
  PerformanceMetrics,
  FairnessMetrics
} from './types';

export * from './types';

export class CreditScoringClient {
  private api: AxiosInstance;
  private config: Required<CreditScoringConfig>;

  constructor(config: CreditScoringConfig) {
    this.config = {
      apiKey: config.apiKey,
      environment: config.environment || 'production',
      baseUrl: config.baseUrl || this.getDefaultBaseUrl(config.environment || 'production'),
      timeout: config.timeout || 5000,
      retries: config.retries || 3
    };

    this.api = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'FIN-020',
        'X-WIA-Version': '2.0'
      }
    });

    // Add retry logic
    this.api.interceptors.response.use(
      response => response,
      async error => {
        const config = error.config;
        if (!config || !config.retry) {
          config.retry = 0;
        }

        if (config.retry >= this.config.retries) {
          return Promise.reject(error);
        }

        config.retry += 1;
        const delay = Math.pow(2, config.retry) * 1000;
        await new Promise(resolve => setTimeout(resolve, delay));
        return this.api(config);
      }
    );
  }

  private getDefaultBaseUrl(environment: string): string {
    const urls = {
      'production': 'https://api.wia.org/v2/credit-scoring',
      'sandbox': 'https://sandbox-api.wia.org/v2/credit-scoring',
      'development': 'http://localhost:8000/v2/credit-scoring'
    };
    return urls[environment as keyof typeof urls] || urls.production;
  }

  /**
   * Score a credit application
   * @param application - Application data
   * @param options - Scoring options
   * @returns Credit score result
   */
  async score(application: Application, options: ScoreOptions = {}): Promise<ScoreResult> {
    const response = await this.api.post<ScoreResult>('/score', {
      application,
      options
    });
    return response.data;
  }

  /**
   * Get existing score by ID
   * @param scoreId - Score ID
   * @returns Score result
   */
  async getScore(scoreId: string): Promise<ScoreResult> {
    const response = await this.api.get<ScoreResult>(`/scores/${scoreId}`);
    return response.data;
  }

  /**
   * Generate adverse action notice
   * @param scoreId - Score ID (for declined application)
   * @returns Adverse action notice
   */
  async generateAdverseActionNotice(scoreId: string): Promise<AdverseActionNotice> {
    const response = await this.api.post<AdverseActionNotice>(
      `/scores/${scoreId}/adverse-action-notice`
    );
    return response.data;
  }

  /**
   * Enable continuous monitoring for an account
   * @param accountId - Account ID
   * @param frequency - Update frequency
   * @param alerts - Alert types to enable
   * @returns Monitoring configuration
   */
  async monitorAccount(params: {
    accountId: string;
    frequency: 'daily' | 'weekly' | 'monthly';
    alerts: string[];
  }): Promise<{ status: string; monitoringId: string }> {
    const response = await this.api.post('/monitoring/subscribe', params);
    return response.data;
  }

  /**
   * Get monitoring alerts for an account
   * @param accountId - Account ID
   * @param limit - Maximum number of alerts to return
   * @returns List of alerts
   */
  async getAlerts(accountId: string, limit: number = 50): Promise<MonitoringAlert[]> {
    const response = await this.api.get<MonitoringAlert[]>(
      `/monitoring/alerts/${accountId}`,
      { params: { limit } }
    );
    return response.data;
  }

  /**
   * Submit batch scoring request
   * @param request - Batch request with multiple applications
   * @returns Batch status
   */
  async batchScore(request: BatchScoreRequest): Promise<BatchScoreStatus> {
    const response = await this.api.post<BatchScoreStatus>('/batch-score', request);
    return response.data;
  }

  /**
   * Get batch scoring status
   * @param batchId - Batch ID
   * @returns Batch status and results
   */
  async getBatchStatus(batchId: string): Promise<BatchScoreStatus> {
    const response = await this.api.get<BatchScoreStatus>(`/batch/${batchId}`);
    return response.data;
  }

  /**
   * Register webhook for events
   * @param registration - Webhook configuration
   * @returns Webhook ID
   */
  async registerWebhook(registration: WebhookRegistration): Promise<{ webhookId: string }> {
    const response = await this.api.post('/webhooks/register', registration);
    return response.data;
  }

  /**
   * Delete webhook
   * @param webhookId - Webhook ID
   */
  async deleteWebhook(webhookId: string): Promise<void> {
    await this.api.delete(`/webhooks/${webhookId}`);
  }

  /**
   * Get model performance metrics
   * @param period - Time period (e.g., "2025-01", "2025-Q1")
   * @returns Performance metrics
   */
  async getPerformanceMetrics(period: string): Promise<PerformanceMetrics> {
    const response = await this.api.get<PerformanceMetrics>('/metrics/performance', {
      params: { period }
    });
    return response.data;
  }

  /**
   * Get fairness metrics
   * @param period - Time period
   * @returns Fairness metrics
   */
  async getFairnessMetrics(period: string): Promise<FairnessMetrics> {
    const response = await this.api.get<FairnessMetrics>('/metrics/fairness', {
      params: { period }
    });
    return response.data;
  }

  /**
   * Simulate "what-if" scenario
   * @param scoreId - Existing score ID
   * @param changes - Changes to simulate
   * @returns Updated score prediction
   */
  async simulateScore(
    scoreId: string,
    changes: Partial<Application>
  ): Promise<ScoreResult> {
    const response = await this.api.post<ScoreResult>(
      `/scores/${scoreId}/simulate`,
      { changes }
    );
    return response.data;
  }
}

// Convenience function for quick scoring
export async function quickScore(
  apiKey: string,
  application: Application
): Promise<ScoreResult> {
  const client = new CreditScoringClient({ apiKey });
  return client.score(application);
}
