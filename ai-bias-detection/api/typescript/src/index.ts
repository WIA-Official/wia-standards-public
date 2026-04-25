/**
 * WIA-AI-013: AI Bias Detection Standard
 * TypeScript SDK
 *
 * Philosophy: 弘益人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @license CC BY 4.0
 */

import type {
  BiasDetectionReport,
  BiasDetectorConfig,
  AnalysisJob,
  APIResponse,
  FairnessMetrics,
  MonitoringAlert,
  PredictionLog,
  DriftDetectionResult,
  ProtectedAttribute,
  FairnessMetric,
} from './types';

export * from './types';

/**
 * Main BiasDetector class for analyzing AI models
 */
export class BiasDetector {
  private config: BiasDetectorConfig;
  private apiKey: string;
  private baseURL: string;

  /**
   * Create a new BiasDetector instance
   *
   * @param config - Configuration options
   * @param apiKey - API key for authentication
   * @param baseURL - Base URL for API (default: https://api.wia.org/ai-013/v1)
   */
  constructor(
    config: BiasDetectorConfig,
    apiKey?: string,
    baseURL: string = 'https://api.wia.org/ai-013/v1'
  ) {
    this.config = config;
    this.apiKey = apiKey || process.env.WIA_API_KEY || '';
    this.baseURL = baseURL;
  }

  /**
   * Analyze a model for bias
   *
   * @param modelId - Unique identifier for the model
   * @param datasetId - Unique identifier for the dataset
   * @param options - Additional analysis options
   * @returns Promise resolving to an AnalysisJob
   */
  async analyzeModel(
    modelId: string,
    datasetId: string,
    options?: {
      fairnessMetrics?: FairnessMetric[];
      includeIntersectional?: boolean;
    }
  ): Promise<AnalysisJob> {
    const response = await this.fetch('/bias/analyze', {
      method: 'POST',
      body: JSON.stringify({
        modelId,
        datasetId,
        protectedAttributes: this.config.protectedAttributes,
        fairnessMetrics: options?.fairnessMetrics || [
          'demographic_parity',
          'equalized_odds',
        ],
        options: {
          includeIntersectional: options?.includeIntersectional ?? true,
          thresholds: this.config.fairnessCriteria,
        },
      }),
    });

    return response.data as AnalysisJob;
  }

  /**
   * Get analysis job status and results
   *
   * @param jobId - Job identifier
   * @returns Promise resolving to an AnalysisJob
   */
  async getAnalysisJob(jobId: string): Promise<AnalysisJob> {
    const response = await this.fetch(`/bias/jobs/${jobId}`);
    return response.data as AnalysisJob;
  }

  /**
   * Wait for an analysis job to complete
   *
   * @param jobId - Job identifier
   * @param pollInterval - Polling interval in milliseconds (default: 5000)
   * @param maxWaitTime - Maximum wait time in milliseconds (default: 300000)
   * @returns Promise resolving to completed AnalysisJob
   */
  async waitForAnalysis(
    jobId: string,
    pollInterval: number = 5000,
    maxWaitTime: number = 300000
  ): Promise<AnalysisJob> {
    const startTime = Date.now();

    while (Date.now() - startTime < maxWaitTime) {
      const job = await this.getAnalysisJob(jobId);

      if (job.status === 'completed') {
        return job;
      }

      if (job.status === 'failed') {
        throw new Error(`Analysis failed: ${job.error}`);
      }

      if (job.status === 'cancelled') {
        throw new Error('Analysis was cancelled');
      }

      await new Promise((resolve) => setTimeout(resolve, pollInterval));
    }

    throw new Error('Analysis timed out');
  }

  /**
   * Calculate a specific fairness metric
   *
   * @param metricName - Name of the metric to calculate
   * @param data - Prediction data
   * @returns Promise resolving to calculated metric
   */
  async calculateMetric(
    metricName: FairnessMetric,
    data: {
      predictions: number[];
      actualLabels: number[];
      protectedAttribute: (string | number)[];
      groups: Record<string | number, string>;
    }
  ): Promise<any> {
    const response = await this.fetch(`/metrics/${metricName}`, {
      method: 'POST',
      body: JSON.stringify(data),
    });

    return response.data;
  }

  /**
   * Analyze a dataset for bias
   *
   * @param datasetId - Dataset identifier
   * @param checks - Types of checks to perform
   * @returns Promise resolving to dataset analysis results
   */
  async analyzeDataset(
    datasetId: string,
    checks?: string[]
  ): Promise<any> {
    const response = await this.fetch('/datasets/analyze', {
      method: 'POST',
      body: JSON.stringify({
        datasetId,
        protectedAttributes: this.config.protectedAttributes,
        checks: checks || [
          'representation_bias',
          'label_bias',
          'proxy_features',
        ],
      }),
    });

    return response.data;
  }

  /**
   * Get mitigation recommendations
   *
   * @param biasType - Type of bias detected
   * @param phase - Mitigation phase
   * @param context - Additional context
   * @returns Promise resolving to recommendations
   */
  async getMitigationRecommendations(
    biasType: string,
    phase: 'pre-processing' | 'in-processing' | 'post-processing',
    context?: {
      industry?: string;
      useCase?: string;
      currentMetrics?: Record<string, number>;
    }
  ): Promise<any> {
    const response = await this.fetch('/mitigation/recommend', {
      method: 'POST',
      body: JSON.stringify({
        biasType,
        phase,
        context,
      }),
    });

    return response.data;
  }

  /**
   * Helper method for making API requests
   */
  private async fetch(
    endpoint: string,
    options?: RequestInit
  ): Promise<APIResponse<any>> {
    const url = `${this.baseURL}${endpoint}`;

    const response = await fetch(url, {
      ...options,
      headers: {
        'Content-Type': 'application/json',
        Authorization: `Bearer ${this.apiKey}`,
        ...options?.headers,
      },
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(
        error.error?.message || `API request failed: ${response.statusText}`
      );
    }

    return response.json();
  }
}

/**
 * Production monitor for deployed models
 */
export class ProductionMonitor {
  private modelId: string;
  private protectedAttributes: ProtectedAttribute[];
  private apiKey: string;
  private baseURL: string;
  private monitoringId?: string;

  constructor(
    modelId: string,
    protectedAttributes: ProtectedAttribute[],
    apiKey?: string,
    baseURL: string = 'https://api.wia.org/ai-013/v1'
  ) {
    this.modelId = modelId;
    this.protectedAttributes = protectedAttributes;
    this.apiKey = apiKey || process.env.WIA_API_KEY || '';
    this.baseURL = baseURL;
  }

  /**
   * Register model for monitoring
   *
   * @param config - Monitoring configuration
   * @returns Promise resolving to monitoring ID
   */
  async registerModel(config: {
    modelName: string;
    frequency?: 'hourly' | 'daily' | 'weekly';
    metrics?: FairnessMetric[];
    thresholds?: Record<string, number>;
    alertWebhook?: string;
  }): Promise<string> {
    const response = await this.fetch('/monitoring/models', {
      method: 'POST',
      body: JSON.stringify({
        modelId: this.modelId,
        modelName: config.modelName,
        protectedAttributes: this.protectedAttributes,
        monitoringConfig: {
          frequency: config.frequency || 'daily',
          metrics: config.metrics || ['demographic_parity', 'equalized_odds'],
          thresholds: config.thresholds,
          alertWebhook: config.alertWebhook,
        },
      }),
    });

    this.monitoringId = response.data.monitoringId;
    return this.monitoringId;
  }

  /**
   * Log a prediction for monitoring
   *
   * @param prediction - Prediction log entry
   * @returns Promise resolving to success status
   */
  async logPrediction(prediction: PredictionLog): Promise<boolean> {
    await this.fetch('/monitoring/predictions', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-ndjson' },
      body: JSON.stringify(prediction),
    });

    return true;
  }

  /**
   * Get monitoring metrics
   *
   * @param timeRange - Optional time range filter
   * @returns Promise resolving to metrics
   */
  async getMetrics(timeRange?: {
    start: string;
    end: string;
  }): Promise<any> {
    if (!this.monitoringId) {
      throw new Error('Model not registered for monitoring');
    }

    let url = `/monitoring/models/${this.monitoringId}/metrics`;

    if (timeRange) {
      const params = new URLSearchParams({
        start: timeRange.start,
        end: timeRange.end,
      });
      url += `?${params.toString()}`;
    }

    const response = await this.fetch(url);
    return response.data;
  }

  /**
   * Helper method for making API requests
   */
  private async fetch(
    endpoint: string,
    options?: RequestInit
  ): Promise<APIResponse<any>> {
    const url = `${this.baseURL}${endpoint}`;

    const response = await fetch(url, {
      ...options,
      headers: {
        'Content-Type': 'application/json',
        Authorization: `Bearer ${this.apiKey}`,
        ...options?.headers,
      },
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(
        error.error?.message || `API request failed: ${response.statusText}`
      );
    }

    return response.json();
  }
}

/**
 * Utility functions
 */
export const utils = {
  /**
   * Validate a bias detection report against schema
   */
  validateReport(report: BiasDetectionReport): boolean {
    // Basic validation
    if (!report.reportMetadata || !report.reportMetadata.standardVersion) {
      return false;
    }

    if (!report.reportMetadata.standardVersion.startsWith('WIA-AI-013')) {
      return false;
    }

    return true;
  },

  /**
   * Check if fairness criteria are met
   */
  checkFairnessCriteria(
    metrics: FairnessMetrics,
    criteria: Partial<Record<FairnessMetric, number>>
  ): boolean {
    if (criteria.demographic_parity && metrics.demographicParity) {
      if (metrics.demographicParity.value < criteria.demographic_parity) {
        return false;
      }
    }

    if (criteria.equalized_odds && metrics.equalizedOdds) {
      if (
        metrics.equalizedOdds.tprDisparity > criteria.equalized_odds ||
        metrics.equalizedOdds.fprDisparity > criteria.equalized_odds
      ) {
        return false;
      }
    }

    return true;
  },
};

/**
 * Default export
 */
export default BiasDetector;
