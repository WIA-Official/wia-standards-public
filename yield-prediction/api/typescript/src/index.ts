/**
 * WIA-AGRI-007 Yield Prediction Standard - TypeScript SDK
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';
import type {
  SDKConfig,
  YieldPredictionConfig,
  YieldPredictionResult,
  PredictionRequest,
  HistoricalYieldData,
  PredictionModel,
  ModelTrainingRequest,
  ModelValidationResult,
  YieldComparison,
  AnalyticsReport,
  ListParams,
} from './types';

export * from './types';

/**
 * Main SDK Client for WIA-AGRI-007 Yield Prediction
 */
export class YieldPredictionClient {
  private axios: AxiosInstance;
  private config: SDKConfig;

  constructor(config: SDKConfig) {
    this.config = {
      baseURL: config.baseURL || 'https://api.wiastandards.com/v1/yield-prediction',
      timeout: config.timeout || 60000, // Longer timeout for predictions
      ...config,
    };

    this.axios = axios.create({
      baseURL: this.config.baseURL,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'X-API-Key': this.config.apiKey || '',
      },
    });
  }

  /**
   * Create yield prediction
   */
  async createPrediction(
    request: PredictionRequest
  ): Promise<YieldPredictionConfig> {
    const response = await this.axios.post('/predictions', request);
    return response.data;
  }

  /**
   * Get prediction result
   */
  async getPrediction(predictionId: string): Promise<YieldPredictionResult> {
    const response = await this.axios.get(`/predictions/${predictionId}`);
    return response.data;
  }

  /**
   * List predictions
   */
  async listPredictions(params?: ListParams): Promise<YieldPredictionConfig[]> {
    const response = await this.axios.get('/predictions', { params });
    return response.data;
  }

  /**
   * Get prediction status
   */
  async getPredictionStatus(predictionId: string): Promise<{
    status: string;
    progress?: number;
    message?: string;
  }> {
    const response = await this.axios.get(`/predictions/${predictionId}/status`);
    return response.data;
  }

  /**
   * Add historical yield data
   */
  async addHistoricalYield(
    data: Omit<HistoricalYieldData, 'recordId'>
  ): Promise<HistoricalYieldData> {
    const response = await this.axios.post('/historical-yields', data);
    return response.data;
  }

  /**
   * Get historical yields
   */
  async getHistoricalYields(
    fieldId: string,
    params?: ListParams
  ): Promise<HistoricalYieldData[]> {
    const response = await this.axios.get('/historical-yields', {
      params: { fieldId, ...params },
    });
    return response.data;
  }

  /**
   * List available models
   */
  async listModels(cropType?: string): Promise<PredictionModel[]> {
    const response = await this.axios.get('/models', {
      params: { cropType },
    });
    return response.data;
  }

  /**
   * Get model details
   */
  async getModel(modelId: string): Promise<PredictionModel> {
    const response = await this.axios.get(`/models/${modelId}`);
    return response.data;
  }

  /**
   * Train new model
   */
  async trainModel(request: ModelTrainingRequest): Promise<PredictionModel> {
    const response = await this.axios.post('/models/train', request);
    return response.data;
  }

  /**
   * Validate model
   */
  async validateModel(
    modelId: string,
    testData?: any[]
  ): Promise<ModelValidationResult> {
    const response = await this.axios.post(`/models/${modelId}/validate`, {
      testData,
    });
    return response.data;
  }

  /**
   * Compare predictions with actual yields
   */
  async getYieldComparisons(
    fieldId?: string,
    params?: ListParams
  ): Promise<YieldComparison[]> {
    const response = await this.axios.get('/comparisons', {
      params: { fieldId: fieldId || this.config.fieldId, ...params },
    });
    return response.data;
  }

  /**
   * Get analytics report
   */
  async getAnalyticsReport(
    params?: { fieldId?: string; startDate?: string; endDate?: string }
  ): Promise<AnalyticsReport> {
    const response = await this.axios.get('/analytics', { params });
    return response.data;
  }

  /**
   * Quick prediction (simplified)
   */
  async quickPredict(
    fieldId: string,
    cropType: string,
    plantingDate: string,
    expectedHarvestDate: string
  ): Promise<YieldPredictionResult> {
    const config = await this.createPrediction({
      fieldId,
      cropType: cropType as any,
      plantingDate,
      expectedHarvestDate,
      area: 1, // Will be fetched from field data
    });

    // Poll for result
    let attempts = 0;
    while (attempts < 30) {
      const status = await this.getPredictionStatus(config.predictionId);
      if (status.status === 'completed') {
        return this.getPrediction(config.predictionId);
      } else if (status.status === 'failed') {
        throw new Error('Prediction failed');
      }
      await new Promise((resolve) => setTimeout(resolve, 2000));
      attempts++;
    }

    throw new Error('Prediction timeout');
  }
}

/**
 * Model Manager for prediction models
 */
export class ModelManager {
  private client: YieldPredictionClient;

  constructor(config: SDKConfig) {
    this.client = new YieldPredictionClient(config);
  }

  /**
   * Get best model for crop type
   */
  async getBestModel(cropType: string): Promise<PredictionModel> {
    const models = await this.client.listModels(cropType);
    if (models.length === 0) {
      throw new Error('No models available for this crop type');
    }

    // Sort by accuracy
    models.sort((a, b) => (b.accuracy || 0) - (a.accuracy || 0));
    return models[0];
  }

  /**
   * Compare model performance
   */
  async compareModels(modelIds: string[]): Promise<
    Record<
      string,
      {
        model: PredictionModel;
        validation: ModelValidationResult;
      }
    >
  > {
    const results: Record<string, any> = {};

    for (const modelId of modelIds) {
      const [model, validation] = await Promise.all([
        this.client.getModel(modelId),
        this.client.validateModel(modelId),
      ]);

      results[modelId] = { model, validation };
    }

    return results;
  }
}

/**
 * Historical Data Manager
 */
export class HistoricalDataManager {
  private client: YieldPredictionClient;

  constructor(config: SDKConfig) {
    this.client = new YieldPredictionClient(config);
  }

  /**
   * Get yield trend
   */
  async getYieldTrend(
    fieldId: string,
    years: number = 5
  ): Promise<{
    average: number;
    trend: 'increasing' | 'stable' | 'decreasing';
    data: HistoricalYieldData[];
  }> {
    const data = await this.client.getHistoricalYields(fieldId, { limit: years });

    if (data.length === 0) {
      throw new Error('No historical data available');
    }

    const average = data.reduce((sum, d) => sum + d.actualYield, 0) / data.length;

    // Simple trend analysis
    const firstHalf = data.slice(0, Math.floor(data.length / 2));
    const secondHalf = data.slice(Math.floor(data.length / 2));

    const firstAvg =
      firstHalf.reduce((sum, d) => sum + d.actualYield, 0) / firstHalf.length;
    const secondAvg =
      secondHalf.reduce((sum, d) => sum + d.actualYield, 0) / secondHalf.length;

    let trend: 'increasing' | 'stable' | 'decreasing';
    if (secondAvg > firstAvg * 1.05) trend = 'increasing';
    else if (secondAvg < firstAvg * 0.95) trend = 'decreasing';
    else trend = 'stable';

    return { average, trend, data };
  }
}

/**
 * Utility functions
 */
export const utils = {
  /**
   * Calculate yield gap
   */
  calculateYieldGap(actualYield: number, potentialYield: number): number {
    return ((potentialYield - actualYield) / potentialYield) * 100;
  },

  /**
   * Calculate prediction accuracy
   */
  calculateAccuracy(predictedYield: number, actualYield: number): number {
    const error = Math.abs(predictedYield - actualYield);
    const percentageError = (error / actualYield) * 100;
    return Math.max(0, 100 - percentageError);
  },

  /**
   * Convert yield units
   */
  convertYieldUnits(
    value: number,
    from: 'tons/ha' | 'bushels/acre',
    to: 'tons/ha' | 'bushels/acre'
  ): number {
    if (from === to) return value;

    // Conversion factors (approximate for corn)
    if (from === 'tons/ha' && to === 'bushels/acre') {
      return value * 15.9; // 1 ton/ha ≈ 15.9 bu/acre
    } else {
      return value / 15.9;
    }
  },

  /**
   * Classify yield level
   */
  classifyYieldLevel(
    actualYield: number,
    averageYield: number
  ): 'low' | 'average' | 'high' | 'exceptional' {
    const ratio = actualYield / averageYield;

    if (ratio < 0.8) return 'low';
    if (ratio < 1.1) return 'average';
    if (ratio < 1.3) return 'high';
    return 'exceptional';
  },
};

/**
 * Default export
 */
export default {
  YieldPredictionClient,
  ModelManager,
  HistoricalDataManager,
  utils,
};
