/**
 * WIA-IND-026: Predictive Maintenance SDK
 *
 * Comprehensive predictive maintenance solution with multi-modal sensor analysis,
 * ML-powered failure prediction, and maintenance optimization.
 *
 * @version 1.0.0
 * @standard WIA-IND-026
 * @category IND (Industry)
 *
 * @example
 * ```typescript
 * import { PredictiveMaintenanceSDK } from '@wia/ind-026';
 *
 * const pdm = new PredictiveMaintenanceSDK({
 *   apiKey: 'your-api-key',
 *   organizationId: 'your-org-id'
 * });
 *
 * // Register an asset
 * await pdm.assets.register({
 *   assetId: 'MOTOR-001',
 *   assetType: 'ROTATING_MACHINERY',
 *   location: 'Plant-A-Line-1'
 * });
 *
 * // Get predictions
 * const predictions = await pdm.predictions.analyze({
 *   assetId: 'MOTOR-001',
 *   timeHorizon: 30
 * });
 * ```
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import EventEmitter from 'eventemitter3';
import WebSocket from 'ws';
import FFT from 'fft.js';
import {
  PredictiveMaintenanceConfig,
  Asset,
  AssetRegisterRequest,
  Sensor,
  SensorAttachRequest,
  DataIngestRequest,
  DataStreamOptions,
  VibrationAnalysisRequest,
  VibrationAnalysisResult,
  ThermalAnalysisRequest,
  ThermalAnalysisResult,
  OilAnalysisRequest,
  OilAnalysisResult,
  AcousticAnalysisRequest,
  AcousticAnalysisResult,
  PredictionRequest,
  PredictionResult,
  CreateWorkOrderRequest,
  WorkOrder,
  OptimizeScheduleRequest,
  OptimizeScheduleResult,
  InventoryCheckResult,
  ApiResponse,
  PaginatedResponse,
  PredictiveMaintenanceError,
  AuthenticationError,
  NotFoundError,
  RateLimitError,
  ValidationError,
  BearingInformation,
  AnalysisType,
  DataPoint,
  DataQuality
} from './types';

// Re-export types for convenience
export * from './types';

/**
 * Main SDK class for WIA-IND-026 Predictive Maintenance
 */
export class PredictiveMaintenanceSDK extends EventEmitter {
  private client: AxiosInstance;
  private config: Required<PredictiveMaintenanceConfig>;
  private wsConnections: Map<string, WebSocket> = new Map();

  public readonly assets: AssetManager;
  public readonly sensors: SensorManager;
  public readonly data: DataManager;
  public readonly analysis: AnalysisManager;
  public readonly predictions: PredictionManager;
  public readonly maintenance: MaintenanceManager;
  public readonly inventory: InventoryManager;

  constructor(config: PredictiveMaintenanceConfig) {
    super();

    // Set defaults
    this.config = {
      ...config,
      endpoint: config.endpoint || 'https://api.wia.org/ind-026/v1',
      timeout: config.timeout || 30000,
      retry: config.retry || {
        maxRetries: 3,
        retryDelay: 1000,
        exponentialBackoff: true
      },
      debug: config.debug || false
    };

    // Create HTTP client
    this.client = axios.create({
      baseURL: this.config.endpoint,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'X-API-Key': this.config.apiKey,
        'X-Organization-ID': this.config.organizationId,
        'User-Agent': 'WIA-IND-026-SDK/1.0.0'
      }
    });

    // Add request interceptor for retry logic
    this.setupInterceptors();

    // Initialize managers
    this.assets = new AssetManager(this);
    this.sensors = new SensorManager(this);
    this.data = new DataManager(this);
    this.analysis = new AnalysisManager(this);
    this.predictions = new PredictionManager(this);
    this.maintenance = new MaintenanceManager(this);
    this.inventory = new InventoryManager(this);
  }

  /**
   * Get the underlying HTTP client
   */
  getClient(): AxiosInstance {
    return this.client;
  }

  /**
   * Get the SDK configuration
   */
  getConfig(): Required<PredictiveMaintenanceConfig> {
    return { ...this.config };
  }

  /**
   * Close all connections and cleanup
   */
  async close(): Promise<void> {
    // Close all WebSocket connections
    for (const [id, ws] of this.wsConnections) {
      ws.close();
      this.wsConnections.delete(id);
    }

    this.emit('close');
  }

  /**
   * Setup request/response interceptors
   */
  private setupInterceptors(): void {
    // Request interceptor
    this.client.interceptors.request.use(
      (config) => {
        if (this.config.debug) {
          console.log(`[WIA-IND-026] ${config.method?.toUpperCase()} ${config.url}`);
        }
        return config;
      },
      (error) => Promise.reject(error)
    );

    // Response interceptor
    this.client.interceptors.response.use(
      (response) => {
        if (this.config.debug) {
          console.log(`[WIA-IND-026] Response: ${response.status}`);
        }
        return response;
      },
      async (error: AxiosError) => {
        return this.handleError(error);
      }
    );
  }

  /**
   * Handle API errors
   */
  private async handleError(error: AxiosError): Promise<never> {
    if (error.response) {
      const status = error.response.status;
      const data = error.response.data as any;

      switch (status) {
        case 401:
          throw new AuthenticationError(data?.error?.message);
        case 404:
          throw new NotFoundError('Resource', data?.error?.details?.id || 'unknown');
        case 429:
          throw new RateLimitError(
            data?.error?.details?.limit || 0,
            data?.error?.details?.reset || 0
          );
        case 400:
          throw new ValidationError(data?.error?.message, data?.error?.details);
        default:
          throw new PredictiveMaintenanceError(
            data?.error?.message || 'API request failed',
            data?.error?.code || 'UNKNOWN_ERROR',
            status,
            data?.error?.details
          );
      }
    } else if (error.request) {
      throw new PredictiveMaintenanceError(
        'No response from server',
        'NETWORK_ERROR'
      );
    } else {
      throw new PredictiveMaintenanceError(
        error.message,
        'REQUEST_ERROR'
      );
    }
  }

  /**
   * Create WebSocket connection
   */
  createWebSocket(path: string): WebSocket {
    const wsUrl = this.config.endpoint.replace(/^http/, 'ws') + path;
    const ws = new WebSocket(wsUrl, {
      headers: {
        'X-API-Key': this.config.apiKey,
        'X-Organization-ID': this.config.organizationId
      }
    });

    const id = Math.random().toString(36).substring(7);
    this.wsConnections.set(id, ws);

    ws.on('close', () => {
      this.wsConnections.delete(id);
    });

    return ws;
  }
}

/**
 * Asset Management
 */
class AssetManager {
  constructor(private sdk: PredictiveMaintenanceSDK) {}

  /**
   * Register a new asset
   */
  async register(request: AssetRegisterRequest): Promise<Asset> {
    const response = await this.sdk.getClient().post<ApiResponse<Asset>>(
      '/assets',
      request
    );
    return response.data.data!;
  }

  /**
   * Get asset by ID
   */
  async get(assetId: string): Promise<Asset> {
    const response = await this.sdk.getClient().get<ApiResponse<Asset>>(
      `/assets/${assetId}`
    );
    return response.data.data!;
  }

  /**
   * Update asset
   */
  async update(assetId: string, updates: Partial<Asset>): Promise<Asset> {
    const response = await this.sdk.getClient().put<ApiResponse<Asset>>(
      `/assets/${assetId}`,
      updates
    );
    return response.data.data!;
  }

  /**
   * Delete asset
   */
  async delete(assetId: string): Promise<void> {
    await this.sdk.getClient().delete(`/assets/${assetId}`);
  }

  /**
   * List assets with optional filters
   */
  async list(filters?: {
    type?: string;
    location?: string;
    status?: string;
    page?: number;
    pageSize?: number;
  }): Promise<PaginatedResponse<Asset>> {
    const response = await this.sdk.getClient().get<ApiResponse<PaginatedResponse<Asset>>>(
      '/assets',
      { params: filters }
    );
    return response.data.data!;
  }
}

/**
 * Sensor Management
 */
class SensorManager {
  constructor(private sdk: PredictiveMaintenanceSDK) {}

  /**
   * Attach sensors to an asset
   */
  async attach(request: SensorAttachRequest): Promise<Sensor[]> {
    const response = await this.sdk.getClient().post<ApiResponse<Sensor[]>>(
      `/assets/${request.assetId}/sensors`,
      request
    );
    return response.data.data!;
  }

  /**
   * Get sensor by ID
   */
  async get(sensorId: string): Promise<Sensor> {
    const response = await this.sdk.getClient().get<ApiResponse<Sensor>>(
      `/sensors/${sensorId}`
    );
    return response.data.data!;
  }

  /**
   * Update sensor
   */
  async update(sensorId: string, updates: Partial<Sensor>): Promise<Sensor> {
    const response = await this.sdk.getClient().put<ApiResponse<Sensor>>(
      `/sensors/${sensorId}`,
      updates
    );
    return response.data.data!;
  }

  /**
   * Detach sensor
   */
  async detach(sensorId: string): Promise<void> {
    await this.sdk.getClient().delete(`/sensors/${sensorId}`);
  }

  /**
   * List sensors for an asset
   */
  async listByAsset(assetId: string): Promise<Sensor[]> {
    const response = await this.sdk.getClient().get<ApiResponse<Sensor[]>>(
      `/assets/${assetId}/sensors`
    );
    return response.data.data!;
  }
}

/**
 * Data Management
 */
class DataManager {
  constructor(private sdk: PredictiveMaintenanceSDK) {}

  /**
   * Ingest sensor data
   */
  async ingest(request: DataIngestRequest): Promise<void> {
    await this.sdk.getClient().post('/data/ingest', request);
  }

  /**
   * Batch ingest sensor data
   */
  async ingestBatch(requests: DataIngestRequest[]): Promise<void> {
    await this.sdk.getClient().post('/data/ingest/batch', { data: requests });
  }

  /**
   * Stream real-time sensor data
   */
  stream(options: DataStreamOptions): () => void {
    const ws = this.sdk.createWebSocket('/data/stream');

    ws.on('open', () => {
      ws.send(JSON.stringify({
        action: 'subscribe',
        assetId: options.assetId,
        sensorIds: options.sensorIds
      }));
    });

    ws.on('message', async (data: Buffer) => {
      try {
        const message = JSON.parse(data.toString());
        await options.onData(message as DataPoint);
      } catch (error) {
        if (options.onError) {
          options.onError(error as Error);
        }
      }
    });

    ws.on('error', (error) => {
      if (options.onError) {
        options.onError(error);
      }
    });

    ws.on('close', () => {
      if (options.onClose) {
        options.onClose();
      }
    });

    // Return cleanup function
    return () => ws.close();
  }

  /**
   * Query historical data
   */
  async query(params: {
    sensorId: string;
    startTime: number;
    endTime: number;
    aggregation?: 'raw' | '1m' | '5m' | '1h';
  }): Promise<DataPoint[]> {
    const response = await this.sdk.getClient().get<ApiResponse<DataPoint[]>>(
      '/data/query',
      { params }
    );
    return response.data.data!;
  }
}

/**
 * Analysis Management
 */
class AnalysisManager {
  constructor(private sdk: PredictiveMaintenanceSDK) {}

  /**
   * Perform vibration analysis
   */
  async vibration(request: VibrationAnalysisRequest): Promise<VibrationAnalysisResult> {
    const response = await this.sdk.getClient().post<ApiResponse<VibrationAnalysisResult>>(
      '/analysis/vibration',
      request
    );
    return response.data.data!;
  }

  /**
   * Perform thermal analysis
   */
  async thermal(request: ThermalAnalysisRequest): Promise<ThermalAnalysisResult> {
    const response = await this.sdk.getClient().post<ApiResponse<ThermalAnalysisResult>>(
      '/analysis/thermal',
      request
    );
    return response.data.data!;
  }

  /**
   * Perform oil analysis
   */
  async oil(request: OilAnalysisRequest): Promise<OilAnalysisResult> {
    const response = await this.sdk.getClient().post<ApiResponse<OilAnalysisResult>>(
      '/analysis/oil',
      request
    );
    return response.data.data!;
  }

  /**
   * Perform acoustic analysis
   */
  async acoustic(request: AcousticAnalysisRequest): Promise<AcousticAnalysisResult> {
    const response = await this.sdk.getClient().post<ApiResponse<AcousticAnalysisResult>>(
      '/analysis/acoustic',
      request
    );
    return response.data.data!;
  }

  /**
   * Calculate bearing fault frequencies
   */
  calculateBearingFrequencies(bearing: BearingInformation): {
    BPFO: number;
    BPFI: number;
    BSF: number;
    FTF: number;
  } {
    const fr = bearing.shaftSpeed / 60; // Convert RPM to Hz
    const ratio = bearing.ballDiameter / bearing.pitchDiameter;
    const cosAlpha = Math.cos(bearing.contactAngle * Math.PI / 180);

    const BPFO = (bearing.numberOfBalls / 2) * (1 - ratio * cosAlpha) * fr;
    const BPFI = (bearing.numberOfBalls / 2) * (1 + ratio * cosAlpha) * fr;
    const BSF = (bearing.pitchDiameter / (2 * bearing.ballDiameter)) *
                (1 - Math.pow(ratio * cosAlpha, 2)) * fr;
    const FTF = (1 / 2) * (1 - ratio * cosAlpha) * fr;

    return { BPFO, BPFI, BSF, FTF };
  }

  /**
   * Perform FFT on time-domain data
   */
  performFFT(data: number[], samplingRate: number): {
    frequencies: number[];
    amplitudes: number[];
  } {
    const n = data.length;
    const fft = new FFT(n);
    const out = fft.createComplexArray();
    fft.realTransform(out, data);

    const frequencies: number[] = [];
    const amplitudes: number[] = [];

    for (let i = 0; i < n / 2; i++) {
      frequencies.push(i * samplingRate / n);
      const real = out[2 * i];
      const imag = out[2 * i + 1];
      amplitudes.push(Math.sqrt(real * real + imag * imag) / n);
    }

    return { frequencies, amplitudes };
  }

  /**
   * Calculate RMS value
   */
  calculateRMS(data: number[]): number {
    const sumSquares = data.reduce((sum, val) => sum + val * val, 0);
    return Math.sqrt(sumSquares / data.length);
  }

  /**
   * Calculate peak value
   */
  calculatePeak(data: number[]): number {
    return Math.max(...data.map(Math.abs));
  }

  /**
   * Calculate crest factor
   */
  calculateCrestFactor(data: number[]): number {
    const peak = this.calculatePeak(data);
    const rms = this.calculateRMS(data);
    return peak / rms;
  }

  /**
   * Calculate kurtosis
   */
  calculateKurtosis(data: number[]): number {
    const n = data.length;
    const mean = data.reduce((sum, val) => sum + val, 0) / n;
    const variance = data.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / n;
    const stdDev = Math.sqrt(variance);
    const fourthMoment = data.reduce((sum, val) =>
      sum + Math.pow((val - mean) / stdDev, 4), 0) / n;

    return fourthMoment;
  }
}

/**
 * Prediction Management
 */
class PredictionManager {
  constructor(private sdk: PredictiveMaintenanceSDK) {}

  /**
   * Analyze and get failure predictions
   */
  async analyze(request: PredictionRequest): Promise<PredictionResult> {
    const response = await this.sdk.getClient().post<ApiResponse<PredictionResult>>(
      '/predictions/analyze',
      request
    );
    return response.data.data!;
  }

  /**
   * Get remaining useful life estimate
   */
  async getRUL(assetId: string): Promise<{
    days: number;
    confidence: number;
    lowerBound: number;
    upperBound: number;
  }> {
    const response = await this.sdk.getClient().get<ApiResponse<any>>(
      `/predictions/rul`,
      { params: { assetId } }
    );
    return response.data.data!;
  }

  /**
   * Get prediction history
   */
  async getHistory(assetId: string, days: number = 30): Promise<PredictionResult[]> {
    const response = await this.sdk.getClient().get<ApiResponse<PredictionResult[]>>(
      `/predictions/history`,
      { params: { assetId, days } }
    );
    return response.data.data!;
  }
}

/**
 * Maintenance Management
 */
class MaintenanceManager {
  constructor(private sdk: PredictiveMaintenanceSDK) {}

  /**
   * Create work order
   */
  async createWorkOrder(request: CreateWorkOrderRequest): Promise<WorkOrder> {
    const response = await this.sdk.getClient().post<ApiResponse<WorkOrder>>(
      '/maintenance/work-orders',
      request
    );
    return response.data.data!;
  }

  /**
   * Get work order by ID
   */
  async getWorkOrder(workOrderId: string): Promise<WorkOrder> {
    const response = await this.sdk.getClient().get<ApiResponse<WorkOrder>>(
      `/maintenance/work-orders/${workOrderId}`
    );
    return response.data.data!;
  }

  /**
   * Update work order
   */
  async updateWorkOrder(workOrderId: string, updates: Partial<WorkOrder>): Promise<WorkOrder> {
    const response = await this.sdk.getClient().put<ApiResponse<WorkOrder>>(
      `/maintenance/work-orders/${workOrderId}`,
      updates
    );
    return response.data.data!;
  }

  /**
   * List work orders
   */
  async listWorkOrders(filters?: {
    status?: string;
    assetId?: string;
    priority?: string;
    startDate?: Date;
    endDate?: Date;
  }): Promise<WorkOrder[]> {
    const response = await this.sdk.getClient().get<ApiResponse<WorkOrder[]>>(
      '/maintenance/work-orders',
      { params: filters }
    );
    return response.data.data!;
  }

  /**
   * Optimize maintenance schedule
   */
  async optimize(request: OptimizeScheduleRequest): Promise<OptimizeScheduleResult> {
    const response = await this.sdk.getClient().post<ApiResponse<OptimizeScheduleResult>>(
      '/maintenance/optimize',
      request
    );
    return response.data.data!;
  }

  /**
   * Complete work order
   */
  async completeWorkOrder(workOrderId: string, notes?: string): Promise<WorkOrder> {
    return this.updateWorkOrder(workOrderId, {
      status: 'COMPLETED',
      completedDate: new Date(),
      notes
    });
  }
}

/**
 * Inventory Management
 */
class InventoryManager {
  constructor(private sdk: PredictiveMaintenanceSDK) {}

  /**
   * Check inventory status
   */
  async check(): Promise<InventoryCheckResult> {
    const response = await this.sdk.getClient().get<ApiResponse<InventoryCheckResult>>(
      '/inventory/check'
    );
    return response.data.data!;
  }

  /**
   * Update spare part quantity
   */
  async updateQuantity(partNumber: string, quantity: number): Promise<void> {
    await this.sdk.getClient().put(`/inventory/parts/${partNumber}`, { quantity });
  }

  /**
   * Forecast spare parts demand
   */
  async forecastDemand(days: number = 90): Promise<Record<string, number>> {
    const response = await this.sdk.getClient().get<ApiResponse<Record<string, number>>>(
      '/inventory/forecast',
      { params: { days } }
    );
    return response.data.data!;
  }
}

/**
 * Utility functions
 */

/**
 * Validate sensor data quality
 */
export function validateDataQuality(
  data: number | number[],
  options: {
    minValue?: number;
    maxValue?: number;
    sigmaThreshold?: number;
  } = {}
): DataQuality {
  const values = Array.isArray(data) ? data : [data];

  // Range check
  if (options.minValue !== undefined || options.maxValue !== undefined) {
    for (const val of values) {
      if (options.minValue !== undefined && val < options.minValue) {
        return DataQuality.BAD;
      }
      if (options.maxValue !== undefined && val > options.maxValue) {
        return DataQuality.BAD;
      }
    }
  }

  // Statistical outlier detection
  if (options.sigmaThreshold && values.length > 3) {
    const mean = values.reduce((sum, val) => sum + val, 0) / values.length;
    const variance = values.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / values.length;
    const stdDev = Math.sqrt(variance);

    for (const val of values) {
      if (Math.abs(val - mean) > options.sigmaThreshold * stdDev) {
        return DataQuality.UNCERTAIN;
      }
    }
  }

  return DataQuality.GOOD;
}

/**
 * Convert ISO cleanliness code to particle counts
 */
export function parseISOCode(isoCode: string): {
  '4μm': number;
  '6μm': number;
  '14μm': number;
} {
  const parts = isoCode.split('/').map(p => parseInt(p));
  if (parts.length !== 3) {
    throw new Error('Invalid ISO cleanliness code');
  }

  return {
    '4μm': Math.pow(2, parts[0] - 13) * 100,
    '6μm': Math.pow(2, parts[1] - 13) * 100,
    '14μm': Math.pow(2, parts[2] - 13) * 100
  };
}

/**
 * Convert particle counts to ISO cleanliness code
 */
export function toISOCode(particleCounts: {
  '4μm': number;
  '6μm': number;
  '14μm': number;
}): string {
  const code4 = Math.round(Math.log2(particleCounts['4μm'] / 100) + 13);
  const code6 = Math.round(Math.log2(particleCounts['6μm'] / 100) + 13);
  const code14 = Math.round(Math.log2(particleCounts['14μm'] / 100) + 13);

  return `${code4}/${code6}/${code14}`;
}

// Default export
export default PredictiveMaintenanceSDK;

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
