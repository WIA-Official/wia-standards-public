/**
 * WIA-MICROPLASTIC_DETECTION SDK
 *
 * Comprehensive TypeScript SDK for microplastic detection and analysis
 *
 * @module @wia/microplastic-detection
 * @version 1.0.0
 * @philosophy 弘益人間 (Benefit All Humanity)
 *
 * @example
 * ```typescript
 * import { MicroplasticDetectionSDK } from '@wia/microplastic-detection';
 *
 * const sdk = new MicroplasticDetectionSDK({
 *   apiKey: 'your_api_key',
 *   environment: 'production'
 * });
 *
 * // Create and analyze a sample
 * const sample = await sdk.samples.create({
 *   sampleName: 'Pacific Ocean Sample',
 *   location: { latitude: 34.0, longitude: -118.5 },
 *   // ... other properties
 * });
 *
 * const job = await sdk.samples.analyze(sample.sampleId);
 * const results = await sdk.jobs.waitForCompletion(job.jobId);
 * ```
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import { EventEmitter } from 'eventemitter3';
import * as WebSocket from 'ws';
import {
  SDKConfig,
  MicroplasticSample,
  MicroplasticParticle,
  DetectionResult,
  AnalysisJob,
  AnalysisJobConfig,
  SensorReading,
  SensorMetadata,
  ReportConfig,
  ReportMetadata,
  RamanSpectrum,
  FTIRSpectrum,
  FluorescenceData,
  PaginatedResponse,
  APIError,
  JobStatus,
  PolymerCode,
  WebhookConfig
} from './types';

// Export all types
export * from './types';

// ============================================================================
// Constants
// ============================================================================

const DEFAULT_CONFIG: Partial<SDKConfig> = {
  baseUrl: 'https://api.wia.org/microplastic-detection/v1',
  timeout: 30000,
  retryAttempts: 3,
  enableWebSocket: false,
  environment: 'production'
};

const BASE_URLS: Record<string, string> = {
  production: 'https://api.wia.org/microplastic-detection/v1',
  staging: 'https://staging-api.wia.org/microplastic-detection/v1',
  development: 'https://dev-api.wia.org/microplastic-detection/v1'
};

// ============================================================================
// Error Classes
// ============================================================================

/**
 * Base error class for SDK errors
 */
export class MicroplasticSDKError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number,
    public details?: any
  ) {
    super(message);
    this.name = 'MicroplasticSDKError';
  }
}

/**
 * Authentication error
 */
export class AuthenticationError extends MicroplasticSDKError {
  constructor(message: string) {
    super(message, 'AUTHENTICATION_ERROR', 401);
    this.name = 'AuthenticationError';
  }
}

/**
 * Validation error
 */
export class ValidationError extends MicroplasticSDKError {
  constructor(message: string, details?: any) {
    super(message, 'VALIDATION_ERROR', 400, details);
    this.name = 'ValidationError';
  }
}

/**
 * Not found error
 */
export class NotFoundError extends MicroplasticSDKError {
  constructor(resource: string, id: string) {
    super(`${resource} with ID '${id}' not found`, 'NOT_FOUND', 404);
    this.name = 'NotFoundError';
  }
}

/**
 * Rate limit error
 */
export class RateLimitError extends MicroplasticSDKError {
  constructor(public retryAfter: number) {
    super(
      `Rate limit exceeded. Retry after ${retryAfter} seconds`,
      'RATE_LIMIT_EXCEEDED',
      429
    );
    this.name = 'RateLimitError';
  }
}

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Handle API errors
 */
function handleAPIError(error: AxiosError): never {
  if (error.response) {
    const { status, data } = error.response;
    const apiError = data as APIError;

    switch (status) {
      case 401:
        throw new AuthenticationError(apiError.message || 'Authentication failed');
      case 404:
        throw new NotFoundError('Resource', 'unknown');
      case 429:
        const retryAfter = parseInt(error.response.headers['retry-after'] || '60');
        throw new RateLimitError(retryAfter);
      case 400:
        throw new ValidationError(apiError.message || 'Validation failed', apiError.details);
      default:
        throw new MicroplasticSDKError(
          apiError.message || 'API request failed',
          apiError.code || 'UNKNOWN_ERROR',
          status,
          apiError.details
        );
    }
  } else if (error.request) {
    throw new MicroplasticSDKError(
      'No response from server',
      'NETWORK_ERROR'
    );
  } else {
    throw new MicroplasticSDKError(
      error.message,
      'REQUEST_ERROR'
    );
  }
}

/**
 * Delay helper for retry logic
 */
function delay(ms: number): Promise<void> {
  return new Promise(resolve => setTimeout(resolve, ms));
}

// ============================================================================
// Sample Management Service
// ============================================================================

/**
 * Sample management service
 */
export class SampleService {
  constructor(private client: AxiosInstance) {}

  /**
   * Create a new sample
   */
  async create(sample: Partial<MicroplasticSample>): Promise<MicroplasticSample> {
    try {
      const response = await this.client.post('/samples', sample);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Get a sample by ID
   */
  async get(sampleId: string): Promise<MicroplasticSample> {
    try {
      const response = await this.client.get(`/samples/${sampleId}`);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Get sample by external ID
   */
  async getByExternalId(externalId: string): Promise<MicroplasticSample> {
    try {
      const response = await this.client.get('/samples', {
        params: { externalId }
      });
      return response.data.samples[0];
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * List samples with pagination
   */
  async list(params?: {
    limit?: number;
    offset?: number;
    status?: string;
    collectionId?: string;
    environmentType?: string;
    startDate?: string;
    endDate?: string;
  }): Promise<PaginatedResponse<MicroplasticSample>> {
    try {
      const response = await this.client.get('/samples', { params });
      return {
        data: response.data.samples,
        total: response.data.total,
        limit: response.data.limit,
        offset: response.data.offset,
        links: response.data.links
      };
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Update a sample
   */
  async update(sampleId: string, updates: Partial<MicroplasticSample>): Promise<MicroplasticSample> {
    try {
      const response = await this.client.patch(`/samples/${sampleId}`, updates);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Delete a sample
   */
  async delete(sampleId: string): Promise<void> {
    try {
      await this.client.delete(`/samples/${sampleId}`);
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Submit sample for analysis
   */
  async analyze(sampleId: string, config?: Partial<AnalysisJobConfig>): Promise<AnalysisJob> {
    try {
      const response = await this.client.post(`/samples/${sampleId}/analyze`, config);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Get analysis results for a sample
   */
  async getResults(sampleId: string): Promise<DetectionResult> {
    try {
      const response = await this.client.get(`/samples/${sampleId}/results`);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Get particles from a sample
   */
  async getParticles(
    sampleId: string,
    params?: {
      limit?: number;
      offset?: number;
      polymerType?: PolymerCode;
      shapeType?: string;
      minSize?: number;
      maxSize?: number;
      verified?: boolean;
    }
  ): Promise<PaginatedResponse<MicroplasticParticle>> {
    try {
      const response = await this.client.get(`/samples/${sampleId}/particles`, { params });
      return {
        data: response.data.particles,
        total: response.data.total,
        limit: response.data.limit,
        offset: response.data.offset,
        links: response.data.links
      };
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Download sample report
   */
  async downloadReport(sampleId: string, format: 'PDF' | 'HTML' | 'CSV' = 'PDF'): Promise<Buffer> {
    try {
      const response = await this.client.get(`/samples/${sampleId}/report`, {
        params: { format },
        responseType: 'arraybuffer'
      });
      return Buffer.from(response.data);
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }
}

// ============================================================================
// Particle Service
// ============================================================================

/**
 * Particle management service
 */
export class ParticleService {
  constructor(private client: AxiosInstance) {}

  /**
   * Get a single particle by ID
   */
  async get(particleId: string): Promise<MicroplasticParticle> {
    try {
      const response = await this.client.get(`/particles/${particleId}`);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Update particle information
   */
  async update(particleId: string, updates: Partial<MicroplasticParticle>): Promise<MicroplasticParticle> {
    try {
      const response = await this.client.patch(`/particles/${particleId}`, updates);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Verify a particle
   */
  async verify(particleId: string, verifiedBy: string, notes?: string): Promise<MicroplasticParticle> {
    return this.update(particleId, {
      verified: true,
      verifiedBy,
      verificationNotes: notes
    });
  }

  /**
   * Get particle image
   */
  async getImage(particleId: string, imageType: 'optical' | 'fluorescence' | 'sem' = 'optical'): Promise<Buffer> {
    try {
      const response = await this.client.get(`/particles/${particleId}/images/${imageType}`, {
        responseType: 'arraybuffer'
      });
      return Buffer.from(response.data);
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }
}

// ============================================================================
// Spectroscopy Service
// ============================================================================

/**
 * Spectroscopy service
 */
export class SpectroscopyService {
  constructor(private client: AxiosInstance) {}

  /**
   * Get Raman spectrum for a particle
   */
  async getRamanSpectrum(particleId: string): Promise<RamanSpectrum> {
    try {
      const response = await this.client.get(`/particles/${particleId}/spectra/raman`);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Upload Raman spectrum
   */
  async uploadRamanSpectrum(particleId: string, spectrum: Partial<RamanSpectrum>): Promise<RamanSpectrum> {
    try {
      const response = await this.client.post(`/particles/${particleId}/spectra/raman`, spectrum);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Get FTIR spectrum for a particle
   */
  async getFTIRSpectrum(particleId: string): Promise<FTIRSpectrum> {
    try {
      const response = await this.client.get(`/particles/${particleId}/spectra/ftir`);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Upload FTIR spectrum
   */
  async uploadFTIRSpectrum(particleId: string, spectrum: Partial<FTIRSpectrum>): Promise<FTIRSpectrum> {
    try {
      const response = await this.client.post(`/particles/${particleId}/spectra/ftir`, spectrum);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Identify polymer from spectrum
   */
  async identifyPolymer(spectrum: {
    spectrumType: 'RAMAN' | 'FTIR';
    wavenumbers: number[];
    intensities: number[];
    spectralLibrary?: string;
  }): Promise<{
    matches: Array<{
      polymerType: PolymerCode;
      confidence: number;
      hitQuality: number;
      libraryEntry: string;
    }>;
    bestMatch: {
      polymerType: PolymerCode;
      confidence: number;
    };
  }> {
    try {
      const response = await this.client.post('/spectra/identify', spectrum);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Get fluorescence data
   */
  async getFluorescenceData(particleId: string): Promise<FluorescenceData> {
    try {
      const response = await this.client.get(`/particles/${particleId}/fluorescence`);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }
}

// ============================================================================
// Job Service
// ============================================================================

/**
 * Analysis job service
 */
export class JobService {
  constructor(private client: AxiosInstance) {}

  /**
   * Get job status
   */
  async getStatus(jobId: string): Promise<AnalysisJob> {
    try {
      const response = await this.client.get(`/jobs/${jobId}`);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Cancel a job
   */
  async cancel(jobId: string): Promise<void> {
    try {
      await this.client.post(`/jobs/${jobId}/cancel`);
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Wait for job completion
   */
  async waitForCompletion(
    jobId: string,
    options?: {
      pollInterval?: number;
      timeout?: number;
      onProgress?: (job: AnalysisJob) => void;
    }
  ): Promise<AnalysisJob> {
    const pollInterval = options?.pollInterval || 5000;
    const timeout = options?.timeout || 3600000; // 1 hour default
    const startTime = Date.now();

    while (true) {
      const job = await this.getStatus(jobId);

      if (options?.onProgress) {
        options.onProgress(job);
      }

      if (job.status === JobStatus.COMPLETED) {
        return job;
      }

      if (job.status === JobStatus.FAILED) {
        throw new MicroplasticSDKError(
          `Job ${jobId} failed: ${job.errorMessage}`,
          'JOB_FAILED'
        );
      }

      if (job.status === JobStatus.CANCELLED) {
        throw new MicroplasticSDKError(
          `Job ${jobId} was cancelled`,
          'JOB_CANCELLED'
        );
      }

      if (Date.now() - startTime > timeout) {
        throw new MicroplasticSDKError(
          `Job ${jobId} timeout after ${timeout}ms`,
          'JOB_TIMEOUT'
        );
      }

      await delay(pollInterval);
    }
  }
}

// ============================================================================
// Sensor Service
// ============================================================================

/**
 * Sensor monitoring service
 */
export class SensorService {
  constructor(private client: AxiosInstance) {}

  /**
   * Submit sensor reading
   */
  async submitReading(sensorId: string, reading: Omit<SensorReading, 'sensorId'>): Promise<SensorReading> {
    try {
      const response = await this.client.post(`/sensors/${sensorId}/readings`, reading);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Get sensor readings
   */
  async getReadings(
    sensorId: string,
    params?: {
      startDate?: string;
      endDate?: string;
      limit?: number;
      offset?: number;
    }
  ): Promise<PaginatedResponse<SensorReading>> {
    try {
      const response = await this.client.get(`/sensors/${sensorId}/readings`, { params });
      return {
        data: response.data.readings,
        total: response.data.total,
        limit: params?.limit || 50,
        offset: params?.offset || 0,
        links: {}
      };
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Get sensor metadata
   */
  async getMetadata(sensorId: string): Promise<SensorMetadata> {
    try {
      const response = await this.client.get(`/sensors/${sensorId}`);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * List all sensors
   */
  async list(params?: {
    status?: string;
    location?: string;
  }): Promise<SensorMetadata[]> {
    try {
      const response = await this.client.get('/sensors', { params });
      return response.data.sensors;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }
}

// ============================================================================
// Report Service
// ============================================================================

/**
 * Report generation service
 */
export class ReportService {
  constructor(private client: AxiosInstance) {}

  /**
   * Generate a report
   */
  async generate(config: ReportConfig): Promise<ReportMetadata> {
    try {
      const response = await this.client.post('/reports/generate', config);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Get report status
   */
  async getStatus(reportId: string): Promise<ReportMetadata> {
    try {
      const response = await this.client.get(`/reports/${reportId}`);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Download report
   */
  async download(reportId: string): Promise<Buffer> {
    try {
      const response = await this.client.get(`/reports/${reportId}/download`, {
        responseType: 'arraybuffer'
      });
      return Buffer.from(response.data);
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Wait for report generation
   */
  async waitForCompletion(reportId: string, pollInterval: number = 2000): Promise<ReportMetadata> {
    while (true) {
      const report = await this.getStatus(reportId);

      if (report.status === 'COMPLETED') {
        return report;
      }

      if (report.status === 'FAILED') {
        throw new MicroplasticSDKError(
          `Report generation failed for ${reportId}`,
          'REPORT_GENERATION_FAILED'
        );
      }

      await delay(pollInterval);
    }
  }
}

// ============================================================================
// Image Analysis Service
// ============================================================================

/**
 * Image analysis service
 */
export class ImageService {
  constructor(private client: AxiosInstance) {}

  /**
   * Upload and analyze image
   */
  async analyze(
    imageData: Buffer,
    metadata: {
      sampleId: string;
      imageType: 'optical' | 'fluorescence' | 'sem';
      magnification?: number;
      pixelSize?: number;
    }
  ): Promise<{ jobId: string; status: string }> {
    try {
      const formData = new FormData();
      formData.append('image', new Blob([imageData]));
      formData.append('sampleId', metadata.sampleId);
      formData.append('imageType', metadata.imageType);
      if (metadata.magnification) formData.append('magnification', metadata.magnification.toString());
      if (metadata.pixelSize) formData.append('pixelSize', metadata.pixelSize.toString());

      const response = await this.client.post('/images/analyze', formData, {
        headers: { 'Content-Type': 'multipart/form-data' }
      });
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Get image analysis results
   */
  async getResults(jobId: string): Promise<any> {
    try {
      const response = await this.client.get(`/images/analyze/${jobId}/results`);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }
}

// ============================================================================
// Webhook Service
// ============================================================================

/**
 * Webhook management service
 */
export class WebhookService {
  constructor(private client: AxiosInstance) {}

  /**
   * Register a webhook
   */
  async register(config: WebhookConfig): Promise<{ webhookId: string }> {
    try {
      const response = await this.client.post('/webhooks', config);
      return response.data;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * List webhooks
   */
  async list(): Promise<Array<WebhookConfig & { webhookId: string }>> {
    try {
      const response = await this.client.get('/webhooks');
      return response.data.webhooks;
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }

  /**
   * Delete a webhook
   */
  async delete(webhookId: string): Promise<void> {
    try {
      await this.client.delete(`/webhooks/${webhookId}`);
    } catch (error) {
      return handleAPIError(error as AxiosError);
    }
  }
}

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * Main SDK class for WIA Microplastic Detection
 *
 * @example
 * ```typescript
 * const sdk = new MicroplasticDetectionSDK({
 *   apiKey: 'your_api_key',
 *   environment: 'production'
 * });
 * ```
 */
export class MicroplasticDetectionSDK extends EventEmitter {
  private client: AxiosInstance;
  private wsClient?: WebSocket;

  public samples: SampleService;
  public particles: ParticleService;
  public spectroscopy: SpectroscopyService;
  public jobs: JobService;
  public sensors: SensorService;
  public reports: ReportService;
  public images: ImageService;
  public webhooks: WebhookService;

  constructor(config: SDKConfig) {
    super();

    const finalConfig = { ...DEFAULT_CONFIG, ...config };

    if (!finalConfig.apiKey) {
      throw new ValidationError('API key is required');
    }

    const baseUrl = finalConfig.environment
      ? BASE_URLS[finalConfig.environment]
      : finalConfig.baseUrl;

    this.client = axios.create({
      baseURL: baseUrl,
      timeout: finalConfig.timeout,
      headers: {
        'Authorization': `Bearer ${finalConfig.apiKey}`,
        'Content-Type': 'application/json',
        'User-Agent': 'WIA-Microplastic-Detection-SDK/1.0.0'
      }
    });

    // Initialize services
    this.samples = new SampleService(this.client);
    this.particles = new ParticleService(this.client);
    this.spectroscopy = new SpectroscopyService(this.client);
    this.jobs = new JobService(this.client);
    this.sensors = new SensorService(this.client);
    this.reports = new ReportService(this.client);
    this.images = new ImageService(this.client);
    this.webhooks = new WebhookService(this.client);

    // Initialize WebSocket if enabled
    if (finalConfig.enableWebSocket) {
      this.initWebSocket(baseUrl!.replace('http', 'ws'), finalConfig.apiKey!);
    }
  }

  /**
   * Initialize WebSocket connection
   */
  private initWebSocket(wsUrl: string, apiKey: string): void {
    this.wsClient = new WebSocket(`${wsUrl}/ws`);

    this.wsClient.on('open', () => {
      this.wsClient!.send(JSON.stringify({
        action: 'authenticate',
        token: apiKey
      }));
      this.emit('websocket:connected');
    });

    this.wsClient.on('message', (data: WebSocket.Data) => {
      try {
        const message = JSON.parse(data.toString());
        this.emit('websocket:message', message);
        this.emit(`websocket:${message.type}`, message.data);
      } catch (error) {
        this.emit('websocket:error', error);
      }
    });

    this.wsClient.on('error', (error) => {
      this.emit('websocket:error', error);
    });

    this.wsClient.on('close', () => {
      this.emit('websocket:disconnected');
    });
  }

  /**
   * Subscribe to real-time updates
   */
  subscribe(channel: string, data?: any): void {
    if (!this.wsClient) {
      throw new MicroplasticSDKError(
        'WebSocket not enabled. Set enableWebSocket: true in config',
        'WEBSOCKET_DISABLED'
      );
    }

    this.wsClient.send(JSON.stringify({
      action: 'subscribe',
      channel,
      ...data
    }));
  }

  /**
   * Unsubscribe from updates
   */
  unsubscribe(channel: string): void {
    if (!this.wsClient) return;

    this.wsClient.send(JSON.stringify({
      action: 'unsubscribe',
      channel
    }));
  }

  /**
   * Close all connections
   */
  close(): void {
    if (this.wsClient) {
      this.wsClient.close();
    }
    this.removeAllListeners();
  }
}

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * Create SDK instance with API key
 */
export function createSDK(apiKey: string, options?: Partial<SDKConfig>): MicroplasticDetectionSDK {
  return new MicroplasticDetectionSDK({ apiKey, ...options });
}

/**
 * Quick sample analysis workflow
 */
export async function analyzeSample(
  sdk: MicroplasticDetectionSDK,
  sampleData: Partial<MicroplasticSample>,
  analysisConfig?: Partial<AnalysisJobConfig>
): Promise<DetectionResult> {
  // Create sample
  const sample = await sdk.samples.create(sampleData);

  // Submit for analysis
  const job = await sdk.samples.analyze(sample.sampleId, analysisConfig);

  // Wait for completion
  await sdk.jobs.waitForCompletion(job.jobId);

  // Get results
  return await sdk.samples.getResults(sample.sampleId);
}

// ============================================================================
// Default Export
// ============================================================================

export default MicroplasticDetectionSDK;
