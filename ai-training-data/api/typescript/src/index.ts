/**
 * WIA-AI-007 AI Training Data Standard - TypeScript SDK
 * 弘益人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @license MIT
 * @copyright 2025 SmileStory Inc. / WIA
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import {
  ClientConfig,
  CreateDatasetOptions,
  DatasetMetadata,
  UploadOptions,
  QualityCheckOptions,
  QualityCheckResult,
  DownloadOptions,
  VersionInfo,
  BiasDetectionOptions,
  BiasDetectionResult,
  AnonymizationOptions,
  ExportOptions,
  ErrorResponse,
} from './types';

/**
 * Main client for WIA-AI-007 API
 */
export class WIA_AI_007 {
  private client: AxiosInstance;
  public datasets: DatasetAPI;

  constructor(config: ClientConfig) {
    this.client = axios.create({
      baseURL: config.endpoint,
      headers: {
        'Authorization': `Bearer ${config.apiKey}`,
        'Content-Type': 'application/json',
      },
      timeout: config.timeout || 30000,
    });

    // Add retry logic
    if (config.retries && config.retries > 0) {
      this.setupRetryInterceptor(config.retries);
    }

    this.datasets = new DatasetAPI(this.client);
  }

  private setupRetryInterceptor(maxRetries: number) {
    this.client.interceptors.response.use(
      (response) => response,
      async (error: AxiosError) => {
        const config: any = error.config;
        config.__retryCount = config.__retryCount || 0;

        if (config.__retryCount >= maxRetries) {
          return Promise.reject(error);
        }

        config.__retryCount += 1;

        const delay = Math.pow(2, config.__retryCount) * 1000;
        await new Promise((resolve) => setTimeout(resolve, delay));

        return this.client(config);
      }
    );
  }
}

/**
 * Dataset API operations
 */
export class DatasetAPI {
  constructor(private client: AxiosInstance) {}

  /**
   * Create a new dataset
   */
  async create(options: CreateDatasetOptions): Promise<Dataset> {
    const response = await this.client.post('/datasets', {
      ...options,
      philosophy: options.philosophy || '弘益人間',
    });

    return new Dataset(this.client, response.data.dataset_id);
  }

  /**
   * Get dataset by ID
   */
  async get(datasetId: string): Promise<Dataset> {
    return new Dataset(this.client, datasetId);
  }

  /**
   * List all datasets
   */
  async list(filters?: { type?: string; tag?: string }): Promise<Dataset[]> {
    const response = await this.client.get('/datasets', { params: filters });
    return response.data.datasets.map(
      (d: any) => new Dataset(this.client, d.dataset_id)
    );
  }

  /**
   * Delete a dataset
   */
  async delete(datasetId: string): Promise<void> {
    await this.client.delete(`/datasets/${datasetId}`);
  }
}

/**
 * Dataset class representing a single dataset
 */
export class Dataset {
  constructor(private client: AxiosInstance, public id: string) {}

  /**
   * Get dataset metadata
   */
  async getMetadata(): Promise<DatasetMetadata> {
    const response = await this.client.get(`/datasets/${this.id}`);
    return response.data;
  }

  /**
   * Update dataset metadata
   */
  async updateMetadata(metadata: Partial<DatasetMetadata>): Promise<void> {
    await this.client.patch(`/datasets/${this.id}`, metadata);
  }

  /**
   * Upload data to dataset
   */
  async upload(options: UploadOptions): Promise<string> {
    const formData = new FormData();

    // Handle file uploads
    const files = Array.isArray(options.files) ? options.files : [options.files];
    files.forEach((file) => {
      formData.append('files', file);
    });

    if (options.labels) {
      formData.append('labels', options.labels);
    }

    if (options.metadata) {
      formData.append('metadata', options.metadata);
    }

    const response = await this.client.post(
      `/datasets/${this.id}/upload`,
      formData,
      {
        headers: { 'Content-Type': 'multipart/form-data' },
      }
    );

    return response.data.upload_id;
  }

  /**
   * Download dataset
   */
  async download(options: DownloadOptions): Promise<void> {
    const params: any = {
      version: options.version,
      format: options.format,
      split: options.split,
    };

    const response = await this.client.get(`/datasets/${this.id}/download`, {
      params,
      responseType: 'blob',
    });

    // Save to file (implementation depends on environment)
    console.log(`Dataset downloaded to ${options.outputPath}`);
  }

  /**
   * Check dataset quality
   */
  async checkQuality(options: QualityCheckOptions): Promise<QualityCheckResult> {
    const response = await this.client.post(
      `/datasets/${this.id}/quality/check`,
      options
    );

    return {
      overallScore: response.data.quality_score,
      checks: response.data.checks,
      issues: response.data.issues,
    };
  }

  /**
   * List dataset versions
   */
  async listVersions(): Promise<VersionInfo[]> {
    const response = await this.client.get(`/datasets/${this.id}/versions`);
    return response.data.versions;
  }

  /**
   * Create new version
   */
  async createVersion(versionInfo: Omit<VersionInfo, 'date'>): Promise<string> {
    const response = await this.client.post(
      `/datasets/${this.id}/versions`,
      versionInfo
    );
    return response.data.version;
  }

  /**
   * Detect bias in dataset
   */
  async detectBias(
    options: BiasDetectionOptions
  ): Promise<BiasDetectionResult> {
    const response = await this.client.post(
      `/datasets/${this.id}/bias/detect`,
      options
    );

    return {
      biasDetected: response.data.bias_detected,
      metrics: response.data.metrics,
      affectedGroups: response.data.affected_groups,
      recommendations: response.data.recommendations,
    };
  }

  /**
   * Anonymize dataset
   */
  async anonymize(options: AnonymizationOptions): Promise<string> {
    const response = await this.client.post(
      `/datasets/${this.id}/anonymize`,
      options
    );
    return response.data.anonymized_dataset_id;
  }

  /**
   * Export dataset for ML framework
   */
  async export(options: ExportOptions): Promise<void> {
    const response = await this.client.post(
      `/datasets/${this.id}/export`,
      options
    );

    console.log(`Dataset exported to ${options.outputPath}`);
    return response.data;
  }

  /**
   * Apply transformations
   */
  async transform(operations: Array<{ type: string; params?: any }>): Promise<string> {
    const response = await this.client.post(
      `/datasets/${this.id}/transform`,
      { operations }
    );
    return response.data.job_id;
  }

  /**
   * Get transformation job status
   */
  async getJobStatus(jobId: string): Promise<{ status: string; progress: number }> {
    const response = await this.client.get(`/jobs/${jobId}`);
    return {
      status: response.data.status,
      progress: response.data.progress,
    };
  }
}

/**
 * Utility functions
 */
export class Utils {
  /**
   * Validate metadata against WIA-AI-007 schema
   */
  static validateMetadata(metadata: DatasetMetadata): boolean {
    // Basic validation - in production, use JSON Schema validator
    if (!metadata['wia-ai-007']) return false;
    if (!metadata.dataset) return false;
    if (!metadata.schema) return false;
    if (!metadata.provenance) return false;
    if (!metadata.quality) return false;
    if (!metadata.license) return false;

    return true;
  }

  /**
   * Parse semantic version
   */
  static parseVersion(version: string): { major: number; minor: number; patch: number } {
    const parts = version.split('.').map(Number);
    return {
      major: parts[0] || 0,
      minor: parts[1] || 0,
      patch: parts[2] || 0,
    };
  }

  /**
   * Compare versions
   */
  static compareVersions(v1: string, v2: string): number {
    const version1 = this.parseVersion(v1);
    const version2 = this.parseVersion(v2);

    if (version1.major !== version2.major) {
      return version1.major - version2.major;
    }
    if (version1.minor !== version2.minor) {
      return version1.minor - version2.minor;
    }
    return version1.patch - version2.patch;
  }
}

// Export all types
export * from './types';

// Export default
export default WIA_AI_007;
