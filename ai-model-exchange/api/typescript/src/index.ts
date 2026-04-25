/**
 * WIA-AI-008 AI Model Exchange Standard - TypeScript SDK
 * 弘益人間 - Benefit All Humanity
 *
 * @module @wia/ai-model-exchange
 * @version 1.0.0
 * @license Apache-2.0
 */

import axios, { AxiosInstance, AxiosRequestConfig } from 'axios';
import FormData from 'form-data';
import * as fs from 'fs';
import * as path from 'path';

export * from './types';

import {
  RegistryConfig,
  ModelSummary,
  ModelVersion,
  ModelCard,
  ModelRegistrationRequest,
  SearchParams,
  SearchResults,
  ModelStage,
  HealthCheckResponse,
  InferenceRequest,
  InferenceResponse,
} from './types';

/**
 * WIA-AI-008 Model Registry Client
 *
 * @example
 * ```typescript
 * import { RegistryClient } from '@wia/ai-model-exchange';
 *
 * const client = new RegistryClient({
 *   registry_url: 'https://registry.example.com',
 *   api_key: 'your-api-key'
 * });
 *
 * // Search models
 * const results = await client.searchModels({
 *   query: 'sentiment analysis',
 *   task_type: TaskType.TextClassification
 * });
 *
 * // Download model
 * await client.downloadModel('bert-sentiment', '1.0.0', './models/');
 * ```
 */
export class RegistryClient {
  private client: AxiosInstance;
  private config: RegistryConfig;

  /**
   * Create a new Registry Client
   *
   * @param config - Registry configuration
   */
  constructor(config: RegistryConfig) {
    this.config = {
      timeout: 30000,
      ...config,
    };

    this.client = axios.create({
      baseURL: `${config.registry_url}/api/v1`,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        ...this.getAuthHeaders(),
        ...config.headers,
      },
    });
  }

  /**
   * Get authentication headers
   */
  private getAuthHeaders(): Record<string, string> {
    if (this.config.api_key) {
      return { Authorization: `Bearer ${this.config.api_key}` };
    }
    if (this.config.token) {
      return { Authorization: `Token ${this.config.token}` };
    }
    return {};
  }

  /**
   * List all models
   *
   * @param limit - Maximum number of results
   * @param offset - Result offset
   * @returns List of model summaries
   */
  async listModels(limit = 50, offset = 0): Promise<{ models: ModelSummary[]; total: number }> {
    const response = await this.client.get('/models', {
      params: { limit, offset },
    });
    return response.data;
  }

  /**
   * Search models
   *
   * @param params - Search parameters
   * @returns Search results
   */
  async searchModels(params: SearchParams): Promise<SearchResults> {
    const response = await this.client.get('/models/search', { params });
    return response.data;
  }

  /**
   * Get model details
   *
   * @param modelName - Model name
   * @returns Model information with all versions
   */
  async getModel(modelName: string): Promise<any> {
    const response = await this.client.get(`/models/${modelName}`);
    return response.data;
  }

  /**
   * Get specific model version
   *
   * @param modelName - Model name
   * @param version - Model version
   * @returns Model version details
   */
  async getModelVersion(modelName: string, version: string): Promise<ModelVersion> {
    const response = await this.client.get(`/models/${modelName}/versions/${version}`);
    return response.data;
  }

  /**
   * Register a new model
   *
   * @param request - Model registration request
   * @returns Registered model information
   */
  async registerModel(request: ModelRegistrationRequest): Promise<any> {
    const response = await this.client.post('/models', request);
    return response.data;
  }

  /**
   * Upload model file and create new version
   *
   * @param modelName - Model name
   * @param version - Version string
   * @param modelPath - Path to model file
   * @param metadata - Model metadata
   * @returns Created version information
   */
  async uploadModelVersion(
    modelName: string,
    version: string,
    modelPath: string,
    metadata: ModelCard
  ): Promise<ModelVersion> {
    const formData = new FormData();
    formData.append('version', version);
    formData.append('model_file', fs.createReadStream(modelPath));
    formData.append('metadata', JSON.stringify(metadata));

    const response = await this.client.post(`/models/${modelName}/versions`, formData, {
      headers: {
        ...formData.getHeaders(),
      },
      maxContentLength: Infinity,
      maxBodyLength: Infinity,
    });

    return response.data;
  }

  /**
   * Download model
   *
   * @param modelName - Model name
   * @param version - Model version or alias ("latest", "production")
   * @param outputDir - Directory to save model
   * @returns Path to downloaded model
   */
  async downloadModel(modelName: string, version: string, outputDir: string): Promise<string> {
    const response = await this.client.get(`/models/${modelName}/versions/${version}/download`, {
      responseType: 'stream',
    });

    // Extract filename from Content-Disposition header
    const contentDisposition = response.headers['content-disposition'];
    let filename = `${modelName}-${version}.onnx`;
    if (contentDisposition) {
      const filenameMatch = contentDisposition.match(/filename="?([^"]+)"?/);
      if (filenameMatch) {
        filename = filenameMatch[1];
      }
    }

    const outputPath = path.join(outputDir, filename);

    // Ensure output directory exists
    if (!fs.existsSync(outputDir)) {
      fs.mkdirSync(outputDir, { recursive: true });
    }

    // Stream to file
    const writer = fs.createWriteStream(outputPath);
    response.data.pipe(writer);

    return new Promise((resolve, reject) => {
      writer.on('finish', () => resolve(outputPath));
      writer.on('error', reject);
    });
  }

  /**
   * Transition model version stage
   *
   * @param modelName - Model name
   * @param version - Model version
   * @param stage - Target stage
   */
  async transitionStage(modelName: string, version: string, stage: ModelStage): Promise<void> {
    await this.client.put(`/models/${modelName}/versions/${version}/stage`, { stage });
  }

  /**
   * Delete model
   *
   * @param modelName - Model name
   */
  async deleteModel(modelName: string): Promise<void> {
    await this.client.delete(`/models/${modelName}`);
  }

  /**
   * Delete specific model version
   *
   * @param modelName - Model name
   * @param version - Model version
   */
  async deleteModelVersion(modelName: string, version: string): Promise<void> {
    await this.client.delete(`/models/${modelName}/versions/${version}`);
  }
}

/**
 * Model Server Client for inference
 *
 * @example
 * ```typescript
 * const server = new ModelServerClient('http://localhost:8080');
 *
 * const result = await server.predict({
 *   model_name: 'resnet50',
 *   inputs: { image: imageData }
 * });
 * ```
 */
export class ModelServerClient {
  private client: AxiosInstance;

  /**
   * Create Model Server Client
   *
   * @param serverUrl - Model server URL
   * @param timeout - Request timeout in ms
   */
  constructor(serverUrl: string, timeout = 30000) {
    this.client = axios.create({
      baseURL: serverUrl,
      timeout,
      headers: {
        'Content-Type': 'application/json',
      },
    });
  }

  /**
   * Run inference
   *
   * @param request - Inference request
   * @returns Inference response
   */
  async predict(request: InferenceRequest): Promise<InferenceResponse> {
    const { model_name, model_version = 'latest', inputs } = request;
    const endpoint = model_version
      ? `/v1/models/${model_name}/versions/${model_version}:predict`
      : `/v1/models/${model_name}:predict`;

    const response = await this.client.post(endpoint, { inputs });
    return response.data;
  }

  /**
   * Run batch inference
   *
   * @param modelName - Model name
   * @param inputs - Batch of inputs
   * @param modelVersion - Model version
   * @returns Batch of predictions
   */
  async batchPredict(
    modelName: string,
    inputs: any[],
    modelVersion = 'latest'
  ): Promise<InferenceResponse[]> {
    const endpoint = `/v1/models/${modelName}/versions/${modelVersion}:batchPredict`;
    const response = await this.client.post(endpoint, { inputs });
    return response.data.predictions;
  }

  /**
   * Get server health status
   *
   * @returns Health check response
   */
  async healthCheck(): Promise<HealthCheckResponse> {
    const response = await this.client.get('/health');
    return response.data;
  }

  /**
   * Get model metadata
   *
   * @param modelName - Model name
   * @param modelVersion - Model version
   * @returns Model metadata
   */
  async getModelMetadata(modelName: string, modelVersion = 'latest'): Promise<ModelCard> {
    const response = await this.client.get(`/v1/models/${modelName}/versions/${modelVersion}/metadata`);
    return response.data;
  }
}

/**
 * Utility functions
 */
export class ModelUtils {
  /**
   * Validate model card against WIA-AI-008 schema
   *
   * @param modelCard - Model card to validate
   * @returns Validation result
   */
  static validateModelCard(modelCard: any): { valid: boolean; errors: string[] } {
    const errors: string[] = [];

    // Check required fields
    if (!modelCard.model_identity) errors.push('Missing model_identity');
    if (!modelCard.model_details) errors.push('Missing model_details');
    if (!modelCard.training_info) errors.push('Missing training_info');
    if (!modelCard.performance_metrics) errors.push('Missing performance_metrics');
    if (!modelCard.input_output_spec) errors.push('Missing input_output_spec');
    if (!modelCard.deployment) errors.push('Missing deployment');
    if (!modelCard.governance) errors.push('Missing governance');
    if (!modelCard.wia_standard) errors.push('Missing wia_standard');

    // Check WIA standard compliance
    if (modelCard.wia_standard) {
      if (modelCard.wia_standard.standard_id !== 'WIA-AI-008') {
        errors.push('Invalid standard_id');
      }
      if (modelCard.wia_standard.philosophy !== '弘益人間 - Benefit All Humanity') {
        errors.push('Missing philosophy statement');
      }
    }

    return {
      valid: errors.length === 0,
      errors,
    };
  }

  /**
   * Parse semantic version
   *
   * @param version - Version string
   * @returns Parsed version components
   */
  static parseVersion(version: string): { major: number; minor: number; patch: number } | null {
    const match = version.match(/^(\d+)\.(\d+)\.(\d+)$/);
    if (!match) return null;

    return {
      major: parseInt(match[1]),
      minor: parseInt(match[2]),
      patch: parseInt(match[3]),
    };
  }

  /**
   * Compare two versions
   *
   * @param v1 - First version
   * @param v2 - Second version
   * @returns -1 if v1 < v2, 0 if equal, 1 if v1 > v2
   */
  static compareVersions(v1: string, v2: string): number {
    const p1 = this.parseVersion(v1);
    const p2 = this.parseVersion(v2);

    if (!p1 || !p2) throw new Error('Invalid version format');

    if (p1.major !== p2.major) return p1.major > p2.major ? 1 : -1;
    if (p1.minor !== p2.minor) return p1.minor > p2.minor ? 1 : -1;
    if (p1.patch !== p2.patch) return p1.patch > p2.patch ? 1 : -1;

    return 0;
  }

  /**
   * Generate model card template
   *
   * @param modelName - Model name
   * @param version - Model version
   * @returns Model card template
   */
  static generateModelCardTemplate(modelName: string, version: string): ModelCard {
    return {
      model_identity: {
        name: modelName,
        version,
        unique_id: this.generateUUID(),
        created_at: new Date().toISOString(),
        updated_at: new Date().toISOString(),
      },
      model_details: {
        architecture: 'TODO',
        task_type: 'custom' as any,
        framework: 'TODO',
        framework_version: 'TODO',
        description: 'TODO',
      },
      training_info: {
        dataset: 'TODO',
        training_date: new Date().toISOString(),
        hyperparameters: {},
      },
      performance_metrics: {
        primary_metric: {
          name: 'accuracy',
          value: 0.0,
        },
        evaluation_dataset: 'TODO',
      },
      input_output_spec: {
        inputs: [],
        outputs: [],
      },
      deployment: {
        target_platforms: [],
        min_memory_mb: 0,
        min_compute: 'TODO',
      },
      governance: {
        owner: 'TODO',
        license: 'apache-2.0',
        intended_use: 'TODO',
        limitations: 'TODO',
        bias_analysis: false,
        privacy_compliant: false,
      },
      provenance: {},
      wia_standard: {
        standard_id: 'WIA-AI-008',
        version: '1.0.0',
        philosophy: '弘益人間 - Benefit All Humanity',
      },
    };
  }

  /**
   * Generate UUID v4
   */
  private static generateUUID(): string {
    return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
      const r = (Math.random() * 16) | 0;
      const v = c === 'x' ? r : (r & 0x3) | 0x8;
      return v.toString(16);
    });
  }
}

/**
 * Default export
 */
export default {
  RegistryClient,
  ModelServerClient,
  ModelUtils,
};
