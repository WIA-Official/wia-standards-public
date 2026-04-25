/**
 * WIA-AI-012: Federated Learning TypeScript SDK
 *
 * Philosophy: 弘益人間 (Hongik Ingan) - Benefit All Humanity
 *
 * This SDK enables privacy-preserving collaborative machine learning,
 * allowing participants to train shared models without exposing raw data.
 *
 * @module @wia/federated-learning
 * @version 1.0.0
 * @license MIT
 * @copyright 2025 SmileStory Inc. / World Certification Industry Association
 */

import { EventEmitter } from 'events';
import axios, { AxiosInstance } from 'axios';
import { v4 as uuidv4 } from 'uuid';

import {
  FederatedLearningConfig,
  DeviceCapabilities,
  RegistrationResponse,
  TrainingRound,
  ModelReference,
  Model,
  ModelUpdate,
  UpdateReceipt,
  AggregationResult,
  TrainingCallback,
  FederatedLearningEvent,
  EventHandler,
  FederatedLearningError,
} from './types';

/**
 * Main Federated Learning Client
 *
 * @example
 * ```typescript
 * const client = new FederatedLearningClient({
 *   serverUrl: 'https://fl.example.com',
 *   apiKey: 'your-api-key',
 *   deviceId: 'device-123'
 * });
 *
 * await client.register();
 *
 * client.on('training_invitation', async (round) => {
 *   const model = await client.downloadModel(round.globalModel);
 *   const updatedModel = await trainLocally(model);
 *   await client.submitUpdate(updatedModel, round.roundNumber);
 * });
 *
 * await client.start();
 * ```
 */
export class FederatedLearningClient extends EventEmitter {
  private config: Required<FederatedLearningConfig>;
  private httpClient: AxiosInstance;
  private clientId?: string;
  private authToken?: string;
  private isRunning: boolean = false;

  constructor(config: FederatedLearningConfig) {
    super();

    // Set defaults
    this.config = {
      ...config,
      platform: config.platform || 'node',
      privacy: {
        differentialPrivacy: true,
        epsilon: 1.0,
        delta: 1e-5,
        clippingNorm: 1.0,
        secureAggregation: false,
        ...config.privacy,
      },
      communication: {
        timeout: 30000,
        maxRetries: 3,
        compression: true,
        chunkSize: 1024 * 1024, // 1MB
        ...config.communication,
      },
      training: {
        localEpochs: 5,
        batchSize: 32,
        learningRate: 0.01,
        optimizer: 'sgd',
        ...config.training,
      },
    };

    // Initialize HTTP client
    this.httpClient = axios.create({
      baseURL: this.config.serverUrl,
      timeout: this.config.communication.timeout,
      headers: {
        'Content-Type': 'application/json',
        'X-WIA-Platform': this.config.platform,
      },
    });

    // Add request interceptor for auth
    this.httpClient.interceptors.request.use((config) => {
      if (this.authToken) {
        config.headers.Authorization = `Bearer ${this.authToken}`;
      }
      return config;
    });

    // Add response interceptor for error handling
    this.httpClient.interceptors.response.use(
      (response) => response,
      (error) => {
        const flError: FederatedLearningError = {
          code: error.response?.data?.error?.code || 'UNKNOWN_ERROR',
          message: error.response?.data?.error?.message || error.message,
          details: error.response?.data?.error?.details,
        };
        this.emit('error', flError);
        throw flError;
      }
    );
  }

  /**
   * Register client with the federated learning server
   */
  async register(capabilities?: DeviceCapabilities): Promise<RegistrationResponse> {
    const defaultCapabilities: DeviceCapabilities = {
      maxModelSizeMB: 500,
      supportedFrameworks: ['tensorflow', 'pytorch', 'onnx'],
      computeCapability: 'medium',
      ...capabilities,
    };

    try {
      const response = await this.httpClient.post<RegistrationResponse>(
        '/api/v1/clients/register',
        {
          deviceId: this.config.deviceId,
          platform: this.config.platform,
          capabilities: defaultCapabilities,
        },
        {
          headers: {
            Authorization: `Bearer ${this.config.apiKey}`,
          },
        }
      );

      this.clientId = response.data.clientId;
      this.authToken = response.data.registrationToken;

      this.emit('registered', response.data);
      return response.data;
    } catch (error) {
      throw this.handleError(error, 'Failed to register client');
    }
  }

  /**
   * Get current training round information
   */
  async getCurrentRound(): Promise<TrainingRound> {
    try {
      const response = await this.httpClient.get<TrainingRound>('/api/v1/rounds/current');
      return response.data;
    } catch (error) {
      throw this.handleError(error, 'Failed to get current round');
    }
  }

  /**
   * Download global model
   */
  async downloadModel(modelRef: ModelReference): Promise<ArrayBuffer> {
    try {
      const response = await this.httpClient.get(modelRef.downloadUrl, {
        responseType: 'arraybuffer',
      });

      // Verify checksum
      const checksum = await this.calculateChecksum(response.data);
      if (checksum !== modelRef.checksum) {
        throw new Error('Model checksum mismatch');
      }

      return response.data;
    } catch (error) {
      throw this.handleError(error, 'Failed to download model');
    }
  }

  /**
   * Submit model update to server
   */
  async submitUpdate(
    updateData: ArrayBuffer,
    roundNumber: number,
    metrics: Partial<ModelUpdate['trainingMetrics']> = {}
  ): Promise<UpdateReceipt> {
    if (!this.clientId) {
      throw new Error('Client not registered');
    }

    try {
      const update: Partial<ModelUpdate> = {
        updateId: uuidv4(),
        clientId: this.clientId,
        roundNumber,
        updateType: 'weights',
        data: updateData,
        trainingMetrics: {
          localEpochs: this.config.training.localEpochs,
          batchSize: this.config.training.batchSize,
          samplesTrained: 0,
          localLoss: 0,
          durationSeconds: 0,
          ...metrics,
        },
        privacy: this.config.privacy.differentialPrivacy
          ? {
              differentialPrivacy: {
                epsilon: this.config.privacy.epsilon,
                delta: this.config.privacy.delta,
                clippingNorm: this.config.privacy.clippingNorm,
              },
              secureAggregation: this.config.privacy.secureAggregation,
            }
          : undefined,
      };

      const response = await this.httpClient.post<UpdateReceipt>(
        `/api/v1/rounds/${roundNumber}/updates`,
        updateData,
        {
          headers: {
            'Content-Type': 'application/octet-stream',
            'X-Update-Checksum': await this.calculateChecksum(updateData),
            'X-Update-Metadata': JSON.stringify({
              trainingMetrics: update.trainingMetrics,
              privacy: update.privacy,
            }),
          },
        }
      );

      this.emit('update_submitted', response.data);
      return response.data;
    } catch (error) {
      throw this.handleError(error, 'Failed to submit update');
    }
  }

  /**
   * Start participating in federated learning
   *
   * Polls for training rounds and emits events
   */
  async start(callback?: TrainingCallback): Promise<void> {
    if (!this.clientId) {
      throw new Error('Client not registered. Call register() first.');
    }

    this.isRunning = true;

    while (this.isRunning) {
      try {
        const round = await this.getCurrentRound();

        if (round.status === 'active') {
          this.emit('training_invitation', round);

          if (callback?.onTrainingStart) {
            await callback.onTrainingStart(round);
          }
        }

        // Poll every 30 seconds
        await this.sleep(30000);
      } catch (error) {
        console.error('Error in training loop:', error);
        await this.sleep(60000); // Wait longer on error
      }
    }
  }

  /**
   * Stop participating in federated learning
   */
  stop(): void {
    this.isRunning = false;
  }

  /**
   * Utility: Calculate SHA-256 checksum
   */
  private async calculateChecksum(data: ArrayBuffer): Promise<string> {
    if (typeof crypto !== 'undefined' && crypto.subtle) {
      const hashBuffer = await crypto.subtle.digest('SHA-256', data);
      const hashArray = Array.from(new Uint8Array(hashBuffer));
      return hashArray.map((b) => b.toString(16).padStart(2, '0')).join('');
    }

    // Fallback for Node.js
    const crypto = require('crypto');
    return crypto.createHash('sha256').update(Buffer.from(data)).digest('hex');
  }

  /**
   * Utility: Sleep for ms milliseconds
   */
  private sleep(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  /**
   * Utility: Error handler
   */
  private handleError(error: any, context: string): Error {
    const message = error.response?.data?.error?.message || error.message;
    return new Error(`${context}: ${message}`);
  }

  /**
   * Add event listener (type-safe)
   */
  on(event: 'registered', handler: EventHandler<RegistrationResponse>): this;
  on(event: 'training_invitation', handler: EventHandler<TrainingRound>): this;
  on(event: 'update_submitted', handler: EventHandler<UpdateReceipt>): this;
  on(event: 'aggregation_completed', handler: EventHandler<AggregationResult>): this;
  on(event: 'error', handler: EventHandler<FederatedLearningError>): this;
  on(event: FederatedLearningEvent, handler: EventHandler): this {
    return super.on(event, handler);
  }
}

/**
 * Export all types
 */
export * from './types';

/**
 * Default export
 */
export default FederatedLearningClient;
