/**
 * WIA-AI-012: Federated Learning TypeScript SDK - Type Definitions
 *
 * Philosophy: 弘益人間 (Hongik Ingan) - Benefit All Humanity
 *
 * This SDK enables privacy-preserving collaborative machine learning
 * by providing type-safe interfaces for federated learning systems.
 *
 * @module @wia/federated-learning
 * @version 1.0.0
 * @license MIT
 */

/**
 * Configuration options for the Federated Learning client
 */
export interface FederatedLearningConfig {
  /** Server URL (e.g., 'https://fl.example.com') */
  serverUrl: string;

  /** API key for authentication */
  apiKey: string;

  /** Unique device identifier */
  deviceId: string;

  /** Client platform */
  platform?: 'web' | 'node' | 'mobile' | 'iot';

  /** Privacy settings */
  privacy?: PrivacyConfig;

  /** Communication settings */
  communication?: CommunicationConfig;

  /** Training settings */
  training?: TrainingConfig;
}

/**
 * Privacy configuration for differential privacy and secure aggregation
 */
export interface PrivacyConfig {
  /** Enable differential privacy */
  differentialPrivacy?: boolean;

  /** Privacy budget (epsilon) */
  epsilon?: number;

  /** Failure probability (delta) */
  delta?: number;

  /** Gradient clipping norm */
  clippingNorm?: number;

  /** Enable secure aggregation */
  secureAggregation?: boolean;
}

/**
 * Communication configuration
 */
export interface CommunicationConfig {
  /** Request timeout in milliseconds */
  timeout?: number;

  /** Number of retry attempts */
  maxRetries?: number;

  /** Enable compression */
  compression?: boolean;

  /** Chunk size for large uploads (bytes) */
  chunkSize?: number;
}

/**
 * Training configuration
 */
export interface TrainingConfig {
  /** Number of local epochs */
  localEpochs?: number;

  /** Batch size */
  batchSize?: number;

  /** Learning rate */
  learningRate?: number;

  /** Optimizer */
  optimizer?: 'sgd' | 'adam' | 'adagrad';
}

/**
 * Device capabilities reported to server
 */
export interface DeviceCapabilities {
  /** Maximum model size in MB */
  maxModelSizeMB: number;

  /** Supported ML frameworks */
  supportedFrameworks: string[];

  /** Compute capability */
  computeCapability: 'low' | 'medium' | 'high';

  /** Available memory in MB */
  availableMemoryMB?: number;

  /** Battery level (0-1) */
  batteryLevel?: number;

  /** Network type */
  networkType?: 'wifi' | '4g' | '5g' | 'ethernet';
}

/**
 * Client registration response
 */
export interface RegistrationResponse {
  /** Unique client identifier */
  clientId: string;

  /** Authentication token */
  registrationToken: string;

  /** Token expiration timestamp */
  expiresAt: string;
}

/**
 * Training round information
 */
export interface TrainingRound {
  /** Round number */
  roundNumber: number;

  /** Round status */
  status: 'pending' | 'active' | 'completed' | 'failed';

  /** Global model reference */
  globalModel: ModelReference;

  /** Training configuration */
  trainingConfig: TrainingConfig;

  /** Submission deadline */
  deadline: string;

  /** Minimum participating clients */
  minClients: number;

  /** Target number of clients */
  targetClients?: number;
}

/**
 * Model reference for download
 */
export interface ModelReference {
  /** Model identifier */
  modelId: string;

  /** Model version */
  version: string;

  /** Download URL */
  downloadUrl: string;

  /** SHA-256 checksum */
  checksum: string;

  /** Size in bytes */
  sizeBytes: number;

  /** Model format */
  format?: 'tensorflow' | 'pytorch' | 'onnx';
}

/**
 * Model update submission
 */
export interface ModelUpdate {
  /** Update identifier */
  updateId?: string;

  /** Client identifier */
  clientId: string;

  /** Round number */
  roundNumber: number;

  /** Base model version */
  baseModelVersion: string;

  /** Update type */
  updateType: 'gradient' | 'weights' | 'delta';

  /** Update data (serialized) */
  data: ArrayBuffer | Uint8Array;

  /** Training metrics */
  trainingMetrics: TrainingMetrics;

  /** Privacy metadata */
  privacy?: PrivacyMetadata;

  /** Cryptographic signature */
  signature?: string;
}

/**
 * Training metrics from local training
 */
export interface TrainingMetrics {
  /** Number of local epochs completed */
  localEpochs: number;

  /** Batch size used */
  batchSize: number;

  /** Number of samples trained */
  samplesTrained: number;

  /** Final local loss */
  localLoss: number;

  /** Final local accuracy */
  localAccuracy?: number;

  /** Training duration in seconds */
  durationSeconds: number;
}

/**
 * Privacy metadata attached to updates
 */
export interface PrivacyMetadata {
  /** Differential privacy applied */
  differentialPrivacy: {
    /** Privacy budget (epsilon) */
    epsilon: number;

    /** Failure probability (delta) */
    delta: number;

    /** Noise multiplier */
    noiseMultiplier?: number;

    /** Clipping norm applied */
    clippingNorm?: number;
  };

  /** Secure aggregation enabled */
  secureAggregation: boolean;
}

/**
 * Update submission receipt
 */
export interface UpdateReceipt {
  /** Update identifier */
  updateId: string;

  /** Submission status */
  status: 'accepted' | 'rejected' | 'pending';

  /** Estimated aggregation completion time */
  estimatedAggregationTime?: string;

  /** Rejection reason (if rejected) */
  rejectionReason?: string;
}

/**
 * Aggregation result notification
 */
export interface AggregationResult {
  /** Round number */
  roundNumber: number;

  /** Aggregation status */
  status: 'success' | 'failed' | 'partial';

  /** New global model */
  newGlobalModel: ModelReference;

  /** Round statistics */
  roundStatistics: RoundStatistics;

  /** Next round information */
  nextRound?: {
    scheduledStart: string;
    estimatedParticipants: number;
  };
}

/**
 * Statistics for a completed training round
 */
export interface RoundStatistics {
  /** Number of clients invited */
  clientsInvited: number;

  /** Number of clients that participated */
  clientsParticipated: number;

  /** Number of updates accepted */
  updatesAccepted: number;

  /** Number of updates rejected */
  updatesRejected: number;

  /** Global model loss */
  globalLoss: number;

  /** Global model accuracy */
  globalAccuracy?: number;

  /** Improvement over previous round */
  improvement?: number;
}

/**
 * Error response from server
 */
export interface FederatedLearningError {
  /** Error code */
  code: string;

  /** Human-readable message */
  message: string;

  /** Detailed error information */
  details?: Record<string, any>;

  /** Retry-after timestamp (for rate limiting) */
  retryAfter?: string;

  /** Support identifier for troubleshooting */
  supportId?: string;
}

/**
 * Event types emitted by the client
 */
export type FederatedLearningEvent =
  | 'registered'
  | 'training_invitation'
  | 'training_started'
  | 'training_progress'
  | 'training_completed'
  | 'update_submitted'
  | 'aggregation_completed'
  | 'error';

/**
 * Event handler type
 */
export type EventHandler<T = any> = (data: T) => void | Promise<void>;

/**
 * Compression configuration
 */
export interface CompressionConfig {
  /** Compression method */
  method: 'quantization' | 'sparsification' | 'low-rank' | 'none';

  /** Compression parameters */
  parameters?: {
    /** Number of bits for quantization */
    numBits?: number;

    /** Sparsity ratio (fraction of values to keep) */
    sparsityRatio?: number;

    /** Rank for low-rank approximation */
    rank?: number;
  };
}

/**
 * Model data structure
 */
export interface Model {
  /** Model identifier */
  id: string;

  /** Model version */
  version: string;

  /** Model weights/parameters */
  weights: Record<string, Tensor>;

  /** Model architecture metadata */
  architecture?: {
    type: string;
    layers: any[];
    parametersCount: number;
  };
}

/**
 * Tensor (simplified representation)
 */
export interface Tensor {
  /** Tensor shape */
  shape: number[];

  /** Data type */
  dtype: 'float32' | 'float16' | 'int8' | 'int32';

  /** Tensor values (flattened) */
  values: number[] | Float32Array | Int8Array;
}

/**
 * Training callback interface
 */
export interface TrainingCallback {
  /** Called when training round starts */
  onTrainingStart?: (round: TrainingRound) => void | Promise<void>;

  /** Called during training for progress updates */
  onTrainingProgress?: (progress: number) => void | Promise<void>;

  /** Called when training completes */
  onTrainingComplete?: (metrics: TrainingMetrics) => void | Promise<void>;

  /** Called on training error */
  onTrainingError?: (error: Error) => void | Promise<void>;
}
