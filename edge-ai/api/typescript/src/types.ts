/**
 * WIA-AI-019 Edge AI Standard - TypeScript Type Definitions
 *
 * @packageDocumentation
 * @module @wia/edge-ai
 */

/**
 * Supported data types for tensor operations
 */
export type TensorDType = 'float32' | 'int8' | 'uint8' | 'int32' | 'bool';

/**
 * Hardware accelerator types
 */
export type Accelerator = 'cpu' | 'gpu' | 'npu' | 'dsp' | 'auto';

/**
 * Model frameworks supported by WIA Edge AI
 */
export type Framework = 'tflite' | 'coreml' | 'onnx' | 'pytorch-mobile';

/**
 * Privacy levels for telemetry
 */
export type PrivacyLevel = 'none' | 'basic' | 'strict';

/**
 * Tensor shape and data representation
 */
export interface Tensor {
  /** Tensor dimensions [batch, height, width, channels] */
  shape: number[];

  /** Data type of tensor elements */
  dtype: TensorDType;

  /** Tensor data as typed array */
  data: Float32Array | Int8Array | Uint8Array | Int32Array;

  /** Optional tensor name */
  name?: string;
}

/**
 * Model metadata following WIA-AI-019 PHASE-1 specification
 */
export interface ModelMetadata {
  /** Unique model identifier */
  modelId: string;

  /** Semantic version (e.g., "1.2.3") */
  version: string;

  /** Model framework */
  framework: Framework;

  /** ISO 8601 creation timestamp */
  created: string;

  /** Model architecture details */
  architecture: {
    type: 'cnn' | 'transformer' | 'rnn' | 'hybrid';
    inputShape: number[];
    outputShape: number[];
    parameters: number;
    layers: number;
  };

  /** Optimization applied to model */
  optimization: {
    quantization: 'int8' | 'fp16' | 'fp32';
    pruning: number;
    compression: 'gzip' | 'none';
  };

  /** Hardware requirements */
  hardware: {
    minRAM: string;
    recommendedAccelerator: Accelerator;
    platforms: string[];
  };

  /** Performance benchmarks */
  performance: {
    avgLatency: string;
    throughput: string;
    accuracy: number;
  };

  /** License identifier */
  license: string;

  /** Author/organization */
  author: string;

  /** SHA-256 checksum */
  checksum: string;
}

/**
 * Configuration for model loading
 */
export interface ModelConfig {
  /** Hardware accelerator to use */
  accelerator?: Accelerator;

  /** Number of threads for CPU inference */
  numThreads?: number;

  /** Enable quantization if model supports it */
  enableQuantization?: boolean;

  /** Cache directory for models */
  cacheDir?: string;

  /** Telemetry configuration */
  telemetry?: TelemetryConfig;
}

/**
 * Inference options
 */
export interface InferenceOptions {
  /** Timeout in milliseconds */
  timeout?: number;

  /** Inference priority */
  priority?: 'low' | 'normal' | 'high';

  /** Fallback strategy on failure */
  fallback?: 'cloud' | 'cache' | 'error';
}

/**
 * Inference result with predictions
 */
export interface InferenceResult {
  /** Output tensors */
  tensors: Tensor[];

  /** Parsed predictions (for classification/detection) */
  predictions?: Prediction[];

  /** Inference metadata */
  metadata: {
    inferenceTime: number;
    modelVersion: string;
    accelerator: Accelerator;
    timestamp: string;
  };
}

/**
 * Individual prediction from model
 */
export interface Prediction {
  /** Class label */
  class: string;

  /** Confidence score [0-1] */
  confidence: number;

  /** Class index */
  index: number;

  /** Optional bounding box for object detection */
  bbox?: {
    x: number;
    y: number;
    width: number;
    height: number;
  };
}

/**
 * Telemetry configuration
 */
export interface TelemetryConfig {
  /** Enable telemetry collection */
  enabled: boolean;

  /** Number of events to batch before sending */
  batchSize: number;

  /** Flush interval in seconds */
  flushInterval: number;

  /** Telemetry endpoint URL */
  endpoint?: string;

  /** Privacy level for data collection */
  privacyLevel: PrivacyLevel;
}

/**
 * Inference telemetry event
 */
export interface InferenceEvent {
  /** Model identifier */
  modelId: string;

  /** Model version */
  modelVersion: string;

  /** Inference latency in milliseconds */
  latency: number;

  /** Prediction confidence */
  confidence: number;

  /** Success status */
  success: boolean;

  /** Error message if failed */
  error?: string;

  /** ISO 8601 timestamp */
  timestamp: string;

  /** Accelerator used */
  accelerator: Accelerator;
}

/**
 * Device capabilities
 */
export interface DeviceCapabilities {
  /** Total RAM in MB */
  totalRAM: number;

  /** Available RAM in MB */
  availableRAM: number;

  /** Battery level 0-100 */
  batteryLevel?: number;

  /** Thermal state */
  thermalState?: 'nominal' | 'fair' | 'serious' | 'critical';

  /** Network connection type */
  networkType?: 'none' | 'wifi' | 'cellular';

  /** Available accelerators */
  accelerators: Accelerator[];
}

/**
 * Model update information
 */
export interface UpdateInfo {
  /** Current version on device */
  currentVersion: string;

  /** Latest available version */
  latestVersion: string;

  /** Update file size in bytes */
  updateSize: number;

  /** Changelog/release notes */
  changelog: string;

  /** Release date ISO 8601 */
  releaseDate: string;
}

/**
 * Model update policy
 */
export interface UpdatePolicy {
  /** Automatically download and apply updates */
  autoUpdate: boolean;

  /** Allow updates on metered network */
  allowMeteredNetwork: boolean;

  /** Require device to be charging */
  requireCharging: boolean;

  /** Time window for updates (HH:MM format) */
  updateWindow?: {
    start: string;
    end: string;
  };
}

/**
 * Health check status
 */
export interface HealthStatus {
  /** Overall health status */
  status: 'healthy' | 'degraded' | 'unhealthy';

  /** Individual health checks */
  checks: {
    modelLoaded: boolean;
    acceleratorAvailable: boolean;
    memoryAvailable: number;
    diskSpace: number;
    lastInferenceTime?: number;
  };

  /** SDK version */
  version: string;
}

/**
 * Error codes from WIA-AI-019 specification
 */
export enum ErrorCode {
  MODEL_NOT_FOUND = 'E001',
  MODEL_LOAD_FAILED = 'E002',
  INFERENCE_FAILED = 'E003',
  INVALID_INPUT = 'E004',
  INSUFFICIENT_MEMORY = 'E005',
  UNSUPPORTED_ACCELERATOR = 'E006',
  NETWORK_ERROR = 'E007',
  TIMEOUT = 'E008',
}

/**
 * WIA Edge AI error class
 */
export class EdgeAIError extends Error {
  constructor(
    public code: ErrorCode,
    message: string,
    public details?: any
  ) {
    super(message);
    this.name = 'EdgeAIError';
  }
}

/**
 * Benchmark result for performance testing
 */
export interface BenchmarkResult {
  /** Average latency in milliseconds */
  avgLatency: number;

  /** P50 latency */
  p50Latency: number;

  /** P95 latency */
  p95Latency: number;

  /** P99 latency */
  p99Latency: number;

  /** Throughput in inferences per second */
  throughput: number;

  /** Peak memory usage in MB */
  peakMemory: number;

  /** Average power consumption in mW */
  avgPower?: number;
}
