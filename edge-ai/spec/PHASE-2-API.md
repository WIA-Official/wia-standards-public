# WIA-AI-019 PHASE 2: API Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-25

## Overview

This specification defines standard APIs for edge AI inference, model management, and telemetry collection. Implementations MUST support these interfaces to ensure interoperability.

## Core Inference API

### Model Loading

```typescript
interface ModelLoader {
  /**
   * Load a model from file or URL
   * @param source - File path or URL to model
   * @param config - Optional configuration
   * @returns Promise resolving to loaded model
   */
  loadModel(source: string, config?: ModelConfig): Promise<Model>;

  /**
   * Unload a model from memory
   * @param modelId - Identifier of model to unload
   */
  unloadModel(modelId: string): Promise<void>;
}

interface ModelConfig {
  accelerator?: 'cpu' | 'gpu' | 'npu' | 'dsp' | 'auto';
  numThreads?: number;
  enableQuantization?: boolean;
  cacheDir?: string;
}
```

### Inference Execution

```typescript
interface InferenceEngine {
  /**
   * Run inference on input tensors
   * @param input - Input tensor(s)
   * @param options - Optional inference options
   * @returns Promise resolving to output tensor(s)
   */
  infer(input: Tensor | Tensor[], options?: InferenceOptions): Promise<Tensor[]>;

  /**
   * Run batch inference
   * @param inputs - Array of input tensor batches
   * @returns Promise resolving to array of output tensors
   */
  batchInfer(inputs: Tensor[][]): Promise<Tensor[][]>;

  /**
   * Get model metadata
   */
  getMetadata(): ModelMetadata;
}

interface InferenceOptions {
  timeout?: number; // milliseconds
  priority?: 'low' | 'normal' | 'high';
  fallback?: 'cloud' | 'cache' | 'error';
}
```

### Tensor Operations

```typescript
interface Tensor {
  shape: number[];
  dtype: 'float32' | 'int8' | 'uint8' | 'int32' | 'bool';
  data: Float32Array | Int8Array | Uint8Array | Int32Array;

  /**
   * Resize tensor to new shape
   */
  resize(newShape: number[]): Tensor;

  /**
   * Convert data type
   */
  asType(dtype: Tensor['dtype']): Tensor;

  /**
   * Get tensor as array buffer
   */
  toBuffer(): ArrayBuffer;

  /**
   * Create tensor from buffer
   */
  static fromBuffer(buffer: ArrayBuffer, shape: number[], dtype: Tensor['dtype']): Tensor;
}
```

## Model Management API

### Model Registry

```typescript
interface ModelRegistry {
  /**
   * Register a new model
   */
  register(metadata: ModelMetadata, modelFile: Blob): Promise<string>;

  /**
   * Get model by ID
   */
  getModel(modelId: string): Promise<Model>;

  /**
   * List all registered models
   */
  listModels(filter?: ModelFilter): Promise<ModelMetadata[]>;

  /**
   * Delete model
   */
  deleteModel(modelId: string): Promise<void>;

  /**
   * Update model metadata
   */
  updateMetadata(modelId: string, metadata: Partial<ModelMetadata>): Promise<void>;
}

interface ModelFilter {
  framework?: string;
  minAccuracy?: number;
  maxSize?: number; // bytes
  tags?: string[];
}
```

### Model Updates

```typescript
interface ModelUpdater {
  /**
   * Check for model updates
   */
  checkForUpdates(modelId: string): Promise<UpdateInfo | null>;

  /**
   * Download and apply update
   */
  update(modelId: string, version: string): Promise<void>;

  /**
   * Rollback to previous version
   */
  rollback(modelId: string): Promise<void>;

  /**
   * Set update policy
   */
  setUpdatePolicy(policy: UpdatePolicy): void;
}

interface UpdateInfo {
  currentVersion: string;
  latestVersion: string;
  updateSize: number; // bytes
  changelog: string;
  releaseDate: string;
}

interface UpdatePolicy {
  autoUpdate: boolean;
  allowMeteredNetwork: boolean;
  requireCharging: boolean;
  updateWindow?: {start: string; end: string}; // HH:MM format
}
```

## Telemetry API

### Event Logging

```typescript
interface TelemetryLogger {
  /**
   * Log inference event
   */
  logInference(event: InferenceEvent): void;

  /**
   * Log model lifecycle event
   */
  logModelEvent(event: ModelEvent): void;

  /**
   * Log performance metrics
   */
  logPerformance(metrics: PerformanceMetrics): void;

  /**
   * Flush pending events to server
   */
  flush(): Promise<void>;

  /**
   * Set telemetry configuration
   */
  configure(config: TelemetryConfig): void;
}

interface InferenceEvent {
  modelId: string;
  modelVersion: string;
  latency: number; // milliseconds
  confidence: number;
  success: boolean;
  error?: string;
  timestamp: string; // ISO 8601
}

interface TelemetryConfig {
  enabled: boolean;
  batchSize: number;
  flushInterval: number; // seconds
  endpoint?: string;
  privacyLevel: 'none' | 'basic' | 'strict';
}
```

## Hardware Abstraction API

### Accelerator Detection

```typescript
interface HardwareInfo {
  /**
   * List available accelerators
   */
  getAvailableAccelerators(): Accelerator[];

  /**
   * Get device capabilities
   */
  getCapabilities(): DeviceCapabilities;

  /**
   * Benchmark accelerator performance
   */
  benchmark(accelerator: Accelerator): Promise<BenchmarkResult>;
}

interface Accelerator {
  type: 'cpu' | 'gpu' | 'npu' | 'dsp';
  name: string;
  vendor: string;
  computeUnits: number;
  memoryMB: number;
}

interface DeviceCapabilities {
  totalRAM: number; // MB
  availableRAM: number; // MB
  batteryLevel?: number; // 0-100
  thermalState?: 'nominal' | 'fair' | 'serious' | 'critical';
  networkType?: 'none' | 'wifi' | 'cellular';
}
```

## Error Handling

### Standard Errors

```typescript
class EdgeAIError extends Error {
  code: string;
  details?: any;
}

// Error codes
const ErrorCodes = {
  MODEL_NOT_FOUND: 'E001',
  MODEL_LOAD_FAILED: 'E002',
  INFERENCE_FAILED: 'E003',
  INVALID_INPUT: 'E004',
  INSUFFICIENT_MEMORY: 'E005',
  UNSUPPORTED_ACCELERATOR: 'E006',
  NETWORK_ERROR: 'E007',
  TIMEOUT: 'E008',
} as const;
```

## Implementation Requirements

### MUST Support
- Model loading from local storage
- Inference on CPU
- Basic telemetry logging
- Error handling with standard error codes

### SHOULD Support
- Hardware accelerator detection and usage
- Batch inference
- Model updates with rollback
- Privacy-preserving telemetry

### MAY Support
- Cloud fallback for failed inferences
- Federated learning integration
- Custom preprocessing pipelines
- Real-time streaming inference

## Security Requirements

1. **Input Validation:** All API inputs MUST be validated
2. **Resource Limits:** Inference MUST respect memory and time limits
3. **Secure Storage:** Models MUST be stored encrypted on disk
4. **Authentication:** Remote model downloads MUST use authenticated connections
5. **Sandboxing:** Inference SHOULD run in sandboxed environment

## Versioning and Compatibility

- API version follows SemVer
- Backwards compatibility MUST be maintained within major versions
- Deprecated features MUST be supported for at least one major version
- Migration guides MUST be provided for breaking changes

---

**Copyright © 2025 World Certification Industry Association (WIA)**
**License:** CC BY 4.0
**弘益人間** - Benefit All Humanity
