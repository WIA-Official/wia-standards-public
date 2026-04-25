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



## Reference Standards Alignment

The Phase 2 API surface is layered above well-established IT primitives.

| Concern | Reference |
|---------|-----------|
| HTTP semantics | RFC 9110 |
| HTTP/1.1 | RFC 9112 |
| HTTP/2 | RFC 9113 |
| HTTP/3 over QUIC | RFC 9114 / RFC 9000 |
| TLS 1.3 | RFC 8446 |
| Certificate format | RFC 5280 (X.509 v3) |
| OpenAPI description | OpenAPI Specification 3.1 |
| JSON | RFC 8259 |
| Errors | RFC 9457 (Problem Details for HTTP APIs) |
| Pagination linking | RFC 8288 (Web Linking) |
| Bearer tokens | RFC 6750 |
| OAuth 2.0 | RFC 6749 + RFC 7636 (PKCE) |
| Mutual TLS | RFC 8705 |
| JWT | RFC 7519 |
| Ed25519 | RFC 8032 |
| ECDSA | NIST FIPS 186-5 |
| Hash | FIPS 180-4, FIPS 202 |
| WebSocket | RFC 6455 |
| Trace context | W3C Trace Context Recommendation |
| OpenTelemetry | OpenTelemetry Specification (CNCF) |
| Locale | BCP 47 (RFC 5646), Unicode CLDR |
| Time | ISO 8601:2019 |
| Information security | ISO/IEC 27001:2022 |

All references conform to the WIA Citation & Veracity Policy v1.0 §2.1 ALLOW.

## Conformance

A Phase 2 implementation is conformant when:

1. The OpenAPI 3.1 description publishes every endpoint with request and response schemas.
2. Authentication accepts at least the `bearerAuth` (JWT) scheme.
3. Errors use RFC 9457 problem-detail responses.
4. Cryptographic primitives match the §Reference Standards Alignment with explicit algorithm identifiers.
5. Telemetry conforms to the OpenTelemetry semantic conventions.

## Implementation Appendix

### A. Idempotency

Endpoints that create or modify resources accept an `Idempotency-Key` request header carrying a UUID. The server stores the response for at least 24 hours and returns the cached response on retry, guarding against duplicate operations under unreliable networking conditions.

### B. Bulk Operations

Bulk inference submission supports up to 100 inferences per request. Bulk responses include per-item success status so the client can retry only the failed records.

### C. Trace Context

Every request carries a W3C Trace Context `traceparent` header so distributed traces span the edge runtime, gateway, model service, and observability backend. Trace identifiers are emitted in error responses to aid debugging.

### D. Versioning Policy

Path versioning (`/api/v1`, `/api/v2`) follows semantic-versioning principles. Minor versions remain wire-compatible with the prior minor of the same major. Major bumps coexist with the prior major for at least 12 months before deprecation.

### E. Edge-Specific Endpoints

Edge AI deployments need endpoints specific to fleet-management concerns:

- `POST /api/v1/inference` — synchronous inference call with optional batching.
- `POST /api/v1/inference/async` — asynchronous inference with callback delivery.
- `POST /api/v1/inference/stream` — streamed inference for time-series inputs.
- `GET /api/v1/devices` — fleet device inventory (paginated).
- `GET /api/v1/devices/{id}` — device detail including current model version, runtime profile, hardware capabilities.
- `POST /api/v1/devices/{id}/deploy` — schedule a model deployment to a specific device.
- `GET /api/v1/models` — model catalog with version metadata.
- `POST /api/v1/models` — register a new model with manifest.
- `GET /api/v1/telemetry` — query device telemetry with time-range filters.
- `GET /api/v1/health` — service health probe (no authentication required).

### F. Backpressure and Rate Limiting

Rate-limit headers follow IETF rate-limit-headers conventions:

```
RateLimit-Limit: 1000, 1000;w=60
RateLimit-Remaining: 947
RateLimit-Reset: 53
```

Throttled requests return HTTP 429 with `Retry-After` per RFC 9110.

### G. Pagination

List endpoints use cursor-based pagination with the `Link` header per RFC 8288.

### H. Webhook Conformance

When the API supports outbound webhooks, webhook conformance follows the three-rule contract: HMAC-SHA-256 signatures over the payload, idempotent delivery identifiers, and exponential-backoff retries up to a configurable maximum.

### I. Capability Discovery

Clients discover server capabilities through the `/.well-known/wia-edge-ai/capabilities` endpoint. The response includes supported runtime profiles, supported hardware target classes, supported accelerator families, supported observability protocols, and the active rate-limit policy.

Capability discovery is part of every conformant client's initialisation flow so the client adapts to the server's actual capabilities rather than assuming a fixed superset.

### J. CORS

CORS responses follow the WHATWG Fetch Living Standard. The reference deployment publishes the allow-list of origins, methods, and headers and reviews it quarterly.

### K. Health and Readiness Probes

`GET /health` returns 200 when the service is operational, 503 when not. `GET /ready` returns 200 when the service is ready to accept production traffic. These endpoints are exempt from authentication and follow the conventions documented in the Cloud Native Computing Foundation operator guidance.

---

**Copyright © 2025 World Certification Industry Association (WIA)**
**License:** CC BY 4.0
**弘益人間** - Benefit All Humanity
