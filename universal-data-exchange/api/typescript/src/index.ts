/**
 * WIA-CORE-003: Universal Data Exchange - SDK Implementation
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Core Standards Group
 *
 * 弘익人間 (Benefit All Humanity)
 */

import * as crypto from 'crypto';
import { v4 as uuidv4 } from 'uuid';
import {
  DataEnvelope,
  EnvelopeMeta,
  IntegrityInfo,
  UDEConfig,
  DEFAULT_CONFIG,
  DataFormat,
  Encoding,
  CompressionAlgorithm,
  HashAlgorithm,
  ValidationResult,
  ValidationError,
  SchemaCompatibility,
  TransformationPipelineConfig,
  TransformationStep,
  TransformationType,
  TransformationResult,
  PerformanceMetrics,
  StreamOptions,
  StreamChunk,
  Result,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * Universal Data Exchange SDK
 *
 * Provides methods for creating, validating, transforming, and exchanging
 * data envelopes according to the WIA-CORE-003 standard.
 */
export class UniversalDataExchange {
  private config: Required<UDEConfig>;
  private schemaCache: Map<string, unknown>;
  private metrics: PerformanceMetrics;

  constructor(config: Partial<UDEConfig> = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };
    this.schemaCache = new Map();
    this.metrics = this.initializeMetrics();
  }

  // ==========================================================================
  // Envelope Creation
  // ==========================================================================

  /**
   * Create a new Universal Data Envelope
   *
   * @param options - Envelope creation options
   * @returns Data envelope
   */
  createEnvelope<T = unknown>(options: {
    schema: string;
    version?: string;
    data: T;
    format?: DataFormat;
    encoding?: Encoding;
    compression?: CompressionAlgorithm;
    source?: {
      system?: string;
      endpoint?: string;
      trace?: string;
    };
  }): DataEnvelope<T> {
    const timestamp = new Date().toISOString();
    const dataStr = JSON.stringify(options.data);
    const size = Buffer.byteLength(dataStr, 'utf-8');

    const meta: EnvelopeMeta = {
      id: uuidv4(),
      version: options.version || this.config.defaultVersion,
      schema: options.schema,
      schemaVersion: options.version || this.config.defaultVersion,
      timestamp,
      format: options.format || this.config.defaultFormat,
      encoding: options.encoding || this.config.defaultEncoding,
      compression: options.compression || this.config.defaultCompression,
      size,
      checksum: `${this.config.defaultHashAlgorithm}:${this.calculateChecksum(dataStr)}`,
      source: options.source,
    };

    const envelope: DataEnvelope<T> = {
      meta,
      data: options.data,
      integrity: {
        algorithm: this.config.defaultHashAlgorithm,
        hash: '',
      },
    };

    // Calculate integrity hash
    envelope.integrity.hash = this.calculateIntegrityHash(envelope);

    // Validate if enabled
    if (this.config.validateOnCreate) {
      const validation = this.validateEnvelope(envelope);
      if (!validation.valid && this.config.strictMode) {
        throw new Error(
          `Envelope validation failed: ${validation.errors.map((e) => e.message).join(', ')}`
        );
      }
    }

    this.updateMetrics({ bytes: size });

    return envelope;
  }

  // ==========================================================================
  // Integrity Verification
  // ==========================================================================

  /**
   * Calculate integrity hash for an envelope
   *
   * @param envelope - Data envelope
   * @returns Integrity hash (hex)
   */
  calculateIntegrityHash<T>(envelope: DataEnvelope<T>): string {
    const metaStr = this.canonicalJson(envelope.meta);
    const dataStr = this.canonicalJson(envelope.data);
    const combined = metaStr + dataStr;

    return this.hash(combined, envelope.integrity?.algorithm || this.config.defaultHashAlgorithm);
  }

  /**
   * Verify envelope integrity
   *
   * @param envelope - Data envelope to verify
   * @returns True if integrity is valid
   */
  verifyIntegrity<T>(envelope: DataEnvelope<T>): boolean {
    const expectedHash = envelope.integrity.hash;
    const actualHash = this.calculateIntegrityHash(envelope);
    return expectedHash === actualHash;
  }

  /**
   * Calculate checksum for data
   *
   * @param data - Data string
   * @returns Checksum hash (hex)
   */
  private calculateChecksum(data: string): string {
    return this.hash(data, this.config.defaultHashAlgorithm);
  }

  // ==========================================================================
  // Validation
  // ==========================================================================

  /**
   * Validate envelope structure and data
   *
   * @param envelope - Data envelope
   * @returns Validation result
   */
  validateEnvelope<T>(envelope: DataEnvelope<T>): ValidationResult {
    const errors: ValidationError[] = [];

    // Validate meta fields
    if (!envelope.meta) {
      errors.push({
        field: 'meta',
        message: 'Missing meta field',
        code: 'MISSING_META',
      });
    } else {
      if (!envelope.meta.id) {
        errors.push({
          field: 'meta.id',
          message: 'Missing envelope ID',
          code: 'MISSING_ID',
        });
      }
      if (!envelope.meta.version) {
        errors.push({
          field: 'meta.version',
          message: 'Missing version',
          code: 'MISSING_VERSION',
        });
      }
      if (!envelope.meta.schema) {
        errors.push({
          field: 'meta.schema',
          message: 'Missing schema',
          code: 'MISSING_SCHEMA',
        });
      }
      if (!envelope.meta.timestamp) {
        errors.push({
          field: 'meta.timestamp',
          message: 'Missing timestamp',
          code: 'MISSING_TIMESTAMP',
        });
      }
    }

    // Validate data field
    if (envelope.data === undefined || envelope.data === null) {
      errors.push({
        field: 'data',
        message: 'Missing data field',
        code: 'MISSING_DATA',
      });
    }

    // Validate integrity
    if (!envelope.integrity) {
      errors.push({
        field: 'integrity',
        message: 'Missing integrity field',
        code: 'MISSING_INTEGRITY',
      });
    } else {
      if (!envelope.integrity.algorithm) {
        errors.push({
          field: 'integrity.algorithm',
          message: 'Missing integrity algorithm',
          code: 'MISSING_ALGORITHM',
        });
      }
      if (!envelope.integrity.hash) {
        errors.push({
          field: 'integrity.hash',
          message: 'Missing integrity hash',
          code: 'MISSING_HASH',
        });
      } else {
        // Verify hash
        if (!this.verifyIntegrity(envelope)) {
          errors.push({
            field: 'integrity.hash',
            message: 'Integrity hash mismatch',
            code: 'HASH_MISMATCH',
          });
        }
      }
    }

    // Validate size
    if (envelope.meta?.size && envelope.meta.size > this.config.maxEnvelopeSize) {
      errors.push({
        field: 'meta.size',
        message: `Envelope size exceeds maximum (${this.config.maxEnvelopeSize} bytes)`,
        code: 'SIZE_EXCEEDED',
        expected: this.config.maxEnvelopeSize,
        actual: envelope.meta.size,
      });
    }

    return {
      valid: errors.length === 0,
      errors,
    };
  }

  // ==========================================================================
  // Schema Compatibility
  // ==========================================================================

  /**
   * Calculate schema compatibility score
   *
   * @param sourceVersion - Source schema version
   * @param targetVersion - Target schema version
   * @returns Compatibility score (0-1)
   */
  calculateCompatibility(sourceVersion: string, targetVersion: string): number {
    // Parse semantic versions
    const source = this.parseVersion(sourceVersion);
    const target = this.parseVersion(targetVersion);

    // Calculate version difference
    const versionDiff = Math.abs(target.major - source.major) * 100 +
                        Math.abs(target.minor - source.minor) * 10 +
                        Math.abs(target.patch - source.patch);

    const maxVersionDiff = 1000; // Maximum acceptable difference

    // Simplified compatibility score
    // In production, this would analyze actual schema definitions
    const versionScore = 1 - Math.min(versionDiff / maxVersionDiff, 1);

    // Major version difference = incompatible
    if (source.major !== target.major) {
      return versionScore * 0.5;
    }

    return versionScore;
  }

  /**
   * Check schema compatibility between versions
   *
   * @param sourceVersion - Source schema version
   * @param targetVersion - Target schema version
   * @returns Compatibility information
   */
  checkSchemaCompatibility(
    sourceVersion: string,
    targetVersion: string
  ): SchemaCompatibility {
    const score = this.calculateCompatibility(sourceVersion, targetVersion);

    return {
      sourceVersion,
      targetVersion,
      score,
      compatible: score >= 0.7,
      matchingFields: [], // Would be populated from actual schema analysis
      missingFields: [],
      extraFields: [],
      typeConflicts: [],
    };
  }

  // ==========================================================================
  // Transformation
  // ==========================================================================

  /**
   * Transform data between formats
   *
   * @param envelope - Source envelope
   * @param targetFormat - Target format
   * @returns Transformed envelope
   */
  async transform<T>(
    envelope: DataEnvelope<T>,
    sourceFormat: DataFormat,
    targetFormat: DataFormat
  ): Promise<DataEnvelope<T>> {
    const startTime = Date.now();

    // For now, only support JSON-based transformations
    if (sourceFormat !== DataFormat.JSON || targetFormat !== DataFormat.JSON) {
      throw new Error(`Format transformation ${sourceFormat} → ${targetFormat} not yet implemented`);
    }

    // Return copy with updated format
    const transformed = {
      ...envelope,
      meta: {
        ...envelope.meta,
        format: targetFormat,
      },
    };

    // Recalculate integrity
    transformed.integrity.hash = this.calculateIntegrityHash(transformed);

    this.updateMetrics({ latency: Date.now() - startTime });

    return transformed;
  }

  /**
   * Upgrade data from one schema version to another
   *
   * @param data - Source data
   * @param sourceVersion - Source schema version
   * @param targetVersion - Target schema version
   * @returns Upgraded data
   */
  upgradeSchema<T extends Record<string, unknown>>(
    data: T,
    sourceVersion: string,
    targetVersion: string
  ): T {
    // Simple upgrade - in production, would use migration rules
    const compatibility = this.checkSchemaCompatibility(sourceVersion, targetVersion);

    if (!compatibility.compatible) {
      throw new Error(
        `Schemas ${sourceVersion} and ${targetVersion} are incompatible (score: ${compatibility.score})`
      );
    }

    // Return data as-is for backward-compatible upgrades
    // In production, would apply field mappings, defaults, etc.
    return { ...data };
  }

  // ==========================================================================
  // Transformation Pipeline
  // ==========================================================================

  /**
   * Execute a transformation pipeline
   *
   * @param envelope - Data envelope
   * @param pipeline - Pipeline configuration
   * @returns Transformation result
   */
  async executePipeline<T>(
    envelope: DataEnvelope<T>,
    pipeline: TransformationPipelineConfig
  ): Promise<TransformationResult<T>> {
    const startTime = Date.now();
    let currentData = envelope.data;
    let stepsExecuted = 0;
    let stepsFailed = 0;
    const warnings: string[] = [];

    for (const step of pipeline.steps) {
      try {
        currentData = await this.executeTransformationStep(currentData, step);
        stepsExecuted++;
      } catch (error) {
        stepsFailed++;
        if (!step.continueOnError) {
          return {
            success: false,
            error: `Step '${step.name}' failed: ${error instanceof Error ? error.message : String(error)}`,
            stepsExecuted,
            stepsFailed,
            executionTime: Date.now() - startTime,
          };
        }
        warnings.push(
          `Step '${step.name}' failed but pipeline continued: ${error instanceof Error ? error.message : String(error)}`
        );
      }
    }

    return {
      success: true,
      data: currentData,
      stepsExecuted,
      stepsFailed,
      executionTime: Date.now() - startTime,
      warnings: warnings.length > 0 ? warnings : undefined,
    };
  }

  /**
   * Execute a single transformation step
   *
   * @param data - Input data
   * @param step - Transformation step
   * @returns Transformed data
   */
  private async executeTransformationStep<T>(
    data: T,
    step: TransformationStep
  ): Promise<T> {
    switch (step.type) {
      case TransformationType.VALIDATOR:
        // Validate against schema
        return data; // Simplified - would validate against step.schema

      case TransformationType.MAPPER:
        // Apply field mappings
        if (step.transform && typeof data === 'object' && data !== null) {
          const result = { ...data };
          // Apply transformations (simplified)
          return result as T;
        }
        return data;

      case TransformationType.ENRICHER:
        // Add fields
        if (step.fields && typeof data === 'object' && data !== null) {
          return { ...data, ...step.fields } as T;
        }
        return data;

      case TransformationType.CUSTOM:
        // Execute custom function
        if (step.customFn) {
          const result = await step.customFn(data);
          return result as T;
        }
        return data;

      default:
        return data;
    }
  }

  // ==========================================================================
  // Streaming
  // ==========================================================================

  /**
   * Create a streaming envelope for large datasets
   *
   * @param options - Stream options
   * @returns Async iterator of stream chunks
   */
  async *createStream<T>(options: {
    schema: string;
    version: string;
    data: T[];
    chunkSize?: number;
  }): AsyncGenerator<StreamChunk<T[]>> {
    const chunkSize = options.chunkSize || 100;
    const total = Math.ceil(options.data.length / chunkSize);

    for (let i = 0; i < options.data.length; i += chunkSize) {
      const chunk = options.data.slice(i, i + chunkSize);
      const sequence = Math.floor(i / chunkSize);

      yield {
        sequence,
        total,
        data: chunk,
        isLast: sequence === total - 1,
        checksum: this.calculateChecksum(JSON.stringify(chunk)),
      };
    }
  }

  // ==========================================================================
  // Utilities
  // ==========================================================================

  /**
   * Convert object to canonical JSON (sorted keys, no whitespace)
   *
   * @param obj - Object to canonicalize
   * @returns Canonical JSON string
   */
  private canonicalJson(obj: unknown): string {
    if (obj === null || obj === undefined) {
      return JSON.stringify(obj);
    }

    if (typeof obj !== 'object') {
      return JSON.stringify(obj);
    }

    if (Array.isArray(obj)) {
      return '[' + obj.map((item) => this.canonicalJson(item)).join(',') + ']';
    }

    const sorted = Object.keys(obj as object)
      .sort()
      .map((key) => `"${key}":${this.canonicalJson((obj as Record<string, unknown>)[key])}`)
      .join(',');

    return `{${sorted}}`;
  }

  /**
   * Calculate hash using specified algorithm
   *
   * @param data - Data to hash
   * @param algorithm - Hash algorithm
   * @returns Hash (hex)
   */
  private hash(data: string, algorithm: HashAlgorithm): string {
    const normalizedAlgorithm = algorithm.replace('-', ''); // sha256, sha512, sha3-256, etc.
    return crypto.createHash(normalizedAlgorithm).update(data, 'utf8').digest('hex');
  }

  /**
   * Parse semantic version string
   *
   * @param version - Version string (e.g., "1.2.3")
   * @returns Parsed version
   */
  private parseVersion(version: string): { major: number; minor: number; patch: number } {
    const parts = version.split('.').map((p) => parseInt(p, 10));
    return {
      major: parts[0] || 0,
      minor: parts[1] || 0,
      patch: parts[2] || 0,
    };
  }

  /**
   * Initialize performance metrics
   *
   * @returns Initial metrics
   */
  private initializeMetrics(): PerformanceMetrics {
    return {
      totalEnvelopes: 0,
      totalBytes: 0,
      avgThroughput: 0,
      avgLatency: 0,
      p50Latency: 0,
      p95Latency: 0,
      p99Latency: 0,
      errorRate: 0,
    };
  }

  /**
   * Update performance metrics
   *
   * @param update - Metric updates
   */
  private updateMetrics(update: { bytes?: number; latency?: number }) {
    if (update.bytes) {
      this.metrics.totalBytes += update.bytes;
      this.metrics.totalEnvelopes++;
    }
    if (update.latency) {
      // Simplified - would maintain latency histogram
      this.metrics.avgLatency =
        (this.metrics.avgLatency * (this.metrics.totalEnvelopes - 1) + update.latency) /
        this.metrics.totalEnvelopes;
    }
  }

  /**
   * Get current performance metrics
   *
   * @returns Performance metrics
   */
  getMetrics(): PerformanceMetrics {
    return { ...this.metrics };
  }

  /**
   * Reset performance metrics
   */
  resetMetrics(): void {
    this.metrics = this.initializeMetrics();
  }
}

// ============================================================================
// Transformation Pipeline Builder
// ============================================================================

/**
 * Fluent API for building transformation pipelines
 */
export class TransformationPipeline {
  private steps: TransformationStep[] = [];

  /**
   * Add a transformation step
   *
   * @param name - Step name
   * @param fn - Transformation function
   * @returns This pipeline (for chaining)
   */
  addStep(name: string, fn?: (data: unknown) => unknown | Promise<unknown>): this {
    this.steps.push({
      name,
      type: TransformationType.CUSTOM,
      customFn: fn,
    });
    return this;
  }

  /**
   * Add a validation step
   *
   * @param schema - Schema URI
   * @returns This pipeline (for chaining)
   */
  validate(schema: string): this {
    this.steps.push({
      name: `validate-${schema}`,
      type: TransformationType.VALIDATOR,
      schema,
    });
    return this;
  }

  /**
   * Add a mapping step
   *
   * @param transform - Field transformations
   * @returns This pipeline (for chaining)
   */
  map(transform: Record<string, string>): this {
    this.steps.push({
      name: 'map',
      type: TransformationType.MAPPER,
      transform,
    });
    return this;
  }

  /**
   * Add an enrichment step
   *
   * @param fields - Fields to add
   * @returns This pipeline (for chaining)
   */
  enrich(fields: Record<string, unknown>): this {
    this.steps.push({
      name: 'enrich',
      type: TransformationType.ENRICHER,
      fields,
    });
    return this;
  }

  /**
   * Execute the pipeline
   *
   * @param envelope - Data envelope
   * @returns Transformation result
   */
  async execute<T>(envelope: DataEnvelope<T>): Promise<TransformationResult<T>> {
    const ude = new UniversalDataExchange();
    return ude.executePipeline(envelope, {
      id: 'pipeline-' + Date.now(),
      version: '1.0.0',
      steps: this.steps,
    });
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Validate schema against JSON Schema
 *
 * @param data - Data to validate
 * @param schema - JSON Schema
 * @returns Validation result
 */
export function validateSchema(data: unknown, schema?: unknown): ValidationResult {
  // Simplified validation - in production, would use ajv or similar
  return {
    valid: true,
    errors: [],
  };
}

/**
 * Calculate sample integrity score
 *
 * @param options - Integrity calculation options
 * @returns Integrity score (0-1)
 */
export function calculateSampleIntegrity(options: {
  initialViability: number;
  currentViability: number;
  tempDeviation: number;
  maxTempDeviation: number;
  storageTime: number;
  maxStorageTime: number;
}): number {
  const viabilityRatio = options.currentViability / options.initialViability;
  const tempFactor = 1 - options.tempDeviation / options.maxTempDeviation;
  const timeFactor = 1 - options.storageTime / options.maxStorageTime;

  return Math.max(0, Math.min(1, viabilityRatio * tempFactor * timeFactor));
}

// ============================================================================
// Exports
// ============================================================================

export * from './types';

// Default export
export default UniversalDataExchange;
