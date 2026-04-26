/**
 * WIA-AI-014 Neural Network Format - TypeScript Type Definitions
 *
 * Philosophy: 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

/**
 * Data types supported by WIA-AI-014
 */
export enum DataType {
    FLOAT32 = 1,
    FLOAT16 = 2,
    BFLOAT16 = 3,
    INT64 = 4,
    INT32 = 5,
    INT16 = 6,
    INT8 = 7,
    UINT8 = 8,
    BOOL = 9,
    COMPLEX64 = 10
}

/**
 * Tensor representation
 */
export interface Tensor {
    name: string;
    shape: number[];
    dtype: DataType;
    data: Float32Array | Int8Array | Uint8Array | Buffer;
}

/**
 * Graph node representing an operation
 */
export interface Node {
    name: string;
    opType: string;
    inputs: string[];
    outputs: string[];
    attributes: Record<string, any>;
    docString?: string;
}

/**
 * Value information for graph inputs/outputs
 */
export interface ValueInfo {
    name: string;
    shape: (number | null)[];
    dtype: DataType;
    docString?: string;
}

/**
 * Computational graph
 */
export interface Graph {
    name: string;
    nodes: Node[];
    inputs: ValueInfo[];
    outputs: ValueInfo[];
    initializers: Tensor[];
}

/**
 * Model metadata
 */
export interface ModelMetadata {
    name: string;
    version: string;
    description?: string;
    author?: string;
    license?: string;
    createdAt?: string;
    framework?: string;
    frameworkVersion?: string;
    tags?: string[];

    trainingInfo?: {
        dataset?: string;
        hyperparameters?: Record<string, any>;
        metrics?: Record<string, number>;
    };

    deploymentInfo?: {
        targetPlatforms?: string[];
        inputPreprocessing?: Record<string, any>;
        outputPostprocessing?: Record<string, any>;
    };
}

/**
 * Model structure
 */
export interface WIAModel {
    metadata: ModelMetadata;
    graph: Graph;
    modelVersion: string;
}

/**
 * Tensor map for inputs/outputs
 */
export type TensorMap = Record<string, Tensor>;

/**
 * Load options
 */
export interface LoadOptions {
    validateChecksums?: boolean;
    executionProviders?: string[];
}

/**
 * Save options
 */
export interface SaveOptions {
    compression?: 'none' | 'gzip' | 'zstd';
    optimization?: boolean;
}

/**
 * Inference options
 */
export interface InferenceOptions {
    timeout?: number;
    batchSize?: number;
    enableProfiling?: boolean;
}

/**
 * Optimization level
 */
export enum OptimizationLevel {
    None = 0,
    Basic = 1,
    Extended = 2,
    All = 99
}

/**
 * Session options
 */
export interface SessionOptions {
    executionProviders?: string[];
    graphOptimizationLevel?: OptimizationLevel;
    enableProfiling?: boolean;
    numThreads?: number;
}

/**
 * Validation result
 */
export interface ValidationResult {
    valid: boolean;
    errors: Array<{
        code: string;
        message: string;
        location?: string;
    }>;
    warnings: Array<{
        code: string;
        message: string;
    }>;
}

/**
 * Model statistics
 */
export interface ModelStats {
    numParameters: number;
    numLayers: number;
    modelSizeMB: number;
    flops?: number;
}

/**
 * Performance metrics
 */
export interface PerformanceMetrics {
    latencyMs: number;
    throughput?: number;
    memoryUsageMB?: number;
}

/**
 * Quantization configuration
 */
export interface QuantizationConfig {
    type: 'int8' | 'int4' | 'fp16';
    calibrationData?: TensorMap[];
    perChannel?: boolean;
}

/**
 * TFLite conversion options
 */
export interface TFLiteOptions {
    quantize?: boolean;
    targetOps?: string[];
}

/**
 * Device information
 */
export interface DeviceInfo {
    name: string;
    type: 'cpu' | 'gpu' | 'npu' | 'tpu';
    vendor?: string;
    memory?: number;
}

/**
 * Model version
 */
export interface ModelVersion {
    major: number;
    minor: number;
    patch: number;
}

/**
 * Comparison result
 */
export interface ComparisonResult {
    maxDiff: number;
    meanDiff: number;
    passed: boolean;
    tolerance: number;
}

/**
 * Profiling data
 */
export interface ProfilingData {
    totalTimeMs: number;
    nodeTimings: Array<{
        nodeName: string;
        timeMs: number;
    }>;
}

/**
 * Error codes
 */
export enum ErrorCode {
    INVALID_FORMAT = 'INVALID_FORMAT',
    UNSUPPORTED_OPERATOR = 'UNSUPPORTED_OPERATOR',
    VALIDATION_FAILED = 'VALIDATION_FAILED',
    INFERENCE_FAILED = 'INFERENCE_FAILED',
    CONVERSION_FAILED = 'CONVERSION_FAILED'
}

/**
 * WIA Error
 */
export class WIAError extends Error {
    code: ErrorCode;
    details?: any;

    constructor(code: ErrorCode, message: string, details?: any) {
        super(message);
        this.name = 'WIAError';
        this.code = code;
        this.details = details;
    }
}
