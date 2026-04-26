/**
 * WIA-SEMI-004 TypeScript SDK - Type Definitions
 * 
 * © 2025 SmileStory Inc. / WIA
 * License: MIT
 */

export enum Precision {
  INT4 = "INT4",
  INT8 = "INT8",
  INT16 = "INT16",
  FP16 = "FP16",
  BF16 = "BF16",
  FP32 = "FP32",
  FP8_E4M3 = "FP8_E4M3",
  FP8_E5M2 = "FP8_E5M2"
}

export enum ChipType {
  NPU = "NPU",
  TPU = "TPU",
  GPU = "GPU",
  ASIC = "ASIC",
  FPGA = "FPGA"
}

export interface ChipSpecification {
  name: string;
  vendor: string;
  type: ChipType;
  tops: number;
  precision: Precision;
  memoryGB: number;
  memoryBandwidthGBps: number;
  tdpWatts: number;
  processNode: number; // nm
}

export interface PerformanceMetrics {
  throughput: number; // inferences/sec
  latency_p50: number; // ms
  latency_p95: number; // ms
  latency_p99: number; // ms
  power: number; // watts
  topsPerWatt: number;
  utilization: number; // percentage
}

export interface BenchmarkResult {
  chip: string;
  model: string;
  batchSize: number;
  precision: Precision;
  metrics: PerformanceMetrics;
  timestamp: Date;
}

export interface ModelConfig {
  path: string;
  precision: Precision;
  batchSize?: number;
  optimization?: OptimizationLevel;
}

export enum OptimizationLevel {
  O0 = 0, // No optimization
  O1 = 1, // Basic
  O2 = 2, // Recommended
  O3 = 3  // Aggressive
}

export interface InferenceRequest {
  inputs: Tensor[];
  outputNames?: string[];
}

export interface InferenceResponse {
  outputs: Tensor[];
  latency: number; // ms
}

export interface Tensor {
  name: string;
  shape: number[];
  dtype: Precision;
  data: Float32Array | Int8Array | Uint8Array | Int32Array;
}

export interface DeviceInfo {
  name: string;
  vendor: string;
  driverVersion: string;
  firmwareVersion: string;
  totalMemory: number; // bytes
  availableMemory: number; // bytes
}
