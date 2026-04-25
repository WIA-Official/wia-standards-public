/**
 * WIA-AI-011: AI Chip Interface Standard
 * TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

// ===== Data Types =====

export type DataType =
  | 'FLOAT64'
  | 'FLOAT32'
  | 'FLOAT16'
  | 'BFLOAT16'
  | 'TF32'
  | 'FP8_E4M3'
  | 'FP8_E5M2'
  | 'INT64'
  | 'INT32'
  | 'INT16'
  | 'INT8'
  | 'INT4'
  | 'UINT64'
  | 'UINT32'
  | 'UINT16'
  | 'UINT8'
  | 'UINT4';

export type MemoryLayout = 'NCHW' | 'NHWC' | 'CHWN' | 'CUSTOM';

export type ByteOrder = 'little_endian' | 'big_endian';

// ===== Quantization =====

export interface QuantizationParams {
  scheme: 'per_tensor_symmetric' | 'per_tensor_asymmetric' | 'per_channel_symmetric' | 'per_channel_asymmetric';
  scale: number | number[];
  zero_point: number | number[];
  range: [number, number];
  channel_axis?: number;
}

// ===== Tensor Descriptor =====

export interface TensorShape {
  dimensions: number[] | string[];
  layout: MemoryLayout;
  symbolic: boolean;
  constraints?: Record<string, { min?: number; max?: number; multiple_of?: number }>;
}

export interface TensorDType {
  base_type: DataType;
  quantization: QuantizationParams | null;
  byte_order: ByteOrder;
}

export interface DeviceDescriptor {
  type: 'NPU' | 'TPU' | 'GPU' | 'CPU';
  id: number;
  memory_space: 'device' | 'host' | 'unified';
}

export interface TensorMemory {
  strides: number[];
  offset_bytes: number;
  total_bytes: number;
  alignment: number;
  device: DeviceDescriptor;
}

export interface TensorProperties {
  requires_grad: boolean;
  is_pinned: boolean;
  is_contiguous: boolean;
  zero_copy_compatible: boolean;
}

export interface TensorDescriptor {
  version: string;
  tensor_id: string;
  name: string;
  shape: TensorShape;
  dtype: TensorDType;
  memory: TensorMemory;
  properties: TensorProperties;
  metadata?: Record<string, any>;
}

// ===== Device Management =====

export type DeviceType = 'NPU' | 'TPU' | 'GPU' | 'CPU';

export interface DeviceInfo {
  name: string;
  type: DeviceType;
  vendor: string;
  memory_size: number;
  peak_tflops: number;
  wia_version: string;
}

export interface DeviceCapabilities {
  peak_tflops_fp32: number;
  peak_tflops_fp16: number;
  peak_tflops_int8: number;
  total_memory: number;
  memory_bandwidth_gbps: number;
  supports_unified_memory: boolean;
  supports_pinned_memory: boolean;
  supports_fp16: boolean;
  supports_bfloat16: boolean;
  supports_int8: boolean;
  supports_sparse_ops: boolean;
  supports_dynamic_shapes: boolean;
  max_tensor_dims: number;
  max_tensor_size: number;
  max_batch_size: number;
}

// ===== Context Management =====

export interface ContextConfig {
  device: Device;
  memory_pool_size: number;
  max_streams: number;
  profiling_enabled: boolean;
}

export interface Context {
  config: ContextConfig;
  synchronize(): Promise<void>;
  destroy(): Promise<void>;
}

// ===== Stream Operations =====

export type StreamType = 'COMPUTE' | 'TRANSFER' | 'GENERIC';

export interface Stream {
  type: StreamType;
  synchronize(): Promise<void>;
  query(): Promise<boolean>;
  destroy(): Promise<void>;
}

export interface Event {
  record(stream: Stream): Promise<void>;
  synchronize(): Promise<void>;
  destroy(): Promise<void>;
}

// ===== Memory Operations =====

export type MemoryType = 'DEVICE' | 'PINNED' | 'UNIFIED' | 'MANAGED';

export interface MemoryDescriptor {
  size: number;
  type: MemoryType;
  alignment: number;
  flags: number;
  device_affinity?: Device;
}

export interface Buffer {
  descriptor: MemoryDescriptor;
  free(): Promise<void>;
  map(): Promise<ArrayBuffer>;
  unmap(): Promise<void>;
}

// ===== Tensor Operations =====

export interface MatMulConfig {
  transpose_a: boolean;
  transpose_b: boolean;
  alpha: number;
  beta: number;
}

export interface Conv2DConfig {
  stride: [number, number];
  padding: [number, number];
  dilation: [number, number];
  groups: number;
}

export interface SoftmaxConfig {
  axis: number;
  temperature: number;
}

export interface Tensor {
  descriptor: TensorDescriptor;
  buffer: Buffer;
  reshape(shape: number[]): Promise<Tensor>;
  transpose(axes: number[]): Promise<Tensor>;
  to(device: Device): Promise<Tensor>;
  free(): Promise<void>;
}

// ===== Profiling =====

export type ProfileMode = 'SAMPLING' | 'INSTRUMENTATION' | 'HARDWARE_COUNTERS';

export interface ProfilingConfig {
  profiling_enabled: boolean;
  mode: ProfileMode;
}

export interface KernelProfile {
  name: string;
  duration_ms: number;
  compute_utilization: number;
  memory_bandwidth_gbps: number;
  cache_hit_rate: number;
}

export interface ProfileResults {
  total_time_ms: number;
  kernels: KernelProfile[];
}

// ===== Status and Errors =====

export enum Status {
  SUCCESS = 0,
  ERROR_INVALID_ARGUMENT = 1,
  ERROR_OUT_OF_MEMORY = 2,
  ERROR_DEVICE_NOT_FOUND = 3,
  ERROR_NOT_SUPPORTED = 4,
  ERROR_RUNTIME_ERROR = 5,
}

export interface ErrorInfo {
  code: Status;
  message: string;
  file?: string;
  line?: number;
  details?: string;
}

// ===== Device Interface =====

export interface Device {
  id: number;
  type: DeviceType;
  getInfo(): Promise<DeviceInfo>;
  getCapabilities(): Promise<DeviceCapabilities>;
  createContext(config: ContextConfig): Promise<Context>;
  allocateMemory(descriptor: MemoryDescriptor): Promise<Buffer>;
  createTensor(descriptor: TensorDescriptor): Promise<Tensor>;
}

// ===== Communicator (Multi-Device) =====

export type ReduceOp = 'SUM' | 'PROD' | 'MIN' | 'MAX' | 'AVG';
export type AllReduceAlgorithm = 'RING' | 'TREE' | 'RABENSEIFNER';

export interface CollectiveConfig {
  op: ReduceOp;
  datatype: DataType;
  algorithm: AllReduceAlgorithm;
}

export interface Communicator {
  devices: Device[];
  num_devices: number;
  rank: number;
  allReduce(sendBuffer: Buffer, recvBuffer: Buffer, count: number, config: CollectiveConfig, stream?: Stream): Promise<void>;
  broadcast(buffer: Buffer, count: number, root: number, datatype: DataType, stream?: Stream): Promise<void>;
  allGather(sendBuffer: Buffer, recvBuffer: Buffer, count: number, datatype: DataType, stream?: Stream): Promise<void>;
  barrier(stream?: Stream): Promise<void>;
  synchronize(): Promise<void>;
  destroy(): Promise<void>;
}
