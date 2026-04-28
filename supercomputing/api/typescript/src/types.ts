/**
 * WIA-COMP-001: Supercomputing - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Computing Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Performance tiers for supercomputers
 */
export type PerformanceTier = 'exascale' | 'petascale' | 'terascale' | 'high-performance';

/**
 * Compute architectures
 */
export type ComputeArchitecture = 'x86-64' | 'ARM' | 'RISC-V' | 'Power' | 'custom';

/**
 * Accelerator types
 */
export type AcceleratorType = 'GPU' | 'FPGA' | 'TPU' | 'ASIC' | 'none';

// ============================================================================
// System Configuration
// ============================================================================

/**
 * Compute node configuration
 */
export interface ComputeNode {
  /** Node identifier */
  id: string;

  /** CPU configuration */
  cpu: {
    architecture: ComputeArchitecture;
    sockets: number;
    coresPerSocket: number;
    clockSpeed: number; // Hz
    vectorWidth: number; // bits (e.g., 512 for AVX-512)
    flopPerCycle: number; // FLOP per cycle per core
  };

  /** Memory configuration */
  memory: {
    capacity: number; // bytes
    type: 'DDR4' | 'DDR5' | 'HBM2' | 'HBM3';
    bandwidth: number; // bytes/sec
  };

  /** Accelerators (GPUs, FPGAs, etc.) */
  accelerators?: {
    type: AcceleratorType;
    count: number;
    peakTFLOPS: number; // per accelerator
    memory: number; // bytes per accelerator
    memoryBandwidth: number; // bytes/sec
  };

  /** Local storage */
  storage?: {
    type: 'NVMe' | 'SSD' | 'HDD';
    capacity: number; // bytes
    bandwidth: number; // bytes/sec
  };

  /** Power consumption */
  power: {
    idle: number; // watts
    typical: number; // watts
    max: number; // watts
  };
}

/**
 * Cluster configuration
 */
export interface ClusterConfig {
  /** Cluster identifier */
  id: string;

  /** Cluster name */
  name: string;

  /** Number of compute nodes */
  nodes: number;

  /** Node configuration template */
  nodeConfig: ComputeNode;

  /** Interconnect network */
  interconnect: {
    technology: 'InfiniBand-HDR' | 'InfiniBand-NDR' | 'Ethernet-100G' | 'Ethernet-200G' | 'RoCE' | 'custom';
    bandwidth: number; // bits/sec per link
    latency: number; // seconds
    topology: 'fat-tree' | 'dragonfly' | 'torus' | 'mesh' | 'custom';
  };

  /** Storage system */
  storage: {
    filesystem: 'Lustre' | 'GPFS' | 'BeeGFS' | 'NFS' | 'custom';
    capacity: number; // bytes
    bandwidth: number; // bytes/sec aggregate
    burstBuffer?: {
      capacity: number; // bytes
      bandwidth: number; // bytes/sec
    };
  };

  /** Power and cooling */
  infrastructure: {
    totalPower: number; // watts
    pue: number; // Power Usage Effectiveness
    coolingType: 'air' | 'liquid' | 'hybrid';
  };
}

// ============================================================================
// Performance Metrics
// ============================================================================

/**
 * FLOPS calculation parameters
 */
export interface FLOPSParams {
  /** Number of compute nodes */
  nodes: number;

  /** Cores per node */
  coresPerNode: number;

  /** CPU clock speed (Hz) */
  clockSpeed: number;

  /** FLOP per cycle (e.g., 32 for AVX-512 double-precision) */
  flopPerCycle: number;

  /** Accelerator configuration */
  accelerators?: {
    type: AcceleratorType;
    count: number; // per node
    peakTFLOPS: number; // per accelerator
  };

  /** Vectorization efficiency (0-1) */
  vectorizationEfficiency?: number;

  /** Memory bandwidth limitation factor (0-1) */
  memoryEfficiency?: number;
}

/**
 * FLOPS calculation result
 */
export interface FLOPSResult {
  /** Theoretical peak FLOPS */
  peakFLOPS: number;

  /** CPU contribution */
  cpuFLOPS: number;

  /** Accelerator contribution */
  acceleratorFLOPS: number;

  /** Performance tier */
  tier: PerformanceTier;

  /** Expected sustained performance (accounting for efficiency) */
  sustainedFLOPS: number;

  /** Efficiency ratio (sustained/peak) */
  efficiency: number;

  /** Formatted human-readable string */
  formatted: string;
}

/**
 * Scalability analysis parameters
 */
export interface ScalabilityParams {
  /** Baseline configuration */
  baseline: {
    nodes: number;
    time: number; // seconds
  };

  /** Target configuration */
  target: {
    nodes: number;
  };

  /** Scaling type */
  scalingType: 'strong' | 'weak';

  /** Algorithm characteristics */
  algorithm: {
    /** Fraction of serial code (Amdahl's law) */
    serialFraction?: number;

    /** Communication overhead model */
    commOverhead?: {
      latency: number; // seconds
      bandwidthPerByte: number; // seconds/byte
      messageSize: number; // bytes
      messagesPerIteration: number;
    };
  };
}

/**
 * Scalability analysis result
 */
export interface ScalabilityResult {
  /** Predicted execution time */
  estimatedTime: number; // seconds

  /** Speedup factor */
  speedup: number;

  /** Parallel efficiency (0-1) */
  efficiency: number;

  /** Scaling regime assessment */
  assessment: 'excellent' | 'good' | 'moderate' | 'poor';

  /** Recommended node count */
  recommendedNodes: number;

  /** Bottleneck analysis */
  bottlenecks: string[];
}

// ============================================================================
// Benchmarking
// ============================================================================

/**
 * Benchmark types
 */
export type BenchmarkType = 'LINPACK' | 'HPCG' | 'STREAM' | 'IOR' | 'custom';

/**
 * Benchmark configuration
 */
export interface BenchmarkConfig {
  /** Benchmark type */
  type: BenchmarkType;

  /** Problem size */
  problemSize: number;

  /** Number of nodes to use */
  nodes: number;

  /** Cores per node */
  coresPerNode: number;

  /** Custom parameters */
  parameters?: Record<string, unknown>;
}

/**
 * Benchmark result
 */
export interface BenchmarkResult {
  /** Benchmark type */
  type: BenchmarkType;

  /** Achieved performance */
  performance: number; // FLOPS or MB/s depending on benchmark

  /** Peak theoretical performance */
  theoretical: number;

  /** Efficiency percentage */
  efficiency: number;

  /** Execution time */
  executionTime: number; // seconds

  /** Additional metrics */
  metrics: Record<string, number>;

  /** Pass/fail status */
  passed: boolean;

  /** Logs and details */
  details: string[];
}

// ============================================================================
// Workload Management
// ============================================================================

/**
 * Job submission parameters
 */
export interface JobSubmission {
  /** Job name */
  name: string;

  /** Executable path */
  executable: string;

  /** Command-line arguments */
  args: string[];

  /** Resource requirements */
  resources: {
    nodes: number;
    coresPerNode?: number;
    walltime: number; // seconds
    memory?: number; // bytes per node
    gpus?: number; // per node
  };

  /** Working directory */
  workdir: string;

  /** Environment variables */
  environment?: Record<string, string>;

  /** Job dependencies */
  dependencies?: string[]; // job IDs

  /** Queue/partition */
  queue?: string;

  /** Email notifications */
  notifications?: {
    email: string;
    events: ('start' | 'end' | 'fail')[];
  };
}

/**
 * Job status
 */
export interface JobStatus {
  /** Job ID */
  id: string;

  /** Job name */
  name: string;

  /** Current state */
  state: 'pending' | 'running' | 'completed' | 'failed' | 'cancelled';

  /** Allocated resources */
  allocation?: {
    nodes: string[];
    startTime: Date;
  };

  /** Completion info */
  completion?: {
    endTime: Date;
    exitCode: number;
    walltime: number; // actual seconds used
  };

  /** Resource usage */
  usage?: {
    cpuHours: number;
    memoryPeak: number; // bytes
    energyConsumed: number; // joules
  };
}

// ============================================================================
// Network Topology
// ============================================================================

/**
 * Network topology validation
 */
export interface TopologyValidation {
  /** Is topology valid? */
  isValid: boolean;

  /** Topology metrics */
  metrics?: {
    diameter: number; // max hops between any two nodes
    bisectionBandwidth: number; // bits/sec
    avgHops: number;
    linkUtilization: number; // 0-1
  };

  /** Warnings */
  warnings: string[];

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Energy Efficiency
// ============================================================================

/**
 * Energy analysis parameters
 */
export interface EnergyParams {
  /** System configuration */
  system: ClusterConfig;

  /** Workload characteristics */
  workload: {
    utilizationCPU: number; // 0-1
    utilizationGPU?: number; // 0-1
    duration: number; // seconds
  };

  /** Power management */
  powerManagement?: {
    dvfs: boolean; // Dynamic Voltage/Frequency Scaling
    powerCapping?: number; // watts per node
  };
}

/**
 * Energy analysis result
 */
export interface EnergyResult {
  /** Total energy consumed */
  totalEnergy: number; // joules

  /** Average power draw */
  averagePower: number; // watts

  /** Power breakdown */
  breakdown: {
    compute: number; // watts
    network: number; // watts
    storage: number; // watts
    cooling: number; // watts
    other: number; // watts
  };

  /** Energy efficiency metrics */
  metrics: {
    flopsPerWatt: number;
    pue: number; // Power Usage Effectiveness
    dcie: number; // Data Center Infrastructure Efficiency
  };

  /** Cost estimate */
  costEstimate?: {
    energyCost: number; // dollars (assuming $0.10/kWh)
    carbonFootprint: number; // kg CO2
  };
}

// ============================================================================
// Fault Tolerance
// ============================================================================

/**
 * Checkpoint configuration
 */
export interface CheckpointConfig {
  /** Checkpoint method */
  method: 'coordinated' | 'uncoordinated' | 'incremental';

  /** Checkpoint interval */
  interval: number; // seconds

  /** Checkpoint size */
  dataSize: number; // bytes per node

  /** Storage location */
  storage: 'local' | 'burst-buffer' | 'pfs';

  /** Compression */
  compression?: {
    enabled: boolean;
    ratio: number; // compressed/original
  };
}

/**
 * Reliability analysis
 */
export interface ReliabilityAnalysis {
  /** System MTBF (Mean Time Between Failures) */
  mtbf: number; // seconds

  /** Recommended checkpoint interval */
  optimalCheckpointInterval: number; // seconds

  /** Expected number of failures during job */
  expectedFailures: number;

  /** Overhead from checkpointing */
  checkpointOverhead: number; // fraction of total time

  /** Total time including restarts */
  estimatedTotalTime: number; // seconds
}

// ============================================================================
// Physical Constants and Utilities
// ============================================================================

/**
 * Supercomputing constants
 */
export const HPC_CONSTANTS = {
  /** FLOPS thresholds */
  EXAFLOPS: 1e18,
  PETAFLOPS: 1e15,
  TERAFLOPS: 1e12,
  GIGAFLOPS: 1e9,

  /** Power efficiency targets */
  TARGET_GFLOPS_PER_WATT: 50,

  /** Typical efficiency factors */
  TYPICAL_SUSTAINED_EFFICIENCY: 0.7, // 70% of peak

  /** Standard energy costs */
  ENERGY_COST_PER_KWH: 0.10, // dollars

  /** Carbon emission factor */
  CO2_PER_KWH: 0.5, // kg CO2
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-COMP-001 error codes
 */
export enum CompErrorCode {
  INVALID_CONFIGURATION = 'COMP001',
  INSUFFICIENT_RESOURCES = 'COMP002',
  BENCHMARK_FAILED = 'COMP003',
  TOPOLOGY_INVALID = 'COMP004',
  SCALABILITY_POOR = 'COMP005',
  ENERGY_EXCEEDED = 'COMP006',
  JOB_SUBMISSION_FAILED = 'COMP007',
  CHECKPOINT_FAILED = 'COMP008',
  INVALID_PARAMETERS = 'COMP009',
  SYSTEM_UNAVAILABLE = 'COMP010',
}

/**
 * Supercomputing error
 */
export class SupercomputingError extends Error {
  constructor(
    public code: CompErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'SupercomputingError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  PerformanceTier,
  ComputeArchitecture,
  AcceleratorType,
  ComputeNode,
  ClusterConfig,
  FLOPSParams,
  FLOPSResult,
  ScalabilityParams,
  ScalabilityResult,
  BenchmarkType,
  BenchmarkConfig,
  BenchmarkResult,
  JobSubmission,
  JobStatus,
  TopologyValidation,
  EnergyParams,
  EnergyResult,
  CheckpointConfig,
  ReliabilityAnalysis,
};

export { HPC_CONSTANTS, CompErrorCode, SupercomputingError };
