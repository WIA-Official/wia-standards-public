/**
 * WIA-COMM-011: Edge Computing - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Edge Computing Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Geographic coordinates
 */
export interface GeoLocation {
  latitude: number;
  longitude: number;
  altitude?: number;
}

/**
 * Edge computing tiers
 */
export type EdgeTier = 'device' | 'access' | 'regional' | 'cloud';

/**
 * Network types
 */
export type NetworkType = '5g' | '6g' | 'wifi6' | 'ethernet' | 'lte';

/**
 * Workload types
 */
export type WorkloadType = 'stateless' | 'stateful' | 'realtime';

// ============================================================================
// Edge Node
// ============================================================================

/**
 * Edge node classification
 */
export type EdgeNodeType =
  | 'micro-edge'
  | 'small-edge'
  | 'medium-edge'
  | 'large-edge'
  | 'mega-edge';

/**
 * Edge node resource specification
 */
export interface EdgeNodeResources {
  /** CPU cores */
  cpu: number;

  /** Memory in GB */
  memory: number;

  /** Storage in GB */
  storage: number;

  /** Number of GPUs */
  gpu?: number;

  /** GPU model */
  gpuModel?: string;

  /** Network bandwidth in Mbps */
  bandwidth: number;
}

/**
 * Edge node specification
 */
export interface EdgeNode {
  /** Unique node ID */
  id: string;

  /** Node name */
  name: string;

  /** Node type */
  type: EdgeNodeType;

  /** Edge tier */
  tier: EdgeTier;

  /** Geographic location */
  location: GeoLocation;

  /** Available resources */
  resources: EdgeNodeResources;

  /** Resource utilization (0-1) */
  utilization: {
    cpu: number;
    memory: number;
    storage: number;
    gpu?: number;
  };

  /** Network connectivity */
  network: {
    type: NetworkType;
    latency: number; // ms to cloud
    bandwidth: number; // Mbps
  };

  /** Node status */
  status: 'active' | 'maintenance' | 'offline';

  /** Health score (0-100) */
  health: number;
}

// ============================================================================
// Latency
// ============================================================================

/**
 * Latency calculation parameters
 */
export interface LatencyParams {
  /** Source tier */
  sourceType: EdgeTier;

  /** Target tier */
  targetType: EdgeTier;

  /** Distance in meters */
  distance: number;

  /** Network type */
  networkType: NetworkType;

  /** Processing time in ms */
  processingTime?: number;

  /** Queueing delay in ms */
  queueingDelay?: number;
}

/**
 * Latency calculation result
 */
export interface LatencyResult {
  /** Network latency (ms) */
  network: number;

  /** Processing latency (ms) */
  processing: number;

  /** Queueing latency (ms) */
  queueing: number;

  /** Total latency (ms) */
  total: number;

  /** Latency breakdown */
  breakdown: {
    propagation: number; // Physical signal propagation
    transmission: number; // Packet transmission
    processing: number; // Edge processing
    queueing: number; // Waiting in queue
  };

  /** Meets target? */
  meetsTarget?: boolean;

  /** Target latency if specified */
  target?: number;
}

// ============================================================================
// Workload and Deployment
// ============================================================================

/**
 * Container resource requirements
 */
export interface ContainerResources {
  /** CPU cores (can be fractional, e.g., "0.5", "2") */
  cpu: string;

  /** Memory (e.g., "512Mi", "4Gi") */
  memory: string;

  /** GPU count */
  gpu?: number;

  /** Ephemeral storage */
  storage?: string;
}

/**
 * Workload specification
 */
export interface WorkloadSpec {
  /** Workload name */
  name: string;

  /** Container image */
  image: string;

  /** Workload type */
  type: WorkloadType;

  /** Resource requirements */
  resources: {
    requests: ContainerResources;
    limits: ContainerResources;
  };

  /** Environment variables */
  env?: Record<string, string>;

  /** Volume mounts */
  volumes?: Array<{
    name: string;
    mountPath: string;
  }>;

  /** Port mappings */
  ports?: Array<{
    containerPort: number;
    protocol: 'TCP' | 'UDP';
  }>;
}

/**
 * Placement strategy
 */
export type PlacementStrategy =
  | 'latency-optimized'
  | 'cost-optimized'
  | 'hybrid'
  | 'manual';

/**
 * Placement constraints
 */
export interface PlacementConstraints {
  /** Maximum latency in ms */
  maxLatency?: number;

  /** Minimum availability (0-1) */
  minAvailability?: number;

  /** Data residency (e.g., "EU", "US", "CN") */
  dataResidency?: string;

  /** Required node type */
  nodeType?: EdgeNodeType;

  /** Required tier */
  tier?: EdgeTier;

  /** GPU required? */
  requireGPU?: boolean;

  /** Preferred location */
  location?: GeoLocation;

  /** Max distance from location (meters) */
  maxDistance?: number;
}

/**
 * Workload placement request
 */
export interface WorkloadPlacement {
  /** Workload to deploy */
  workload: WorkloadSpec;

  /** Placement strategy */
  placement: {
    strategy: PlacementStrategy;
    constraints: PlacementConstraints;
    preferences?: {
      preferEdge?: boolean;
      costOptimized?: boolean;
    };
  };

  /** Scaling configuration */
  scaling?: {
    min: number;
    max: number;
    metric: 'cpu' | 'memory' | 'latency' | 'requests';
    target: number;
  };

  /** High availability */
  highAvailability?: {
    replicas: number;
    antiAffinity: boolean;
  };
}

/**
 * Deployment result
 */
export interface DeploymentResult {
  /** Deployment ID */
  deploymentId: string;

  /** Status */
  status: 'pending' | 'running' | 'failed' | 'completed';

  /** Selected edge node(s) */
  nodes: string[];

  /** Actual latency achieved */
  latency: number;

  /** Resource allocation */
  allocatedResources: ContainerResources;

  /** Estimated cost per hour */
  costPerHour?: number;

  /** Deployment time */
  deploymentTime: Date;

  /** Errors if any */
  errors?: string[];
}

// ============================================================================
// Edge AI/ML
// ============================================================================

/**
 * Edge AI accelerator types
 */
export type AIAccelerator =
  | 'cpu'
  | 'gpu'
  | 'edge-tpu'
  | 'npu'
  | 'vpu'
  | 'fpga';

/**
 * Model quantization types
 */
export type Quantization = 'fp32' | 'fp16' | 'int8' | 'int4';

/**
 * Edge AI configuration
 */
export interface EdgeAIConfig {
  /** Model name or path */
  model: string;

  /** AI accelerator */
  accelerator: AIAccelerator;

  /** Quantization level */
  quantization: Quantization;

  /** Batch size */
  batchSize?: number;

  /** Framework */
  framework?: 'tensorflow-lite' | 'onnx' | 'pytorch-mobile' | 'openvino';

  /** Model optimization */
  optimization?: {
    pruning?: boolean;
    distillation?: boolean;
    graphOptimization?: boolean;
  };
}

/**
 * Inference request
 */
export interface InferenceRequest {
  /** Input data (base64 encoded or URL) */
  input: string | Buffer;

  /** Input type */
  inputType: 'image' | 'video' | 'audio' | 'text' | 'binary';

  /** Inference timeout in ms */
  timeout?: number;
}

/**
 * Inference result
 */
export interface InferenceResult {
  /** Model output */
  output: any;

  /** Inference latency in ms */
  latency: number;

  /** Confidence score (0-1) */
  confidence?: number;

  /** Model version */
  modelVersion: string;

  /** Accelerator used */
  accelerator: AIAccelerator;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// MEC (Multi-access Edge Computing)
// ============================================================================

/**
 * MEC service types
 */
export type MECServiceType =
  | 'rnis' // Radio Network Information Service
  | 'location'
  | 'bandwidth-management'
  | 'traffic-steering';

/**
 * Radio Network Information
 */
export interface RadioNetworkInfo {
  /** User equipment ID */
  ueId: string;

  /** Cell ID (ECGI) */
  cellId: string;

  /** Cell load (0-100%) */
  cellLoad: number;

  /** Signal strength (dBm) */
  signalStrength: number;

  /** RSRQ (Reference Signal Received Quality) */
  rsrq: number;

  /** RSRP (Reference Signal Received Power) */
  rsrp: number;

  /** Latency to base station (ms) */
  latency: number;

  /** Throughput (Mbps) */
  throughput: number;
}

/**
 * Location service request
 */
export interface LocationRequest {
  /** Device ID */
  deviceId: string;

  /** Accuracy required (meters) */
  accuracy?: number;
}

/**
 * Location service response
 */
export interface LocationResponse {
  /** Device location */
  location: GeoLocation;

  /** Accuracy achieved (meters) */
  accuracy: number;

  /** Timestamp */
  timestamp: Date;

  /** Location method */
  method: 'gps' | 'cell-id' | 'wifi' | 'hybrid';
}

// ============================================================================
// Workload Optimization
// ============================================================================

/**
 * Placement optimization request
 */
export interface OptimizationRequest {
  /** Workload specification */
  workload: WorkloadSpec;

  /** Current deployment (if any) */
  currentNode?: string;

  /** Optimization objective */
  objective: 'minimize-latency' | 'minimize-cost' | 'maximize-availability';

  /** Constraints */
  constraints: PlacementConstraints;

  /** Available nodes */
  availableNodes: EdgeNode[];
}

/**
 * Placement optimization result
 */
export interface OptimizationResult {
  /** Recommended node ID */
  nodeId: string;

  /** Expected latency (ms) */
  expectedLatency: number;

  /** Expected cost per hour */
  expectedCost: number;

  /** Expected availability */
  expectedAvailability: number;

  /** Optimization score (0-100) */
  score: number;

  /** Alternative nodes (ranked) */
  alternatives?: Array<{
    nodeId: string;
    latency: number;
    cost: number;
    score: number;
  }>;

  /** Reasoning */
  reasoning: string;
}

// ============================================================================
// Monitoring and Metrics
// ============================================================================

/**
 * Edge metrics
 */
export interface EdgeMetrics {
  /** Node ID */
  nodeId: string;

  /** Timestamp */
  timestamp: Date;

  /** CPU utilization (0-1) */
  cpuUtilization: number;

  /** Memory utilization (0-1) */
  memoryUtilization: number;

  /** Storage utilization (0-1) */
  storageUtilization: number;

  /** GPU utilization (0-1) */
  gpuUtilization?: number;

  /** Network metrics */
  network: {
    inbound: number; // Mbps
    outbound: number; // Mbps
    latency: number; // ms
    packetLoss: number; // percentage
  };

  /** Workload count */
  workloadCount: number;

  /** Request rate (req/s) */
  requestRate: number;

  /** Error rate (0-1) */
  errorRate: number;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Edge computing constants
 */
export const EDGE_CONSTANTS = {
  /** Latency targets by tier (ms) */
  LATENCY_TARGETS: {
    device: 1,
    access: 5,
    regional: 20,
    cloud: 100,
  },

  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Network latency by type (ms baseline) */
  NETWORK_LATENCY: {
    '5g': 1,
    '6g': 0.5,
    wifi6: 2,
    ethernet: 0.5,
    lte: 10,
  },

  /** Resource limits by node type */
  NODE_RESOURCES: {
    'micro-edge': { cpu: 2, memory: 4, storage: 32, gpu: 0 },
    'small-edge': { cpu: 4, memory: 8, storage: 128, gpu: 0 },
    'medium-edge': { cpu: 8, memory: 16, storage: 512, gpu: 1 },
    'large-edge': { cpu: 16, memory: 32, storage: 1024, gpu: 2 },
    'mega-edge': { cpu: 32, memory: 64, storage: 2048, gpu: 4 },
  },

  /** AI inference latency by accelerator (ms) */
  AI_LATENCY: {
    cpu: 50,
    gpu: 10,
    'edge-tpu': 5,
    npu: 8,
    vpu: 15,
    fpga: 3,
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Edge computing error codes
 */
export enum EdgeErrorCode {
  INVALID_NODE = 'INVALID_NODE',
  INSUFFICIENT_RESOURCES = 'INSUFFICIENT_RESOURCES',
  LATENCY_VIOLATION = 'LATENCY_VIOLATION',
  DEPLOYMENT_FAILED = 'DEPLOYMENT_FAILED',
  OPTIMIZATION_FAILED = 'OPTIMIZATION_FAILED',
  NETWORK_ERROR = 'NETWORK_ERROR',
  AI_INFERENCE_ERROR = 'AI_INFERENCE_ERROR',
  INVALID_PLACEMENT = 'INVALID_PLACEMENT',
  NODE_UNAVAILABLE = 'NODE_UNAVAILABLE',
}

/**
 * Edge computing error
 */
export class EdgeComputingError extends Error {
  constructor(
    public code: EdgeErrorCode,
    message: string,
    public details?: any
  ) {
    super(message);
    this.name = 'EdgeComputingError';
  }
}

// ============================================================================
// Exports
// ============================================================================

export type {
  EdgeNode,
  EdgeNodeResources,
  LatencyParams,
  LatencyResult,
  WorkloadSpec,
  WorkloadPlacement,
  DeploymentResult,
  EdgeAIConfig,
  InferenceRequest,
  InferenceResult,
  OptimizationRequest,
  OptimizationResult,
  EdgeMetrics,
  RadioNetworkInfo,
  LocationRequest,
  LocationResponse,
};
