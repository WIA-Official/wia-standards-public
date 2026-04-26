/**
 * WIA-COMM-010: Network Slicing - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Network Slicing Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Network slice types based on 3GPP specifications
 */
export type SliceType = 'eMBB' | 'URLLC' | 'mMTC' | 'hybrid';

/**
 * Resource isolation levels
 */
export type IsolationLevel = 'dedicated' | 'shared' | 'best-effort';

/**
 * Slice lifecycle states
 */
export type SliceState =
  | 'preparing'
  | 'instantiating'
  | 'configuring'
  | 'active'
  | 'modifying'
  | 'deactivating'
  | 'terminated'
  | 'error';

/**
 * QoS Class Identifier (3GPP TS 23.501)
 */
export type QCI = 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9;

// ============================================================================
// Network Slice Configuration
// ============================================================================

/**
 * Service Level Agreement policy
 */
export interface SLAPolicy {
  // Availability guarantee (0-1, e.g., 0.9999 = 99.99%)
  availability: number;

  // Performance guarantees
  maxLatency: number; // milliseconds
  minBandwidth: number; // Mbps
  maxPacketLoss: number; // 0-1
  maxJitter: number; // milliseconds

  // Financial terms
  violationPenalty?: number; // USD per violation
  compensationRate?: number; // Percentage

  // Monitoring
  monitoringInterval: number; // seconds
  alertThreshold: number; // Percentage (0-100)
}

/**
 * Network slice configuration
 */
export interface NetworkSliceConfig {
  // Identity
  id?: string;
  name: string;
  description?: string;
  type: SliceType;

  // Performance requirements
  bandwidth: number; // Mbps
  latency: number; // milliseconds
  reliability: number; // 0-1 (e.g., 0.999999 = 99.9999%)
  jitter?: number; // milliseconds
  packetLoss?: number; // 0-1

  // Resource allocation
  isolation: IsolationLevel;
  cpuCores?: number;
  memory?: number; // GB
  storage?: number; // GB

  // QoS
  qci?: QCI;
  priority?: number; // 1-15 (1 = highest)

  // SLA
  sla: SLAPolicy;

  // Multi-tenancy
  tenantId: string;
  subscribers?: number;

  // Network topology
  ranNodes?: string[];
  coreNodes?: string[];
  edgeNodes?: string[];

  // Coverage
  coverageArea?: string;
  geoFencing?: GeographicArea;

  // Advanced features
  enableTSN?: boolean; // Time-Sensitive Networking
  enableCDN?: boolean; // Content Delivery Network
  enableEdgeCompute?: boolean;
}

/**
 * Geographic area definition
 */
export interface GeographicArea {
  type: 'circle' | 'polygon';
  center?: { latitude: number; longitude: number };
  radius?: number; // meters
  polygon?: Array<{ latitude: number; longitude: number }>;
}

/**
 * Resource update for slice modification
 */
export interface ResourceUpdate {
  bandwidth?: number;
  cpuCores?: number;
  memory?: number;
  subscribers?: number;
}

// ============================================================================
// Slice Templates
// ============================================================================

/**
 * Pre-configured slice templates
 */
export enum SliceTemplate {
  AUTONOMOUS_VEHICLE = 'autonomous-vehicle',
  SMART_FACTORY = 'smart-factory',
  VIDEO_STREAMING = 'video-streaming',
  IOT_SENSOR = 'iot-sensor',
  REMOTE_SURGERY = 'remote-surgery',
  CLOUD_GAMING = 'cloud-gaming',
  SMART_CITY = 'smart-city',
  EMERGENCY_SERVICES = 'emergency-services',
}

// ============================================================================
// Network Slice Instance
// ============================================================================

/**
 * Active network slice instance
 */
export interface NetworkSlice {
  // Configuration
  config: NetworkSliceConfig;

  // Runtime state
  state: SliceState;
  createdAt: Date;
  activatedAt?: Date;
  lastModifiedAt?: Date;

  // Network endpoints
  ranNode?: string;
  coreNode?: string;
  edgeNode?: string;

  // Statistics
  activeConnections: number;
  totalTraffic: number; // GB
  resourceUtilization: number; // Percentage

  // SLA compliance
  slaCompliance: number; // Percentage
  violations: SLAViolation[];
}

/**
 * SLA violation record
 */
export interface SLAViolation {
  timestamp: Date;
  metric: 'latency' | 'bandwidth' | 'packetLoss' | 'jitter' | 'availability';
  expected: number;
  actual: number;
  duration: number; // seconds
  severity: 'warning' | 'critical' | 'emergency';
  penalty?: number; // USD
}

// ============================================================================
// Monitoring and Metrics
// ============================================================================

/**
 * Real-time slice metrics
 */
export interface SliceMetrics {
  timestamp: Date;
  sliceId: string;

  // Performance metrics
  latency: number; // milliseconds
  throughput: number; // Mbps
  packetLoss: number; // 0-1
  jitter: number; // milliseconds

  // Resource utilization
  cpuUsage: number; // Percentage
  memoryUsage: number; // Percentage
  bandwidthUsage: number; // Percentage

  // Connection statistics
  activeConnections: number;
  totalConnections: number;
  connectionRate: number; // connections/second

  // Quality metrics
  signalQuality: number; // dB
  snr: number; // Signal-to-Noise Ratio (dB)

  // SLA compliance
  slaCompliance: number; // Percentage
  uptime: number; // Percentage
}

/**
 * Aggregated slice statistics
 */
export interface SliceStatistics {
  sliceId: string;
  period: { start: Date; end: Date };

  // Performance statistics
  avgLatency: number;
  maxLatency: number;
  p99Latency: number;
  avgThroughput: number;
  peakThroughput: number;
  totalTraffic: number; // GB

  // Reliability statistics
  totalPackets: number;
  lostPackets: number;
  packetLossRate: number;

  // Availability
  uptime: number; // Percentage
  downtime: number; // seconds
  incidents: number;

  // SLA
  slaCompliance: number;
  violations: number;
  totalPenalties: number; // USD
}

// ============================================================================
// SDN/NFV Integration
// ============================================================================

/**
 * SDN controller configuration
 */
export interface SDNController {
  id: string;
  endpoint: string;
  protocol: 'OpenFlow' | 'NETCONF' | 'REST';
  version: string;
  credentials?: {
    username: string;
    password: string;
    apiKey?: string;
  };
}

/**
 * Virtual Network Function (VNF)
 */
export interface VNF {
  type: 'vAMF' | 'vSMF' | 'vUPF' | 'vPCF' | 'vUDM' | 'vNSSF';
  instance: string;
  status: 'active' | 'inactive' | 'scaling' | 'failed';
  resources: {
    cpuCores: number;
    memory: number; // GB
    storage: number; // GB
  };
  endpoint: string;
}

/**
 * NFV Infrastructure (NFVI)
 */
export interface NFVI {
  platform: 'OpenStack' | 'Kubernetes' | 'VMware' | 'AWS' | 'Azure';
  region: string;
  availabilityZone?: string;

  compute: {
    totalCores: number;
    availableCores: number;
    totalMemory: number; // GB
    availableMemory: number; // GB
  };

  storage: {
    totalStorage: number; // GB
    availableStorage: number; // GB
  };

  network: {
    totalBandwidth: number; // Gbps
    availableBandwidth: number; // Gbps
  };
}

// ============================================================================
// Orchestrator Configuration
// ============================================================================

/**
 * Network slice orchestrator configuration
 */
export interface OrchestratorConfig {
  controller: string;
  nfvPlatform: string;
  region: string;
  credentials?: {
    apiKey?: string;
    username?: string;
    password?: string;
  };
  options?: {
    autoScaling?: boolean;
    autoHealing?: boolean;
    monitoring?: boolean;
    logging?: boolean;
  };
}

// ============================================================================
// Result Types
// ============================================================================

/**
 * Operation result
 */
export interface Result<T> {
  success: boolean;
  data?: T;
  error?: string;
  code?: string;
}

/**
 * Async operation result
 */
export type AsyncResult<T> = Promise<Result<T>>;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Network slicing error codes
 */
export enum ErrorCode {
  INSUFFICIENT_RESOURCES = 'INSUFFICIENT_RESOURCES',
  INVALID_CONFIGURATION = 'INVALID_CONFIGURATION',
  SLA_VIOLATION = 'SLA_VIOLATION',
  ORCHESTRATION_FAILED = 'ORCHESTRATION_FAILED',
  AUTHENTICATION_FAILED = 'AUTHENTICATION_FAILED',
  SLICE_NOT_FOUND = 'SLICE_NOT_FOUND',
  NETWORK_ERROR = 'NETWORK_ERROR',
  TIMEOUT = 'TIMEOUT',
}

/**
 * Network slicing error
 */
export class NetworkSlicingError extends Error {
  code: ErrorCode;
  details?: any;

  constructor(code: ErrorCode, message: string, details?: any) {
    super(message);
    this.code = code;
    this.details = details;
    this.name = 'NetworkSlicingError';
  }
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Network slicing constants
 */
export const NETWORK_SLICING_CONSTANTS = {
  // Latency budgets (ms)
  LATENCY: {
    URLLC_TARGET: 1,
    EMBB_TARGET: 20,
    MMTC_TARGET: 1000,
  },

  // Reliability targets
  RELIABILITY: {
    URLLC: 0.999999, // 99.9999%
    EMBB: 0.999, // 99.9%
    MMTC: 0.99, // 99%
  },

  // Device density (devices/km²)
  DEVICE_DENSITY: {
    MMTC_TARGET: 1000000,
  },

  // QCI mappings
  QCI_MAPPING: {
    URLLC_CONTROL: 1,
    URLLC_DATA: 2,
    EMBB_REALTIME: 3,
    EMBB_VIDEO: 4,
    EMBB_BESTEFFORT: 5,
    MMTC_BACKGROUND: 9,
  },

  // SLA defaults
  SLA_DEFAULTS: {
    MONITORING_INTERVAL: 1, // seconds
    ALERT_THRESHOLD: 95, // percentage
  },
};

// ============================================================================
// Export All
// ============================================================================

export * from './types';
