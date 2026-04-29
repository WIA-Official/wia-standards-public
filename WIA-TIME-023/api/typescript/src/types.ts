/**
 * WIA-TIME-023: Temporal Tether - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Geometry Types
// ============================================================================

/**
 * Three-dimensional spatial coordinates
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Spacetime coordinates (4D)
 */
export interface SpacetimeCoordinates {
  /** Spatial coordinates in meters */
  position: Vector3;

  /** Temporal coordinate */
  time: Date | string;

  /** Reference frame identifier */
  referenceFrame?: string;
}

// ============================================================================
// Tether Types
// ============================================================================

/**
 * Temporal tether type
 */
export type TetherType =
  | 'point_to_point'
  | 'multi_point'
  | 'broadcast'
  | 'emergency';

/**
 * Tether status
 */
export type TetherStatus =
  | 'establishing'
  | 'active'
  | 'degraded'
  | 'recovering'
  | 'failed'
  | 'offline';

/**
 * Tether mode (direction of data flow)
 */
export type TetherMode = 'unidirectional' | 'bidirectional';

/**
 * Network topology for multi-point tethers
 */
export type NetworkTopology = 'star' | 'mesh' | 'tree' | 'ring';

/**
 * Temporal tether configuration
 */
export interface TemporalTether {
  /** Unique tether identifier */
  id: string;

  /** Tether type */
  type: TetherType;

  /** Primary endpoint */
  endpoint1: SpacetimeCoordinates;

  /** Secondary endpoint */
  endpoint2: SpacetimeCoordinates;

  /** Data transfer bandwidth in bits/second */
  bandwidth: number;

  /** Tether strength (0-1) */
  strength: number;

  /** Communication mode */
  mode: TetherMode;

  /** Current status */
  status: TetherStatus;

  /** Establishment timestamp */
  established: Date;

  /** Last maintenance timestamp */
  lastMaintenance?: Date;

  /** Encryption method */
  encryption: string;

  /** Redundancy level (1-10) */
  redundancy?: number;

  /** Health metrics */
  health?: TetherHealth;

  /** Custom metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Tether health metrics
 */
export interface TetherHealth {
  /** Overall health status */
  overall: 'excellent' | 'good' | 'fair' | 'poor' | 'critical';

  /** Tether strength (0-1) */
  strength: number;

  /** Decoherence rate (s⁻¹) */
  decoherenceRate: number;

  /** Bit error rate */
  bitErrorRate: number;

  /** Round-trip latency in milliseconds */
  latency: number;

  /** Packet loss rate (0-1) */
  packetLoss: number;

  /** Quantum correlation measure (0-1) */
  quantumCorrelation: number;

  /** Signal-to-noise ratio in dB */
  snr: number;

  /** Uptime in seconds */
  uptime: number;

  /** Total data transferred in bytes */
  dataTransferred: number;

  /** Last heartbeat timestamp */
  lastHeartbeat: Date;

  /** Predicted time to failure in seconds */
  timeToFailure?: number;
}

// ============================================================================
// Tether Establishment
// ============================================================================

/**
 * Tether establishment parameters
 */
export interface TetherEstablishment {
  /** Primary endpoint */
  endpoint1: SpacetimeCoordinates;

  /** Secondary endpoint */
  endpoint2: SpacetimeCoordinates;

  /** Desired bandwidth in bits/second */
  bandwidth: number;

  /** Target strength (0-1) */
  strength: number;

  /** Communication mode */
  mode: TetherMode;

  /** Redundancy level (default: 1) */
  redundancy?: number;

  /** Encryption method */
  encryption?: string;

  /** Custom metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Tether establishment result
 */
export interface TetherResult {
  /** Success status */
  success: boolean;

  /** Created tether ID */
  tetherId: string;

  /** Actual achieved strength */
  actualStrength: number;

  /** Time taken to establish in seconds */
  establishmentTime: number;

  /** Estimated lifetime in seconds */
  estimatedLifetime: number;

  /** Energy consumed in joules */
  energyConsumed: number;

  /** Warnings */
  warnings?: string[];

  /** Errors (if failed) */
  errors?: string[];
}

// ============================================================================
// Multi-Point Tethering
// ============================================================================

/**
 * Multi-point tether configuration
 */
export interface MultiPointTether {
  /** List of endpoints to connect */
  endpoints: SpacetimeCoordinates[];

  /** Network topology */
  topology: NetworkTopology;

  /** Bandwidth per link in bits/second */
  bandwidth: number;

  /** Redundancy level */
  redundancy: number;

  /** Encryption method */
  encryption?: string;
}

/**
 * Multi-point tether network
 */
export interface TetherNetwork {
  /** Unique network identifier */
  networkId: string;

  /** Network name */
  name: string;

  /** Network topology */
  topology: NetworkTopology;

  /** All endpoints in network */
  endpoints: TetherEndpoint[];

  /** Individual tethers in network */
  tethers: TemporalTether[];

  /** Total network capacity in bits/second */
  totalCapacity: number;

  /** Average strength across all tethers */
  averageStrength: number;

  /** Network status */
  status: 'operational' | 'degraded' | 'offline';

  /** Network health metrics */
  health?: NetworkHealth;
}

/**
 * Tether endpoint information
 */
export interface TetherEndpoint {
  /** Endpoint identifier */
  id: string;

  /** Endpoint name */
  name?: string;

  /** Spacetime coordinates */
  coordinates: SpacetimeCoordinates;

  /** Connected tether IDs */
  connectedTethers: string[];

  /** Endpoint status */
  status: 'active' | 'standby' | 'offline';
}

/**
 * Network health metrics
 */
export interface NetworkHealth {
  /** Overall network health */
  overall: 'excellent' | 'good' | 'fair' | 'poor' | 'critical';

  /** Active tethers count */
  activeTethers: number;

  /** Total tethers count */
  totalTethers: number;

  /** Average tether strength */
  avgStrength: number;

  /** Average bit error rate */
  avgBitErrorRate: number;

  /** Average latency in milliseconds */
  avgLatency: number;

  /** Total throughput in bits/second */
  totalThroughput: number;

  /** Redundancy level achieved */
  redundancy: number;

  /** Last update timestamp */
  lastUpdate: Date;
}

/**
 * Multi-point tether result
 */
export interface NetworkResult {
  /** Success status */
  success: boolean;

  /** Created network ID */
  networkId: string;

  /** Individual tether IDs */
  tethers: string[];

  /** Total network capacity */
  totalCapacity: number;

  /** Redundancy level achieved */
  redundancyLevel: number;

  /** Establishment time in seconds */
  establishmentTime: number;

  /** Warnings */
  warnings?: string[];

  /** Errors (if failed) */
  errors?: string[];
}

// ============================================================================
// Data Transfer
// ============================================================================

/**
 * Data transfer parameters
 */
export interface DataTransfer {
  /** Data to transfer */
  data: Buffer | Uint8Array | string | object;

  /** Direction of transfer */
  direction: 'forward' | 'backward' | 'bidirectional';

  /** Transfer priority */
  priority: 'low' | 'medium' | 'high' | 'critical';

  /** Encryption enabled */
  encrypted?: boolean;

  /** Compression enabled */
  compressed?: boolean;

  /** Timeout in milliseconds */
  timeout?: number;

  /** Callback for progress updates */
  onProgress?: (progress: TransferProgress) => void;
}

/**
 * Transfer progress information
 */
export interface TransferProgress {
  /** Bytes transferred */
  bytesTransferred: number;

  /** Total bytes */
  totalBytes: number;

  /** Progress percentage (0-100) */
  percentage: number;

  /** Transfer rate in bytes/second */
  transferRate: number;

  /** Estimated time remaining in seconds */
  estimatedTimeRemaining: number;
}

/**
 * Data transfer result
 */
export interface TransferResult {
  /** Success status */
  success: boolean;

  /** Bytes transferred */
  bytesTransferred: number;

  /** Transfer duration in milliseconds */
  duration: number;

  /** Average transfer rate in bytes/second */
  averageRate: number;

  /** Final tether strength after transfer */
  finalStrength: number;

  /** Errors encountered */
  errors?: string[];
}

// ============================================================================
// Monitoring
// ============================================================================

/**
 * Tether monitoring configuration
 */
export interface MonitoringConfig {
  /** Monitoring interval in milliseconds */
  interval: number;

  /** Metrics to monitor */
  metrics: MonitoringMetric[];

  /** Alert thresholds */
  thresholds: AlertThresholds;

  /** Callback for alerts */
  onAlert?: (alert: TetherAlert) => void;

  /** Callback for updates */
  onUpdate?: (health: TetherHealth) => void;
}

/**
 * Monitoring metric types
 */
export type MonitoringMetric =
  | 'strength'
  | 'decoherence_rate'
  | 'bit_error_rate'
  | 'latency'
  | 'packet_loss'
  | 'throughput'
  | 'quantum_correlation';

/**
 * Alert thresholds
 */
export interface AlertThresholds {
  /** Minimum acceptable strength */
  minStrength?: number;

  /** Maximum decoherence rate */
  maxDecoherenceRate?: number;

  /** Maximum bit error rate */
  maxBitErrorRate?: number;

  /** Maximum latency in milliseconds */
  maxLatency?: number;

  /** Maximum packet loss rate */
  maxPacketLoss?: number;
}

/**
 * Tether alert
 */
export interface TetherAlert {
  /** Alert ID */
  id: string;

  /** Tether ID */
  tetherId: string;

  /** Alert severity */
  severity: 'info' | 'warning' | 'critical';

  /** Alert type */
  type: 'strength_low' | 'ber_high' | 'latency_high' | 'packet_loss' | 'failure';

  /** Alert message */
  message: string;

  /** Current metric value */
  value: number;

  /** Threshold that was exceeded */
  threshold: number;

  /** Alert timestamp */
  timestamp: Date;

  /** Recommended action */
  recommendation?: string;
}

// ============================================================================
// Failover & Recovery
// ============================================================================

/**
 * Failover configuration
 */
export interface FailoverConfig {
  /** Enable automatic failover */
  enabled: boolean;

  /** Backup tether IDs */
  backupTethers: string[];

  /** Failover trigger threshold */
  triggerThreshold: number;

  /** Maximum failover time in seconds */
  maxFailoverTime: number;

  /** Attempt primary recovery */
  attemptRecovery: boolean;

  /** Maximum recovery attempts */
  maxRecoveryAttempts: number;
}

/**
 * Failover event
 */
export interface FailoverEvent {
  /** Event ID */
  id: string;

  /** Failed tether ID */
  failedTetherId: string;

  /** Backup tether ID (if switched) */
  backupTetherId?: string;

  /** Failure reason */
  reason: string;

  /** Failover timestamp */
  timestamp: Date;

  /** Failover duration in seconds */
  duration: number;

  /** Success status */
  success: boolean;

  /** Recovery attempted */
  recoveryAttempted: boolean;
}

/**
 * Recovery parameters
 */
export interface RecoveryParams {
  /** Tether ID to recover */
  tetherId: string;

  /** Maximum recovery time in seconds */
  timeout: number;

  /** Maximum attempts */
  maxAttempts: number;

  /** Force re-establishment if repair fails */
  forceReestablish: boolean;

  /** Callback for progress updates */
  onProgress?: (progress: RecoveryProgress) => void;
}

/**
 * Recovery progress information
 */
export interface RecoveryProgress {
  /** Recovery phase */
  phase: 'diagnosis' | 'isolation' | 'repair' | 'testing' | 'restoration';

  /** Progress percentage (0-100) */
  percentage: number;

  /** Current attempt number */
  attempt: number;

  /** Phase message */
  message: string;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Recovery result
 */
export interface RecoveryResult {
  /** Success status */
  success: boolean;

  /** Recovered tether ID */
  tetherId: string;

  /** Recovery duration in seconds */
  duration: number;

  /** Attempts made */
  attempts: number;

  /** Final tether strength */
  finalStrength: number;

  /** Recovery phases completed */
  phasesCompleted: string[];

  /** Errors encountered */
  errors?: string[];
}

// ============================================================================
// Maintenance
// ============================================================================

/**
 * Maintenance operation types
 */
export type MaintenanceOperation =
  | 'calibration'
  | 'boost'
  | 'purification'
  | 'error_correction'
  | 'synchronization';

/**
 * Maintenance schedule
 */
export interface MaintenanceSchedule {
  /** Tether ID */
  tetherId: string;

  /** Operation type */
  operation: MaintenanceOperation;

  /** Scheduled time */
  scheduledTime: Date;

  /** Estimated duration in seconds */
  estimatedDuration: number;

  /** Automatic or manual */
  automatic: boolean;

  /** Recurrence interval in seconds (0 for one-time) */
  recurrence?: number;
}

/**
 * Maintenance result
 */
export interface MaintenanceResult {
  /** Success status */
  success: boolean;

  /** Tether ID */
  tetherId: string;

  /** Operation performed */
  operation: MaintenanceOperation;

  /** Duration in seconds */
  duration: number;

  /** Strength before maintenance */
  strengthBefore: number;

  /** Strength after maintenance */
  strengthAfter: number;

  /** Improvement achieved */
  improvement: number;

  /** Timestamp */
  timestamp: Date;

  /** Issues resolved */
  issuesResolved?: string[];
}

// ============================================================================
// Boost Operations
// ============================================================================

/**
 * Boost parameters
 */
export interface BoostParams {
  /** Tether ID */
  tetherId: string;

  /** Target strength (0-1) */
  targetStrength: number;

  /** Energy budget in joules */
  energyBudget?: number;

  /** Duration of boost in seconds */
  duration?: number;

  /** Boost mode */
  mode: 'immediate' | 'gradual';
}

/**
 * Boost result
 */
export interface BoostResult {
  /** Success status */
  success: boolean;

  /** Tether ID */
  tetherId: string;

  /** Strength before boost */
  strengthBefore: number;

  /** Strength after boost */
  strengthAfter: number;

  /** Energy consumed in joules */
  energyConsumed: number;

  /** Boost duration in seconds */
  duration: number;

  /** Estimated effect duration in seconds */
  effectDuration: number;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical and system constants
 */
export const TETHER_CONSTANTS = {
  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Planck constant (J·s) */
  PLANCK: 6.62607015e-34,

  /** Reduced Planck constant (J·s) */
  HBAR: 1.054571817e-34,

  /** Typical decoherence rates (s⁻¹) */
  DECOHERENCE: {
    LOW: 1e-9,
    MEDIUM: 1e-8,
    HIGH: 1e-6,
  },

  /** Strength thresholds */
  STRENGTH: {
    EXCELLENT: 0.95,
    GOOD: 0.80,
    FAIR: 0.60,
    POOR: 0.40,
    CRITICAL: 0.20,
  },

  /** Bit error rate thresholds */
  BER: {
    EXCELLENT: 1e-9,
    GOOD: 1e-6,
    FAIR: 1e-3,
    POOR: 0.01,
    CRITICAL: 0.1,
  },

  /** Latency ranges (ms) */
  LATENCY: {
    MIN: 0.001,
    TYPICAL: 1.0,
    MAX: 100.0,
  },

  /** Bandwidth ranges (bits/second) */
  BANDWIDTH: {
    MIN: 1e9, // 1 Gb/s
    TYPICAL: 1e12, // 1 Tb/s
    MAX: 1e15, // 1 Pb/s
  },

  /** Energy costs (joules) */
  ENERGY: {
    ESTABLISH: 1e-5, // 10 μJ
    BOOST: 1e-6, // 1 μJ
    MAINTAIN: 1e-7, // 100 nJ per second
  },

  /** Timeout values (seconds) */
  TIMEOUT: {
    HEARTBEAT: 30,
    ESTABLISHMENT: 300,
    RECOVERY: 600,
    FAILOVER: 30,
  },
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
 * WIA-TIME-023 error codes
 */
export enum TetherErrorCode {
  INSUFFICIENT_ENTANGLEMENT = 'TT001',
  CAUSALITY_VIOLATION = 'TT002',
  ENDPOINT_UNREACHABLE = 'TT003',
  STRENGTH_TOO_LOW = 'TT004',
  DECOHERENCE_TOO_HIGH = 'TT005',
  BANDWIDTH_EXCEEDED = 'TT006',
  FAILOVER_FAILED = 'TT007',
  RECOVERY_FAILED = 'TT008',
  ESTABLISHMENT_TIMEOUT = 'TT009',
  ENCRYPTION_FAILURE = 'TT010',
  NETWORK_PARTITION = 'TT011',
  ENDPOINT_COLLISION = 'TT012',
}

/**
 * Tether error class
 */
export class TetherError extends Error {
  constructor(
    public code: TetherErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'TetherError';
  }
}

// ============================================================================
// Events
// ============================================================================

/**
 * Tether event types
 */
export type TetherEventType =
  | 'established'
  | 'degraded'
  | 'boosted'
  | 'failed'
  | 'recovered'
  | 'data_received'
  | 'data_sent'
  | 'maintenance_due'
  | 'threshold_exceeded';

/**
 * Tether event
 */
export interface TetherEvent {
  /** Event type */
  type: TetherEventType;

  /** Tether ID */
  tetherId: string;

  /** Event timestamp */
  timestamp: Date;

  /** Event data */
  data?: Record<string, unknown>;
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  Vector3,
  SpacetimeCoordinates,

  // Tether
  TetherType,
  TetherStatus,
  TetherMode,
  NetworkTopology,
  TemporalTether,
  TetherHealth,

  // Establishment
  TetherEstablishment,
  TetherResult,

  // Multi-point
  MultiPointTether,
  TetherNetwork,
  TetherEndpoint,
  NetworkHealth,
  NetworkResult,

  // Data transfer
  DataTransfer,
  TransferProgress,
  TransferResult,

  // Monitoring
  MonitoringConfig,
  MonitoringMetric,
  AlertThresholds,
  TetherAlert,

  // Failover & Recovery
  FailoverConfig,
  FailoverEvent,
  RecoveryParams,
  RecoveryProgress,
  RecoveryResult,

  // Maintenance
  MaintenanceOperation,
  MaintenanceSchedule,
  MaintenanceResult,

  // Boost
  BoostParams,
  BoostResult,

  // Events
  TetherEventType,
  TetherEvent,
};

export { TETHER_CONSTANTS, TetherErrorCode, TetherError };
