/**
 * WIA-TIME-020: Temporal Beacon - TypeScript Type Definitions
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
// Beacon Types
// ============================================================================

/**
 * Temporal beacon type
 */
export type BeaconType =
  | 'fixed_primary'
  | 'fixed_secondary'
  | 'mobile'
  | 'emergency'
  | 'micro';

/**
 * Beacon status
 */
export type BeaconStatus =
  | 'active'
  | 'standby'
  | 'maintenance'
  | 'offline'
  | 'emergency';

/**
 * Signal modulation type
 */
export type ModulationType = 'TPSK' | 'TFSK' | 'CDMA' | 'OFDM';

/**
 * Temporal beacon configuration
 */
export interface TemporalBeacon {
  /** Unique beacon identifier */
  id: string;

  /** Beacon type */
  type: BeaconType;

  /** Spatial position in meters */
  position: Vector3;

  /** Temporal anchor point (fixed time reference) */
  temporalAnchor: Date;

  /** Signal configuration */
  signal: {
    /** Carrier frequency in Hz */
    frequency: number;

    /** Transmission power in watts */
    power: number;

    /** Modulation type */
    modulation: ModulationType;

    /** Phase offset (encodes beacon ID) */
    phaseOffset?: number;
  };

  /** Coverage range */
  range: {
    /** Spatial range in meters */
    spatial: number;

    /** Temporal range in seconds (±) */
    temporal: number;
  };

  /** Network membership */
  network: {
    /** Network identifier */
    networkId: string;

    /** Network hierarchy level (0-3) */
    level: number;

    /** Priority (higher = more important) */
    priority: number;
  };

  /** Current status */
  status: BeaconStatus;

  /** Deployment timestamp */
  deployed: Date;

  /** Last maintenance timestamp */
  lastMaintenance?: Date;

  /** Health metrics */
  health?: BeaconHealth;

  /** Custom metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Beacon health metrics
 */
export interface BeaconHealth {
  /** Overall health status */
  overall: 'excellent' | 'good' | 'fair' | 'poor' | 'critical';

  /** Signal quality metrics */
  signalQuality: {
    /** Signal-to-Noise Ratio in dB */
    SNR: number;

    /** Bit Error Rate */
    BER: number;
  };

  /** Power system status */
  power: {
    /** Battery level (0-100%) */
    battery: number;

    /** Solar panel status */
    solar: 'optimal' | 'degraded' | 'offline';

    /** Estimated runtime in hours */
    runtime: number;
  };

  /** Clock synchronization */
  clock: {
    /** Frequency drift in Hz/s */
    drift: number;

    /** Stability rating */
    stability: 'excellent' | 'good' | 'fair' | 'poor';

    /** Last sync timestamp */
    lastSync: Date;

    /** Sync error in nanoseconds */
    syncError: number;
  };

  /** Operating temperature in Celsius */
  temperature: number;

  /** Position verification */
  position: {
    /** GPS/stellar verified position */
    verified: Vector3;

    /** Position drift from nominal in meters */
    drift: number;
  };

  /** Network connectivity */
  connectivity: {
    /** Number of peer beacons visible */
    peersVisible: number;

    /** Network latency in milliseconds */
    latency: number;
  };
}

// ============================================================================
// Beacon Signals
// ============================================================================

/**
 * Beacon signal measurement
 */
export interface BeaconSignal {
  /** Beacon identifier */
  beaconId: string;

  /** Received signal strength (0-1 normalized) */
  signalStrength: number;

  /** Time delay in seconds */
  timeDelay: number;

  /** Received frequency in Hz */
  frequency: number;

  /** Signal-to-noise ratio in dB */
  SNR?: number;

  /** Reception timestamp */
  receivedAt: Date;

  /** Decoded beacon data */
  decodedData?: BeaconData;
}

/**
 * Decoded beacon transmission data
 */
export interface BeaconData {
  /** Beacon ID (from signal) */
  beaconId: string;

  /** Beacon position */
  position: Vector3;

  /** Temporal anchor */
  temporalAnchor: Date;

  /** Network ID */
  networkId: string;

  /** Status flags */
  status: number;

  /** Checksum */
  checksum: number;

  /** Checksum valid? */
  checksumValid: boolean;
}

// ============================================================================
// Positioning
// ============================================================================

/**
 * Temporal position result from triangulation
 */
export interface TemporalPosition {
  /** Spatial coordinates in meters */
  coordinates: Vector3;

  /** Temporal coordinate */
  time: Date;

  /** Position accuracy */
  accuracy: {
    /** Spatial accuracy in meters */
    spatial: number;

    /** Temporal accuracy in seconds */
    temporal: number;
  };

  /** Confidence level (0-1) */
  confidence: number;

  /** Beacon IDs used for triangulation */
  beaconsUsed: string[];

  /** Geometric Dilution of Precision */
  GDOP: number;

  /** Calculation method */
  method: 'triangulation' | 'trilateration' | 'kalman' | 'weighted_least_squares';

  /** Raw signal measurements */
  rawSignals?: BeaconSignal[];
}

/**
 * Triangulation parameters
 */
export interface TriangulationParams {
  /** Beacon signals to use */
  signals: BeaconSignal[];

  /** Reference beacons (with known positions) */
  beacons: TemporalBeacon[];

  /** Minimum required beacons (default: 4) */
  minBeacons?: number;

  /** Use weighted least squares? */
  weighted?: boolean;

  /** Use Kalman filtering? */
  kalman?: boolean;

  /** Previous position (for Kalman) */
  previousPosition?: TemporalPosition;
}

// ============================================================================
// Navigation
// ============================================================================

/**
 * Temporal waypoint
 */
export interface TemporalWaypoint {
  /** Unique waypoint identifier */
  id: string;

  /** Waypoint name */
  name: string;

  /** Spacetime coordinates */
  coordinates: SpacetimeCoordinates;

  /** Associated beacon ID */
  beaconId?: string;

  /** Waypoint importance level */
  importance: 'critical' | 'high' | 'medium' | 'low';

  /** Safety level */
  safetyLevel: 'safe' | 'caution' | 'danger' | 'prohibited';

  /** Waypoint type */
  type: 'historical' | 'emergency' | 'tourism' | 'scientific' | 'navigation';

  /** Description */
  description?: string;

  /** Custom metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Navigation route
 */
export interface NavigationRoute {
  /** Route identifier */
  id: string;

  /** Ordered list of waypoints */
  waypoints: TemporalWaypoint[];

  /** Total route distance (spatial + temporal) */
  totalDistance: {
    /** Spatial distance in meters */
    spatial: number;

    /** Temporal distance in seconds */
    temporal: number;
  };

  /** Estimated travel time */
  estimatedDuration: number;

  /** Risk assessment */
  riskLevel: 'low' | 'medium' | 'high' | 'extreme';

  /** Route safety score (0-100) */
  safetyScore: number;

  /** Required energy in joules */
  energyRequired?: number;
}

/**
 * Route planning parameters
 */
export interface RoutePlanningParams {
  /** Starting waypoint or coordinates */
  start: TemporalWaypoint | SpacetimeCoordinates;

  /** Destination waypoint or coordinates */
  destination: TemporalWaypoint | SpacetimeCoordinates;

  /** Available waypoints for routing */
  waypoints?: TemporalWaypoint[];

  /** Maximum acceptable risk */
  maxRisk?: 'low' | 'medium' | 'high';

  /** Optimization criterion */
  optimize?: 'shortest' | 'fastest' | 'safest' | 'energy';

  /** Avoid specific waypoints */
  avoid?: string[];
}

// ============================================================================
// Beacon Network
// ============================================================================

/**
 * Beacon network configuration
 */
export interface BeaconNetwork {
  /** Network identifier */
  networkId: string;

  /** Network name */
  name: string;

  /** Coverage area */
  coverage: 'global' | 'continental' | 'regional' | 'local';

  /** Temporal coverage range */
  temporalRange: {
    /** Start time */
    start: Date | string;

    /** End time */
    end: Date | string;
  };

  /** Beacons in this network */
  beacons: TemporalBeacon[];

  /** Network master beacon */
  masterBeacon?: string;

  /** Synchronization protocol */
  syncProtocol: 'NTP' | 'PTP' | 'custom';

  /** Network status */
  status: 'operational' | 'degraded' | 'offline';

  /** Network health metrics */
  health?: NetworkHealth;
}

/**
 * Network health metrics
 */
export interface NetworkHealth {
  /** Overall network health */
  overall: 'excellent' | 'good' | 'fair' | 'poor' | 'critical';

  /** Active beacons count */
  activeBeacons: number;

  /** Total beacons count */
  totalBeacons: number;

  /** Average signal quality */
  avgSignalQuality: number;

  /** Network synchronization error (ns) */
  syncError: number;

  /** Coverage percentage (0-100) */
  coverage: number;

  /** Last update timestamp */
  lastUpdate: Date;
}

/**
 * Coverage map
 */
export interface CoverageMap {
  /** Network ID */
  networkId: string;

  /** Coverage data as 4D grid */
  grid: CoveragePoint[];

  /** Resolution (grid spacing) */
  resolution: {
    /** Spatial resolution in meters */
    spatial: number;

    /** Temporal resolution in seconds */
    temporal: number;
  };

  /** Coverage statistics */
  statistics: {
    /** Total coverage volume */
    totalVolume: number;

    /** Excellent coverage percentage */
    excellentCoverage: number;

    /** Good coverage percentage */
    goodCoverage: number;

    /** Poor coverage percentage */
    poorCoverage: number;
  };
}

/**
 * Single coverage point
 */
export interface CoveragePoint {
  /** Position */
  position: SpacetimeCoordinates;

  /** Number of visible beacons */
  beaconCount: number;

  /** Coverage quality */
  quality: 'excellent' | 'good' | 'adequate' | 'poor';

  /** Average signal strength */
  avgSignalStrength: number;

  /** GDOP at this point */
  GDOP: number;
}

// ============================================================================
// Emergency Protocols
// ============================================================================

/**
 * Emergency severity level
 */
export type EmergencySeverity = 'CRITICAL' | 'HIGH' | 'MEDIUM' | 'LOW';

/**
 * Emergency type
 */
export type EmergencyType =
  | 'DISPLACEMENT'
  | 'ENERGY'
  | 'PARADOX'
  | 'MEDICAL'
  | 'EQUIPMENT'
  | 'OTHER';

/**
 * Emergency beacon transmission
 */
export interface EmergencyBeacon {
  /** Emergency beacon ID */
  id: string;

  /** Severity level */
  severity: EmergencySeverity;

  /** Emergency type */
  type: EmergencyType;

  /** Current spacetime position */
  position: SpacetimeCoordinates;

  /** Distress message */
  message: string;

  /** Vital signs (if available) */
  vitals?: VitalSigns;

  /** System status */
  systemStatus?: SystemStatus;

  /** Auto-activated or manual? */
  autoActivated: boolean;

  /** Activation timestamp */
  activatedAt: Date;

  /** Contact information */
  contact?: ContactInfo;
}

/**
 * Vital signs data
 */
export interface VitalSigns {
  /** Heart rate (bpm) */
  heartRate?: number;

  /** Blood pressure (systolic/diastolic) */
  bloodPressure?: { systolic: number; diastolic: number };

  /** Body temperature (Celsius) */
  temperature?: number;

  /** Oxygen saturation (%) */
  oxygenSaturation?: number;

  /** Consciousness level */
  consciousness?: 'alert' | 'drowsy' | 'unconscious';

  /** Timestamp */
  timestamp: Date;
}

/**
 * System status for emergency
 */
export interface SystemStatus {
  /** Energy remaining (%) */
  energyLevel: number;

  /** Temporal field stability (0-1) */
  fieldStability?: number;

  /** Equipment failures */
  failures?: string[];

  /** Diagnostics data */
  diagnostics?: Record<string, unknown>;
}

/**
 * Contact information
 */
export interface ContactInfo {
  /** Name */
  name?: string;

  /** Emergency contact */
  emergencyContact?: string;

  /** Time traveler ID */
  travelerId?: string;

  /** Home time */
  homeTime?: Date;
}

/**
 * Emergency response
 */
export interface EmergencyResponse {
  /** Response ID */
  responseId: string;

  /** Emergency beacon ID being responded to */
  emergencyId: string;

  /** Estimated arrival time (seconds) */
  estimatedArrival: number;

  /** Rescue assets deployed */
  rescueAssets: RescueAsset[];

  /** Instructions for the casualty */
  instructions: string;

  /** Response status */
  status: 'dispatched' | 'en_route' | 'on_scene' | 'extracting' | 'complete';

  /** Response team contact */
  contactFrequency?: number;
}

/**
 * Rescue asset
 */
export interface RescueAsset {
  /** Asset ID */
  id: string;

  /** Asset type */
  type: 'temporal_vehicle' | 'emergency_beacon' | 'medical_team' | 'support';

  /** Current position */
  position: SpacetimeCoordinates;

  /** ETA in seconds */
  eta: number;

  /** Asset status */
  status: 'standby' | 'deployed' | 'active' | 'returning';
}

// ============================================================================
// Deployment
// ============================================================================

/**
 * Beacon deployment parameters
 */
export interface BeaconDeployment {
  /** Beacon ID */
  id: string;

  /** Beacon type */
  type: BeaconType;

  /** Deployment position */
  position: Vector3;

  /** Temporal anchor */
  temporalAnchor: Date;

  /** Signal power in watts */
  signalPower: number;

  /** Coverage range */
  range: {
    /** Spatial range in meters */
    spatial: number;

    /** Temporal range in seconds */
    temporal: number;
  };

  /** Signal frequency in Hz */
  frequency: number;

  /** Network to join */
  networkId: string;

  /** Priority level */
  priority?: number;

  /** Metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Deployment result
 */
export interface DeploymentResult {
  /** Success status */
  success: boolean;

  /** Deployed beacon ID */
  beaconId: string;

  /** Activation timestamp */
  activationTime: Date;

  /** Estimated coverage map */
  estimatedCoverage: CoverageMap;

  /** Warnings */
  warnings?: string[];

  /** Errors (if failed) */
  errors?: string[];
}

// ============================================================================
// Maintenance
// ============================================================================

/**
 * Calibration data
 */
export interface CalibrationData {
  /** Beacon ID */
  beaconId: string;

  /** Calibration timestamp */
  timestamp: Date;

  /** Frequency drift in Hz */
  frequencyDrift: number;

  /** Power variation (%) */
  powerVariation: number;

  /** Sync error in nanoseconds */
  syncError: number;

  /** Position drift in meters */
  positionDrift: number;

  /** Calibration passed? */
  passed: boolean;

  /** Adjustments made */
  adjustments?: {
    frequency?: number;
    power?: number;
    position?: Vector3;
  };
}

/**
 * Synchronization result
 */
export interface SyncResult {
  /** Beacons synchronized */
  beacons: string[];

  /** Master beacon */
  masterBeacon: string;

  /** Synchronization timestamp */
  timestamp: Date;

  /** Maximum sync error (ns) */
  maxSyncError: number;

  /** Average sync error (ns) */
  avgSyncError: number;

  /** Success status */
  success: boolean;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical and system constants
 */
export const BEACON_CONSTANTS = {
  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Standard frequencies (Hz) */
  FREQUENCIES: {
    EMERGENCY: 10e12, // 10 THz
    PRIMARY: 5e12, // 5 THz
    SECONDARY: 2.5e12, // 2.5 THz
    MICRO: 1.25e12, // 1.25 THz
  },

  /** Standard power levels (watts) */
  POWER: {
    PRIMARY: 1e16,
    SECONDARY: 1e13,
    MOBILE: 1e11,
    EMERGENCY: 1e17,
    MICRO: 1e9,
  },

  /** Standard ranges */
  RANGE: {
    PRIMARY: { spatial: 10e6, temporal: 3.154e9 }, // 10,000 km, ±100 years
    SECONDARY: { spatial: 1e6, temporal: 1.577e9 }, // 1,000 km, ±50 years
    LOCAL: { spatial: 1e5, temporal: 3.154e8 }, // 100 km, ±10 years
    MICRO: { spatial: 1e4, temporal: 86400 }, // 10 km, ±1 day
  },

  /** Temporal attenuation coefficient (s⁻¹) */
  TEMPORAL_ATTENUATION: 3.17e-10,

  /** Minimum detectable signal (W/m²) */
  MIN_SIGNAL: 1.0,

  /** Minimum required beacons for 4D positioning */
  MIN_BEACONS: 4,

  /** Clock synchronization accuracy (ns) */
  SYNC_ACCURACY: 1,

  /** Calibration thresholds */
  CALIBRATION: {
    MAX_FREQ_DRIFT: 1000, // Hz
    MAX_POWER_VAR: 0.05, // 5%
    MAX_SYNC_ERROR: 10, // ns
    MAX_POS_DRIFT: 1, // meter
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
 * WIA-TIME-020 error codes
 */
export enum BeaconErrorCode {
  INSUFFICIENT_SIGNAL = 'B001',
  CLOCK_DESYNC = 'B002',
  TOO_FEW_BEACONS = 'B003',
  POSITION_AMBIGUOUS = 'B004',
  EMERGENCY_JAMMED = 'B005',
  BEACON_OFFLINE = 'B006',
  NETWORK_PARTITION = 'B007',
  INVALID_DEPLOYMENT = 'B008',
  CALIBRATION_FAILED = 'B009',
  COVERAGE_GAP = 'B010',
}

/**
 * Beacon error class
 */
export class BeaconError extends Error {
  constructor(
    public code: BeaconErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'BeaconError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  Vector3,
  SpacetimeCoordinates,

  // Beacon
  BeaconType,
  BeaconStatus,
  ModulationType,
  TemporalBeacon,
  BeaconHealth,
  BeaconSignal,
  BeaconData,

  // Positioning
  TemporalPosition,
  TriangulationParams,

  // Navigation
  TemporalWaypoint,
  NavigationRoute,
  RoutePlanningParams,

  // Network
  BeaconNetwork,
  NetworkHealth,
  CoverageMap,
  CoveragePoint,

  // Emergency
  EmergencySeverity,
  EmergencyType,
  EmergencyBeacon,
  VitalSigns,
  SystemStatus,
  ContactInfo,
  EmergencyResponse,
  RescueAsset,

  // Deployment
  BeaconDeployment,
  DeploymentResult,

  // Maintenance
  CalibrationData,
  SyncResult,
};

export { BEACON_CONSTANTS, BeaconErrorCode, BeaconError };
