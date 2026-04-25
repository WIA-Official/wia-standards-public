/**
 * WIA-TIME-004: Temporal Coordinate System - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @organization WIA - World Certification Industry Association
 * @philosophy 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

// ============================================================================
// Basic Types
// ============================================================================

/**
 * 3D Vector
 */
export interface Vector3D {
  x: number;
  y: number;
  z: number;
}

/**
 * 4D Vector (includes time)
 */
export interface Vector4D {
  x: number;
  y: number;
  z: number;
  t: number;
}

/**
 * Uncertainty quantification for coordinates
 */
export interface CoordinateUncertainty {
  /** Spatial uncertainty in meters (3D Euclidean distance) */
  spatial: number;
  /** Temporal uncertainty in seconds */
  temporal: number;
  /** Confidence level (0.0-1.0, typically 0.95 for 95% confidence) */
  confidence: number;
}

// ============================================================================
// Temporal Coordinate System
// ============================================================================

/**
 * 4D Spacetime Coordinate
 */
export interface TemporalCoordinate4D {
  /** X coordinate (longitude in degrees, or X-axis in meters) */
  x: number;
  /** Y coordinate (latitude in degrees, or Y-axis in meters) */
  y: number;
  /** Z coordinate (altitude in meters) */
  z: number;
  /** T coordinate (time in seconds since reference epoch) */
  t: number;
  /** Reference frame identifier */
  referenceFrame: string;
  /** Coordinate uncertainty */
  uncertainty: CoordinateUncertainty;
  /** Optional metadata */
  metadata?: {
    locationName?: string;
    era?: string;
    timezone?: string;
    [key: string]: any;
  };
}

/**
 * Reference frame types
 */
export type ReferenceFrameId =
  | 'EARTH_J2000'
  | 'EARTH_GREENWICH'
  | 'GALACTIC_CENTER'
  | 'SOLAR_BARYCENTER'
  | 'COSMIC_MICROWAVE_BACKGROUND'
  | 'MULTIVERSE_ABSOLUTE'
  | string; // Allow custom frames

/**
 * Time scale types
 */
export type TimeScale =
  | 'TT'    // Terrestrial Time
  | 'TAI'   // International Atomic Time
  | 'UTC'   // Coordinated Universal Time
  | 'GMT'   // Greenwich Mean Time
  | 'TDB'   // Barycentric Dynamical Time
  | 'GST'   // Galactic Standard Time
  | 'AMT'   // Absolute Multiverse Time
  | string;

/**
 * Physical constants for a universe
 */
export interface PhysicalConstants {
  /** Speed of light (m/s) */
  c: number;
  /** Gravitational constant (m³/kg/s²) */
  G: number;
  /** Planck constant (J·s) */
  h: number;
  /** Elementary charge (C) */
  e: number;
  /** Electron mass (kg) */
  me: number;
  /** Proton mass (kg) */
  mp: number;
  /** Fine-structure constant (dimensionless) */
  alpha?: number;
  /** Cosmological constant (m⁻²) */
  lambda?: number;
  /** Custom constants */
  [key: string]: number | undefined;
}

/**
 * Temporal Reference Frame (TRF)
 */
export interface TemporalReferenceFrame {
  /** Unique identifier */
  id: string;
  /** Human-readable name */
  name: string;
  /** Origin point of the frame */
  origin: TemporalCoordinate4D;
  /** Coordinate axes orientation */
  axes: {
    x: Vector3D;
    y: Vector3D;
    z: Vector3D;
  };
  /** Time scale used in this frame */
  timeScale: TimeScale;
  /** Velocity relative to CMB rest frame */
  velocity: Vector3D;
  /** Physical constants (if different from standard) */
  constants?: PhysicalConstants;
  /** Additional metadata */
  metadata?: Record<string, any>;
}

// ============================================================================
// Universal Time Index (UTI)
// ============================================================================

/**
 * Universal Time Index - absolute time measurement
 */
export interface UniversalTimeIndex {
  /** Time value in Planck time units since Big Bang (as bigint for precision) */
  value: bigint;
  /** Reference epoch (0 = Big Bang) */
  epoch: number;
  /** Uncertainty in Planck time units */
  uncertainty: number;
  /** Reference frame used for calculation */
  referenceFrame: string;
}

// ============================================================================
// Timeline Branching
// ============================================================================

/**
 * Timeline state types
 */
export type TimelineState =
  | 'stable'      // Timeline is stable and persistent
  | 'collapsing'  // Timeline is collapsing/merging
  | 'merging'     // Timeline is merging with another
  | 'quantum'     // Timeline in quantum superposition
  | 'hypothetical'; // Theoretical/simulated timeline

/**
 * Timeline branch address
 */
export interface TimelineAddress {
  /** Universe identifier (e.g., "U-P-001") */
  universeId: string;
  /** Branch identifier (e.g., "B-042") */
  branchId: string;
  /** When the branch diverged (UTI or Unix timestamp) */
  divergencePoint: number;
  /** Parent branch ID (null for prime timeline) */
  parentBranch: string | null;
  /** Depth in branch tree (0 = prime timeline) */
  depth: number;
  /** Existence probability (0.0-1.0) */
  probability: number;
  /** Current state of the timeline */
  state: TimelineState;
  /** Child branches (if any) */
  childBranches?: string[];
  /** Additional metadata */
  metadata?: {
    description?: string;
    keyEvents?: string[];
    [key: string]: any;
  };
}

/**
 * Branch divergence point
 */
export interface BranchPoint {
  /** Spacetime coordinate where branch occurred */
  coordinate: TemporalCoordinate4D;
  /** Description of the divergence event */
  event: string;
  /** Parent timeline ID */
  parentBranch: string;
  /** Resulting child branch IDs */
  childBranches: string[];
  /** Probability of this branching event */
  probability: number;
  /** Quantum state descriptor (if applicable) */
  quantumState?: string;
  /** Metadata */
  metadata?: {
    cause?: string;
    significance?: string;
    observers?: string[];
    [key: string]: any;
  };
}

// ============================================================================
// Parallel Universes
// ============================================================================

/**
 * Universe types
 */
export type UniverseType =
  | 'physical'      // Standard physics
  | 'quantum'       // Alternative quantum outcomes
  | 'mathematical'  // Different physical constants
  | 'simulation'    // Computed reality
  | 'hypothetical'; // Theoretical universe

/**
 * Universe topology types
 */
export type UniverseTopology =
  | 'flat'        // Euclidean/Minkowski
  | 'spherical'   // Positive curvature
  | 'hyperbolic'  // Negative curvature
  | 'toroidal'    // Periodic boundaries
  | 'custom';     // Custom topology

/**
 * Parallel universe identifier
 */
export interface ParallelUniverseId {
  /** Universe type */
  type: UniverseType;
  /** Unique index within type */
  index: number;
  /** Physical constants for this universe */
  constants: PhysicalConstants;
  /** Number of spatial dimensions */
  dimensionality: number;
  /** Spacetime topology */
  topology: UniverseTopology;
  /** Accessibility factor (0.0-1.0, how easily reachable) */
  accessibility: number;
  /** Full ID string (e.g., "U-P-001") */
  id?: string;
  /** Metadata */
  metadata?: {
    name?: string;
    description?: string;
    discoveryDate?: number;
    [key: string]: any;
  };
}

/**
 * Cross-universe coordinate mapping
 */
export interface UniverseMapping {
  /** Source universe ID */
  sourceUniverse: string;
  /** Target universe ID */
  targetUniverse: string;
  /** 4x4 transformation matrix */
  transformMatrix: number[][];
  /** Ratios of physical constants between universes */
  constantsRatio: Record<string, number>;
  /** Topology mapping function (represented as parameters) */
  topologyMap: {
    type: string;
    parameters: Record<string, any>;
  };
  /** Mapping quality/accuracy (0.0-1.0) */
  accuracy: number;
}

// ============================================================================
// Temporal GPS System
// ============================================================================

/**
 * Temporal beacon status
 */
export type BeaconStatus = 'active' | 'inactive' | 'maintenance' | 'error';

/**
 * Temporal GPS beacon
 */
export interface TemporalBeacon {
  /** Unique beacon identifier */
  id: string;
  /** Fixed spacetime position */
  position: TemporalCoordinate4D;
  /** Signal broadcast frequency (Hz) */
  signalFrequency: number;
  /** Signal power output (Watts) */
  powerOutput: number;
  /** Reference frame of the beacon */
  referenceFrame: string;
  /** Current operational status */
  status: BeaconStatus;
  /** Metadata */
  metadata?: {
    name?: string;
    installDate?: number;
    lastMaintenance?: number;
    coverage?: number; // Coverage radius in meters
    [key: string]: any;
  };
}

/**
 * TGPS signal from a beacon
 */
export interface TGPSSignal {
  /** Beacon that sent this signal */
  beaconId: string;
  /** Time when signal was transmitted */
  transmitTime: UniversalTimeIndex;
  /** Position of beacon at transmit time */
  beaconPosition: TemporalCoordinate4D;
  /** Signal metadata */
  metadata: {
    /** Received signal strength */
    signalStrength: number;
    /** Signal frequency (Hz) */
    frequency: number;
    /** Modulation type */
    modulation: string;
    /** Signal-to-noise ratio */
    snr?: number;
    [key: string]: any;
  };
}

/**
 * Real-time TGPS position
 */
export interface TemporalGPSPosition {
  /** Current spacetime coordinate */
  coordinate: TemporalCoordinate4D;
  /** Velocity vector (spatial + temporal rate) */
  velocity: Vector4D;
  /** Acceleration vector */
  acceleration: Vector4D;
  /** Position uncertainty */
  uncertainty: CoordinateUncertainty;
  /** IDs of beacons used in calculation */
  beaconsUsed: string[];
  /** Timestamp of this position */
  timestamp: UniversalTimeIndex;
  /** Fix quality indicator (0.0-1.0) */
  quality: number;
}

// ============================================================================
// Time Zones and Eras
// ============================================================================

/**
 * Temporal era definition
 */
export interface TemporalEra {
  /** Unique era identifier */
  id: string;
  /** Human-readable name */
  name: string;
  /** Era start time (UTI) */
  startTime: UniversalTimeIndex;
  /** Era end time (UTI) */
  endTime: UniversalTimeIndex;
  /** Calendar system used in this era */
  calendarSystem: string;
  /** Time zones active in this era */
  timeZones: TimeZone[];
  /** Metadata */
  metadata?: {
    description?: string;
    keyEvents?: string[];
    [key: string]: any;
  };
}

/**
 * Time zone rule (for DST, etc.)
 */
export interface TimeZoneRule {
  /** Rule name */
  name: string;
  /** When rule takes effect */
  effectiveFrom: number;
  /** When rule expires */
  effectiveUntil: number;
  /** Offset change in seconds */
  offsetChange: number;
  /** Description */
  description?: string;
}

/**
 * Time zone definition
 */
export interface TimeZone {
  /** Time zone identifier (e.g., "Asia/Tokyo") */
  id: string;
  /** Human-readable name */
  name: string;
  /** Offset from UTC in seconds */
  offsetUTC: number;
  /** Whether daylight saving time is used */
  daylightSaving: boolean;
  /** When this time zone definition is valid from */
  validFrom: UniversalTimeIndex;
  /** When this time zone definition is valid until */
  validUntil: UniversalTimeIndex;
  /** DST and other rules */
  rules: TimeZoneRule[];
  /** Metadata */
  metadata?: {
    country?: string;
    abbreviation?: string;
    [key: string]: any;
  };
}

// ============================================================================
// Coordinate Transformations
// ============================================================================

/**
 * Transformation matrix (4x4 for spacetime)
 */
export type TransformMatrix4x4 = [
  [number, number, number, number],
  [number, number, number, number],
  [number, number, number, number],
  [number, number, number, number]
];

/**
 * Coordinate transformation parameters
 */
export interface CoordinateTransformation {
  /** Source reference frame */
  sourceFrame: string;
  /** Target reference frame */
  targetFrame: string;
  /** 4x4 transformation matrix */
  matrix: TransformMatrix4x4;
  /** Translation offset */
  offset: Vector4D;
  /** Rotation angles (radians) */
  rotation?: {
    alpha: number; // rotation about x-axis
    beta: number;  // rotation about y-axis
    gamma: number; // rotation about z-axis
  };
  /** Lorentz factor (for relativistic transforms) */
  lorentzFactor?: number;
  /** Velocity (for frame motion) */
  velocity?: Vector3D;
}

// ============================================================================
// Security and Access Control
// ============================================================================

/**
 * Encrypted coordinate
 */
export interface EncryptedCoordinate {
  /** Base64-encoded ciphertext */
  ciphertext: string;
  /** Encryption algorithm */
  algorithm: 'AES-256-GCM' | 'ChaCha20-Poly1305' | string;
  /** Initialization vector / nonce */
  nonce: string;
  /** Authentication tag */
  tag: string;
  /** Key identifier */
  keyId: string;
  /** Timestamp of encryption */
  timestamp?: number;
}

/**
 * Geographic boundary for access control
 */
export interface GeographicBoundary {
  /** Boundary type */
  type: 'rectangle' | 'circle' | 'polygon';
  /** Coordinates defining the boundary */
  coordinates: number[][];
  /** Altitude range (optional) */
  altitudeRange?: {
    min: number;
    max: number;
  };
}

/**
 * Access control for coordinates
 */
export interface CoordinateAccessControl {
  /** Coordinate identifier */
  coordinateId: string;
  /** Owner user ID */
  owner: string;
  /** Permissions */
  permissions: {
    /** Users who can read */
    read: string[];
    /** Users who can write/modify */
    write: string[];
    /** Users who can delete */
    delete: string[];
  };
  /** Temporal access restrictions */
  temporalRestrictions?: {
    minTime: UniversalTimeIndex;
    maxTime: UniversalTimeIndex;
  };
  /** Spatial access restrictions */
  spatialRestrictions?: {
    allowedRegions: GeographicBoundary[];
  };
  /** Metadata */
  metadata?: {
    createdAt?: number;
    modifiedAt?: number;
    [key: string]: any;
  };
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * Coordinate error codes
 */
export enum CoordinateError {
  INVALID_SPATIAL_RANGE = 'COORD_E001',
  INVALID_TEMPORAL_RANGE = 'COORD_E002',
  UNKNOWN_REFERENCE_FRAME = 'COORD_E003',
  TRANSFORMATION_FAILED = 'COORD_E004',
  TIMELINE_NOT_FOUND = 'COORD_E005',
  UNIVERSE_INACCESSIBLE = 'COORD_E006',
  CAUSALITY_VIOLATION = 'COORD_E007',
  PRECISION_LIMIT_EXCEEDED = 'COORD_E008',
  ENCRYPTION_FAILED = 'COORD_E009',
  ACCESS_DENIED = 'COORD_E010',
  INVALID_UTC_CONVERSION = 'COORD_E011',
  BEACON_UNAVAILABLE = 'COORD_E012',
  INSUFFICIENT_BEACONS = 'COORD_E013'
}

/**
 * Coordinate exception
 */
export class CoordinateException extends Error {
  constructor(
    public code: CoordinateError,
    message: string,
    public details?: Record<string, any>
  ) {
    super(`[${code}] ${message}`);
    this.name = 'CoordinateException';
  }
}

// ============================================================================
// Configuration and Constants
// ============================================================================

/**
 * System configuration
 */
export interface TemporalCoordinateSystemConfig {
  /** Default reference frame */
  defaultReferenceFrame: ReferenceFrameId;
  /** Default time scale */
  defaultTimeScale: TimeScale;
  /** Default uncertainty */
  defaultUncertainty: CoordinateUncertainty;
  /** Precision settings */
  precision: {
    spatial: number;  // decimal places
    temporal: number; // decimal places
  };
  /** TGPS settings */
  tgps?: {
    minBeacons: number;
    updateInterval: number; // milliseconds
    timeout: number;        // milliseconds
  };
  /** Security settings */
  security?: {
    encryptionAlgorithm: string;
    accessControlEnabled: boolean;
    auditLog: boolean;
  };
}

/**
 * Physical and mathematical constants
 */
export interface SystemConstants {
  // Physical constants
  SPEED_OF_LIGHT: number;
  GRAVITATIONAL_CONSTANT: number;
  PLANCK_TIME: number;
  PLANCK_LENGTH: number;

  // Earth parameters
  EARTH_RADIUS_EQUATORIAL: number;
  EARTH_RADIUS_POLAR: number;
  EARTH_ECCENTRICITY_SQUARED: number;

  // Time conversions
  UNIX_EPOCH_OFFSET: number;
  PLANCK_TIME_PER_SECOND: number;
  JULIAN_DAY_OFFSET: number;

  // Coordinate limits
  MAX_LONGITUDE: number;
  MIN_LONGITUDE: number;
  MAX_LATITUDE: number;
  MIN_LATITUDE: number;
  MAX_ALTITUDE: number;
  MIN_ALTITUDE: number;
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Temporal distance between two coordinates
 */
export interface TemporalDistance {
  /** Spatial distance in meters */
  spatial: number;
  /** Temporal distance in seconds */
  temporal: number;
  /** Spacetime interval (relativistic) */
  interval: number;
  /** Distance type */
  type: 'timelike' | 'spacelike' | 'lightlike';
}

/**
 * Coordinate validation result
 */
export interface CoordinateValidation {
  /** Whether coordinate is valid */
  valid: boolean;
  /** Error messages (if invalid) */
  errors: string[];
  /** Warnings (non-critical issues) */
  warnings?: string[];
}

/**
 * Batch coordinate operation result
 */
export interface BatchOperationResult<T> {
  /** Successfully processed items */
  successful: T[];
  /** Failed items with error messages */
  failed: Array<{
    item: T;
    error: string;
    code?: CoordinateError;
  }>;
  /** Processing statistics */
  stats: {
    total: number;
    successful: number;
    failed: number;
    duration: number; // milliseconds
  };
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  Vector3D,
  Vector4D,
  CoordinateUncertainty,
  TemporalCoordinate4D,
  ReferenceFrameId,
  TimeScale,
  PhysicalConstants,
  TemporalReferenceFrame,
  UniversalTimeIndex,
  TimelineState,
  TimelineAddress,
  BranchPoint,
  UniverseType,
  UniverseTopology,
  ParallelUniverseId,
  UniverseMapping,
  BeaconStatus,
  TemporalBeacon,
  TGPSSignal,
  TemporalGPSPosition,
  TemporalEra,
  TimeZoneRule,
  TimeZone,
  TransformMatrix4x4,
  CoordinateTransformation,
  EncryptedCoordinate,
  GeographicBoundary,
  CoordinateAccessControl,
  TemporalCoordinateSystemConfig,
  SystemConstants,
  TemporalDistance,
  CoordinateValidation,
  BatchOperationResult
};

export {
  CoordinateError,
  CoordinateException
};
