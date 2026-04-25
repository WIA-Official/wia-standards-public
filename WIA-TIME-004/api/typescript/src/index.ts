/**
 * WIA-TIME-004: Temporal Coordinate System - SDK Implementation
 *
 * @version 1.0.0
 * @license MIT
 * @organization WIA - World Certification Industry Association
 * @philosophy 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

import {
  TemporalCoordinate4D,
  CoordinateUncertainty,
  UniversalTimeIndex,
  TemporalReferenceFrame,
  TimelineAddress,
  ParallelUniverseId,
  TemporalGPSPosition,
  TGPSSignal,
  CoordinateTransformation,
  TemporalDistance,
  CoordinateValidation,
  CoordinateError,
  CoordinateException,
  Vector3D,
  Vector4D,
  ReferenceFrameId,
  TransformMatrix4x4,
  SystemConstants,
  TemporalCoordinateSystemConfig
} from './types';

// ============================================================================
// Constants
// ============================================================================

export const CONSTANTS: SystemConstants = {
  // Physical constants
  SPEED_OF_LIGHT: 299792458,              // m/s
  GRAVITATIONAL_CONSTANT: 6.67430e-11,    // m³/kg/s²
  PLANCK_TIME: 5.391247e-44,              // seconds
  PLANCK_LENGTH: 1.616255e-35,            // meters

  // Earth parameters
  EARTH_RADIUS_EQUATORIAL: 6378137,       // meters (WGS84)
  EARTH_RADIUS_POLAR: 6356752.314245,     // meters (WGS84)
  EARTH_ECCENTRICITY_SQUARED: 0.00669437999014,

  // Time conversions
  UNIX_EPOCH_OFFSET: 4.3544832e17,        // seconds (Unix epoch to Big Bang)
  PLANCK_TIME_PER_SECOND: 1.855e43,
  JULIAN_DAY_OFFSET: 2440587.5,           // Unix epoch to Julian Day

  // Coordinate limits
  MAX_LONGITUDE: 180,
  MIN_LONGITUDE: -180,
  MAX_LATITUDE: 90,
  MIN_LATITUDE: -90,
  MAX_ALTITUDE: 1e9,                      // meters
  MIN_ALTITUDE: -1e9,                     // meters
};

// Default configuration
const DEFAULT_CONFIG: TemporalCoordinateSystemConfig = {
  defaultReferenceFrame: 'EARTH_J2000',
  defaultTimeScale: 'UTC',
  defaultUncertainty: {
    spatial: 0.001,
    temporal: 0.000001,
    confidence: 0.95
  },
  precision: {
    spatial: 6,
    temporal: 6
  },
  tgps: {
    minBeacons: 4,
    updateInterval: 100,
    timeout: 5000
  },
  security: {
    encryptionAlgorithm: 'AES-256-GCM',
    accessControlEnabled: false,
    auditLog: false
  }
};

let config: TemporalCoordinateSystemConfig = { ...DEFAULT_CONFIG };

// ============================================================================
// Configuration
// ============================================================================

/**
 * Initialize the Temporal Coordinate System with custom configuration
 */
export function initialize(customConfig?: Partial<TemporalCoordinateSystemConfig>): void {
  config = { ...DEFAULT_CONFIG, ...customConfig };
}

/**
 * Get current configuration
 */
export function getConfig(): TemporalCoordinateSystemConfig {
  return { ...config };
}

// ============================================================================
// Core Coordinate Functions
// ============================================================================

/**
 * Create a 4D spacetime coordinate
 *
 * @param params - Coordinate parameters
 * @returns Complete temporal coordinate
 *
 * @example
 * ```typescript
 * const coord = createTemporalCoordinate({
 *   x: 139.6917,  // Tokyo longitude
 *   y: 35.6895,   // Tokyo latitude
 *   z: 40,        // 40m altitude
 *   t: 1735084800 // Unix timestamp
 * });
 * ```
 */
export function createTemporalCoordinate(params: {
  x: number;
  y: number;
  z: number;
  t: number;
  referenceFrame?: ReferenceFrameId;
  uncertainty?: CoordinateUncertainty;
  metadata?: Record<string, any>;
}): TemporalCoordinate4D {
  const coord: TemporalCoordinate4D = {
    x: params.x,
    y: params.y,
    z: params.z,
    t: params.t,
    referenceFrame: params.referenceFrame || config.defaultReferenceFrame,
    uncertainty: params.uncertainty || config.defaultUncertainty,
    metadata: params.metadata
  };

  // Validate the coordinate
  const validation = validateCoordinate(coord);
  if (!validation.valid) {
    throw new CoordinateException(
      CoordinateError.INVALID_SPATIAL_RANGE,
      `Invalid coordinate: ${validation.errors.join(', ')}`,
      { coordinate: coord, validation }
    );
  }

  return coord;
}

/**
 * Transform coordinates between reference frames
 *
 * @param coordinate - Source coordinate
 * @param sourceFrame - Source reference frame ID
 * @param targetFrame - Target reference frame ID
 * @returns Transformed coordinate
 *
 * @example
 * ```typescript
 * const earthCoord = createTemporalCoordinate({ x: 0, y: 0, z: 0, t: 0 });
 * const galacticCoord = transformCoordinates(
 *   earthCoord,
 *   'EARTH_J2000',
 *   'GALACTIC_CENTER'
 * );
 * ```
 */
export function transformCoordinates(
  coordinate: TemporalCoordinate4D,
  sourceFrame: ReferenceFrameId,
  targetFrame: ReferenceFrameId
): TemporalCoordinate4D {
  // If same frame, return copy
  if (sourceFrame === targetFrame) {
    return { ...coordinate, referenceFrame: targetFrame };
  }

  // Get transformation parameters
  const transformation = getTransformationMatrix(sourceFrame, targetFrame);

  // Apply transformation
  const transformed = applyTransformation(coordinate, transformation);

  return {
    ...transformed,
    referenceFrame: targetFrame
  };
}

/**
 * Resolve a timeline branch address to full timeline information
 *
 * @param params - Timeline address parameters
 * @returns Complete timeline address
 *
 * @example
 * ```typescript
 * const timeline = resolveTimelineAddress({
 *   universeId: 'U-P-001',
 *   branchId: 'B-042',
 *   divergencePoint: 1609459200
 * });
 * ```
 */
export function resolveTimelineAddress(params: {
  universeId: string;
  branchId: string;
  divergencePoint: number;
  parentBranch?: string | null;
  depth?: number;
  probability?: number;
  state?: 'stable' | 'collapsing' | 'merging' | 'quantum' | 'hypothetical';
}): TimelineAddress {
  const address: TimelineAddress = {
    universeId: params.universeId,
    branchId: params.branchId,
    divergencePoint: params.divergencePoint,
    parentBranch: params.parentBranch || null,
    depth: params.depth || 0,
    probability: params.probability !== undefined ? params.probability : 1.0,
    state: params.state || 'stable'
  };

  // Validate timeline address
  if (address.probability < 0 || address.probability > 1) {
    throw new CoordinateException(
      CoordinateError.TIMELINE_NOT_FOUND,
      'Timeline probability must be between 0 and 1',
      { address }
    );
  }

  return address;
}

/**
 * Get Universal Time Index for a given coordinate
 *
 * @param coordinate - Temporal coordinate
 * @returns Universal Time Index
 *
 * @example
 * ```typescript
 * const coord = createTemporalCoordinate({ x: 0, y: 0, z: 0, t: 1735084800 });
 * const uti = getUniversalTimeIndex(coord);
 * ```
 */
export function getUniversalTimeIndex(
  coordinate: TemporalCoordinate4D
): UniversalTimeIndex {
  // Convert Unix time to UTI (simplified calculation)
  // In real implementation, this would account for relativistic effects
  const secondsSinceBigBang = coordinate.t + CONSTANTS.UNIX_EPOCH_OFFSET;
  const planckTimeUnits = BigInt(
    Math.floor(secondsSinceBigBang * CONSTANTS.PLANCK_TIME_PER_SECOND)
  );

  return {
    value: planckTimeUnits,
    epoch: 0, // Big Bang
    uncertainty: coordinate.uncertainty.temporal * CONSTANTS.PLANCK_TIME_PER_SECOND,
    referenceFrame: coordinate.referenceFrame
  };
}

/**
 * Locate a coordinate in spacetime with full context
 *
 * @param params - Location parameters
 * @returns Complete spacetime location
 *
 * @example
 * ```typescript
 * const location = locateInSpacetime({
 *   universeId: 'U-P-001',
 *   coordinate: coord,
 *   referenceFrame: 'EARTH_J2000'
 * });
 * ```
 */
export function locateInSpacetime(params: {
  universeId: string;
  coordinate: TemporalCoordinate4D;
  referenceFrame?: ReferenceFrameId;
  timeline?: TimelineAddress;
}): {
  universe: string;
  coordinate: TemporalCoordinate4D;
  timeline: TimelineAddress;
  uti: UniversalTimeIndex;
} {
  const frame = params.referenceFrame || params.coordinate.referenceFrame;
  let coordinate = params.coordinate;

  // Transform to specified frame if needed
  if (coordinate.referenceFrame !== frame) {
    coordinate = transformCoordinates(coordinate, coordinate.referenceFrame, frame);
  }

  // Default timeline (prime timeline)
  const timeline = params.timeline || resolveTimelineAddress({
    universeId: params.universeId,
    branchId: 'B-000',
    divergencePoint: 0
  });

  const uti = getUniversalTimeIndex(coordinate);

  return {
    universe: params.universeId,
    coordinate,
    timeline,
    uti
  };
}

/**
 * Calculate temporal distance between two coordinates
 *
 * @param coord1 - First coordinate
 * @param coord2 - Second coordinate
 * @returns Temporal distance information
 *
 * @example
 * ```typescript
 * const distance = calculateTemporalDistance(coord1, coord2);
 * console.log(`Spatial: ${distance.spatial}m, Temporal: ${distance.temporal}s`);
 * ```
 */
export function calculateTemporalDistance(
  coord1: TemporalCoordinate4D,
  coord2: TemporalCoordinate4D
): TemporalDistance {
  // Transform to same reference frame if needed
  let c2 = coord2;
  if (coord1.referenceFrame !== coord2.referenceFrame) {
    c2 = transformCoordinates(coord2, coord2.referenceFrame, coord1.referenceFrame);
  }

  // Calculate spatial distance (Euclidean)
  const dx = c2.x - coord1.x;
  const dy = c2.y - coord1.y;
  const dz = c2.z - coord1.z;
  const spatialDistance = Math.sqrt(dx * dx + dy * dy + dz * dz);

  // Calculate temporal distance
  const temporalDistance = Math.abs(c2.t - coord1.t);

  // Calculate spacetime interval (Minkowski metric)
  const c = CONSTANTS.SPEED_OF_LIGHT;
  const interval = -c * c * temporalDistance * temporalDistance + spatialDistance * spatialDistance;

  // Determine interval type
  let type: 'timelike' | 'spacelike' | 'lightlike';
  if (Math.abs(interval) < 1e-6) {
    type = 'lightlike';
  } else if (interval < 0) {
    type = 'timelike';
  } else {
    type = 'spacelike';
  }

  return {
    spatial: spatialDistance,
    temporal: temporalDistance,
    interval: Math.sqrt(Math.abs(interval)),
    type
  };
}

// ============================================================================
// Coordinate Validation
// ============================================================================

/**
 * Validate a temporal coordinate
 *
 * @param coordinate - Coordinate to validate
 * @returns Validation result
 */
export function validateCoordinate(
  coordinate: TemporalCoordinate4D
): CoordinateValidation {
  const errors: string[] = [];
  const warnings: string[] = [];

  // Validate spatial coordinates (for Earth-based frames)
  if (coordinate.referenceFrame.startsWith('EARTH')) {
    if (coordinate.x < CONSTANTS.MIN_LONGITUDE || coordinate.x > CONSTANTS.MAX_LONGITUDE) {
      errors.push(`Longitude out of range: ${coordinate.x} (must be -180 to 180)`);
    }
    if (coordinate.y < CONSTANTS.MIN_LATITUDE || coordinate.y > CONSTANTS.MAX_LATITUDE) {
      errors.push(`Latitude out of range: ${coordinate.y} (must be -90 to 90)`);
    }
    if (coordinate.z < CONSTANTS.MIN_ALTITUDE || coordinate.z > CONSTANTS.MAX_ALTITUDE) {
      warnings.push(`Altitude extreme: ${coordinate.z}m`);
    }
  }

  // Validate temporal coordinate
  const BIG_BANG = -CONSTANTS.UNIX_EPOCH_OFFSET;
  const FAR_FUTURE = 1e15;
  if (coordinate.t < BIG_BANG) {
    errors.push(`Time before Big Bang: ${coordinate.t}`);
  }
  if (coordinate.t > FAR_FUTURE) {
    warnings.push(`Time far in future: ${coordinate.t}`);
  }

  // Validate uncertainty
  if (coordinate.uncertainty) {
    if (coordinate.uncertainty.spatial < 0) {
      errors.push('Spatial uncertainty cannot be negative');
    }
    if (coordinate.uncertainty.temporal < 0) {
      errors.push('Temporal uncertainty cannot be negative');
    }
    if (coordinate.uncertainty.confidence < 0 || coordinate.uncertainty.confidence > 1) {
      errors.push('Confidence must be between 0 and 1');
    }
  }

  return {
    valid: errors.length === 0,
    errors,
    warnings: warnings.length > 0 ? warnings : undefined
  };
}

// ============================================================================
// Transformation Functions
// ============================================================================

/**
 * Get transformation matrix between two reference frames
 */
function getTransformationMatrix(
  sourceFrame: ReferenceFrameId,
  targetFrame: ReferenceFrameId
): CoordinateTransformation {
  // Predefined transformations
  const transformations: Record<string, Record<string, CoordinateTransformation>> = {
    'EARTH_J2000': {
      'EARTH_GREENWICH': {
        sourceFrame: 'EARTH_J2000',
        targetFrame: 'EARTH_GREENWICH',
        matrix: identityMatrix4x4(),
        offset: { x: 0, y: 0, z: 0, t: -3155760000 } // J2000 to 1884 offset
      },
      'GALACTIC_CENTER': {
        sourceFrame: 'EARTH_J2000',
        targetFrame: 'GALACTIC_CENTER',
        matrix: createRotationMatrix(0, 0, Math.PI / 4), // Simplified
        offset: { x: 8000, y: 0, z: 0, t: 0 } // ~8 kpc to galactic center
      }
    }
  };

  // Check for predefined transformation
  if (transformations[sourceFrame]?.[targetFrame]) {
    return transformations[sourceFrame][targetFrame];
  }

  // Default: identity transformation with warning
  console.warn(`No predefined transformation from ${sourceFrame} to ${targetFrame}, using identity`);
  return {
    sourceFrame,
    targetFrame,
    matrix: identityMatrix4x4(),
    offset: { x: 0, y: 0, z: 0, t: 0 }
  };
}

/**
 * Apply transformation to a coordinate
 */
function applyTransformation(
  coordinate: TemporalCoordinate4D,
  transformation: CoordinateTransformation
): TemporalCoordinate4D {
  // Apply matrix transformation
  const vec = [coordinate.x, coordinate.y, coordinate.z, coordinate.t];
  const result = multiplyMatrix4x4(transformation.matrix, vec);

  // Apply offset
  return {
    x: result[0] + transformation.offset.x,
    y: result[1] + transformation.offset.y,
    z: result[2] + transformation.offset.z,
    t: result[3] + transformation.offset.t,
    referenceFrame: transformation.targetFrame,
    uncertainty: coordinate.uncertainty,
    metadata: coordinate.metadata
  };
}

// ============================================================================
// Matrix Operations
// ============================================================================

/**
 * Create 4x4 identity matrix
 */
function identityMatrix4x4(): TransformMatrix4x4 {
  return [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
  ];
}

/**
 * Create rotation matrix for 3D space (embedded in 4x4)
 */
function createRotationMatrix(alpha: number, beta: number, gamma: number): TransformMatrix4x4 {
  const ca = Math.cos(alpha);
  const sa = Math.sin(alpha);
  const cb = Math.cos(beta);
  const sb = Math.sin(beta);
  const cg = Math.cos(gamma);
  const sg = Math.sin(gamma);

  // Combined rotation matrix Rz(γ) × Ry(β) × Rx(α)
  return [
    [cb * cg, -cb * sg, sb, 0],
    [ca * sg + sa * sb * cg, ca * cg - sa * sb * sg, -sa * cb, 0],
    [sa * sg - ca * sb * cg, sa * cg + ca * sb * sg, ca * cb, 0],
    [0, 0, 0, 1]
  ];
}

/**
 * Multiply 4x4 matrix by 4D vector
 */
function multiplyMatrix4x4(matrix: TransformMatrix4x4, vector: number[]): number[] {
  return [
    matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2] + matrix[0][3] * vector[3],
    matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2] + matrix[1][3] * vector[3],
    matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2] + matrix[2][3] * vector[3],
    matrix[3][0] * vector[0] + matrix[3][1] * vector[1] + matrix[3][2] * vector[2] + matrix[3][3] * vector[3]
  ];
}

// ============================================================================
// Geodetic Conversions
// ============================================================================

/**
 * Convert geodetic coordinates (lon, lat, alt) to Cartesian (x, y, z)
 *
 * @param lon - Longitude in degrees
 * @param lat - Latitude in degrees
 * @param alt - Altitude in meters
 * @returns Cartesian coordinates
 */
export function geodeticToCartesian(
  lon: number,
  lat: number,
  alt: number
): Vector3D {
  const a = CONSTANTS.EARTH_RADIUS_EQUATORIAL;
  const e2 = CONSTANTS.EARTH_ECCENTRICITY_SQUARED;

  const phi = lat * Math.PI / 180;
  const lambda = lon * Math.PI / 180;

  const N = a / Math.sqrt(1 - e2 * Math.sin(phi) * Math.sin(phi));

  return {
    x: (N + alt) * Math.cos(phi) * Math.cos(lambda),
    y: (N + alt) * Math.cos(phi) * Math.sin(lambda),
    z: (N * (1 - e2) + alt) * Math.sin(phi)
  };
}

/**
 * Convert Cartesian coordinates (x, y, z) to geodetic (lon, lat, alt)
 *
 * @param x - X coordinate in meters
 * @param y - Y coordinate in meters
 * @param z - Z coordinate in meters
 * @returns Geodetic coordinates { lon, lat, alt }
 */
export function cartesianToGeodetic(
  x: number,
  y: number,
  z: number
): { lon: number; lat: number; alt: number } {
  const a = CONSTANTS.EARTH_RADIUS_EQUATORIAL;
  const b = CONSTANTS.EARTH_RADIUS_POLAR;
  const e2 = CONSTANTS.EARTH_ECCENTRICITY_SQUARED;
  const ep2 = (a * a - b * b) / (b * b);

  const p = Math.sqrt(x * x + y * y);
  const theta = Math.atan2(z * a, p * b);

  const lon = Math.atan2(y, x);
  const lat = Math.atan2(
    z + ep2 * b * Math.sin(theta) ** 3,
    p - e2 * a * Math.cos(theta) ** 3
  );

  const N = a / Math.sqrt(1 - e2 * Math.sin(lat) * Math.sin(lat));
  const alt = p / Math.cos(lat) - N;

  return {
    lon: lon * 180 / Math.PI,
    lat: lat * 180 / Math.PI,
    alt
  };
}

// ============================================================================
// Time Conversion Utilities
// ============================================================================

/**
 * Convert Unix timestamp to Universal Time Index
 */
export function unixToUTI(unixTime: number): UniversalTimeIndex {
  const secondsSinceBigBang = unixTime + CONSTANTS.UNIX_EPOCH_OFFSET;
  const planckTimeUnits = BigInt(
    Math.floor(secondsSinceBigBang * CONSTANTS.PLANCK_TIME_PER_SECOND)
  );

  return {
    value: planckTimeUnits,
    epoch: 0,
    uncertainty: 0.000001 * CONSTANTS.PLANCK_TIME_PER_SECOND,
    referenceFrame: 'EARTH_J2000'
  };
}

/**
 * Convert Universal Time Index to Unix timestamp
 */
export function utiToUnix(uti: UniversalTimeIndex): number {
  const secondsSinceBigBang = Number(uti.value) / CONSTANTS.PLANCK_TIME_PER_SECOND;
  return secondsSinceBigBang - CONSTANTS.UNIX_EPOCH_OFFSET;
}

/**
 * Get current temporal coordinate (present moment)
 */
export function getCurrentCoordinate(params?: {
  x?: number;
  y?: number;
  z?: number;
  referenceFrame?: ReferenceFrameId;
}): TemporalCoordinate4D {
  return createTemporalCoordinate({
    x: params?.x || 0,
    y: params?.y || 0,
    z: params?.z || 0,
    t: Date.now() / 1000, // Current Unix time
    referenceFrame: params?.referenceFrame
  });
}

// ============================================================================
// Temporal GPS Functions
// ============================================================================

/**
 * Calculate position from TGPS signals
 *
 * @param signals - Array of TGPS signals from beacons
 * @returns Calculated position
 */
export function calculateTGPSPosition(signals: TGPSSignal[]): TemporalGPSPosition {
  if (signals.length < (config.tgps?.minBeacons || 4)) {
    throw new CoordinateException(
      CoordinateError.INSUFFICIENT_BEACONS,
      `Need at least ${config.tgps?.minBeacons || 4} beacons, got ${signals.length}`,
      { signalCount: signals.length }
    );
  }

  // Simplified position calculation (trilateration)
  // In real implementation, this would solve the system of equations
  let sumX = 0, sumY = 0, sumZ = 0, sumT = 0;
  for (const signal of signals) {
    sumX += signal.beaconPosition.x;
    sumY += signal.beaconPosition.y;
    sumZ += signal.beaconPosition.z;
    sumT += signal.beaconPosition.t;
  }

  const n = signals.length;
  const coordinate = createTemporalCoordinate({
    x: sumX / n,
    y: sumY / n,
    z: sumZ / n,
    t: sumT / n
  });

  return {
    coordinate,
    velocity: { x: 0, y: 0, z: 0, t: 1 },
    acceleration: { x: 0, y: 0, z: 0, t: 0 },
    uncertainty: config.defaultUncertainty,
    beaconsUsed: signals.map(s => s.beaconId),
    timestamp: getUniversalTimeIndex(coordinate),
    quality: Math.min(1.0, signals.length / 8) // Quality based on number of beacons
  };
}

// ============================================================================
// Export All Functions
// ============================================================================

export {
  // Configuration
  initialize,
  getConfig,

  // Core functions
  createTemporalCoordinate,
  transformCoordinates,
  resolveTimelineAddress,
  getUniversalTimeIndex,
  locateInSpacetime,
  calculateTemporalDistance,

  // Validation
  validateCoordinate,

  // Geodetic conversions
  geodeticToCartesian,
  cartesianToGeodetic,

  // Time conversions
  unixToUTI,
  utiToUnix,
  getCurrentCoordinate,

  // TGPS
  calculateTGPSPosition,

  // Constants
  CONSTANTS
};

// Re-export types
export * from './types';

// ============================================================================
// Philosophy
// ============================================================================

/**
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 *
 * The Temporal Coordinate System enables humanity to navigate spacetime
 * with precision, opening possibilities for time travel, historical research,
 * and cross-universe collaboration.
 */
