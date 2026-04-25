/**
 * WIA-DEF-012: Space Surveillance - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense & Space Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Vector Types
// ============================================================================

/**
 * Three-dimensional vector for spatial coordinates
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * 3x3 Covariance matrix for position uncertainty
 */
export type CovarianceMatrix = [
  [number, number, number],
  [number, number, number],
  [number, number, number]
];

// ============================================================================
// Orbital Elements
// ============================================================================

/**
 * Keplerian orbital elements (Classical Orbital Elements)
 */
export interface KeplerianElements {
  /** Semi-major axis in meters */
  semiMajorAxis: number;

  /** Eccentricity (0-1, dimensionless) */
  eccentricity: number;

  /** Inclination in degrees (0-180) */
  inclination: number;

  /** Right Ascension of Ascending Node in degrees (0-360) */
  raan: number;

  /** Argument of Periapsis in degrees (0-360) */
  argumentOfPerigee: number;

  /** Mean Anomaly in degrees (0-360) */
  meanAnomaly: number;

  /** Epoch time for elements */
  epoch: Date;
}

/**
 * Two-Line Element (TLE) format
 */
export interface TLE {
  /** Line 1 of TLE (69 characters) */
  line1: string;

  /** Line 2 of TLE (69 characters) */
  line2: string;

  /** Epoch extracted from TLE */
  epoch: Date;

  /** NORAD catalog number */
  noradId: number;

  /** International designator */
  intlDesignator?: string;
}

/**
 * State vector (position and velocity)
 */
export interface StateVector {
  /** Position vector in meters (ECI frame) */
  position: Vector3;

  /** Velocity vector in m/s (ECI frame) */
  velocity: Vector3;

  /** Epoch time for state */
  epoch: Date;

  /** Reference frame */
  frame?: 'ECI' | 'ECEF' | 'RTN';
}

// ============================================================================
// Space Objects
// ============================================================================

/**
 * Space object catalog entry
 */
export interface SpaceObject {
  /** Unique object identifier (e.g., "NORAD-25544") */
  objectId: string;

  /** International designator (e.g., "1998-067A") */
  intlDesignator?: string;

  /** Object name */
  name?: string;

  /** Country or organization */
  country?: string;

  /** Launch date */
  launchDate?: Date;

  /** Launch site */
  launchSite?: string;

  /** Decay/re-entry date (if applicable) */
  decayDate?: Date | null;

  /** Orbital period in minutes */
  period?: number;

  /** Inclination in degrees */
  inclination?: number;

  /** Apogee altitude in km */
  apogee?: number;

  /** Perigee altitude in km */
  perigee?: number;

  /** Radar Cross-Section category */
  rcs?: 'SMALL' | 'MEDIUM' | 'LARGE';

  /** Object type */
  objectType: ObjectType;

  /** Operational status */
  status: ObjectStatus;

  /** Two-Line Element set */
  tle?: TLE;

  /** Keplerian orbital elements */
  orbitalElements?: KeplerianElements;

  /** Physical characteristics */
  physical?: PhysicalCharacteristics;
}

/**
 * Object type classification
 */
export type ObjectType =
  | 'PAYLOAD'
  | 'ROCKET_BODY'
  | 'DEBRIS'
  | 'UNKNOWN';

/**
 * Object operational status
 */
export type ObjectStatus =
  | 'OPERATIONAL'
  | 'NONOPERATIONAL'
  | 'DECAYED'
  | 'FRAGMENTED'
  | 'UNKNOWN';

/**
 * Physical characteristics of space object
 */
export interface PhysicalCharacteristics {
  /** Mass in kilograms */
  mass?: number;

  /** Approximate size in meters */
  size?: number;

  /** Cross-sectional area in m² */
  crossSectionalArea?: number;

  /** Drag coefficient */
  dragCoefficient?: number;

  /** Reflectivity coefficient (0-2) */
  reflectivityCoefficient?: number;

  /** Shape description */
  shape?: string;
}

// ============================================================================
// Tracking and Observations
// ============================================================================

/**
 * Tracking request parameters
 */
export interface TrackingRequest {
  /** Object identifier */
  objectId: string;

  /** Sensor network used */
  sensorNetwork: string;

  /** Observation time */
  observationTime: Date;

  /** Observed position (ECI frame) */
  position: Vector3;

  /** Observed velocity (ECI frame) */
  velocity: Vector3;

  /** Position/velocity uncertainty */
  uncertainty?: CovarianceMatrix;

  /** Sensor ID */
  sensorId?: string;

  /** Observation quality (0-1) */
  quality?: number;
}

/**
 * Tracking result
 */
export interface TrackingResult {
  /** Object identifier */
  objectId: string;

  /** Updated orbital elements */
  orbitalElements: KeplerianElements;

  /** Altitude in kilometers */
  altitude: number;

  /** Orbital period in minutes */
  period: number;

  /** Inclination in degrees */
  inclination: number;

  /** Orbit type */
  orbitType: 'LEO' | 'MEO' | 'GEO' | 'HEO' | 'UNKNOWN';

  /** Next scheduled update time */
  nextUpdate: Date;

  /** Position covariance matrix */
  covariance?: CovarianceMatrix;

  /** Tracking quality score (0-1) */
  trackingQuality: number;
}

/**
 * Sensor observation data
 */
export interface SensorObservation {
  /** Sensor identifier */
  sensorId: string;

  /** Observation time */
  observationTime: Date;

  /** Sensor type */
  sensorType: 'RADAR' | 'OPTICAL' | 'LASER' | 'SPACE_BASED';

  /** Observation data (sensor-dependent) */
  data: RadarObservation | OpticalObservation | LaserObservation;

  /** Measurement uncertainty */
  uncertainty: number;

  /** Signal-to-noise ratio */
  snr?: number;
}

/**
 * Radar observation data
 */
export interface RadarObservation {
  /** Range in meters */
  range: number;

  /** Range rate in m/s */
  rangeRate: number;

  /** Azimuth in degrees */
  azimuth: number;

  /** Elevation in degrees */
  elevation: number;

  /** Radar Cross-Section in m² */
  rcs?: number;
}

/**
 * Optical observation data
 */
export interface OpticalObservation {
  /** Right Ascension in degrees */
  rightAscension: number;

  /** Declination in degrees */
  declination: number;

  /** Visual magnitude */
  magnitude?: number;

  /** Angular rate in arcsec/s */
  angularRate?: number;
}

/**
 * Laser ranging observation data
 */
export interface LaserObservation {
  /** Range in meters */
  range: number;

  /** Range accuracy in millimeters */
  accuracy: number;

  /** Number of photons returned */
  returnCount?: number;
}

// ============================================================================
// Conjunction Assessment
// ============================================================================

/**
 * Conjunction request parameters
 */
export interface ConjunctionRequest {
  /** Primary object ID */
  primary: string;

  /** Secondary object ID */
  secondary: string;

  /** Time window for screening in seconds */
  timeWindow: number;

  /** Minimum distance threshold in meters */
  minDistance: number;

  /** Start time for screening */
  startTime?: Date;

  /** Include covariance in analysis */
  includeCovariance?: boolean;
}

/**
 * Conjunction event result
 */
export interface ConjunctionResult {
  /** Unique conjunction identifier */
  id: string;

  /** Primary object ID */
  primary: string;

  /** Secondary object ID */
  secondary: string;

  /** Time of closest approach */
  timeOfClosestApproach: Date;

  /** Miss distance in meters */
  missDistance: number;

  /** Relative velocity in m/s */
  relativeVelocity: number;

  /** Radial separation in meters */
  radialDistance: number;

  /** Along-track separation in meters */
  alongTrackDistance: number;

  /** Cross-track separation in meters */
  crossTrackDistance: number;

  /** Relative position vector at TCA */
  relativePosition: Vector3;

  /** Relative velocity vector at TCA */
  relativeVelocityVector: Vector3;

  /** Screening status */
  status: 'ACTIVE' | 'CLEARED' | 'IGNORED';
}

// ============================================================================
// Collision Risk
// ============================================================================

/**
 * Risk assessment request
 */
export interface RiskAssessmentRequest {
  /** Conjunction event ID */
  conjunctionId: string;

  /** Primary object size in meters */
  primarySize: number;

  /** Secondary object size in meters */
  secondarySize: number;

  /** Position uncertainty in meters (1-sigma) */
  positionUncertainty: number;

  /** Primary object covariance */
  primaryCovariance?: CovarianceMatrix;

  /** Secondary object covariance */
  secondaryCovariance?: CovarianceMatrix;

  /** Calculation method */
  method?: 'FOSTER' | 'CHAN' | 'MONTE_CARLO';
}

/**
 * Risk assessment result
 */
export interface RiskAssessmentResult {
  /** Collision probability (0-1) */
  probability: number;

  /** Risk level classification */
  riskLevel: RiskLevel;

  /** Recommended action */
  recommendedAction: string;

  /** Is maneuver required? */
  maneuverRequired: boolean;

  /** Estimated delta-V for avoidance in m/s */
  estimatedDeltaV?: number;

  /** Combined hard-body radius in meters */
  combinedRadius: number;

  /** Mahalanobis distance */
  mahalanobisDistance?: number;

  /** 2D collision probability (in-plane) */
  probability2D?: number;
}

/**
 * Risk level categories
 */
export type RiskLevel =
  | 'MINIMAL'    // Pc < 1e-5
  | 'LOW'        // 1e-5 < Pc < 1e-4
  | 'MEDIUM'     // 1e-4 < Pc < 1e-3
  | 'HIGH'       // 1e-3 < Pc < 1e-2
  | 'CRITICAL';  // Pc > 1e-2

/**
 * Maneuver plan for collision avoidance
 */
export interface ManeuverPlan {
  /** Maneuver identifier */
  id: string;

  /** Object to maneuver */
  objectId: string;

  /** Maneuver execution time */
  executionTime: Date;

  /** Delta-V vector in m/s (RTN frame) */
  deltaV: Vector3;

  /** Total delta-V magnitude in m/s */
  deltaVMagnitude: number;

  /** Maneuver direction */
  direction: 'RADIAL' | 'TANGENTIAL' | 'NORMAL' | 'COMBINED';

  /** Expected Pc after maneuver */
  postManeuverPc: number;

  /** Fuel consumption in kg */
  fuelConsumption?: number;

  /** Mission impact assessment */
  missionImpact?: string;
}

// ============================================================================
// Space Catalog
// ============================================================================

/**
 * Space catalog search parameters
 */
export interface CatalogSearchParams {
  /** Object type filter */
  objectType?: ObjectType[];

  /** Status filter */
  status?: ObjectStatus[];

  /** Orbital regime filter */
  orbitType?: ('LEO' | 'MEO' | 'GEO' | 'HEO')[];

  /** Minimum altitude in km */
  minAltitude?: number;

  /** Maximum altitude in km */
  maxAltitude?: number;

  /** Country filter */
  country?: string[];

  /** Search by name */
  name?: string;

  /** Maximum results */
  limit?: number;
}

/**
 * Catalog update request
 */
export interface CatalogUpdateRequest {
  /** Data source */
  source: string;

  /** Objects to update */
  objects: SpaceObject[];

  /** Update timestamp */
  timestamp: Date;

  /** Update type */
  updateType: 'NEW' | 'MODIFIED' | 'DELETED';
}

// ============================================================================
// Debris Analysis
// ============================================================================

/**
 * Debris analysis request
 */
export interface DebrisAnalysisRequest {
  /** Orbital regime */
  orbit: 'LEO' | 'MEO' | 'GEO';

  /** Size range in cm */
  sizeRange: string;

  /** Time period for analysis */
  timePeriod?: {
    start: Date;
    end: Date;
  };

  /** Include statistical projection */
  includeProjection?: boolean;
}

/**
 * Debris analysis result
 */
export interface DebrisAnalysisResult {
  /** Total debris count */
  totalCount: number;

  /** Debris by size category */
  bySize: {
    large: number;    // >10 cm
    medium: number;   // 1-10 cm
    small: number;    // <1 cm
  };

  /** Spatial distribution */
  distribution: {
    altitude: number;  // km
    count: number;
  }[];

  /** Growth rate per year */
  growthRate?: number;

  /** Collision risk index */
  collisionRiskIndex: number;

  /** Future projection (if requested) */
  projection?: {
    year: number;
    estimatedCount: number;
  }[];
}

// ============================================================================
// Orbital Decay
// ============================================================================

/**
 * Decay prediction request
 */
export interface DecayPredictionRequest {
  /** Object identifier */
  objectId: string;

  /** Prediction horizon in days */
  horizon: number;

  /** Atmospheric model */
  atmosphericModel?: 'NRLMSISE00' | 'JB2008' | 'DTM2000';

  /** Solar activity forecast */
  solarActivity?: 'LOW' | 'MEDIUM' | 'HIGH';
}

/**
 * Decay prediction result
 */
export interface DecayPredictionResult {
  /** Object identifier */
  objectId: string;

  /** Predicted decay date */
  decayDate: Date | null;

  /** Uncertainty in days */
  uncertainty: number;

  /** Current lifetime estimate in days */
  lifetimeEstimate: number;

  /** Altitude decay rate in km/day */
  decayRate: number;

  /** Re-entry location uncertainty */
  reentryUncertainty?: {
    latitude: number;
    longitude: number;
    uncertainty: number; // km
  };
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Physical and orbital constants
 */
export const SPACE_CONSTANTS = {
  /** Earth's gravitational parameter (m³/s²) */
  EARTH_MU: 3.986004418e14,

  /** Earth's radius (m) */
  EARTH_RADIUS: 6.378137e6,

  /** Earth's J2 coefficient */
  EARTH_J2: 1.08263e-3,

  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 2.998e8,

  /** Solar radiation pressure at 1 AU (N/m²) */
  SOLAR_PRESSURE: 4.56e-6,

  /** LEO altitude range (km) */
  LEO_ALTITUDE: { min: 160, max: 2000 },

  /** MEO altitude range (km) */
  MEO_ALTITUDE: { min: 2000, max: 35786 },

  /** GEO altitude (km) */
  GEO_ALTITUDE: 35786,

  /** Conjunction screening thresholds (m) */
  SCREENING_THRESHOLD: {
    LEO: 5000,
    MEO: 10000,
    GEO: 50000,
  },

  /** Collision probability thresholds */
  PC_THRESHOLD: {
    MINIMAL: 1e-5,
    LOW: 1e-4,
    MEDIUM: 1e-3,
    HIGH: 1e-2,
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-DEF-012 error codes
 */
export enum SpaceSurveillanceErrorCode {
  OBJECT_NOT_FOUND = 'S001',
  INSUFFICIENT_DATA = 'S002',
  ORBIT_DETERMINATION_FAILED = 'S003',
  CONJUNCTION_THRESHOLD_EXCEEDED = 'S004',
  SENSOR_UNAVAILABLE = 'S005',
  DATA_QUALITY_POOR = 'S006',
  INVALID_PARAMETERS = 'S007',
  CATALOG_UPDATE_FAILED = 'S008',
  MANEUVER_PLANNING_FAILED = 'S009',
  TLE_PARSE_ERROR = 'S010',
}

/**
 * Space surveillance error
 */
export class SpaceSurveillanceError extends Error {
  constructor(
    public code: SpaceSurveillanceErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'SpaceSurveillanceError';
  }
}

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

/**
 * Pagination parameters
 */
export interface PaginationParams {
  /** Page number (0-indexed) */
  page: number;

  /** Items per page */
  limit: number;

  /** Sort field */
  sortBy?: string;

  /** Sort order */
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated results
 */
export interface PaginatedResult<T> {
  /** Data items */
  items: T[];

  /** Total count */
  total: number;

  /** Current page */
  page: number;

  /** Items per page */
  limit: number;

  /** Total pages */
  totalPages: number;

  /** Has next page */
  hasNext: boolean;

  /** Has previous page */
  hasPrev: boolean;
}
