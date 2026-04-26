/**
 * WIA-DEF-015: Missile Defense - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense & Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Defense Types
// ============================================================================

/**
 * Missile threat types
 */
export type ThreatType =
  | 'ballistic'
  | 'cruise'
  | 'hypersonic'
  | 'srbm'
  | 'mrbm'
  | 'irbm'
  | 'icbm';

/**
 * Threat classification levels
 */
export type ThreatLevel = 'CRITICAL' | 'HIGH' | 'MEDIUM' | 'LOW';

/**
 * Engagement status
 */
export type EngagementStatus =
  | 'detecting'
  | 'tracking'
  | 'calculating'
  | 'authorized'
  | 'launched'
  | 'intercepting'
  | 'intercepted'
  | 'missed'
  | 'aborted';

/**
 * Defense layer types
 */
export type DefenseLayer = 'boost' | 'midcourse' | 'terminal' | 'point';

/**
 * Interceptor types
 */
export type InterceptorType =
  | 'THAAD'
  | 'PAC-3'
  | 'SM-3'
  | 'SM-6'
  | 'Aegis'
  | 'Arrow-3'
  | 'Iron-Dome'
  | 'CIWS'
  | 'Custom';

/**
 * Assessment confidence levels
 */
export type ConfidenceLevel = 'high' | 'medium' | 'low' | 'uncertain';

// ============================================================================
// Geographical & Physical Types
// ============================================================================

/**
 * Geographic location with altitude
 */
export interface GeoLocation {
  /** Latitude in degrees (-90 to 90) */
  latitude: number;

  /** Longitude in degrees (-180 to 180) */
  longitude: number;

  /** Altitude in meters above sea level */
  altitude: number;

  /** Horizontal uncertainty in meters (optional) */
  horizontalUncertainty?: number;

  /** Vertical uncertainty in meters (optional) */
  verticalUncertainty?: number;
}

/**
 * 3D velocity vector
 */
export interface Velocity {
  /** X component (m/s) - eastward */
  x: number;

  /** Y component (m/s) - northward */
  y: number;

  /** Z component (m/s) - upward */
  z: number;

  /** Magnitude (m/s) */
  magnitude?: number;
}

/**
 * 3D acceleration vector
 */
export interface Acceleration {
  /** X component (m/s²) */
  x: number;

  /** Y component (m/s²) */
  y: number;

  /** Z component (m/s²) */
  z: number;
}

/**
 * Uncertainty ellipse for position estimation
 */
export interface UncertaintyEllipse {
  /** Semi-major axis (meters) */
  semiMajor: number;

  /** Semi-minor axis (meters) */
  semiMinor: number;

  /** Orientation angle (degrees from north) */
  orientation: number;

  /** Confidence level (e.g., 0.95 for 95%) */
  confidence: number;
}

// ============================================================================
// Threat Detection
// ============================================================================

/**
 * Detected missile threat
 */
export interface Threat {
  /** Unique threat identifier */
  id: string;

  /** Threat type classification */
  type: ThreatType;

  /** Threat level */
  level: ThreatLevel;

  /** Detection timestamp */
  detectedAt: Date;

  /** Detection confidence (0-1) */
  confidence: number;

  /** Current position */
  position: GeoLocation;

  /** Current velocity */
  velocity: Velocity;

  /** Current acceleration (if available) */
  acceleration?: Acceleration;

  /** Predicted impact location */
  predictedImpact?: ImpactPrediction;

  /** Detecting sensors */
  detectedBy: string[];

  /** Track quality (0-1) */
  trackQuality: number;

  /** Threat characteristics */
  characteristics?: {
    rcs?: number; // Radar cross-section (m²)
    speed?: number; // Velocity magnitude (m/s)
    apogee?: number; // Peak altitude (m)
    range?: number; // Estimated range (m)
  };
}

/**
 * Threat detection parameters
 */
export interface ThreatDetectionParams {
  /** Sensor network to use */
  sensorNetwork: string[];

  /** Minimum detection confidence (0-1) */
  detectionThreshold: number;

  /** Threat types to detect */
  threatTypes?: ThreatType[];

  /** Search volume (optional) */
  searchVolume?: SearchVolume;

  /** Enable multi-sensor fusion */
  enableFusion?: boolean;
}

/**
 * Search volume definition
 */
export interface SearchVolume {
  /** Center point */
  center: GeoLocation;

  /** Radius in meters */
  radius: number;

  /** Minimum altitude (m) */
  minAltitude?: number;

  /** Maximum altitude (m) */
  maxAltitude?: number;
}

/**
 * Threat detection result
 */
export interface ThreatDetectionResult {
  /** Detected threats */
  threats: Threat[];

  /** Detection timestamp */
  timestamp: Date;

  /** Overall confidence (0-1) */
  confidence: number;

  /** Contributing sensors */
  sensorContributors: string[];

  /** Detection metadata */
  metadata?: {
    searchVolume?: SearchVolume;
    processingTime?: number; // milliseconds
    falseAlarmProbability?: number;
  };
}

// ============================================================================
// Trajectory Tracking
// ============================================================================

/**
 * Trajectory state vector
 */
export interface TrajectoryState {
  /** Timestamp */
  time: Date;

  /** Position */
  position: GeoLocation;

  /** Velocity */
  velocity: Velocity;

  /** Acceleration */
  acceleration?: Acceleration;

  /** State uncertainty */
  uncertainty?: UncertaintyEllipse;
}

/**
 * Impact prediction
 */
export interface ImpactPrediction {
  /** Predicted impact time */
  impactTime: Date;

  /** Predicted impact location */
  impactLocation: GeoLocation;

  /** Circular Error Probable (meters) */
  cep: number;

  /** Time to impact (seconds) */
  timeToImpact: number;

  /** Prediction confidence (0-1) */
  confidence: number;
}

/**
 * Trajectory tracking parameters
 */
export interface TrackingParams {
  /** Threat ID to track */
  threatId: string;

  /** Update interval (milliseconds) */
  updateInterval: number;

  /** Prediction horizon (seconds) */
  predictionHorizon: number;

  /** Filter type */
  filterType?: 'kalman' | 'particle' | 'hybrid';

  /** Enable atmospheric modeling */
  enableAtmospheric?: boolean;
}

/**
 * Trajectory tracking result
 */
export interface TrackingResult {
  /** Threat ID */
  threatId: string;

  /** Trajectory history */
  trajectory: TrajectoryState[];

  /** Impact prediction */
  prediction: ImpactPrediction;

  /** Track uncertainty */
  uncertainty: UncertaintyEllipse;

  /** Track quality (0-1) */
  trackQuality: number;

  /** Tracking status */
  status: 'acquired' | 'tracking' | 'lost' | 'predicted';
}

// ============================================================================
// Intercept Calculations
// ============================================================================

/**
 * Interceptor trajectory
 */
export interface InterceptorTrajectory {
  /** Launch point */
  launchPoint: GeoLocation;

  /** Intercept point */
  interceptPoint: GeoLocation;

  /** Trajectory waypoints */
  waypoints: TrajectoryState[];

  /** Flight time (seconds) */
  flightTime: number;

  /** Fuel consumption (kg) */
  fuelConsumption?: number;
}

/**
 * Intercept calculation parameters
 */
export interface InterceptParams {
  /** Threat trajectory */
  threatTrajectory: TrajectoryState[];

  /** Interceptor type */
  interceptorType: InterceptorType;

  /** Interceptor location */
  interceptorLocation: GeoLocation;

  /** Engagement constraints */
  constraints?: {
    /** Minimum intercept altitude (m) */
    minAltitude?: number;

    /** Maximum engagement range (m) */
    maxRange?: number;

    /** Minimum time to impact (s) */
    minTimeToImpact?: number;

    /** Maximum lateral acceleration (m/s²) */
    maxAcceleration?: number;
  };

  /** Engagement time */
  engagementTime?: Date;
}

/**
 * Intercept calculation result
 */
export interface InterceptResult {
  /** Intercept feasibility */
  feasible: boolean;

  /** Optimal intercept point */
  interceptPoint?: GeoLocation;

  /** Time to intercept (seconds) */
  timeToIntercept?: number;

  /** Kill probability (0-1) */
  killProbability?: number;

  /** Recommended launch time */
  launchTime?: Date;

  /** Interceptor trajectory */
  trajectory?: InterceptorTrajectory;

  /** Engagement constraints satisfied */
  constraintsSatisfied?: {
    altitude: boolean;
    range: boolean;
    timeToImpact: boolean;
    acceleration: boolean;
  };

  /** Reasons if not feasible */
  infeasibilityReasons?: string[];
}

// ============================================================================
// Engagement Management
// ============================================================================

/**
 * Interceptor launch parameters
 */
export interface LaunchParams {
  /** Intercept calculation result */
  interceptResult: InterceptResult;

  /** Interceptor ID */
  interceptorId: string;

  /** Battery ID */
  batteryId?: string;

  /** Launch command */
  launchCommand: 'ENGAGE' | 'SIMULATE' | 'TEST';

  /** Authorization token */
  authorization?: string;

  /** Rules of engagement verification */
  roeVerified?: boolean;
}

/**
 * Launch execution result
 */
export interface LaunchResult {
  /** Engagement ID */
  engagementId: string;

  /** Launch status */
  status: 'launched' | 'failed' | 'aborted';

  /** Launch timestamp */
  launchTime: Date;

  /** Interceptor details */
  interceptor: {
    id: string;
    type: InterceptorType;
    batteryId?: string;
  };

  /** Expected intercept time */
  expectedInterceptTime?: Date;

  /** Telemetry available */
  telemetryAvailable: boolean;

  /** Error message if failed */
  errorMessage?: string;
}

/**
 * Engagement record
 */
export interface Engagement {
  /** Unique engagement ID */
  id: string;

  /** Threat ID */
  threatId: string;

  /** Engagement status */
  status: EngagementStatus;

  /** Timeline */
  timeline: {
    detected: Date;
    authorized?: Date;
    launched?: Date;
    intercept?: Date;
    assessment?: Date;
  };

  /** Interceptor used */
  interceptor: {
    type: InterceptorType;
    id: string;
    batteryId?: string;
  };

  /** Kill assessment */
  killAssessment?: KillAssessment;

  /** Engagement metadata */
  metadata?: {
    operatorId?: string;
    authorizingAuthority?: string;
    engagementLayer?: DefenseLayer;
  };
}

// ============================================================================
// Kill Assessment
// ============================================================================

/**
 * Kill assessment method
 */
export type AssessmentMethod = 'radar' | 'infrared' | 'optical' | 'debris-analysis' | 'multi-sensor';

/**
 * Kill assessment parameters
 */
export interface KillAssessmentParams {
  /** Engagement ID */
  engagementId: string;

  /** Assessment sensors */
  sensors: string[];

  /** Assessment method */
  assessmentMethod: AssessmentMethod;

  /** Post-intercept observation time (seconds) */
  observationTime?: number;
}

/**
 * Kill assessment evidence
 */
export interface AssessmentEvidence {
  /** Evidence type */
  type: 'radar' | 'infrared' | 'optical' | 'telemetry';

  /** Evidence timestamp */
  timestamp: Date;

  /** Sensor ID */
  sensorId: string;

  /** Evidence data */
  data: {
    /** Debris count (radar) */
    debrisCount?: number;

    /** Flash intensity (IR) */
    flashIntensity?: number;

    /** Trajectory deviation (degrees) */
    trajectoryDeviation?: number;

    /** RCS change (dB) */
    rcsChange?: number;

    /** Fragmentation observed (optical) */
    fragmentationObserved?: boolean;
  };

  /** Evidence confidence (0-1) */
  confidence: number;
}

/**
 * Kill assessment result
 */
export interface KillAssessment {
  /** Engagement ID */
  engagementId: string;

  /** Assessment timestamp */
  timestamp: Date;

  /** Kill probability (0-1) */
  killProbability: number;

  /** Threat neutralized */
  threatNeutralized: boolean;

  /** Confidence level */
  confidence: ConfidenceLevel;

  /** Assessment method used */
  method: AssessmentMethod[];

  /** Evidence collected */
  evidence: AssessmentEvidence[];

  /** Recommendation */
  recommendation: 'confirmed-kill' | 'probable-kill' | 'uncertain' | 'missed' | 'second-shot-required';

  /** Detailed analysis */
  analysis?: {
    debrisPattern?: string;
    thermalSignature?: string;
    radarSignature?: string;
    visualConfirmation?: boolean;
  };
}

// ============================================================================
// Defense Systems
// ============================================================================

/**
 * Interceptor system characteristics
 */
export interface InterceptorSystemSpec {
  /** System type */
  type: InterceptorType;

  /** System name */
  name: string;

  /** Performance characteristics */
  performance: {
    /** Maximum range (meters) */
    maxRange: number;

    /** Minimum range (meters) */
    minRange: number;

    /** Maximum altitude (meters) */
    maxAltitude: number;

    /** Minimum altitude (meters) */
    minAltitude: number;

    /** Maximum velocity (m/s) */
    maxVelocity: number;

    /** Maximum acceleration (m/s²) */
    maxAcceleration: number;

    /** Single-shot kill probability */
    killProbability: number;

    /** Reaction time (seconds) */
    reactionTime: number;
  };

  /** Engagement envelope */
  envelope: {
    /** Optimal intercept altitude (meters) */
    optimalAltitude: number;

    /** Optimal range (meters) */
    optimalRange: number;

    /** Defense layer */
    layer: DefenseLayer;
  };

  /** System availability */
  availability?: {
    /** Ready interceptors */
    ready: number;

    /** Total inventory */
    total: number;

    /** Reloading */
    reloading: number;
  };
}

/**
 * Defense battery configuration
 */
export interface DefenseBattery {
  /** Battery ID */
  id: string;

  /** Battery name */
  name: string;

  /** Location */
  location: GeoLocation;

  /** Interceptor system type */
  systemType: InterceptorType;

  /** System specifications */
  systemSpec: InterceptorSystemSpec;

  /** Operational status */
  status: 'operational' | 'degraded' | 'offline' | 'maintenance';

  /** Coverage area */
  coverage: {
    /** Coverage radius (meters) */
    radius: number;

    /** Sector coverage (degrees) */
    sector?: {
      azimuthStart: number;
      azimuthEnd: number;
    };

    /** Altitude coverage */
    altitudeRange: {
      min: number;
      max: number;
    };
  };

  /** Current engagements */
  activeEngagements?: number;

  /** Maximum simultaneous engagements */
  maxSimultaneousEngagements: number;
}

// ============================================================================
// Defense Network
// ============================================================================

/**
 * Sensor network node
 */
export interface SensorNode {
  /** Sensor ID */
  id: string;

  /** Sensor type */
  type: 'radar' | 'infrared' | 'optical' | 'space-based';

  /** Location */
  location: GeoLocation;

  /** Operational status */
  status: 'active' | 'standby' | 'degraded' | 'offline';

  /** Coverage */
  coverage: {
    range: number; // meters
    azimuthCoverage?: { start: number; end: number }; // degrees
    elevationCoverage?: { min: number; max: number }; // degrees
  };

  /** Detection performance */
  performance?: {
    detectionRange: number; // meters
    trackingAccuracy: number; // meters
    updateRate: number; // Hz
  };
}

/**
 * Integrated defense network
 */
export interface DefenseNetwork {
  /** Network ID */
  id: string;

  /** Network name */
  name: string;

  /** Defense batteries */
  batteries: DefenseBattery[];

  /** Sensor network */
  sensors: SensorNode[];

  /** Command & control */
  commandControl: {
    nodeId: string;
    location: GeoLocation;
    status: 'active' | 'backup' | 'offline';
  };

  /** Network status */
  status: 'fully-operational' | 'degraded' | 'critical' | 'offline';

  /** Coverage assessment */
  coverage?: {
    totalArea: number; // km²
    gaps: GeoLocation[]; // Coverage gap locations
    overlaps: number; // Number of overlapping coverage zones
  };
}

// ============================================================================
// Monitoring & Analytics
// ============================================================================

/**
 * Defense dashboard metrics
 */
export interface DefenseDashboard {
  /** Dashboard timestamp */
  timestamp: Date;

  /** Active threats */
  activeThreats: {
    total: number;
    byType: Record<ThreatType, number>;
    byLevel: Record<ThreatLevel, number>;
  };

  /** Active engagements */
  activeEngagements: {
    total: number;
    byLayer: Record<DefenseLayer, number>;
    byStatus: Record<EngagementStatus, number>;
  };

  /** System readiness */
  systemReadiness: {
    /** Available interceptors by type */
    interceptorsAvailable: Record<InterceptorType, number>;

    /** Sensor availability (percentage) */
    sensorAvailability: number;

    /** Overall readiness score (0-1) */
    readinessScore: number;
  };

  /** Performance metrics */
  performance: {
    /** Total engagements (last 24h) */
    totalEngagements: number;

    /** Successful intercepts (last 24h) */
    successfulIntercepts: number;

    /** Success rate (0-1) */
    successRate: number;

    /** Average response time (seconds) */
    avgResponseTime: number;
  };

  /** Alert status */
  alertStatus: 'NORMAL' | 'ELEVATED' | 'HIGH' | 'CRITICAL';
}

/**
 * Defense monitoring parameters
 */
export interface MonitoringParams {
  /** Real-time updates */
  realTime?: boolean;

  /** Update interval (milliseconds) */
  updateInterval?: number;

  /** Include historical data */
  includeHistorical?: boolean;

  /** Filter options */
  filters?: {
    threatTypes?: ThreatType[];
    threatLevels?: ThreatLevel[];
    defenseLayers?: DefenseLayer[];
  };
}

// ============================================================================
// Configuration
// ============================================================================

/**
 * Missile Defense SDK configuration
 */
export interface MissileDefenseConfig {
  /** Defense network ID */
  defenseNetwork?: string;

  /** Enable sensor fusion */
  sensorFusion?: boolean;

  /** Auto-engagement mode */
  autoEngagement?: boolean;

  /** Default interceptor type */
  defaultInterceptorType?: InterceptorType;

  /** Minimum kill probability for engagement */
  minKillProbability?: number;

  /** Rules of engagement */
  rulesOfEngagement?: {
    requireHumanAuthorization?: boolean;
    minThreatLevel?: ThreatLevel;
    noFireZones?: GeoLocation[];
  };

  /** Logging configuration */
  logging?: {
    enabled: boolean;
    level: 'debug' | 'info' | 'warn' | 'error';
    destination?: string;
  };

  /** API configuration */
  api?: {
    endpoint?: string;
    apiKey?: string;
    timeout?: number; // milliseconds
  };
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Physical constants
 */
export const PHYSICAL_CONSTANTS = {
  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Gravitational constant (m³/kg·s²) */
  GRAVITATIONAL_CONSTANT: 6.674e-11,

  /** Earth's gravitational parameter (m³/s²) */
  EARTH_MU: 3.986004418e14,

  /** Earth's radius (m) */
  EARTH_RADIUS: 6371000,

  /** Earth's rotation rate (rad/s) */
  EARTH_ROTATION_RATE: 7.2921159e-5,

  /** Standard gravity (m/s²) */
  STANDARD_GRAVITY: 9.80665,
} as const;

/**
 * Intercept probability thresholds
 */
export const KILL_PROBABILITY_THRESHOLDS = {
  /** Minimum acceptable P(kill) */
  MINIMUM: 0.6,

  /** Target P(kill) */
  TARGET: 0.8,

  /** High confidence P(kill) */
  HIGH: 0.9,
} as const;

/**
 * Threat level scoring
 */
export const THREAT_LEVEL_SCORES = {
  CRITICAL: 10,
  HIGH: 7,
  MEDIUM: 5,
  LOW: 3,
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-DEF-015 error codes
 */
export enum MissileDefenseErrorCode {
  DETECTION_FAILED = 'MD001',
  TRACK_LOST = 'MD002',
  INTERCEPT_NOT_FEASIBLE = 'MD003',
  LAUNCH_AUTHORIZATION_DENIED = 'MD004',
  INTERCEPTOR_FAILURE = 'MD005',
  KILL_ASSESSMENT_UNCERTAIN = 'MD006',
  INVALID_PARAMETERS = 'MD007',
  SYSTEM_OFFLINE = 'MD008',
  SENSOR_MALFUNCTION = 'MD009',
  ENGAGEMENT_ABORT = 'MD010',
}

/**
 * Missile defense error
 */
export class MissileDefenseError extends Error {
  constructor(
    public code: MissileDefenseErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'MissileDefenseError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Geographical
  GeoLocation,
  Velocity,
  Acceleration,
  UncertaintyEllipse,

  // Threats
  Threat,
  ThreatDetectionParams,
  SearchVolume,
  ThreatDetectionResult,

  // Tracking
  TrajectoryState,
  ImpactPrediction,
  TrackingParams,
  TrackingResult,

  // Intercept
  InterceptorTrajectory,
  InterceptParams,
  InterceptResult,

  // Engagement
  LaunchParams,
  LaunchResult,
  Engagement,

  // Assessment
  AssessmentEvidence,
  KillAssessmentParams,
  KillAssessment,

  // Systems
  InterceptorSystemSpec,
  DefenseBattery,
  SensorNode,
  DefenseNetwork,

  // Monitoring
  DefenseDashboard,
  MonitoringParams,

  // Configuration
  MissileDefenseConfig,
};

export {
  PHYSICAL_CONSTANTS,
  KILL_PROBABILITY_THRESHOLDS,
  THREAT_LEVEL_SCORES,
  MissileDefenseErrorCode,
  MissileDefenseError,
};
