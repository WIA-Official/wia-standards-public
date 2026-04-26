/**
 * WIA-DEF-010: Military Satellite - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense & Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Three-dimensional vector for position and velocity
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Geographic coordinates
 */
export interface GeographicCoordinate {
  /** Latitude in degrees (-90 to 90) */
  latitude: number;

  /** Longitude in degrees (-180 to 180) */
  longitude: number;

  /** Altitude above sea level in meters */
  altitude: number;
}

/**
 * Orbital element set (Keplerian elements)
 */
export interface OrbitalElements {
  /** Semi-major axis in meters */
  semiMajorAxis: number;

  /** Eccentricity (0 to 1, 0 = circular) */
  eccentricity: number;

  /** Inclination in degrees (0 to 180) */
  inclination: number;

  /** Right ascension of ascending node in degrees */
  raan: number;

  /** Argument of periapsis in degrees */
  argumentOfPeriapsis: number;

  /** True anomaly in degrees */
  trueAnomaly: number;

  /** Epoch time */
  epoch: Date;
}

// ============================================================================
// Satellite Types
// ============================================================================

/**
 * Satellite mission type
 */
export type SatelliteMissionType =
  | 'reconnaissance'
  | 'communication'
  | 'navigation'
  | 'early-warning'
  | 'weather'
  | 'relay';

/**
 * Reconnaissance satellite subtype
 */
export type ReconnaissanceType = 'imint' | 'sigint' | 'elint' | 'sar' | 'multi-spectral';

/**
 * Orbit type classification
 */
export type OrbitType = 'leo' | 'meo' | 'geo' | 'heo' | 'sso' | 'polar';

/**
 * Military satellite configuration
 */
export interface MilitarySatellite {
  /** Unique satellite identifier */
  id: string;

  /** Satellite name/designation */
  name: string;

  /** Mission type */
  missionType: SatelliteMissionType;

  /** Reconnaissance subtype (if applicable) */
  reconType?: ReconnaissanceType;

  /** Orbit type */
  orbitType: OrbitType;

  /** Orbital elements */
  orbitalElements: OrbitalElements;

  /** Current position (ECI coordinates) */
  position: Vector3;

  /** Current velocity (ECI coordinates) */
  velocity: Vector3;

  /** Launch date */
  launchDate: Date;

  /** Operational status */
  status: 'operational' | 'standby' | 'maintenance' | 'degraded' | 'decommissioned';

  /** Payload information */
  payloads: Payload[];

  /** Ground stations */
  groundStations: string[];

  /** Security classification */
  classification: 'unclassified' | 'confidential' | 'secret' | 'top-secret' | 'sci';
}

/**
 * Satellite payload
 */
export interface Payload {
  /** Payload identifier */
  id: string;

  /** Payload type */
  type: 'optical' | 'radar' | 'infrared' | 'rf-sensor' | 'transponder' | 'other';

  /** Payload name */
  name: string;

  /** Operating status */
  status: 'active' | 'standby' | 'failed';

  /** Power consumption in watts */
  power: number;

  /** Specifications */
  specifications: Record<string, unknown>;
}

// ============================================================================
// Orbital Mechanics
// ============================================================================

/**
 * Orbital parameters request
 */
export interface OrbitalRequest {
  /** Altitude above Earth surface in meters */
  altitude: number;

  /** Inclination in degrees (0-180) */
  inclination?: number;

  /** Eccentricity (0-1, 0 = circular) */
  eccentricity?: number;

  /** Orbit type */
  orbitalType?: OrbitType;
}

/**
 * Orbital parameters response
 */
export interface OrbitalResponse {
  /** Orbital velocity in m/s */
  velocity: number;

  /** Orbital period in seconds */
  period: number;

  /** Semi-major axis in meters */
  semiMajorAxis: number;

  /** Apogee altitude in meters */
  apogee: number;

  /** Perigee altitude in meters */
  perigee: number;

  /** Orbital radius in meters */
  radius: number;

  /** Angular velocity in rad/s */
  angularVelocity: number;
}

/**
 * Ground track point
 */
export interface GroundTrackPoint {
  /** Time */
  time: Date;

  /** Geographic position */
  position: GeographicCoordinate;

  /** Azimuth in degrees */
  azimuth?: number;

  /** Elevation in degrees */
  elevation?: number;
}

/**
 * Satellite pass information
 */
export interface SatellitePass {
  /** Satellite ID */
  satelliteId: string;

  /** Rise time (AOS - Acquisition of Signal) */
  riseTime: Date;

  /** Rise azimuth in degrees */
  riseAzimuth: number;

  /** Maximum elevation time (TCA - Time of Closest Approach) */
  maxElevationTime: Date;

  /** Maximum elevation in degrees */
  maxElevation: number;

  /** Set time (LOS - Loss of Signal) */
  setTime: Date;

  /** Set azimuth in degrees */
  setAzimuth: number;

  /** Pass duration in seconds */
  duration: number;

  /** Average range in meters */
  averageRange: number;
}

// ============================================================================
// Communication Systems
// ============================================================================

/**
 * Frequency band
 */
export type FrequencyBand =
  | 'uhf'      // 300 MHz - 3 GHz
  | 'l-band'   // 1-2 GHz
  | 's-band'   // 2-4 GHz
  | 'c-band'   // 4-8 GHz
  | 'x-band'   // 8-12 GHz
  | 'ku-band'  // 12-18 GHz
  | 'ka-band'  // 26-40 GHz
  | 'ehf';     // 30-300 GHz

/**
 * Modulation scheme
 */
export type ModulationScheme = 'bpsk' | 'qpsk' | '8psk' | '16qam' | '64qam' | 'oqpsk';

/**
 * Encryption level
 */
export type EncryptionLevel = 'none' | 'aes-128' | 'aes-256' | 'rsa-2048' | 'rsa-4096' | 'quantum';

/**
 * Link validation request
 */
export interface LinkValidation {
  /** Satellite identifier */
  satelliteId: string;

  /** Ground station identifier */
  groundStation: string;

  /** Frequency in Hz */
  frequency: number;

  /** Bandwidth in Hz */
  bandwidth?: number;

  /** Encryption level */
  encryptionLevel: EncryptionLevel;

  /** Weather conditions (if available) */
  weather?: WeatherConditions;

  /** Minimum required signal strength in dBm */
  minSignalStrength?: number;
}

/**
 * Link validation result
 */
export interface LinkResult {
  /** Is the link valid? */
  isValid: boolean;

  /** Signal strength in dBm */
  signalStrength: number;

  /** Link margin in dB */
  linkMargin: number;

  /** Carrier-to-noise ratio in dB */
  cnr: number;

  /** Maximum data rate in bps */
  dataRate: number;

  /** Latency in milliseconds */
  latency: number;

  /** Errors (blocking) */
  errors: string[];

  /** Warnings (non-blocking) */
  warnings: string[];

  /** Link budget details */
  linkBudget?: LinkBudget;
}

/**
 * Link budget calculation
 */
export interface LinkBudget {
  /** Transmitter EIRP in dBW */
  eirp: number;

  /** Path loss in dB */
  pathLoss: number;

  /** Atmospheric loss in dB */
  atmosphericLoss: number;

  /** Rain attenuation in dB */
  rainAttenuation: number;

  /** Receiver G/T in dB/K */
  gOverT: number;

  /** System noise temperature in K */
  systemNoise: number;

  /** C/N0 in dB-Hz */
  cn0: number;

  /** Required C/N0 in dB-Hz */
  requiredCn0: number;

  /** Link margin in dB */
  margin: number;
}

/**
 * Weather conditions
 */
export interface WeatherConditions {
  /** Cloud cover (0-1) */
  cloudCover: number;

  /** Precipitation rate in mm/hr */
  precipitationRate: number;

  /** Temperature in Celsius */
  temperature: number;

  /** Humidity (0-1) */
  humidity: number;

  /** Wind speed in m/s */
  windSpeed: number;
}

// ============================================================================
// Reconnaissance Systems
// ============================================================================

/**
 * Imaging parameters
 */
export interface ImagingParameters {
  /** Ground sample distance in meters */
  gsd: number;

  /** Swath width in meters */
  swathWidth: number;

  /** Spectral bands */
  spectralBands: SpectralBand[];

  /** Bits per pixel */
  bitDepth: number;

  /** Signal-to-noise ratio in dB */
  snr: number;
}

/**
 * Spectral band definition
 */
export interface SpectralBand {
  /** Band name */
  name: string;

  /** Wavelength range (min, max) in nanometers */
  wavelengthRange: [number, number];

  /** Center wavelength in nanometers */
  centerWavelength: number;

  /** Bandwidth in nanometers */
  bandwidth: number;
}

/**
 * Imaging request
 */
export interface ImagingRequest {
  /** Satellite ID */
  satelliteId: string;

  /** Target location */
  target: GeographicCoordinate;

  /** Imaging mode */
  mode: 'spotlight' | 'stripmap' | 'scansar' | 'video';

  /** Required resolution in meters */
  resolution: number;

  /** Priority */
  priority: 'routine' | 'priority' | 'immediate' | 'flash';

  /** Start time */
  startTime?: Date;

  /** End time */
  endTime?: Date;
}

/**
 * SAR imaging parameters
 */
export interface SARParameters {
  /** Frequency in Hz */
  frequency: number;

  /** Bandwidth in Hz */
  bandwidth: number;

  /** Azimuth resolution in meters */
  azimuthResolution: number;

  /** Range resolution in meters */
  rangeResolution: number;

  /** Swath width in meters */
  swathWidth: number;

  /** Polarization */
  polarization: 'HH' | 'VV' | 'HV' | 'VH' | 'dual' | 'quad';

  /** Look angle in degrees */
  lookAngle: number;
}

// ============================================================================
// Navigation Systems
// ============================================================================

/**
 * GNSS system
 */
export type GNSSSystem = 'gps' | 'glonass' | 'galileo' | 'beidou' | 'qzss' | 'irnss';

/**
 * Navigation signal
 */
export interface NavigationSignal {
  /** GNSS system */
  system: GNSSSystem;

  /** Signal name */
  signal: string;

  /** Frequency in Hz */
  frequency: number;

  /** Chip rate in chips/sec */
  chipRate: number;

  /** Code length */
  codeLength: number;

  /** Modulation */
  modulation: string;

  /** Civilian or military */
  type: 'civilian' | 'military';
}

/**
 * Position solution
 */
export interface PositionSolution {
  /** Position */
  position: GeographicCoordinate;

  /** Velocity (ENU frame) in m/s */
  velocity: Vector3;

  /** Position accuracy (1-sigma) in meters */
  accuracy: {
    horizontal: number;
    vertical: number;
  };

  /** Velocity accuracy (1-sigma) in m/s */
  velocityAccuracy: number;

  /** Number of satellites used */
  satellitesUsed: number;

  /** Dilution of precision */
  dop: {
    gdop: number;
    pdop: number;
    hdop: number;
    vdop: number;
    tdop: number;
  };

  /** Solution time */
  time: Date;

  /** Fix type */
  fixType: 'no-fix' | '2d' | '3d' | 'dgps' | 'rtk-float' | 'rtk-fixed';
}

// ============================================================================
// Early Warning Systems
// ============================================================================

/**
 * Infrared detection event
 */
export interface IRDetection {
  /** Detection ID */
  id: string;

  /** Satellite ID */
  satelliteId: string;

  /** Detection time */
  detectionTime: Date;

  /** Geographic location of event */
  location: GeographicCoordinate;

  /** IR intensity in W/sr */
  intensity: number;

  /** Wavelength band in micrometers */
  wavelength: number;

  /** Event classification */
  classification: 'missile-launch' | 'explosion' | 'fire' | 'unknown' | 'false-alarm';

  /** Confidence level (0-1) */
  confidence: number;

  /** Track ID (if correlated) */
  trackId?: string;
}

/**
 * Ballistic trajectory
 */
export interface BallisticTrajectory {
  /** Track ID */
  trackId: string;

  /** Launch time */
  launchTime: Date;

  /** Launch location */
  launchLocation: GeographicCoordinate;

  /** Estimated impact time */
  impactTime?: Date;

  /** Estimated impact location */
  impactLocation?: GeographicCoordinate;

  /** Apogee altitude in meters */
  apogee: number;

  /** Maximum velocity in m/s */
  maxVelocity: number;

  /** Range in meters */
  range: number;

  /** Missile type estimate */
  missileType?: 'icbm' | 'slbm' | 'irbm' | 'srbm' | 'cruise' | 'unknown';

  /** Threat level */
  threatLevel: 'low' | 'medium' | 'high' | 'critical';
}

// ============================================================================
// Ground Control
// ============================================================================

/**
 * Ground station
 */
export interface GroundStation {
  /** Station identifier */
  id: string;

  /** Station name */
  name: string;

  /** Geographic location */
  location: GeographicCoordinate;

  /** Antenna systems */
  antennas: Antenna[];

  /** Supported frequency bands */
  frequencyBands: FrequencyBand[];

  /** Status */
  status: 'operational' | 'maintenance' | 'offline';

  /** Maximum elevation mask in degrees */
  elevationMask: number;

  /** Security level */
  securityLevel: 'unclassified' | 'secret' | 'top-secret';
}

/**
 * Antenna system
 */
export interface Antenna {
  /** Antenna ID */
  id: string;

  /** Antenna type */
  type: 'parabolic' | 'phased-array' | 'helical' | 'patch';

  /** Diameter in meters (for parabolic) */
  diameter?: number;

  /** Gain in dBi */
  gain: number;

  /** Beamwidth in degrees */
  beamwidth: number;

  /** Frequency range in Hz */
  frequencyRange: [number, number];

  /** Tracking capability */
  tracking: 'manual' | 'program-track' | 'auto-track';

  /** Slew rate in degrees/sec */
  slewRate?: number;
}

/**
 * Telemetry data point
 */
export interface TelemetryData {
  /** Satellite ID */
  satelliteId: string;

  /** Timestamp */
  timestamp: Date;

  /** Battery voltage in volts */
  batteryVoltage: number;

  /** Battery current in amps */
  batteryCurrent: number;

  /** Solar array power in watts */
  solarPower: number;

  /** Subsystem temperatures in Celsius */
  temperatures: Record<string, number>;

  /** Attitude (quaternion) */
  attitude: {
    q0: number;
    q1: number;
    q2: number;
    q3: number;
  };

  /** Angular rates in deg/s */
  angularRates: Vector3;

  /** Payload status */
  payloadStatus: Record<string, string>;

  /** Anomalies */
  anomalies: string[];
}

// ============================================================================
// Security & Encryption
// ============================================================================

/**
 * Encryption key
 */
export interface EncryptionKey {
  /** Key ID */
  id: string;

  /** Key type */
  type: 'symmetric' | 'asymmetric';

  /** Algorithm */
  algorithm: 'aes-128' | 'aes-256' | 'rsa-2048' | 'rsa-4096' | 'ecc-256' | 'ecc-521';

  /** Key material (encrypted) */
  keyMaterial: string;

  /** Creation time */
  created: Date;

  /** Expiration time */
  expires: Date;

  /** Purpose */
  purpose: 'uplink' | 'downlink' | 'storage' | 'authentication';

  /** Status */
  status: 'active' | 'expired' | 'revoked';
}

/**
 * Encrypted message
 */
export interface EncryptedMessage {
  /** Message ID */
  id: string;

  /** Encryption algorithm */
  algorithm: string;

  /** Key ID used */
  keyId: string;

  /** Initialization vector (if applicable) */
  iv?: string;

  /** Encrypted payload (base64) */
  payload: string;

  /** Message authentication code */
  mac?: string;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Tracking & Position
// ============================================================================

/**
 * Satellite tracking request
 */
export interface TrackingRequest {
  /** Satellite ID */
  satelliteId: string;

  /** Start time */
  startTime: Date;

  /** Duration in seconds */
  duration: number;

  /** Observer location (optional) */
  observerLocation?: GeographicCoordinate;

  /** Time step in seconds */
  timeStep?: number;
}

/**
 * Satellite position (state vector)
 */
export interface SatellitePosition {
  /** Time */
  time: Date;

  /** Position in ECI coordinates (meters) */
  position: Vector3;

  /** Velocity in ECI coordinates (m/s) */
  velocity: Vector3;

  /** Geographic coordinates */
  geographic: GeographicCoordinate;

  /** Observer-relative parameters (if observer specified) */
  observer?: {
    azimuth: number;    // degrees
    elevation: number;  // degrees
    range: number;      // meters
    rangeRate: number;  // m/s
  };
}

// ============================================================================
// Mission Planning
// ============================================================================

/**
 * Tasking request
 */
export interface TaskingRequest {
  /** Request ID */
  id: string;

  /** Satellite ID or constellation */
  satelliteId: string | string[];

  /** Mission type */
  missionType: SatelliteMissionType;

  /** Target area */
  target: GeographicCoordinate | GeographicCoordinate[];

  /** Time window */
  timeWindow: {
    start: Date;
    end: Date;
  };

  /** Priority */
  priority: 'routine' | 'priority' | 'immediate' | 'flash';

  /** Requirements */
  requirements: {
    minResolution?: number;
    minElevation?: number;
    maxCloudCover?: number;
    requiredBands?: string[];
  };

  /** Requestor information */
  requestor: {
    organization: string;
    contact: string;
    classification: string;
  };
}

/**
 * Collection plan
 */
export interface CollectionPlan {
  /** Plan ID */
  id: string;

  /** Tasking request ID */
  requestId: string;

  /** Satellite ID */
  satelliteId: string;

  /** Collection windows */
  windows: CollectionWindow[];

  /** Total collection time in seconds */
  totalTime: number;

  /** Coverage percentage */
  coverage: number;

  /** Status */
  status: 'planned' | 'scheduled' | 'executing' | 'completed' | 'failed';
}

/**
 * Collection window
 */
export interface CollectionWindow {
  /** Window start time */
  startTime: Date;

  /** Window end time */
  endTime: Date;

  /** Duration in seconds */
  duration: number;

  /** Target location */
  target: GeographicCoordinate;

  /** Imaging parameters */
  parameters: ImagingParameters | SARParameters;

  /** Expected quality metrics */
  quality: {
    resolution: number;
    cloudCover?: number;
    sunAngle?: number;
    offNadir?: number;
  };
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical and orbital constants
 */
export const SATELLITE_CONSTANTS = {
  /** Earth mass in kg */
  EARTH_MASS: 5.972e24,

  /** Earth equatorial radius in meters */
  EARTH_RADIUS: 6378137,

  /** Earth mean radius in meters */
  EARTH_RADIUS_MEAN: 6371000,

  /** Gravitational constant in m³/kg·s² */
  GRAVITATIONAL_CONSTANT: 6.67430e-11,

  /** Earth gravitational parameter (GM) in m³/s² */
  EARTH_MU: 3.986004418e14,

  /** Speed of light in m/s */
  SPEED_OF_LIGHT: 299792458,

  /** Standard gravity in m/s² */
  STANDARD_GRAVITY: 9.80665,

  /** GEO altitude in meters */
  GEO_ALTITUDE: 35786000,

  /** GPS orbit altitude in meters */
  GPS_ALTITUDE: 20200000,

  /** LEO altitude range in meters */
  LEO_RANGE: { min: 160000, max: 2000000 },

  /** MEO altitude range in meters */
  MEO_RANGE: { min: 2000000, max: 35786000 },

  /** Boltzmann constant in J/K */
  BOLTZMANN: 1.380649e-23,
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
 * WIA-DEF-010 error codes
 */
export enum SatelliteErrorCode {
  SATELLITE_NOT_FOUND = 'S001',
  ORBIT_PROPAGATION_FAILED = 'S002',
  LINK_BUDGET_INSUFFICIENT = 'S003',
  ENCRYPTION_KEY_EXPIRED = 'S004',
  GROUND_STATION_UNAVAILABLE = 'S005',
  WEATHER_INTERFERENCE = 'S006',
  COLLISION_RISK = 'S007',
  PAYLOAD_FAILURE = 'S008',
  INVALID_PARAMETERS = 'S009',
  COMMUNICATION_TIMEOUT = 'S010',
}

/**
 * Satellite operation error
 */
export class SatelliteError extends Error {
  constructor(
    public code: SatelliteErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'SatelliteError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  GeographicCoordinate,
  OrbitalElements,

  // Satellite types
  MilitarySatellite,
  Payload,

  // Orbital mechanics
  OrbitalRequest,
  OrbitalResponse,
  GroundTrackPoint,
  SatellitePass,

  // Communications
  LinkValidation,
  LinkResult,
  LinkBudget,
  WeatherConditions,

  // Reconnaissance
  ImagingParameters,
  SpectralBand,
  ImagingRequest,
  SARParameters,

  // Navigation
  NavigationSignal,
  PositionSolution,

  // Early warning
  IRDetection,
  BallisticTrajectory,

  // Ground control
  GroundStation,
  Antenna,
  TelemetryData,

  // Security
  EncryptionKey,
  EncryptedMessage,

  // Tracking
  TrackingRequest,
  SatellitePosition,

  // Mission planning
  TaskingRequest,
  CollectionPlan,
  CollectionWindow,
};

export { SATELLITE_CONSTANTS, SatelliteErrorCode, SatelliteError };
