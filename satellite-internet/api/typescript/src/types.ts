/**
 * WIA-COMM-005: Satellite Internet - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communications Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Orbital Types
// ============================================================================

/**
 * Orbit classification
 */
export type OrbitType = 'LEO' | 'MEO' | 'GEO' | 'HEO';

/**
 * Frequency band classification
 */
export type FrequencyBand = 'L' | 'S' | 'C' | 'X' | 'Ku' | 'Ka' | 'V' | 'W' | 'Q';

/**
 * Link type classification
 */
export type LinkType = 'user-uplink' | 'user-downlink' | 'gateway-uplink' | 'gateway-downlink' | 'isl-optical' | 'isl-rf';

/**
 * Modulation scheme (DVB-S2X)
 */
export type ModulationType = 'QPSK' | '8PSK' | '16APSK' | '32APSK' | '64APSK' | '128APSK' | '256APSK';

/**
 * Forward error correction code rate
 */
export type CodeRate = '1/4' | '1/3' | '2/5' | '1/2' | '3/5' | '2/3' | '3/4' | '4/5' | '5/6' | '8/9' | '9/10';

/**
 * Polarization type
 */
export type Polarization = 'linear-horizontal' | 'linear-vertical' | 'circular-left' | 'circular-right' | 'dual';

// ============================================================================
// Geographic and Coordinate Types
// ============================================================================

/**
 * Geographic coordinate (WGS84)
 */
export interface GeoCoordinate {
  /** Latitude in degrees (-90 to +90) */
  latitude: number;

  /** Longitude in degrees (-180 to +180) */
  longitude: number;

  /** Altitude in meters above sea level */
  altitude?: number;
}

/**
 * Three-dimensional vector
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Orbital elements (Keplerian)
 */
export interface OrbitalElements {
  /** Semi-major axis in km */
  semiMajorAxis: number;

  /** Eccentricity (0 = circular, 0-1 = elliptical) */
  eccentricity: number;

  /** Inclination in degrees (0-180) */
  inclination: number;

  /** Right ascension of ascending node in degrees (0-360) */
  raan: number;

  /** Argument of perigee in degrees (0-360) */
  argumentOfPerigee: number;

  /** True anomaly in degrees (0-360) */
  trueAnomaly: number;

  /** Epoch time (ISO 8601) */
  epoch: Date | string;
}

// ============================================================================
// Satellite and Constellation Types
// ============================================================================

/**
 * Satellite configuration
 */
export interface Satellite {
  /** Unique satellite identifier */
  id: string;

  /** Satellite name */
  name?: string;

  /** Orbital elements */
  orbit: OrbitalElements;

  /** Orbit type classification */
  orbitType: OrbitType;

  /** Status */
  status: 'active' | 'standby' | 'deorbiting' | 'decommissioned';

  /** Antenna configuration */
  antennas?: AntennaConfig[];

  /** Transponders */
  transponders?: Transponder[];

  /** Inter-satellite link capability */
  islCapable: boolean;

  /** Launch date */
  launchDate?: Date | string;

  /** End-of-life date */
  endOfLife?: Date | string;
}

/**
 * Antenna configuration
 */
export interface AntennaConfig {
  /** Antenna type */
  type: 'phased-array' | 'reflector' | 'horn';

  /** Frequency band */
  band: FrequencyBand;

  /** Gain in dBi */
  gain: number;

  /** Beamwidth in degrees */
  beamwidth?: number;

  /** Polarization */
  polarization: Polarization;

  /** Number of beams (for phased array) */
  numBeams?: number;
}

/**
 * Transponder configuration
 */
export interface Transponder {
  /** Transponder ID */
  id: string;

  /** Uplink frequency in GHz */
  uplinkFrequency: number;

  /** Downlink frequency in GHz */
  downlinkFrequency: number;

  /** Bandwidth in MHz */
  bandwidth: number;

  /** EIRP in dBW */
  eirp: number;

  /** G/T ratio in dB/K */
  gainToNoiseTemp?: number;
}

/**
 * Satellite constellation configuration
 */
export interface ConstellationConfig {
  /** Constellation name */
  name: string;

  /** Orbit type */
  constellation: OrbitType;

  /** Altitude in km */
  altitude: number;

  /** Inclination in degrees */
  inclination: number;

  /** Number of orbital planes */
  orbitalPlanes: number;

  /** Satellites per plane */
  satellitesPerPlane: number;

  /** Total satellites in constellation */
  totalSatellites: number;

  /** Phase offset between planes in degrees */
  phaseOffset?: number;

  /** Inter-satellite link topology */
  islTopology?: 'mesh' | 'ring' | 'star' | 'none';
}

// ============================================================================
// Link Budget Types
// ============================================================================

/**
 * Link budget parameters
 */
export interface LinkBudgetParams {
  /** Distance in km (satellite to ground) */
  distance: number;

  /** Carrier frequency in GHz */
  frequency: number;

  /** Transmit power in watts */
  txPower: number;

  /** Transmit antenna gain in dBi */
  txGain?: number;

  /** Receive antenna gain in dBi */
  rxGain?: number;

  /** System noise temperature in Kelvin */
  noiseTemp?: number;

  /** Bandwidth in MHz */
  bandwidth: number;

  /** Atmospheric losses in dB */
  atmosphericLoss?: number;

  /** Rain fade margin in dB */
  rainFade?: number;

  /** Polarization mismatch loss in dB */
  polarizationLoss?: number;

  /** Pointing loss in dB */
  pointingLoss?: number;
}

/**
 * Link budget calculation result
 */
export interface LinkBudgetResult {
  /** Effective isotropic radiated power in dBW */
  eirp: number;

  /** Free-space path loss in dB */
  pathLoss: number;

  /** Total losses in dB */
  totalLoss: number;

  /** Received power in dBm */
  receivedPower: number;

  /** Noise power in dBm */
  noisePower: number;

  /** Carrier-to-noise ratio in dB */
  cnr: number;

  /** Signal-to-noise ratio in dB */
  snr: number;

  /** Theoretical capacity in Mbps (Shannon limit) */
  capacity: number;

  /** Link margin in dB */
  linkMargin: number;

  /** Link feasibility */
  feasible: boolean;
}

// ============================================================================
// Handover and Tracking Types
// ============================================================================

/**
 * Satellite visibility prediction
 */
export interface SatelliteVisibility {
  /** Satellite ID */
  satelliteId: string;

  /** Acquisition time */
  acquisitionTime: Date;

  /** Loss of signal time */
  lossOfSignalTime: Date;

  /** Visibility duration in seconds */
  duration: number;

  /** Maximum elevation angle in degrees */
  maxElevation: number;

  /** Azimuth at acquisition in degrees */
  azimuthAcquisition: number;

  /** Azimuth at LOS in degrees */
  azimuthLoss: number;

  /** Range at closest approach in km */
  minRange: number;

  /** Doppler shift at closest approach in Hz */
  maxDoppler: number;
}

/**
 * Handover prediction
 */
export interface HandoverPrediction {
  /** Current satellite ID */
  currentSatelliteId: string;

  /** Next satellite ID */
  nextSatelliteId: string;

  /** Time until handover in seconds */
  timeRemaining: number;

  /** Handover trigger time */
  handoverTime: Date;

  /** Handover type */
  handoverType: 'intra-plane' | 'inter-plane' | 'gateway';

  /** Expected interruption duration in milliseconds */
  interruptionDuration: number;

  /** Signal quality of next satellite */
  nextSatelliteQuality: number;
}

/**
 * Doppler shift calculation
 */
export interface DopplerShift {
  /** Carrier frequency in Hz */
  carrierFrequency: number;

  /** Doppler shift in Hz */
  dopplerShift: number;

  /** Doppler rate in Hz/s */
  dopplerRate: number;

  /** Corrected frequency in Hz */
  correctedFrequency: number;
}

// ============================================================================
// User Terminal Types
// ============================================================================

/**
 * User terminal configuration
 */
export interface UserTerminal {
  /** Terminal ID */
  id: string;

  /** Terminal type */
  type: 'fixed' | 'mobile' | 'maritime' | 'aviation';

  /** Geographic location */
  location: GeoCoordinate;

  /** Antenna configuration */
  antenna: AntennaConfig;

  /** Transmit power in watts */
  txPower: number;

  /** Receiver noise figure in dB */
  noiseFigure: number;

  /** Minimum elevation angle in degrees */
  minElevation: number;

  /** Maximum EIRP in dBW */
  maxEirp: number;

  /** Supported frequency bands */
  supportedBands: FrequencyBand[];

  /** Supported modulation schemes */
  supportedModulations: ModulationType[];
}

/**
 * Terminal performance metrics
 */
export interface TerminalMetrics {
  /** Download speed in Mbps */
  downloadSpeed: number;

  /** Upload speed in Mbps */
  uploadSpeed: number;

  /** Round-trip latency in ms */
  latency: number;

  /** Packet loss percentage */
  packetLoss: number;

  /** Jitter in ms */
  jitter: number;

  /** Current SNR in dB */
  currentSnr: number;

  /** Current satellite ID */
  currentSatelliteId?: string;

  /** Number of satellites visible */
  visibleSatellites: number;

  /** Handovers in last hour */
  handoversLastHour: number;
}

// ============================================================================
// Ground Station Types
// ============================================================================

/**
 * Ground station (gateway) configuration
 */
export interface GroundStation {
  /** Station ID */
  id: string;

  /** Station name */
  name: string;

  /** Geographic location */
  location: GeoCoordinate;

  /** Antenna diameter in meters */
  antennaDiameter: number;

  /** Frequency bands supported */
  supportedBands: FrequencyBand[];

  /** Maximum data rate in Gbps */
  maxDataRate: number;

  /** Number of simultaneous satellite links */
  maxSimultaneousLinks: number;

  /** Internet backbone capacity in Gbps */
  backboneCapacity: number;

  /** Status */
  status: 'active' | 'standby' | 'maintenance' | 'offline';
}

// ============================================================================
// Inter-Satellite Link Types
// ============================================================================

/**
 * Inter-satellite link configuration
 */
export interface ISLConfig {
  /** Link type */
  type: 'optical' | 'rf';

  /** Source satellite ID */
  sourceSatelliteId: string;

  /** Destination satellite ID */
  destinationSatelliteId: string;

  /** Link capacity in Gbps */
  capacity: number;

  /** Link distance in km */
  distance: number;

  /** Link latency in ms */
  latency: number;

  /** Link quality (0-1) */
  quality: number;

  /** Optical wavelength in nm (for optical links) */
  wavelength?: number;

  /** RF frequency in GHz (for RF links) */
  frequency?: number;
}

/**
 * Mesh network topology
 */
export interface MeshTopology {
  /** Constellation ID */
  constellationId: string;

  /** Active ISL links */
  links: ISLConfig[];

  /** Total mesh capacity in Tbps */
  totalCapacity: number;

  /** Average mesh latency in ms */
  averageLatency: number;

  /** Mesh resilience (0-1) */
  resilience: number;
}

// ============================================================================
// Regulatory and Compliance Types
// ============================================================================

/**
 * ITU frequency filing
 */
export interface ITUFiling {
  /** Filing ID */
  filingId: string;

  /** Administering country */
  country: string;

  /** Frequency band */
  band: FrequencyBand;

  /** Uplink frequency range in GHz */
  uplinkRange: [number, number];

  /** Downlink frequency range in GHz */
  downlinkRange: [number, number];

  /** Filing status */
  status: 'advance-publication' | 'coordination' | 'notification' | 'recorded';

  /** Filing date */
  filingDate: Date | string;

  /** Coordination expiry */
  expiryDate?: Date | string;
}

/**
 * Orbital debris compliance
 */
export interface DebrisCompliance {
  /** Satellite ID */
  satelliteId: string;

  /** Deorbit strategy */
  deorbitStrategy: 'active' | 'passive' | 'graveyard';

  /** Expected deorbit time after end-of-life (years) */
  deorbitTimeframe: number;

  /** Collision avoidance maneuvers performed */
  collisionAvoidanceManeuvers: number;

  /** Propellant remaining (%) */
  propellantRemaining: number;

  /** Compliance with 25-year rule */
  compliant: boolean;
}

// ============================================================================
// Error and Validation Types
// ============================================================================

/**
 * Error codes for satellite operations
 */
export enum SatelliteErrorCode {
  INVALID_ORBIT = 'INVALID_ORBIT',
  INSUFFICIENT_LINK_BUDGET = 'INSUFFICIENT_LINK_BUDGET',
  NO_SATELLITE_VISIBLE = 'NO_SATELLITE_VISIBLE',
  HANDOVER_FAILED = 'HANDOVER_FAILED',
  FREQUENCY_OUT_OF_BAND = 'FREQUENCY_OUT_OF_BAND',
  DEBRIS_COLLISION_RISK = 'DEBRIS_COLLISION_RISK',
  REGULATORY_VIOLATION = 'REGULATORY_VIOLATION',
  INSUFFICIENT_CAPACITY = 'INSUFFICIENT_CAPACITY',
  INVALID_PARAMETERS = 'INVALID_PARAMETERS',
  SYSTEM_ERROR = 'SYSTEM_ERROR',
}

/**
 * Satellite operation error
 */
export class SatelliteError extends Error {
  constructor(
    public code: SatelliteErrorCode,
    message: string,
    public details?: any
  ) {
    super(message);
    this.name = 'SatelliteError';
  }
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical and satellite communication constants
 */
export const SATELLITE_CONSTANTS = {
  /** Speed of light in m/s */
  SPEED_OF_LIGHT: 299792458,

  /** Speed of light in km/s */
  SPEED_OF_LIGHT_KMS: 299792.458,

  /** Earth radius in km */
  EARTH_RADIUS: 6371,

  /** Earth mass in kg */
  EARTH_MASS: 5.972e24,

  /** Gravitational constant in m³/kg·s² */
  GRAVITATIONAL_CONSTANT: 6.674e-11,

  /** Standard gravitational parameter (GM) in km³/s² */
  MU_EARTH: 398600.4418,

  /** Boltzmann constant in J/K */
  BOLTZMANN: 1.380649e-23,

  /** GEO altitude in km */
  GEO_ALTITUDE: 35786,

  /** LEO altitude range in km */
  LEO_ALTITUDE_MIN: 500,
  LEO_ALTITUDE_MAX: 2000,

  /** MEO altitude range in km */
  MEO_ALTITUDE_MIN: 2000,
  MEO_ALTITUDE_MAX: 35786,

  /** Standard atmospheric temperature in K */
  STANDARD_TEMP: 290,

  /** Typical user terminal noise temperature in K */
  USER_TERMINAL_NOISE_TEMP: 250,

  /** Typical satellite noise temperature in K */
  SATELLITE_NOISE_TEMP: 500,
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Time-series data point
 */
export interface TimeSeriesPoint<T> {
  timestamp: Date | string;
  value: T;
}

/**
 * Simulation result
 */
export interface SimulationResult {
  /** Simulation start time */
  startTime: Date;

  /** Simulation end time */
  endTime: Date;

  /** Simulation duration in seconds */
  duration: number;

  /** Number of data points */
  dataPoints: number;

  /** Simulation success */
  success: boolean;

  /** Warnings encountered */
  warnings: string[];

  /** Errors encountered */
  errors: string[];
}
