/**
 * WIA-COMM-004: 5G/6G Spectrum - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Standards Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Frequency Ranges and Bands
// ============================================================================

/**
 * 3GPP Frequency Range definitions
 */
export type FrequencyRange = 'FR1' | 'FR2' | 'FR3' | 'THz';

/**
 * Duplex mode
 */
export type DuplexMode = 'FDD' | 'TDD' | 'SDL' | 'SUL';

/**
 * 5G NR band definition
 */
export interface NRBand {
  /** Band number (e.g., n77, n78, n257) */
  band: string;

  /** Frequency range */
  range: FrequencyRange;

  /** Duplex mode */
  duplexMode: DuplexMode;

  /** Uplink frequency range (MHz) */
  uplinkFrequency?: {
    min: number;
    max: number;
  };

  /** Downlink frequency range (MHz) */
  downlinkFrequency: {
    min: number;
    max: number;
  };

  /** Total bandwidth available (MHz) */
  totalBandwidth: number;

  /** Operating class */
  operatingClass?: string;

  /** Geographic region(s) */
  regions: string[];
}

/**
 * Spectrum allocation
 */
export interface SpectrumAllocation {
  /** Allocation ID */
  id: string;

  /** Frequency band */
  band: NRBand;

  /** Center frequency (MHz) */
  centerFrequency: number;

  /** Channel bandwidth (MHz) */
  bandwidth: number;

  /** Operator/licensee */
  licensee?: string;

  /** License type */
  licenseType: 'exclusive' | 'shared' | 'unlicensed' | 'local';

  /** Geographic coverage area */
  coverageArea?: {
    type: 'national' | 'regional' | 'local' | 'custom';
    coordinates?: Array<{ latitude: number; longitude: number }>;
    radius?: number; // km
  };

  /** License expiry date */
  expiryDate?: Date;

  /** Maximum transmit power (dBm) */
  maxTransmitPower: number;
}

// ============================================================================
// Dynamic Spectrum Access (DSA)
// ============================================================================

/**
 * Spectrum sharing tier (CBRS model)
 */
export type SpectrumTier = 'Incumbent' | 'PAL' | 'GAA';

/**
 * CBRS Priority Access License
 */
export interface CBRSLicense {
  /** License ID */
  id: string;

  /** Tier level */
  tier: 'PAL' | 'GAA';

  /** Frequency assignment (MHz) */
  frequencyRange: {
    min: number;
    max: number;
  };

  /** Geographic license area */
  licenseArea: {
    censusTracts: string[];
    counties: string[];
  };

  /** License term */
  term: {
    start: Date;
    end: Date;
    renewable: boolean;
  };

  /** CBSD (base station) registration */
  cbsdRegistration?: CBSDRegistration;
}

/**
 * CBSD (Citizens Broadband Service Device) Registration
 */
export interface CBSDRegistration {
  /** CBSD identifier */
  cbsdId: string;

  /** User/operator ID */
  userId: string;

  /** FCC ID */
  fccId: string;

  /** CBSD category (A=indoor, B=outdoor) */
  cbsdCategory: 'A' | 'B';

  /** Installation location */
  location: {
    latitude: number;
    longitude: number;
    height: number; // meters
    heightType: 'AGL' | 'AMSL';
    horizontalAccuracy: number; // meters
    indoorDeployment: boolean;
  };

  /** Antenna characteristics */
  antenna: {
    gain: number; // dBi
    azimuth?: number; // degrees
    beamwidth?: number; // degrees
    downtilt?: number; // degrees
  };

  /** Requested transmit power */
  requestedPower: number; // dBm EIRP

  /** Approved grant */
  grant?: SpectrumGrant;
}

/**
 * Spectrum grant from SAS
 */
export interface SpectrumGrant {
  /** Grant ID */
  grantId: string;

  /** Granted frequency range (MHz) */
  frequencyRange: {
    min: number;
    max: number;
  };

  /** Maximum EIRP (dBm) */
  maxEirp: number;

  /** Grant expiration */
  grantExpireTime: Date;

  /** Heartbeat interval (seconds) */
  heartbeatInterval: number;

  /** Channel type */
  channelType: 'PAL' | 'GAA';
}

/**
 * DSA spectrum query
 */
export interface DSASpectrumQuery {
  /** Device descriptor */
  device: {
    serialNumber: string;
    manufacturer: string;
    model: string;
    fccId?: string;
  };

  /** Query location */
  location: {
    latitude: number;
    longitude: number;
    height?: number;
    uncertainty?: number; // meters
  };

  /** Antenna configuration */
  antenna: {
    gain: number; // dBi
    azimuth?: number;
    beamwidth?: number;
  };

  /** Requested bandwidth (MHz) */
  requestedBandwidth?: number;

  /** Requested tier */
  requestedTier?: SpectrumTier;
}

/**
 * DSA spectrum response
 */
export interface DSASpectrumResponse {
  /** Available channels */
  availableChannels: Array<{
    frequencyRange: {
      low: number;
      high: number;
    };
    maxEirp: number;
    ruleApplied: 'PAL' | 'GAA' | 'Licensed';
  }>;

  /** Response timestamp */
  timestamp: Date;

  /** Validity period */
  validUntil: Date;

  /** Nearby incumbents (if any) */
  incumbents?: Array<{
    type: string;
    distance: number;
    exclusionZone: number;
  }>;
}

// ============================================================================
// Carrier Configuration
// ============================================================================

/**
 * Subcarrier spacing (numerology)
 */
export type SubcarrierSpacing = 15 | 30 | 60 | 120 | 240; // kHz

/**
 * Modulation scheme
 */
export type Modulation = 'QPSK' | '16-QAM' | '64-QAM' | '256-QAM' | '1024-QAM';

/**
 * Carrier configuration
 */
export interface CarrierConfig {
  /** Carrier ID */
  id: string;

  /** NR band */
  band: string;

  /** Center frequency (MHz) */
  centerFrequency: number;

  /** Bandwidth (MHz) */
  bandwidth: number;

  /** Subcarrier spacing (kHz) */
  subcarrierSpacing: SubcarrierSpacing;

  /** Duplex mode */
  duplexMode: DuplexMode;

  /** TDD pattern (if TDD) */
  tddPattern?: string; // e.g., "DDDSU", "DSUUU"

  /** TDD periodicity (ms) */
  tddPeriodicity?: number;

  /** MIMO configuration */
  mimo: MIMOConfig;

  /** Maximum transmit power (dBm) */
  maxTxPower: number;

  /** Modulation and coding scheme */
  mcs?: number; // 0-31

  /** Number of PRBs (Physical Resource Blocks) */
  numPRBs: number;
}

/**
 * MIMO configuration
 */
export interface MIMOConfig {
  /** Number of transmit antennas */
  txAntennas: number;

  /** Number of receive antennas */
  rxAntennas: number;

  /** Maximum MIMO layers */
  maxLayers: number;

  /** MIMO type */
  type: 'SU-MIMO' | 'MU-MIMO' | 'Massive-MIMO';

  /** Beamforming enabled */
  beamforming: boolean;

  /** Number of beams (if beamforming) */
  numBeams?: number;

  /** Beam width (degrees) */
  beamWidth?: number;
}

/**
 * Carrier aggregation configuration
 */
export interface CarrierAggregationConfig {
  /** Aggregation ID */
  id: string;

  /** Primary component carrier */
  primaryCarrier: CarrierConfig;

  /** Secondary component carriers */
  secondaryCarriers: CarrierConfig[];

  /** CA type */
  type: 'intra-band-contiguous' | 'intra-band-non-contiguous' | 'inter-band';

  /** Total aggregated bandwidth (MHz) */
  totalBandwidth: number;

  /** Peak throughput estimate (Mbps) */
  peakThroughput: number;

  /** Spectral efficiency (bps/Hz) */
  spectralEfficiency: number;
}

// ============================================================================
// Beamforming
// ============================================================================

/**
 * Beamforming type
 */
export type BeamformingType = 'analog' | 'digital' | 'hybrid';

/**
 * Beam pattern
 */
export interface BeamPattern {
  /** Beam ID */
  beamId: string;

  /** Azimuth angle (degrees) */
  azimuth: number;

  /** Elevation angle (degrees) */
  elevation: number;

  /** Beam width (degrees) */
  beamWidth: number;

  /** Main lobe gain (dBi) */
  mainLobeGain: number;

  /** Side lobe level (dB) */
  sideLobeLevel: number;

  /** Front-to-back ratio (dB) */
  frontToBackRatio: number;
}

/**
 * Beamforming configuration
 */
export interface BeamformingConfig {
  /** Beamforming type */
  type: BeamformingType;

  /** Number of antenna elements */
  numElements: number;

  /** Array configuration */
  arrayConfig: {
    rows: number;
    columns: number;
    polarization: 'single' | 'dual' | 'cross';
    spacing: number; // wavelengths
  };

  /** Active beam patterns */
  beamPatterns: BeamPattern[];

  /** Beam management */
  beamManagement: {
    sweepPeriod: number; // ms
    trackingEnabled: boolean;
    refinementEnabled: boolean;
  };

  /** Spatial multiplexing gain (dB) */
  spatialGain: number;
}

// ============================================================================
// Interference Management
// ============================================================================

/**
 * Interference type
 */
export type InterferenceType =
  | 'co-channel'
  | 'adjacent-channel'
  | 'intermodulation'
  | 'spurious';

/**
 * Interference measurement
 */
export interface InterferenceMeasurement {
  /** Measurement ID */
  id: string;

  /** Interference type */
  type: InterferenceType;

  /** Frequency (MHz) */
  frequency: number;

  /** Interference power (dBm) */
  power: number;

  /** Signal-to-interference ratio (dB) */
  sir?: number;

  /** Signal-to-interference-plus-noise ratio (dB) */
  sinr?: number;

  /** Measurement timestamp */
  timestamp: Date;

  /** Location of measurement */
  location?: {
    latitude: number;
    longitude: number;
  };

  /** Suspected interferer */
  interferer?: {
    type: string;
    distance?: number;
    bearing?: number;
  };
}

/**
 * Interference analysis
 */
export interface InterferenceAnalysis {
  /** Analysis ID */
  id: string;

  /** Target system */
  targetSystem: {
    band: string;
    frequency: number;
    location: { latitude: number; longitude: number };
    transmitPower: number;
    antennaHeight: number;
  };

  /** Co-channel interference (dB) */
  coChannelInterference: number;

  /** Adjacent channel interference (dB) */
  adjacentChannelInterference: number;

  /** Aggregate interference (dBm) */
  aggregateInterference: number;

  /** Protection ratio (dB) */
  protectionRatio: number;

  /** Meets protection criteria */
  meetsProtection: boolean;

  /** Recommended mitigation */
  mitigation?: string[];
}

// ============================================================================
// Link Budget and Coverage
// ============================================================================

/**
 * Link budget calculation
 */
export interface LinkBudget {
  /** Transmitter parameters */
  transmitter: {
    power: number; // dBm
    antennaGain: number; // dBi
    cableLoss: number; // dB
    eirp: number; // dBm
  };

  /** Path loss */
  pathLoss: {
    freeSpace: number; // dB
    shadowFading: number; // dB
    buildingPenetration: number; // dB
    foliageLoss: number; // dB
    total: number; // dB
  };

  /** Receiver parameters */
  receiver: {
    antennaGain: number; // dBi
    cableLoss: number; // dB
    noiseFigure: number; // dB
    noisePower: number; // dBm
    sensitivity: number; // dBm
  };

  /** Margin */
  margin: {
    required: number; // dB
    interference: number; // dB
    fade: number; // dB
    total: number; // dB
  };

  /** Link closes? */
  linkCloses: boolean;

  /** Excess margin (dB) */
  excessMargin: number;
}

/**
 * Coverage prediction
 */
export interface CoveragePrediction {
  /** Prediction ID */
  id: string;

  /** Cell site */
  site: {
    latitude: number;
    longitude: number;
    height: number; // meters
    antennaType: string;
  };

  /** Carrier configuration */
  carrier: CarrierConfig;

  /** Coverage radius (km) */
  coverageRadius: number;

  /** Coverage area (km²) */
  coverageArea: number;

  /** Edge SINR (dB) */
  edgeSINR: number;

  /** Cell capacity (Mbps) */
  cellCapacity: number;

  /** Users per cell */
  usersPerCell: number;
}

// ============================================================================
// Spectrum Efficiency Metrics
// ============================================================================

/**
 * Spectrum efficiency metrics
 */
export interface SpectrumEfficiency {
  /** Modulation */
  modulation: Modulation;

  /** Code rate */
  codeRate: number;

  /** MIMO layers */
  mimoLayers: number;

  /** Bandwidth (MHz) */
  bandwidth: number;

  /** Overhead factor */
  overhead: number;

  /** Theoretical spectral efficiency (bps/Hz) */
  theoreticalEfficiency: number;

  /** Practical spectral efficiency (bps/Hz) */
  practicalEfficiency: number;

  /** Peak throughput (Mbps) */
  peakThroughput: number;

  /** Average throughput (Mbps) */
  averageThroughput: number;

  /** Required SINR (dB) */
  requiredSINR: number;
}

// ============================================================================
// Regulatory Compliance
// ============================================================================

/**
 * Regulatory region
 */
export type RegulatoryRegion =
  | 'US'
  | 'EU'
  | 'CN'
  | 'KR'
  | 'JP'
  | 'IN'
  | 'AU'
  | 'BR'
  | 'Global';

/**
 * Regulatory compliance check
 */
export interface RegulatoryCompliance {
  /** Region */
  region: RegulatoryRegion;

  /** Band */
  band: string;

  /** Frequency (MHz) */
  frequency: number;

  /** Transmit power (dBm) */
  transmitPower: number;

  /** Compliant? */
  compliant: boolean;

  /** Maximum allowed power (dBm) */
  maxAllowedPower: number;

  /** Applicable regulations */
  regulations: string[];

  /** Restrictions */
  restrictions?: string[];

  /** License required? */
  licenseRequired: boolean;

  /** Coordination required? */
  coordinationRequired?: boolean;
}

// ============================================================================
// Performance Monitoring
// ============================================================================

/**
 * Spectrum utilization
 */
export interface SpectrumUtilization {
  /** Band */
  band: string;

  /** Frequency range (MHz) */
  frequencyRange: {
    min: number;
    max: number;
  };

  /** Time period */
  timePeriod: {
    start: Date;
    end: Date;
  };

  /** Average utilization (%) */
  averageUtilization: number;

  /** Peak utilization (%) */
  peakUtilization: number;

  /** Utilization by hour */
  hourlyUtilization: Array<{
    hour: number;
    utilization: number;
  }>;

  /** Interference events */
  interferenceEvents: number;
}

/**
 * Network performance metrics
 */
export interface NetworkPerformance {
  /** Timestamp */
  timestamp: Date;

  /** Cell ID */
  cellId: string;

  /** Band */
  band: string;

  /** Active users */
  activeUsers: number;

  /** Throughput (Mbps) */
  throughput: {
    downlink: number;
    uplink: number;
  };

  /** Latency (ms) */
  latency: {
    average: number;
    p50: number;
    p95: number;
    p99: number;
  };

  /** Spectral efficiency (bps/Hz) */
  spectralEfficiency: number;

  /** Resource utilization (%) */
  resourceUtilization: {
    prbs: number;
    power: number;
  };

  /** SINR distribution */
  sinrDistribution: {
    average: number;
    cellEdge: number; // 5th percentile
    median: number;
  };
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-COMM-004 error codes
 */
export enum SpectrumErrorCode {
  INVALID_FREQUENCY = 'SPEC001',
  INVALID_BANDWIDTH = 'SPEC002',
  BAND_NOT_SUPPORTED = 'SPEC003',
  INTERFERENCE_DETECTED = 'SPEC004',
  LICENSE_REQUIRED = 'SPEC005',
  REGULATORY_VIOLATION = 'SPEC006',
  POWER_EXCEEDS_LIMIT = 'SPEC007',
  COORDINATION_REQUIRED = 'SPEC008',
  SPECTRUM_UNAVAILABLE = 'SPEC009',
  GRANT_EXPIRED = 'SPEC010',
  INVALID_CONFIGURATION = 'SPEC011',
  COEXISTENCE_VIOLATION = 'SPEC012',
}

/**
 * Spectrum management error
 */
export class SpectrumError extends Error {
  constructor(
    public code: SpectrumErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'SpectrumError';
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
 * Geolocation
 */
export interface GeoLocation {
  latitude: number;
  longitude: number;
  altitude?: number;
  accuracy?: number;
}

/**
 * Time range
 */
export interface TimeRange {
  start: Date;
  end: Date;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for spectrum calculations
 */
export const SPECTRUM_CONSTANTS = {
  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Boltzmann constant (J/K) */
  BOLTZMANN: 1.380649e-23,

  /** Temperature (K) */
  TEMPERATURE: 290,

  /** Thermal noise density (dBm/Hz) */
  THERMAL_NOISE_DENSITY: -174,

  /** Maximum EIRP for UE (dBm) */
  MAX_EIRP_UE: 23,

  /** Maximum EIRP for BS (dBm) */
  MAX_EIRP_BS: 68,

  /** CBRS GAA max EIRP (dBm) */
  CBRS_GAA_MAX_EIRP_OUTDOOR: 30,
  CBRS_GAA_MAX_EIRP_INDOOR: 24,

  /** Standard atmosphere pressure (Pa) */
  STANDARD_PRESSURE: 101325,

  /** Water vapor density (g/m³) */
  WATER_VAPOR_DENSITY: 7.5,
} as const;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  FrequencyRange,
  DuplexMode,
  NRBand,
  SpectrumAllocation,
  SpectrumTier,
  CBRSLicense,
  CBSDRegistration,
  SpectrumGrant,
  DSASpectrumQuery,
  DSASpectrumResponse,
  SubcarrierSpacing,
  Modulation,
  CarrierConfig,
  MIMOConfig,
  CarrierAggregationConfig,
  BeamformingType,
  BeamPattern,
  BeamformingConfig,
  InterferenceType,
  InterferenceMeasurement,
  InterferenceAnalysis,
  LinkBudget,
  CoveragePrediction,
  SpectrumEfficiency,
  RegulatoryRegion,
  RegulatoryCompliance,
  SpectrumUtilization,
  NetworkPerformance,
  GeoLocation,
  TimeRange,
};

export { SPECTRUM_CONSTANTS, SpectrumErrorCode, SpectrumError };

/**
 * 弘益人間 (Benefit All Humanity)
 */
