/**
 * WIA-DEF-011: Reconnaissance Satellite - TypeScript Type Definitions
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
 * Sensor type classification
 */
export type SensorType = 'optical' | 'sar' | 'ir' | 'hyperspectral' | 'sigint';

/**
 * SAR band classification
 */
export type SARBand = 'x-band' | 'c-band' | 'l-band' | 's-band';

/**
 * SAR imaging mode
 */
export type SARMode = 'stripmap' | 'spotlight' | 'scansar' | 'gmti' | 'interferometric';

/**
 * Image processing level
 */
export type ProcessingLevel = 0 | 1 | 2 | 3 | 4;

/**
 * Data format type
 */
export type DataFormat = 'geotiff' | 'nitf' | 'hdf' | 'jpeg2000' | 'png' | 'raw';

/**
 * Polarization mode
 */
export type Polarization = 'VV' | 'HH' | 'VH' | 'HV' | 'quad-pol';

// ============================================================================
// Satellite and Sensor Configuration
// ============================================================================

/**
 * Satellite orbital parameters
 */
export interface OrbitalParameters {
  /** Semi-major axis in meters */
  semiMajorAxis: number;

  /** Orbital eccentricity (0-1) */
  eccentricity: number;

  /** Orbital inclination in degrees */
  inclination: number;

  /** Right ascension of ascending node in degrees */
  raan: number;

  /** Argument of perigee in degrees */
  argumentOfPerigee: number;

  /** Mean anomaly in degrees */
  meanAnomaly: number;

  /** Epoch time */
  epoch: Date;
}

/**
 * Optical sensor configuration
 */
export interface OpticalSensorConfig {
  /** Sensor type */
  type: 'optical' | 'hyperspectral';

  /** Focal length in meters */
  focalLength: number;

  /** Pixel pitch (detector size) in meters */
  pixelPitch: number;

  /** Detector array size */
  arraySize: {
    width: number;
    height: number;
  };

  /** Spectral bands */
  spectralBands: SpectralBand[];

  /** Dynamic range in bits */
  dynamicRange: number;

  /** Signal-to-noise ratio */
  snr: number;

  /** Modulation transfer function at Nyquist */
  mtf: number;
}

/**
 * Spectral band definition
 */
export interface SpectralBand {
  /** Band identifier */
  id: string;

  /** Band name (e.g., "Red", "NIR") */
  name: string;

  /** Center wavelength in micrometers */
  centerWavelength: number;

  /** Bandwidth in micrometers */
  bandwidth: number;

  /** Band purpose/application */
  purpose: string;
}

/**
 * SAR sensor configuration
 */
export interface SARSensorConfig {
  /** Sensor type */
  type: 'sar';

  /** SAR frequency band */
  band: SARBand;

  /** Center frequency in Hz */
  frequency: number;

  /** Wavelength in meters */
  wavelength: number;

  /** Bandwidth in Hz */
  bandwidth: number;

  /** Antenna length in meters */
  antennaLength: number;

  /** Antenna width in meters */
  antennaWidth: number;

  /** Polarization modes */
  polarizations: Polarization[];

  /** Imaging modes */
  modes: SARMode[];

  /** Peak transmit power in watts */
  peakPower: number;

  /** Pulse repetition frequency in Hz */
  prf: number;
}

/**
 * SIGINT sensor configuration
 */
export interface SIGINTSensorConfig {
  /** Sensor type */
  type: 'sigint';

  /** SIGINT subtype */
  subtype: 'comint' | 'elint' | 'fisint';

  /** Frequency coverage ranges */
  frequencyRanges: Array<{
    min: number;  // Hz
    max: number;  // Hz
    band: string;
  }>;

  /** Antenna configuration */
  antennas: Array<{
    type: string;
    diameter?: number;
    gain: number;
    beamwidth: number;
  }>;

  /** Instantaneous bandwidth in Hz */
  instantaneousBandwidth: number;

  /** Minimum detectable signal in dBm */
  sensitivity: number;

  /** Geolocation accuracy in meters */
  geolocationAccuracy: number;
}

// ============================================================================
// Ground Resolution and Image Quality
// ============================================================================

/**
 * Ground resolution calculation parameters
 */
export interface GroundResolutionParams {
  /** Orbital altitude in meters */
  altitude: number;

  /** Focal length in meters (for optical) */
  focalLength?: number;

  /** Pixel pitch in meters (for optical) */
  pixelPitch?: number;

  /** Off-nadir angle in degrees */
  offNadirAngle?: number;

  /** Sensor type */
  sensorType: SensorType;

  /** SAR bandwidth in Hz (for SAR) */
  sarBandwidth?: number;

  /** SAR antenna length in meters (for SAR) */
  sarAntennaLength?: number;

  /** Modulation transfer function */
  mtf?: number;
}

/**
 * Ground resolution calculation result
 */
export interface GroundResolutionResult {
  /** Ground sample distance in meters */
  gsd: number;

  /** Spatial resolution in meters (accounting for MTF) */
  spatialResolution: number;

  /** NIIRS rating (0-9) */
  niirs: number;

  /** Swath width in meters */
  swathWidth: number;

  /** Field of view in degrees */
  fieldOfView: number;

  /** Feasibility rating */
  feasibility: 'excellent' | 'good' | 'acceptable' | 'poor';

  /** For SAR: range and azimuth resolution */
  sarResolution?: {
    range: number;
    azimuth: number;
  };
}

/**
 * Image quality metrics
 */
export interface ImageQualityMetrics {
  /** NIIRS rating */
  niirs: number;

  /** Signal-to-noise ratio */
  snr: number;

  /** Relative edge response */
  rer: number;

  /** Modulation transfer function */
  mtf: number;

  /** Cloud cover percentage */
  cloudCover: number;

  /** Contrast */
  contrast: number;

  /** Sharpness metric */
  sharpness: number;

  /** Overall quality score (0-100) */
  qualityScore: number;
}

// ============================================================================
// Orbital Coverage and Tasking
// ============================================================================

/**
 * Target location
 */
export interface TargetLocation {
  /** Latitude in degrees */
  latitude: number;

  /** Longitude in degrees */
  longitude: number;

  /** Altitude in meters (optional) */
  altitude?: number;

  /** Target identifier */
  id?: string;

  /** Target name */
  name?: string;
}

/**
 * Coverage analysis parameters
 */
export interface CoverageAnalysisParams {
  /** Orbital altitude in meters */
  altitude: number;

  /** Orbital inclination in degrees */
  inclination: number;

  /** Swath width in meters */
  swathWidth: number;

  /** Target location */
  target: TargetLocation;

  /** Analysis time window */
  timeWindow?: {
    start: Date;
    end: Date;
  };

  /** Minimum elevation angle in degrees */
  minElevation?: number;

  /** Maximum off-nadir angle in degrees */
  maxOffNadir?: number;
}

/**
 * Access window (opportunity to image target)
 */
export interface AccessWindow {
  /** Window start time */
  startTime: Date;

  /** Window end time */
  endTime: Date;

  /** Duration in seconds */
  duration: number;

  /** Maximum elevation angle in degrees */
  maxElevation: number;

  /** Off-nadir angle at maximum elevation */
  offNadirAngle: number;

  /** Sun elevation angle */
  sunElevation: number;

  /** Sun azimuth angle */
  sunAzimuth: number;

  /** Imaging feasibility */
  feasible: boolean;

  /** Reasons if not feasible */
  constraints?: string[];
}

/**
 * Coverage analysis result
 */
export interface CoverageAnalysisResult {
  /** Revisit time in hours */
  revisitTime: number;

  /** Number of access windows */
  numberOfPasses: number;

  /** Access windows */
  accessWindows: AccessWindow[];

  /** Coverage percentage (0-100) */
  coverage: number;

  /** Average time between passes in hours */
  averageGap: number;

  /** Maximum gap between passes in hours */
  maxGap: number;

  /** Orbital period in minutes */
  orbitalPeriod: number;
}

/**
 * Satellite tasking request
 */
export interface TaskingRequest {
  /** Request identifier */
  id: string;

  /** Target location */
  target: TargetLocation;

  /** Desired collection window */
  collectionWindow: {
    start: Date;
    end: Date;
  };

  /** Sensor requirements */
  sensorRequirements: {
    type: SensorType;
    minResolution?: number;
    maxCloudCover?: number;
    maxOffNadir?: number;
    mode?: SARMode;
  };

  /** Priority (1-10, higher = more urgent) */
  priority: number;

  /** Requester information */
  requester: string;

  /** Classification level */
  classification: 'unclassified' | 'confidential' | 'secret' | 'top-secret';
}

/**
 * Tasking response
 */
export interface TaskingResponse {
  /** Request ID */
  requestId: string;

  /** Tasking status */
  status: 'accepted' | 'scheduled' | 'rejected' | 'completed' | 'failed';

  /** Scheduled collection time */
  scheduledTime?: Date;

  /** Assigned satellite */
  satelliteId?: string;

  /** Estimated delivery time */
  deliveryTime?: Date;

  /** Reason if rejected */
  rejectionReason?: string;

  /** Collection metadata */
  metadata?: CollectionMetadata;
}

// ============================================================================
// Image Data and Metadata
// ============================================================================

/**
 * Collection metadata
 */
export interface CollectionMetadata {
  /** Image identifier */
  imageId: string;

  /** Satellite identifier */
  satelliteId: string;

  /** Sensor identifier */
  sensorId: string;

  /** Acquisition time */
  acquisitionTime: Date;

  /** Center coordinates */
  center: {
    latitude: number;
    longitude: number;
  };

  /** Corner coordinates */
  corners: {
    topLeft: { latitude: number; longitude: number };
    topRight: { latitude: number; longitude: number };
    bottomLeft: { latitude: number; longitude: number };
    bottomRight: { latitude: number; longitude: number };
  };

  /** Ground sample distance in meters */
  gsd: number;

  /** Sun angles */
  sun: {
    elevation: number;
    azimuth: number;
  };

  /** Satellite angles */
  satellite: {
    elevation: number;
    azimuth: number;
    offNadir: number;
  };

  /** Cloud cover percentage */
  cloudCover: number;

  /** Processing level */
  processingLevel: ProcessingLevel;

  /** Image dimensions */
  dimensions: {
    width: number;
    height: number;
    bands: number;
  };

  /** Data format */
  format: DataFormat;

  /** File size in bytes */
  fileSize: number;

  /** Quality metrics */
  quality: ImageQualityMetrics;
}

/**
 * Image processing request
 */
export interface ImageProcessingRequest {
  /** Input image data or file path */
  inputData: ArrayBuffer | string;

  /** Sensor type */
  sensorType: SensorType;

  /** Target processing level */
  targetLevel: ProcessingLevel;

  /** Output format */
  outputFormat: DataFormat;

  /** Processing corrections */
  corrections: {
    radiometric: boolean;
    geometric: boolean;
    atmospheric: boolean;
    terrain?: boolean;
  };

  /** Enhancement options */
  enhancements?: {
    contrastEnhancement: boolean;
    sharpening: boolean;
    noiseReduction: boolean;
  };

  /** Compression settings */
  compression?: {
    algorithm: 'lossless' | 'jpeg2000' | 'png';
    quality?: number;  // 1-100 for lossy
  };
}

/**
 * Image processing result
 */
export interface ImageProcessingResult {
  /** Output image data */
  outputData: ArrayBuffer;

  /** Metadata */
  metadata: CollectionMetadata;

  /** Quality metrics */
  quality: ImageQualityMetrics;

  /** Processing time in milliseconds */
  processingTime: number;

  /** Processing log */
  log: string[];

  /** Warnings encountered */
  warnings: string[];

  /** Success status */
  success: boolean;

  /** Error message if failed */
  error?: string;
}

// ============================================================================
// SAR-Specific Types
// ============================================================================

/**
 * SAR processing parameters
 */
export interface SARProcessingParams {
  /** Imaging mode */
  mode: SARMode;

  /** Polarization */
  polarization: Polarization;

  /** Range bandwidth in Hz */
  rangeBandwidth: number;

  /** Azimuth bandwidth in Hz */
  azimuthBandwidth: number;

  /** Incidence angle in degrees */
  incidenceAngle: number;

  /** Look direction */
  lookDirection: 'left' | 'right';

  /** Number of looks (averaging) */
  numberOfLooks: number;

  /** Focus processing */
  focusing: {
    algorithm: 'omega-k' | 'range-doppler' | 'chirp-scaling';
    autofocus: boolean;
  };

  /** Interferometric parameters (if applicable) */
  interferometric?: {
    baseline: number;  // meters
    coherenceThreshold: number;
  };
}

/**
 * SAR image product
 */
export interface SARImageProduct {
  /** Amplitude image */
  amplitude: ArrayBuffer;

  /** Phase image (if complex) */
  phase?: ArrayBuffer;

  /** Coherence map (if interferometric) */
  coherence?: ArrayBuffer;

  /** Resolution */
  resolution: {
    range: number;  // meters
    azimuth: number;  // meters
  };

  /** Polarization */
  polarization: Polarization;

  /** Incidence angle in degrees */
  incidenceAngle: number;

  /** Metadata */
  metadata: CollectionMetadata;
}

// ============================================================================
// SIGINT-Specific Types
// ============================================================================

/**
 * Signal detection
 */
export interface SignalDetection {
  /** Detection identifier */
  id: string;

  /** Detection time */
  timestamp: Date;

  /** Center frequency in Hz */
  frequency: number;

  /** Bandwidth in Hz */
  bandwidth: number;

  /** Signal strength in dBm */
  power: number;

  /** Signal-to-noise ratio in dB */
  snr: number;

  /** Modulation type */
  modulation: string;

  /** Estimated location */
  location?: {
    latitude: number;
    longitude: number;
    accuracy: number;  // meters
  };

  /** Signal classification */
  classification: {
    type: 'communication' | 'radar' | 'navigation' | 'unknown';
    subtype?: string;
    confidence: number;  // 0-1
  };

  /** Technical parameters */
  parameters?: {
    prf?: number;  // Hz
    pulseWidth?: number;  // seconds
    scanRate?: number;  // RPM
  };
}

/**
 * Emitter database record
 */
export interface EmitterRecord {
  /** Emitter identifier */
  id: string;

  /** Emitter name/designation */
  name: string;

  /** Frequency range */
  frequencyRange: {
    min: number;
    max: number;
  };

  /** Typical parameters */
  parameters: {
    bandwidth: number;
    pulseWidth?: number;
    prf?: number;
    scanType?: string;
    scanRate?: number;
  };

  /** Platform information */
  platform: {
    type: string;
    name: string;
    country?: string;
  };

  /** Last observed */
  lastObserved?: Date;

  /** Confidence level */
  confidence: number;  // 0-1
}

// ============================================================================
// Security and Encryption
// ============================================================================

/**
 * Encryption configuration
 */
export interface EncryptionConfig {
  /** Encryption algorithm */
  algorithm: 'aes-256-gcm' | 'aes-256-cbc' | 'chacha20-poly1305';

  /** Key length in bits */
  keyLength: 256 | 384 | 512;

  /** Initialization vector */
  iv?: ArrayBuffer;

  /** Authentication tag length */
  tagLength?: number;

  /** Additional authenticated data */
  aad?: ArrayBuffer;
}

/**
 * Access control policy
 */
export interface AccessControlPolicy {
  /** User identifier */
  userId: string;

  /** User role */
  role: 'admin' | 'analyst' | 'operator' | 'viewer';

  /** Permissions */
  permissions: {
    task: boolean;
    view: boolean;
    download: boolean;
    process: boolean;
    delete: boolean;
  };

  /** Clearance level */
  clearance: 'unclassified' | 'confidential' | 'secret' | 'top-secret';

  /** Expiration time */
  expiresAt?: Date;
}

/**
 * Audit log entry
 */
export interface AuditLogEntry {
  /** Log entry identifier */
  id: string;

  /** Timestamp */
  timestamp: Date;

  /** User identifier */
  userId: string;

  /** Action performed */
  action: string;

  /** Resource accessed */
  resource: string;

  /** IP address */
  ipAddress: string;

  /** Success status */
  success: boolean;

  /** Classification level */
  classification: string;

  /** Additional details */
  details?: Record<string, unknown>;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical and orbital constants
 */
export const RECON_CONSTANTS = {
  /** Speed of light in m/s */
  SPEED_OF_LIGHT: 299792458,

  /** Earth radius in meters */
  EARTH_RADIUS: 6371000,

  /** Earth gravitational parameter in m³/s² */
  EARTH_MU: 3.986004418e14,

  /** Typical LEO altitudes */
  LEO_ALTITUDE: {
    MIN: 300000,  // 300 km
    MAX: 2000000,  // 2000 km
    TYPICAL: 550000,  // 550 km
  },

  /** Typical sun-synchronous inclination */
  SSO_INCLINATION: 97.4,  // degrees

  /** SAR frequency bands */
  SAR_BANDS: {
    X_BAND: 9.6e9,  // Hz
    C_BAND: 5.4e9,  // Hz
    L_BAND: 1.25e9,  // Hz
    S_BAND: 3.0e9,  // Hz
  },

  /** NIIRS scale range */
  NIIRS_RANGE: {
    MIN: 0,
    MAX: 9,
  },

  /** Minimum acceptable SNR */
  MIN_SNR: 50,

  /** Maximum acceptable cloud cover */
  MAX_CLOUD_COVER: 20,  // percent
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-DEF-011 error codes
 */
export enum ReconErrorCode {
  INSUFFICIENT_RESOLUTION = 'R001',
  TARGET_NOT_IN_SWATH = 'R002',
  CLOUD_COVER_EXCESSIVE = 'R003',
  PROCESSING_FAILED = 'R004',
  ENCRYPTION_ERROR = 'R005',
  COVERAGE_GAP = 'R006',
  INVALID_PARAMETERS = 'R007',
  SENSOR_MALFUNCTION = 'R008',
  DOWNLINK_FAILED = 'R009',
  UNAUTHORIZED_ACCESS = 'R010',
}

/**
 * Reconnaissance satellite error
 */
export class ReconSatelliteError extends Error {
  constructor(
    public code: ReconErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'ReconSatelliteError';
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

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  OrbitalParameters,
  OpticalSensorConfig,
  SARSensorConfig,
  SIGINTSensorConfig,
  SpectralBand,

  // Resolution and quality
  GroundResolutionParams,
  GroundResolutionResult,
  ImageQualityMetrics,

  // Coverage and tasking
  TargetLocation,
  CoverageAnalysisParams,
  CoverageAnalysisResult,
  AccessWindow,
  TaskingRequest,
  TaskingResponse,

  // Image data
  CollectionMetadata,
  ImageProcessingRequest,
  ImageProcessingResult,

  // SAR-specific
  SARProcessingParams,
  SARImageProduct,

  // SIGINT-specific
  SignalDetection,
  EmitterRecord,

  // Security
  EncryptionConfig,
  AccessControlPolicy,
  AuditLogEntry,
};

export { RECON_CONSTANTS, ReconErrorCode, ReconSatelliteError };
