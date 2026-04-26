/**
 * WIA-SEMI-014 LiDAR Sensor TypeScript SDK
 * Type Definitions
 * 
 * @copyright 2025 WIA (World Industry Association)
 * @license MIT
 */

/**
 * 3D Point in Cartesian coordinates
 */
export interface Point3D {
  /** X coordinate in meters */
  x: number;
  /** Y coordinate in meters */
  y: number;
  /** Z coordinate in meters */
  z: number;
  /** Intensity value (0-255 or 0.0-1.0 normalized) */
  intensity: number;
  /** Timestamp in nanoseconds (optional) */
  timestamp?: number;
  /** Ring/layer ID for multi-channel LiDAR (optional) */
  ring?: number;
  /** Azimuth angle in radians (optional) */
  azimuth?: number;
  /** Range/distance in meters (optional) */
  range?: number;
  /** Velocity in m/s (FMCW LiDAR only, optional) */
  velocity?: number;
}

/**
 * Point Cloud - collection of 3D points
 */
export interface PointCloud {
  /** Array of 3D points */
  points: Point3D[];
  /** Frame timestamp (Unix time in microseconds) */
  timestamp: number;
  /** Frame sequence number */
  sequenceNumber: number;
  /** Sensor coordinate frame ID */
  frameId: string;
  /** Number of points in cloud */
  pointCount: number;
  /** Optional metadata */
  metadata?: PointCloudMetadata;
}

/**
 * Point Cloud Metadata
 */
export interface PointCloudMetadata {
  /** Sensor model name */
  sensorModel?: string;
  /** Firmware version */
  firmwareVersion?: string;
  /** Operating mode */
  operatingMode?: OperatingMode;
  /** Environmental conditions */
  environmentalConditions?: EnvironmentalConditions;
}

/**
 * Environmental conditions during capture
 */
export interface EnvironmentalConditions {
  /** Temperature in Celsius */
  temperature?: number;
  /** Humidity percentage (0-100) */
  humidity?: number;
  /** Weather condition */
  weather?: 'clear' | 'rain' | 'fog' | 'snow' | 'unknown';
}

/**
 * LiDAR Operating Mode
 */
export enum OperatingMode {
  STANDARD = 'standard',
  LONG_RANGE = 'long_range',
  SHORT_RANGE = 'short_range',
  HIGH_RESOLUTION = 'high_resolution',
  POWER_SAVE = 'power_save',
}

/**
 * LiDAR Architecture Type
 */
export enum LiDARArchitecture {
  MECHANICAL = 'mechanical',
  MEMS = 'mems',
  OPA = 'opa',
  FLASH = 'flash',
  FMCW = 'fmcw',
}

/**
 * Laser Wavelength
 */
export enum Wavelength {
  NM_905 = 905,
  NM_1550 = 1550,
}

/**
 * Detector Type
 */
export enum DetectorType {
  SI_APD = 'si_apd',
  INGAAS_APD = 'ingaas_apd',
  SPAD = 'spad',
  SIPM = 'sipm',
}

/**
 * LiDAR Sensor Specifications
 */
export interface LiDARSpecification {
  /** Sensor model identifier */
  model: string;
  /** Manufacturer name */
  manufacturer: string;
  /** Architecture type */
  architecture: LiDARArchitecture;
  /** Laser wavelength in nanometers */
  wavelength: Wavelength;
  /** Detector technology */
  detectorType: DetectorType;
  /** Performance specifications */
  performance: PerformanceSpec;
  /** Field of view specifications */
  fieldOfView: FieldOfViewSpec;
  /** Safety classification */
  safetyClass: 'Class1' | 'Class1M';
  /** IP rating */
  ipRating: string;
}

/**
 * Performance Specifications
 */
export interface PerformanceSpec {
  /** Maximum range in meters @ 10% reflectivity */
  maxRange: number;
  /** Range accuracy in meters (1 sigma) */
  rangeAccuracy: number;
  /** Angular resolution in degrees */
  angularResolution: number;
  /** Points per second */
  pointsPerSecond: number;
  /** Frame rate in Hz */
  frameRate: number;
}

/**
 * Field of View Specifications
 */
export interface FieldOfViewSpec {
  /** Horizontal FOV in degrees */
  horizontal: number;
  /** Vertical FOV in degrees */
  vertical: number;
  /** Horizontal angular resolution in degrees */
  horizontalResolution: number;
  /** Vertical angular resolution in degrees */
  verticalResolution: number;
}

/**
 * LiDAR Sensor Configuration
 */
export interface LiDARConfiguration {
  /** Operating mode */
  mode: OperatingMode;
  /** Frame rate (Hz) */
  frameRate: number;
  /** Return mode */
  returnMode: 'strongest' | 'last' | 'dual' | 'all';
  /** Enable intensity calibration */
  intensityCalibration: boolean;
  /** Minimum range filter (meters) */
  minRange?: number;
  /** Maximum range filter (meters) */
  maxRange?: number;
}

/**
 * LiDAR Sensor Status
 */
export interface LiDARStatus {
  /** Operational state */
  state: 'initializing' | 'normal' | 'degraded' | 'error' | 'safe_state';
  /** Health status flags */
  health: HealthStatus;
  /** Current temperature in Celsius */
  temperature: number;
  /** Power consumption in Watts */
  powerConsumption: number;
  /** Uptime in hours */
  uptime: number;
  /** Frame rate (actual) */
  actualFrameRate: number;
  /** Error codes (if any) */
  errorCodes?: string[];
}

/**
 * Health Status Flags
 */
export interface HealthStatus {
  /** Laser operational */
  laserOk: boolean;
  /** Detector operational */
  detectorOk: boolean;
  /** Thermal warning */
  thermalWarning: boolean;
  /** Communication error */
  communicationError: boolean;
}

/**
 * Bounding Box (3D)
 */
export interface BoundingBox3D {
  /** Center point */
  center: Point3D;
  /** Dimensions (width, height, depth) in meters */
  dimensions: { width: number; height: number; depth: number };
  /** Rotation (roll, pitch, yaw) in radians */
  rotation: { roll: number; pitch: number; yaw: number };
}

/**
 * Detected Object
 */
export interface DetectedObject {
  /** Object ID */
  id: string;
  /** Object classification */
  class: 'vehicle' | 'pedestrian' | 'cyclist' | 'static' | 'unknown';
  /** 3D bounding box */
  boundingBox: BoundingBox3D;
  /** Confidence score (0.0 - 1.0) */
  confidence: number;
  /** Velocity vector (m/s) */
  velocity?: { vx: number; vy: number; vz: number };
  /** Tracking state */
  trackingState?: 'new' | 'tracked' | 'lost';
}

/**
 * Calibration Parameters
 */
export interface CalibrationParameters {
  /** Extrinsic calibration (sensor to vehicle frame) */
  extrinsic: {
    /** Translation vector [x, y, z] in meters */
    translation: [number, number, number];
    /** Rotation matrix (3x3) or quaternion [w, x, y, z] */
    rotation: number[][] | [number, number, number, number];
  };
  /** Intrinsic calibration */
  intrinsic?: {
    /** Range offset correction */
    rangeOffset?: number;
    /** Intensity calibration curve */
    intensityCurve?: number[];
  };
}
