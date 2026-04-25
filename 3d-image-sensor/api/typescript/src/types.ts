/**
 * WIA-SEMI-013: 3D Image Sensor Standard - Type Definitions
 *
 * @packageDocumentation
 * @module wia-semi-013
 */

/**
 * Sensor technology type
 */
export enum SensorTechnology {
  /** Indirect Time-of-Flight */
  iToF = 'iToF',
  /** Direct Time-of-Flight (SPAD-based) */
  dToF = 'dToF',
  /** Structured Light (pattern projection) */
  StructuredLight = 'structured_light',
  /** Stereo Vision (passive or active) */
  Stereo = 'stereo',
  /** Hybrid multi-technology */
  Hybrid = 'hybrid'
}

/**
 * Depth map encoding format
 */
export enum DepthEncoding {
  /** 16-bit unsigned integer */
  UInt16 = 'uint16',
  /** 32-bit unsigned integer */
  UInt32 = 'uint32',
  /** 32-bit floating point */
  Float32 = 'float32'
}

/**
 * Point cloud data format
 */
export enum PointCloudFormat {
  /** XYZ ASCII format */
  XYZ = 'xyz',
  /** PCD (Point Cloud Data) binary */
  PCD = 'pcd',
  /** PLY (Polygon File Format) */
  PLY = 'ply',
  /** JSON format */
  JSON = 'json'
}

/**
 * Sensor resolution configuration
 */
export interface Resolution {
  /** Image width in pixels */
  width: number;
  /** Image height in pixels */
  height: number;
}

/**
 * Depth range specification
 */
export interface DepthRange {
  /** Minimum depth in meters */
  min: number;
  /** Maximum depth in meters */
  max: number;
  /** Units (always meters for WIA-SEMI-013) */
  units: 'meters';
}

/**
 * Depth accuracy specification
 */
export interface DepthAccuracy {
  /** Accuracy value */
  value: number;
  /** Units (mm, cm, or percent) */
  units: 'mm' | 'cm' | 'percent';
  /** Distance at which accuracy is specified */
  at_distance: number;
}

/**
 * Field of view specification
 */
export interface FieldOfView {
  /** Horizontal FOV */
  horizontal: number;
  /** Vertical FOV */
  vertical: number;
  /** Units (degrees or radians) */
  units: 'degrees' | 'radians';
}

/**
 * Complete sensor specification
 */
export interface SensorSpecification {
  /** WIA standard identifier */
  standard: 'WIA-SEMI-013';
  /** Standard version */
  version: string;
  /** Sensor details */
  sensor: {
    /** Sensor technology type */
    technology: SensorTechnology;
    /** Manufacturer name */
    manufacturer?: string;
    /** Model identifier */
    model?: string;
    /** Image resolution */
    resolution: Resolution;
    /** Frame rate in fps */
    frameRate: number;
    /** Operating depth range */
    depthRange: DepthRange;
    /** Depth accuracy specification */
    accuracy?: DepthAccuracy;
    /** Field of view */
    fieldOfView?: FieldOfView;
    /** Illumination wavelength in nm */
    wavelength?: number;
    /** Illumination type */
    illumination?: 'active' | 'passive' | 'hybrid';
  };
}

/**
 * Sensor configuration options
 */
export interface SensorConfig {
  /** Device identifier */
  deviceId?: string;
  /** Image resolution */
  resolution?: Resolution;
  /** Frame rate in fps */
  frameRate?: number;
  /** Depth quality preset */
  depthQuality?: 'low' | 'balanced' | 'high' | 'maximum';
  /** Filtering options */
  filters?: {
    /** Temporal filtering (frame averaging) */
    temporal?: {
      enabled: boolean;
      frames: number;
    };
    /** Spatial filtering */
    spatial?: {
      enabled: boolean;
      sigma: number;
    };
    /** Edge-preserving filter */
    edge_preserving?: boolean;
  };
  /** Power mode */
  powerMode?: 'low_power' | 'balanced' | 'performance';
}

/**
 * Camera intrinsic calibration
 */
export interface IntrinsicCalibration {
  /** Camera matrix parameters */
  camera_matrix: {
    /** Focal length X */
    fx: number;
    /** Focal length Y */
    fy: number;
    /** Principal point X */
    cx: number;
    /** Principal point Y */
    cy: number;
  };
  /** Lens distortion parameters */
  distortion: {
    /** Distortion model */
    model: 'brown_conrady' | 'fisheye';
    /** Distortion coefficients */
    coefficients: {
      k1: number;
      k2: number;
      p1: number;
      p2: number;
      k3?: number;
    };
  };
}

/**
 * 3D transformation (rotation + translation)
 */
export interface Transform3D {
  /** Rotation as quaternion */
  rotation: {
    format: 'quaternion';
    w: number;
    x: number;
    y: number;
    z: number;
  };
  /** Translation vector */
  translation: {
    x: number;
    y: number;
    z: number;
    units: 'meters';
  };
}

/**
 * Depth frame data
 */
export interface DepthFrame {
  /** Frame sequence number */
  sequenceNumber: number;
  /** Timestamp in microseconds */
  timestamp: number;
  /** Image dimensions */
  width: number;
  height: number;
  /** Depth encoding format */
  encoding: DepthEncoding;
  /** Depth scale factor (actual_depth = value * scale) */
  depthScale: number;
  /** Depth data buffer */
  data: ArrayBuffer;
  /** Optional confidence map */
  confidence?: Uint8Array;
}

/**
 * 3D point in space
 */
export interface Point3D {
  /** X coordinate */
  x: number;
  /** Y coordinate */
  y: number;
  /** Z coordinate (depth) */
  z: number;
  /** Red color component (0-255) */
  r?: number;
  /** Green color component (0-255) */
  g?: number;
  /** Blue color component (0-255) */
  b?: number;
  /** Point confidence (0-255) */
  confidence?: number;
}

/**
 * Point cloud data structure
 */
export interface PointCloud {
  /** Array of 3D points */
  points: Point3D[];
  /** Coordinate system identifier */
  coordinate_system: 'camera' | 'world';
  /** Units for coordinates */
  units: 'meters' | 'millimeters';
  /** Timestamp */
  timestamp?: number;
}

/**
 * Point cloud processing options
 */
export interface PointCloudOptions {
  /** Remove invalid points (depth = 0) */
  removeInvalid?: boolean;
  /** Downsample using voxel grid */
  voxelSize?: number;
  /** Apply statistical outlier removal */
  outlierRemoval?: {
    enabled: boolean;
    k_neighbors: number;
    std_dev_multiplier: number;
  };
  /** Transform to world coordinates */
  transformToWorld?: Transform3D;
}

/**
 * Sensor event types
 */
export type SensorEventType =
  | 'depth-frame'
  | 'rgb-frame'
  | 'point-cloud'
  | 'error'
  | 'disconnected'
  | 'calibration-changed';

/**
 * Event callback function type
 */
export type EventCallback<T = any> = (data: T) => void;

/**
 * Certification level
 */
export enum CertificationLevel {
  Bronze = 'bronze',
  Silver = 'silver',
  Gold = 'gold'
}

/**
 * Certification criteria
 */
export interface CertificationCriteria {
  /** Certification level */
  level: CertificationLevel;
  /** Depth accuracy requirement */
  depthAccuracy: string;
  /** Minimum resolution */
  minResolution: Resolution;
  /** Minimum frame rate */
  minFrameRate: number;
  /** Operating range requirement */
  operatingRange: DepthRange;
  /** Ambient light robustness */
  ambientLight: 'indoor' | 'indoor_outdoor' | 'full_sunlight';
  /** Multi-path handling capability */
  multiPathHandling: 'basic' | 'advanced' | 'expert';
}

/**
 * Testing result
 */
export interface TestResult {
  /** Test name */
  testName: string;
  /** Pass/fail status */
  passed: boolean;
  /** Measured value */
  measuredValue?: number;
  /** Required value */
  requiredValue?: number;
  /** Units */
  units?: string;
  /** Additional notes */
  notes?: string;
}

/**
 * WIA-SEMI-013 compliance test report
 */
export interface ComplianceReport {
  /** Standard version */
  standard: 'WIA-SEMI-013';
  /** Test date */
  testDate: string;
  /** Sensor under test */
  sensor: SensorSpecification;
  /** Target certification level */
  targetLevel: CertificationLevel;
  /** Individual test results */
  tests: TestResult[];
  /** Overall pass/fail */
  passed: boolean;
  /** Achieved certification level */
  achievedLevel?: CertificationLevel;
}
