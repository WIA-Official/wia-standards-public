/**
 * @file lidar.ts
 * @description WIA Smart Wheelchair LiDAR Interface
 * @version 1.0.0
 */

import {
  Header,
  Vector3D,
  PointCloud,
  BaseSensorConfig,
  SensorStatus,
  SensorCallback,
  Disposable,
} from './types';

/**
 * LiDAR configuration
 */
export interface LiDARConfig extends BaseSensorConfig {
  model: LiDARModel;
  scanRate: number;           // Hz (5-20)
  angularResolution: number;  // degrees
  maxRange: number;           // meters
  minRange: number;           // meters
  invertScan: boolean;        // Reverse scan direction
  angleOffset: number;        // degrees, mounting offset
}

/**
 * Supported LiDAR models
 */
export type LiDARModel =
  | 'rplidar_a1'
  | 'rplidar_a2'
  | 'rplidar_a3'
  | 'ydlidar_x4'
  | 'ydlidar_g4'
  | 'hokuyo_ust10lx'
  | 'sick_tim551'
  | 'generic';

/**
 * Laser scan data (single sweep)
 */
export interface LaserScan {
  header: Header;
  angleMin: number;           // radians, start angle
  angleMax: number;           // radians, end angle
  angleIncrement: number;     // radians, angular distance between measurements
  timeIncrement: number;      // seconds, time between measurements
  scanTime: number;           // seconds, time for complete scan
  rangeMin: number;           // meters, minimum valid range
  rangeMax: number;           // meters, maximum valid range
  ranges: Float32Array;       // meters, range measurements (Inf = no return)
  intensities?: Float32Array; // optional intensity values
}

/**
 * LiDAR scan statistics
 */
export interface ScanStatistics {
  validPoints: number;
  invalidPoints: number;
  averageRange: number;
  minRange: number;
  maxRange: number;
  scanDuration: number;       // milliseconds
}

/**
 * Default LiDAR configurations by model
 */
export const LIDAR_PRESETS: Record<LiDARModel, Partial<LiDARConfig>> = {
  rplidar_a1: {
    scanRate: 5.5,
    angularResolution: 1.0,
    maxRange: 12.0,
    minRange: 0.15,
  },
  rplidar_a2: {
    scanRate: 10,
    angularResolution: 0.9,
    maxRange: 18.0,
    minRange: 0.2,
  },
  rplidar_a3: {
    scanRate: 15,
    angularResolution: 0.225,
    maxRange: 25.0,
    minRange: 0.2,
  },
  ydlidar_x4: {
    scanRate: 7,
    angularResolution: 0.5,
    maxRange: 10.0,
    minRange: 0.12,
  },
  ydlidar_g4: {
    scanRate: 9,
    angularResolution: 0.28,
    maxRange: 16.0,
    minRange: 0.28,
  },
  hokuyo_ust10lx: {
    scanRate: 40,
    angularResolution: 0.25,
    maxRange: 10.0,
    minRange: 0.06,
  },
  sick_tim551: {
    scanRate: 15,
    angularResolution: 0.33,
    maxRange: 10.0,
    minRange: 0.05,
  },
  generic: {
    scanRate: 10,
    angularResolution: 1.0,
    maxRange: 12.0,
    minRange: 0.1,
  },
};

/**
 * LiDAR Interface
 */
export interface LiDARInterface extends Disposable {
  /** Current configuration */
  readonly config: LiDARConfig;

  /** Current status */
  readonly status: SensorStatus;

  /** Initialize the LiDAR */
  initialize(): Promise<void>;

  /** Start scanning */
  start(): Promise<void>;

  /** Stop scanning */
  stop(): Promise<void>;

  /** Register scan callback */
  onScan(callback: SensorCallback<LaserScan>): void;

  /** Remove scan callback */
  offScan(callback: SensorCallback<LaserScan>): void;

  /** Get latest scan */
  getLatestScan(): LaserScan | null;

  /** Convert scan to point cloud */
  toPointCloud(scan: LaserScan): PointCloud;

  /** Get scan statistics */
  getStatistics(): ScanStatistics;

  /** Set motor speed (if supported) */
  setMotorSpeed?(rpm: number): Promise<void>;
}

/**
 * Convert laser scan to 2D point cloud
 */
export function scanToPointCloud(
  scan: LaserScan,
  transform?: { x: number; y: number; theta: number }
): PointCloud {
  const points: Vector3D[] = [];
  const intensities: number[] = [];

  const cos = transform ? Math.cos(transform.theta) : 1;
  const sin = transform ? Math.sin(transform.theta) : 0;
  const tx = transform?.x ?? 0;
  const ty = transform?.y ?? 0;

  for (let i = 0; i < scan.ranges.length; i++) {
    const range = scan.ranges[i];

    // Skip invalid measurements
    if (!isFinite(range) || range < scan.rangeMin || range > scan.rangeMax) {
      continue;
    }

    const angle = scan.angleMin + i * scan.angleIncrement;
    let x = range * Math.cos(angle);
    let y = range * Math.sin(angle);

    // Apply transform
    if (transform) {
      const nx = cos * x - sin * y + tx;
      const ny = sin * x + cos * y + ty;
      x = nx;
      y = ny;
    }

    points.push({ x, y, z: 0 });

    if (scan.intensities) {
      intensities.push(scan.intensities[i]);
    }
  }

  return {
    header: { ...scan.header, frameId: 'base_link' },
    points,
    intensities: intensities.length > 0 ? intensities : undefined,
  };
}

/**
 * Filter scan by range
 */
export function filterScanByRange(
  scan: LaserScan,
  minRange: number,
  maxRange: number
): LaserScan {
  const filteredRanges = new Float32Array(scan.ranges.length);
  const filteredIntensities = scan.intensities
    ? new Float32Array(scan.intensities.length)
    : undefined;

  for (let i = 0; i < scan.ranges.length; i++) {
    const range = scan.ranges[i];
    if (range >= minRange && range <= maxRange) {
      filteredRanges[i] = range;
      if (filteredIntensities && scan.intensities) {
        filteredIntensities[i] = scan.intensities[i];
      }
    } else {
      filteredRanges[i] = Infinity;
      if (filteredIntensities) {
        filteredIntensities[i] = 0;
      }
    }
  }

  return {
    ...scan,
    ranges: filteredRanges,
    intensities: filteredIntensities,
  };
}

/**
 * Filter scan by angle
 */
export function filterScanByAngle(
  scan: LaserScan,
  minAngle: number,
  maxAngle: number
): LaserScan {
  const startIndex = Math.max(
    0,
    Math.floor((minAngle - scan.angleMin) / scan.angleIncrement)
  );
  const endIndex = Math.min(
    scan.ranges.length,
    Math.ceil((maxAngle - scan.angleMin) / scan.angleIncrement)
  );

  const count = endIndex - startIndex;
  const filteredRanges = new Float32Array(count);
  const filteredIntensities = scan.intensities
    ? new Float32Array(count)
    : undefined;

  for (let i = 0; i < count; i++) {
    filteredRanges[i] = scan.ranges[startIndex + i];
    if (filteredIntensities && scan.intensities) {
      filteredIntensities[i] = scan.intensities[startIndex + i];
    }
  }

  return {
    ...scan,
    angleMin: scan.angleMin + startIndex * scan.angleIncrement,
    angleMax: scan.angleMin + endIndex * scan.angleIncrement,
    ranges: filteredRanges,
    intensities: filteredIntensities,
  };
}

/**
 * Downsample scan
 */
export function downsampleScan(scan: LaserScan, factor: number): LaserScan {
  const newLength = Math.ceil(scan.ranges.length / factor);
  const ranges = new Float32Array(newLength);
  const intensities = scan.intensities ? new Float32Array(newLength) : undefined;

  for (let i = 0; i < newLength; i++) {
    ranges[i] = scan.ranges[i * factor];
    if (intensities && scan.intensities) {
      intensities[i] = scan.intensities[i * factor];
    }
  }

  return {
    ...scan,
    angleIncrement: scan.angleIncrement * factor,
    timeIncrement: scan.timeIncrement * factor,
    ranges,
    intensities,
  };
}

/**
 * Calculate scan statistics
 */
export function calculateScanStatistics(scan: LaserScan): ScanStatistics {
  let validPoints = 0;
  let sum = 0;
  let minRange = Infinity;
  let maxRange = -Infinity;

  for (let i = 0; i < scan.ranges.length; i++) {
    const range = scan.ranges[i];
    if (isFinite(range) && range >= scan.rangeMin && range <= scan.rangeMax) {
      validPoints++;
      sum += range;
      minRange = Math.min(minRange, range);
      maxRange = Math.max(maxRange, range);
    }
  }

  return {
    validPoints,
    invalidPoints: scan.ranges.length - validPoints,
    averageRange: validPoints > 0 ? sum / validPoints : 0,
    minRange: isFinite(minRange) ? minRange : 0,
    maxRange: isFinite(maxRange) ? maxRange : 0,
    scanDuration: scan.scanTime * 1000,
  };
}
