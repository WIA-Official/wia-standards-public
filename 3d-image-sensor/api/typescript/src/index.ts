/**
 * WIA-SEMI-013: 3D Image Sensor Standard - SDK Implementation
 *
 * @packageDocumentation
 * @module wia-semi-013
 */

import { EventEmitter } from 'events';
import * as types from './types';

// Re-export all types
export * from './types';

/**
 * Main 3D Image Sensor class
 * Provides unified interface for ToF, Structured Light, and Stereo Vision sensors
 *
 * @example
 * ```typescript
 * const sensor = new WIA3DImageSensor({
 *   deviceId: 'sensor-001',
 *   resolution: { width: 640, height: 480 },
 *   frameRate: 30
 * });
 *
 * await sensor.start();
 *
 * sensor.on('depth-frame', (frame) => {
 *   const pointCloud = convertToPointCloud(frame);
 *   console.log(`Points: ${pointCloud.points.length}`);
 * });
 * ```
 */
export class WIA3DImageSensor extends EventEmitter {
  private config: types.SensorConfig;
  private specification?: types.SensorSpecification;
  private intrinsicCalibration?: types.IntrinsicCalibration;
  private isStreaming: boolean = false;
  private frameSequence: number = 0;

  /**
   * Create a new 3D image sensor instance
   * @param config - Sensor configuration
   */
  constructor(config: types.SensorConfig) {
    super();
    this.config = config;
  }

  /**
   * Initialize and start the sensor
   * @throws Error if sensor cannot be initialized
   */
  async start(): Promise<void> {
    // Implementation would connect to actual hardware/API
    console.log(`Starting sensor: ${this.config.deviceId}`);
    this.isStreaming = true;
    this.frameSequence = 0;

    // Simulate frame streaming
    this.simulateFrameStream();
  }

  /**
   * Stop the sensor and release resources
   */
  async stop(): Promise<void> {
    console.log('Stopping sensor');
    this.isStreaming = false;
  }

  /**
   * Configure sensor parameters
   * @param config - Configuration options to apply
   */
  async configure(config: Partial<types.SensorConfig>): Promise<void> {
    this.config = { ...this.config, ...config };
    console.log('Sensor configured:', this.config);
  }

  /**
   * Get current sensor specification
   * @returns Sensor specification
   */
  getSpecification(): types.SensorSpecification | undefined {
    return this.specification;
  }

  /**
   * Get intrinsic calibration parameters
   * @returns Intrinsic calibration
   */
  getCalibration(): types.IntrinsicCalibration | undefined {
    return this.intrinsicCalibration;
  }

  /**
   * Capture a single depth frame
   * @returns Depth frame data
   */
  async captureFrame(): Promise<types.DepthFrame> {
    const frame: types.DepthFrame = {
      sequenceNumber: this.frameSequence++,
      timestamp: Date.now() * 1000, // microseconds
      width: this.config.resolution?.width || 640,
      height: this.config.resolution?.height || 480,
      encoding: types.DepthEncoding.UInt16,
      depthScale: 1.0,
      data: new ArrayBuffer(640 * 480 * 2), // Simulated data
      confidence: new Uint8Array(640 * 480).fill(255)
    };

    return frame;
  }

  /**
   * Simulate frame streaming (for demonstration)
   * In production, this would read from actual hardware
   */
  private simulateFrameStream(): void {
    const interval = 1000 / (this.config.frameRate || 30);

    const streamLoop = () => {
      if (!this.isStreaming) return;

      this.captureFrame().then(frame => {
        this.emit('depth-frame', frame);
      });

      setTimeout(streamLoop, interval);
    };

    streamLoop();
  }
}

/**
 * Convert depth frame to point cloud
 *
 * @param frame - Depth frame data
 * @param calibration - Camera intrinsic calibration
 * @param options - Processing options
 * @returns Point cloud
 *
 * @example
 * ```typescript
 * const pointCloud = convertToPointCloud(depthFrame, calibration, {
 *   removeInvalid: true,
 *   voxelSize: 0.01
 * });
 * ```
 */
export function convertToPointCloud(
  frame: types.DepthFrame,
  calibration: types.IntrinsicCalibration,
  options?: types.PointCloudOptions
): types.PointCloud {
  const points: types.Point3D[] = [];

  // Parse depth data based on encoding
  let depthValues: number[];
  if (frame.encoding === types.DepthEncoding.UInt16) {
    const uint16Array = new Uint16Array(frame.data);
    depthValues = Array.from(uint16Array);
  } else if (frame.encoding === types.DepthEncoding.Float32) {
    const float32Array = new Float32Array(frame.data);
    depthValues = Array.from(float32Array);
  } else {
    throw new Error(`Unsupported encoding: ${frame.encoding}`);
  }

  // Convert depth pixels to 3D points
  for (let v = 0; v < frame.height; v++) {
    for (let u = 0; u < frame.width; u++) {
      const idx = v * frame.width + u;
      const depth = depthValues[idx] * frame.depthScale / 1000; // Convert to meters

      // Skip invalid depth
      if (options?.removeInvalid && depth === 0) {
        continue;
      }

      // Project to 3D using camera intrinsics
      const x = (u - calibration.camera_matrix.cx) * depth / calibration.camera_matrix.fx;
      const y = (v - calibration.camera_matrix.cy) * depth / calibration.camera_matrix.fy;
      const z = depth;

      points.push({
        x,
        y,
        z,
        confidence: frame.confidence?.[idx]
      });
    }
  }

  // Apply voxel downsampling if requested
  let processedPoints = points;
  if (options?.voxelSize) {
    processedPoints = voxelDownsample(points, options.voxelSize);
  }

  return {
    points: processedPoints,
    coordinate_system: 'camera',
    units: 'meters',
    timestamp: frame.timestamp
  };
}

/**
 * Downsample point cloud using voxel grid
 *
 * @param points - Input points
 * @param voxelSize - Voxel size in meters
 * @returns Downsampled points
 */
export function voxelDownsample(
  points: types.Point3D[],
  voxelSize: number
): types.Point3D[] {
  const voxelMap = new Map<string, types.Point3D[]>();

  // Group points by voxel
  for (const point of points) {
    const vx = Math.floor(point.x / voxelSize);
    const vy = Math.floor(point.y / voxelSize);
    const vz = Math.floor(point.z / voxelSize);
    const key = `${vx},${vy},${vz}`;

    if (!voxelMap.has(key)) {
      voxelMap.set(key, []);
    }
    voxelMap.get(key)!.push(point);
  }

  // Compute centroid for each voxel
  const downsampled: types.Point3D[] = [];
  for (const voxelPoints of voxelMap.values()) {
    const centroid: types.Point3D = {
      x: 0,
      y: 0,
      z: 0,
      r: 0,
      g: 0,
      b: 0,
      confidence: 0
    };

    for (const p of voxelPoints) {
      centroid.x += p.x;
      centroid.y += p.y;
      centroid.z += p.z;
      centroid.r! += p.r || 0;
      centroid.g! += p.g || 0;
      centroid.b! += p.b || 0;
      centroid.confidence! += p.confidence || 0;
    }

    const n = voxelPoints.length;
    centroid.x /= n;
    centroid.y /= n;
    centroid.z /= n;
    centroid.r! /= n;
    centroid.g! /= n;
    centroid.b! /= n;
    centroid.confidence! /= n;

    downsampled.push(centroid);
  }

  return downsampled;
}

/**
 * Export point cloud to PCD format
 *
 * @param pointCloud - Point cloud data
 * @returns PCD format string
 */
export function exportToPCD(pointCloud: types.PointCloud): string {
  const hasColor = pointCloud.points.length > 0 && pointCloud.points[0].r !== undefined;

  let pcd = '# .PCD v0.7 - Point Cloud Data file format\n';
  pcd += 'VERSION 0.7\n';
  pcd += hasColor ? 'FIELDS x y z rgb\n' : 'FIELDS x y z\n';
  pcd += hasColor ? 'SIZE 4 4 4 4\n' : 'SIZE 4 4 4\n';
  pcd += hasColor ? 'TYPE F F F U\n' : 'TYPE F F F\n';
  pcd += hasColor ? 'COUNT 1 1 1 1\n' : 'COUNT 1 1 1\n';
  pcd += `WIDTH ${pointCloud.points.length}\n`;
  pcd += 'HEIGHT 1\n';
  pcd += 'VIEWPOINT 0 0 0 1 0 0 0\n';
  pcd += `POINTS ${pointCloud.points.length}\n`;
  pcd += 'DATA ascii\n';

  for (const point of pointCloud.points) {
    if (hasColor) {
      const r = point.r || 0;
      const g = point.g || 0;
      const b = point.b || 0;
      const rgb = (r << 16) | (g << 8) | b;
      pcd += `${point.x.toFixed(6)} ${point.y.toFixed(6)} ${point.z.toFixed(6)} ${rgb}\n`;
    } else {
      pcd += `${point.x.toFixed(6)} ${point.y.toFixed(6)} ${point.z.toFixed(6)}\n`;
    }
  }

  return pcd;
}

/**
 * Statistical outlier removal filter
 *
 * @param points - Input points
 * @param k_neighbors - Number of neighbors to analyze
 * @param std_dev_multiplier - Standard deviation multiplier threshold
 * @returns Filtered points
 */
export function removeStatisticalOutliers(
  points: types.Point3D[],
  k_neighbors: number = 50,
  std_dev_multiplier: number = 1.0
): types.Point3D[] {
  if (points.length < k_neighbors) {
    return points;
  }

  // Compute mean distances to k nearest neighbors
  const meanDistances: number[] = [];

  for (let i = 0; i < points.length; i++) {
    const p1 = points[i];

    // Find k nearest neighbors (simplified - full implementation would use KD-tree)
    const distances: number[] = [];
    for (let j = 0; j < points.length; j++) {
      if (i === j) continue;
      const p2 = points[j];
      const dist = Math.sqrt(
        Math.pow(p1.x - p2.x, 2) +
        Math.pow(p1.y - p2.y, 2) +
        Math.pow(p1.z - p2.z, 2)
      );
      distances.push(dist);
    }

    distances.sort((a, b) => a - b);
    const nearestK = distances.slice(0, k_neighbors);
    const mean = nearestK.reduce((a, b) => a + b, 0) / nearestK.length;
    meanDistances.push(mean);
  }

  // Compute global mean and std dev
  const globalMean = meanDistances.reduce((a, b) => a + b, 0) / meanDistances.length;
  const variance = meanDistances.reduce((a, b) => a + Math.pow(b - globalMean, 2), 0) / meanDistances.length;
  const stdDev = Math.sqrt(variance);

  // Filter outliers
  const threshold = globalMean + std_dev_multiplier * stdDev;
  return points.filter((_, i) => meanDistances[i] < threshold);
}

/**
 * WIA-SEMI-013 compliance checker
 *
 * @param specification - Sensor specification
 * @param targetLevel - Target certification level
 * @returns Compliance status
 */
export function checkCompliance(
  specification: types.SensorSpecification,
  targetLevel: types.CertificationLevel
): boolean {
  const criteria: Record<types.CertificationLevel, types.CertificationCriteria> = {
    [types.CertificationLevel.Bronze]: {
      level: types.CertificationLevel.Bronze,
      depthAccuracy: '±5mm @ 2m',
      minResolution: { width: 320, height: 240 },
      minFrameRate: 15,
      operatingRange: { min: 0.5, max: 5, units: 'meters' },
      ambientLight: 'indoor',
      multiPathHandling: 'basic'
    },
    [types.CertificationLevel.Silver]: {
      level: types.CertificationLevel.Silver,
      depthAccuracy: '±2mm @ 2m',
      minResolution: { width: 640, height: 480 },
      minFrameRate: 30,
      operatingRange: { min: 0.2, max: 8, units: 'meters' },
      ambientLight: 'indoor_outdoor',
      multiPathHandling: 'advanced'
    },
    [types.CertificationLevel.Gold]: {
      level: types.CertificationLevel.Gold,
      depthAccuracy: '±1mm @ 2m',
      minResolution: { width: 1280, height: 960 },
      minFrameRate: 60,
      operatingRange: { min: 0.1, max: 10, units: 'meters' },
      ambientLight: 'full_sunlight',
      multiPathHandling: 'expert'
    }
  };

  const required = criteria[targetLevel];
  const sensor = specification.sensor;

  // Check resolution
  if (sensor.resolution.width < required.minResolution.width ||
      sensor.resolution.height < required.minResolution.height) {
    return false;
  }

  // Check frame rate
  if (sensor.frameRate < required.minFrameRate) {
    return false;
  }

  // Check depth range
  if (sensor.depthRange.min > required.operatingRange.min ||
      sensor.depthRange.max < required.operatingRange.max) {
    return false;
  }

  return true;
}

/**
 * Default export for convenience
 */
export default {
  WIA3DImageSensor,
  convertToPointCloud,
  voxelDownsample,
  exportToPCD,
  removeStatisticalOutliers,
  checkCompliance
};
