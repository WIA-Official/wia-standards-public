/**
 * @file types.ts
 * @description WIA Smart Wheelchair Sensor Common Types
 * @version 1.0.0
 */

/**
 * 3D Vector
 */
export interface Vector3D {
  x: number;
  y: number;
  z: number;
}

/**
 * 2D Vector
 */
export interface Vector2D {
  x: number;
  y: number;
}

/**
 * Quaternion for orientation
 */
export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

/**
 * Standard message header
 */
export interface Header {
  timestamp: number;      // Unix timestamp in milliseconds
  frameId: string;        // TF frame ID
  seq?: number;           // Sequence number
}

/**
 * 3D Pose
 */
export interface Pose {
  position: Vector3D;
  orientation: Quaternion;
}

/**
 * 2D Pose (for planar navigation)
 */
export interface Pose2D {
  x: number;
  y: number;
  theta: number;          // radians
}

/**
 * Twist (velocity)
 */
export interface Twist {
  linear: Vector3D;
  angular: Vector3D;
}

/**
 * Point cloud data
 */
export interface PointCloud {
  header: Header;
  points: Vector3D[];
  intensities?: number[];
  colors?: { r: number; g: number; b: number }[];
}

/**
 * Covariance matrix (flattened)
 */
export type CovarianceMatrix6x6 = [
  number, number, number, number, number, number,
  number, number, number, number, number, number,
  number, number, number, number, number, number,
  number, number, number, number, number, number,
  number, number, number, number, number, number,
  number, number, number, number, number, number
];

/**
 * Sensor status
 */
export enum SensorStatus {
  DISCONNECTED = 0,
  INITIALIZING = 1,
  READY = 2,
  RUNNING = 3,
  ERROR = 4,
  CALIBRATING = 5,
}

/**
 * Base sensor configuration
 */
export interface BaseSensorConfig {
  enabled: boolean;
  frameId: string;
  updateRate: number;     // Hz
}

/**
 * Sensor source for fusion
 */
export interface SensorSource {
  id: string;
  type: 'lidar' | 'camera' | 'imu' | 'encoder' | 'gps';
  trustLevel: number;     // 0.0 - 1.0
}

/**
 * Event emitter type helper
 */
export type SensorCallback<T> = (data: T) => void;

/**
 * Disposable interface
 */
export interface Disposable {
  dispose(): void;
}
