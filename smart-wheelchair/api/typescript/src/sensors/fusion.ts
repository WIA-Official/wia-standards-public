/**
 * @file fusion.ts
 * @description WIA Smart Wheelchair Sensor Fusion & Obstacle Detection
 * @version 1.0.0
 */

import {
  Header,
  Vector2D,
  Vector3D,
  Quaternion,
  Pose,
  Pose2D,
  Twist,
  SensorSource,
  SensorCallback,
  CovarianceMatrix6x6,
  Disposable,
} from './types';
import { LaserScan } from './lidar';
import { IMUData } from './imu';

/*******************************************************************************
 * Odometry Types
 ******************************************************************************/

/**
 * Wheel encoder data
 */
export interface WheelEncoderData {
  header: Header;
  leftPosition: number;       // radians
  rightPosition: number;      // radians
  leftVelocity: number;       // rad/s
  rightVelocity: number;      // rad/s
}

/**
 * Odometry output
 */
export interface Odometry {
  header: Header;
  childFrameId: string;
  pose: Pose2D;
  velocity: {
    linear: number;           // m/s
    angular: number;          // rad/s
  };
  covariance: CovarianceMatrix6x6;
}

/**
 * Full pose with covariance
 */
export interface PoseWithCovariance {
  pose: Pose;
  covariance: CovarianceMatrix6x6;
}

/*******************************************************************************
 * Sensor Fusion
 ******************************************************************************/

/**
 * Fusion algorithm types
 */
export type FusionAlgorithm = 'ekf' | 'ukf' | 'particle_filter' | 'complementary';

/**
 * Fusion configuration
 */
export interface SensorFusionConfig {
  algorithm: FusionAlgorithm;
  updateRate: number;           // Hz
  wheelBase: number;            // meters (distance between wheels)
  wheelRadius: number;          // meters

  // Process noise
  processNoisePosition: number;
  processNoiseOrientation: number;
  processNoiseVelocity: number;

  // Initial covariance
  initialCovariance: number[];
}

/**
 * Sensor Fusion Interface
 */
export interface SensorFusionInterface extends Disposable {
  /** Current configuration */
  readonly config: SensorFusionConfig;

  /** Register sensor source */
  registerSource(source: SensorSource): void;

  /** Unregister sensor source */
  unregisterSource(sourceId: string): void;

  /** Update with IMU data */
  updateIMU(data: IMUData): void;

  /** Update with encoder data */
  updateEncoders(data: WheelEncoderData): void;

  /** Update with external pose (e.g., from localization) */
  updatePose(pose: PoseWithCovariance): void;

  /** Get current odometry estimate */
  getOdometry(): Odometry;

  /** Get current pose */
  getPose(): Pose2D;

  /** Get velocity */
  getVelocity(): { linear: number; angular: number };

  /** Get covariance matrix */
  getCovariance(): CovarianceMatrix6x6;

  /** Reset the filter */
  reset(initialPose?: Pose2D): void;

  /** Register odometry callback */
  onOdometry(callback: SensorCallback<Odometry>): void;
}

/**
 * Simple differential drive odometry calculator
 */
export class DifferentialDriveOdometry {
  private pose: Pose2D = { x: 0, y: 0, theta: 0 };
  private velocity = { linear: 0, angular: 0 };
  private lastLeftPos: number | null = null;
  private lastRightPos: number | null = null;
  private lastTime: number | null = null;

  constructor(
    private wheelBase: number,
    private wheelRadius: number
  ) {}

  /**
   * Update odometry from encoder readings
   */
  update(encoders: WheelEncoderData): Odometry {
    const now = encoders.header.timestamp;

    if (this.lastLeftPos === null || this.lastRightPos === null || this.lastTime === null) {
      this.lastLeftPos = encoders.leftPosition;
      this.lastRightPos = encoders.rightPosition;
      this.lastTime = now;
      return this.createOdometry(encoders.header);
    }

    const dt = (now - this.lastTime) / 1000;  // seconds
    if (dt <= 0) {
      return this.createOdometry(encoders.header);
    }

    // Calculate wheel displacements
    const leftDelta = encoders.leftPosition - this.lastLeftPos;
    const rightDelta = encoders.rightPosition - this.lastRightPos;

    // Convert to linear displacements
    const leftDist = leftDelta * this.wheelRadius;
    const rightDist = rightDelta * this.wheelRadius;

    // Calculate robot displacement
    const linearDist = (leftDist + rightDist) / 2;
    const angularDist = (rightDist - leftDist) / this.wheelBase;

    // Update pose
    if (Math.abs(angularDist) < 1e-6) {
      // Straight line motion
      this.pose.x += linearDist * Math.cos(this.pose.theta);
      this.pose.y += linearDist * Math.sin(this.pose.theta);
    } else {
      // Arc motion
      const radius = linearDist / angularDist;
      this.pose.x += radius * (Math.sin(this.pose.theta + angularDist) - Math.sin(this.pose.theta));
      this.pose.y += radius * (Math.cos(this.pose.theta) - Math.cos(this.pose.theta + angularDist));
      this.pose.theta += angularDist;
    }

    // Normalize theta to [-pi, pi]
    this.pose.theta = normalizeAngle(this.pose.theta);

    // Calculate velocities
    this.velocity.linear = linearDist / dt;
    this.velocity.angular = angularDist / dt;

    // Store for next iteration
    this.lastLeftPos = encoders.leftPosition;
    this.lastRightPos = encoders.rightPosition;
    this.lastTime = now;

    return this.createOdometry(encoders.header);
  }

  private createOdometry(header: Header): Odometry {
    return {
      header: { ...header, frameId: 'odom' },
      childFrameId: 'base_link',
      pose: { ...this.pose },
      velocity: { ...this.velocity },
      covariance: createDiagonalCovariance([0.01, 0.01, 0, 0, 0, 0.01]),
    };
  }

  /**
   * Reset odometry
   */
  reset(pose?: Pose2D): void {
    this.pose = pose ?? { x: 0, y: 0, theta: 0 };
    this.velocity = { linear: 0, angular: 0 };
    this.lastLeftPos = null;
    this.lastRightPos = null;
    this.lastTime = null;
  }

  getPose(): Pose2D {
    return { ...this.pose };
  }

  getVelocity(): { linear: number; angular: number } {
    return { ...this.velocity };
  }
}

/*******************************************************************************
 * Obstacle Detection
 ******************************************************************************/

/**
 * Obstacle type
 */
export type ObstacleType = 'static' | 'dynamic' | 'unknown';

/**
 * Detected obstacle
 */
export interface Obstacle {
  id: number;
  type: ObstacleType;
  position: Vector2D;           // relative to robot base
  velocity?: Vector2D;          // for dynamic obstacles
  size: {
    width: number;              // meters
    depth: number;              // meters
  };
  confidence: number;           // 0.0 - 1.0
  lastSeen: number;             // timestamp
}

/**
 * Zone status
 */
export type ZoneState = 'clear' | 'warning' | 'danger';

/**
 * Zone status for all directions
 */
export interface ZoneStatus {
  front: ZoneState;
  frontLeft: ZoneState;
  frontRight: ZoneState;
  left: ZoneState;
  right: ZoneState;
  rear: ZoneState;
}

/**
 * Obstacle detector configuration
 */
export interface ObstacleDetectorConfig {
  detectionRange: number;       // meters
  safetyMargin: number;         // meters
  updateRate: number;           // Hz

  // Zone thresholds
  dangerDistance: number;       // meters
  warningDistance: number;      // meters

  // Clustering
  clusterTolerance: number;     // meters
  minClusterSize: number;       // points

  // Tracking
  trackingTimeout: number;      // ms
  maxTrackedObstacles: number;
}

/**
 * Default obstacle detector config
 */
export const DEFAULT_OBSTACLE_CONFIG: ObstacleDetectorConfig = {
  detectionRange: 5.0,
  safetyMargin: 0.3,
  updateRate: 20,
  dangerDistance: 0.5,
  warningDistance: 1.5,
  clusterTolerance: 0.1,
  minClusterSize: 3,
  trackingTimeout: 500,
  maxTrackedObstacles: 50,
};

/**
 * Obstacle Detector Interface
 */
export interface ObstacleDetectorInterface extends Disposable {
  /** Current configuration */
  readonly config: ObstacleDetectorConfig;

  /** Update with laser scan */
  updateScan(scan: LaserScan): void;

  /** Get all detected obstacles */
  getObstacles(): Obstacle[];

  /** Get zone status */
  getZoneStatus(): ZoneStatus;

  /** Check if path is clear */
  isPathClear(direction: number, distance: number): boolean;

  /** Get closest obstacle in direction */
  getClosestObstacle(direction?: number, fov?: number): Obstacle | null;

  /** Register obstacle callback */
  onObstacle(callback: SensorCallback<Obstacle[]>): void;

  /** Register zone status callback */
  onZoneStatus(callback: SensorCallback<ZoneStatus>): void;
}

/**
 * Simple obstacle detector implementation
 */
export class SimpleObstacleDetector implements ObstacleDetectorInterface {
  readonly config: ObstacleDetectorConfig;
  private obstacles: Obstacle[] = [];
  private zoneStatus: ZoneStatus = {
    front: 'clear',
    frontLeft: 'clear',
    frontRight: 'clear',
    left: 'clear',
    right: 'clear',
    rear: 'clear',
  };
  private obstacleCallbacks: SensorCallback<Obstacle[]>[] = [];
  private zoneCallbacks: SensorCallback<ZoneStatus>[] = [];
  private nextObstacleId = 1;

  constructor(config: Partial<ObstacleDetectorConfig> = {}) {
    this.config = { ...DEFAULT_OBSTACLE_CONFIG, ...config };
  }

  updateScan(scan: LaserScan): void {
    // Extract points from scan
    const points: Vector2D[] = [];

    for (let i = 0; i < scan.ranges.length; i++) {
      const range = scan.ranges[i];
      if (!isFinite(range) || range < scan.rangeMin || range > scan.rangeMax) {
        continue;
      }
      if (range > this.config.detectionRange) {
        continue;
      }

      const angle = scan.angleMin + i * scan.angleIncrement;
      points.push({
        x: range * Math.cos(angle),
        y: range * Math.sin(angle),
      });
    }

    // Simple clustering (Euclidean)
    const clusters = this.clusterPoints(points);

    // Convert clusters to obstacles
    this.obstacles = clusters.map(cluster => this.clusterToObstacle(cluster, scan.header.timestamp));

    // Update zone status
    this.updateZoneStatus(scan);

    // Notify callbacks
    this.obstacleCallbacks.forEach(cb => cb(this.obstacles));
    this.zoneCallbacks.forEach(cb => cb(this.zoneStatus));
  }

  private clusterPoints(points: Vector2D[]): Vector2D[][] {
    const clusters: Vector2D[][] = [];
    const visited = new Set<number>();

    for (let i = 0; i < points.length; i++) {
      if (visited.has(i)) continue;

      const cluster: Vector2D[] = [points[i]];
      visited.add(i);

      // Find neighbors
      for (let j = i + 1; j < points.length; j++) {
        if (visited.has(j)) continue;

        const dist = this.distance(points[i], points[j]);
        if (dist < this.config.clusterTolerance) {
          cluster.push(points[j]);
          visited.add(j);
        }
      }

      if (cluster.length >= this.config.minClusterSize) {
        clusters.push(cluster);
      }
    }

    return clusters.slice(0, this.config.maxTrackedObstacles);
  }

  private clusterToObstacle(cluster: Vector2D[], timestamp: number): Obstacle {
    // Calculate centroid
    let sumX = 0, sumY = 0;
    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;

    for (const p of cluster) {
      sumX += p.x;
      sumY += p.y;
      minX = Math.min(minX, p.x);
      maxX = Math.max(maxX, p.x);
      minY = Math.min(minY, p.y);
      maxY = Math.max(maxY, p.y);
    }

    return {
      id: this.nextObstacleId++,
      type: 'unknown',
      position: {
        x: sumX / cluster.length,
        y: sumY / cluster.length,
      },
      size: {
        width: maxY - minY + this.config.safetyMargin,
        depth: maxX - minX + this.config.safetyMargin,
      },
      confidence: Math.min(1.0, cluster.length / 20),
      lastSeen: timestamp,
    };
  }

  private updateZoneStatus(scan: LaserScan): void {
    // Define zone angles (in radians)
    const zones = {
      front: { min: -Math.PI / 6, max: Math.PI / 6 },
      frontLeft: { min: Math.PI / 6, max: Math.PI / 2 },
      frontRight: { min: -Math.PI / 2, max: -Math.PI / 6 },
      left: { min: Math.PI / 2, max: Math.PI * 5 / 6 },
      right: { min: -Math.PI * 5 / 6, max: -Math.PI / 2 },
      rear: { min: Math.PI * 5 / 6, max: Math.PI, min2: -Math.PI, max2: -Math.PI * 5 / 6 },
    };

    for (const [zoneName, zone] of Object.entries(zones)) {
      let minDist = Infinity;

      for (let i = 0; i < scan.ranges.length; i++) {
        const range = scan.ranges[i];
        if (!isFinite(range) || range < scan.rangeMin || range > scan.rangeMax) {
          continue;
        }

        const angle = scan.angleMin + i * scan.angleIncrement;
        const inZone = (angle >= zone.min && angle <= zone.max) ||
                       ('min2' in zone && angle >= (zone as any).min2 && angle <= (zone as any).max2);

        if (inZone) {
          minDist = Math.min(minDist, range);
        }
      }

      let state: ZoneState = 'clear';
      if (minDist <= this.config.dangerDistance) {
        state = 'danger';
      } else if (minDist <= this.config.warningDistance) {
        state = 'warning';
      }

      (this.zoneStatus as any)[zoneName] = state;
    }
  }

  private distance(a: Vector2D, b: Vector2D): number {
    return Math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2);
  }

  getObstacles(): Obstacle[] {
    return [...this.obstacles];
  }

  getZoneStatus(): ZoneStatus {
    return { ...this.zoneStatus };
  }

  isPathClear(direction: number, distance: number): boolean {
    const halfWidth = 0.4;  // Robot half-width

    for (const obs of this.obstacles) {
      const obsAngle = Math.atan2(obs.position.y, obs.position.x);
      const obsDist = Math.sqrt(obs.position.x ** 2 + obs.position.y ** 2);

      const angleDiff = Math.abs(normalizeAngle(obsAngle - direction));
      if (angleDiff < Math.PI / 4 && obsDist < distance) {
        // Check if obstacle is in path
        const lateralDist = obsDist * Math.sin(angleDiff);
        if (lateralDist < halfWidth + obs.size.width / 2) {
          return false;
        }
      }
    }

    return true;
  }

  getClosestObstacle(direction: number = 0, fov: number = Math.PI): Obstacle | null {
    let closest: Obstacle | null = null;
    let minDist = Infinity;

    for (const obs of this.obstacles) {
      const obsAngle = Math.atan2(obs.position.y, obs.position.x);
      const angleDiff = Math.abs(normalizeAngle(obsAngle - direction));

      if (angleDiff <= fov / 2) {
        const dist = Math.sqrt(obs.position.x ** 2 + obs.position.y ** 2);
        if (dist < minDist) {
          minDist = dist;
          closest = obs;
        }
      }
    }

    return closest;
  }

  onObstacle(callback: SensorCallback<Obstacle[]>): void {
    this.obstacleCallbacks.push(callback);
  }

  onZoneStatus(callback: SensorCallback<ZoneStatus>): void {
    this.zoneCallbacks.push(callback);
  }

  dispose(): void {
    this.obstacleCallbacks = [];
    this.zoneCallbacks = [];
    this.obstacles = [];
  }
}

/*******************************************************************************
 * Utility Functions
 ******************************************************************************/

/**
 * Normalize angle to [-pi, pi]
 */
export function normalizeAngle(angle: number): number {
  while (angle > Math.PI) angle -= 2 * Math.PI;
  while (angle < -Math.PI) angle += 2 * Math.PI;
  return angle;
}

/**
 * Create diagonal covariance matrix
 */
export function createDiagonalCovariance(diag: number[]): CovarianceMatrix6x6 {
  const cov = new Array(36).fill(0) as CovarianceMatrix6x6;
  for (let i = 0; i < 6; i++) {
    cov[i * 6 + i] = diag[i] ?? 0;
  }
  return cov;
}

/**
 * Transform pose by delta
 */
export function transformPose(pose: Pose2D, delta: Pose2D): Pose2D {
  const cos = Math.cos(pose.theta);
  const sin = Math.sin(pose.theta);

  return {
    x: pose.x + delta.x * cos - delta.y * sin,
    y: pose.y + delta.x * sin + delta.y * cos,
    theta: normalizeAngle(pose.theta + delta.theta),
  };
}
