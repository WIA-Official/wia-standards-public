/**
 * @file imu.ts
 * @description WIA Smart Wheelchair IMU Interface
 * @version 1.0.0
 */

import {
  Header,
  Vector3D,
  Quaternion,
  BaseSensorConfig,
  SensorStatus,
  SensorCallback,
  Disposable,
  CovarianceMatrix6x6,
} from './types';

/**
 * IMU configuration
 */
export interface IMUConfig extends BaseSensorConfig {
  model: IMUModel;
  accelRange: AccelRange;       // ±g
  gyroRange: GyroRange;         // ±dps
  sampleRate: number;           // Hz
  enableMagnetometer: boolean;
  enableTemperature: boolean;
  calibration?: IMUCalibration;
}

/**
 * Supported IMU models
 */
export type IMUModel =
  | 'mpu6050'
  | 'mpu9250'
  | 'bno055'
  | 'bno085'
  | 'icm20948'
  | 'lsm6ds3'
  | 'generic';

/**
 * Accelerometer range options
 */
export type AccelRange = 2 | 4 | 8 | 16;

/**
 * Gyroscope range options
 */
export type GyroRange = 250 | 500 | 1000 | 2000;

/**
 * IMU calibration data
 */
export interface IMUCalibration {
  accelBias: Vector3D;          // m/s²
  accelScale: Vector3D;         // scale factors
  gyroBias: Vector3D;           // rad/s
  magBias?: Vector3D;           // μT
  magScale?: Vector3D;          // scale factors
  temperature?: number;         // calibration temperature
}

/**
 * Raw IMU data
 */
export interface IMURawData {
  header: Header;
  accelRaw: Vector3D;           // raw ADC values
  gyroRaw: Vector3D;            // raw ADC values
  magRaw?: Vector3D;            // raw ADC values
  tempRaw?: number;             // raw ADC value
}

/**
 * Processed IMU data
 */
export interface IMUData {
  header: Header;
  accel: Vector3D;              // m/s²
  gyro: Vector3D;               // rad/s
  mag?: Vector3D;               // μT (microtesla)
  orientation?: Quaternion;     // from sensor fusion (if available)
  linearAccel?: Vector3D;       // gravity-compensated acceleration
  temperature?: number;         // °C
}

/**
 * Full IMU message (ROS-compatible)
 */
export interface IMUMessage {
  header: Header;
  orientation: Quaternion;
  orientationCovariance: number[];  // 3x3 row-major
  angularVelocity: Vector3D;
  angularVelocityCovariance: number[];  // 3x3 row-major
  linearAcceleration: Vector3D;
  linearAccelerationCovariance: number[];  // 3x3 row-major
}

/**
 * IMU diagnostic info
 */
export interface IMUDiagnostics {
  status: SensorStatus;
  accelSelfTest: boolean;
  gyroSelfTest: boolean;
  magSelfTest?: boolean;
  temperature: number;
  sampleRateActual: number;
  overruns: number;
  calibrationStatus: CalibrationStatus;
}

/**
 * Calibration status
 */
export interface CalibrationStatus {
  system: number;               // 0-3 (3 = fully calibrated)
  gyro: number;
  accel: number;
  mag?: number;
}

/**
 * IMU presets
 */
export const IMU_PRESETS: Record<IMUModel, Partial<IMUConfig>> = {
  mpu6050: {
    accelRange: 8,
    gyroRange: 1000,
    sampleRate: 200,
    enableMagnetometer: false,
  },
  mpu9250: {
    accelRange: 8,
    gyroRange: 1000,
    sampleRate: 200,
    enableMagnetometer: true,
  },
  bno055: {
    accelRange: 4,
    gyroRange: 2000,
    sampleRate: 100,
    enableMagnetometer: true,
  },
  bno085: {
    accelRange: 8,
    gyroRange: 2000,
    sampleRate: 400,
    enableMagnetometer: true,
  },
  icm20948: {
    accelRange: 8,
    gyroRange: 1000,
    sampleRate: 225,
    enableMagnetometer: true,
  },
  lsm6ds3: {
    accelRange: 8,
    gyroRange: 1000,
    sampleRate: 416,
    enableMagnetometer: false,
  },
  generic: {
    accelRange: 8,
    gyroRange: 1000,
    sampleRate: 100,
    enableMagnetometer: false,
  },
};

/**
 * IMU Interface
 */
export interface IMUInterface extends Disposable {
  /** Current configuration */
  readonly config: IMUConfig;

  /** Current status */
  readonly status: SensorStatus;

  /** Initialize IMU */
  initialize(): Promise<void>;

  /** Start data acquisition */
  start(): Promise<void>;

  /** Stop data acquisition */
  stop(): Promise<void>;

  /** Register data callback */
  onData(callback: SensorCallback<IMUData>): void;

  /** Remove data callback */
  offData(callback: SensorCallback<IMUData>): void;

  /** Get latest data */
  getLatestData(): IMUData | null;

  /** Get raw data (uncalibrated) */
  getRawData(): IMURawData | null;

  /** Get diagnostics */
  getDiagnostics(): IMUDiagnostics;

  /** Run self-test */
  selfTest(): Promise<boolean>;

  /** Calibrate IMU */
  calibrate(): Promise<IMUCalibration>;

  /** Apply calibration */
  applyCalibration(cal: IMUCalibration): void;

  /** Reset orientation (for sensors with fusion) */
  resetOrientation?(): Promise<void>;
}

/**
 * Apply calibration to raw IMU data
 */
export function applyCalibration(
  raw: IMURawData,
  cal: IMUCalibration
): IMUData {
  const accel: Vector3D = {
    x: (raw.accelRaw.x - cal.accelBias.x) * cal.accelScale.x,
    y: (raw.accelRaw.y - cal.accelBias.y) * cal.accelScale.y,
    z: (raw.accelRaw.z - cal.accelBias.z) * cal.accelScale.z,
  };

  const gyro: Vector3D = {
    x: raw.gyroRaw.x - cal.gyroBias.x,
    y: raw.gyroRaw.y - cal.gyroBias.y,
    z: raw.gyroRaw.z - cal.gyroBias.z,
  };

  let mag: Vector3D | undefined;
  if (raw.magRaw && cal.magBias && cal.magScale) {
    mag = {
      x: (raw.magRaw.x - cal.magBias.x) * cal.magScale.x,
      y: (raw.magRaw.y - cal.magBias.y) * cal.magScale.y,
      z: (raw.magRaw.z - cal.magBias.z) * cal.magScale.z,
    };
  }

  return {
    header: raw.header,
    accel,
    gyro,
    mag,
  };
}

/**
 * Calculate roll and pitch from accelerometer
 */
export function accelToRollPitch(accel: Vector3D): { roll: number; pitch: number } {
  const roll = Math.atan2(accel.y, accel.z);
  const pitch = Math.atan2(-accel.x, Math.sqrt(accel.y * accel.y + accel.z * accel.z));

  return { roll, pitch };
}

/**
 * Calculate yaw from magnetometer (with tilt compensation)
 */
export function magToYaw(
  mag: Vector3D,
  roll: number,
  pitch: number
): number {
  const cosRoll = Math.cos(roll);
  const sinRoll = Math.sin(roll);
  const cosPitch = Math.cos(pitch);
  const sinPitch = Math.sin(pitch);

  // Tilt-compensated magnetic field
  const magX = mag.x * cosPitch + mag.y * sinRoll * sinPitch + mag.z * cosRoll * sinPitch;
  const magY = mag.y * cosRoll - mag.z * sinRoll;

  return Math.atan2(-magY, magX);
}

/**
 * Convert Euler angles to quaternion (ZYX convention)
 */
export function eulerToQuaternion(
  roll: number,
  pitch: number,
  yaw: number
): Quaternion {
  const cy = Math.cos(yaw * 0.5);
  const sy = Math.sin(yaw * 0.5);
  const cp = Math.cos(pitch * 0.5);
  const sp = Math.sin(pitch * 0.5);
  const cr = Math.cos(roll * 0.5);
  const sr = Math.sin(roll * 0.5);

  return {
    w: cr * cp * cy + sr * sp * sy,
    x: sr * cp * cy - cr * sp * sy,
    y: cr * sp * cy + sr * cp * sy,
    z: cr * cp * sy - sr * sp * cy,
  };
}

/**
 * Convert quaternion to Euler angles (ZYX convention)
 */
export function quaternionToEuler(q: Quaternion): {
  roll: number;
  pitch: number;
  yaw: number;
} {
  // Roll (x-axis rotation)
  const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  const roll = Math.atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  const sinp = 2 * (q.w * q.y - q.z * q.x);
  const pitch = Math.abs(sinp) >= 1
    ? (Math.sign(sinp) * Math.PI / 2)
    : Math.asin(sinp);

  // Yaw (z-axis rotation)
  const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  const yaw = Math.atan2(siny_cosp, cosy_cosp);

  return { roll, pitch, yaw };
}

/**
 * Remove gravity from acceleration using orientation
 */
export function removeGravity(
  accel: Vector3D,
  orientation: Quaternion,
  gravity: number = 9.81
): Vector3D {
  // Rotate gravity vector by inverse of orientation
  const q = orientation;
  const gx = 2 * (q.x * q.z - q.w * q.y) * gravity;
  const gy = 2 * (q.w * q.x + q.y * q.z) * gravity;
  const gz = (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * gravity;

  return {
    x: accel.x - gx,
    y: accel.y - gy,
    z: accel.z - gz,
  };
}

/**
 * Simple complementary filter for orientation
 */
export class ComplementaryFilter {
  private orientation: Quaternion = { x: 0, y: 0, z: 0, w: 1 };
  private lastTimestamp: number = 0;
  private alpha: number;

  constructor(alpha: number = 0.98) {
    this.alpha = alpha;
  }

  update(imu: IMUData): Quaternion {
    const dt = this.lastTimestamp > 0
      ? (imu.header.timestamp - this.lastTimestamp) / 1000
      : 0;
    this.lastTimestamp = imu.header.timestamp;

    if (dt <= 0 || dt > 0.5) {
      // Initialize from accelerometer
      const { roll, pitch } = accelToRollPitch(imu.accel);
      const yaw = imu.mag ? magToYaw(imu.mag, roll, pitch) : 0;
      this.orientation = eulerToQuaternion(roll, pitch, yaw);
      return this.orientation;
    }

    // Integrate gyroscope
    const gyroQuat = this.integrateGyro(imu.gyro, dt);

    // Get orientation from accelerometer/magnetometer
    const { roll, pitch } = accelToRollPitch(imu.accel);
    const yaw = imu.mag
      ? magToYaw(imu.mag, roll, pitch)
      : quaternionToEuler(this.orientation).yaw;
    const accelQuat = eulerToQuaternion(roll, pitch, yaw);

    // Complementary filter: blend gyro and accel
    this.orientation = this.slerp(accelQuat, gyroQuat, this.alpha);

    return this.orientation;
  }

  private integrateGyro(gyro: Vector3D, dt: number): Quaternion {
    const halfDt = dt * 0.5;
    const dq: Quaternion = {
      w: 1,
      x: gyro.x * halfDt,
      y: gyro.y * halfDt,
      z: gyro.z * halfDt,
    };

    return this.multiplyQuaternion(this.orientation, dq);
  }

  private multiplyQuaternion(a: Quaternion, b: Quaternion): Quaternion {
    return {
      w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
  }

  private slerp(a: Quaternion, b: Quaternion, t: number): Quaternion {
    let dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;

    if (dot < 0) {
      b = { x: -b.x, y: -b.y, z: -b.z, w: -b.w };
      dot = -dot;
    }

    if (dot > 0.9995) {
      // Linear interpolation for very close quaternions
      const result = {
        x: a.x + t * (b.x - a.x),
        y: a.y + t * (b.y - a.y),
        z: a.z + t * (b.z - a.z),
        w: a.w + t * (b.w - a.w),
      };
      return this.normalizeQuaternion(result);
    }

    const theta = Math.acos(dot);
    const sinTheta = Math.sin(theta);
    const wa = Math.sin((1 - t) * theta) / sinTheta;
    const wb = Math.sin(t * theta) / sinTheta;

    return {
      x: wa * a.x + wb * b.x,
      y: wa * a.y + wb * b.y,
      z: wa * a.z + wb * b.z,
      w: wa * a.w + wb * b.w,
    };
  }

  private normalizeQuaternion(q: Quaternion): Quaternion {
    const norm = Math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return {
      x: q.x / norm,
      y: q.y / norm,
      z: q.z / norm,
      w: q.w / norm,
    };
  }

  reset(): void {
    this.orientation = { x: 0, y: 0, z: 0, w: 1 };
    this.lastTimestamp = 0;
  }
}
