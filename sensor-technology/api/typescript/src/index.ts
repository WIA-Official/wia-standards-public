/**
 * WIA-SEMI-012 Sensor Technology Standard
 * TypeScript SDK Main Export
 */

export * from './types';

import {
  ISensor,
  IAccelerometer,
  IGyroscope,
  IMagnetometer,
  IIMU,
  ITemperatureSensor,
  IHumiditySensor,
  IPressureSensor,
  IGasSensor,
  IImageSensor,
  SensorType,
  PowerMode,
  SensorError,
  SensorErrorCode,
  CalibrationData,
  CommunicationConfig,
} from './types';

/**
 * Sensor Factory
 * Creates sensor instances based on type
 */
export class SensorFactory {
  static create(type: SensorType, config: any): ISensor {
    // Implementation would create specific sensor instances
    throw new Error('Not implemented - extend for specific hardware');
  }
}

/**
 * Sensor Fusion Library
 * Combines multiple sensor readings for improved accuracy
 */
export class SensorFusion {
  /**
   * Complementary filter for orientation estimation
   * @param accel Accelerometer reading [x, y, z]
   * @param gyro Gyroscope reading [x, y, z]
   * @param dt Time delta in seconds
   * @param alpha Filter coefficient (0.95-0.98 typical)
   */
  static complementaryFilter(
    accel: [number, number, number],
    gyro: [number, number, number],
    dt: number,
    alpha: number = 0.98
  ): [number, number] {
    // Simplified implementation
    const accelPitch = Math.atan2(accel[1], accel[2]) * (180 / Math.PI);
    const accelRoll = Math.atan2(accel[0], accel[2]) * (180 / Math.PI);

    // In real implementation, would maintain state and apply gyro integration
    return [accelRoll, accelPitch];
  }

  /**
   * Calculate altitude from pressure
   * @param pressure Current pressure in hPa
   * @param seaLevelPressure Sea level pressure in hPa (default 1013.25)
   */
  static pressureToAltitude(
    pressure: number,
    seaLevelPressure: number = 1013.25
  ): number {
    return 44330 * (1 - Math.pow(pressure / seaLevelPressure, 1/5.255));
  }
}

/**
 * Calibration Utilities
 */
export class CalibrationUtils {
  /**
   * Six-position accelerometer calibration
   * @param readings Array of 6 readings (X+, X-, Y+, Y-, Z+, Z-)
   */
  static sixPositionCalibration(
    readings: Array<[number, number, number]>
  ): CalibrationData {
    if (readings.length !== 6) {
      throw new SensorError(
        'Six-position calibration requires exactly 6 readings',
        SensorErrorCode.INVALID_CONFIGURATION
      );
    }

    // Calculate bias and scale factors
    const bias = [0, 0, 0];
    const scale = [1, 1, 1];

    for (let axis = 0; axis < 3; axis++) {
      const plus = readings[axis * 2][axis];
      const minus = readings[axis * 2 + 1][axis];

      bias[axis] = (plus + minus) / 2;
      scale[axis] = 2 / (plus - minus); // Expected range is 2g (±1g)
    }

    return {
      sensorId: 'unknown',
      date: new Date(),
      method: 'six_position' as any,
      coefficients: {
        bias_x: bias[0],
        bias_y: bias[1],
        bias_z: bias[2],
        scale_x: scale[0],
        scale_y: scale[1],
        scale_z: scale[2],
      },
    };
  }

  /**
   * Apply calibration to raw sensor reading
   */
  static applyCalibration(
    raw: [number, number, number],
    calibration: CalibrationData
  ): [number, number, number] {
    const { coefficients } = calibration;

    return [
      (raw[0] - coefficients.bias_x) * coefficients.scale_x,
      (raw[1] - coefficients.bias_y) * coefficients.scale_y,
      (raw[2] - coefficients.bias_z) * coefficients.scale_z,
    ];
  }
}

/**
 * Data Processing Utilities
 */
export class DataProcessing {
  /**
   * Low-pass filter
   * @param newValue Current sensor reading
   * @param previousValue Previous filtered value
   * @param alpha Filter coefficient (0-1, lower = more filtering)
   */
  static lowPassFilter(
    newValue: number,
    previousValue: number,
    alpha: number
  ): number {
    return alpha * newValue + (1 - alpha) * previousValue;
  }

  /**
   * Moving average filter
   * @param values Array of recent values
   * @param windowSize Number of samples to average
   */
  static movingAverage(values: number[], windowSize: number): number {
    const window = values.slice(-windowSize);
    return window.reduce((a, b) => a + b, 0) / window.length;
  }

  /**
   * Calculate RMS (Root Mean Square)
   */
  static rms(values: number[]): number {
    const sumSquares = values.reduce((sum, val) => sum + val * val, 0);
    return Math.sqrt(sumSquares / values.length);
  }
}

/**
 * Unit Conversion Utilities
 */
export class UnitConversion {
  static celsiusToFahrenheit(celsius: number): number {
    return celsius * 9/5 + 32;
  }

  static fahrenheitToCelsius(fahrenheit: number): number {
    return (fahrenheit - 32) * 5/9;
  }

  static celsiusToKelvin(celsius: number): number {
    return celsius + 273.15;
  }

  static paToHpa(pa: number): number {
    return pa / 100;
  }

  static ppmToMgM3(ppm: number, molecularWeight: number, temperature: number = 25): number {
    // At 25°C and 1 atm
    return (ppm * molecularWeight) / 24.45;
  }
}

/**
 * Example IMU Implementation
 */
export class IMU implements IIMU {
  constructor(
    public accelerometer: IAccelerometer,
    public gyroscope: IGyroscope,
    public magnetometer?: IMagnetometer
  ) {}

  async getOrientation() {
    // Simplified - real implementation would use sensor fusion
    throw new Error('Not implemented');
  }

  async getEulerAngles() {
    const accelData = await this.accelerometer.read();
    const gyroData = await this.gyroscope.read();

    const [roll, pitch] = SensorFusion.complementaryFilter(
      accelData.value,
      gyroData.value,
      0.01 // 100 Hz sampling
    );

    return {
      roll,
      pitch,
      yaw: 0, // Would require magnetometer
    };
  }

  async fuseSensors() {
    // Implementation of sensor fusion algorithm
    // (Kalman filter, Madgwick, etc.)
    throw new Error('Not implemented');
  }
}

// Version information
export const VERSION = '1.0.0';
export const STANDARD = 'WIA-SEMI-012';
