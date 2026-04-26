/**
 * WIA-SEMI-016 Electronic Skin SDK
 *
 * @packageDocumentation
 */

export * from './types';

import {
  DeviceConfiguration,
  DeviceSpecification,
  SensorReading,
  ArrayReading,
  CalibrationData,
  DeviceStatus,
  EventType,
  EventHandler,
  ComplianceResult,
  ProcessingOptions,
  ComputedFeatures,
  DeviceClass,
  WirelessProtocol
} from './types';

/**
 * Main Electronic Skin Device class
 */
export class ElectronicSkinDevice {
  private config: DeviceConfiguration;
  private spec: DeviceSpecification;
  private calibrationData: Map<string, CalibrationData> = new Map();
  private eventHandlers: Map<EventType, Set<EventHandler>> = new Map();
  private status: DeviceStatus;
  private connected: boolean = false;

  /**
   * Create a new Electronic Skin Device
   * @param specification Complete device specification
   */
  constructor(specification: DeviceSpecification) {
    this.spec = specification;
    this.config = specification.configuration;
    this.status = {
      connected: false,
      errors: [],
      warnings: [],
      lastUpdate: Date.now()
    };
  }

  /**
   * Connect to the device
   * @returns Promise that resolves when connected
   */
  async connect(): Promise<void> {
    // Implementation would handle actual device connection (BLE, USB, etc.)
    this.connected = true;
    this.status.connected = true;
    this.status.lastUpdate = Date.now();
    this.emit(EventType.CONNECTED, { deviceId: this.config.deviceId });
  }

  /**
   * Disconnect from the device
   */
  async disconnect(): Promise<void> {
    this.connected = false;
    this.status.connected = false;
    this.status.lastUpdate = Date.now();
    this.emit(EventType.DISCONNECTED, { deviceId: this.config.deviceId });
  }

  /**
   * Get current device status
   */
  getStatus(): DeviceStatus {
    return { ...this.status };
  }

  /**
   * Read sensor data
   * @param sensorId Optional specific sensor ID, or read all
   * @returns Single sensor reading or array reading
   */
  async readSensor(sensorId?: string): Promise<SensorReading | ArrayReading> {
    if (!this.connected) {
      throw new Error('Device not connected');
    }

    if (sensorId) {
      // Read specific sensor
      return this.readSingleSensor(sensorId);
    } else {
      // Read all sensors
      return this.readAllSensors();
    }
  }

  /**
   * Read a single sensor
   */
  private async readSingleSensor(sensorId: string): Promise<SensorReading> {
    // Simulated raw reading - in real implementation, this would read from device
    const rawValue = Math.random() * 100;
    const calibration = this.calibrationData.get(sensorId);
    const calibratedValue = calibration
      ? this.applyCal ibration(rawValue, calibration)
      : rawValue;

    return {
      timestamp: Date.now(),
      sensorId,
      rawValue,
      calibratedValue,
      unit: 'kPa',
      quality: 1.0
    };
  }

  /**
   * Read all sensors in array
   */
  private async readAllSensors(): Promise<ArrayReading> {
    const sensors: SensorReading[] = [];
    const timestamp = Date.now();

    for (let i = 0; i < this.config.sensorCount; i++) {
      const sensorId = `sensor_${i}`;
      const reading = await this.readSingleSensor(sensorId);
      reading.timestamp = timestamp;
      sensors.push(reading);
    }

    const features = this.computeFeatures(sensors);

    return {
      timestamp,
      sensors,
      features
    };
  }

  /**
   * Apply calibration to raw sensor value
   */
  private applyCalibration(rawValue: number, calibration: CalibrationData): number {
    switch (calibration.type) {
      case 'linear':
        // y = mx + b
        return calibration.coefficients[0] * rawValue + calibration.coefficients[1];

      case 'polynomial':
        // y = a0 + a1*x + a2*x^2 + ...
        return calibration.coefficients.reduce((sum, coef, i) => {
          return sum + coef * Math.pow(rawValue, i);
        }, 0);

      case 'lookup':
        // Linear interpolation in lookup table
        // coefficients are [x0, y0, x1, y1, x2, y2, ...]
        for (let i = 0; i < calibration.coefficients.length - 2; i += 2) {
          const x0 = calibration.coefficients[i];
          const y0 = calibration.coefficients[i + 1];
          const x1 = calibration.coefficients[i + 2];
          const y1 = calibration.coefficients[i + 3];

          if (rawValue >= x0 && rawValue <= x1) {
            const t = (rawValue - x0) / (x1 - x0);
            return y0 + t * (y1 - y0);
          }
        }
        return rawValue; // Out of range, return raw value

      default:
        return rawValue;
    }
  }

  /**
   * Compute features from sensor array
   */
  private computeFeatures(sensors: SensorReading[]): ComputedFeatures {
    const values = sensors.map(s => s.calibratedValue);
    const totalForce = values.reduce((sum, v) => sum + v, 0);
    const maxValue = Math.max(...values);
    const meanValue = totalForce / values.length;
    const variance = values.reduce((sum, v) => sum + Math.pow(v - meanValue, 2), 0) / values.length;
    const standardDeviation = Math.sqrt(variance);

    // Center of pressure (assuming sensors in grid)
    let sumX = 0, sumY = 0, sumForce = 0;
    const cols = Math.ceil(Math.sqrt(sensors.length));

    sensors.forEach((sensor, i) => {
      const x = i % cols;
      const y = Math.floor(i / cols);
      const force = sensor.calibratedValue;
      sumX += x * force;
      sumY += y * force;
      sumForce += force;
    });

    const centerOfPressureX = sumForce > 0 ? sumX / sumForce : 0;
    const centerOfPressureY = sumForce > 0 ? sumY / sumForce : 0;

    // Contact area (sensors above threshold)
    const threshold = maxValue * 0.1; // 10% of max
    const contactArea = values.filter(v => v > threshold).length;

    return {
      totalForce,
      centerOfPressureX,
      centerOfPressureY,
      contactArea,
      maxValue,
      meanValue,
      standardDeviation
    };
  }

  /**
   * Set calibration data for a sensor
   */
  setCalibration(calibration: CalibrationData): void {
    this.calibrationData.set(calibration.sensorId, calibration);
  }

  /**
   * Get calibration data for a sensor
   */
  getCalibration(sensorId: string): CalibrationData | undefined {
    return this.calibrationData.get(sensorId);
  }

  /**
   * Perform auto-calibration
   * @param duration Duration in milliseconds to collect baseline data
   */
  async autoCalibrate(duration: number = 1000): Promise<Map<string, CalibrationData>> {
    if (!this.connected) {
      throw new Error('Device not connected');
    }

    const samples: Map<string, number[]> = new Map();

    // Collect samples
    const startTime = Date.now();
    while (Date.now() - startTime < duration) {
      const reading = await this.readAllSensors() as ArrayReading;
      reading.sensors.forEach(sensor => {
        if (!samples.has(sensor.sensorId)) {
          samples.set(sensor.sensorId, []);
        }
        samples.get(sensor.sensorId)!.push(sensor.rawValue);
      });
      await new Promise(resolve => setTimeout(resolve, 100)); // 10 Hz sampling
    }

    // Compute calibration (simple offset correction)
    const calibrations = new Map<string, CalibrationData>();
    samples.forEach((values, sensorId) => {
      const mean = values.reduce((sum, v) => sum + v, 0) / values.length;
      const calibration: CalibrationData = {
        sensorId,
        type: 'linear',
        coefficients: [1, -mean], // slope=1, offset=-mean
        timestamp: Date.now()
      };
      this.setCalibration(calibration);
      calibrations.set(sensorId, calibration);
    });

    return calibrations;
  }

  /**
   * Check compliance with WIA-SEMI-016 standard
   */
  checkCompliance(): ComplianceResult {
    const requirements: ComplianceResult['requirements'] = [];
    let allPassed = true;

    // Check mechanical properties
    if (this.spec.mechanical.stretchability < 30) {
      allPassed = false;
      requirements.push({
        name: 'Stretchability',
        passed: false,
        actualValue: this.spec.mechanical.stretchability,
        requiredValue: '≥30%',
        message: 'Stretchability below minimum requirement'
      });
    } else {
      requirements.push({
        name: 'Stretchability',
        passed: true,
        actualValue: this.spec.mechanical.stretchability,
        requiredValue: '≥30%'
      });
    }

    if (this.spec.mechanical.durability < 10000) {
      allPassed = false;
      requirements.push({
        name: 'Durability',
        passed: false,
        actualValue: this.spec.mechanical.durability,
        requiredValue: '≥10,000 cycles',
        message: 'Durability cycles below minimum requirement'
      });
    } else {
      requirements.push({
        name: 'Durability',
        passed: true,
        actualValue: this.spec.mechanical.durability,
        requiredValue: '≥10,000 cycles'
      });
    }

    // Check pressure sensing (if applicable)
    if (this.spec.pressureSensing) {
      if (this.spec.pressureSensing.sensitivity < 0.1) {
        allPassed = false;
        requirements.push({
          name: 'Pressure Sensitivity',
          passed: false,
          actualValue: this.spec.pressureSensing.sensitivity,
          requiredValue: '≥0.1 kPa⁻¹',
          message: 'Pressure sensitivity below minimum'
        });
      } else {
        requirements.push({
          name: 'Pressure Sensitivity',
          passed: true,
          actualValue: this.spec.pressureSensing.sensitivity,
          requiredValue: '≥0.1 kPa⁻¹'
        });
      }

      if (this.spec.pressureSensing.hysteresis > 10) {
        allPassed = false;
        requirements.push({
          name: 'Hysteresis',
          passed: false,
          actualValue: this.spec.pressureSensing.hysteresis,
          requiredValue: '≤10%',
          message: 'Hysteresis exceeds maximum'
        });
      } else {
        requirements.push({
          name: 'Hysteresis',
          passed: true,
          actualValue: this.spec.pressureSensing.hysteresis,
          requiredValue: '≤10%'
        });
      }
    }

    // Check electrical characteristics
    if (this.spec.electrical.snr < 40) {
      allPassed = false;
      requirements.push({
        name: 'Signal-to-Noise Ratio',
        passed: false,
        actualValue: this.spec.electrical.snr,
        requiredValue: '≥40 dB',
        message: 'SNR below minimum requirement'
      });
    } else {
      requirements.push({
        name: 'Signal-to-Noise Ratio',
        passed: true,
        actualValue: this.spec.electrical.snr,
        requiredValue: '≥40 dB'
      });
    }

    // Check biocompatibility (for medical devices)
    if (this.config.deviceClass === DeviceClass.MEDICAL && this.spec.biocompatibility) {
      if (!this.spec.biocompatibility.iso10993Compliant) {
        allPassed = false;
        requirements.push({
          name: 'ISO 10993 Compliance',
          passed: false,
          actualValue: false,
          requiredValue: true,
          message: 'ISO 10993 compliance required for medical devices'
        });
      } else {
        requirements.push({
          name: 'ISO 10993 Compliance',
          passed: true,
          actualValue: true,
          requiredValue: true
        });
      }
    }

    return {
      compliant: allPassed,
      standardVersion: '1.0',
      requirements,
      timestamp: Date.now()
    };
  }

  /**
   * Register event handler
   */
  on(event: EventType, handler: EventHandler): void {
    if (!this.eventHandlers.has(event)) {
      this.eventHandlers.set(event, new Set());
    }
    this.eventHandlers.get(event)!.add(handler);
  }

  /**
   * Unregister event handler
   */
  off(event: EventType, handler: EventHandler): void {
    if (this.eventHandlers.has(event)) {
      this.eventHandlers.get(event)!.delete(handler);
    }
  }

  /**
   * Emit event to all registered handlers
   */
  private emit(event: EventType, data: any): void {
    if (this.eventHandlers.has(event)) {
      this.eventHandlers.get(event)!.forEach(handler => {
        try {
          handler(data);
        } catch (error) {
          console.error(`Error in event handler for ${event}:`, error);
        }
      });
    }
  }

  /**
   * Get device specification
   */
  getSpecification(): DeviceSpecification {
    return { ...this.spec };
  }

  /**
   * Get device configuration
   */
  getConfiguration(): DeviceConfiguration {
    return { ...this.config };
  }

  /**
   * Process sensor data with options
   */
  async processData(
    data: ArrayReading,
    options: ProcessingOptions = {}
  ): Promise<ArrayReading> {
    let processedSensors = [...data.sensors];

    // Baseline correction
    if (options.baselineCorrection) {
      const baselines = this.computeBaselines(processedSensors);
      processedSensors = processedSensors.map(sensor => ({
        ...sensor,
        calibratedValue: sensor.calibratedValue - (baselines.get(sensor.sensorId) || 0)
      }));
    }

    // Apply calibration
    if (options.applyCalibration !== false) { // default true
      processedSensors = processedSensors.map(sensor => {
        const calibration = this.calibrationData.get(sensor.sensorId);
        if (calibration) {
          return {
            ...sensor,
            calibratedValue: this.applyCalibration(sensor.rawValue, calibration)
          };
        }
        return sensor;
      });
    }

    // Compute features
    let features = data.features;
    if (options.computeFeatures !== false) { // default true
      features = this.computeFeatures(processedSensors);
    }

    return {
      timestamp: data.timestamp,
      sensors: processedSensors,
      features
    };
  }

  /**
   * Compute baseline values for sensors
   */
  private computeBaselines(sensors: SensorReading[]): Map<string, number> {
    // Simplified: In real implementation, would track baselines over time
    const baselines = new Map<string, number>();
    sensors.forEach(sensor => {
      baselines.set(sensor.sensorId, 0);
    });
    return baselines;
  }
}

/**
 * Utility functions
 */
export const utils = {
  /**
   * Convert pressure units
   */
  convertPressure(value: number, from: string, to: string): number {
    const toP a = {
      kPa: 1,
      Pa: 1000,
      psi: 6.89476,
      bar: 100,
      mmHg: 133.322 / 1000
    };

    if (!toP a[from] || !toP a[to]) {
      throw new Error(`Unsupported pressure unit: ${from} or ${to}`);
    }

    const inKPa = value * toP a[from];
    return inKPa / toP a[to];
  },

  /**
   * Validate device specification against WIA-SEMI-016
   */
  validateSpecification(spec: DeviceSpecification): string[] {
    const errors: string[] = [];

    if (spec.mechanical.stretchability < 30) {
      errors.push('Stretchability must be ≥30%');
    }
    if (spec.mechanical.durability < 10000) {
      errors.push('Durability must be ≥10,000 cycles');
    }
    if (spec.electrical.snr < 40) {
      errors.push('SNR must be ≥40 dB');
    }

    return errors;
  }
};

/**
 * Create a new Electronic Skin Device instance
 * @param specification Device specification
 * @returns ElectronicSkinDevice instance
 */
export function createDevice(specification: DeviceSpecification): ElectronicSkinDevice {
  return new ElectronicSkinDevice(specification);
}

/**
 * Default export
 */
export default {
  ElectronicSkinDevice,
  createDevice,
  utils
};
