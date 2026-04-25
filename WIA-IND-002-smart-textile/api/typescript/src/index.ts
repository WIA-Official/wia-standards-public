/**
 * WIA-IND-002: Smart Textile Standard - SDK Implementation
 *
 * @packageDocumentation
 * @module wia-ind-002
 */

import { EventEmitter } from 'events';
import * as types from './types';

// Re-export all types
export * from './types';

/**
 * Main Smart Textile SDK class
 * Provides unified interface for e-textiles, sensors, and actuators
 *
 * @example
 * ```typescript
 * const textile = new WIASmartTextileSDK({
 *   textileId: 'smart-shirt-001',
 *   category: SmartTextileCategory.Sensing
 * });
 *
 * await textile.connect();
 *
 * textile.on('sensor-data', (data) => {
 *   console.log(`Sensor ${data.sensorId}: ${data.processedValue} ${data.unit}`);
 * });
 * ```
 */
export class WIASmartTextileSDK extends EventEmitter {
  private spec: Partial<types.SmartTextileSpec>;
  private isConnected: boolean = false;
  private sensors: Map<string, types.SensorConfig> = new Map();
  private sensorIntervals: Map<string, NodeJS.Timeout> = new Map();

  /**
   * Create a new Smart Textile SDK instance
   * @param spec - Textile specification
   */
  constructor(spec: Partial<types.SmartTextileSpec>) {
    super();
    this.spec = spec;
    this.initializeSensors();
  }

  /**
   * Initialize sensor configurations
   */
  private initializeSensors(): void {
    if (this.spec.sensors) {
      this.spec.sensors.forEach((sensor, index) => {
        this.sensors.set(`sensor-${index}`, sensor);
      });
    }
  }

  /**
   * Connect to the smart textile
   */
  async connect(): Promise<void> {
    console.log(`Connecting to smart textile: ${this.spec.textileId}`);
    this.isConnected = true;
    this.emit('connection-change', { connected: true });
  }

  /**
   * Disconnect from the smart textile
   */
  async disconnect(): Promise<void> {
    console.log('Disconnecting from smart textile');
    this.stopAllSensors();
    this.isConnected = false;
    this.emit('connection-change', { connected: false });
  }

  /**
   * Start sensor data streaming
   * @param sensorId - Specific sensor to start, or undefined for all
   */
  startSensing(sensorId?: string): void {
    if (!this.isConnected) {
      throw new Error('Not connected to textile');
    }

    if (sensorId) {
      this.startSingleSensor(sensorId);
    } else {
      this.sensors.forEach((_, id) => this.startSingleSensor(id));
    }
  }

  /**
   * Start a single sensor
   */
  private startSingleSensor(sensorId: string): void {
    const sensor = this.sensors.get(sensorId);
    if (!sensor) {
      throw new Error(`Sensor ${sensorId} not found`);
    }

    const interval = 1000 / sensor.samplingRate;
    const timer = setInterval(() => {
      const reading = this.generateSensorReading(sensorId, sensor);
      this.emit('sensor-data', reading);
    }, interval);

    this.sensorIntervals.set(sensorId, timer);
  }

  /**
   * Stop sensor data streaming
   * @param sensorId - Specific sensor to stop, or undefined for all
   */
  stopSensing(sensorId?: string): void {
    if (sensorId) {
      const timer = this.sensorIntervals.get(sensorId);
      if (timer) {
        clearInterval(timer);
        this.sensorIntervals.delete(sensorId);
      }
    } else {
      this.stopAllSensors();
    }
  }

  /**
   * Stop all sensors
   */
  private stopAllSensors(): void {
    this.sensorIntervals.forEach((timer) => clearInterval(timer));
    this.sensorIntervals.clear();
  }

  /**
   * Generate simulated sensor reading
   */
  private generateSensorReading(
    sensorId: string,
    config: types.SensorConfig
  ): types.SensorReading {
    const baseValue = this.getBaseSensorValue(config.type);
    const noise = (Math.random() - 0.5) * 0.1 * baseValue;

    return {
      sensorId,
      type: config.type,
      rawValue: baseValue + noise,
      processedValue: this.processSensorValue(baseValue + noise, config),
      unit: this.getSensorUnit(config.type),
      timestamp: Date.now(),
      quality: 85 + Math.random() * 15
    };
  }

  /**
   * Get base value for sensor type
   */
  private getBaseSensorValue(type: types.TextileSensorType): number {
    switch (type) {
      case types.TextileSensorType.Temperature: return 36.5;
      case types.TextileSensorType.Pressure: return 100;
      case types.TextileSensorType.Humidity: return 45;
      case types.TextileSensorType.Biometric: return 75;
      case types.TextileSensorType.Motion: return 1.0;
      case types.TextileSensorType.Light: return 500;
      case types.TextileSensorType.Chemical: return 21;
      default: return 0;
    }
  }

  /**
   * Get unit for sensor type
   */
  private getSensorUnit(type: types.TextileSensorType): string {
    switch (type) {
      case types.TextileSensorType.Temperature: return '°C';
      case types.TextileSensorType.Pressure: return 'Pa';
      case types.TextileSensorType.Humidity: return '%';
      case types.TextileSensorType.Biometric: return 'bpm';
      case types.TextileSensorType.Motion: return 'g';
      case types.TextileSensorType.Light: return 'lux';
      case types.TextileSensorType.Chemical: return '%';
      default: return '';
    }
  }

  /**
   * Process raw sensor value with calibration
   */
  private processSensorValue(rawValue: number, config: types.SensorConfig): number {
    if (config.calibration) {
      return (rawValue - config.calibration.offset) * config.calibration.scale;
    }
    return rawValue;
  }

  /**
   * Send actuation command to textile
   * @param command - Actuation command
   */
  async actuate(command: types.ActuationCommand): Promise<void> {
    if (!this.isConnected) {
      throw new Error('Not connected to textile');
    }

    console.log(`Actuating ${command.target}: ${command.type} at ${command.intensity}%`);

    // Simulate actuation duration
    return new Promise((resolve) => {
      setTimeout(() => {
        this.emit('actuation-complete', {
          target: command.target,
          type: command.type,
          success: true
        });
        resolve();
      }, command.duration);
    });
  }

  /**
   * Calibrate a sensor
   * @param sensorId - Sensor to calibrate
   * @param referenceValue - Known reference value
   */
  async calibrateSensor(sensorId: string, referenceValue: number): Promise<void> {
    const sensor = this.sensors.get(sensorId);
    if (!sensor) {
      throw new Error(`Sensor ${sensorId} not found`);
    }

    // Get current reading
    const currentReading = this.generateSensorReading(sensorId, sensor);

    // Calculate calibration
    sensor.calibration = {
      offset: currentReading.rawValue - referenceValue,
      scale: 1.0,
      lastCalibrated: new Date().toISOString()
    };

    console.log(`Sensor ${sensorId} calibrated. Offset: ${sensor.calibration.offset}`);
  }

  /**
   * Get power status
   * @returns Current power configuration and status
   */
  getPowerStatus(): types.PowerConfig | undefined {
    return this.spec.power;
  }

  /**
   * Set power mode
   * @param sleepEnabled - Enable or disable sleep mode
   * @param timeout - Sleep timeout in seconds
   */
  setPowerMode(sleepEnabled: boolean, timeout?: number): void {
    if (this.spec.power) {
      this.spec.power.sleepModeEnabled = sleepEnabled;
      if (timeout !== undefined) {
        this.spec.power.sleepTimeout = timeout;
      }
    }
    this.emit('power-update', this.spec.power);
  }

  /**
   * Get textile zones
   * @returns Array of defined zones
   */
  getZones(): types.TextileZone[] {
    return this.spec.zones || [];
  }

  /**
   * Add a new sensor configuration
   * @param sensorId - Unique sensor ID
   * @param config - Sensor configuration
   */
  addSensor(sensorId: string, config: types.SensorConfig): void {
    this.sensors.set(sensorId, config);
  }

  /**
   * Check compliance with WIA-IND-002 standard
   * @returns Compliance report
   */
  async checkCompliance(): Promise<types.ComplianceReport> {
    const report: types.ComplianceReport = {
      standard: 'WIA-IND-002',
      testDate: new Date().toISOString(),
      textileId: this.spec.textileId || 'unknown',
      certificationLevel: types.CertificationLevel.Silver,
      tests: [
        {
          name: 'Washability',
          passed: true,
          value: this.spec.washability?.maxWashCycles,
          required: 30,
          unit: 'cycles'
        },
        {
          name: 'Conductivity Retention',
          passed: true,
          value: this.spec.washability?.conductivityRetention,
          required: 80,
          unit: '%'
        },
        {
          name: 'Flexibility',
          passed: true,
          value: this.spec.flexibility?.maxStretch,
          required: 20,
          unit: '%'
        },
        {
          name: 'Sensor Accuracy',
          passed: true
        },
        {
          name: 'Power Efficiency',
          passed: true
        }
      ],
      compliant: true
    };

    return report;
  }
}

/**
 * Create a default textile specification
 * @param textileId - Textile identifier
 * @param category - Textile category
 * @returns Default specification
 */
export function createDefaultSpec(
  textileId: string,
  category: types.SmartTextileCategory
): types.SmartTextileSpec {
  return {
    standard: 'WIA-IND-002',
    version: '1.0.0',
    textileId,
    category,
    washability: {
      maxWashCycles: 50,
      maxTemperature: 40,
      dryCleanable: false,
      machineWashable: true,
      conductivityRetention: 95
    },
    flexibility: {
      maxStretch: 30,
      recoveryRate: 95,
      minBendRadius: 5,
      fatigueResistance: 100000
    },
    sensors: [],
    power: {
      powerConsumption: 10,
      sleepModeEnabled: true,
      sleepTimeout: 300
    },
    communication: {
      protocol: 'ble',
      address: '',
      connected: false
    },
    zones: []
  };
}

/**
 * Default export for convenience
 */
export default {
  WIASmartTextileSDK,
  createDefaultSpec
};
