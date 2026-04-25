/**
 * WIA-QUA-004: Quantum Sensor SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum Research Group
 *
 * 弘익人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for quantum sensors including:
 * - Atomic clocks for ultra-precise timekeeping
 * - Quantum magnetometers for magnetic field sensing
 * - Quantum gravimeters for gravity measurements
 * - Quantum accelerometers and gyroscopes for inertial sensing
 * - Quantum imaging systems
 * - Quantum radar systems
 */

import {
  SensorType,
  SensorConfig,
  SensorInit,
  AtomicClockConfig,
  AtomicClockMeasurement,
  MagnetometerConfig,
  MagnetometerMeasurement,
  GravimeterConfig,
  GravimeterMeasurement,
  AccelerometerConfig,
  GyroscopeConfig,
  InertialMeasurement,
  QuantumImagingConfig,
  QuantumImageMeasurement,
  QuantumRadarConfig,
  QuantumRadarMeasurement,
  MeasurementRequest,
  MeasurementResult,
  CalibrationResult,
  QualityMetrics,
  Vector3,
  MagneticField,
  QUANTUM_CONSTANTS,
  GYROMAGNETIC_RATIOS,
  QuantumSensorErrorCode,
  QuantumSensorError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-QUA-004 Quantum Sensor SDK
 */
export class QuantumSensorSDK {
  private version = '1.0.0';
  private initialized = false;
  private sensors: Map<string, SensorConfig> = new Map();

  constructor() {
    this.initialized = true;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Register a sensor
   */
  registerSensor(config: SensorConfig): string {
    const sensorId = `${config.sensorType}-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
    this.sensors.set(sensorId, config);
    return sensorId;
  }

  /**
   * Get sensor configuration
   */
  getSensor(sensorId: string): SensorConfig | undefined {
    return this.sensors.get(sensorId);
  }

  /**
   * Create atomic clock instance
   */
  createAtomicClock(config: Omit<AtomicClockConfig, 'sensorType'>): AtomicClock {
    const fullConfig: AtomicClockConfig = {
      ...config,
      sensorType: 'atomic-clock',
    };
    return new AtomicClock(fullConfig);
  }

  /**
   * Create magnetometer instance
   */
  createMagnetometer(config: Omit<MagnetometerConfig, 'sensorType'>): QuantumMagnetometer {
    const fullConfig: MagnetometerConfig = {
      ...config,
      sensorType: 'magnetometer',
    };
    return new QuantumMagnetometer(fullConfig);
  }

  /**
   * Create gravimeter instance
   */
  createGravimeter(config: Omit<GravimeterConfig, 'sensorType'>): QuantumGravimeter {
    const fullConfig: GravimeterConfig = {
      ...config,
      sensorType: 'gravimeter',
    };
    return new QuantumGravimeter(fullConfig);
  }

  /**
   * Create accelerometer instance
   */
  createAccelerometer(config: Omit<AccelerometerConfig, 'sensorType'>): QuantumAccelerometer {
    const fullConfig: AccelerometerConfig = {
      ...config,
      sensorType: 'accelerometer',
    };
    return new QuantumAccelerometer(fullConfig);
  }

  /**
   * Create gyroscope instance
   */
  createGyroscope(config: Omit<GyroscopeConfig, 'sensorType'>): QuantumGyroscope {
    const fullConfig: GyroscopeConfig = {
      ...config,
      sensorType: 'gyroscope',
    };
    return new QuantumGyroscope(fullConfig);
  }

  /**
   * Create quantum imaging system
   */
  createImagingSystem(config: Omit<QuantumImagingConfig, 'sensorType'>): QuantumImaging {
    const fullConfig: QuantumImagingConfig = {
      ...config,
      sensorType: 'imaging',
    };
    return new QuantumImaging(fullConfig);
  }

  /**
   * Create quantum radar system
   */
  createRadarSystem(config: Omit<QuantumRadarConfig, 'sensorType'>): QuantumRadar {
    const fullConfig: QuantumRadarConfig = {
      ...config,
      sensorType: 'radar',
    };
    return new QuantumRadar(fullConfig);
  }
}

// ============================================================================
// Atomic Clock
// ============================================================================

/**
 * Atomic Clock implementation
 */
export class AtomicClock {
  private config: AtomicClockConfig;
  private isWarmedUp = false;

  constructor(config: AtomicClockConfig) {
    this.config = config;
  }

  /**
   * Initialize and warm up the atomic clock
   */
  async initialize(): Promise<void> {
    // Simulate warmup time
    await this.sleep(this.config.warmupTime * 1000);
    this.isWarmedUp = true;
  }

  /**
   * Measure time with quantum precision
   */
  async measureTime(params?: {
    integrationTime?: number;
    repetitions?: number;
  }): Promise<AtomicClockMeasurement> {
    if (!this.isWarmedUp) {
      throw new QuantumSensorError(
        QuantumSensorErrorCode.SENSOR_NOT_INITIALIZED,
        'Atomic clock not initialized. Call initialize() first.'
      );
    }

    const integrationTime = params?.integrationTime || 1000; // ms
    const repetitions = params?.repetitions || 1;

    // Simulate measurement
    await this.sleep(integrationTime * repetitions);

    const now = new Date();
    const frequency = this.config.transitionFrequency;
    const uncertainty = this.config.accuracy * frequency;

    // Calculate Allan deviation for different averaging times
    const allanDeviation = {
      '1s': this.config.stability,
      '100s': this.config.stability / Math.sqrt(100),
      '10000s': this.config.stability / Math.sqrt(10000),
    };

    return {
      measurementId: `AC-${now.toISOString()}-${Math.random().toString(36).substr(2, 9)}`,
      timestamp: now,
      atomType: this.config.atomType,
      frequency,
      uncertainty,
      fractionalAccuracy: this.config.accuracy,
      allanDeviation,
      environmental: {
        temperature: this.config.temperature,
        magneticField: 1e-6, // Typical residual field in µT
        pressure: 1e-11, // Ultra-high vacuum in Torr
      },
      utc: now.toISOString(),
    };
  }

  /**
   * Get current configuration
   */
  getConfig(): AtomicClockConfig {
    return { ...this.config };
  }

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Quantum Magnetometer
// ============================================================================

/**
 * Quantum Magnetometer implementation
 */
export class QuantumMagnetometer {
  private config: MagnetometerConfig;
  private isInitialized = false;

  constructor(config: MagnetometerConfig) {
    this.config = config;
  }

  /**
   * Initialize the magnetometer
   */
  async initialize(): Promise<void> {
    if (this.config.coolingRequired) {
      // Simulate cooling to operating temperature
      await this.sleep(30000); // 30 seconds for SQUID cooling
    }
    this.isInitialized = true;
  }

  /**
   * Measure magnetic field
   */
  async measure(params?: {
    integrationTime?: number;
    bandwidth?: [number, number];
  }): Promise<MagnetometerMeasurement> {
    if (!this.isInitialized) {
      throw new QuantumSensorError(
        QuantumSensorErrorCode.SENSOR_NOT_INITIALIZED,
        'Magnetometer not initialized. Call initialize() first.'
      );
    }

    const integrationTime = params?.integrationTime || 100; // ms
    await this.sleep(integrationTime);

    const now = new Date();

    // Simulate magnetic field measurement (Earth's field + noise)
    const earthField = 50e-6; // 50 µT typical Earth's field
    const noise = (Math.random() - 0.5) * this.config.sensitivity * 2;

    const Bx = (Math.random() - 0.5) * earthField * 0.3;
    const By = (Math.random() - 0.5) * earthField * 0.3;
    const Bz = earthField + noise;
    const magnitude = Math.sqrt(Bx * Bx + By * By + Bz * Bz);

    const fieldVector: MagneticField = {
      x: Bx,
      y: By,
      z: Bz,
      magnitude,
      unit: 'Tesla',
      direction: {
        azimuth: Math.atan2(By, Bx) * 180 / Math.PI,
        elevation: Math.asin(Bz / magnitude) * 180 / Math.PI,
      },
    };

    const quality: QualityMetrics = {
      snr: 20 * Math.log10(magnitude / this.config.sensitivity),
      validity: true,
      flags: [],
    };

    return {
      measurementId: `MAG-${now.toISOString()}-${Math.random().toString(36).substr(2, 9)}`,
      timestamp: now,
      sensorType: this.config.type,
      fieldVector,
      uncertainty: this.config.sensitivity,
      bandwidth: this.config.bandwidth[1] - this.config.bandwidth[0],
      integrationTime,
      environmental: {
        temperature: this.config.operatingTemperature,
        shieldingFactor: this.config.type === 'SQUID' ? 1e6 : undefined,
      },
      quality,
    };
  }

  /**
   * Calibrate magnetometer at zero field
   */
  async calibrateZeroField(): Promise<CalibrationResult> {
    const now = new Date();

    return {
      calibrationId: `CAL-MAG-${now.toISOString()}`,
      sensorId: this.config.serialNumber,
      timestamp: now,
      procedure: 'zero-field-calibration',
      traceability: 'Magnetically shielded room',
      results: [
        {
          parameter: 'zero_offset_x',
          measuredValue: 1e-16,
          referenceValue: 0,
          deviation: 1e-16,
          pass: true,
        },
        {
          parameter: 'zero_offset_y',
          measuredValue: -5e-17,
          referenceValue: 0,
          deviation: 5e-17,
          pass: true,
        },
        {
          parameter: 'zero_offset_z',
          measuredValue: 2e-16,
          referenceValue: 0,
          deviation: 2e-16,
          pass: true,
        },
      ],
      uncertainty: {
        typeA: this.config.sensitivity * 0.1,
        typeB: {
          instrumental: this.config.sensitivity * 0.5,
          environmental: this.config.sensitivity * 0.3,
          model: this.config.sensitivity * 0.2,
        },
        combined: this.config.sensitivity,
        expanded: this.config.sensitivity * 2,
        coverageFactor: 2,
      },
      nextCalibrationDue: new Date(now.getTime() + 365 * 24 * 3600 * 1000), // 1 year
    };
  }

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Quantum Gravimeter
// ============================================================================

/**
 * Quantum Gravimeter implementation
 */
export class QuantumGravimeter {
  private config: GravimeterConfig;
  private isInitialized = false;

  constructor(config: GravimeterConfig) {
    this.config = config;
  }

  /**
   * Initialize the gravimeter
   */
  async initialize(): Promise<void> {
    // System startup and stabilization
    await this.sleep(60000); // 1 minute warmup
    this.isInitialized = true;
  }

  /**
   * Measure gravitational acceleration
   */
  async measureGravity(params?: {
    repetitions?: number;
    location?: { latitude: number; longitude: number; altitude: number };
  }): Promise<GravimeterMeasurement> {
    if (!this.isInitialized) {
      throw new QuantumSensorError(
        QuantumSensorErrorCode.SENSOR_NOT_INITIALIZED,
        'Gravimeter not initialized. Call initialize() first.'
      );
    }

    const repetitions = params?.repetitions || 10;
    const location = params?.location || {
      latitude: 0,
      longitude: 0,
      altitude: 0,
    };

    // Time for atom drops
    await this.sleep(repetitions / this.config.measurementRate * 1000);

    const now = new Date();

    // Standard gravity with small variation
    const g = QUANTUM_CONSTANTS.STANDARD_GRAVITY;
    const noise = (Math.random() - 0.5) * this.config.sensitivity * g * 2;
    const measuredG = g + noise;

    const quality: QualityMetrics = {
      snr: 20 * Math.log10(g / (this.config.sensitivity * g)),
      validity: true,
      flags: [],
      chiSquared: 1.05,
      residualsRms: this.config.sensitivity * g * 0.5,
    };

    return {
      measurementId: `GRAV-${now.toISOString()}-${Math.random().toString(36).substr(2, 9)}`,
      timestamp: now,
      gravity: {
        value: measuredG,
        uncertainty: this.config.sensitivity * g,
        unit: 'm/s²',
      },
      location: {
        ...location,
        geoid: 'WGS84',
      },
      gradient: {
        dg_dz: -3.086e-6,  // Free-air gradient
        dg_dx: 0,
        dg_dy: 0,
      },
      tides: {
        correction: -1.234e-7, // Simulated tidal correction
        model: 'ETGTAB',
      },
      quality,
    };
  }

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Quantum Accelerometer
// ============================================================================

/**
 * Quantum Accelerometer implementation
 */
export class QuantumAccelerometer {
  private config: AccelerometerConfig;

  constructor(config: AccelerometerConfig) {
    this.config = config;
  }

  /**
   * Measure acceleration
   */
  async measure(): Promise<Vector3> {
    await this.sleep(10); // Quick measurement

    // Simulate acceleration measurement
    const noise = this.config.noiseDensity;

    return {
      x: (Math.random() - 0.5) * noise * 2,
      y: (Math.random() - 0.5) * noise * 2,
      z: QUANTUM_CONSTANTS.STANDARD_GRAVITY + (Math.random() - 0.5) * noise * 2,
    };
  }

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Quantum Gyroscope
// ============================================================================

/**
 * Quantum Gyroscope implementation
 */
export class QuantumGyroscope {
  private config: GyroscopeConfig;

  constructor(config: GyroscopeConfig) {
    this.config = config;
  }

  /**
   * Measure angular velocity
   */
  async measure(): Promise<Vector3> {
    await this.sleep(10);

    const noise = this.config.sensitivity;

    return {
      x: (Math.random() - 0.5) * noise * 2,
      y: (Math.random() - 0.5) * noise * 2,
      z: (Math.random() - 0.5) * noise * 2,
    };
  }

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Quantum Imaging
// ============================================================================

/**
 * Quantum Imaging implementation
 */
export class QuantumImaging {
  private config: QuantumImagingConfig;

  constructor(config: QuantumImagingConfig) {
    this.config = config;
  }

  /**
   * Capture quantum image
   */
  async capture(params?: { photonCount?: number }): Promise<QuantumImageMeasurement> {
    const photonCount = params?.photonCount || this.config.photonBudget;
    const integrationTime = photonCount / 1000; // Assume 1000 photons/ms

    await this.sleep(integrationTime);

    const now = new Date();
    const snrEnhancement = this.config.squeezingDb
      ? Math.pow(10, this.config.squeezingDb / 20)
      : 1.0;

    return {
      imageId: `QI-${now.toISOString()}-${Math.random().toString(36).substr(2, 9)}`,
      type: this.config.mode,
      resolution: this.config.resolution,
      photonCount,
      integrationTime,
      squeezingDb: this.config.squeezingDb,
      snrEnhancement,
      wavelength: this.config.wavelength,
      data: 'base64_encoded_image_data', // Placeholder
    };
  }

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Quantum Radar
// ============================================================================

/**
 * Quantum Radar implementation
 */
export class QuantumRadar {
  private config: QuantumRadarConfig;

  constructor(config: QuantumRadarConfig) {
    this.config = config;
  }

  /**
   * Detect targets using quantum illumination
   */
  async detect(): Promise<QuantumRadarMeasurement> {
    await this.sleep(100); // Measurement cycle

    const now = new Date();
    const detected = Math.random() > 0.5;

    const quality: QualityMetrics = {
      snr: 15 + Math.random() * 10,
      validity: true,
      flags: [],
    };

    return {
      detectionId: `QR-${now.toISOString()}-${Math.random().toString(36).substr(2, 9)}`,
      timestamp: now,
      target: {
        detected,
        range: detected ? 1000 + Math.random() * 5000 : 0,
        velocity: detected ? (Math.random() - 0.5) * 200 : 0,
        azimuth: detected ? Math.random() * 360 : 0,
        elevation: detected ? (Math.random() - 0.5) * 180 : 0,
        radarCrossSection: detected ? Math.random() * 10 : 0,
        confidence: detected ? 0.8 + Math.random() * 0.2 : 0,
      },
      quantumCorrelation: this.config.entanglementVisibility * (0.8 + Math.random() * 0.2),
      snr: quality.snr,
      falseAlarmRate: 1e-6,
      entanglementVisibility: this.config.entanglementVisibility,
      quality,
    };
  }

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Standalone Helper Functions
// ============================================================================

/**
 * Calculate quantum sensing sensitivity limit (Heisenberg limit)
 */
export function calculateHeisenbergLimit(numParticles: number): number {
  return 1 / numParticles;
}

/**
 * Calculate standard quantum limit (SQL)
 */
export function calculateStandardQuantumLimit(numParticles: number): number {
  return 1 / Math.sqrt(numParticles);
}

/**
 * Calculate Allan deviation for given averaging time
 */
export function calculateAllanDeviation(
  stability1s: number,
  averagingTime: number
): number {
  return stability1s / Math.sqrt(averagingTime);
}

/**
 * Convert magnetic field units
 */
export function convertMagneticField(
  value: number,
  from: 'Tesla' | 'Gauss' | 'nT',
  to: 'Tesla' | 'Gauss' | 'nT'
): number {
  const toTesla = {
    'Tesla': 1,
    'Gauss': 1e-4,
    'nT': 1e-9,
  };

  const fromTesla = {
    'Tesla': 1,
    'Gauss': 1e4,
    'nT': 1e9,
  };

  return value * toTesla[from] * fromTesla[to];
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export {
  QuantumSensorSDK,
  AtomicClock,
  QuantumMagnetometer,
  QuantumGravimeter,
  QuantumAccelerometer,
  QuantumGyroscope,
  QuantumImaging,
  QuantumRadar,
};

export default QuantumSensorSDK;

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
