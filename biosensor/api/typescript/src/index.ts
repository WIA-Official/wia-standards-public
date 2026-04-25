/**
 * WIA-BIO-011: Biosensor SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for biosensor development including:
 * - Sensor configuration and management
 * - Calibration and performance metrics
 * - Signal processing and measurements
 * - Quality control and validation
 */

import {
  BiosensorConfig,
  Biosensor,
  CalibrationRequest,
  CalibrationData,
  CalibrationPoint,
  MeasurementRequest,
  MeasurementResult,
  PerformanceMetrics,
  LODCalculation,
  LODResult,
  ValidationParameters,
  ValidationResult,
  QualityControlSample,
  QualityControlResult,
  SignalProcessingParams,
  ProcessedSignal,
  POCTestResult,
  QualityFlag,
  BIOSENSOR_CONSTANTS,
  BiosensorErrorCode,
  BiosensorError,
  BatchMeasurement,
  TimeSeries,
  TimeSeriesPoint,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-BIO-011 Biosensor SDK
 */
export class BiosensorSDK {
  private version = '1.0.0';
  private sensors: Map<string, Biosensor> = new Map();
  private measurements: Map<string, MeasurementResult[]> = new Map();

  constructor() {
    // Initialize SDK
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Create a new biosensor instance
   *
   * @param config - Biosensor configuration
   * @returns Biosensor instance
   */
  createSensor(config: BiosensorConfig): Biosensor {
    // Validate configuration
    this.validateConfig(config);

    // Generate sensor ID
    const id = `BS-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;

    // Create sensor instance
    const sensor: Biosensor = {
      id,
      config,
      status: 'idle',
      created: new Date(),
      measurementCount: 0,
    };

    // Set default lifetime for enzymatic sensors
    if (config.biorecognitionElement.includes('oxidase') ||
        config.biorecognitionElement.includes('ase')) {
      sensor.lifetime = {
        expirationDate: new Date(Date.now() + 180 * 24 * 60 * 60 * 1000), // 180 days
        maxMeasurements: 1000,
        storageConditions: '2-8°C, dry',
      };
    }

    // Store sensor
    this.sensors.set(id, sensor);

    return sensor;
  }

  /**
   * Get sensor by ID
   *
   * @param sensorId - Sensor identifier
   * @returns Biosensor instance
   */
  getSensor(sensorId: string): Biosensor {
    const sensor = this.sensors.get(sensorId);
    if (!sensor) {
      throw new BiosensorError(
        BiosensorErrorCode.SENSOR_NOT_FOUND,
        `Sensor ${sensorId} not found`
      );
    }
    return sensor;
  }

  /**
   * Calibrate a biosensor
   *
   * @param request - Calibration request
   * @returns Calibration data
   */
  calibrateSensor(request: CalibrationRequest): CalibrationData {
    const {
      sensorId,
      standardConcentrations,
      measuredSignals,
      concentrationUnit = 'mM',
      signalUnit = 'μA',
      calibrationType = 'linear',
    } = request;

    // Get sensor
    const sensor = this.getSensor(sensorId);
    sensor.status = 'calibrating';

    // Validate inputs
    if (standardConcentrations.length !== measuredSignals.length) {
      throw new BiosensorError(
        BiosensorErrorCode.INVALID_CONFIGURATION,
        'Number of concentrations must match number of signals'
      );
    }

    if (standardConcentrations.length < BIOSENSOR_CONSTANTS.STANDARD_CALIBRATION_POINTS) {
      throw new BiosensorError(
        BiosensorErrorCode.INSUFFICIENT_CALIBRATION_POINTS,
        `Need at least ${BIOSENSOR_CONSTANTS.STANDARD_CALIBRATION_POINTS} calibration points`
      );
    }

    // Create calibration points
    const points: CalibrationPoint[] = standardConcentrations.map((conc, i) => ({
      concentration: conc,
      signal: measuredSignals[i],
      signalUnit,
      concentrationUnit,
    }));

    // Perform linear regression
    const regression = this.linearRegression(standardConcentrations, measuredSignals);

    // Calculate performance metrics
    const sensitivity = regression.slope;
    const rSquared = regression.rSquared;

    // Validate calibration quality
    if (rSquared < BIOSENSOR_CONSTANTS.MIN_R_SQUARED) {
      throw new BiosensorError(
        BiosensorErrorCode.POOR_LINEARITY,
        `Poor linearity: R² = ${rSquared.toFixed(4)}, required: ${BIOSENSOR_CONSTANTS.MIN_R_SQUARED}`
      );
    }

    // Determine linear range
    const linearRange = {
      min: Math.min(...standardConcentrations.filter(c => c > 0)),
      max: Math.max(...standardConcentrations),
      unit: concentrationUnit,
    };

    // Calculate residuals for QC
    const residuals = standardConcentrations.map((conc, i) => {
      const predicted = regression.intercept + regression.slope * conc;
      return measuredSignals[i] - predicted;
    });

    const maxResidual = Math.max(...residuals.map(r => Math.abs(r)));

    // Create calibration data
    const calibrationId = `CAL-${Date.now()}`;
    const calibration: CalibrationData = {
      id: calibrationId,
      points,
      type: calibrationType,
      regression: {
        slope: regression.slope,
        intercept: regression.intercept,
      },
      rSquared,
      sensitivity,
      sensitivityUnit: `${signalUnit}/${concentrationUnit}`,
      linearRange,
      calibratedAt: new Date(),
      validUntil: new Date(Date.now() + 24 * 60 * 60 * 1000), // 24 hours
      qualityControl: {
        passed: true,
        residuals,
        maxResidual,
      },
    };

    // Update sensor
    sensor.calibration = calibration;
    sensor.status = 'idle';

    return calibration;
  }

  /**
   * Measure analyte concentration
   *
   * @param request - Measurement request
   * @returns Measurement result
   */
  measureAnalyte(request: MeasurementRequest): MeasurementResult {
    const {
      sensorId,
      signal,
      signalUnit = 'μA',
      temperature = 25,
      pH,
      sampleId,
      sampleType,
      calibration,
    } = request;

    // Get sensor
    const sensor = this.getSensor(sensorId);
    sensor.status = 'measuring';

    // Check sensor expiration
    if (sensor.lifetime?.expirationDate && sensor.lifetime.expirationDate < new Date()) {
      throw new BiosensorError(
        BiosensorErrorCode.SENSOR_EXPIRED,
        'Sensor has expired'
      );
    }

    // Get calibration
    const cal = calibration || sensor.calibration;
    if (!cal) {
      throw new BiosensorError(
        BiosensorErrorCode.CALIBRATION_FAILED,
        'Sensor not calibrated'
      );
    }

    // Check calibration validity
    if (cal.validUntil && cal.validUntil < new Date()) {
      throw new BiosensorError(
        BiosensorErrorCode.CALIBRATION_EXPIRED,
        'Calibration has expired'
      );
    }

    // Process signal (temperature correction, baseline, etc.)
    const processedSignal = this.processSignal({
      rawSignal: signal,
      temperatureCorrection: temperature !== 25 ? {
        temperature,
        referenceTemperature: 25,
        coefficient: BIOSENSOR_CONSTANTS.ENZYME_TEMP_COEFFICIENT,
      } : undefined,
    });

    const correctedSignal = processedSignal.final;

    // Calculate concentration using calibration
    let concentration: number;
    if (cal.type === 'linear' && cal.regression.slope && cal.regression.intercept !== undefined) {
      concentration = (correctedSignal - cal.regression.intercept) / cal.regression.slope;
    } else {
      throw new BiosensorError(
        BiosensorErrorCode.INVALID_CONFIGURATION,
        'Unsupported calibration type'
      );
    }

    // Quality flags
    const qualityFlags: QualityFlag[] = ['ok'];
    let withinRange = true;

    // Check if below LOD
    if (sensor.performance?.lod && concentration < sensor.performance.lod) {
      qualityFlags.push('below-lod');
    }

    // Check if below LOQ
    if (sensor.performance?.loq && concentration < sensor.performance.loq) {
      qualityFlags.push('below-loq');
    }

    // Check if within linear range
    if (concentration < cal.linearRange.min || concentration > cal.linearRange.max) {
      if (concentration < cal.linearRange.min) {
        qualityFlags.push('below-lod');
      } else {
        qualityFlags.push('above-range');
      }
      withinRange = false;
    }

    // Check temperature
    if (Math.abs(temperature - 25) > 10) {
      qualityFlags.push('temperature-out-of-range');
    }

    // Calculate confidence (simplified)
    const confidence = withinRange && qualityFlags.length === 1 ? 0.95 : 0.70;

    // Calculate standard error (simplified)
    const standardError = Math.abs(concentration * 0.05); // 5% of value

    // Create measurement result
    const measurementId = `M-${Date.now()}-${Math.random().toString(36).substr(2, 6)}`;
    const result: MeasurementResult = {
      id: measurementId,
      sensorId,
      concentration,
      concentrationUnit: cal.linearRange.unit,
      signal,
      signalUnit,
      correctedSignal,
      confidence,
      standardError,
      qualityFlags,
      withinRange,
      temperature,
      pH,
      timestamp: new Date(),
    };

    if (sampleId || sampleType) {
      result.sample = {
        id: sampleId || 'unknown',
        type: sampleType || 'unknown',
      };
    }

    // Update sensor
    sensor.measurementCount++;
    sensor.lastMeasurement = new Date();
    sensor.status = 'idle';

    // Store measurement
    if (!this.measurements.has(sensorId)) {
      this.measurements.set(sensorId, []);
    }
    this.measurements.get(sensorId)!.push(result);

    return result;
  }

  /**
   * Calculate limit of detection (LOD)
   *
   * @param params - LOD calculation parameters
   * @returns LOD result
   */
  calculateLOD(params: LODCalculation): LODResult {
    const {
      blankSignals,
      sensitivity,
      method = 'iupac',
      confidenceLevel = 0.99,
    } = params;

    // Calculate blank statistics
    const blankMean = blankSignals.reduce((a, b) => a + b, 0) / blankSignals.length;
    const variance = blankSignals.reduce((sum, val) => sum + Math.pow(val - blankMean, 2), 0) /
                     (blankSignals.length - 1);
    const blankStdDev = Math.sqrt(variance);

    // Calculate LOD based on method
    let lod: number;
    if (method === 'iupac') {
      lod = BIOSENSOR_CONSTANTS.LOD_MULTIPLIER * (blankStdDev / sensitivity);
    } else if (method === '3-sigma') {
      lod = 3 * (blankStdDev / sensitivity);
    } else {
      // EPA method (simplified)
      lod = 3.14 * (blankStdDev / sensitivity);
    }

    // Calculate LOQ
    const loq = BIOSENSOR_CONSTANTS.LOQ_MULTIPLIER * (blankStdDev / sensitivity);

    // Calculate SNR at LOD
    const signalAtLOD = blankMean + BIOSENSOR_CONSTANTS.LOD_MULTIPLIER * blankStdDev;
    const snrAtLOD = signalAtLOD / blankStdDev;

    return {
      lod,
      loq,
      blankMean,
      blankStdDev,
      snrAtLOD,
      unit: 'concentration units',
      method,
    };
  }

  /**
   * Update sensor performance metrics
   *
   * @param sensorId - Sensor ID
   * @param lodResult - LOD calculation result
   * @param responseTime - Response time in seconds
   */
  updatePerformanceMetrics(
    sensorId: string,
    lodResult: LODResult,
    responseTime: number
  ): PerformanceMetrics {
    const sensor = this.getSensor(sensorId);

    if (!sensor.calibration) {
      throw new BiosensorError(
        BiosensorErrorCode.CALIBRATION_FAILED,
        'Sensor must be calibrated before performance metrics can be set'
      );
    }

    const dynamicRange = Math.log10(
      sensor.calibration.linearRange.max / lodResult.lod
    );

    const metrics: PerformanceMetrics = {
      lod: lodResult.lod,
      loq: lodResult.loq,
      dynamicRange,
      concentrationUnit: sensor.calibration.linearRange.unit,
      responseTime,
      measuredAt: new Date(),
    };

    sensor.performance = metrics;

    return metrics;
  }

  /**
   * Validate sensor performance
   *
   * @param params - Validation parameters
   * @returns Validation result
   */
  validateSensor(params: ValidationParameters): ValidationResult {
    const { sensorId } = params;
    const sensor = this.getSensor(sensorId);

    const result: ValidationResult = {
      sensorId,
      validated: true,
      validatedAt: new Date(),
    };

    // Linearity validation
    if (params.linearity) {
      const regression = this.linearRegression(
        params.linearity.concentrations,
        params.linearity.signals
      );

      const passed = regression.rSquared >= params.linearity.requiredR2;
      result.linearity = {
        passed,
        rSquared: regression.rSquared,
        requiredR2: params.linearity.requiredR2,
      };

      if (!passed) result.validated = false;
    }

    // Accuracy validation
    if (params.accuracy) {
      const recoveries = params.accuracy.spikedSamples.map(sample => {
        return (sample.measuredConcentration / sample.expectedConcentration) * 100;
      });

      const meanRecovery = recoveries.reduce((a, b) => a + b, 0) / recoveries.length;
      const [minRecovery, maxRecovery] = params.accuracy.requiredRecovery;
      const passed = meanRecovery >= minRecovery && meanRecovery <= maxRecovery;

      result.accuracy = {
        passed,
        recovery: meanRecovery,
        requiredRange: [minRecovery, maxRecovery],
      };

      if (!passed) result.validated = false;
    }

    // Precision validation
    if (params.precision) {
      const mean = params.precision.replicates.reduce((a, b) => a + b, 0) /
                   params.precision.replicates.length;
      const variance = params.precision.replicates.reduce(
        (sum, val) => sum + Math.pow(val - mean, 2), 0
      ) / (params.precision.replicates.length - 1);
      const stdDev = Math.sqrt(variance);
      const cv = (stdDev / mean) * 100;

      const passed = cv <= params.precision.requiredCV;

      result.precision = {
        passed,
        cv,
        requiredCV: params.precision.requiredCV,
      };

      if (!passed) result.validated = false;
    }

    // Selectivity validation
    if (params.selectivity) {
      const coefficients: { [key: string]: number } = {};
      let allPassed = true;

      params.selectivity.interferents.forEach(interferent => {
        const selectivityCoefficient = params.selectivity!.targetSignal / interferent.signal;
        coefficients[interferent.name] = selectivityCoefficient;

        if (selectivityCoefficient < params.selectivity!.requiredSelectivity) {
          allPassed = false;
        }
      });

      result.selectivity = {
        passed: allPassed,
        coefficients,
        requiredSelectivity: params.selectivity.requiredSelectivity,
      };

      if (!allPassed) result.validated = false;
    }

    return result;
  }

  /**
   * Run quality control
   *
   * @param sensorId - Sensor ID
   * @param qcSample - Quality control sample
   * @param measuredConcentration - Measured concentration
   * @returns QC result
   */
  runQualityControl(
    sensorId: string,
    qcSample: QualityControlSample,
    measuredConcentration: number
  ): QualityControlResult {
    const sensor = this.getSensor(sensorId);

    // Check if within acceptable range
    const passed = measuredConcentration >= qcSample.acceptableRange.min &&
                   measuredConcentration <= qcSample.acceptableRange.max;

    // Calculate deviation
    const deviation = ((measuredConcentration - qcSample.targetConcentration) /
                      qcSample.targetConcentration) * 100;

    // Calculate z-score if historical data available
    let zScore: number | undefined;
    if (qcSample.historicalMean && qcSample.historicalStdDev) {
      zScore = (measuredConcentration - qcSample.historicalMean) /
               qcSample.historicalStdDev;
    }

    // Determine corrective action
    let correctiveAction: string | undefined;
    if (!passed) {
      if (Math.abs(deviation) > 20) {
        correctiveAction = 'Recalibrate sensor immediately';
      } else if (Math.abs(deviation) > 10) {
        correctiveAction = 'Run additional QC sample to confirm';
      } else {
        correctiveAction = 'Monitor next QC result';
      }
    }

    return {
      sample: qcSample,
      measuredConcentration,
      passed,
      deviation,
      zScore,
      timestamp: new Date(),
      correctiveAction,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Validate sensor configuration
   */
  private validateConfig(config: BiosensorConfig): void {
    if (!config.type) {
      throw new BiosensorError(
        BiosensorErrorCode.INVALID_CONFIGURATION,
        'Sensor type is required'
      );
    }

    if (!config.biorecognitionElement) {
      throw new BiosensorError(
        BiosensorErrorCode.INVALID_CONFIGURATION,
        'Biorecognition element is required'
      );
    }

    if (!config.targetAnalyte) {
      throw new BiosensorError(
        BiosensorErrorCode.INVALID_CONFIGURATION,
        'Target analyte is required'
      );
    }
  }

  /**
   * Linear regression
   */
  private linearRegression(x: number[], y: number[]): {
    slope: number;
    intercept: number;
    rSquared: number;
  } {
    const n = x.length;
    const sumX = x.reduce((a, b) => a + b, 0);
    const sumY = y.reduce((a, b) => a + b, 0);
    const sumXY = x.reduce((sum, xi, i) => sum + xi * y[i], 0);
    const sumX2 = x.reduce((sum, xi) => sum + xi * xi, 0);
    const sumY2 = y.reduce((sum, yi) => sum + yi * yi, 0);

    const slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    const intercept = (sumY - slope * sumX) / n;

    // Calculate R²
    const yMean = sumY / n;
    const ssTot = y.reduce((sum, yi) => sum + Math.pow(yi - yMean, 2), 0);
    const ssRes = y.reduce((sum, yi, i) => {
      const predicted = intercept + slope * x[i];
      return sum + Math.pow(yi - predicted, 2);
    }, 0);
    const rSquared = 1 - (ssRes / ssTot);

    return { slope, intercept, rSquared };
  }

  /**
   * Process signal
   */
  private processSignal(params: SignalProcessingParams): ProcessedSignal {
    const { rawSignal, baseline, temperatureCorrection } = params;

    const stepsApplied: string[] = [];
    let signal = rawSignal;

    // Baseline correction
    if (baseline !== undefined) {
      signal = signal - baseline;
      stepsApplied.push('baseline-correction');
    }

    // Temperature correction
    if (temperatureCorrection) {
      const { temperature, referenceTemperature, coefficient } = temperatureCorrection;
      const tempDelta = temperature - referenceTemperature;
      const correctionFactor = 1 + (coefficient / 100) * tempDelta;
      signal = signal / correctionFactor;
      stepsApplied.push('temperature-correction');
    }

    return {
      original: rawSignal,
      baselineCorrected: baseline !== undefined ? signal : undefined,
      temperatureCorrected: temperatureCorrection ? signal : undefined,
      final: signal,
      stepsApplied,
    };
  }

  /**
   * Get sensor measurements
   */
  getMeasurements(sensorId: string): MeasurementResult[] {
    return this.measurements.get(sensorId) || [];
  }

  /**
   * Get all sensors
   */
  getAllSensors(): Biosensor[] {
    return Array.from(this.sensors.values());
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate sensitivity from calibration data (standalone)
 */
export function calculateSensitivity(
  concentrations: number[],
  signals: number[]
): number {
  const sdk = new BiosensorSDK();
  const regression = (sdk as any).linearRegression(concentrations, signals);
  return regression.slope;
}

/**
 * Calculate LOD (standalone)
 */
export function calculateLOD(params: LODCalculation): LODResult {
  const sdk = new BiosensorSDK();
  return sdk.calculateLOD(params);
}

/**
 * Process biosensor signal (standalone)
 */
export function processBiosensorSignal(params: SignalProcessingParams): ProcessedSignal {
  const sdk = new BiosensorSDK();
  return (sdk as any).processSignal(params);
}

/**
 * Calibrate sensor (standalone)
 */
export function calibrateSensor(request: CalibrationRequest): CalibrationData {
  const sdk = new BiosensorSDK();
  return sdk.calibrateSensor(request);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { BiosensorSDK };
export default BiosensorSDK;
