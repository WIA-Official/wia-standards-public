/**
 * WIA-TIME-024: Time Measurement - TypeScript SDK Implementation
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

import {
  PrecisionLevel,
  ClockType,
  TimeMeasurementError,
  type ReferenceFrame,
  type Timeline,
  type TimeMeasurementParams,
  type TimeMeasurementResult,
  type PrecisionTimestamp,
  type AtomicClockCalibration,
  type CalibrationResult,
  type ClockSyncParams,
  type ClockSyncResult,
  type SpecialRelativityParams,
  type SpecialRelativityResult,
  type GeneralRelativityParams,
  type GeneralRelativityResult,
  type RelativisticCorrections,
  type CrossTimelineMeasurementParams,
  type CrossTimelineMeasurementResult,
  type TimelineSyncParams,
  type TimelineSyncResult,
  type QuantumTimeMeasurementParams,
  type QuantumTimeMeasurementResult,
  type MeasurementConfig,
  type TimeMeasurementException,
  type TimeInterval,
  type ClockDrift,
  type TemporalResolution,
} from './types.js';

// ============================================================================
// Physical Constants
// ============================================================================

const CONSTANTS = {
  SPEED_OF_LIGHT: 299792458, // m/s
  PLANCK_CONSTANT: 1.054571817e-34, // J⋅s
  GRAVITATIONAL_CONSTANT: 6.67430e-11, // m³⋅kg⁻¹⋅s⁻²
  PLANCK_TIME: 5.391247e-44, // s
  CESIUM_FREQUENCY: 9192631770, // Hz
  BOLTZMANN_CONSTANT: 1.380649e-23, // J/K
} as const;

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Calculate Lorentz factor (gamma)
 */
function calculateLorentzFactor(velocity: number): number {
  const beta = velocity / CONSTANTS.SPEED_OF_LIGHT;
  if (beta >= 1) {
    throw new Error('Velocity must be less than speed of light');
  }
  return 1 / Math.sqrt(1 - beta * beta);
}

/**
 * Calculate gravitational time dilation factor
 */
function calculateGravitationalFactor(
  gravitationalPotential: number
): number {
  const factor = 1 + (2 * gravitationalPotential) / (CONSTANTS.SPEED_OF_LIGHT ** 2);
  return Math.sqrt(Math.max(0, factor));
}

/**
 * Convert time value to Date object
 */
function toDate(time: Date | number | string): Date {
  if (time instanceof Date) return time;
  if (typeof time === 'number') return new Date(time);
  return new Date(time);
}

/**
 * Get precision in seconds based on level
 */
function getPrecisionInSeconds(level: PrecisionLevel): number {
  const precisions = {
    [PrecisionLevel.CLASSICAL]: 1e-3, // millisecond
    [PrecisionLevel.SEMI_CLASSICAL]: 1e-6, // microsecond
    [PrecisionLevel.QUANTUM_CORRECTIONS]: 1e-9, // nanosecond
    [PrecisionLevel.QUANTUM_SIGNIFICANT]: 1e-12, // picosecond
    [PrecisionLevel.QUANTUM_DOMINANT]: 1e-15, // femtosecond
    [PrecisionLevel.FULL_QUANTUM]: 1e-18, // attosecond
  };
  return precisions[level];
}

// ============================================================================
// Main SDK Class
// ============================================================================

export class TimeMeasurementSDK {
  private config: MeasurementConfig;
  private calibrationCache: Map<string, CalibrationResult> = new Map();
  private lastMeasurement?: TimeMeasurementResult;

  constructor(config?: Partial<MeasurementConfig>) {
    this.config = {
      defaultClockType: ClockType.CESIUM_FOUNTAIN,
      defaultPrecisionLevel: PrecisionLevel.QUANTUM_CORRECTIONS,
      autoCorrections: {
        specialRelativity: true,
        generalRelativity: true,
        quantum: false,
      },
      calibrationInterval: 86400, // 24 hours
      syncInterval: 3600, // 1 hour
      cacheSize: 100,
      parallelProcessing: true,
      ...config,
    };
  }

  /**
   * Measure precision time with all corrections applied
   */
  public measurePrecisionTime(
    params: TimeMeasurementParams
  ): TimeMeasurementResult {
    const properTime = toDate(params.properTime).getTime() / 1000; // Convert to seconds
    const velocity = params.velocity ?? 0;
    const gravitationalPotential = params.gravitationalPotential ?? 0;
    const precisionLevel =
      params.precisionLevel ?? this.config.defaultPrecisionLevel;

    // Calculate Lorentz factor
    const lorentzFactor =
      velocity > 0 ? calculateLorentzFactor(velocity * CONSTANTS.SPEED_OF_LIGHT) : 1;

    // Special relativity correction
    let relativisticOffset = 0;
    if (
      this.config.autoCorrections.specialRelativity &&
      Math.abs(velocity) > 0.001
    ) {
      relativisticOffset = properTime * (1 - lorentzFactor);
    }

    // General relativity correction
    let gravitationalOffset = 0;
    if (
      this.config.autoCorrections.generalRelativity &&
      gravitationalPotential !== 0
    ) {
      const gravFactor = calculateGravitationalFactor(gravitationalPotential);
      gravitationalOffset = properTime * (gravFactor - 1);
    }

    // Quantum uncertainty correction
    let quantumOffset = 0;
    if (
      this.config.autoCorrections.quantum &&
      params.includeQuantumUncertainty
    ) {
      quantumOffset = this.calculateQuantumUncertainty(properTime, precisionLevel);
    }

    // Total correction
    const totalCorrection =
      relativisticOffset + gravitationalOffset + quantumOffset;
    const coordinateTime = properTime + totalCorrection;

    // Calculate uncertainty based on precision level
    const uncertainty = getPrecisionInSeconds(precisionLevel);

    // Reference frame
    const referenceFrame: ReferenceFrame = params.referenceFrame ?? {
      id: 'default',
      name: 'Earth Surface',
      velocity: velocity * CONSTANTS.SPEED_OF_LIGHT,
      gravitationalPotential: gravitationalPotential,
      inertial: true,
    };

    const result: TimeMeasurementResult = {
      properTime,
      coordinateTime,
      correctedTime: new Date(coordinateTime * 1000),
      relativisticOffset,
      gravitationalOffset,
      quantumOffset,
      totalCorrection,
      uncertainty,
      unit: this.getUnitForPrecision(precisionLevel),
      lorentzFactor,
      referenceFrame,
      timeline: params.timeline,
    };

    this.lastMeasurement = result;
    return result;
  }

  /**
   * Get precision timestamp with full metadata
   */
  public getPrecisionTimestamp(
    time?: Date | number | string
  ): PrecisionTimestamp {
    const date = time ? toDate(time) : new Date();
    const unixMs = date.getTime();
    const unixNs = BigInt(unixMs) * BigInt(1000000);
    const unixSeconds = unixMs / 1000;

    // TAI is UTC + leap seconds (currently 37 as of 2023)
    const tai = unixSeconds + 37;

    return {
      iso8601: date.toISOString(),
      unixMs,
      unixNs,
      tai,
      utc: date,
      gpsTime: unixSeconds + 315964800 - 18, // GPS epoch + leap seconds
      properTime: unixSeconds,
      coordinateTime: unixSeconds,
      uncertainty: getPrecisionInSeconds(this.config.defaultPrecisionLevel),
      precision: this.config.defaultPrecisionLevel,
    };
  }

  /**
   * Calibrate atomic clock
   */
  public calibrateAtomicClock(
    params: AtomicClockCalibration
  ): CalibrationResult {
    const measurementDuration = params.measurementDuration ?? 86400; // 24 hours

    // Simulate frequency offset (in real implementation, measure against reference)
    const baseAccuracy = this.getClockAccuracy(params.clockType);
    const frequencyOffset = (Math.random() - 0.5) * baseAccuracy * CONSTANTS.CESIUM_FREQUENCY;

    const fractionalDeviation = frequencyOffset / CONSTANTS.CESIUM_FREQUENCY;
    const allanDeviation = baseAccuracy;

    // Drift rate (seconds per day)
    const driftRate = fractionalDeviation * 86400;

    // Phase difference accumulates over measurement duration
    const phaseDifference = fractionalDeviation * measurementDuration;

    const now = new Date();
    const nextCalibration = new Date(
      now.getTime() + this.getCalibrationInterval(params.clockType) * 1000
    );

    const result: CalibrationResult = {
      success: Math.abs(fractionalDeviation) < params.targetAccuracy,
      frequencyOffset,
      fractionalDeviation,
      allanDeviation,
      driftRate,
      accuracy: baseAccuracy,
      phaseDifference,
      calibrationTime: now,
      nextCalibration,
    };

    // Cache calibration result
    this.calibrationCache.set(params.clockType, result);

    return result;
  }

  /**
   * Synchronize clocks using two-way time transfer
   */
  public synchronizeClocks(params: ClockSyncParams): ClockSyncResult {
    // Simulate two-way time transfer
    const t1 = Date.now();
    const networkDelay = Math.random() * 10; // 0-10 ms network delay
    const t2 = t1 + networkDelay;
    const t3 = t2 + 1; // Processing time
    const t4 = t3 + networkDelay;

    // Calculate offset and delay
    const offset = ((t2 - t1) - (t4 - t3)) / 2;
    const delay = ((t2 - t1) + (t4 - t3)) / 2;

    const accuracy = params.targetAccuracy ?? 1e-6; // 1 microsecond default
    const success = Math.abs(offset) < accuracy * 1000; // Convert to ms

    return {
      success,
      offset: offset / 1000, // Convert to seconds
      delay: delay / 1000,
      accuracy: Math.max(accuracy, delay / 1000),
      syncTime: new Date(),
      nextSync: new Date(Date.now() + this.config.syncInterval * 1000),
    };
  }

  /**
   * Apply special relativity correction
   */
  public applySpecialRelativityCorrection(
    params: SpecialRelativityParams
  ): SpecialRelativityResult {
    const velocity = params.velocityAsFractionOfC
      ? params.velocity * CONSTANTS.SPEED_OF_LIGHT
      : params.velocity;

    const velocityFraction = velocity / CONSTANTS.SPEED_OF_LIGHT;

    if (velocityFraction >= 1) {
      throw this.createError(
        TimeMeasurementError.INVALID_VELOCITY,
        'Velocity must be less than speed of light'
      );
    }

    const lorentzFactor = calculateLorentzFactor(velocity);
    const timeDilationFactor = 1 / lorentzFactor;
    const correction = params.properTime * (1 - lorentzFactor);
    const correctedTime = params.properTime + correction;

    return {
      lorentzFactor,
      timeDilationFactor,
      correction,
      correctedTime,
      velocityFraction,
    };
  }

  /**
   * Apply general relativity correction
   */
  public applyGeneralRelativityCorrection(
    params: GeneralRelativityParams
  ): GeneralRelativityResult {
    let potential = params.gravitationalPotential;

    // If radius and mass provided, calculate potential
    if (params.radius && params.mass) {
      potential = -(CONSTANTS.GRAVITATIONAL_CONSTANT * params.mass) / params.radius;
    }

    const gravitationalFactor = calculateGravitationalFactor(potential);
    const correction = params.properTime * (gravitationalFactor - 1);
    const correctedTime = params.properTime + correction;

    return {
      gravitationalFactor,
      correction,
      correctedTime,
      potential,
    };
  }

  /**
   * Apply combined relativistic corrections
   */
  public applyRelativisticCorrections(
    properTime: number,
    velocity: number,
    gravitationalPotential: number
  ): RelativisticCorrections {
    const sr = this.applySpecialRelativityCorrection({
      velocity,
      velocityAsFractionOfC: true,
      properTime,
    });

    const gr = this.applyGeneralRelativityCorrection({
      gravitationalPotential,
      properTime,
    });

    const totalCorrection = sr.correction + gr.correction;
    const finalTime = properTime + totalCorrection;

    return {
      specialRelativity: sr,
      generalRelativity: gr,
      totalCorrection,
      finalTime,
    };
  }

  /**
   * Measure time across multiple timelines
   */
  public measureCrossTimeline(
    params: CrossTimelineMeasurementParams
  ): CrossTimelineMeasurementResult {
    const sourceTime =
      typeof params.sourceTime === 'number'
        ? params.sourceTime
        : toDate(params.sourceTime).getTime() / 1000;

    // Calculate divergence factor
    const divergenceFactor = params.includeDivergence
      ? this.calculateTimelineDivergence(
          params.sourceTimeline,
          params.targetTimeline
        )
      : 0;

    // Calculate target time
    const displacement = divergenceFactor;
    const targetTime = sourceTime + displacement;

    return {
      sourceTime,
      targetTime,
      divergenceFactor,
      displacement,
      accuracy: 1e-9, // nanosecond accuracy
      branchPoint: params.sourceTimeline.branchPoint
        ? toDate(params.sourceTimeline.branchPoint)
        : undefined,
    };
  }

  /**
   * Synchronize multiple timelines
   */
  public syncTimelines(params: TimelineSyncParams): TimelineSyncResult {
    const offsets = new Map<string, number>();

    const referenceTimeline = params.referenceTimeline ?? params.timelines[0];

    // Calculate offsets for each timeline
    params.timelines.forEach((timeline) => {
      if (timeline.id === referenceTimeline.id) {
        offsets.set(timeline.id, 0);
      } else {
        const divergence = this.calculateTimelineDivergence(
          referenceTimeline,
          timeline
        );
        offsets.set(timeline.id, divergence);
      }
    });

    // Calculate coherence time (how long sync is valid)
    const maxDivergenceRate = Math.max(
      ...params.timelines.map((t) => t.divergenceRate ?? 0)
    );
    const coherenceTime =
      maxDivergenceRate > 0 ? 1 / maxDivergenceRate : Infinity;

    return {
      success: true,
      offsets,
      coherenceTime,
      uncertainty: 1e-9,
      nextSync: new Date(Date.now() + (params.updateFrequency ?? 1000)),
    };
  }

  /**
   * Measure quantum time interval
   */
  public measureQuantumTime(
    params: QuantumTimeMeasurementParams
  ): QuantumTimeMeasurementResult {
    const interval = params.interval;

    // Heisenberg uncertainty principle: ΔE × Δt ≥ ℏ/2
    const energyUncertainty = params.energyUncertainty ?? 1e-19; // Joules
    const heisenbergLimit = CONSTANTS.PLANCK_CONSTANT / (2 * energyUncertainty);

    // Thermal uncertainty
    const temperature = params.temperature ?? 300; // Kelvin
    const thermalEnergy = CONSTANTS.BOLTZMANN_CONSTANT * temperature;
    const thermalLimit = CONSTANTS.PLANCK_CONSTANT / thermalEnergy;

    // Quantum uncertainty is the maximum of all limits
    const quantumUncertainty = Math.max(
      heisenbergLimit,
      thermalLimit,
      CONSTANTS.PLANCK_TIME
    );

    const minimumInterval = quantumUncertainty;
    const planckTimeLimitReached = interval <= CONSTANTS.PLANCK_TIME;

    if (planckTimeLimitReached) {
      throw this.createError(
        TimeMeasurementError.QUANTUM_LIMIT,
        'Cannot measure intervals below Planck time'
      );
    }

    return {
      interval,
      quantumUncertainty,
      heisenbergLimit: params.includeHeisenberg ? heisenbergLimit : undefined,
      planckTimeLimitReached,
      minimumInterval,
    };
  }

  /**
   * Calculate temporal resolution for a given configuration
   */
  public calculateTemporalResolution(
    clockType: ClockType,
    precisionLevel: PrecisionLevel
  ): TemporalResolution {
    const accuracy = this.getClockAccuracy(clockType);
    const minimum = getPrecisionInSeconds(precisionLevel);

    return {
      minimum,
      maximum: 1e15, // ~31 million years
      accuracy,
      class: precisionLevel,
    };
  }

  /**
   * Get clock drift information
   */
  public getClockDrift(clockType: ClockType): ClockDrift {
    const calibration = this.calibrationCache.get(clockType);

    if (!calibration) {
      return {
        rate: 0,
        accumulated: 0,
        lastCalibration: new Date(0),
        predicted: 0,
      };
    }

    const timeSinceCalibration =
      (Date.now() - calibration.calibrationTime.getTime()) / 1000;
    const accumulated = calibration.driftRate * (timeSinceCalibration / 86400);

    const timeUntilNextCalibration =
      (calibration.nextCalibration.getTime() - Date.now()) / 1000;
    const predicted =
      calibration.driftRate * (timeUntilNextCalibration / 86400);

    return {
      rate: calibration.driftRate / 86400, // Convert to seconds/second
      accumulated,
      lastCalibration: calibration.calibrationTime,
      predicted,
    };
  }

  // ============================================================================
  // Private Helper Methods
  // ============================================================================

  private calculateQuantumUncertainty(
    time: number,
    precisionLevel: PrecisionLevel
  ): number {
    // Simplified quantum uncertainty calculation
    const baseUncertainty = getPrecisionInSeconds(precisionLevel);
    return baseUncertainty * Math.random();
  }

  private getUnitForPrecision(level: PrecisionLevel): string {
    const units = {
      [PrecisionLevel.CLASSICAL]: 'ms',
      [PrecisionLevel.SEMI_CLASSICAL]: 'μs',
      [PrecisionLevel.QUANTUM_CORRECTIONS]: 'ns',
      [PrecisionLevel.QUANTUM_SIGNIFICANT]: 'ps',
      [PrecisionLevel.QUANTUM_DOMINANT]: 'fs',
      [PrecisionLevel.FULL_QUANTUM]: 'as',
    };
    return units[level];
  }

  private getClockAccuracy(clockType: ClockType): number {
    const accuracies = {
      [ClockType.QUARTZ]: 1e-6,
      [ClockType.RUBIDIUM]: 1e-11,
      [ClockType.CESIUM_BEAM]: 1e-14,
      [ClockType.CESIUM_FOUNTAIN]: 1e-16,
      [ClockType.OPTICAL_LATTICE]: 1e-18,
      [ClockType.QUANTUM]: 1e-21,
    };
    return accuracies[clockType];
  }

  private getCalibrationInterval(clockType: ClockType): number {
    const intervals = {
      [ClockType.QUARTZ]: 2592000, // 30 days
      [ClockType.RUBIDIUM]: 15552000, // 180 days
      [ClockType.CESIUM_BEAM]: 31536000, // 365 days
      [ClockType.CESIUM_FOUNTAIN]: 63072000, // 730 days
      [ClockType.OPTICAL_LATTICE]: 157680000, // 1825 days (5 years)
      [ClockType.QUANTUM]: 157680000,
    };
    return intervals[clockType];
  }

  private calculateTimelineDivergence(
    timeline1: Timeline,
    timeline2: Timeline
  ): number {
    if (!timeline1.branchPoint || !timeline2.branchPoint) {
      return 0;
    }

    const branchTime1 = toDate(timeline1.branchPoint).getTime();
    const branchTime2 = toDate(timeline2.branchPoint).getTime();
    const timeDiff = Math.abs(branchTime2 - branchTime1) / 1000; // seconds

    const rate1 = timeline1.divergenceRate ?? 0;
    const rate2 = timeline2.divergenceRate ?? 0;
    const avgRate = (rate1 + rate2) / 2;

    return timeDiff * avgRate;
  }

  private createError(
    code: TimeMeasurementError,
    message: string
  ): TimeMeasurementException {
    return {
      code,
      message,
      timestamp: new Date(),
    };
  }
}

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * Measure precision time (convenience function)
 */
export function measurePrecisionTime(
  params: TimeMeasurementParams
): TimeMeasurementResult {
  const sdk = new TimeMeasurementSDK();
  return sdk.measurePrecisionTime(params);
}

/**
 * Calibrate atomic clock (convenience function)
 */
export function calibrateAtomicClock(
  params: AtomicClockCalibration
): CalibrationResult {
  const sdk = new TimeMeasurementSDK();
  return sdk.calibrateAtomicClock(params);
}

/**
 * Apply relativistic correction (convenience function)
 */
export function applyRelativisticCorrection(
  properTime: number,
  velocity: number,
  gravitationalPotential: number
): RelativisticCorrections {
  const sdk = new TimeMeasurementSDK();
  return sdk.applyRelativisticCorrections(
    properTime,
    velocity,
    gravitationalPotential
  );
}

/**
 * Synchronize timelines (convenience function)
 */
export function syncMultiTimeline(
  params: TimelineSyncParams
): TimelineSyncResult {
  const sdk = new TimeMeasurementSDK();
  return sdk.syncTimelines(params);
}

/**
 * Calculate temporal resolution (convenience function)
 */
export function calculateTemporalResolution(
  clockType: ClockType,
  precisionLevel: PrecisionLevel
): TemporalResolution {
  const sdk = new TimeMeasurementSDK();
  return sdk.calculateTemporalResolution(clockType, precisionLevel);
}

// ============================================================================
// Exports
// ============================================================================

export * from './types.js';
export { CONSTANTS };
export default TimeMeasurementSDK;
