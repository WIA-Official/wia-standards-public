/**
 * WIA-SLEEP SDK - Light Dose Calculator
 *
 * Utilities for calculating light exposure doses and therapy parameters.
 */

/**
 * Light spectrum type
 */
export type LightSpectrumType = 'broad' | 'blue_enriched' | 'blue_depleted' | 'red' | 'green';

/**
 * Light source parameters
 */
export interface LightSource {
  /** Illuminance in lux */
  illuminance_lux: number;
  /** Color temperature in Kelvin */
  colorTemperature_K: number;
  /** Spectrum type */
  spectrumType: LightSpectrumType;
  /** Melanopic EDI (Equivalent Daylight Illuminance) */
  melanopicEDI?: number;
}

/**
 * Light exposure session
 */
export interface LightExposure {
  /** Light source */
  source: LightSource;
  /** Duration in minutes */
  duration_min: number;
  /** Start time (hours from midnight, 0-24) */
  startTime_hours: number;
}

/**
 * Light dose calculation result
 */
export interface LightDose {
  /** Total photopic dose (lux-hours) */
  photopicDose_lux_hours: number;
  /** Melanopic dose (melanopic lux-hours) */
  melanopicDose_mlux_hours: number;
  /** Estimated circadian effectiveness (0-1) */
  circadianEffectiveness: number;
  /** Expected phase shift (hours) */
  expectedPhaseShift_hours: number;
}

/**
 * Calculate melanopic EDI from photopic illuminance and color temperature
 *
 * This is a simplified approximation. Accurate melanopic calculations
 * require full spectral power distribution data.
 */
export function estimateMelanopicEDI(
  illuminance_lux: number,
  colorTemperature_K: number
): number {
  // Melanopic ratio varies with color temperature
  // Higher color temperatures (bluer light) have higher melanopic content
  let melanopicRatio: number;

  if (colorTemperature_K < 2700) {
    melanopicRatio = 0.45;
  } else if (colorTemperature_K < 4000) {
    melanopicRatio = 0.45 + ((colorTemperature_K - 2700) / 1300) * 0.25;
  } else if (colorTemperature_K < 6500) {
    melanopicRatio = 0.70 + ((colorTemperature_K - 4000) / 2500) * 0.30;
  } else {
    melanopicRatio = 1.0 + ((colorTemperature_K - 6500) / 3500) * 0.15;
  }

  return illuminance_lux * melanopicRatio;
}

/**
 * Calculate effective circadian illuminance
 *
 * Accounts for:
 * - Melanopsin spectral sensitivity
 * - Intensity-response curve (saturation at high levels)
 * - Exposure duration
 */
export function calculateCircadianIlluminance(
  melanopicEDI: number,
  duration_min: number
): number {
  // Circadian system shows logarithmic response to intensity
  // Saturation begins around 1000-2000 melanopic lux
  const saturationPoint = 1500;

  let effectiveIntensity: number;
  if (melanopicEDI <= saturationPoint) {
    effectiveIntensity = melanopicEDI;
  } else {
    // Logarithmic saturation above saturation point
    effectiveIntensity =
      saturationPoint +
      saturationPoint * Math.log10(melanopicEDI / saturationPoint);
  }

  // Duration effect - diminishing returns after 30-60 minutes
  const durationFactor = Math.min(1, duration_min / 45);

  return effectiveIntensity * durationFactor;
}

/**
 * Calculate light dose for a single exposure session
 */
export function calculateLightDose(exposure: LightExposure): LightDose {
  const { source, duration_min, startTime_hours } = exposure;

  // Calculate melanopic EDI if not provided
  const melanopicEDI =
    source.melanopicEDI ||
    estimateMelanopicEDI(source.illuminance_lux, source.colorTemperature_K);

  // Photopic dose
  const photopicDose_lux_hours = (source.illuminance_lux * duration_min) / 60;

  // Melanopic dose
  const melanopicDose_mlux_hours = (melanopicEDI * duration_min) / 60;

  // Circadian effectiveness depends on timing and intensity
  const circadianIlluminance = calculateCircadianIlluminance(
    melanopicEDI,
    duration_min
  );

  // Normalize to 0-1 scale (full effectiveness at ~10000 lux for 30 min)
  const circadianEffectiveness = Math.min(
    1,
    circadianIlluminance / 10000
  );

  // Estimate phase shift based on timing relative to CBT nadir
  // Assuming CBT nadir around 4-5 AM (4.5 hours)
  const expectedPhaseShift = calculatePhaseShiftFromTiming(
    startTime_hours,
    circadianEffectiveness,
    4.5 // CBT nadir time
  );

  return {
    photopicDose_lux_hours,
    melanopicDose_mlux_hours,
    circadianEffectiveness,
    expectedPhaseShift_hours: expectedPhaseShift
  };
}

/**
 * Calculate expected phase shift based on light timing
 *
 * Uses simplified Phase Response Curve (PRC)
 */
export function calculatePhaseShiftFromTiming(
  exposureTime_hours: number,
  effectiveness: number,
  cbtNadir_hours: number = 4.5
): number {
  // Normalize exposure time relative to CBT nadir
  let relativeTime = exposureTime_hours - cbtNadir_hours;

  // Handle wraparound
  if (relativeTime > 12) relativeTime -= 24;
  if (relativeTime < -12) relativeTime += 24;

  // Maximum phase shift at full effectiveness
  const maxShift = 2.0;

  // Phase Response Curve (simplified)
  let phaseShift: number;

  if (relativeTime >= -6 && relativeTime < 0) {
    // Delay zone (before CBT nadir)
    // Peak delay at ~3 hours before nadir
    phaseShift = maxShift * Math.sin(((relativeTime + 3) / 3) * (Math.PI / 2));
  } else if (relativeTime >= 0 && relativeTime <= 6) {
    // Advance zone (after CBT nadir)
    // Peak advance at ~3 hours after nadir
    phaseShift = -maxShift * Math.sin((relativeTime / 3) * (Math.PI / 2));
  } else {
    // Dead zone - minimal phase shift
    phaseShift = 0;
  }

  return phaseShift * effectiveness;
}

/**
 * Calculate cumulative light dose over multiple exposures
 */
export function calculateCumulativeDose(exposures: LightExposure[]): LightDose {
  let totalPhotopic = 0;
  let totalMelanopic = 0;
  let weightedEffectiveness = 0;
  let weightedPhaseShift = 0;
  let totalDuration = 0;

  for (const exposure of exposures) {
    const dose = calculateLightDose(exposure);

    totalPhotopic += dose.photopicDose_lux_hours;
    totalMelanopic += dose.melanopicDose_mlux_hours;
    weightedEffectiveness +=
      dose.circadianEffectiveness * exposure.duration_min;
    weightedPhaseShift +=
      dose.expectedPhaseShift_hours * dose.circadianEffectiveness;
    totalDuration += exposure.duration_min;
  }

  return {
    photopicDose_lux_hours: totalPhotopic,
    melanopicDose_mlux_hours: totalMelanopic,
    circadianEffectiveness:
      totalDuration > 0 ? weightedEffectiveness / totalDuration : 0,
    expectedPhaseShift_hours: weightedPhaseShift
  };
}

/**
 * Light therapy recommendation parameters
 */
export interface LightTherapyRecommendation {
  /** Recommended intensity in lux */
  intensity_lux: number;
  /** Recommended duration in minutes */
  duration_min: number;
  /** Recommended timing (hours from midnight) */
  timing_hours: number;
  /** Recommended color temperature */
  colorTemperature_K: number;
  /** Expected phase shift */
  expectedPhaseShift_hours: number;
  /** Treatment duration in days */
  treatmentDuration_days: number;
}

/**
 * Generate light therapy recommendation for phase advance
 */
export function recommendPhaseAdvanceTherapy(
  targetAdvance_hours: number,
  currentCBTNadir_hours: number = 4.5,
  constraints?: {
    maxIntensity_lux?: number;
    maxDuration_min?: number;
    earliestStart_hours?: number;
  }
): LightTherapyRecommendation {
  // Optimal timing is 0-3 hours after CBT nadir
  let optimalTiming = currentCBTNadir_hours + 1.5;
  if (optimalTiming > 24) optimalTiming -= 24;

  // Apply constraints
  if (constraints?.earliestStart_hours !== undefined) {
    optimalTiming = Math.max(optimalTiming, constraints.earliestStart_hours);
  }

  // Calculate required intensity and duration
  // More phase shift needed = higher dose required
  const baseIntensity = 10000;
  const baseDuration = 30;

  let intensity = Math.min(
    constraints?.maxIntensity_lux || 10000,
    baseIntensity
  );
  let duration = Math.min(
    constraints?.maxDuration_min || 45,
    baseDuration + Math.abs(targetAdvance_hours) * 10
  );

  // Treatment duration depends on total phase shift needed
  // Approximately 0.5-1 hour shift per day is achievable
  const treatmentDays = Math.ceil(Math.abs(targetAdvance_hours) / 0.75);

  return {
    intensity_lux: intensity,
    duration_min: duration,
    timing_hours: optimalTiming,
    colorTemperature_K: 6500, // Blue-enriched for phase advance
    expectedPhaseShift_hours: -Math.min(1.5, Math.abs(targetAdvance_hours)),
    treatmentDuration_days: treatmentDays
  };
}

/**
 * Generate light therapy recommendation for phase delay
 */
export function recommendPhaseDelayTherapy(
  targetDelay_hours: number,
  currentCBTNadir_hours: number = 4.5,
  constraints?: {
    maxIntensity_lux?: number;
    maxDuration_min?: number;
    latestEnd_hours?: number;
  }
): LightTherapyRecommendation {
  // Optimal timing is 3-6 hours before CBT nadir (evening)
  let optimalTiming = currentCBTNadir_hours - 4;
  if (optimalTiming < 0) optimalTiming += 24;

  // Apply constraints
  if (constraints?.latestEnd_hours !== undefined) {
    optimalTiming = Math.min(optimalTiming, constraints.latestEnd_hours);
  }

  const baseIntensity = 10000;
  const baseDuration = 30;

  let intensity = Math.min(
    constraints?.maxIntensity_lux || 10000,
    baseIntensity
  );
  let duration = Math.min(
    constraints?.maxDuration_min || 45,
    baseDuration + targetDelay_hours * 10
  );

  const treatmentDays = Math.ceil(targetDelay_hours / 0.75);

  return {
    intensity_lux: intensity,
    duration_min: duration,
    timing_hours: optimalTiming,
    colorTemperature_K: 6500,
    expectedPhaseShift_hours: Math.min(1.5, targetDelay_hours),
    treatmentDuration_days: treatmentDays
  };
}

/**
 * Calculate daily light exposure requirements for circadian health
 */
export function getDailyLightRequirements(): {
  morning: { minLux: number; minDuration_min: number; optimalTiming: string };
  daytime: { minLux: number; totalHours: number };
  evening: { maxLux: number; startTime: string };
} {
  return {
    morning: {
      minLux: 1000,
      minDuration_min: 30,
      optimalTiming: 'Within 30 minutes of waking'
    },
    daytime: {
      minLux: 250,
      totalHours: 2 // Cumulative bright light exposure
    },
    evening: {
      maxLux: 50, // For 2-3 hours before bed
      startTime: '2-3 hours before intended sleep'
    }
  };
}

export default {
  estimateMelanopicEDI,
  calculateCircadianIlluminance,
  calculateLightDose,
  calculatePhaseShiftFromTiming,
  calculateCumulativeDose,
  recommendPhaseAdvanceTherapy,
  recommendPhaseDelayTherapy,
  getDailyLightRequirements
};
