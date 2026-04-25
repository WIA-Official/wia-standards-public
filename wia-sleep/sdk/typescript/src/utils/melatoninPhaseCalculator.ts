/**
 * WIA-SLEEP SDK - Melatonin Phase Calculator
 *
 * Utilities for calculating DLMO and melatonin-based circadian phase markers.
 */

/**
 * Melatonin measurement point
 */
export interface MelatoninSample {
  /** Time of sample (HH:MM format or Date) */
  time: string | Date;
  /** Melatonin level in pg/mL */
  level_pg_ml: number;
}

/**
 * DLMO calculation result
 */
export interface DLMOResult {
  /** Estimated DLMO time */
  dlmoTime: string;
  /** Threshold used for calculation */
  threshold_pg_ml: number;
  /** Confidence score (0-1) */
  confidence: number;
  /** Whether interpolation was used */
  interpolated: boolean;
}

/**
 * Parse time to hours (0-24)
 */
function parseTimeToHours(time: string | Date): number {
  if (time instanceof Date) {
    return time.getHours() + time.getMinutes() / 60;
  }
  const parts = time.split(':').map(Number);
  return parts[0] + (parts[1] || 0) / 60;
}

/**
 * Convert hours to time string
 */
function hoursToTimeString(hours: number): string {
  while (hours >= 24) hours -= 24;
  while (hours < 0) hours += 24;

  const h = Math.floor(hours);
  const m = Math.round((hours - h) * 60);

  return `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}`;
}

/**
 * Calculate Dim Light Melatonin Onset (DLMO) from serial samples
 *
 * DLMO is the time when melatonin levels exceed a threshold (typically 3-10 pg/mL for saliva)
 * during dim light conditions.
 *
 * @param samples Array of melatonin samples with time and level
 * @param threshold Threshold for DLMO determination (default 3 pg/mL for saliva)
 * @returns DLMO calculation result
 */
export function calculateDLMO(
  samples: MelatoninSample[],
  threshold: number = 3
): DLMOResult | null {
  if (samples.length < 3) {
    return null;
  }

  // Sort samples by time
  const sortedSamples = [...samples].sort((a, b) => {
    return parseTimeToHours(a.time) - parseTimeToHours(b.time);
  });

  // Handle times that cross midnight
  const times = sortedSamples.map((s) => {
    let t = parseTimeToHours(s.time);
    // Assume samples taken in evening/night period
    if (t < 12) t += 24;
    return t;
  });

  const levels = sortedSamples.map((s) => s.level_pg_ml);

  // Find crossing point
  let dlmoTime: number | null = null;
  let interpolated = false;

  for (let i = 0; i < levels.length - 1; i++) {
    if (levels[i] <= threshold && levels[i + 1] > threshold) {
      // Linear interpolation to find exact crossing time
      const t1 = times[i];
      const t2 = times[i + 1];
      const l1 = levels[i];
      const l2 = levels[i + 1];

      const fraction = (threshold - l1) / (l2 - l1);
      dlmoTime = t1 + fraction * (t2 - t1);
      interpolated = true;
      break;
    } else if (levels[i] > threshold && (i === 0 || levels[i - 1] <= threshold)) {
      // First sample above threshold
      dlmoTime = times[i];
      interpolated = false;
      break;
    }
  }

  if (dlmoTime === null) {
    // Could not determine DLMO - levels may not have crossed threshold
    return null;
  }

  // Calculate confidence based on sampling frequency and level rise
  let confidence = 0.7; // Base confidence

  // Better confidence with more samples
  if (samples.length >= 6) confidence += 0.1;
  if (samples.length >= 10) confidence += 0.1;

  // Better confidence with interpolation (samples bracket the threshold)
  if (interpolated) confidence += 0.1;

  confidence = Math.min(1, confidence);

  return {
    dlmoTime: hoursToTimeString(dlmoTime),
    threshold_pg_ml: threshold,
    confidence,
    interpolated
  };
}

/**
 * Calculate Dim Light Melatonin Offset (DLMOff)
 *
 * @param samples Array of melatonin samples covering the night/morning
 * @param threshold Threshold for DLMOff determination
 * @returns DLMOff time or null
 */
export function calculateDLMOff(
  samples: MelatoninSample[],
  threshold: number = 3
): string | null {
  if (samples.length < 3) {
    return null;
  }

  // Sort samples by time
  const sortedSamples = [...samples].sort((a, b) => {
    return parseTimeToHours(a.time) - parseTimeToHours(b.time);
  });

  const times = sortedSamples.map((s) => {
    let t = parseTimeToHours(s.time);
    // Handle overnight samples
    if (t < 12) t += 24;
    return t;
  });

  const levels = sortedSamples.map((s) => s.level_pg_ml);

  // Find where melatonin falls below threshold (after peak)
  let peakFound = false;
  let dlmoffTime: number | null = null;

  for (let i = 1; i < levels.length - 1; i++) {
    // Find peak
    if (levels[i] > levels[i - 1] && levels[i] > levels[i + 1]) {
      peakFound = true;
    }

    // After peak, find where it falls below threshold
    if (peakFound && levels[i] >= threshold && levels[i + 1] < threshold) {
      const t1 = times[i];
      const t2 = times[i + 1];
      const l1 = levels[i];
      const l2 = levels[i + 1];

      const fraction = (l1 - threshold) / (l1 - l2);
      dlmoffTime = t1 + fraction * (t2 - t1);
      break;
    }
  }

  if (dlmoffTime === null) {
    return null;
  }

  return hoursToTimeString(dlmoffTime);
}

/**
 * Calculate melatonin synthesis duration (DLMO to DLMOff)
 *
 * @param dlmo DLMO time string
 * @param dlmoff DLMOff time string
 * @returns Duration in hours
 */
export function calculateMelatoninDuration(dlmo: string, dlmoff: string): number {
  let dlmoHours = parseTimeToHours(dlmo);
  let dlmoffHours = parseTimeToHours(dlmoff);

  // Handle overnight duration
  if (dlmoHours > 12) dlmoHours = dlmoHours;
  if (dlmoffHours < 12) dlmoffHours += 24;

  return dlmoffHours - dlmoHours;
}

/**
 * Estimate DLMO from habitual bedtime (population-based estimate)
 *
 * Research shows DLMO typically occurs 2-3 hours before habitual bedtime.
 * This is a rough estimate when direct measurement isn't available.
 *
 * @param habitualBedtime Typical bedtime in HH:MM format
 * @param chronotypeAdjustment Adjustment for chronotype (-0.5 to +0.5 hours)
 * @returns Estimated DLMO time
 */
export function estimateDLMOFromBedtime(
  habitualBedtime: string,
  chronotypeAdjustment: number = 0
): string {
  const bedtimeHours = parseTimeToHours(habitualBedtime);
  const baseOffset = 2.5; // Average DLMO-to-bedtime interval

  let dlmoHours = bedtimeHours - baseOffset + chronotypeAdjustment;

  return hoursToTimeString(dlmoHours);
}

/**
 * Calculate phase angle between DLMO and sleep onset
 *
 * Healthy adults typically have a phase angle of 2-3 hours
 * (DLMO occurs 2-3 hours before sleep onset)
 *
 * @param dlmo DLMO time
 * @param sleepOnset Sleep onset time
 * @returns Phase angle in hours (positive = DLMO before sleep onset)
 */
export function calculatePhaseAngleToSleep(dlmo: string, sleepOnset: string): number {
  let dlmoHours = parseTimeToHours(dlmo);
  let sleepHours = parseTimeToHours(sleepOnset);

  // Handle overnight times
  if (dlmoHours > 18) dlmoHours = dlmoHours;
  if (sleepHours < 12) sleepHours += 24;

  return sleepHours - dlmoHours;
}

/**
 * Interpret phase angle
 *
 * @param phaseAngle Phase angle in hours
 * @returns Interpretation of phase angle
 */
export function interpretPhaseAngle(phaseAngle: number): {
  status: 'normal' | 'early_sleep' | 'late_sleep';
  interpretation: string;
} {
  if (phaseAngle >= 2 && phaseAngle <= 3) {
    return {
      status: 'normal',
      interpretation: 'Optimal alignment between circadian phase and sleep timing'
    };
  } else if (phaseAngle < 2) {
    return {
      status: 'early_sleep',
      interpretation: 'Sleeping before circadian-optimal time; may experience difficulty falling asleep'
    };
  } else {
    return {
      status: 'late_sleep',
      interpretation: 'Sleeping after circadian-optimal time; may have accumulated sleep pressure'
    };
  }
}

export default {
  calculateDLMO,
  calculateDLMOff,
  calculateMelatoninDuration,
  estimateDLMOFromBedtime,
  calculatePhaseAngleToSleep,
  interpretPhaseAngle
};
