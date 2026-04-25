/**
 * WIA-SLEEP SDK - Circadian Calculator Service
 *
 * Service for calculating circadian phase, timing, and related parameters.
 */

import {
  CircadianPhase,
  CircadianPhaseEstimate,
  EntrainmentStatus,
  AlertnessWindow,
  AlertnessLevel,
  MelatoninRhythm,
  TemperatureRhythm,
  ActivityRhythm
} from '../types/CircadianMarker';
import { ChronotypeProfile, ChronotypeClass } from '../types/Chronotype';
import { TimeString } from '../types/SleepRecord';

/**
 * Convert time string to decimal hours (0-24)
 */
function timeToHours(time: TimeString): number {
  const [hours, minutes] = time.split(':').map(Number);
  return hours + minutes / 60;
}

/**
 * Convert decimal hours to time string
 */
function hoursToTime(hours: number): TimeString {
  while (hours >= 24) hours -= 24;
  while (hours < 0) hours += 24;

  const h = Math.floor(hours);
  const m = Math.round((hours - h) * 60);

  return `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:00`;
}

/**
 * Calculate circular mean for times
 */
function circularMean(times: number[]): number {
  const radians = times.map((t) => (t / 24) * 2 * Math.PI);
  const sinSum = radians.reduce((sum, r) => sum + Math.sin(r), 0);
  const cosSum = radians.reduce((sum, r) => sum + Math.cos(r), 0);
  let mean = Math.atan2(sinSum / times.length, cosSum / times.length);
  if (mean < 0) mean += 2 * Math.PI;
  return (mean / (2 * Math.PI)) * 24;
}

/**
 * Circadian rhythm calculation and phase estimation
 */
export class CircadianCalculator {
  /**
   * Estimate DLMO from chronotype profile
   */
  static estimateDLMOFromChronotype(profile: ChronotypeProfile): TimeString {
    // DLMO typically occurs ~2.5 hours before habitual bedtime
    const bedtimeHours = timeToHours(profile.optimalSleepWindow.bedtime);
    let dlmoHours = bedtimeHours - 2.5;
    return hoursToTime(dlmoHours);
  }

  /**
   * Estimate CBT nadir from DLMO
   * CBT nadir typically occurs ~7 hours after DLMO
   */
  static estimateCBTNadirFromDLMO(dlmo: TimeString): TimeString {
    const dlmoHours = timeToHours(dlmo);
    const cbtNadirHours = dlmoHours + 7;
    return hoursToTime(cbtNadirHours);
  }

  /**
   * Estimate current circadian phase based on available markers
   */
  static estimatePhase(
    currentTime: Date,
    dlmo?: TimeString,
    cbtNadir?: TimeString,
    chronotype?: ChronotypeClass
  ): CircadianPhase {
    // Use DLMO as primary phase marker
    let referencePhase: number;

    if (dlmo) {
      referencePhase = timeToHours(dlmo);
    } else if (cbtNadir) {
      // CBT nadir is ~7 hours after DLMO
      referencePhase = timeToHours(cbtNadir) - 7;
    } else if (chronotype) {
      // Estimate from chronotype
      referencePhase = this.estimateDLMOFromChronotype({
        mctqScore: this.chronotypeToMsfsc(chronotype),
        classification: chronotype,
        optimalSleepWindow: this.getDefaultSleepWindow(chronotype),
        optimalWakeTime: '07:00:00',
        socialJetlag: 0,
        subjectId: '',
        lastUpdated: ''
      }).slice(0, 5) as unknown as number;
    } else {
      // Default to average DLMO of 21:00
      referencePhase = 21;
    }

    // Normalize reference phase
    while (referencePhase < 0) referencePhase += 24;
    while (referencePhase >= 24) referencePhase -= 24;

    // Calculate current phase (0 = DLMO, circadian hours)
    const currentHours = currentTime.getHours() + currentTime.getMinutes() / 60;
    let phaseAngle = currentHours - referencePhase;
    while (phaseAngle < 0) phaseAngle += 24;
    while (phaseAngle >= 24) phaseAngle -= 24;

    // Estimate entrainment status (simplified)
    let entrainmentStatus = EntrainmentStatus.ENTRAINED;

    // Calculate CBT nadir phase (should be ~7 hours after DLMO)
    const cbtNadirPhase = 7;

    return {
      phaseAngle_hours: phaseAngle,
      estimatedPhase_hours: phaseAngle,
      entrainmentStatus,
      tau_hours: 24.2, // Average human tau
      phaseAngleToSleep: 2.5 // Average DLMO to sleep onset
    };
  }

  /**
   * Get estimated chronotype MSFsc
   */
  private static chronotypeToMsfsc(chronotype: ChronotypeClass): number {
    switch (chronotype) {
      case ChronotypeClass.EXTREME_EARLY:
        return 2.0;
      case ChronotypeClass.MODERATE_EARLY:
        return 3.0;
      case ChronotypeClass.INTERMEDIATE:
        return 4.0;
      case ChronotypeClass.MODERATE_LATE:
        return 5.5;
      case ChronotypeClass.EXTREME_LATE:
        return 7.0;
    }
  }

  /**
   * Get default sleep window for chronotype
   */
  private static getDefaultSleepWindow(
    chronotype: ChronotypeClass
  ): { bedtime: TimeString; wakeTime: TimeString } {
    switch (chronotype) {
      case ChronotypeClass.EXTREME_EARLY:
        return { bedtime: '21:00:00', wakeTime: '05:00:00' };
      case ChronotypeClass.MODERATE_EARLY:
        return { bedtime: '22:00:00', wakeTime: '06:00:00' };
      case ChronotypeClass.INTERMEDIATE:
        return { bedtime: '23:00:00', wakeTime: '07:00:00' };
      case ChronotypeClass.MODERATE_LATE:
        return { bedtime: '00:00:00', wakeTime: '08:00:00' };
      case ChronotypeClass.EXTREME_LATE:
        return { bedtime: '01:00:00', wakeTime: '09:00:00' };
    }
  }

  /**
   * Generate full circadian phase estimate with alertness predictions
   */
  static generatePhaseEstimate(
    subjectId: string,
    chronotype: ChronotypeProfile,
    currentTime: Date = new Date()
  ): CircadianPhaseEstimate {
    const dlmo = chronotype.dlmo || this.estimateDLMOFromChronotype(chronotype);
    const phase = this.estimatePhase(currentTime, dlmo);

    // Generate alertness windows based on circadian phase
    const alertnessWindows = this.generateAlertnessWindows(dlmo);

    // Calculate recommended phase shift based on social jetlag
    const recommendedShift =
      chronotype.socialJetlag > 1 ? -chronotype.socialJetlag * 0.5 : 0;

    return {
      subjectId,
      timestamp: currentTime.toISOString(),
      currentPhase: phase.estimatedPhase_hours,
      phaseAngle: phase.phaseAngle_hours,
      entrainmentStatus: phase.entrainmentStatus,
      recommendedShift,
      optimalAlertness: alertnessWindows
    };
  }

  /**
   * Generate alertness windows based on DLMO
   */
  static generateAlertnessWindows(dlmo: TimeString): AlertnessWindow[] {
    const dlmoHours = timeToHours(dlmo);

    // Peak alertness: ~6-10 hours after CBT nadir (which is ~7h after DLMO)
    // So peak is ~13-17 hours after DLMO, or ~6-10 hours before DLMO
    const peakStart = dlmoHours - 10;
    const peakEnd = dlmoHours - 6;

    // Good alertness extends around peak
    const goodMorningStart = dlmoHours - 12;
    const goodAfternoonEnd = dlmoHours - 4;

    // Post-lunch dip (circadian trough)
    const dipStart = dlmoHours - 8;
    const dipEnd = dlmoHours - 6;

    // Low alertness in evening before sleep
    const lowStart = dlmoHours - 2;
    const lowEnd = dlmoHours + 2;

    return [
      {
        startTime: hoursToTime(goodMorningStart),
        endTime: hoursToTime(peakStart),
        level: 'good' as AlertnessLevel
      },
      {
        startTime: hoursToTime(peakStart),
        endTime: hoursToTime(dipStart),
        level: 'peak' as AlertnessLevel
      },
      {
        startTime: hoursToTime(dipStart),
        endTime: hoursToTime(dipEnd),
        level: 'moderate' as AlertnessLevel
      },
      {
        startTime: hoursToTime(dipEnd),
        endTime: hoursToTime(peakEnd),
        level: 'peak' as AlertnessLevel
      },
      {
        startTime: hoursToTime(peakEnd),
        endTime: hoursToTime(lowStart),
        level: 'good' as AlertnessLevel
      },
      {
        startTime: hoursToTime(lowStart),
        endTime: hoursToTime(lowEnd),
        level: 'low' as AlertnessLevel
      }
    ];
  }

  /**
   * Calculate phase response to light exposure
   * Returns expected phase shift in hours (negative = advance, positive = delay)
   */
  static calculateLightPhaseResponse(
    lightTimingRelativeToCBTNadir_hours: number,
    intensity_lux: number,
    duration_min: number
  ): number {
    // Simplified Phase Response Curve (PRC)
    // Light before CBT nadir delays phase
    // Light after CBT nadir advances phase

    // Maximum phase shift achievable per day is ~2-3 hours
    const maxShift = 2.0;

    // Light effectiveness based on intensity (saturates around 10000 lux)
    const intensityFactor = Math.min(1, Math.log10(intensity_lux + 1) / 4);

    // Duration factor (diminishing returns after 30 min)
    const durationFactor = Math.min(1, duration_min / 60);

    // Phase response based on timing
    let phaseResponse: number;
    const timing = lightTimingRelativeToCBTNadir_hours;

    if (timing >= -6 && timing < 0) {
      // Delay zone (before CBT nadir)
      phaseResponse = maxShift * Math.sin(((timing + 3) / 3) * (Math.PI / 2));
    } else if (timing >= 0 && timing <= 6) {
      // Advance zone (after CBT nadir)
      phaseResponse = -maxShift * Math.sin((timing / 3) * (Math.PI / 2));
    } else {
      // Dead zone - minimal response
      phaseResponse = 0;
    }

    return phaseResponse * intensityFactor * durationFactor;
  }

  /**
   * Calculate phase response to melatonin administration
   */
  static calculateMelatoninPhaseResponse(
    adminTimeRelativeToDLMO_hours: number,
    dose_mg: number
  ): number {
    // Melatonin PRC is roughly opposite to light PRC
    // Before DLMO: advances phase
    // After DLMO: delays phase

    const maxShift = 1.5; // Melatonin typically has smaller effect than light

    // Dose response (saturates around 0.5-1 mg)
    const doseFactor = Math.min(1, dose_mg / 0.5);

    let phaseResponse: number;
    const timing = adminTimeRelativeToDLMO_hours;

    if (timing >= -6 && timing < 0) {
      // Advance zone (before DLMO)
      phaseResponse = -maxShift * Math.sin(((timing + 3) / 3) * (Math.PI / 2));
    } else if (timing >= 0 && timing <= 6) {
      // Delay zone (after DLMO) - usually we don't use melatonin here
      phaseResponse = maxShift * Math.sin((timing / 3) * (Math.PI / 2)) * 0.5;
    } else {
      phaseResponse = 0;
    }

    return phaseResponse * doseFactor;
  }

  /**
   * Assess entrainment status from activity rhythm data
   */
  static assessEntrainment(activityRhythm: ActivityRhythm): EntrainmentStatus {
    const { interdailyStability, intradailyVariability, relativeAmplitude } =
      activityRhythm;

    // High IS, low IV, high RA = well-entrained
    if (interdailyStability > 0.6 && intradailyVariability < 1.0 && relativeAmplitude > 0.8) {
      return EntrainmentStatus.ENTRAINED;
    }

    // Low IS suggests free-running or irregular
    if (interdailyStability < 0.4) {
      if (relativeAmplitude > 0.6) {
        return EntrainmentStatus.FREE_RUNNING;
      } else {
        return EntrainmentStatus.IRREGULAR;
      }
    }

    // Check for phase disorders based on L5 onset
    if (activityRhythm.l5?.onset) {
      const l5Hours = timeToHours(activityRhythm.l5.onset);
      if (l5Hours > 3 && l5Hours < 6) {
        return EntrainmentStatus.DELAYED;
      } else if (l5Hours < 23 && l5Hours > 20) {
        return EntrainmentStatus.ADVANCED;
      }
    }

    return EntrainmentStatus.ENTRAINED;
  }
}

export default CircadianCalculator;
