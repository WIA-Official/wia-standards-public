/**
 * WIA-SLEEP SDK - Chronotype Service
 *
 * Service for chronotype assessment and classification based on
 * MCTQ (Munich Chronotype Questionnaire) and biological markers.
 */

import {
  ChronotypeClass,
  MCTQResponses,
  SleepSchedule,
  ActimetryData,
  ChronotypeBiologicalMarkers,
  ChronotypeAssessmentInput,
  ChronotypeResult,
  ChronotypeProfile,
  ChronotypeRecommendation,
  ChronotypeInterpretation
} from '../types/Chronotype';
import { TimeString } from '../types/SleepRecord';

/**
 * Parse time string to decimal hours
 */
function timeToDecimalHours(time: TimeString): number {
  const [hours, minutes] = time.split(':').map(Number);
  let decimal = hours + minutes / 60;
  // Handle times after midnight (e.g., 01:00 should be 25.0 for calculations)
  if (decimal < 12) {
    decimal += 24;
  }
  return decimal;
}

/**
 * Convert decimal hours to time string
 */
function decimalHoursToTime(decimal: number): TimeString {
  // Normalize to 24-hour format
  while (decimal >= 24) decimal -= 24;
  while (decimal < 0) decimal += 24;

  const hours = Math.floor(decimal);
  const minutes = Math.round((decimal - hours) * 60);

  return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:00`;
}

/**
 * Calculate sleep onset from schedule
 */
function calculateSleepOnset(schedule: SleepSchedule): number {
  const bedtime = timeToDecimalHours(schedule.bedtime);
  const prepTime = (schedule.sleepPrepTime_min || 0) / 60;
  const latency = (schedule.sleepLatency_min || 15) / 60;
  return bedtime + prepTime + latency;
}

/**
 * Calculate sleep midpoint
 */
function calculateSleepMidpoint(onset: number, duration: number): number {
  return onset + duration / 2;
}

/**
 * Chronotype assessment and classification service
 */
export class ChronotypeService {
  /**
   * Perform MCTQ-based chronotype assessment
   */
  static assessChronotype(input: ChronotypeAssessmentInput): ChronotypeResult {
    const { mctqResponses, biologicalMarkers, actimetryData } = input;

    // Calculate MSF (mid-sleep on free days)
    const freeOnset = calculateSleepOnset(mctqResponses.freeDays);
    const freeDuration = mctqResponses.freeDays.sleepDuration_hours;
    const msf = calculateSleepMidpoint(freeOnset, freeDuration);

    // Calculate average sleep duration
    const workDuration = mctqResponses.workdays.sleepDuration_hours;
    const avgSleepDuration =
      (workDuration * mctqResponses.workDaysPerWeek +
        freeDuration * (7 - mctqResponses.workDaysPerWeek)) /
      7;

    // Calculate sleep debt on work days
    const sleepDebt = Math.max(0, avgSleepDuration - workDuration);

    // Calculate MSFsc (sleep-corrected MSF)
    let msfsc: number;
    if (freeDuration > avgSleepDuration && !mctqResponses.alarmUsageFreeDays) {
      msfsc = msf - sleepDebt / 2;
    } else {
      msfsc = msf;
    }

    // Normalize MSFsc to 0-12 range
    const normalizedMsfsc = msfsc > 24 ? msfsc - 24 : msfsc;

    // Calculate social jetlag
    const workOnset = calculateSleepOnset(mctqResponses.workdays);
    const msw = calculateSleepMidpoint(workOnset, workDuration);
    const socialJetlag = Math.abs(msf - msw);

    // Classify chronotype
    const chronotype = this.classifyChronotype(normalizedMsfsc);

    // Calculate optimal sleep window
    const sleepNeed = avgSleepDuration;
    const optimalMidpoint = normalizedMsfsc;
    const optimalBedtime = optimalMidpoint - sleepNeed / 2;
    const optimalWake = optimalMidpoint + sleepNeed / 2;

    // Calculate confidence based on data quality
    let confidence = 0.7; // Base confidence from MCTQ alone
    if (actimetryData) {
      confidence += 0.15;
    }
    if (biologicalMarkers?.dlmo) {
      confidence += 0.15;
    }

    return {
      assessmentId: crypto.randomUUID ? crypto.randomUUID() : `assessment-${Date.now()}`,
      subjectId: input.subjectId,
      assessmentDate: new Date().toISOString(),
      msfsc: normalizedMsfsc,
      chronotype,
      socialJetlag_hours: socialJetlag,
      optimalSleepWindow: {
        bedtime: decimalHoursToTime(optimalBedtime),
        wakeTime: decimalHoursToTime(optimalWake)
      },
      chronotypeScore: this.chronotypeToScore(normalizedMsfsc),
      confidence: Math.min(1, confidence)
    };
  }

  /**
   * Classify chronotype based on MSFsc
   */
  static classifyChronotype(msfsc: number): ChronotypeClass {
    if (msfsc < 2.5) {
      return ChronotypeClass.EXTREME_EARLY;
    } else if (msfsc < 3.5) {
      return ChronotypeClass.MODERATE_EARLY;
    } else if (msfsc < 5.0) {
      return ChronotypeClass.INTERMEDIATE;
    } else if (msfsc < 6.5) {
      return ChronotypeClass.MODERATE_LATE;
    } else {
      return ChronotypeClass.EXTREME_LATE;
    }
  }

  /**
   * Convert MSFsc to normalized score (0-100)
   */
  static chronotypeToScore(msfsc: number): number {
    // Map MSFsc range (roughly 1-8) to 0-100
    const minMsfsc = 1;
    const maxMsfsc = 8;
    const score = ((msfsc - minMsfsc) / (maxMsfsc - minMsfsc)) * 100;
    return Math.max(0, Math.min(100, score));
  }

  /**
   * Estimate DLMO from MCTQ responses
   * DLMO typically occurs 2-3 hours before habitual bedtime
   */
  static estimateDLMO(mctqResponses: MCTQResponses): TimeString {
    const avgBedtime =
      (timeToDecimalHours(mctqResponses.workdays.bedtime) * mctqResponses.workDaysPerWeek +
        timeToDecimalHours(mctqResponses.freeDays.bedtime) *
          (7 - mctqResponses.workDaysPerWeek)) /
      7;

    const estimatedDLMO = avgBedtime - 2.5; // DLMO ~2.5 hours before bedtime
    return decimalHoursToTime(estimatedDLMO);
  }

  /**
   * Estimate CBT nadir from DLMO
   * CBT nadir typically occurs ~7 hours after DLMO
   */
  static estimateCBTNadir(dlmo: TimeString): TimeString {
    const dlmoHours = timeToDecimalHours(dlmo);
    const cbtNadir = dlmoHours + 7;
    return decimalHoursToTime(cbtNadir);
  }

  /**
   * Create full chronotype profile
   */
  static createProfile(result: ChronotypeResult): ChronotypeProfile {
    return {
      subjectId: result.subjectId,
      mctqScore: result.msfsc,
      classification: result.chronotype,
      optimalSleepWindow: result.optimalSleepWindow,
      optimalWakeTime: result.optimalSleepWindow.wakeTime,
      socialJetlag: result.socialJetlag_hours,
      lastUpdated: new Date().toISOString()
    };
  }

  /**
   * Generate interpretation and recommendations
   */
  static interpret(result: ChronotypeResult): ChronotypeInterpretation {
    const insights: string[] = [];
    const recommendations: ChronotypeRecommendation[] = [];

    // Generate insights based on chronotype
    switch (result.chronotype) {
      case ChronotypeClass.EXTREME_EARLY:
        insights.push('Strong morning preference with early DLMO');
        insights.push('Peak cognitive performance typically in early morning hours');
        break;
      case ChronotypeClass.MODERATE_EARLY:
        insights.push('Morning-oriented circadian phase');
        insights.push('Natural wake time aligns well with typical work schedules');
        break;
      case ChronotypeClass.INTERMEDIATE:
        insights.push('Intermediate chronotype with flexible sleep timing');
        insights.push('Can adapt to various schedules relatively easily');
        break;
      case ChronotypeClass.MODERATE_LATE:
        insights.push('Evening-oriented circadian phase');
        insights.push('May experience difficulty with early work schedules');
        break;
      case ChronotypeClass.EXTREME_LATE:
        insights.push('Strong evening preference with late DLMO');
        insights.push('Significant mismatch with typical social schedules');
        break;
    }

    // Social jetlag insights
    if (result.socialJetlag_hours > 2) {
      insights.push(
        `Significant social jetlag of ${result.socialJetlag_hours.toFixed(1)} hours detected`
      );
      insights.push('High social jetlag is associated with metabolic and mood issues');
      recommendations.push({
        category: 'sleep_timing',
        recommendation:
          'Reduce difference between workday and free day sleep schedules',
        priority: 'high'
      });
    } else if (result.socialJetlag_hours > 1) {
      insights.push(
        `Moderate social jetlag of ${result.socialJetlag_hours.toFixed(1)} hours`
      );
    }

    // Generate recommendations based on chronotype and social jetlag
    if (
      result.chronotype === ChronotypeClass.MODERATE_LATE ||
      result.chronotype === ChronotypeClass.EXTREME_LATE
    ) {
      recommendations.push({
        category: 'light_exposure',
        recommendation:
          'Morning bright light exposure (10,000 lux for 30 min upon waking)',
        priority: 'high'
      });
      recommendations.push({
        category: 'light_avoidance',
        recommendation: 'Reduce evening blue light exposure after 20:00',
        priority: 'high'
      });
      recommendations.push({
        category: 'sleep_timing',
        recommendation: 'Gradually advance bedtime by 15 minutes per week',
        priority: 'medium'
      });
    }

    if (
      result.chronotype === ChronotypeClass.EXTREME_EARLY ||
      result.chronotype === ChronotypeClass.MODERATE_EARLY
    ) {
      recommendations.push({
        category: 'light_exposure',
        recommendation: 'Evening light exposure to maintain phase if needed',
        priority: 'low'
      });
      recommendations.push({
        category: 'light_avoidance',
        recommendation: 'Avoid bright light in early morning if phase advance is a concern',
        priority: 'low'
      });
    }

    // General recommendations
    recommendations.push({
      category: 'sleep_timing',
      recommendation: `Aim for bedtime around ${result.optimalSleepWindow.bedtime.slice(0, 5)}`,
      priority: 'medium'
    });

    const summary = `${result.chronotype.replace('_', ' ')} chronotype with ${
      result.socialJetlag_hours > 1 ? 'notable' : 'minimal'
    } social jetlag`;

    return {
      summary,
      insights,
      recommendations
    };
  }
}

export default ChronotypeService;
