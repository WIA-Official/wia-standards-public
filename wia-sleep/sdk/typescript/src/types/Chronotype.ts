/**
 * WIA-SLEEP SDK - Chronotype Types
 *
 * Type definitions for chronotype assessment, classification, and profiles.
 */

import { TimeString, DurationMinutes } from './SleepRecord';

/**
 * Chronotype classification categories
 */
export enum ChronotypeClass {
  EXTREME_EARLY = 'extreme_early',
  MODERATE_EARLY = 'moderate_early',
  INTERMEDIATE = 'intermediate',
  MODERATE_LATE = 'moderate_late',
  EXTREME_LATE = 'extreme_late'
}

/**
 * Sleep schedule for workdays or free days
 */
export interface SleepSchedule {
  /** Time of going to bed */
  bedtime: TimeString;
  /** Time spent in bed before attempting sleep */
  sleepPrepTime_min?: DurationMinutes;
  /** Time to fall asleep */
  sleepLatency_min?: DurationMinutes;
  /** Time of waking up */
  wakeTime: TimeString;
  /** Time of getting out of bed */
  getUpTime?: TimeString;
  /** Total sleep duration in hours */
  sleepDuration_hours: number;
  /** Duration of sleep inertia */
  sleepInertia_min?: DurationMinutes;
}

/**
 * MCTQ (Munich Chronotype Questionnaire) responses
 */
export interface MCTQResponses {
  /** Workday sleep schedule */
  workdays: SleepSchedule;
  /** Free day sleep schedule */
  freeDays: SleepSchedule;
  /** Number of work days per week */
  workDaysPerWeek: number;
  /** Uses alarm on work days */
  alarmUsage: boolean;
  /** Uses alarm on free days */
  alarmUsageFreeDays?: boolean;
}

/**
 * Actigraphy-based assessment data
 */
export interface ActimetryData {
  /** Number of recording days */
  recordingDays: number;
  /** Average sleep onset time */
  averageSleepOnset: TimeString;
  /** Average wake time */
  averageWakeTime: TimeString;
  /** Sleep midpoint */
  sleepMidpoint: TimeString;
  /** Interdaily stability (0-1) */
  interdailyStability: number;
  /** Intradaily variability */
  intradailyVariability: number;
  /** Relative amplitude (0-1) */
  relativeAmplitude: number;
}

/**
 * DLMO measurement method
 */
export enum DLMOMethod {
  SALIVA = 'saliva',
  PLASMA = 'plasma',
  URINE_6SMT = 'urine_6SMT'
}

/**
 * Biological markers for chronotype
 */
export interface ChronotypeBiologicalMarkers {
  /** Dim Light Melatonin Onset */
  dlmo?: {
    time: TimeString;
    method: DLMOMethod;
    threshold_pg_ml?: number;
    confidence?: number;
  };
  /** Core body temperature nadir */
  coreBodyTempNadir?: {
    time: TimeString;
    temperature_C?: number;
  };
  /** Cortisol peak */
  cortisolPeak?: {
    time: TimeString;
    level_nmol_L?: number;
  };
}

/**
 * Time window for optimal sleep
 */
export interface TimeWindow {
  /** Start time */
  start: TimeString;
  /** End time */
  end: TimeString;
}

/**
 * Chronotype assessment input
 */
export interface ChronotypeAssessmentInput {
  /** Subject identifier */
  subjectId: string;
  /** MCTQ questionnaire responses */
  mctqResponses: MCTQResponses;
  /** Optional actigraphy data */
  actimetryData?: ActimetryData;
  /** Optional biological markers */
  biologicalMarkers?: ChronotypeBiologicalMarkers;
}

/**
 * Chronotype assessment result
 */
export interface ChronotypeResult {
  /** Assessment identifier */
  assessmentId: string;
  /** Subject identifier */
  subjectId: string;
  /** Assessment date */
  assessmentDate: string;
  /** MSFsc (corrected mid-sleep on free days) */
  msfsc: number;
  /** Chronotype classification */
  chronotype: ChronotypeClass;
  /** Social jetlag in hours */
  socialJetlag_hours: number;
  /** Optimal sleep window */
  optimalSleepWindow: {
    bedtime: TimeString;
    wakeTime: TimeString;
  };
  /** Normalized chronotype score (0=extreme early, 100=extreme late) */
  chronotypeScore?: number;
  /** Assessment confidence (0-1) */
  confidence: number;
}

/**
 * Complete chronotype profile
 */
export interface ChronotypeProfile {
  /** Subject identifier */
  subjectId: string;
  /** MCTQ score (MSFsc) */
  mctqScore: number;
  /** Chronotype classification */
  classification: ChronotypeClass;
  /** Dim Light Melatonin Onset time */
  dlmo?: TimeString;
  /** Core body temperature nadir time */
  cbtNadir?: TimeString;
  /** Optimal sleep window */
  optimalSleepWindow: {
    bedtime: TimeString;
    wakeTime: TimeString;
  };
  /** Optimal wake time */
  optimalWakeTime: TimeString;
  /** Social jetlag in hours */
  socialJetlag: number;
  /** Profile last updated */
  lastUpdated: string;
}

/**
 * Chronotype recommendation
 */
export interface ChronotypeRecommendation {
  /** Recommendation category */
  category: 'sleep_timing' | 'light_exposure' | 'light_avoidance' | 'meal_timing' | 'exercise_timing';
  /** Recommendation text */
  recommendation: string;
  /** Priority level */
  priority: 'critical' | 'high' | 'medium' | 'low';
}

/**
 * Detailed chronotype interpretation
 */
export interface ChronotypeInterpretation {
  /** Summary of chronotype assessment */
  summary: string;
  /** Key insights */
  insights: string[];
  /** Recommendations */
  recommendations: ChronotypeRecommendation[];
}
