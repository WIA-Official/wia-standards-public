/**
 * WIA-SLEEP SDK - Circadian Marker Types
 *
 * Type definitions for circadian rhythm biomarkers and phase indicators.
 */

import { TimeString } from './SleepRecord';

/**
 * Entrainment status categories
 */
export enum EntrainmentStatus {
  ENTRAINED = 'entrained',
  FREE_RUNNING = 'free_running',
  DELAYED = 'delayed',
  ADVANCED = 'advanced',
  IRREGULAR = 'irregular'
}

/**
 * Melatonin measurement sample type
 */
export enum MelatoninSampleType {
  SALIVA = 'saliva',
  PLASMA = 'plasma'
}

/**
 * Melatonin time point measurement
 */
export interface MelatoninMeasurement {
  time: TimeString;
  level_pg_ml: number;
  sampleType: MelatoninSampleType;
}

/**
 * Melatonin rhythm data
 */
export interface MelatoninRhythm {
  /** Dim Light Melatonin Onset */
  dlmo?: {
    time: TimeString;
    method: MelatoninSampleType;
    threshold_pg_ml?: number;
    confidence?: number;
  };
  /** Dim Light Melatonin Offset */
  dlmoff?: {
    time: TimeString;
  };
  /** Duration of elevated melatonin */
  melatoninDuration_hours?: number;
  /** Peak melatonin level */
  peakLevel_pg_ml?: number;
  /** Peak time */
  peakTime?: TimeString;
  /** Time series measurements */
  measurements?: MelatoninMeasurement[];
}

/**
 * Cortisol Awakening Response (CAR)
 */
export interface CortisolAwakeningResponse {
  /** Cortisol at awakening */
  wakeLevel_nmol_L: number;
  /** Cortisol 30 min after awakening */
  peak30Level_nmol_L: number;
  /** CAR magnitude */
  carMagnitude_nmol_L: number;
  /** Area under curve */
  carAuc_nmol_L_min?: number;
}

/**
 * Cortisol rhythm data
 */
export interface CortisolRhythm {
  /** Cortisol awakening response */
  awakeningResponse?: CortisolAwakeningResponse;
  /** Diurnal cortisol slope */
  diurnalSlope?: number;
  /** Time of cortisol nadir */
  nadirTime?: TimeString;
  /** Nadir level */
  nadirLevel_nmol_L?: number;
}

/**
 * Core body temperature measurement
 */
export interface TemperatureMeasurement {
  time: string;
  temperature_C: number;
}

/**
 * Core body temperature rhythm
 */
export interface TemperatureRhythm {
  /** Temperature nadir */
  nadir?: {
    time: TimeString;
    temperature_C: number;
  };
  /** Temperature peak */
  peak?: {
    time: TimeString;
    temperature_C: number;
  };
  /** Rhythm amplitude */
  amplitude_C?: number;
  /** Time series measurements */
  measurements?: TemperatureMeasurement[];
}

/**
 * Heart Rate Variability metrics
 */
export interface HRVMetrics {
  /** RMSSD in milliseconds */
  rmssd_ms: number;
  /** SDNN in milliseconds */
  sdnn_ms?: number;
  /** LF/HF ratio */
  lf_hf_ratio: number;
  /** High frequency power */
  hf_power_ms2?: number;
}

/**
 * HRV circadian modulation
 */
export interface HRVCircadianModulation {
  amplitude: number;
  acrophase: TimeString;
  mesor: number;
}

/**
 * HRV-based circadian data
 */
export interface HRVCircadianData {
  /** Nighttime HRV metrics */
  nighttimeMetrics?: HRVMetrics;
  /** Circadian modulation parameters */
  circadianModulation?: HRVCircadianModulation;
}

/**
 * Rest-activity rhythm parameters
 */
export interface ActivityRhythm {
  /** 10 most active hours */
  m10?: {
    onset: TimeString;
    activityLevel: number;
  };
  /** 5 least active hours */
  l5?: {
    onset: TimeString;
    activityLevel: number;
  };
  /** Day-to-day consistency (0-1) */
  interdailyStability: number;
  /** Rhythm fragmentation */
  intradailyVariability: number;
  /** Rhythm robustness (0-1) */
  relativeAmplitude: number;
}

/**
 * Circadian phase estimate
 */
export interface CircadianPhase {
  /** Phase angle relative to external time */
  phaseAngle_hours: number;
  /** Current circadian phase (0-24 hours) */
  estimatedPhase_hours: number;
  /** Entrainment status */
  entrainmentStatus: EntrainmentStatus;
  /** Intrinsic period (tau) */
  tau_hours?: number;
  /** Phase angle between DLMO and habitual sleep onset */
  phaseAngleToSleep?: number;
}

/**
 * Alertness level categories
 */
export type AlertnessLevel = 'peak' | 'good' | 'moderate' | 'low';

/**
 * Alertness window
 */
export interface AlertnessWindow {
  startTime: TimeString;
  endTime: TimeString;
  level: AlertnessLevel;
}

/**
 * Circadian phase estimate with alertness prediction
 */
export interface CircadianPhaseEstimate {
  /** Subject identifier */
  subjectId: string;
  /** Timestamp of estimate */
  timestamp: string;
  /** Current circadian phase (0-24) */
  currentPhase: number;
  /** Phase angle */
  phaseAngle: number;
  /** Entrainment status */
  entrainmentStatus: EntrainmentStatus;
  /** Recommended phase shift */
  recommendedShift: number;
  /** Optimal alertness windows */
  optimalAlertness?: AlertnessWindow[];
}

/**
 * Complete circadian markers record
 */
export interface CircadianMarkersRecord {
  /** Record identifier */
  recordId: string;
  /** Subject identifier */
  subjectId: string;
  /** Measurement date */
  measurementDate: string;
  /** Melatonin data */
  melatonin?: MelatoninRhythm;
  /** Cortisol data */
  cortisol?: CortisolRhythm;
  /** Temperature data */
  coreBodyTemperature?: TemperatureRhythm;
  /** HRV data */
  heartRateVariability?: HRVCircadianData;
  /** Activity rhythm */
  activityRhythm?: ActivityRhythm;
  /** Computed phase estimate */
  circadianPhase?: CircadianPhase;
}

/**
 * Input for submitting circadian markers
 */
export interface CircadianMarkersInput {
  subjectId: string;
  measurementTimestamp: string;
  melatonin?: {
    level_pg_ml: number;
    sampleType: MelatoninSampleType;
  };
  cortisol?: {
    level_nmol_L: number;
    minutesAfterWaking: number;
  };
  coreBodyTemp_C?: number;
  hrv?: HRVMetrics;
}
