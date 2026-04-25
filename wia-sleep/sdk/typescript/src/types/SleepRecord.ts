/**
 * WIA-SLEEP SDK - Sleep Record Types
 *
 * Core type definitions for sleep records, architecture, and related data.
 */

/**
 * Time string in HH:MM:SS format
 */
export type TimeString = string;

/**
 * Duration in minutes
 */
export type DurationMinutes = number;

/**
 * Percentage value (0-100)
 */
export type Percentage = number;

/**
 * Sleep data source types
 */
export enum SleepDataSource {
  PSG = 'psg',
  HSAT = 'hsat',
  CONSUMER_WEARABLE = 'consumer_wearable',
  CLINICAL_WEARABLE = 'clinical_wearable',
  ACTIGRAPHY = 'actigraphy'
}

/**
 * Sleep scoring methodology
 */
export enum ScoringMethod {
  AASM_2020 = 'AASM_2020',
  AASM_2018 = 'AASM_2018',
  AASM_2012 = 'AASM_2012',
  R_AND_K = 'R_and_K',
  AUTOMATED = 'automated'
}

/**
 * Sleep stage codes (AASM standard)
 */
export enum SleepStage {
  WAKE = 0,
  N1 = 1,
  N2 = 2,
  N3 = 3,
  REM = 5
}

/**
 * Sleep stage percentages
 */
export interface SleepStagePercentages {
  /** Wake percentage during sleep period */
  wake_pct?: Percentage;
  /** N1 (light sleep) percentage */
  n1_pct: Percentage;
  /** N2 (intermediate sleep) percentage */
  n2_pct: Percentage;
  /** N3/SWS (slow-wave/deep sleep) percentage */
  n3_sws_pct: Percentage;
  /** REM sleep percentage */
  rem_pct: Percentage;
}

/**
 * Sleep stage durations in minutes
 */
export interface SleepStageDurations {
  wake_min: DurationMinutes;
  n1_min: DurationMinutes;
  n2_min: DurationMinutes;
  n3_min: DurationMinutes;
  rem_min: DurationMinutes;
}

/**
 * Individual sleep cycle data
 */
export interface SleepCycle {
  /** Cycle number (1-based) */
  cycleNumber: number;
  /** Cycle start time */
  startTime: TimeString;
  /** Cycle end time */
  endTime: TimeString;
  /** Total cycle duration */
  duration_min: DurationMinutes;
  /** NREM duration in this cycle */
  nremDuration_min: DurationMinutes;
  /** REM duration in this cycle */
  remDuration_min: DurationMinutes;
  /** Deep sleep (N3) duration */
  n3Duration_min: DurationMinutes;
  /** Whether this is a complete cycle */
  complete: boolean;
}

/**
 * Hypnogram data (epoch-by-epoch staging)
 */
export interface Hypnogram {
  /** Duration of each epoch in seconds (30 or 20) */
  epochDuration_sec: 30 | 20;
  /** Recording start time */
  startTime: string;
  /** Array of sleep stages for each epoch */
  stages: SleepStage[];
}

/**
 * Sleep microstructure metrics
 */
export interface SleepMicrostructure {
  /** Sleep spindle metrics */
  spindles?: {
    density_per_min: number;
    meanAmplitude_uV: number;
    meanFrequency_Hz: number;
    meanDuration_sec: number;
  };
  /** K-complex metrics */
  kComplexes?: {
    density_per_min: number;
    meanAmplitude_uV: number;
  };
  /** Slow wave metrics */
  slowWaves?: {
    density_per_min: number;
    meanAmplitude_uV: number;
    slowWaveActivity_uV2: number;
  };
  /** REM density percentage */
  remDensity_pct?: Percentage;
}

/**
 * Sleep quality assessment metrics
 */
export interface SleepQualityMetrics {
  /** Composite sleep quality score (0-100) */
  sleepQualityIndex: number;
  /** Physical restoration score based on SWS */
  restorationScore: number;
  /** Cognitive recovery score based on REM and spindles */
  cognitiveRecoveryScore: number;
  /** Sleep fragmentation index */
  sleepFragmentation: number;
}

/**
 * Sleep architecture summary
 */
export interface SleepArchitectureSummary {
  /** Total time in bed */
  timeInBed_min?: DurationMinutes;
  /** Sleep period time */
  sleepPeriod_min?: DurationMinutes;
  /** Total sleep time */
  totalSleepTime_min: DurationMinutes;
  /** Sleep efficiency (TST/TIB * 100) */
  sleepEfficiency_pct: Percentage;
  /** Time to fall asleep */
  sleepOnsetLatency_min: DurationMinutes;
  /** Latency to first REM */
  remLatency_min?: DurationMinutes;
  /** Wake after sleep onset */
  wakeAfterSleepOnset_min: DurationMinutes;
  /** Number of awakenings */
  awakenings?: number;
  /** Arousal index */
  arousals_per_hour?: number;
}

/**
 * Complete sleep architecture data
 */
export interface SleepArchitecture {
  /** Architecture summary */
  summary: SleepArchitectureSummary;
  /** Stage percentages */
  stagePercentages?: SleepStagePercentages;
  /** Stage durations */
  stageDurations?: SleepStageDurations;
  /** Individual sleep cycles */
  sleepCycles?: SleepCycle[];
  /** Number of complete cycles */
  cycleCount?: number;
  /** Hypnogram data */
  hypnogram?: Hypnogram;
  /** Microstructure metrics */
  microstructure?: SleepMicrostructure;
  /** Quality metrics */
  qualityMetrics?: SleepQualityMetrics;
}

/**
 * Environmental factors affecting sleep
 */
export interface EnvironmentalFactors {
  lightExposure?: {
    /** Morning light exposure (lux-hours) */
    morning_lux_hours: number;
    /** Evening blue light exposure (minutes) */
    evening_blue_light_min: number;
    /** Melanopic EDI */
    melanopic_edi: number;
  };
  /** Room temperature (Celsius) */
  temperature_C?: number;
  /** Room humidity (%) */
  humidity_pct?: Percentage;
  /** Ambient noise (dB) */
  noise_db?: number;
  /** Altitude (meters) */
  altitude_m?: number;
}

/**
 * Sleep disorder screening metrics
 */
export interface SleepDisorderMetrics {
  /** Apnea-Hypopnea Index */
  ahi_events_per_hour?: number;
  /** Oxygen Desaturation Index */
  oxygenDesaturationIndex?: number;
  /** Periodic Limb Movement Index */
  plm_index?: number;
  /** Insomnia Severity Index (0-28) */
  insomniaSeverityIndex?: number;
  /** Epworth Sleepiness Scale (0-24) */
  epworthSleepinessScale?: number;
}

/**
 * Complete WIA-SLEEP record
 */
export interface SleepRecord {
  /** Unique record identifier */
  recordId: string;
  /** Subject identifier */
  subjectId: string;
  /** Record timestamp */
  timestamp: string;
  /** Data source */
  dataSource?: SleepDataSource;
  /** Scoring method used */
  scoringMethod?: ScoringMethod;
  /** Sleep architecture data */
  sleepArchitecture: SleepArchitecture;
  /** Environmental factors */
  environmentalFactors?: EnvironmentalFactors;
  /** Disorder screening metrics */
  sleepDisorders?: SleepDisorderMetrics;
}

/**
 * Sleep record input for API submission
 */
export interface SleepRecordInput {
  subjectId: string;
  sleepDate?: string;
  dataSource?: SleepDataSource;
  sleepArchitecture: Partial<SleepArchitecture> & {
    summary: Partial<SleepArchitectureSummary> & {
      totalSleepTime_min: DurationMinutes;
      sleepEfficiency_pct: Percentage;
    };
  };
  environmentalFactors?: EnvironmentalFactors;
}
