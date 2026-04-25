/**
 * WIA-SLEEP SDK - Sleep Stage Scorer
 *
 * Utilities for sleep stage analysis and scoring validation.
 */

import { SleepStage, SleepStagePercentages, Hypnogram } from '../types/SleepRecord';

/**
 * Sleep stage scoring rules (AASM-based)
 */
export interface SleepScoringRules {
  /** Minimum epochs for stage to count */
  minEpochsForStage: number;
  /** Wake criteria */
  wake: {
    alphaActivityThreshold: number;
    eyeMovementAllowed: boolean;
  };
  /** N1 criteria */
  n1: {
    alphaReductionPercent: number;
    maxAmplitude_uV: number;
  };
  /** N2 criteria */
  n2: {
    requireKComplex: boolean;
    requireSpindle: boolean;
    minDuration_sec: number;
  };
  /** N3/SWS criteria */
  n3: {
    slowWavePercent: number;
    minAmplitude_uV: number;
  };
  /** REM criteria */
  rem: {
    requireLowEMG: boolean;
    requireREMs: boolean;
  };
}

/**
 * Default AASM 2020 scoring rules
 */
export const AASM_2020_RULES: SleepScoringRules = {
  minEpochsForStage: 1,
  wake: {
    alphaActivityThreshold: 50,
    eyeMovementAllowed: true
  },
  n1: {
    alphaReductionPercent: 50,
    maxAmplitude_uV: 75
  },
  n2: {
    requireKComplex: false,
    requireSpindle: false,
    minDuration_sec: 0.5
  },
  n3: {
    slowWavePercent: 20,
    minAmplitude_uV: 75
  },
  rem: {
    requireLowEMG: true,
    requireREMs: false
  }
};

/**
 * Stage transition metrics
 */
export interface StageTransitions {
  /** Total number of transitions */
  totalTransitions: number;
  /** Transitions per hour of sleep */
  transitionsPerHour: number;
  /** Transition matrix (from stage -> to stage -> count) */
  transitionMatrix: Map<SleepStage, Map<SleepStage, number>>;
  /** Most common transitions */
  commonTransitions: Array<{
    from: SleepStage;
    to: SleepStage;
    count: number;
    percentage: number;
  }>;
}

/**
 * Calculate stage percentages from hypnogram
 */
export function calculateStagePercentages(
  stages: SleepStage[],
  excludeWake: boolean = true
): SleepStagePercentages {
  const counts = {
    wake: 0,
    n1: 0,
    n2: 0,
    n3: 0,
    rem: 0
  };

  for (const stage of stages) {
    switch (stage) {
      case SleepStage.WAKE:
        counts.wake++;
        break;
      case SleepStage.N1:
        counts.n1++;
        break;
      case SleepStage.N2:
        counts.n2++;
        break;
      case SleepStage.N3:
        counts.n3++;
        break;
      case SleepStage.REM:
        counts.rem++;
        break;
    }
  }

  const total = excludeWake
    ? stages.length - counts.wake
    : stages.length;

  if (total === 0) {
    return {
      wake_pct: 100,
      n1_pct: 0,
      n2_pct: 0,
      n3_sws_pct: 0,
      rem_pct: 0
    };
  }

  return {
    wake_pct: excludeWake ? undefined : (counts.wake / stages.length) * 100,
    n1_pct: (counts.n1 / total) * 100,
    n2_pct: (counts.n2 / total) * 100,
    n3_sws_pct: (counts.n3 / total) * 100,
    rem_pct: (counts.rem / total) * 100
  };
}

/**
 * Calculate stage transitions
 */
export function analyzeStageTransitions(
  stages: SleepStage[],
  epochDuration_sec: number = 30
): StageTransitions {
  const transitionMatrix = new Map<SleepStage, Map<SleepStage, number>>();
  let totalTransitions = 0;

  // Initialize matrix
  for (const stage of [SleepStage.WAKE, SleepStage.N1, SleepStage.N2, SleepStage.N3, SleepStage.REM]) {
    transitionMatrix.set(stage, new Map());
    for (const toStage of [SleepStage.WAKE, SleepStage.N1, SleepStage.N2, SleepStage.N3, SleepStage.REM]) {
      transitionMatrix.get(stage)!.set(toStage, 0);
    }
  }

  // Count transitions
  for (let i = 0; i < stages.length - 1; i++) {
    const from = stages[i];
    const to = stages[i + 1];

    if (from !== to) {
      totalTransitions++;
      const fromMap = transitionMatrix.get(from)!;
      fromMap.set(to, (fromMap.get(to) || 0) + 1);
    }
  }

  // Calculate total sleep time for rate calculation
  const sleepEpochs = stages.filter((s) => s !== SleepStage.WAKE).length;
  const sleepHours = (sleepEpochs * epochDuration_sec) / 3600;

  const transitionsPerHour = sleepHours > 0 ? totalTransitions / sleepHours : 0;

  // Find most common transitions
  const allTransitions: Array<{
    from: SleepStage;
    to: SleepStage;
    count: number;
    percentage: number;
  }> = [];

  transitionMatrix.forEach((toMap, from) => {
    toMap.forEach((count, to) => {
      if (count > 0 && from !== to) {
        allTransitions.push({
          from,
          to,
          count,
          percentage: totalTransitions > 0 ? (count / totalTransitions) * 100 : 0
        });
      }
    });
  });

  allTransitions.sort((a, b) => b.count - a.count);

  return {
    totalTransitions,
    transitionsPerHour,
    transitionMatrix,
    commonTransitions: allTransitions.slice(0, 10)
  };
}

/**
 * Calculate sleep stage continuity metrics
 */
export function calculateStageContinuity(stages: SleepStage[]): {
  meanBoutLength: Map<SleepStage, number>;
  maxBoutLength: Map<SleepStage, number>;
  boutCounts: Map<SleepStage, number>;
} {
  const bouts = new Map<SleepStage, number[]>();

  // Initialize
  for (const stage of [SleepStage.WAKE, SleepStage.N1, SleepStage.N2, SleepStage.N3, SleepStage.REM]) {
    bouts.set(stage, []);
  }

  // Calculate bout lengths
  let currentStage = stages[0];
  let boutLength = 1;

  for (let i = 1; i < stages.length; i++) {
    if (stages[i] === currentStage) {
      boutLength++;
    } else {
      bouts.get(currentStage)!.push(boutLength);
      currentStage = stages[i];
      boutLength = 1;
    }
  }
  // Add final bout
  bouts.get(currentStage)!.push(boutLength);

  // Calculate metrics
  const meanBoutLength = new Map<SleepStage, number>();
  const maxBoutLength = new Map<SleepStage, number>();
  const boutCounts = new Map<SleepStage, number>();

  bouts.forEach((lengths, stage) => {
    if (lengths.length > 0) {
      meanBoutLength.set(
        stage,
        lengths.reduce((a, b) => a + b, 0) / lengths.length
      );
      maxBoutLength.set(stage, Math.max(...lengths));
      boutCounts.set(stage, lengths.length);
    } else {
      meanBoutLength.set(stage, 0);
      maxBoutLength.set(stage, 0);
      boutCounts.set(stage, 0);
    }
  });

  return { meanBoutLength, maxBoutLength, boutCounts };
}

/**
 * Validate hypnogram against AASM rules
 */
export function validateHypnogram(
  hypnogram: Hypnogram,
  rules: SleepScoringRules = AASM_2020_RULES
): {
  valid: boolean;
  warnings: string[];
  errors: string[];
} {
  const warnings: string[] = [];
  const errors: string[] = [];

  const { stages, epochDuration_sec } = hypnogram;

  // Check for valid epoch duration
  if (epochDuration_sec !== 30 && epochDuration_sec !== 20) {
    errors.push(`Invalid epoch duration: ${epochDuration_sec}s. Must be 30s or 20s.`);
  }

  // Check for valid stage codes
  const validStages = new Set([0, 1, 2, 3, 5]);
  for (let i = 0; i < stages.length; i++) {
    if (!validStages.has(stages[i])) {
      errors.push(`Invalid stage code ${stages[i]} at epoch ${i + 1}`);
    }
  }

  // Check stage percentages are reasonable
  const percentages = calculateStagePercentages(stages);

  if (percentages.n1_pct > 20) {
    warnings.push(`Elevated N1 percentage (${percentages.n1_pct.toFixed(1)}%) may indicate fragmented sleep`);
  }

  if (percentages.n3_sws_pct < 10) {
    warnings.push(`Low SWS percentage (${percentages.n3_sws_pct.toFixed(1)}%) - verify scoring accuracy`);
  }

  if (percentages.rem_pct < 15) {
    warnings.push(`Low REM percentage (${percentages.rem_pct.toFixed(1)}%) - may indicate REM suppression`);
  }

  if (percentages.rem_pct > 30) {
    warnings.push(`Elevated REM percentage (${percentages.rem_pct.toFixed(1)}%) - verify scoring accuracy`);
  }

  // Check for biological plausibility
  const transitions = analyzeStageTransitions(stages, epochDuration_sec);

  // N3 should not transition directly to REM frequently
  const n3ToRem = transitions.transitionMatrix.get(SleepStage.N3)?.get(SleepStage.REM) || 0;
  if (n3ToRem > 5) {
    warnings.push(`Unusual number of N3→REM transitions (${n3ToRem})`);
  }

  // First REM typically shouldn't occur too early
  const firstRemIndex = stages.findIndex((s) => s === SleepStage.REM);
  if (firstRemIndex > 0) {
    const remLatency_min = (firstRemIndex * epochDuration_sec) / 60;
    if (remLatency_min < 15) {
      warnings.push(`Very short REM latency (${remLatency_min.toFixed(0)} min) - may indicate narcolepsy or REM rebound`);
    }
  }

  return {
    valid: errors.length === 0,
    warnings,
    errors
  };
}

/**
 * Get stage name from code
 */
export function getStageName(stage: SleepStage): string {
  switch (stage) {
    case SleepStage.WAKE:
      return 'Wake';
    case SleepStage.N1:
      return 'N1 (Light Sleep)';
    case SleepStage.N2:
      return 'N2';
    case SleepStage.N3:
      return 'N3 (Deep Sleep/SWS)';
    case SleepStage.REM:
      return 'REM';
    default:
      return 'Unknown';
  }
}

/**
 * Check if stage percentages are within normal range
 */
export function assessStageBalance(percentages: SleepStagePercentages): {
  overall: 'normal' | 'abnormal';
  details: Array<{
    stage: string;
    status: 'low' | 'normal' | 'high';
    value: number;
    normalRange: [number, number];
  }>;
} {
  const normalRanges: Record<string, [number, number]> = {
    n1: [2, 10],
    n2: [45, 55],
    n3: [15, 25],
    rem: [20, 25]
  };

  const details: Array<{
    stage: string;
    status: 'low' | 'normal' | 'high';
    value: number;
    normalRange: [number, number];
  }> = [];

  let abnormalCount = 0;

  const checkStage = (
    name: string,
    value: number,
    range: [number, number]
  ) => {
    let status: 'low' | 'normal' | 'high';
    if (value < range[0]) {
      status = 'low';
      abnormalCount++;
    } else if (value > range[1]) {
      status = 'high';
      abnormalCount++;
    } else {
      status = 'normal';
    }

    details.push({
      stage: name,
      status,
      value,
      normalRange: range
    });
  };

  checkStage('N1', percentages.n1_pct, normalRanges.n1);
  checkStage('N2', percentages.n2_pct, normalRanges.n2);
  checkStage('N3/SWS', percentages.n3_sws_pct, normalRanges.n3);
  checkStage('REM', percentages.rem_pct, normalRanges.rem);

  return {
    overall: abnormalCount > 1 ? 'abnormal' : 'normal',
    details
  };
}

export default {
  calculateStagePercentages,
  analyzeStageTransitions,
  calculateStageContinuity,
  validateHypnogram,
  getStageName,
  assessStageBalance,
  AASM_2020_RULES
};
