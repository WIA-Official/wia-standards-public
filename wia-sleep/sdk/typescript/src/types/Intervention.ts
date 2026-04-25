/**
 * WIA-SLEEP SDK - Intervention Types
 *
 * Type definitions for sleep interventions, optimization plans, and treatments.
 */

import { TimeString, DurationMinutes, Percentage } from './SleepRecord';
import { ChronotypeClass } from './Chronotype';
import { EntrainmentStatus } from './CircadianMarker';

/**
 * Intervention types
 */
export enum InterventionType {
  LIGHT_THERAPY = 'light_therapy',
  CBTI = 'cbti',
  CHRONOTHERAPY = 'chronotherapy',
  PHARMACOLOGICAL = 'pharmacological',
  NUTRACEUTICAL = 'nutraceutical',
  DEVICE = 'device'
}

/**
 * Sleep optimization goals
 */
export enum OptimizationGoal {
  REDUCE_LATENCY = 'reduce_latency',
  INCREASE_EFFICIENCY = 'increase_efficiency',
  PHASE_ADVANCE = 'phase_advance',
  PHASE_DELAY = 'phase_delay',
  IMPROVE_SWS = 'improve_sws',
  IMPROVE_REM = 'improve_rem',
  REDUCE_WAKEUPS = 'reduce_wakeups',
  REDUCE_SOCIAL_JETLAG = 'reduce_social_jetlag'
}

/**
 * Light therapy reference events
 */
export enum LightTimingReference {
  WAKE = 'wake',
  DLMO = 'dlmo',
  CBT_NADIR = 'cbt_nadir'
}

/**
 * Light therapy session
 */
export interface LightTherapySession {
  /** Session timing */
  timing: TimeString;
  /** Duration in minutes */
  duration_min: DurationMinutes;
  /** Light intensity in lux */
  intensity_lux: number;
  /** Melanopic EDI */
  melanopicEDI?: number;
  /** Color temperature in Kelvin */
  colorTemp_K?: number;
  /** Timing relative to event */
  relativeToEvent?: LightTimingReference;
  /** Offset from reference event in minutes */
  offsetMinutes?: number;
}

/**
 * Light avoidance window
 */
export interface LightAvoidanceWindow {
  start: TimeString;
  end: TimeString;
}

/**
 * Light therapy prescription
 */
export interface LightPrescription {
  /** Prescription identifier */
  prescriptionId?: string;
  /** Subject identifier */
  subjectId?: string;
  /** Therapy sessions */
  sessions: LightTherapySession[];
  /** Light avoidance window */
  avoidanceWindow?: LightAvoidanceWindow;
  /** Expected phase shift in hours */
  expectedPhaseShift_hours: number;
  /** Treatment duration in days */
  durationOfTreatment_days: number;
}

/**
 * Light prescription request
 */
export interface LightPrescriptionRequest {
  subjectId: string;
  goal: 'phase_advance' | 'phase_delay' | 'amplitude_increase' | 'maintain';
  targetShift_hours?: number;
  constraints?: {
    earliestExposure?: TimeString;
    latestExposure?: TimeString;
    maxDuration_min?: number;
  };
}

/**
 * Temperature schedule
 */
export interface TemperatureSchedule {
  /** Evening cooldown start time */
  eveningCooldown: TimeString;
  /** Target bedroom temperature */
  bedroomTarget_C: number;
  /** Warm bath timing */
  warmBathTime?: TimeString;
  /** Bath duration */
  bathDuration_min?: number;
}

/**
 * Meal timing recommendations
 */
export interface MealTimingPlan {
  /** Breakfast window */
  breakfastWindow?: {
    start: TimeString;
    end: TimeString;
  };
  /** Dinner deadline */
  dinnerDeadline?: TimeString;
  /** Hours between last meal and sleep */
  lastMealToSleep_hours?: number;
  /** Caffeine cutoff time */
  caffeineDeadline: TimeString;
  /** Alcohol avoidance hours before bed */
  alcoholAvoidanceHours?: number;
}

/**
 * Exercise timing recommendations
 */
export interface ExerciseTimingPlan {
  /** Optimal exercise window */
  optimalWindow: {
    start: TimeString;
    end: TimeString;
  };
  /** Time to avoid exercise after */
  avoidAfter: TimeString;
  /** Recommended exercise type */
  recommendedType?: string;
  /** Duration in minutes */
  duration_min?: number;
}

/**
 * Sleep hygiene intervention
 */
export interface SleepHygieneIntervention {
  /** Intervention name */
  intervention: string;
  /** Description */
  description: string;
  /** Priority level */
  priority: 'critical' | 'high' | 'medium' | 'low';
}

/**
 * Expected optimization outcomes
 */
export interface OptimizationOutcomes {
  /** Sleep efficiency improvement */
  sleepEfficiencyImprovement_pct?: number;
  /** Latency reduction in minutes */
  latencyReduction_min?: number;
  /** Phase shift in hours */
  phaseShift_hours?: number;
  /** Social jetlag reduction */
  socialJetlagReduction_hours?: number;
  /** Days to see effect */
  timeToEffect_days?: number;
  /** Sustainability score (0-1) */
  sustainabilityScore?: number;
}

/**
 * Treatment phase
 */
export interface TreatmentPhase {
  /** Phase name */
  name: string;
  /** Duration in days */
  duration_days: number;
  /** Focus area */
  focus: string;
  /** Expected outcome */
  expectedOutcome: string;
}

/**
 * Monitoring requirements
 */
export interface MonitoringPlan {
  /** Daily tracking items */
  dailyTracking: string[];
  /** Weekly assessment items */
  weeklyAssessment: string[];
  /** Follow-up date */
  followUpDate?: string;
}

/**
 * Warning for optimization plan
 */
export interface PlanWarning {
  /** Condition that triggers warning */
  condition: string;
  /** Recommended action */
  action: string;
  /** Severity level */
  severity: 'high' | 'medium' | 'low';
}

/**
 * Optimization request
 */
export interface OptimizationRequest {
  subjectId: string;
  goals: OptimizationGoal[];
  constraints?: {
    workSchedule?: {
      wakeTimeRequired?: TimeString;
      flexibility?: 'fixed' | 'flexible' | 'highly_flexible';
    };
    preferredBedtime?: TimeString;
  };
}

/**
 * Current sleep state
 */
export interface CurrentSleepState {
  chronotype: ChronotypeClass;
  averageBedtime: TimeString;
  averageWakeTime: TimeString;
  sleepEfficiency_pct: Percentage;
  socialJetlag_hours: number;
  dlmo?: TimeString;
}

/**
 * Target sleep state
 */
export interface TargetSleepState {
  targetBedtime: TimeString;
  targetWakeTime: TimeString;
  targetSleepEfficiency_pct: Percentage;
  targetSocialJetlag_hours: number;
  targetDLMO?: TimeString;
}

/**
 * Complete sleep optimization plan
 */
export interface SleepOptimizationPlan {
  /** Plan identifier */
  planId: string;
  /** Subject identifier */
  subjectId: string;
  /** Generation timestamp */
  generatedAt: string;
  /** Optimization goals */
  goals: OptimizationGoal[];
  /** Current state */
  currentState?: CurrentSleepState;
  /** Target state */
  targetState: TargetSleepState;
  /** Interventions */
  interventions: {
    lightTherapy?: LightPrescription;
    melatoninTiming?: TimeString;
    melatoninDose_mg?: number;
    temperatureSchedule?: TemperatureSchedule;
    mealTiming?: MealTimingPlan;
    exerciseTiming?: ExerciseTimingPlan;
    sleepHygiene?: SleepHygieneIntervention[];
  };
  /** Treatment timeline */
  timeline?: TreatmentPhase[];
  /** Expected outcomes */
  expectedOutcomes: OptimizationOutcomes;
  /** Monitoring plan */
  monitoring?: MonitoringPlan;
  /** Contraindications */
  contraindications?: string[];
  /** Warnings */
  warnings?: PlanWarning[];
}

/**
 * Single intervention record
 */
export interface Intervention {
  /** Intervention type */
  type: InterventionType;
  /** Intervention name */
  name: string;
  /** Timing */
  timing?: string;
  /** Dosage or intensity */
  dosage?: string;
  /** Effectiveness score (0-1) */
  effectiveness_score?: number;
}

/**
 * Sleep need prediction request
 */
export interface SleepNeedRequest {
  subjectId: string;
  age?: number;
  geneticFactors?: {
    per3Vntr?: '4/4' | '4/5' | '5/5';
    adenosineSensitivity?: 'high' | 'medium' | 'low';
  };
  activityLevel?: 'sedentary' | 'light' | 'moderate' | 'high' | 'athlete';
  cognitiveLoad?: 'low' | 'moderate' | 'high';
}

/**
 * Sleep need prediction result
 */
export interface SleepNeedPrediction {
  subjectId: string;
  predictedNeed_hours: number;
  range: {
    minimum_hours: number;
    maximum_hours: number;
  };
  confidence: number;
  factors: Array<{
    factor: string;
    impact: 'increases' | 'decreases' | 'neutral';
    magnitude: 'small' | 'moderate' | 'large';
  }>;
}
