/**
 * WIA-SLEEP SDK - Sleep Optimization Engine
 *
 * Service for generating personalized sleep optimization plans based on
 * chronotype, circadian markers, and specified goals.
 */

import {
  OptimizationGoal,
  OptimizationRequest,
  SleepOptimizationPlan,
  LightPrescription,
  LightTherapySession,
  LightPrescriptionRequest,
  TemperatureSchedule,
  MealTimingPlan,
  ExerciseTimingPlan,
  SleepHygieneIntervention,
  OptimizationOutcomes,
  TreatmentPhase,
  PlanWarning,
  CurrentSleepState,
  TargetSleepState
} from '../types/Intervention';
import { ChronotypeProfile, ChronotypeClass } from '../types/Chronotype';
import { TimeString } from '../types/SleepRecord';
import { CircadianCalculator } from './CircadianCalculator';

/**
 * Convert time string to decimal hours
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
 * Sleep optimization engine
 */
export class OptimizationEngine {
  /**
   * Generate complete sleep optimization plan
   */
  static generatePlan(
    request: OptimizationRequest,
    chronotype: ChronotypeProfile,
    currentState?: CurrentSleepState
  ): SleepOptimizationPlan {
    const { subjectId, goals, constraints } = request;

    // Determine target state based on goals
    const targetState = this.calculateTargetState(chronotype, goals, constraints);

    // Generate interventions based on goals
    const lightTherapy = goals.includes(OptimizationGoal.PHASE_ADVANCE) ||
      goals.includes(OptimizationGoal.PHASE_DELAY) ||
      goals.includes(OptimizationGoal.REDUCE_SOCIAL_JETLAG)
      ? this.generateLightPrescription({
          subjectId,
          goal: goals.includes(OptimizationGoal.PHASE_ADVANCE)
            ? 'phase_advance'
            : goals.includes(OptimizationGoal.PHASE_DELAY)
            ? 'phase_delay'
            : 'phase_advance', // Default for social jetlag
          targetShift_hours: chronotype.socialJetlag > 0 ? -chronotype.socialJetlag * 0.5 : 0
        }, chronotype)
      : undefined;

    const melatoninTiming = this.calculateMelatoninTiming(chronotype, goals);
    const temperatureSchedule = this.generateTemperatureSchedule(targetState);
    const mealTiming = this.generateMealTiming(targetState, chronotype);
    const exerciseTiming = this.generateExerciseTiming(targetState, chronotype);
    const sleepHygiene = this.generateSleepHygieneRecommendations(goals);

    // Calculate expected outcomes
    const expectedOutcomes = this.calculateExpectedOutcomes(
      goals,
      chronotype,
      currentState
    );

    // Generate timeline
    const timeline = this.generateTimeline(goals);

    // Generate warnings
    const warnings = this.generateWarnings(goals);

    return {
      planId: `plan-${Date.now()}`,
      subjectId,
      generatedAt: new Date().toISOString(),
      goals,
      currentState,
      targetState,
      interventions: {
        lightTherapy,
        melatoninTiming,
        melatoninDose_mg: melatoninTiming ? 0.5 : undefined,
        temperatureSchedule,
        mealTiming,
        exerciseTiming,
        sleepHygiene
      },
      timeline,
      expectedOutcomes,
      monitoring: {
        dailyTracking: [
          'bedtime',
          'wake_time',
          'sleep_quality_rating',
          'morning_alertness',
          lightTherapy ? 'light_therapy_compliance' : undefined
        ].filter(Boolean) as string[],
        weeklyAssessment: [
          'sleep_efficiency',
          'social_jetlag',
          'mood_score',
          'energy_levels'
        ],
        followUpDate: new Date(Date.now() + 14 * 24 * 60 * 60 * 1000).toISOString()
      },
      contraindications: [],
      warnings
    };
  }

  /**
   * Calculate target sleep state
   */
  private static calculateTargetState(
    chronotype: ChronotypeProfile,
    goals: OptimizationGoal[],
    constraints?: OptimizationRequest['constraints']
  ): TargetSleepState {
    let targetBedtime = chronotype.optimalSleepWindow.bedtime;
    let targetWakeTime = chronotype.optimalSleepWindow.wakeTime;

    // Adjust for work schedule constraints
    if (constraints?.workSchedule?.wakeTimeRequired) {
      targetWakeTime = constraints.workSchedule.wakeTimeRequired;
      // Calculate bedtime for 8 hours of sleep
      const wakeHours = timeToHours(targetWakeTime);
      targetBedtime = hoursToTime(wakeHours - 8);
    }

    // Adjust for phase goals
    if (goals.includes(OptimizationGoal.PHASE_ADVANCE)) {
      const currentBedtime = timeToHours(targetBedtime);
      targetBedtime = hoursToTime(currentBedtime - 1);
      const currentWake = timeToHours(targetWakeTime);
      targetWakeTime = hoursToTime(currentWake - 1);
    } else if (goals.includes(OptimizationGoal.PHASE_DELAY)) {
      const currentBedtime = timeToHours(targetBedtime);
      targetBedtime = hoursToTime(currentBedtime + 1);
      const currentWake = timeToHours(targetWakeTime);
      targetWakeTime = hoursToTime(currentWake + 1);
    }

    // Target sleep efficiency
    const targetEfficiency = goals.includes(OptimizationGoal.INCREASE_EFFICIENCY) ? 92 : 88;

    // Target social jetlag
    const targetSocialJetlag = goals.includes(OptimizationGoal.REDUCE_SOCIAL_JETLAG)
      ? Math.min(0.5, chronotype.socialJetlag / 2)
      : chronotype.socialJetlag;

    return {
      targetBedtime,
      targetWakeTime,
      targetSleepEfficiency_pct: targetEfficiency,
      targetSocialJetlag_hours: targetSocialJetlag
    };
  }

  /**
   * Generate light therapy prescription
   */
  static generateLightPrescription(
    request: LightPrescriptionRequest,
    chronotype: ChronotypeProfile
  ): LightPrescription {
    const { goal, targetShift_hours, constraints } = request;

    const sessions: LightTherapySession[] = [];
    let expectedShift = 0;

    // Estimate current DLMO
    const dlmo = chronotype.dlmo ||
      CircadianCalculator.estimateDLMOFromChronotype(chronotype);
    const dlmoHours = timeToHours(dlmo);

    // Estimate CBT nadir (DLMO + 7 hours)
    const cbtNadirHours = dlmoHours + 7;

    if (goal === 'phase_advance') {
      // Morning light exposure after CBT nadir
      // Optimal window: 0-3 hours after CBT nadir
      let lightTime = cbtNadirHours + 1.5; // 1.5 hours after nadir
      if (lightTime > 24) lightTime -= 24;

      // Adjust for constraints
      if (constraints?.earliestExposure) {
        const earliest = timeToHours(constraints.earliestExposure);
        lightTime = Math.max(lightTime, earliest);
      }

      sessions.push({
        timing: hoursToTime(lightTime),
        duration_min: constraints?.maxDuration_min || 30,
        intensity_lux: 10000,
        melanopicEDI: 250,
        colorTemp_K: 6500,
        relativeToEvent: 'wake',
        offsetMinutes: 0
      });

      expectedShift = -Math.min(1.5, Math.abs(targetShift_hours || 1));

      // Light avoidance in evening
    } else if (goal === 'phase_delay') {
      // Evening light exposure before DLMO
      let lightTime = dlmoHours - 2;
      if (lightTime < 0) lightTime += 24;

      if (constraints?.latestExposure) {
        const latest = timeToHours(constraints.latestExposure);
        lightTime = Math.min(lightTime, latest);
      }

      sessions.push({
        timing: hoursToTime(lightTime),
        duration_min: constraints?.maxDuration_min || 30,
        intensity_lux: 10000,
        melanopicEDI: 250,
        colorTemp_K: 6500,
        relativeToEvent: 'dlmo',
        offsetMinutes: -120
      });

      expectedShift = Math.min(1.5, targetShift_hours || 1);
    } else if (goal === 'amplitude_increase') {
      // Both morning and evening exposure
      sessions.push(
        {
          timing: hoursToTime(cbtNadirHours + 1),
          duration_min: 30,
          intensity_lux: 10000,
          melanopicEDI: 250,
          colorTemp_K: 6500,
          relativeToEvent: 'wake',
          offsetMinutes: 0
        }
      );
    }

    // Calculate avoidance window for phase advance
    let avoidanceWindow: { start: TimeString; end: TimeString } | undefined;
    if (goal === 'phase_advance') {
      avoidanceWindow = {
        start: hoursToTime(dlmoHours - 3),
        end: hoursToTime(cbtNadirHours)
      };
    }

    return {
      prescriptionId: `rx-${Date.now()}`,
      subjectId: request.subjectId,
      sessions,
      avoidanceWindow,
      expectedPhaseShift_hours: expectedShift,
      durationOfTreatment_days: 14
    };
  }

  /**
   * Calculate optimal melatonin timing
   */
  private static calculateMelatoninTiming(
    chronotype: ChronotypeProfile,
    goals: OptimizationGoal[]
  ): TimeString | undefined {
    if (!goals.includes(OptimizationGoal.PHASE_ADVANCE) &&
        !goals.includes(OptimizationGoal.REDUCE_LATENCY)) {
      return undefined;
    }

    const dlmo = chronotype.dlmo ||
      CircadianCalculator.estimateDLMOFromChronotype(chronotype);
    const dlmoHours = timeToHours(dlmo);

    // For phase advance, give melatonin 4-6 hours before DLMO
    // For reducing latency, give 30-60 min before target bedtime
    let melatoninTime: number;

    if (goals.includes(OptimizationGoal.PHASE_ADVANCE)) {
      melatoninTime = dlmoHours - 5;
    } else {
      melatoninTime = timeToHours(chronotype.optimalSleepWindow.bedtime) - 0.5;
    }

    return hoursToTime(melatoninTime);
  }

  /**
   * Generate temperature schedule
   */
  private static generateTemperatureSchedule(
    targetState: TargetSleepState
  ): TemperatureSchedule {
    const bedtimeHours = timeToHours(targetState.targetBedtime);

    return {
      eveningCooldown: hoursToTime(bedtimeHours - 2),
      bedroomTarget_C: 18.5,
      warmBathTime: hoursToTime(bedtimeHours - 1.5),
      bathDuration_min: 20
    };
  }

  /**
   * Generate meal timing recommendations
   */
  private static generateMealTiming(
    targetState: TargetSleepState,
    chronotype: ChronotypeProfile
  ): MealTimingPlan {
    const wakeHours = timeToHours(targetState.targetWakeTime);
    const bedtimeHours = timeToHours(targetState.targetBedtime);

    return {
      breakfastWindow: {
        start: hoursToTime(wakeHours),
        end: hoursToTime(wakeHours + 1)
      },
      dinnerDeadline: hoursToTime(bedtimeHours - 3),
      lastMealToSleep_hours: 3,
      caffeineDeadline: hoursToTime(bedtimeHours - 8),
      alcoholAvoidanceHours: 4
    };
  }

  /**
   * Generate exercise timing recommendations
   */
  private static generateExerciseTiming(
    targetState: TargetSleepState,
    chronotype: ChronotypeProfile
  ): ExerciseTimingPlan {
    const wakeHours = timeToHours(targetState.targetWakeTime);
    const bedtimeHours = timeToHours(targetState.targetBedtime);

    // Morning exercise for late chronotypes to help advance phase
    const isMorningExerciseBetter =
      chronotype.classification === ChronotypeClass.MODERATE_LATE ||
      chronotype.classification === ChronotypeClass.EXTREME_LATE;

    return {
      optimalWindow: {
        start: hoursToTime(isMorningExerciseBetter ? wakeHours : wakeHours + 4),
        end: hoursToTime(bedtimeHours - 5)
      },
      avoidAfter: hoursToTime(bedtimeHours - 4),
      recommendedType: isMorningExerciseBetter ? 'morning_aerobic' : 'afternoon_strength',
      duration_min: 30
    };
  }

  /**
   * Generate sleep hygiene recommendations
   */
  private static generateSleepHygieneRecommendations(
    goals: OptimizationGoal[]
  ): SleepHygieneIntervention[] {
    const recommendations: SleepHygieneIntervention[] = [
      {
        intervention: 'Consistent wake time',
        description: 'Wake at the same time every day including weekends',
        priority: 'critical'
      },
      {
        intervention: 'Bedroom environment',
        description: 'Keep bedroom dark, quiet, and cool (18-19°C)',
        priority: 'high'
      }
    ];

    if (goals.includes(OptimizationGoal.REDUCE_LATENCY)) {
      recommendations.push({
        intervention: 'Stimulus control',
        description: 'Use bed only for sleep; leave bedroom if awake >20 min',
        priority: 'high'
      });
    }

    if (goals.includes(OptimizationGoal.INCREASE_EFFICIENCY)) {
      recommendations.push({
        intervention: 'Sleep restriction',
        description: 'Initially limit time in bed to actual sleep time',
        priority: 'high'
      });
    }

    if (goals.includes(OptimizationGoal.PHASE_ADVANCE)) {
      recommendations.push({
        intervention: 'Screen curfew',
        description: 'No screens 1-2 hours before bed or use blue-light blocking',
        priority: 'high'
      });
    }

    recommendations.push({
      intervention: 'Wind-down routine',
      description: 'Begin relaxation activities 1 hour before bed',
      priority: 'medium'
    });

    return recommendations;
  }

  /**
   * Calculate expected outcomes
   */
  private static calculateExpectedOutcomes(
    goals: OptimizationGoal[],
    chronotype: ChronotypeProfile,
    currentState?: CurrentSleepState
  ): OptimizationOutcomes {
    const outcomes: OptimizationOutcomes = {
      timeToEffect_days: 14,
      sustainabilityScore: 0.8
    };

    if (goals.includes(OptimizationGoal.INCREASE_EFFICIENCY)) {
      outcomes.sleepEfficiencyImprovement_pct = currentState
        ? Math.min(15, 92 - currentState.sleepEfficiency_pct)
        : 8;
    }

    if (goals.includes(OptimizationGoal.REDUCE_LATENCY)) {
      outcomes.latencyReduction_min = 10;
    }

    if (goals.includes(OptimizationGoal.PHASE_ADVANCE) ||
        goals.includes(OptimizationGoal.PHASE_DELAY)) {
      outcomes.phaseShift_hours = goals.includes(OptimizationGoal.PHASE_ADVANCE) ? -1.5 : 1.5;
    }

    if (goals.includes(OptimizationGoal.REDUCE_SOCIAL_JETLAG)) {
      outcomes.socialJetlagReduction_hours = chronotype.socialJetlag * 0.5;
    }

    return outcomes;
  }

  /**
   * Generate treatment timeline
   */
  private static generateTimeline(goals: OptimizationGoal[]): TreatmentPhase[] {
    const phases: TreatmentPhase[] = [];

    if (goals.includes(OptimizationGoal.PHASE_ADVANCE) ||
        goals.includes(OptimizationGoal.PHASE_DELAY)) {
      phases.push({
        name: 'Light therapy initiation',
        duration_days: 7,
        focus: 'Establish morning/evening light routine',
        expectedOutcome: '30-45 min phase shift'
      });
    }

    phases.push({
      name: 'Schedule consolidation',
      duration_days: 7,
      focus: 'Stabilize new sleep times',
      expectedOutcome: 'Improved sleep efficiency'
    });

    phases.push({
      name: 'Maintenance',
      duration_days: 14,
      focus: 'Maintain gains and fine-tune',
      expectedOutcome: 'Sustained improvement'
    });

    return phases;
  }

  /**
   * Generate warnings and contraindications
   */
  private static generateWarnings(goals: OptimizationGoal[]): PlanWarning[] {
    const warnings: PlanWarning[] = [];

    if (goals.includes(OptimizationGoal.PHASE_ADVANCE) ||
        goals.includes(OptimizationGoal.PHASE_DELAY)) {
      warnings.push({
        condition: 'Bipolar disorder',
        action: 'Light therapy may trigger mania - consult psychiatrist',
        severity: 'high'
      });
      warnings.push({
        condition: 'Eye conditions',
        action: 'Consult ophthalmologist before bright light therapy',
        severity: 'medium'
      });
    }

    return warnings;
  }
}

export default OptimizationEngine;
