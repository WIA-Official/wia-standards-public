/**
 * WIA-AUG-006: Physical Enhancement SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Physical Augmentation Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for physical enhancement including:
 * - Baseline physical assessment across six domains
 * - Enhancement configuration and monitoring
 * - Load capacity management
 * - Fatigue monitoring and prediction
 * - Injury prevention protocols
 * - Recovery optimization
 */

import {
  PhysicalDomain,
  EnhancementTech,
  RecoveryModality,
  UserProfile,
  BaselineAssessment,
  PerformanceMetrics,
  StrengthMetrics,
  EnduranceMetrics,
  SpeedMetrics,
  FlexibilityMetrics,
  CoordinationMetrics,
  BalanceMetrics,
  EnhancementTarget,
  EnhancementResult,
  LoadMonitorInput,
  LoadMonitorResult,
  LoadStatus,
  FatigueMonitorInput,
  FatigueMonitorResult,
  FatigueMetrics,
  FatigueLevel,
  InjuryRisk,
  InjuryRiskLevel,
  InjuryPreventionProtocol,
  RecoveryProtocol,
  RecoveryAssessment,
  RecoveryStatus,
  PerformanceReport,
  ENHANCEMENT_CONSTANTS,
  PhysicalEnhancementErrorCode,
  PhysicalEnhancementError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUG-006 Physical Enhancement SDK
 */
export class PhysicalEnhancementSDK {
  private version = '1.0.0';

  constructor() {}

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Assess physical baseline across all domains
   */
  assessPhysicalBaseline(user: UserProfile): BaselineAssessment {
    // Calculate baseline metrics for each domain
    const strength = this.assessStrengthBaseline(user);
    const endurance = this.assessEnduranceBaseline(user);
    const speed = this.assessSpeedBaseline(user);
    const flexibility = this.assessFlexibilityBaseline(user);
    const coordination = this.assessCoordinationBaseline(user);
    const balance = this.assessBalanceBaseline(user);

    const metrics: PerformanceMetrics = {
      strength,
      endurance,
      speed,
      flexibility,
      coordination,
      balance,
    };

    // Calculate overall fitness score
    const fitnessScore = this.calculateFitnessScore(metrics, user);

    // Determine certification level
    const certificationLevel = this.determineCertificationLevel(fitnessScore, user);

    // Medical clearance is required for moderate+ fitness or age > 50
    const medicalClearance = user.fitnessLevel !== 'sedentary' && user.age < 50;

    return {
      user,
      assessmentDate: new Date(),
      metrics,
      fitnessScore,
      assessorId: 'AUTO',
      medicalClearance,
      certificationLevel,
    };
  }

  /**
   * Configure enhancement for a specific domain
   */
  enhanceDomain(target: EnhancementTarget): EnhancementResult {
    // Validate enhancement factor
    this.validateEnhancementFactor(target.targetFactor, target.technology);

    // Calculate enhanced performance
    const enhanced = target.baseline * target.targetFactor;

    // Calculate safe load capacity
    const safetyMargin = this.getSafetyMargin(target.safetyLevel);
    const safeLoad = target.baseline * target.targetFactor * safetyMargin;

    // Calculate force and power based on domain
    let maxForce = 0;
    let powerOutput = 0;

    switch (target.domain) {
      case PhysicalDomain.STRENGTH:
        maxForce = enhanced; // Baseline is in Newtons
        powerOutput = maxForce * 0.5; // Assume 0.5 m/s velocity
        break;
      case PhysicalDomain.ENDURANCE:
        powerOutput = enhanced * 3.5; // VO2 to watts approximation
        maxForce = powerOutput / 0.5;
        break;
      case PhysicalDomain.SPEED:
        // Speed is in m/s, estimate force from mass and acceleration
        maxForce = 70 * enhanced; // Assume 70kg and 1s acceleration
        powerOutput = maxForce * enhanced;
        break;
      default:
        maxForce = target.baseline * target.targetFactor;
        powerOutput = maxForce * 0.5;
    }

    // Calculate duration limit
    const durationLimit = this.calculateDurationLimit(
      target.targetFactor,
      target.technology,
      target.domain
    );

    // Generate warnings
    const warnings = this.generateEnhancementWarnings(target);

    return {
      domain: target.domain,
      technology: target.technology,
      enhancementFactor: target.targetFactor,
      baseline: target.baseline,
      enhanced,
      safeLoad,
      maxForce,
      powerOutput,
      durationLimit,
      warnings,
    };
  }

  /**
   * Monitor load in real-time
   */
  monitorLoad(input: LoadMonitorInput): LoadMonitorResult {
    // Calculate load percentage
    const loadPercentage = (input.currentLoad / input.maxCapacity) * 100;

    // Determine load status
    let status: LoadStatus;
    const thresholds = ENHANCEMENT_CONSTANTS.LOAD_THRESHOLDS;

    if (loadPercentage < thresholds.SAFE * 100) {
      status = 'safe';
    } else if (loadPercentage < thresholds.CAUTION * 100) {
      status = 'caution';
    } else if (loadPercentage < thresholds.WARNING * 100) {
      status = 'warning';
    } else {
      status = 'critical';
    }

    // Calculate remaining safe duration
    const loadFactor = loadPercentage / 100;
    const maxSafeDuration = 3600; // 1 hour baseline
    const remainingDuration = Math.max(
      0,
      maxSafeDuration * (1 - loadFactor) - input.duration
    );

    // Calculate cumulative load index
    const cumulativeLoadIndex =
      (input.currentLoad * input.duration) / (input.maxCapacity * 3600);

    // Generate recommendation
    const recommendation = this.generateLoadRecommendation(
      status,
      loadPercentage,
      remainingDuration
    );

    // Generate warnings
    const warnings = this.generateLoadWarnings(status, loadPercentage, input.duration);

    return {
      status,
      loadPercentage,
      remainingDuration,
      recommendation,
      warnings,
      cumulativeLoadIndex,
    };
  }

  /**
   * Check and predict fatigue levels
   */
  checkFatigue(input: FatigueMonitorInput): FatigueMonitorResult {
    // Calculate component fatigue scores
    const physicalFatigue = this.calculatePhysicalFatigue(
      input.sessionDuration,
      input.intensity
    );

    const cardiovascularFatigue = this.calculateCardiovascularFatigue(
      input.heartRate,
      input.maxHeartRate
    );

    const neuromuscularFatigue = this.calculateNeuromuscularFatigue(input.forceDecline);

    const mentalFatigue = this.calculateMentalFatigue(input.sessionDuration, input.intensity);

    // Calculate composite fatigue
    const compositeFatigue =
      physicalFatigue * 0.35 +
      mentalFatigue * 0.15 +
      neuromuscularFatigue * 0.3 +
      cardiovascularFatigue * 0.2;

    // Calculate predictive fatigue index (PFI)
    const predictiveFatigueIndex = this.calculatePredictiveFatigueIndex(
      input.sessionDuration,
      input.intensity,
      input.cumulativeLoad,
      input.heartRate,
      input.maxHeartRate
    );

    const metrics: FatigueMetrics = {
      physicalFatigue,
      mentalFatigue,
      neuromuscularFatigue,
      cardiovascularFatigue,
      compositeFatigue,
      predictiveFatigueIndex,
    };

    // Determine fatigue level
    const level = this.determineFatigueLevel(predictiveFatigueIndex);

    // Calculate time to critical fatigue
    const timeToCritical = this.calculateTimeToCriticalFatigue(
      predictiveFatigueIndex,
      input.intensity
    );

    // Generate action recommendation
    const action = this.generateFatigueAction(level, predictiveFatigueIndex);

    // Calculate required rest duration
    const restRequired = this.calculateRestDuration(compositeFatigue, input.sessionDuration);

    // Generate warnings
    const warnings = this.generateFatigueWarnings(level, predictiveFatigueIndex);

    return {
      level,
      metrics,
      timeToCritical,
      action,
      restRequired,
      warnings,
    };
  }

  /**
   * Assess injury risk
   */
  assessInjuryRisk(
    fatigue: FatigueMetrics,
    load: LoadMonitorResult,
    sessionDuration: number
  ): InjuryRisk {
    // Calculate component risks
    const overexertionRisk = this.calculateOverexertionRisk(fatigue, load);
    const overuseRisk = this.calculateOveruseRisk(sessionDuration, load.cumulativeLoadIndex);
    const mechanicalFailureRisk = this.calculateMechanicalFailureRisk(load.loadPercentage);
    const cardiovascularRisk = this.calculateCardiovascularRisk(
      fatigue.cardiovascularFatigue
    );

    // Calculate composite risk
    const compositeRisk = Math.max(
      overexertionRisk,
      overuseRisk,
      mechanicalFailureRisk,
      cardiovascularRisk
    );

    // Determine risk level
    const riskLevel = this.determineInjuryRiskLevel(compositeRisk);

    return {
      overexertionRisk,
      overuseRisk,
      mechanicalFailureRisk,
      cardiovascularRisk,
      compositeRisk,
      riskLevel,
    };
  }

  /**
   * Generate injury prevention protocol
   */
  preventInjury(domain: PhysicalDomain, enhancementFactor: number): InjuryPreventionProtocol {
    // Warm-up requirements
    const warmup = {
      duration: 10 + Math.floor(enhancementFactor * 5), // 10-35 minutes
      intensity: Math.min(50, enhancementFactor * 10), // 12-50%
      exercises: this.getWarmupExercises(domain),
    };

    // Load progression limits
    const loadProgression = {
      dailyIncrease: Math.max(5, 15 - enhancementFactor * 2), // 5-13%
      weeklyIncrease: Math.max(15, 35 - enhancementFactor * 4), // 15-31%
      monthlyIncrease: Math.max(50, 110 - enhancementFactor * 12), // 50-98%
    };

    // Rest requirements
    const restRequirements = {
      minRestBetweenSessions: Math.max(2, enhancementFactor * 1.5), // 1.8-7.5 hours
      mandatoryBreaks: 120, // every 2 hours
      breakDuration: 15, // 15 minutes
    };

    // Joint-specific limits
    const jointLimits = this.getJointLimits(domain);

    // Emergency shutdown triggers
    const shutdownTriggers = [
      'Force overload > 120% safe limit',
      'Joint angle violation > 10°',
      'Acceleration spike > 30 m/s²',
      'Fatigue PFI > 0.90',
      'Pain signal detected',
      'Heart rate > 95% max',
      'Core temperature > 39.5°C',
    ];

    return {
      warmup,
      loadProgression,
      restRequirements,
      jointLimits,
      shutdownTriggers,
    };
  }

  /**
   * Optimize recovery protocol
   */
  optimizeRecovery(
    fatigue: FatigueMetrics,
    sessionDuration: number,
    intensity: number
  ): RecoveryProtocol {
    // Determine activity level
    let activityLevel: 'rest' | 'active' | 'light' | 'moderate';
    if (fatigue.compositeFatigue > 80) {
      activityLevel = 'rest';
    } else if (fatigue.compositeFatigue > 60) {
      activityLevel = 'active';
    } else if (fatigue.compositeFatigue > 40) {
      activityLevel = 'light';
    } else {
      activityLevel = 'moderate';
    }

    // Calculate recovery duration
    const duration = Math.ceil((sessionDuration / 60) * (fatigue.compositeFatigue / 50));

    // Select recovery modalities
    const modalities = this.selectRecoveryModalities(fatigue, intensity);

    // Create nutrition plan
    const nutritionPlan = this.createNutritionPlan(sessionDuration, intensity);

    // Determine sleep target
    const sleepTarget =
      ENHANCEMENT_CONSTANTS.RECOVERY_TARGETS.MIN_SLEEP +
      (fatigue.compositeFatigue > 70 ? 2 : fatigue.compositeFatigue > 50 ? 1 : 0);

    // Calculate expected recovery
    const expectedRecovery = Math.min(
      100,
      50 + modalities.length * 10 + (sleepTarget - 7) * 10
    );

    // Generate recommendations
    const recommendations = this.generateRecoveryRecommendations(
      fatigue,
      activityLevel,
      modalities
    );

    return {
      activityLevel,
      duration,
      modalities,
      nutritionPlan,
      sleepTarget,
      expectedRecovery,
      recommendations,
    };
  }

  /**
   * Assess current recovery status
   */
  assessRecovery(
    sleepHours: number,
    sleepQuality: number,
    hrvScore: number,
    muscleSoreness: number,
    restingHR: number,
    baselineHR: number
  ): RecoveryAssessment {
    // Calculate recovery score components
    const sleepScore = this.calculateSleepScore(sleepHours, sleepQuality);
    const hrvComponent = hrvScore;
    const sorenessScore = 100 - muscleSoreness * 10;
    const hrScore = 100 - Math.abs(restingHR - baselineHR) * 2;

    // Calculate overall recovery score
    const recoveryScore =
      sleepScore * 0.25 + hrvComponent * 0.2 + sorenessScore * 0.15 + hrScore * 0.15 + 25;

    // Calculate current recovery percentage
    const currentRecovery = Math.min(100, recoveryScore);

    // Determine time to full recovery
    const recoveryDeficit = 100 - currentRecovery;
    const timeToFullRecovery = recoveryDeficit * 6; // ~6 minutes per percentage point

    // Determine if ready for activity
    const readyForActivity =
      recoveryScore >= ENHANCEMENT_CONSTANTS.RECOVERY_TARGETS.READY_THRESHOLD;

    // Determine status
    let status: RecoveryStatus;
    if (recoveryScore < 60) {
      status = 'not_ready';
    } else if (recoveryScore < 75) {
      status = 'partially_ready';
    } else if (recoveryScore < 90) {
      status = 'ready';
    } else {
      status = 'fully_ready';
    }

    // Generate recommendations
    const recommendations = this.generateRecoveryAssessmentRecommendations(
      recoveryScore,
      sleepHours,
      muscleSoreness
    );

    return {
      currentRecovery,
      recoveryScore,
      timeToFullRecovery,
      readyForActivity,
      status,
      sleepQuality,
      hrvScore,
      muscleSoreness,
      recommendations,
    };
  }

  /**
   * Measure overall performance across all domains
   */
  measurePerformance(
    baseline: BaselineAssessment,
    current: PerformanceMetrics
  ): PerformanceReport {
    // Calculate progress for each domain
    const progress = [
      {
        domain: PhysicalDomain.STRENGTH,
        baseline: baseline.metrics.strength.maxForce,
        current: current.strength.maxForce,
        improvement:
          ((current.strength.maxForce - baseline.metrics.strength.maxForce) /
            baseline.metrics.strength.maxForce) *
          100,
      },
      {
        domain: PhysicalDomain.ENDURANCE,
        baseline: baseline.metrics.endurance.vo2Max,
        current: current.endurance.vo2Max,
        improvement:
          ((current.endurance.vo2Max - baseline.metrics.endurance.vo2Max) /
            baseline.metrics.endurance.vo2Max) *
          100,
      },
      {
        domain: PhysicalDomain.SPEED,
        baseline: baseline.metrics.speed.velocity,
        current: current.speed.velocity,
        improvement:
          ((current.speed.velocity - baseline.metrics.speed.velocity) /
            baseline.metrics.speed.velocity) *
          100,
      },
      {
        domain: PhysicalDomain.FLEXIBILITY,
        baseline: baseline.metrics.flexibility.mobilityScore,
        current: current.flexibility.mobilityScore,
        improvement:
          ((current.flexibility.mobilityScore - baseline.metrics.flexibility.mobilityScore) /
            baseline.metrics.flexibility.mobilityScore) *
          100,
      },
      {
        domain: PhysicalDomain.COORDINATION,
        baseline: baseline.metrics.coordination.accuracy,
        current: current.coordination.accuracy,
        improvement:
          ((current.coordination.accuracy - baseline.metrics.coordination.accuracy) /
            baseline.metrics.coordination.accuracy) *
          100,
      },
      {
        domain: PhysicalDomain.BALANCE,
        baseline: baseline.metrics.balance.stabilityScore,
        current: current.balance.stabilityScore,
        improvement:
          ((current.balance.stabilityScore - baseline.metrics.balance.stabilityScore) /
            baseline.metrics.balance.stabilityScore) *
          100,
      },
    ];

    // Generate recommendations
    const recommendations = this.generatePerformanceRecommendations(progress);

    return {
      id: `PR-${Date.now()}`,
      userId: baseline.user.id,
      period: {
        start: baseline.assessmentDate,
        end: new Date(),
      },
      baseline,
      current,
      enhancementHistory: [],
      trainingSummary: {
        totalSessions: 0,
        totalDuration: 0,
        totalLoad: 0,
        avgSessionDuration: 0,
      },
      safetySummary: {
        totalAlerts: 0,
        criticalAlerts: 0,
        emergencyShutdowns: 0,
        injuryIncidents: 0,
      },
      recoverySummary: {
        avgRecoveryScore: 0,
        avgSleepHours: 0,
        avgRestDays: 0,
      },
      progress,
      recommendations,
      generatedAt: new Date(),
    };
  }

  // ============================================================================
  // Private Helper Methods
  // ============================================================================

  private assessStrengthBaseline(user: UserProfile): StrengthMetrics {
    // Simplified baseline calculation based on user profile
    const ageFactors = 1 - (user.age - 25) * 0.01;
    const fitnessMultiplier = this.getFitnessMultiplier(user.fitnessLevel);
    const baseForce = user.weight * 9.81 * 3; // ~3x body weight

    return {
      maxForce: baseForce * fitnessMultiplier * ageFactors,
      peakPower: (baseForce * fitnessMultiplier * ageFactors * 0.5) / 9.81,
      sustainedDuration: 30 * fitnessMultiplier,
      loadCapacity: user.weight * 2 * fitnessMultiplier,
      oneRepMax: {
        benchPress: user.weight * fitnessMultiplier * 1.2,
        squat: user.weight * fitnessMultiplier * 1.8,
        deadlift: user.weight * fitnessMultiplier * 2.0,
      },
    };
  }

  private assessEnduranceBaseline(user: UserProfile): EnduranceMetrics {
    const ageFactors = 1 - (user.age - 25) * 0.005;
    const fitnessMultiplier = this.getFitnessMultiplier(user.fitnessLevel);
    const baseVO2 = 35; // Average VO2 max

    return {
      vo2Max: baseVO2 * fitnessMultiplier * ageFactors,
      lactateThreshold: 60 + fitnessMultiplier * 10,
      duration: 30 * fitnessMultiplier,
      hrRecovery: 120 - fitnessMultiplier * 20,
      restingHR: 70 - fitnessMultiplier * 5,
    };
  }

  private assessSpeedBaseline(user: UserProfile): SpeedMetrics {
    const ageFactors = 1 - (user.age - 25) * 0.008;
    const fitnessMultiplier = this.getFitnessMultiplier(user.fitnessLevel);

    return {
      velocity: 5.5 * fitnessMultiplier * ageFactors,
      acceleration: 3.0 * fitnessMultiplier * ageFactors,
      reactionTime: 250 - fitnessMultiplier * 30,
      cadence: 3.0 * fitnessMultiplier,
      sprintTimes: {
        tenMeter: 2.0 / (fitnessMultiplier * ageFactors),
        twentyMeter: 3.5 / (fitnessMultiplier * ageFactors),
        fortyMeter: 6.0 / (fitnessMultiplier * ageFactors),
      },
    };
  }

  private assessFlexibilityBaseline(user: UserProfile): FlexibilityMetrics {
    const ageFactors = 1 - (user.age - 25) * 0.006;
    const fitnessMultiplier = this.getFitnessMultiplier(user.fitnessLevel);

    return {
      rangeOfMotion: {
        shoulder: {
          flexion: 150 * fitnessMultiplier * ageFactors,
          extension: 50 * fitnessMultiplier * ageFactors,
          abduction: 150 * fitnessMultiplier * ageFactors,
        },
        hip: {
          flexion: 110 * fitnessMultiplier * ageFactors,
          extension: 20 * fitnessMultiplier * ageFactors,
          abduction: 40 * fitnessMultiplier * ageFactors,
        },
        spine: {
          flexion: 60 * fitnessMultiplier * ageFactors,
          extension: 20 * fitnessMultiplier * ageFactors,
          lateralBend: 25 * fitnessMultiplier * ageFactors,
        },
      },
      sitAndReach: 15 * fitnessMultiplier * ageFactors,
      stiffnessIndex: 50 - fitnessMultiplier * 5,
      mobilityScore: 50 + fitnessMultiplier * 10,
    };
  }

  private assessCoordinationBaseline(user: UserProfile): CoordinationMetrics {
    const ageFactors = 1 - (user.age - 25) * 0.007;
    const fitnessMultiplier = this.getFitnessMultiplier(user.fitnessLevel);

    return {
      precision: 15 - fitnessMultiplier * 2,
      accuracy: 60 + fitnessMultiplier * 10,
      smoothness: 50 - fitnessMultiplier * 5,
      errorRate: 20 - fitnessMultiplier * 3,
      completionTime: 10 / (fitnessMultiplier * ageFactors),
    };
  }

  private assessBalanceBaseline(user: UserProfile): BalanceMetrics {
    const ageFactors = 1 - (user.age - 25) * 0.008;
    const fitnessMultiplier = this.getFitnessMultiplier(user.fitnessLevel);

    return {
      stabilityScore: 50 + fitnessMultiplier * 12,
      swayArea: 50 - fitnessMultiplier * 8,
      swayVelocity: 30 - fitnessMultiplier * 5,
      singleLegStand: 20 * fitnessMultiplier * ageFactors,
      yBalanceScore: 80 * fitnessMultiplier * ageFactors,
    };
  }

  private getFitnessMultiplier(level: string): number {
    switch (level) {
      case 'sedentary':
        return 0.7;
      case 'low':
        return 0.85;
      case 'moderate':
        return 1.0;
      case 'high':
        return 1.2;
      case 'elite':
        return 1.5;
      default:
        return 1.0;
    }
  }

  private calculateFitnessScore(metrics: PerformanceMetrics, user: UserProfile): number {
    // Normalize and weight each domain
    const scores = {
      strength: (metrics.strength.maxForce / (user.weight * 30)) * 100,
      endurance: (metrics.endurance.vo2Max / 60) * 100,
      speed: (metrics.speed.velocity / 10) * 100,
      flexibility: metrics.flexibility.mobilityScore,
      coordination: metrics.coordination.accuracy,
      balance: metrics.balance.stabilityScore,
    };

    // Weighted average
    return (
      scores.strength * 0.2 +
      scores.endurance * 0.2 +
      scores.speed * 0.15 +
      scores.flexibility * 0.15 +
      scores.coordination * 0.15 +
      scores.balance * 0.15
    );
  }

  private determineCertificationLevel(
    score: number,
    user: UserProfile
  ): 'Level1' | 'Level2' | 'Level3' {
    if (score < 60 || user.fitnessLevel === 'sedentary' || user.fitnessLevel === 'low') {
      return 'Level1';
    } else if (score < 80 || user.fitnessLevel === 'moderate') {
      return 'Level2';
    } else {
      return 'Level3';
    }
  }

  private validateEnhancementFactor(factor: number, tech: EnhancementTech): void {
    const range = ENHANCEMENT_CONSTANTS.TECH_RANGES[tech];
    if (factor < range.min || factor > range.max) {
      throw new PhysicalEnhancementError(
        PhysicalEnhancementErrorCode.INVALID_ENHANCEMENT_FACTOR,
        `Enhancement factor ${factor} is outside valid range [${range.min}, ${range.max}] for ${tech}`
      );
    }
  }

  private getSafetyMargin(level: SafetyLevel): number {
    switch (level) {
      case 'basic':
        return ENHANCEMENT_CONSTANTS.SAFETY_MARGINS.BASIC;
      case 'standard':
        return ENHANCEMENT_CONSTANTS.SAFETY_MARGINS.STANDARD;
      case 'high':
      case 'critical':
        return ENHANCEMENT_CONSTANTS.SAFETY_MARGINS.HIGH;
      default:
        return ENHANCEMENT_CONSTANTS.SAFETY_MARGINS.STANDARD;
    }
  }

  private calculateDurationLimit(
    factor: number,
    tech: EnhancementTech,
    domain: PhysicalDomain
  ): number {
    // Base duration in minutes
    let baseDuration = 120;

    // Adjust for enhancement factor (higher factor = lower duration)
    baseDuration = baseDuration / Math.sqrt(factor);

    // Adjust for technology
    const techMultiplier =
      tech === EnhancementTech.EXOSKELETON
        ? 1.5
        : tech === EnhancementTech.CARDIO_BOOST
          ? 0.8
          : 1.0;

    // Adjust for domain
    const domainMultiplier =
      domain === PhysicalDomain.ENDURANCE ? 1.5 : domain === PhysicalDomain.STRENGTH ? 0.7 : 1.0;

    return Math.floor(baseDuration * techMultiplier * domainMultiplier);
  }

  private generateEnhancementWarnings(target: EnhancementTarget): string[] {
    const warnings: string[] = [];

    if (target.targetFactor > ENHANCEMENT_CONSTANTS.ENHANCEMENT_FACTORS.SAFE_MAX) {
      warnings.push('Enhancement factor exceeds safe maximum (3.0x). Medical supervision required.');
    }

    if (target.technology === EnhancementTech.BONE_REINFORCE) {
      warnings.push('Bone reinforcement requires 6-12 month recovery period.');
    }

    if (target.domain === PhysicalDomain.STRENGTH && target.targetFactor > 3.0) {
      warnings.push('High strength enhancement may stress joints and connective tissue.');
    }

    return warnings;
  }

  private generateLoadRecommendation(
    status: LoadStatus,
    percentage: number,
    remaining: number
  ): string {
    switch (status) {
      case 'safe':
        return `Load at ${percentage.toFixed(1)}% capacity. Continue normal operation.`;
      case 'caution':
        return `Load at ${percentage.toFixed(1)}% capacity. Monitor closely. ${Math.floor(remaining / 60)} minutes remaining at this load.`;
      case 'warning':
        return `Load at ${percentage.toFixed(1)}% capacity. Reduce load or take break within ${Math.floor(remaining / 60)} minutes.`;
      case 'critical':
        return `Critical load at ${percentage.toFixed(1)}% capacity. Reduce load immediately.`;
      default:
        return 'Unknown status';
    }
  }

  private generateLoadWarnings(status: LoadStatus, percentage: number, duration: number): string[] {
    const warnings: string[] = [];

    if (status === 'warning' || status === 'critical') {
      warnings.push(`Load exceeds ${percentage.toFixed(0)}% of safe capacity`);
    }

    if (duration > 7200) {
      warnings.push('Session duration exceeds 2 hours. Mandatory break required.');
    }

    if (status === 'critical') {
      warnings.push('CRITICAL: Risk of equipment failure or injury. Reduce load now.');
    }

    return warnings;
  }

  private calculatePhysicalFatigue(duration: number, intensity: number): number {
    // Physical fatigue increases with time and intensity
    const timeFactor = Math.min(100, (duration / 3600) * 30); // 30% per hour
    const intensityFactor = intensity * 0.7; // Up to 70% from intensity
    return Math.min(100, timeFactor + intensityFactor);
  }

  private calculateCardiovascularFatigue(hr: number, maxHR: number): number {
    const hrPercent = (hr / maxHR) * 100;
    return Math.min(100, hrPercent);
  }

  private calculateNeuromuscularFatigue(forceDecline: number): number {
    // Force decline is a direct indicator of neuromuscular fatigue
    return Math.min(100, forceDecline);
  }

  private calculateMentalFatigue(duration: number, intensity: number): number {
    // Mental fatigue accumulates with time, especially at high intensity
    const timeFactor = Math.min(100, (duration / 3600) * 25);
    const intensityFactor = intensity * 0.5;
    return Math.min(100, timeFactor + intensityFactor);
  }

  private calculatePredictiveFatigueIndex(
    duration: number,
    intensity: number,
    cumulativeLoad: number,
    hr: number,
    maxHR: number
  ): number {
    const alpha = 0.25;
    const beta = 0.35;
    const gamma = 0.25;
    const delta = 0.15;

    const T = 7200; // Predicted time to fatigue (2 hours)
    const Imax = 100;
    const CLImax = 300;

    const pfi =
      alpha * (duration / T) +
      beta * (intensity / Imax) +
      gamma * (cumulativeLoad / CLImax) +
      delta * (hr / maxHR);

    return Math.min(1.0, pfi);
  }

  private determineFatigueLevel(pfi: number): FatigueLevel {
    const thresholds = ENHANCEMENT_CONSTANTS.FATIGUE_THRESHOLDS;

    if (pfi < thresholds.LOW) {
      return 'low';
    } else if (pfi < thresholds.MODERATE) {
      return 'moderate';
    } else if (pfi < thresholds.HIGH) {
      return 'high';
    } else {
      return 'critical';
    }
  }

  private calculateTimeToCriticalFatigue(pfi: number, intensity: number): number {
    // Estimate time to critical fatigue (PFI = 0.95)
    if (pfi >= 0.95) return 0;

    const remaining = 0.95 - pfi;
    const rate = intensity / 10000; // Fatigue accumulation rate

    return Math.floor(remaining / rate);
  }

  private generateFatigueAction(level: FatigueLevel, pfi: number): string {
    switch (level) {
      case 'low':
        return 'Continue normal operation. Monitor fatigue levels.';
      case 'moderate':
        return `Reduce enhancement factor by 20%. Increase monitoring frequency. PFI: ${(pfi * 100).toFixed(1)}%`;
      case 'high':
        return `Reduce enhancement factor by 50%. Take break within 10 minutes. PFI: ${(pfi * 100).toFixed(1)}%`;
      case 'critical':
        return `Initiating gradual shutdown. Mandatory 30-minute rest required. PFI: ${(pfi * 100).toFixed(1)}%`;
      default:
        return 'Unknown fatigue level';
    }
  }

  private calculateRestDuration(fatigue: number, sessionDuration: number): number {
    // Rest duration is proportional to fatigue and session duration
    const baseDuration = 30; // 30 minutes baseline
    const fatigueFactor = fatigue / 50; // Doubles at 50% fatigue
    const durationFactor = sessionDuration / 3600; // Hours

    return Math.ceil(baseDuration * fatigueFactor * durationFactor);
  }

  private generateFatigueWarnings(level: FatigueLevel, pfi: number): string[] {
    const warnings: string[] = [];

    if (level === 'moderate') {
      warnings.push('Moderate fatigue detected. Consider reducing intensity.');
    }

    if (level === 'high') {
      warnings.push('High fatigue level. Break recommended within 10 minutes.');
    }

    if (level === 'critical') {
      warnings.push('CRITICAL FATIGUE. Immediate rest required to prevent injury.');
    }

    if (pfi > 0.85) {
      warnings.push(`Predictive Fatigue Index at ${(pfi * 100).toFixed(1)}%`);
    }

    return warnings;
  }

  private calculateOverexertionRisk(fatigue: FatigueMetrics, load: LoadMonitorResult): number {
    // Combine fatigue and load to assess overexertion risk
    return Math.min(100, fatigue.compositeFatigue * 0.6 + load.loadPercentage * 0.4);
  }

  private calculateOveruseRisk(duration: number, cumulativeLoad: number): number {
    // Risk increases with session duration and cumulative load
    const durationRisk = Math.min(100, (duration / 14400) * 100); // 4 hours = 100%
    const loadRisk = Math.min(100, (cumulativeLoad / 300) * 100); // CLI 300 = 100%
    return Math.max(durationRisk, loadRisk);
  }

  private calculateMechanicalFailureRisk(loadPercentage: number): number {
    // Risk increases exponentially as load approaches capacity
    if (loadPercentage < 70) return 0;
    if (loadPercentage < 85) return 20;
    if (loadPercentage < 95) return 50;
    return 90;
  }

  private calculateCardiovascularRisk(cardiovascularFatigue: number): number {
    // Direct mapping of cardiovascular fatigue to risk
    return cardiovascularFatigue;
  }

  private determineInjuryRiskLevel(risk: number): InjuryRiskLevel {
    if (risk < 20) return 'minimal';
    if (risk < 40) return 'low';
    if (risk < 60) return 'moderate';
    if (risk < 80) return 'high';
    return 'critical';
  }

  private getWarmupExercises(domain: PhysicalDomain): string[] {
    const exercises: Record<PhysicalDomain, string[]> = {
      [PhysicalDomain.STRENGTH]: [
        'Light cardio (5 min)',
        'Dynamic stretching',
        'Progressive resistance warm-up sets',
      ],
      [PhysicalDomain.ENDURANCE]: [
        'Low-intensity cardio (10 min)',
        'Dynamic mobility',
        'Gradual pace increase',
      ],
      [PhysicalDomain.SPEED]: [
        'Light jogging (5 min)',
        'Dynamic leg swings',
        'Progressive sprint drills',
      ],
      [PhysicalDomain.FLEXIBILITY]: [
        'Light movement (5 min)',
        'Gentle stretching',
        'Progressive range of motion',
      ],
      [PhysicalDomain.COORDINATION]: [
        'Simple movement patterns',
        'Progressive complexity drills',
        'Reaction time exercises',
      ],
      [PhysicalDomain.BALANCE]: [
        'Static balance holds',
        'Dynamic balance exercises',
        'Progressive instability challenges',
      ],
    };

    return exercises[domain] || [];
  }

  private getJointLimits(domain: PhysicalDomain): Record<string, any> {
    // Simplified joint limits
    return {
      shoulder: { maxAngle: 180, maxForce: 500, maxRepetitions: 10000 },
      elbow: { maxAngle: 145, maxForce: 300, maxRepetitions: 15000 },
      hip: { maxAngle: 120, maxForce: 2000, maxRepetitions: 8000 },
      knee: { maxAngle: 140, maxForce: 1500, maxRepetitions: 12000 },
      ankle: { maxAngle: 90, maxForce: 1000, maxRepetitions: 15000 },
    };
  }

  private selectRecoveryModalities(
    fatigue: FatigueMetrics,
    intensity: number
  ): RecoveryModality[] {
    const modalities: RecoveryModality[] = [];

    if (fatigue.compositeFatigue < 50) {
      modalities.push(RecoveryModality.ACTIVE_RECOVERY);
    } else {
      modalities.push(RecoveryModality.PASSIVE_RECOVERY);
    }

    if (intensity > 70) {
      modalities.push(RecoveryModality.CONTRAST_THERAPY);
      modalities.push(RecoveryModality.COMPRESSION);
    }

    if (fatigue.physicalFatigue > 60) {
      modalities.push(RecoveryModality.MASSAGE);
    }

    if (intensity > 85) {
      modalities.push(RecoveryModality.CRYOTHERAPY);
    }

    return modalities;
  }

  private createNutritionPlan(duration: number, intensity: number): any {
    const durationHours = duration / 3600;
    const baseWeight = 75; // Assume 75kg

    return {
      carbs: Math.ceil(baseWeight * 1.0 * (intensity / 100)),
      protein: Math.ceil(baseWeight * 0.35),
      fats: Math.ceil(baseWeight * 0.8),
      hydration: Math.ceil(durationHours * 0.5 + 1.5),
      timing: ['Immediately post-exercise', 'Within 2 hours', 'Before sleep'],
      supplements: ['Protein shake', 'Electrolytes', 'Vitamin C'],
    };
  }

  private generateRecoveryRecommendations(
    fatigue: FatigueMetrics,
    activityLevel: string,
    modalities: RecoveryModality[]
  ): string[] {
    const recommendations: string[] = [];

    recommendations.push(`Recovery activity level: ${activityLevel}`);

    if (fatigue.compositeFatigue > 70) {
      recommendations.push('Prioritize sleep and complete rest');
    }

    modalities.forEach((m) => {
      recommendations.push(`Apply ${m.toLowerCase().replace(/_/g, ' ')}`);
    });

    recommendations.push('Hydrate with 150% of fluid lost');
    recommendations.push('Consume protein and carbohydrates within 30 minutes');

    return recommendations;
  }

  private calculateSleepScore(hours: number, quality: number): number {
    const targetHours = ENHANCEMENT_CONSTANTS.RECOVERY_TARGETS.MIN_SLEEP;
    const durationScore = Math.min(100, (hours / targetHours) * 100);
    return (durationScore + quality) / 2;
  }

  private generateRecoveryAssessmentRecommendations(
    score: number,
    sleepHours: number,
    soreness: number
  ): string[] {
    const recommendations: string[] = [];

    if (score < 60) {
      recommendations.push('Not ready for training. Continue rest and recovery.');
    } else if (score < 75) {
      recommendations.push('Partially recovered. Light activity only (< 60% intensity).');
    } else if (score < 90) {
      recommendations.push('Well recovered. Normal training intensity allowed (60-85%).');
    } else {
      recommendations.push('Fully recovered. High-intensity training allowed (up to 100%).');
    }

    if (sleepHours < 7) {
      recommendations.push('Increase sleep duration to 7-9 hours.');
    }

    if (soreness > 5) {
      recommendations.push('High muscle soreness. Consider massage or active recovery.');
    }

    return recommendations;
  }

  private generatePerformanceRecommendations(
    progress: { domain: PhysicalDomain; improvement: number }[]
  ): string[] {
    const recommendations: string[] = [];

    progress.forEach((p) => {
      if (p.improvement > 20) {
        recommendations.push(
          `Excellent progress in ${p.domain} (${p.improvement.toFixed(1)}% improvement)`
        );
      } else if (p.improvement > 10) {
        recommendations.push(
          `Good progress in ${p.domain} (${p.improvement.toFixed(1)}% improvement)`
        );
      } else if (p.improvement > 0) {
        recommendations.push(
          `Moderate progress in ${p.domain} (${p.improvement.toFixed(1)}% improvement)`
        );
      } else {
        recommendations.push(
          `Focus needed on ${p.domain} (${p.improvement.toFixed(1)}% change). Consider increasing training volume.`
        );
      }
    });

    return recommendations;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Assess physical baseline (standalone)
 */
export function assessPhysicalBaseline(user: UserProfile): BaselineAssessment {
  const sdk = new PhysicalEnhancementSDK();
  return sdk.assessPhysicalBaseline(user);
}

/**
 * Enhance domain (standalone)
 */
export function enhanceDomain(target: EnhancementTarget): EnhancementResult {
  const sdk = new PhysicalEnhancementSDK();
  return sdk.enhanceDomain(target);
}

/**
 * Monitor load (standalone)
 */
export function monitorLoad(input: LoadMonitorInput): LoadMonitorResult {
  const sdk = new PhysicalEnhancementSDK();
  return sdk.monitorLoad(input);
}

/**
 * Prevent injury (standalone)
 */
export function preventInjury(
  domain: PhysicalDomain,
  enhancementFactor: number
): InjuryPreventionProtocol {
  const sdk = new PhysicalEnhancementSDK();
  return sdk.preventInjury(domain, enhancementFactor);
}

/**
 * Optimize recovery (standalone)
 */
export function optimizeRecovery(
  fatigue: FatigueMetrics,
  sessionDuration: number,
  intensity: number
): RecoveryProtocol {
  const sdk = new PhysicalEnhancementSDK();
  return sdk.optimizeRecovery(fatigue, sessionDuration, intensity);
}

/**
 * Measure performance (standalone)
 */
export function measurePerformance(
  baseline: BaselineAssessment,
  current: PerformanceMetrics
): PerformanceReport {
  const sdk = new PhysicalEnhancementSDK();
  return sdk.measurePerformance(baseline, current);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { PhysicalEnhancementSDK };
export default PhysicalEnhancementSDK;
