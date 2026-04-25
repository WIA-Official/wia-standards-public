/**
 * WIA-AUG-007: Bionic Limb SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Bionics Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for bionic limb systems including:
 * - Limb classification and assessment
 * - Control system calibration
 * - Sensory feedback management
 * - Grip pattern control
 * - Gait analysis (lower limbs)
 * - Power and battery management
 * - Maintenance scheduling
 */

import {
  LimbType,
  LimbCategory,
  LimbLocation,
  LimbClassificationInput,
  LimbClassification,
  ControlMethod,
  ControlCalibration,
  FeedbackType,
  PressureFeedback,
  GripPattern,
  GripConfig,
  GaitAnalysis,
  SpatiotemporalParameters,
  SymmetryMetrics,
  MaintenanceSchedule,
  MaintenanceTask,
  MaintenanceInterval,
  BionicLimbDevice,
  PatientInfo,
  CompatibilityAssessment,
  DexterityAssessment,
  LimbReport,
  PowerManagement,
  BatteryState,
  BIONIC_LIMB_CONSTANTS,
  BionicLimbErrorCode,
  BionicLimbError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUG-007 Bionic Limb SDK
 */
export class BionicLimbSDK {
  private version = '1.0.0';

  constructor() {}

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Classify a bionic limb
   */
  classifyLimb(input: LimbClassificationInput): LimbClassification {
    // Validate input
    this.validateLimbInput(input);

    // Determine location
    const location: LimbLocation = ['UPPER_ARM', 'FOREARM', 'HAND', 'FINGER'].includes(input.type)
      ? 'UPPER'
      : 'LOWER';

    // Calculate complexity score
    const sensorCount = input.sensorCount || 0;
    const controlMethods = input.controlMethods || 1;
    const complexityScore =
      input.dof * 0.4 + sensorCount * 0.3 + controlMethods * 0.3 * 10;

    // Determine category
    let category: LimbCategory;
    if (complexityScore <= 5) category = 'Minimal';
    else if (complexityScore <= 10) category = 'Basic';
    else if (complexityScore <= 15) category = 'Moderate';
    else category = 'Advanced';

    // Recommend control methods
    const recommendedControls = this.recommendControlMethods(input.type, input.dof, category);

    // Determine amputation level
    const amputationLevel = this.getAmputationLevel(input.type);

    return {
      type: input.type,
      location,
      amputationLevel,
      dof: input.dof,
      category,
      complexityScore,
      recommendedControls,
    };
  }

  /**
   * Calibrate control system
   */
  calibrateControl(
    limbId: string,
    userId: string,
    method: ControlMethod,
    trainingData?: unknown
  ): ControlCalibration {
    // Simulate calibration process
    const accuracy = 0.85 + Math.random() * 0.1; // 85-95%

    const now = new Date();
    const validUntil = new Date(now);
    validUntil.setMonth(validUntil.getMonth() + 3); // Valid for 3 months

    const nextCalibration = new Date(now);
    nextCalibration.setMonth(nextCalibration.getMonth() + 1); // Recalibrate monthly

    return {
      limbId,
      userId,
      method,
      timestamp: now,
      accuracy,
      trainingPoints: trainingData ? 100 : 0,
      validUntil,
      nextCalibration,
    };
  }

  /**
   * Provide sensory feedback
   */
  provideFeedback(
    type: FeedbackType,
    intensity: number,
    location: string,
    duration?: number
  ): { success: boolean; message: string } {
    // Validate intensity
    if (intensity < 0 || intensity > 1) {
      throw new BionicLimbError(
        BionicLimbErrorCode.FEEDBACK_FAILED,
        'Intensity must be between 0 and 1'
      );
    }

    // Simulate feedback delivery
    return {
      success: true,
      message: `${type} feedback delivered at ${location} with intensity ${intensity}`,
    };
  }

  /**
   * Analyze gait (for lower limb prosthetics)
   */
  analyzeGait(limbId: string, duration: number, sensorData?: unknown[]): GaitAnalysis {
    // Simulate gait analysis
    const spatiotemporal: SpatiotemporalParameters = {
      strideLength: 130 + Math.random() * 15,
      stepLength: 65 + Math.random() * 8,
      stepWidth: 10 + Math.random() * 5,
      cadence: 105 + Math.random() * 10,
      velocity: 1.1 + Math.random() * 0.2,
      stanceTime: 60 + Math.random() * 2,
      swingTime: 40 - Math.random() * 2,
      doubleSupport: 10 + Math.random() * 2,
    };

    const symmetry: SymmetryMetrics = {
      spatialSymmetry: 0.9 + Math.random() * 0.08,
      temporalSymmetry: 0.88 + Math.random() * 0.1,
      forceSymmetry: 0.85 + Math.random() * 0.12,
      overallSymmetry: 0.88 + Math.random() * 0.1,
    };

    return {
      timestamp: new Date(),
      duration,
      spatiotemporal,
      kinematics: {
        hipFlexion: { min: 0, max: 30, mean: 15 },
        kneeFlexion: { min: 0, max: 65, mean: 30 },
        ankleDorsiflexion: { min: -15, max: 10, mean: 0 },
        pelvicTilt: { min: -5, max: 5, mean: 0 },
      },
      symmetry,
      efficiency: 0.85 + Math.random() * 0.1,
      stability: 0.9 + Math.random() * 0.08,
      anomalies: [],
    };
  }

  /**
   * Select and execute grip pattern
   */
  selectGrip(pattern: GripPattern, force: number, speed: number = 0.5): GripConfig {
    // Validate force
    const { MIN, MAX } = BIONIC_LIMB_CONSTANTS.FORCE;
    if (force < MIN || force > MAX) {
      throw new BionicLimbError(
        BionicLimbErrorCode.GRIP_FAILURE,
        `Force must be between ${MIN}N and ${MAX}N`
      );
    }

    // Validate speed
    if (speed < 0 || speed > 1) {
      throw new BionicLimbError(
        BionicLimbErrorCode.GRIP_FAILURE,
        'Speed must be between 0 and 1'
      );
    }

    // Determine if precision mode based on pattern
    const precision = [
      'PRECISION_GRIP',
      'LATERAL_PINCH',
      'TRIPOD_PINCH',
      'TIP_PINCH',
    ].includes(pattern);

    return {
      pattern,
      force,
      speed,
      precision,
    };
  }

  /**
   * Schedule maintenance
   */
  scheduleMaintenance(
    limbId: string,
    interval: MaintenanceInterval = 'monthly'
  ): MaintenanceSchedule {
    const tasks: MaintenanceTask[] = [];
    const now = new Date();

    // Daily tasks
    if (['daily', 'weekly', 'monthly', 'quarterly', 'annual'].includes(interval)) {
      const nextDaily = new Date(now);
      nextDaily.setDate(nextDaily.getDate() + 1);

      tasks.push({
        id: 'daily-001',
        name: 'Visual inspection and cleaning',
        type: 'inspection',
        interval: 'daily',
        duration: 10,
        requiredBy: 'user',
        nextDue: nextDaily,
        status: 'pending',
      });
    }

    // Weekly tasks
    if (['weekly', 'monthly', 'quarterly', 'annual'].includes(interval)) {
      const nextWeekly = new Date(now);
      nextWeekly.setDate(nextWeekly.getDate() + 7);

      tasks.push({
        id: 'weekly-001',
        name: 'Deep cleaning and electrode check',
        type: 'cleaning',
        interval: 'weekly',
        duration: 30,
        requiredBy: 'user',
        nextDue: nextWeekly,
        status: 'pending',
      });
    }

    // Monthly tasks
    if (['monthly', 'quarterly', 'annual'].includes(interval)) {
      const nextMonthly = new Date(now);
      nextMonthly.setMonth(nextMonthly.getMonth() + 1);

      tasks.push({
        id: 'monthly-001',
        name: 'Full system diagnostic and calibration',
        type: 'calibration',
        interval: 'monthly',
        duration: 60,
        requiredBy: 'technician',
        nextDue: nextMonthly,
        status: 'pending',
      });
    }

    // Annual tasks
    if (interval === 'annual') {
      const nextAnnual = new Date(now);
      nextAnnual.setFullYear(nextAnnual.getFullYear() + 1);

      tasks.push({
        id: 'annual-001',
        name: 'Comprehensive service and component replacement',
        type: 'service',
        interval: 'annual',
        duration: 180,
        requiredBy: 'specialist',
        nextDue: nextAnnual,
        status: 'pending',
      });
    }

    // Find next maintenance date
    const nextMaintenance = tasks.reduce((earliest, task) =>
      task.nextDue < earliest ? task.nextDue : earliest
    , tasks[0]?.nextDue || now);

    return {
      limbId,
      tasks,
      overdueCount: 0,
      nextMaintenance,
    };
  }

  /**
   * Assess limb compatibility for a patient
   */
  assessLimb(input: {
    limb: Partial<BionicLimbDevice>;
    patient: Partial<PatientInfo>;
    requirements?: { primaryActivities?: string[]; sensoryFeedback?: boolean };
  }): CompatibilityAssessment {
    // Calculate compatibility based on various factors
    let compatibility = 70; // Base score

    // Muscle quality affects control
    if (input.patient.muscleQuality === 'excellent') compatibility += 15;
    else if (input.patient.muscleQuality === 'good') compatibility += 10;
    else if (input.patient.muscleQuality === 'fair') compatibility += 5;

    // Activity level affects suitability
    if (input.patient.activityLevel === 'very_active') compatibility += 10;
    else if (input.patient.activityLevel === 'active') compatibility += 8;
    else if (input.patient.activityLevel === 'moderate') compatibility += 5;

    // DOF affects functionality
    const dof = input.limb.dof || 6;
    if (dof >= 12) compatibility += 5;

    compatibility = Math.min(100, compatibility);

    // Recommend control method
    const recommendedControl = this.selectControlMethod(
      input.patient.muscleQuality || 'good',
      input.patient.activityLevel || 'moderate'
    );

    // Estimate training time
    const estimatedTrainingTime =
      recommendedControl === 'PATTERN_RECOGNITION'
        ? 20
        : recommendedControl === 'MYOELECTRIC'
        ? 10
        : 5;

    // Calculate functionality score
    const functionalityScore = compatibility * 0.9;

    return {
      compatibility,
      recommendedControl,
      functionalityScore,
      estimatedTrainingTime,
      recommendations: [
        `Use ${recommendedControl} control method`,
        `${estimatedTrainingTime} hours initial training recommended`,
        compatibility > 85
          ? 'Excellent candidate for advanced features'
          : 'May require additional support during adaptation',
      ],
      challenges:
        compatibility < 70
          ? ['Limited muscle quality may affect control', 'Extended training period may be needed']
          : [],
    };
  }

  /**
   * Assess dexterity of a bionic hand/arm
   */
  assessDexterity(limb: { dof: number; gripPatterns: string[]; averageTransitionTime: number }): DexterityAssessment {
    const score =
      limb.gripPatterns.length * 5 +
      (100 / limb.averageTransitionTime) * 10 +
      limb.dof * 3;

    return {
      gripPatterns: limb.gripPatterns.length,
      transitionTime: limb.averageTransitionTime,
      forceResolution: BIONIC_LIMB_CONSTANTS.FORCE.RESOLUTION,
      independentFingers: limb.dof,
      manipulationScore: Math.min(100, score),
    };
  }

  /**
   * Manage power consumption
   */
  managePower(battery: BatteryState, activityLevel: 'low' | 'medium' | 'high'): PowerManagement {
    // Determine mode based on battery and activity
    let mode: PowerManagement['mode'];
    if (battery.level > 50 && activityLevel === 'high') {
      mode = 'performance';
    } else if (battery.level > 20) {
      mode = 'balanced';
    } else {
      mode = 'economy';
    }

    // Power saving mode
    const powerSaving = battery.level < BIONIC_LIMB_CONSTANTS.BATTERY.CRITICAL_LEVEL;

    return {
      mode,
      batteryLevel: battery.level,
      estimatedRuntime: battery.estimatedRuntime,
      powerSaving,
    };
  }

  /**
   * Generate comprehensive limb report
   */
  generateReport(
    device: BionicLimbDevice,
    patient: PatientInfo,
    calibration: ControlCalibration,
    battery: BatteryState
  ): LimbReport {
    const classification = this.classifyLimb({
      type: device.type,
      dof: device.dof,
      controlComplexity: 7,
      sensorCount: device.feedbackTypes.length,
      controlMethods: 2,
    });

    const maintenance = this.scheduleMaintenance(device.id, 'monthly');

    // Determine overall status
    let status: LimbReport['status'];
    if (calibration.accuracy >= 0.95 && battery.health === 'excellent') {
      status = 'excellent';
    } else if (calibration.accuracy >= 0.85 && battery.health !== 'replace') {
      status = 'good';
    } else if (calibration.accuracy >= 0.75) {
      status = 'fair';
    } else {
      status = 'needs_service';
    }

    const recommendations: string[] = [];
    if (calibration.accuracy < 0.9) {
      recommendations.push('Consider recalibration to improve control accuracy');
    }
    if (battery.health === 'fair' || battery.health === 'replace') {
      recommendations.push('Battery replacement recommended');
    }
    if (maintenance.overdueCount > 0) {
      recommendations.push(`${maintenance.overdueCount} maintenance tasks overdue`);
    }

    return {
      id: `RPT-${Date.now()}`,
      generatedAt: new Date(),
      device,
      patient,
      classification,
      calibration,
      battery,
      maintenance,
      performance: {
        controlAccuracy: calibration.accuracy,
        responseTime: 180,
        batteryRuntime: battery.estimatedRuntime / 60,
        functionalityScore: 90,
      },
      status,
      recommendations,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  private validateLimbInput(input: LimbClassificationInput): void {
    if (input.dof < 1 || input.dof > 20) {
      throw new BionicLimbError(
        BionicLimbErrorCode.CLASSIFICATION_INVALID_INPUT,
        'DOF must be between 1 and 20'
      );
    }

    if (input.controlComplexity < 1 || input.controlComplexity > 10) {
      throw new BionicLimbError(
        BionicLimbErrorCode.CLASSIFICATION_INVALID_INPUT,
        'Control complexity must be between 1 and 10'
      );
    }
  }

  private recommendControlMethods(
    type: LimbType,
    dof: number,
    category: LimbCategory
  ): ControlMethod[] {
    const methods: ControlMethod[] = [];

    // Myoelectric is standard for most prosthetics
    methods.push('MYOELECTRIC');

    // Pattern recognition for moderate to advanced
    if (category === 'Moderate' || category === 'Advanced') {
      methods.push('PATTERN_RECOGNITION');
    }

    // Hybrid for advanced systems
    if (category === 'Advanced' && dof >= 10) {
      methods.push('HYBRID');
    }

    // Body-powered as fallback
    if (['HAND', 'FOREARM'].includes(type)) {
      methods.push('BODY_POWERED');
    }

    return methods;
  }

  private getAmputationLevel(type: LimbType): string {
    const levels: Record<LimbType, string> = {
      FINGER: 'Partial hand',
      HAND: 'Wrist disarticulation',
      FOREARM: 'Below elbow (transradial)',
      UPPER_ARM: 'Above elbow (transhumeral)',
      FOOT: 'Partial foot / ankle',
      LOWER_LEG: 'Below knee (transtibial)',
      THIGH: 'Above knee (transfemoral)',
    };
    return levels[type];
  }

  private selectControlMethod(
    muscleQuality: string,
    activityLevel: string
  ): ControlMethod {
    if (muscleQuality === 'excellent' && activityLevel === 'very_active') {
      return 'PATTERN_RECOGNITION';
    } else if (muscleQuality === 'good' || muscleQuality === 'excellent') {
      return 'MYOELECTRIC';
    } else {
      return 'BODY_POWERED';
    }
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Classify limb (standalone)
 */
export function classifyLimb(input: LimbClassificationInput): LimbClassification {
  const sdk = new BionicLimbSDK();
  return sdk.classifyLimb(input);
}

/**
 * Calibrate control system (standalone)
 */
export function calibrateControl(
  limbId: string,
  userId: string,
  method: ControlMethod,
  trainingData?: unknown
): ControlCalibration {
  const sdk = new BionicLimbSDK();
  return sdk.calibrateControl(limbId, userId, method, trainingData);
}

/**
 * Provide feedback (standalone)
 */
export function provideFeedback(
  type: FeedbackType,
  intensity: number,
  location: string,
  duration?: number
): { success: boolean; message: string } {
  const sdk = new BionicLimbSDK();
  return sdk.provideFeedback(type, intensity, location, duration);
}

/**
 * Analyze gait (standalone)
 */
export function analyzeGait(limbId: string, duration: number, sensorData?: unknown[]): GaitAnalysis {
  const sdk = new BionicLimbSDK();
  return sdk.analyzeGait(limbId, duration, sensorData);
}

/**
 * Select grip (standalone)
 */
export function selectGrip(pattern: GripPattern, force: number, speed?: number): GripConfig {
  const sdk = new BionicLimbSDK();
  return sdk.selectGrip(pattern, force, speed);
}

/**
 * Schedule maintenance (standalone)
 */
export function scheduleMaintenance(
  limbId: string,
  interval?: MaintenanceInterval
): MaintenanceSchedule {
  const sdk = new BionicLimbSDK();
  return sdk.scheduleMaintenance(limbId, interval);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { BionicLimbSDK };
export default BionicLimbSDK;
