/**
 * WIA-AUG-006: Physical Enhancement - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Physical Augmentation Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Physical Domain Types
// ============================================================================

/**
 * Six primary physical domains for enhancement
 */
export enum PhysicalDomain {
  STRENGTH = 'STRENGTH',
  ENDURANCE = 'ENDURANCE',
  SPEED = 'SPEED',
  FLEXIBILITY = 'FLEXIBILITY',
  COORDINATION = 'COORDINATION',
  BALANCE = 'BALANCE',
}

/**
 * Enhancement technology categories
 */
export enum EnhancementTech {
  EXOSKELETON = 'EXOSKELETON',
  MUSCLE_AUG = 'MUSCLE_AUG',
  BONE_REINFORCE = 'BONE_REINFORCE',
  TENDON_ENHANCE = 'TENDON_ENHANCE',
  CARDIO_BOOST = 'CARDIO_BOOST',
}

/**
 * Fitness level classification
 */
export type FitnessLevel = 'sedentary' | 'low' | 'moderate' | 'high' | 'elite';

/**
 * Safety level classification
 */
export type SafetyLevel = 'basic' | 'standard' | 'high' | 'critical';

// ============================================================================
// Performance Metrics Types
// ============================================================================

/**
 * Strength performance metrics
 */
export interface StrengthMetrics {
  /** Maximum isometric force (N) */
  maxForce: number;

  /** Peak power output (W) */
  peakPower: number;

  /** Sustained force duration (s) */
  sustainedDuration: number;

  /** Load capacity (kg) */
  loadCapacity: number;

  /** One-rep max estimates */
  oneRepMax: {
    benchPress: number;
    squat: number;
    deadlift: number;
  };
}

/**
 * Endurance performance metrics
 */
export interface EnduranceMetrics {
  /** VO2 max (ml/kg/min) */
  vo2Max: number;

  /** Lactate threshold (% VO2 max) */
  lactateThreshold: number;

  /** Sustained activity duration (min) */
  duration: number;

  /** Heart rate recovery time (s) */
  hrRecovery: number;

  /** Resting heart rate (bpm) */
  restingHR: number;
}

/**
 * Speed performance metrics
 */
export interface SpeedMetrics {
  /** Sprint velocity (m/s) */
  velocity: number;

  /** Acceleration (m/s²) */
  acceleration: number;

  /** Reaction time (ms) */
  reactionTime: number;

  /** Movement frequency (Hz) */
  cadence: number;

  /** Sprint times */
  sprintTimes: {
    tenMeter: number;
    twentyMeter: number;
    fortyMeter: number;
  };
}

/**
 * Flexibility performance metrics
 */
export interface FlexibilityMetrics {
  /** Range of motion measurements (degrees) */
  rangeOfMotion: {
    shoulder: { flexion: number; extension: number; abduction: number };
    hip: { flexion: number; extension: number; abduction: number };
    spine: { flexion: number; extension: number; lateralBend: number };
  };

  /** Sit-and-reach test (cm) */
  sitAndReach: number;

  /** Stiffness index */
  stiffnessIndex: number;

  /** Joint mobility score (0-100) */
  mobilityScore: number;
}

/**
 * Coordination performance metrics
 */
export interface CoordinationMetrics {
  /** Target precision (mm) */
  precision: number;

  /** Accuracy percentage (0-100) */
  accuracy: number;

  /** Movement smoothness (jerk score, m/s³) */
  smoothness: number;

  /** Error rate (%) */
  errorRate: number;

  /** Task completion time (s) */
  completionTime: number;
}

/**
 * Balance performance metrics
 */
export interface BalanceMetrics {
  /** Stability score (0-100) */
  stabilityScore: number;

  /** Center of pressure sway area (cm²) */
  swayArea: number;

  /** Sway velocity (cm/s) */
  swayVelocity: number;

  /** Single-leg stand duration (s) */
  singleLegStand: number;

  /** Y-Balance composite score (cm) */
  yBalanceScore: number;
}

/**
 * Composite performance metrics for all domains
 */
export interface PerformanceMetrics {
  strength: StrengthMetrics;
  endurance: EnduranceMetrics;
  speed: SpeedMetrics;
  flexibility: FlexibilityMetrics;
  coordination: CoordinationMetrics;
  balance: BalanceMetrics;
}

// ============================================================================
// Baseline Assessment Types
// ============================================================================

/**
 * User profile for baseline assessment
 */
export interface UserProfile {
  /** User identifier */
  id: string;

  /** Age (years) */
  age: number;

  /** Weight (kg) */
  weight: number;

  /** Height (cm) */
  height: number;

  /** Fitness level */
  fitnessLevel: FitnessLevel;

  /** Medical history */
  medicalHistory: string[];

  /** Known allergies */
  allergies: string[];

  /** Current medications */
  medications: string[];
}

/**
 * Baseline assessment result
 */
export interface BaselineAssessment {
  /** User profile */
  user: UserProfile;

  /** Assessment date */
  assessmentDate: Date;

  /** Performance metrics */
  metrics: PerformanceMetrics;

  /** Overall fitness score (0-100) */
  fitnessScore: number;

  /** Assessor ID */
  assessorId: string;

  /** Medical clearance status */
  medicalClearance: boolean;

  /** Certification level */
  certificationLevel: 'Level1' | 'Level2' | 'Level3';
}

// ============================================================================
// Enhancement Configuration Types
// ============================================================================

/**
 * Enhancement target configuration
 */
export interface EnhancementTarget {
  /** Target physical domain */
  domain: PhysicalDomain;

  /** Enhancement technology */
  technology: EnhancementTech;

  /** Target enhancement factor (1.2 - 5.0) */
  targetFactor: number;

  /** Baseline metrics for the domain */
  baseline: number;

  /** Safety level */
  safetyLevel: SafetyLevel;

  /** Maximum allowed load (kg) */
  maxLoad?: number;

  /** Session duration limit (minutes) */
  durationLimit?: number;
}

/**
 * Enhancement result
 */
export interface EnhancementResult {
  /** Domain being enhanced */
  domain: PhysicalDomain;

  /** Technology used */
  technology: EnhancementTech;

  /** Achieved enhancement factor */
  enhancementFactor: number;

  /** Baseline performance */
  baseline: number;

  /** Enhanced performance */
  enhanced: number;

  /** Safe load capacity (kg) */
  safeLoad: number;

  /** Maximum force output (N) */
  maxForce: number;

  /** Power output (W) */
  powerOutput: number;

  /** Estimated duration limit (minutes) */
  durationLimit: number;

  /** Safety warnings */
  warnings: string[];
}

// ============================================================================
// Load Monitoring Types
// ============================================================================

/**
 * Load status classification
 */
export type LoadStatus = 'safe' | 'caution' | 'warning' | 'critical';

/**
 * Load monitoring input
 */
export interface LoadMonitorInput {
  /** Current load (kg) */
  currentLoad: number;

  /** Maximum safe capacity (kg) */
  maxCapacity: number;

  /** Duration of load (seconds) */
  duration: number;

  /** Number of repetitions */
  repetitions?: number;

  /** Joint being monitored */
  joint?: string;
}

/**
 * Load monitoring result
 */
export interface LoadMonitorResult {
  /** Load status classification */
  status: LoadStatus;

  /** Load percentage (0-100) */
  loadPercentage: number;

  /** Safe remaining duration (seconds) */
  remainingDuration: number;

  /** Recommendation */
  recommendation: string;

  /** Warnings */
  warnings: string[];

  /** Cumulative load index */
  cumulativeLoadIndex: number;
}

// ============================================================================
// Fatigue Monitoring Types
// ============================================================================

/**
 * Fatigue level classification
 */
export type FatigueLevel = 'low' | 'moderate' | 'high' | 'critical';

/**
 * Fatigue metrics
 */
export interface FatigueMetrics {
  /** Physical fatigue (0-100) */
  physicalFatigue: number;

  /** Mental fatigue (0-100) */
  mentalFatigue: number;

  /** Neuromuscular fatigue (0-100) */
  neuromuscularFatigue: number;

  /** Cardiovascular fatigue (0-100) */
  cardiovascularFatigue: number;

  /** Composite fatigue score (0-100) */
  compositeFatigue: number;

  /** Predictive fatigue index (0-1) */
  predictiveFatigueIndex: number;
}

/**
 * Fatigue monitoring input
 */
export interface FatigueMonitorInput {
  /** Session duration (seconds) */
  sessionDuration: number;

  /** Activity intensity (0-100) */
  intensity: number;

  /** Current heart rate (bpm) */
  heartRate: number;

  /** Maximum heart rate (bpm) */
  maxHeartRate: number;

  /** Force decline from peak (%) */
  forceDecline: number;

  /** Cumulative load index */
  cumulativeLoad: number;
}

/**
 * Fatigue monitoring result
 */
export interface FatigueMonitorResult {
  /** Fatigue level */
  level: FatigueLevel;

  /** Detailed fatigue metrics */
  metrics: FatigueMetrics;

  /** Time to critical fatigue (seconds) */
  timeToCritical: number;

  /** Recommended action */
  action: string;

  /** Rest duration required (minutes) */
  restRequired: number;

  /** Warnings */
  warnings: string[];
}

// ============================================================================
// Injury Prevention Types
// ============================================================================

/**
 * Injury risk level
 */
export type InjuryRiskLevel = 'minimal' | 'low' | 'moderate' | 'high' | 'critical';

/**
 * Injury risk assessment
 */
export interface InjuryRisk {
  /** Overexertion risk (0-100) */
  overexertionRisk: number;

  /** Overuse risk (0-100) */
  overuseRisk: number;

  /** Mechanical failure risk (0-100) */
  mechanicalFailureRisk: number;

  /** Cardiovascular risk (0-100) */
  cardiovascularRisk: number;

  /** Composite risk (0-100) */
  compositeRisk: number;

  /** Risk level classification */
  riskLevel: InjuryRiskLevel;
}

/**
 * Injury prevention protocol
 */
export interface InjuryPreventionProtocol {
  /** Warm-up requirements */
  warmup: {
    duration: number; // minutes
    intensity: number; // 0-100
    exercises: string[];
  };

  /** Load progression limits */
  loadProgression: {
    dailyIncrease: number; // percentage
    weeklyIncrease: number; // percentage
    monthlyIncrease: number; // percentage
  };

  /** Rest requirements */
  restRequirements: {
    minRestBetweenSessions: number; // hours
    mandatoryBreaks: number; // every X minutes
    breakDuration: number; // minutes
  };

  /** Joint-specific limits */
  jointLimits: {
    [joint: string]: {
      maxAngle: number; // degrees
      maxForce: number; // N
      maxRepetitions: number; // per day
    };
  };

  /** Emergency shutdown triggers */
  shutdownTriggers: string[];
}

// ============================================================================
// Recovery Optimization Types
// ============================================================================

/**
 * Recovery status
 */
export type RecoveryStatus = 'not_ready' | 'partially_ready' | 'ready' | 'fully_ready';

/**
 * Recovery modality
 */
export enum RecoveryModality {
  ACTIVE_RECOVERY = 'ACTIVE_RECOVERY',
  PASSIVE_RECOVERY = 'PASSIVE_RECOVERY',
  CONTRAST_THERAPY = 'CONTRAST_THERAPY',
  COMPRESSION = 'COMPRESSION',
  CRYOTHERAPY = 'CRYOTHERAPY',
  MASSAGE = 'MASSAGE',
}

/**
 * Nutrition plan
 */
export interface NutritionPlan {
  /** Carbohydrates (g) */
  carbs: number;

  /** Protein (g) */
  protein: number;

  /** Fats (g) */
  fats: number;

  /** Hydration (liters) */
  hydration: number;

  /** Timing recommendations */
  timing: string[];

  /** Supplements */
  supplements: string[];
}

/**
 * Recovery protocol
 */
export interface RecoveryProtocol {
  /** Activity level during recovery */
  activityLevel: 'rest' | 'active' | 'light' | 'moderate';

  /** Duration (minutes) */
  duration: number;

  /** Recovery modalities to use */
  modalities: RecoveryModality[];

  /** Nutrition plan */
  nutritionPlan: NutritionPlan;

  /** Sleep target (hours) */
  sleepTarget: number;

  /** Expected recovery percentage (0-100) */
  expectedRecovery: number;

  /** Recommendations */
  recommendations: string[];
}

/**
 * Recovery assessment
 */
export interface RecoveryAssessment {
  /** Current recovery percentage (0-100) */
  currentRecovery: number;

  /** Recovery score (0-100) */
  recoveryScore: number;

  /** Time to full recovery (minutes) */
  timeToFullRecovery: number;

  /** Ready for activity */
  readyForActivity: boolean;

  /** Recovery status */
  status: RecoveryStatus;

  /** Sleep quality (0-100) */
  sleepQuality: number;

  /** Heart rate variability score (0-100) */
  hrvScore: number;

  /** Muscle soreness (0-10) */
  muscleSoreness: number;

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Session Tracking Types
// ============================================================================

/**
 * Training session record
 */
export interface TrainingSession {
  /** Session ID */
  id: string;

  /** User ID */
  userId: string;

  /** Start time */
  startTime: Date;

  /** End time */
  endTime: Date;

  /** Duration (seconds) */
  duration: number;

  /** Domain trained */
  domain: PhysicalDomain;

  /** Technology used */
  technology: EnhancementTech;

  /** Enhancement factor */
  enhancementFactor: number;

  /** Load profile */
  loadProfile: {
    maxLoad: number; // kg
    avgLoad: number; // kg
    cumulativeLoadIndex: number;
  };

  /** Fatigue tracking */
  fatigueTracking: {
    initialPFI: number;
    maxPFI: number;
    finalPFI: number;
  };

  /** Performance metrics */
  performance: Partial<PerformanceMetrics>;

  /** Alerts and incidents */
  alerts: {
    cautionCount: number;
    criticalCount: number;
    emergencyShutdown: boolean;
  };

  /** Notes */
  notes: string;
}

/**
 * Activity log entry
 */
export interface ActivityLogEntry {
  /** Timestamp */
  timestamp: Date;

  /** Event type */
  eventType: 'session_start' | 'session_end' | 'alert' | 'shutdown' | 'recovery' | 'baseline_assessment';

  /** Domain (if applicable) */
  domain?: PhysicalDomain;

  /** Enhancement factor (if applicable) */
  enhancementFactor?: number;

  /** Load (kg, if applicable) */
  load?: number;

  /** Fatigue level (if applicable) */
  fatigueLevel?: FatigueLevel;

  /** Description */
  description: string;

  /** Additional data */
  metadata?: Record<string, unknown>;
}

// ============================================================================
// Performance Report Types
// ============================================================================

/**
 * Performance report
 */
export interface PerformanceReport {
  /** Report ID */
  id: string;

  /** User ID */
  userId: string;

  /** Report period */
  period: {
    start: Date;
    end: Date;
  };

  /** Baseline assessment */
  baseline: BaselineAssessment;

  /** Current performance */
  current: PerformanceMetrics;

  /** Enhancement history */
  enhancementHistory: {
    domain: PhysicalDomain;
    avgFactor: number;
    maxFactor: number;
    sessionCount: number;
  }[];

  /** Training summary */
  trainingSummary: {
    totalSessions: number;
    totalDuration: number; // hours
    totalLoad: number; // cumulative kg
    avgSessionDuration: number; // minutes
  };

  /** Safety summary */
  safetySummary: {
    totalAlerts: number;
    criticalAlerts: number;
    emergencyShutdowns: number;
    injuryIncidents: number;
  };

  /** Recovery summary */
  recoverySummary: {
    avgRecoveryScore: number;
    avgSleepHours: number;
    avgRestDays: number;
  };

  /** Progress analysis */
  progress: {
    domain: PhysicalDomain;
    baseline: number;
    current: number;
    improvement: number; // percentage
  }[];

  /** Recommendations */
  recommendations: string[];

  /** Generated date */
  generatedAt: Date;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Physical enhancement constants
 */
export const ENHANCEMENT_CONSTANTS = {
  /** Enhancement factor limits */
  ENHANCEMENT_FACTORS: {
    MIN: 1.2,
    MAX: 5.0,
    SAFE_MAX: 3.0,
  },

  /** Load status thresholds */
  LOAD_THRESHOLDS: {
    SAFE: 0.7,
    CAUTION: 0.85,
    WARNING: 0.95,
    CRITICAL: 1.0,
  },

  /** Fatigue thresholds */
  FATIGUE_THRESHOLDS: {
    LOW: 0.5,
    MODERATE: 0.7,
    HIGH: 0.85,
    CRITICAL: 0.95,
  },

  /** Safety margins */
  SAFETY_MARGINS: {
    BASIC: 0.9,
    STANDARD: 0.8,
    HIGH: 0.7,
  },

  /** Recovery targets */
  RECOVERY_TARGETS: {
    MIN_SLEEP: 7, // hours
    MAX_SLEEP: 9, // hours
    MIN_SCORE: 75, // 0-100
    READY_THRESHOLD: 75, // 0-100
  },

  /** Session limits */
  SESSION_LIMITS: {
    MAX_DURATION: 480, // minutes (8 hours)
    MAX_DAILY_SESSIONS: 2,
    MIN_REST_BETWEEN: 240, // minutes (4 hours)
  },

  /** Enhancement ranges by technology */
  TECH_RANGES: {
    EXOSKELETON: { min: 2.0, max: 5.0 },
    MUSCLE_AUG: { min: 1.3, max: 2.0 },
    BONE_REINFORCE: { min: 1.5, max: 2.5 },
    TENDON_ENHANCE: { min: 1.3, max: 2.0 },
    CARDIO_BOOST: { min: 1.3, max: 2.5 },
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Physical enhancement error codes
 */
export enum PhysicalEnhancementErrorCode {
  INVALID_ENHANCEMENT_FACTOR = 'PE001',
  LOAD_EXCEEDED = 'PE002',
  FATIGUE_CRITICAL = 'PE003',
  INJURY_RISK_HIGH = 'PE004',
  BASELINE_NOT_ASSESSED = 'PE005',
  MEDICAL_CLEARANCE_REQUIRED = 'PE006',
  RECOVERY_INSUFFICIENT = 'PE007',
  SESSION_LIMIT_EXCEEDED = 'PE008',
  SAFETY_VIOLATION = 'PE009',
  EQUIPMENT_MALFUNCTION = 'PE010',
}

/**
 * Physical enhancement error class
 */
export class PhysicalEnhancementError extends Error {
  constructor(
    public code: PhysicalEnhancementErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'PhysicalEnhancementError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  FitnessLevel,
  SafetyLevel,
  StrengthMetrics,
  EnduranceMetrics,
  SpeedMetrics,
  FlexibilityMetrics,
  CoordinationMetrics,
  BalanceMetrics,
  PerformanceMetrics,
  UserProfile,
  BaselineAssessment,
  EnhancementTarget,
  EnhancementResult,
  LoadStatus,
  LoadMonitorInput,
  LoadMonitorResult,
  FatigueLevel,
  FatigueMetrics,
  FatigueMonitorInput,
  FatigueMonitorResult,
  InjuryRiskLevel,
  InjuryRisk,
  InjuryPreventionProtocol,
  RecoveryStatus,
  NutritionPlan,
  RecoveryProtocol,
  RecoveryAssessment,
  TrainingSession,
  ActivityLogEntry,
  PerformanceReport,
};

export {
  PhysicalDomain,
  EnhancementTech,
  RecoveryModality,
  ENHANCEMENT_CONSTANTS,
  PhysicalEnhancementErrorCode,
  PhysicalEnhancementError,
};
