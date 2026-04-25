/**
 * WIA-AUG-005: Cognitive Enhancement - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Cognitive Enhancement Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Cognitive Domain Types
// ============================================================================

/**
 * Primary cognitive domains
 */
export type CognitiveDomain =
  | 'MEMORY'
  | 'ATTENTION'
  | 'REASONING'
  | 'CREATIVITY'
  | 'LANGUAGE'
  | 'EXECUTIVE'
  | 'SPATIAL';

/**
 * Memory sub-domains
 */
export type MemoryType =
  | 'working'
  | 'longterm'
  | 'episodic'
  | 'semantic'
  | 'procedural';

/**
 * Attention sub-domains
 */
export type AttentionType =
  | 'sustained'
  | 'selective'
  | 'divided'
  | 'switching';

// ============================================================================
// Enhancement Method Types
// ============================================================================

/**
 * Enhancement methods
 */
export type EnhancementMethod =
  | 'PHARMACOLOGICAL'
  | 'ELECTRICAL'
  | 'COMPUTATIONAL'
  | 'TRAINING'
  | 'HYBRID';

/**
 * Pharmacological agent types
 */
export type PharmacologicalAgent =
  | 'modafinil'
  | 'methylphenidate'
  | 'piracetam'
  | 'caffeine'
  | 'other';

/**
 * Electrical stimulation techniques
 */
export type ElectricalTechnique =
  | 'tDCS'
  | 'tACS'
  | 'tRNS'
  | 'TMS';

/**
 * Computational enhancement types
 */
export type ComputationalType =
  | 'memory_augmentation'
  | 'attention_optimization'
  | 'decision_support'
  | 'learning_acceleration'
  | 'creative_assistance';

/**
 * Training program types
 */
export type TrainingType =
  | 'working_memory'
  | 'attention_training'
  | 'reasoning_exercises'
  | 'creativity_training'
  | 'executive_training';

// ============================================================================
// Performance Metrics Types
// ============================================================================

/**
 * Cognitive performance metrics
 */
export interface CognitiveMetrics {
  /** Baseline performance before enhancement */
  baseline: number;

  /** Current performance level */
  current: number;

  /** Enhancement ratio (current - baseline) / baseline */
  enhancementRatio: number;

  /** Performance percentile (0-100) */
  percentile: number;

  /** Standard score (mean=100, SD=15) */
  standardScore: number;

  /** Confidence interval (95%) */
  confidence95: [number, number];
}

/**
 * Domain-specific score
 */
export interface DomainScore {
  /** Domain identifier */
  domain: CognitiveDomain;

  /** Raw score */
  rawScore: number;

  /** Normalized score (0-100) */
  normalizedScore: number;

  /** Enhancement metrics */
  metrics: CognitiveMetrics;

  /** Sub-domain scores */
  subDomains?: Record<string, number>;

  /** Measurement timestamp */
  timestamp: Date;
}

/**
 * Comprehensive cognitive assessment
 */
export interface CognitiveAssessment {
  /** Assessment identifier */
  assessmentId: string;

  /** User identifier */
  userId: string;

  /** Assessment date */
  date: Date;

  /** Overall cognitive index (weighted average) */
  cognitiveIndex: number;

  /** Domain-specific scores */
  domainScores: Record<CognitiveDomain, DomainScore>;

  /** IQ estimate */
  iqEstimate?: {
    full: number;
    verbal: number;
    performance: number;
    processing: number;
  };

  /** Assessment type */
  type: 'baseline' | 'ongoing' | 'endpoint';

  /** Duration of assessment (minutes) */
  duration: number;
}

// ============================================================================
// Cognitive Load Types
// ============================================================================

/**
 * Cognitive load status
 */
export interface CognitiveLoadStatus {
  /** Current cognitive load index (0-1) */
  currentLoad: number;

  /** Load category */
  category: 'low' | 'moderate' | 'high' | 'critical';

  /** Component loads */
  components: {
    taskDemand: number;
    attentionAllocation: number;
    memoryLoad: number;
    processingSpeed: number;
  };

  /** Available cognitive resources (0-1) */
  availableResources: number;

  /** Recommendation */
  recommendation: 'continue' | 'reduce_load' | 'take_break' | 'stop_session';

  /** Timestamp */
  timestamp: Date;
}

/**
 * Cognitive load thresholds
 */
export interface LoadThresholds {
  /** Safe operating range */
  safeRange: [number, number];

  /** Warning threshold */
  warning: number;

  /** Critical threshold */
  critical: number;

  /** Emergency threshold */
  emergency: number;
}

// ============================================================================
// Fatigue Management Types
// ============================================================================

/**
 * Fatigue level
 */
export type FatigueLevel = 'low' | 'moderate' | 'high' | 'critical';

/**
 * Fatigue action
 */
export type FatigueAction =
  | 'continue'
  | 'suggest_break'
  | 'mandatory_break'
  | 'end_session';

/**
 * Fatigue indicators
 */
export interface FatigueIndicators {
  /** Performance decline (percentage) */
  performanceDecline: number;

  /** Error rate increase (percentage) */
  errorRateIncrease: number;

  /** Response time increase (percentage) */
  responseTimeIncrease: number;

  /** Physiological indicators */
  physiological?: {
    heartRateElevation: number;
    pupilDilation: number;
    blinkRate: number;
  };

  /** Self-reported fatigue (1-10) */
  selfReportedFatigue: number;

  /** Motivation level (1-10) */
  motivationLevel: number;

  /** Composite fatigue score (0-100) */
  fatigueScore: number;
}

/**
 * Fatigue assessment result
 */
export interface FatigueAssessment {
  /** Current fatigue level */
  level: FatigueLevel;

  /** Indicators */
  indicators: FatigueIndicators;

  /** Recommended action */
  action: FatigueAction;

  /** Recommended break duration (minutes) */
  recommendedBreak: number;

  /** Time until next check (minutes) */
  nextCheckIn: number;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Enhancement Session Types
// ============================================================================

/**
 * Enhancement request
 */
export interface EnhancementRequest {
  /** User identifier */
  userId: string;

  /** Target cognitive domain(s) */
  targetDomains: CognitiveDomain[];

  /** Enhancement method */
  method: EnhancementMethod;

  /** Target enhancement ratio (0.0-0.8) */
  targetRatio: number;

  /** Session duration (minutes) */
  duration: number;

  /** Intensity level (0.0-1.0) */
  intensity?: number;

  /** Additional parameters */
  parameters?: Record<string, unknown>;
}

/**
 * Enhancement session
 */
export interface EnhancementSession {
  /** Session identifier */
  sessionId: string;

  /** User identifier */
  userId: string;

  /** Start timestamp */
  startTime: Date;

  /** End timestamp (if ended) */
  endTime?: Date;

  /** Enhancement method */
  method: EnhancementMethod;

  /** Target domain */
  targetDomain: CognitiveDomain;

  /** Target enhancement ratio */
  targetRatio: number;

  /** Current enhancement ratio */
  currentRatio: number;

  /** Session duration (minutes) */
  duration: number;

  /** Intensity level */
  intensity: number;

  /** Current status */
  status: 'active' | 'paused' | 'completed' | 'terminated';

  /** Performance data */
  performanceData: PerformanceIndicators[];

  /** Cognitive load history */
  loadHistory: CognitiveLoadStatus[];

  /** Fatigue assessments */
  fatigueAssessments: FatigueAssessment[];
}

/**
 * Enhancement result
 */
export interface EnhancementResult {
  /** Session identifier */
  sessionId: string;

  /** Success status */
  success: boolean;

  /** Final enhancement ratio achieved */
  achievedRatio: number;

  /** Domain-specific improvements */
  improvements: Record<CognitiveDomain, number>;

  /** Session summary */
  summary: {
    duration: number;
    averageLoad: number;
    peakLoad: number;
    fatigueLevel: FatigueLevel;
    adverseEvents: string[];
  };

  /** Recommendations */
  recommendations: string[];

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Performance Monitoring Types
// ============================================================================

/**
 * Real-time performance indicators
 */
export interface PerformanceIndicators {
  /** Timestamp */
  timestamp: Date;

  /** Task accuracy (0-1) */
  accuracyRate: number;

  /** Response time (milliseconds) */
  responseTime: number;

  /** Task completion rate (0-1) */
  taskCompletionRate: number;

  /** Error rate (0-1) */
  errorRate: number;

  /** Perseveration index (repetitive errors) */
  perseverationIndex: number;

  /** Novelty score (creative/novel responses) */
  noveltyScore: number;

  /** Physiological correlates */
  physiological?: {
    heartRateVariability: number;
    eyeTracking?: {
      fixationDuration: number;
      saccadeVelocity: number;
      blinkRate: number;
    };
  };

  /** Subjective metrics */
  subjective?: {
    perceivedDifficulty: number; // 1-10
    perceivedPerformance: number; // 1-10
    mentalEffort: number; // 1-10
  };
}

/**
 * Performance measurement
 */
export interface PerformanceMeasurement {
  /** Measurement identifier */
  measurementId: string;

  /** User identifier */
  userId: string;

  /** Session identifier (if during session) */
  sessionId?: string;

  /** Domain being measured */
  domain: CognitiveDomain;

  /** Performance indicators */
  indicators: PerformanceIndicators;

  /** Comparison to baseline */
  baselineComparison: {
    baseline: number;
    current: number;
    improvement: number;
    enhancementRatio: number;
  };

  /** Cognitive load at time of measurement */
  cognitiveLoad: number;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Baseline Assessment Types
// ============================================================================

/**
 * Baseline assessment request
 */
export interface BaselineRequest {
  /** User identifier */
  userId: string;

  /** Domains to assess (default: all) */
  domains?: CognitiveDomain[];

  /** Assessment type */
  type: 'comprehensive' | 'rapid';

  /** Include demographic data */
  includeDemographics?: boolean;
}

/**
 * Baseline assessment result
 */
export interface BaselineAssessment {
  /** Assessment identifier */
  assessmentId: string;

  /** User identifier */
  userId: string;

  /** Assessment date */
  date: Date;

  /** Demographic information */
  demographics?: {
    age: number;
    education: number; // years
    occupation?: string;
  };

  /** Cognitive assessment */
  cognitive: CognitiveAssessment;

  /** Enhancement potential per domain */
  enhancementPotential: Record<CognitiveDomain, number>;

  /** Recommended methods */
  recommendedMethods: EnhancementMethod[];

  /** Contraindications */
  contraindications: string[];

  /** Medical clearance required */
  medicalClearanceRequired: boolean;
}

// ============================================================================
// Decision Support Types
// ============================================================================

/**
 * Problem definition for decision support
 */
export interface Problem {
  /** Problem identifier */
  problemId: string;

  /** Problem description */
  description: string;

  /** Problem type */
  type: 'analytical' | 'creative' | 'strategic' | 'operational';

  /** Complexity level (1-10) */
  complexity: number;

  /** Time constraint (minutes) */
  timeConstraint?: number;

  /** Stakeholders */
  stakeholders?: string[];

  /** Constraints */
  constraints?: string[];
}

/**
 * Decision support request
 */
export interface DecisionSupportRequest {
  /** Session identifier */
  sessionId: string;

  /** Problem to solve */
  problem: Problem;

  /** Desired support level */
  supportLevel: 'minimal' | 'moderate' | 'comprehensive';

  /** Autonomy level */
  autonomy: 'full-human' | 'assisted' | 'collaborative' | 'delegated';
}

/**
 * Decision recommendation
 */
export interface Recommendation {
  /** Recommendation identifier */
  id: string;

  /** Option description */
  option: string;

  /** Confidence score (0-1) */
  confidence: number;

  /** Expected outcome */
  expectedOutcome: string;

  /** Risks */
  risks: string[];

  /** Benefits */
  benefits: string[];

  /** Rationale */
  rationale: string;
}

/**
 * Decision support result
 */
export interface DecisionSupport {
  /** Request identifier */
  requestId: string;

  /** Problem analysis */
  analysis: {
    decomposition: string[];
    keyFactors: string[];
    uncertainties: string[];
  };

  /** Generated recommendations */
  recommendations: Recommendation[];

  /** Cognitive augmentation applied */
  augmentation: {
    domains: CognitiveDomain[];
    methods: string[];
    enhancementLevel: number;
  };

  /** Decision quality estimate */
  qualityEstimate: {
    accuracy: number;
    completeness: number;
    robustness: number;
    confidence: number;
  };

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Safety and Constraints Types
// ============================================================================

/**
 * Safety profile
 */
export type SafetyProfile = 'conservative' | 'moderate' | 'aggressive';

/**
 * Safety thresholds
 */
export interface SafetyThresholds {
  /** Maximum enhancement ratio by domain */
  maxEnhancementRatio: Record<CognitiveDomain, number>;

  /** Maximum rate of enhancement (per week) */
  maxEnhancementRate: number;

  /** Cognitive load thresholds */
  cognitiveLoad: LoadThresholds;

  /** Fatigue thresholds */
  fatigueThresholds: {
    warning: number;
    critical: number;
  };

  /** Session duration limits (minutes) */
  maxSessionDuration: number;

  /** Daily enhancement limit (minutes) */
  maxDailyDuration: number;
}

/**
 * Contraindications
 */
export interface Contraindications {
  /** Absolute contraindications (must not use) */
  absolute: string[];

  /** Relative contraindications (caution required) */
  relative: string[];

  /** Method-specific contraindications */
  methodSpecific: Partial<Record<EnhancementMethod, string[]>>;
}

// ============================================================================
// Protocol Types
// ============================================================================

/**
 * Enhancement protocol
 */
export interface EnhancementProtocol {
  /** Protocol identifier */
  protocolId: string;

  /** Protocol name */
  name: string;

  /** Target domain */
  targetDomain: CognitiveDomain;

  /** Enhancement method */
  method: EnhancementMethod;

  /** Duration (weeks) */
  duration: number;

  /** Schedule */
  schedule: {
    sessionsPerWeek: number;
    sessionDuration: number; // minutes
    restDays: number[];
  };

  /** Phases */
  phases: ProtocolPhase[];

  /** Monitoring requirements */
  monitoring: {
    daily?: string[];
    weekly?: string[];
    monthly?: string[];
  };

  /** Safety parameters */
  safety: SafetyThresholds;

  /** Expected outcomes */
  expectedOutcomes: {
    targetER: number;
    timeToTarget: number; // days
    sustainabilityIndex: number;
  };
}

/**
 * Protocol phase
 */
export interface ProtocolPhase {
  /** Phase name */
  name: string;

  /** Phase duration (weeks) */
  duration: number;

  /** Activities */
  activities: string[];

  /** Target enhancement ratio */
  targetER: number;

  /** Intensity level */
  intensity: number;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Cognitive enhancement constants
 */
export const COGNITIVE_CONSTANTS = {
  /** Default enhancement ratio targets by domain */
  DEFAULT_ENHANCEMENT_TARGETS: {
    MEMORY: 0.4,
    ATTENTION: 0.5,
    REASONING: 0.3,
    CREATIVITY: 0.4,
    LANGUAGE: 0.3,
    EXECUTIVE: 0.4,
    SPATIAL: 0.3,
  },

  /** Maximum safe enhancement ratios */
  MAX_ENHANCEMENT_RATIOS: {
    MEMORY: 0.6,
    ATTENTION: 0.8,
    REASONING: 0.5,
    CREATIVITY: 0.7,
    LANGUAGE: 0.5,
    EXECUTIVE: 0.6,
    SPATIAL: 0.5,
  },

  /** Cognitive load thresholds */
  LOAD_THRESHOLDS: {
    SAFE_MIN: 0.3,
    SAFE_MAX: 0.7,
    WARNING: 0.7,
    CRITICAL: 0.9,
    EMERGENCY: 0.95,
  },

  /** Fatigue thresholds */
  FATIGUE_THRESHOLDS: {
    LOW: 30,
    MODERATE: 60,
    HIGH: 80,
    CRITICAL: 90,
  },

  /** Domain interaction weights */
  DOMAIN_WEIGHTS: {
    MEMORY: 0.18,
    ATTENTION: 0.16,
    REASONING: 0.18,
    CREATIVITY: 0.12,
    LANGUAGE: 0.12,
    EXECUTIVE: 0.16,
    SPATIAL: 0.08,
  },

  /** Session parameters */
  SESSION_PARAMS: {
    MIN_DURATION: 10, // minutes
    MAX_DURATION: 120, // minutes
    DEFAULT_DURATION: 45, // minutes
    BREAK_INTERVAL: 45, // minutes
    BREAK_DURATION: 10, // minutes
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Cognitive enhancement error codes
 */
export enum CognitiveErrorCode {
  INVALID_DOMAIN = 'CE001',
  INVALID_METHOD = 'CE002',
  ENHANCEMENT_LIMIT_EXCEEDED = 'CE003',
  COGNITIVE_LOAD_CRITICAL = 'CE004',
  FATIGUE_CRITICAL = 'CE005',
  CONTRAINDICATION_DETECTED = 'CE006',
  SESSION_EXPIRED = 'CE007',
  BASELINE_REQUIRED = 'CE008',
  SAFETY_THRESHOLD_EXCEEDED = 'CE009',
  INVALID_PARAMETERS = 'CE010',
}

/**
 * Cognitive enhancement error
 */
export class CognitiveError extends Error {
  constructor(
    public code: CognitiveErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'CognitiveError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  CognitiveDomain,
  MemoryType,
  AttentionType,
  EnhancementMethod,
  PharmacologicalAgent,
  ElectricalTechnique,
  ComputationalType,
  TrainingType,
  CognitiveMetrics,
  DomainScore,
  CognitiveAssessment,
  CognitiveLoadStatus,
  LoadThresholds,
  FatigueLevel,
  FatigueAction,
  FatigueIndicators,
  FatigueAssessment,
  EnhancementRequest,
  EnhancementSession,
  EnhancementResult,
  PerformanceIndicators,
  PerformanceMeasurement,
  BaselineRequest,
  BaselineAssessment,
  Problem,
  DecisionSupportRequest,
  Recommendation,
  DecisionSupport,
  SafetyProfile,
  SafetyThresholds,
  Contraindications,
  EnhancementProtocol,
  ProtocolPhase,
};

export { COGNITIVE_CONSTANTS, CognitiveErrorCode, CognitiveError };
