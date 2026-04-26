/**
 * WIA-AUG-016: Memory Enhancement - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Cognitive Augmentation Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Memory Type Classifications
// ============================================================================

/**
 * Primary memory system types
 */
export type MemoryType =
  | 'WORKING'
  | 'SHORT_TERM'
  | 'LONG_TERM'
  | 'EPISODIC'
  | 'SEMANTIC'
  | 'PROCEDURAL'
  | 'AUTOBIOGRAPHICAL';

/**
 * Memory enhancement methods
 */
export type EnhancementMethod =
  | 'PHARMACOLOGICAL'
  | 'ELECTRICAL'
  | 'NEURAL_IMPLANT'
  | 'COMPUTATIONAL'
  | 'TRAINING'
  | 'HYBRID';

/**
 * Memory processing phases
 */
export type MemoryPhase = 'ENCODING' | 'CONSOLIDATION' | 'STORAGE' | 'RETRIEVAL';

/**
 * Sleep stages for consolidation
 */
export type SleepStage = 'N1' | 'N2' | 'N3' | 'REM';

// ============================================================================
// Memory Metrics
// ============================================================================

/**
 * Memory capacity units
 */
export type CapacityUnit = 'bits' | 'items' | 'patterns' | 'megabytes' | 'gigabytes';

/**
 * Comprehensive memory metrics
 */
export interface MemoryMetrics {
  /** Storage capacity in specified unit */
  capacity: number;

  /** Unit of measurement */
  unit: CapacityUnit;

  /** Retention rate (0-1) */
  retentionRate: number;

  /** Recall accuracy percentage (0-100) */
  recallAccuracy: number;

  /** Encoding speed (items per second) */
  encodingSpeed: number;

  /** Retrieval latency (milliseconds) */
  retrievalLatency: number;

  /** Decay constant (lambda) */
  decayConstant: number;
}

/**
 * Memory capacity assessment
 */
export interface MemoryCapacity {
  /** Memory type being measured */
  memoryType: MemoryType;

  /** Working memory span (items) */
  workingMemorySpan?: number;

  /** Total storage capacity */
  totalCapacity: number;

  /** Available capacity */
  availableCapacity: number;

  /** Capacity utilization (0-1) */
  utilization: number;

  /** Capacity unit */
  unit: CapacityUnit;
}

// ============================================================================
// Enhancement Parameters
// ============================================================================

/**
 * Encoding enhancement parameters
 */
export interface EncodingParams {
  /** Enhancement method */
  method: EnhancementMethod;

  /** Target memory type */
  targetMemoryType: MemoryType;

  /** Enhancement level (1.0 = baseline, higher = stronger) */
  enhancementLevel: number;

  /** Multi-modal encoding enabled */
  multiModal?: boolean;

  /** Contextual tagging enabled */
  contextualTagging?: boolean;

  /** Emotional enhancement factor (0-1) */
  emotionalEnhancement?: number;
}

/**
 * Consolidation enhancement parameters
 */
export interface ConsolidationParams {
  /** Enhancement method */
  method: EnhancementMethod;

  /** Target sleep stage (if applicable) */
  sleepStage?: SleepStage;

  /** Duration in minutes */
  duration: number;

  /** Target memories (if selective) */
  targetMemories?: string[];

  /** Consolidation boost factor */
  boostFactor: number;
}

/**
 * Retrieval optimization parameters
 */
export interface RetrievalParams {
  /** Target memory type */
  targetMemoryType: MemoryType;

  /** Retrieval strategy */
  strategy: 'DIRECT' | 'ASSOCIATIVE' | 'CONTEXTUAL' | 'SEMANTIC';

  /** Cue types to use */
  cueTypes: Array<'CONTEXTUAL' | 'SEMANTIC' | 'TEMPORAL' | 'EMOTIONAL' | 'SENSORY'>;

  /** Neural assistance enabled */
  neuralAssistance?: boolean;

  /** Computational assistance enabled */
  computationalAssistance?: boolean;
}

// ============================================================================
// Enhancement Results
// ============================================================================

/**
 * Encoding enhancement result
 */
export interface EncodingResult {
  /** Success status */
  success: boolean;

  /** Encoding efficiency (0-1) */
  efficiency: number;

  /** Enhancement factor achieved */
  enhancementFactor: number;

  /** Estimated retention duration (seconds) */
  estimatedRetention: number;

  /** Quality score (0-1) */
  quality: number;

  /** Warnings or issues */
  warnings: string[];
}

/**
 * Consolidation enhancement result
 */
export interface ConsolidationResult {
  /** Success status */
  success: boolean;

  /** Consolidation strength (1.0-10.0) */
  consolidationStrength: number;

  /** Memory stabilization achieved */
  stabilized: boolean;

  /** Estimated long-term retention (percentage) */
  longTermRetention: number;

  /** Memories processed */
  memoriesProcessed: number;

  /** Issues encountered */
  issues: string[];
}

/**
 * Retrieval optimization result
 */
export interface RetrievalResult {
  /** Success status */
  success: boolean;

  /** Retrieval accuracy (0-1) */
  accuracy: number;

  /** Retrieval speed improvement factor */
  speedImprovement: number;

  /** Retrieved memories */
  retrievedMemories: Memory[];

  /** Confidence scores */
  confidenceScores: number[];

  /** Retrieval latency (milliseconds) */
  latency: number;
}

// ============================================================================
// Memory Data Structures
// ============================================================================

/**
 * Memory representation
 */
export interface Memory {
  /** Unique memory identifier */
  id: string;

  /** Memory type */
  type: MemoryType;

  /** Encoding timestamp */
  encodedAt: Date;

  /** Last accessed timestamp */
  lastAccessed?: Date;

  /** Content (varies by type) */
  content: MemoryContent;

  /** Context information */
  context: MemoryContext;

  /** Strength/vividness (0-1) */
  strength: number;

  /** Authenticity score (0-1) */
  authenticity: number;

  /** Source information */
  source: MemorySource;

  /** Associated memories */
  associations: string[];

  /** Metadata */
  metadata: Record<string, unknown>;
}

/**
 * Memory content (varies by type)
 */
export interface MemoryContent {
  /** Semantic content */
  semantic?: {
    facts: string[];
    concepts: string[];
    knowledge: Record<string, unknown>;
  };

  /** Episodic content */
  episodic?: {
    event: string;
    narrative: string;
    sensoryDetails: SensoryDetails;
  };

  /** Procedural content */
  procedural?: {
    skill: string;
    steps: string[];
    motorPatterns: unknown[];
  };

  /** Raw data */
  raw?: unknown;
}

/**
 * Sensory details
 */
export interface SensoryDetails {
  visual?: string;
  auditory?: string;
  tactile?: string;
  olfactory?: string;
  gustatory?: string;
}

/**
 * Memory context
 */
export interface MemoryContext {
  /** Temporal context */
  temporal?: {
    timestamp: Date;
    timeOfDay: string;
    season: string;
  };

  /** Spatial context */
  spatial?: {
    location: string;
    coordinates?: { lat: number; lon: number };
    environment: string;
  };

  /** Social context */
  social?: {
    people: string[];
    interactions: string[];
  };

  /** Emotional context */
  emotional?: {
    valence: number; // -1 (negative) to 1 (positive)
    arousal: number; // 0 (calm) to 1 (excited)
    dominantEmotion: string;
  };

  /** Physiological context */
  physiological?: {
    alertness: number;
    stress: number;
    healthState: string;
  };
}

/**
 * Memory source information
 */
export interface MemorySource {
  /** Source type */
  type: 'DIRECT_EXPERIENCE' | 'TOLD' | 'READ' | 'MEDIA' | 'IMAGINED' | 'TRANSFERRED' | 'SYNTHETIC';

  /** Source details */
  details: string;

  /** Verification status */
  verified: boolean;

  /** External corroboration */
  corroboration?: string[];

  /** Chain of custody (for transferred memories) */
  chainOfCustody?: string[];
}

// ============================================================================
// Assessment and Baseline
// ============================================================================

/**
 * Subject information
 */
export interface Subject {
  /** Subject identifier */
  id: string;

  /** Age */
  age: number;

  /** Baseline cognitive status */
  baseline: 'excellent' | 'good' | 'normal' | 'impaired';

  /** Medical history (relevant) */
  medicalHistory?: string[];

  /** Current enhancements */
  currentEnhancements?: Enhancement[];
}

/**
 * Enhancement record
 */
export interface Enhancement {
  /** Enhancement method */
  method: EnhancementMethod;

  /** Start date */
  startDate: Date;

  /** Parameters */
  parameters: Record<string, unknown>;

  /** Current status */
  status: 'active' | 'paused' | 'discontinued';
}

/**
 * Memory baseline assessment
 */
export interface MemoryBaseline {
  /** Subject identifier */
  subjectId: string;

  /** Assessment date */
  assessmentDate: Date;

  /** Working memory metrics */
  workingMemory: {
    span: number; // items
    capacity: number; // bits
    efficiency: number; // 0-1
  };

  /** Short-term memory metrics */
  shortTermMemory: {
    capacity: number; // items
    duration: number; // seconds
    accuracy: number; // 0-1
  };

  /** Long-term memory metrics */
  longTermMemory: {
    capacity: number; // gigabytes (estimated)
    retentionRate: number; // 0-1
    recallAccuracy: number; // 0-1
  };

  /** Memory type-specific assessments */
  episodicMemory?: MemoryTypeAssessment;
  semanticMemory?: MemoryTypeAssessment;
  proceduralMemory?: MemoryTypeAssessment;

  /** Overall cognitive score */
  overallScore: number; // 0-100

  /** Recommendations */
  recommendations: string[];
}

/**
 * Memory type-specific assessment
 */
export interface MemoryTypeAssessment {
  /** Capacity */
  capacity: number;

  /** Accuracy */
  accuracy: number;

  /** Speed */
  speed: number;

  /** Strength */
  strength: number;

  /** Unit */
  unit: CapacityUnit;
}

// ============================================================================
// Memory Transfer and Backup
// ============================================================================

/**
 * Memory backup options
 */
export interface BackupOptions {
  /** Memory types to backup */
  memoryTypes: MemoryType[];

  /** Include neural patterns */
  includeNeuralPatterns: boolean;

  /** Compression level (0-9) */
  compressionLevel: number;

  /** Encryption enabled */
  encrypt: boolean;

  /** Destination path */
  destination: string;
}

/**
 * Memory backup
 */
export interface MemoryBackup {
  /** Backup identifier */
  id: string;

  /** Subject identifier */
  subjectId: string;

  /** Backup timestamp */
  timestamp: Date;

  /** Memory types included */
  memoryTypes: MemoryType[];

  /** Backed up memories */
  memories: Memory[];

  /** Neural patterns (if included) */
  neuralPatterns?: NeuralPattern[];

  /** Integrity checksum */
  checksum: string;

  /** Encryption details */
  encryption?: {
    algorithm: string;
    keyId: string;
  };

  /** Compression details */
  compression: {
    algorithm: string;
    ratio: number;
  };

  /** Total size (bytes) */
  size: number;
}

/**
 * Backup restoration result
 */
export interface RestoreResult {
  /** Success status */
  success: boolean;

  /** Memories restored */
  memoriesRestored: number;

  /** Memories failed */
  memoriesFailed: number;

  /** Restoration errors */
  errors: string[];

  /** Validation result */
  validation: ValidationResult;
}

/**
 * Memory transfer result
 */
export interface TransferResult {
  /** Success status */
  success: boolean;

  /** Memories transferred */
  memoriesTransferred: number;

  /** Transfer efficiency (0-1) */
  efficiency: number;

  /** Integration status */
  integrationStatus: 'complete' | 'partial' | 'pending';

  /** Conflicts encountered */
  conflicts: MemoryConflict[];

  /** Verification result */
  verification: ValidationResult;
}

/**
 * Memory conflict (during transfer or restoration)
 */
export interface MemoryConflict {
  /** Conflict type */
  type: 'DUPLICATE' | 'CONTRADICTORY' | 'INCOMPATIBLE';

  /** Conflicting memories */
  memories: [Memory, Memory];

  /** Resolution strategy */
  resolution: 'MERGE' | 'KEEP_ORIGINAL' | 'KEEP_NEW' | 'MANUAL';

  /** Resolution result */
  resolved: boolean;
}

// ============================================================================
// False Memory Detection
// ============================================================================

/**
 * Memory validation result
 */
export interface ValidationResult {
  /** Overall validation passed */
  valid: boolean;

  /** Authenticity score (0-1) */
  authenticityScore: number;

  /** Confidence level */
  confidence: 'high' | 'medium' | 'low' | 'questionable';

  /** Validation checks */
  checks: {
    sourceVerification: boolean;
    temporalConsistency: boolean;
    semanticCoherence: boolean;
    neuralSignature: boolean;
  };

  /** Warning flags */
  warnings: string[];

  /** Recommendation */
  recommendation: 'ACCEPT' | 'REVIEW' | 'REJECT';
}

/**
 * False memory detection result
 */
export interface FalseMemoryReport {
  /** Subject identifier */
  subjectId: string;

  /** Assessment date */
  assessmentDate: Date;

  /** Total memories assessed */
  totalMemories: number;

  /** Suspected false memories */
  suspectedFalseMemories: Memory[];

  /** Authenticity scores */
  authenticityScores: number[];

  /** Overall false memory rate */
  falseMemoryRate: number;

  /** Detailed findings */
  findings: string[];

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Neural Patterns
// ============================================================================

/**
 * Neural pattern representation
 */
export interface NeuralPattern {
  /** Pattern identifier */
  id: string;

  /** Associated memory */
  memoryId: string;

  /** Neural region */
  region: string;

  /** Firing pattern */
  firingPattern: number[][];

  /** Connectivity matrix */
  connectivity?: number[][];

  /** Synaptic weights */
  synapticWeights?: number[];

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Monitoring and Reports
// ============================================================================

/**
 * Memory monitoring data
 */
export interface MonitoringData {
  /** Subject identifier */
  subjectId: string;

  /** Monitoring timestamp */
  timestamp: Date;

  /** Current memory metrics */
  metrics: MemoryMetrics;

  /** Enhancement status */
  enhancementStatus: {
    active: boolean;
    method?: EnhancementMethod;
    level?: number;
  };

  /** Alerts */
  alerts: Alert[];

  /** Performance indicators */
  performance: {
    encodingRate: number; // items/hour
    retrievalSuccess: number; // 0-1
    consolidationQuality: number; // 0-1
    cognitiveLoad: number; // 0-1
  };
}

/**
 * Alert
 */
export interface Alert {
  /** Alert identifier */
  id: string;

  /** Alert type */
  type: 'CAPACITY' | 'ACCURACY' | 'FALSE_MEMORY' | 'OVERLOAD' | 'MALFUNCTION';

  /** Severity */
  severity: 'info' | 'warning' | 'critical';

  /** Message */
  message: string;

  /** Timestamp */
  timestamp: Date;

  /** Acknowledged */
  acknowledged: boolean;
}

/**
 * Enhancement report
 */
export interface EnhancementReport {
  /** Report identifier */
  id: string;

  /** Subject identifier */
  subjectId: string;

  /** Report period */
  period: {
    start: Date;
    end: Date;
  };

  /** Baseline metrics */
  baseline: MemoryBaseline;

  /** Current metrics */
  current: MemoryMetrics;

  /** Enhancement summary */
  enhancement: {
    method: EnhancementMethod;
    duration: number; // days
    averageLevel: number;
  };

  /** Improvement metrics */
  improvement: {
    capacityIncrease: number; // percentage
    accuracyImprovement: number; // percentage
    speedImprovement: number; // percentage
    retentionImprovement: number; // percentage
  };

  /** Safety metrics */
  safety: {
    falseMemoryRate: number;
    adverseEvents: number;
    cognitiveOverload: number;
  };

  /** Overall assessment */
  assessment: 'EXCELLENT' | 'GOOD' | 'ADEQUATE' | 'POOR' | 'UNSAFE';

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Memory enhancement constants
 */
export const MEMORY_CONSTANTS = {
  /** Natural capacity estimates */
  NATURAL_CAPACITY: {
    WORKING_MEMORY_SPAN: 7, // Miller's 7±2
    SHORT_TERM_DURATION: 30, // seconds
    LONG_TERM_CAPACITY_GB: 2.5, // estimated (2.5 petabytes = 2.5M GB)
  },

  /** Enhancement factors */
  ENHANCEMENT_FACTORS: {
    TRAINING: { min: 1.2, max: 1.5 },
    PHARMACOLOGICAL: { min: 1.3, max: 2.0 },
    ELECTRICAL: { min: 1.5, max: 2.5 },
    NEURAL_IMPLANT: { min: 2.0, max: 5.0 },
    COMPUTATIONAL: { min: 3.0, max: 10.0 },
    HYBRID: { min: 5.0, max: 20.0 },
  },

  /** Decay constants (lambda) */
  DECAY_CONSTANTS: {
    WEAK_MEMORY: 1.0, // 50% in 17 hours
    AVERAGE_MEMORY: 0.5, // 50% in 34 hours
    STRONG_MEMORY: 0.1, // 50% in 7 days
    ENHANCED_MEMORY: 0.05, // 50% in 14 days
  },

  /** Threshold values */
  THRESHOLDS: {
    FALSE_MEMORY_RATE_WARNING: 0.05, // 5%
    FALSE_MEMORY_RATE_CRITICAL: 0.10, // 10%
    AUTHENTICITY_MINIMUM: 0.70, // 70%
    COGNITIVE_LOAD_WARNING: 0.75, // 75%
    COGNITIVE_LOAD_CRITICAL: 0.90, // 90%
  },

  /** Validation criteria */
  VALIDATION: {
    HIGH_CONFIDENCE_THRESHOLD: 0.90,
    MEDIUM_CONFIDENCE_THRESHOLD: 0.70,
    LOW_CONFIDENCE_THRESHOLD: 0.50,
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Memory enhancement error codes
 */
export enum MemoryErrorCode {
  ASSESSMENT_FAILED = 'M001',
  ENHANCEMENT_FAILED = 'M002',
  ENCODING_ERROR = 'M003',
  CONSOLIDATION_ERROR = 'M004',
  RETRIEVAL_ERROR = 'M005',
  BACKUP_FAILED = 'M006',
  RESTORE_FAILED = 'M007',
  TRANSFER_FAILED = 'M008',
  VALIDATION_FAILED = 'M009',
  FALSE_MEMORY_DETECTED = 'M010',
  CAPACITY_EXCEEDED = 'M011',
  COGNITIVE_OVERLOAD = 'M012',
  AUTHENTICATION_ERROR = 'M013',
  ENCRYPTION_ERROR = 'M014',
}

/**
 * Memory enhancement error
 */
export class MemoryError extends Error {
  constructor(
    public code: MemoryErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'MemoryError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  MemoryType,
  EnhancementMethod,
  MemoryPhase,
  SleepStage,
  CapacityUnit,
  MemoryMetrics,
  MemoryCapacity,
  EncodingParams,
  ConsolidationParams,
  RetrievalParams,
  EncodingResult,
  ConsolidationResult,
  RetrievalResult,
  Memory,
  MemoryContent,
  SensoryDetails,
  MemoryContext,
  MemorySource,
  Subject,
  Enhancement,
  MemoryBaseline,
  MemoryTypeAssessment,
  BackupOptions,
  MemoryBackup,
  RestoreResult,
  TransferResult,
  MemoryConflict,
  ValidationResult,
  FalseMemoryReport,
  NeuralPattern,
  MonitoringData,
  Alert,
  EnhancementReport,
};

export { MEMORY_CONSTANTS, MemoryErrorCode, MemoryError };
