/**
 * WIA-AUG-001: Human Augmentation - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Augmentation Type Classification
// ============================================================================

/**
 * Primary augmentation type categories
 */
export type AugmentationType = 'PHYSICAL' | 'SENSORY' | 'COGNITIVE' | 'NEURAL' | 'HYBRID';

/**
 * Integration mode defines physical connection depth
 */
export type IntegrationMode = 'EXTERNAL' | 'SEMI_INVASIVE' | 'FULLY_INVASIVE';

/**
 * Enhancement level based on capability improvement ratio
 */
export type EnhancementLevel = 'MINIMAL' | 'MODERATE' | 'SIGNIFICANT' | 'TRANSFORMATIVE';

/**
 * Operational state of augmentation device
 */
export type OperationalState = 'STANDBY' | 'ACTIVE' | 'DEGRADED' | 'ERROR' | 'MAINTENANCE';

/**
 * Compatibility classification between augmentations
 */
export type CompatibilityLevel = 'HIGHLY_COMPATIBLE' | 'COMPATIBLE' | 'CONDITIONAL' | 'INCOMPATIBLE';

// ============================================================================
// Augmentation Classification Types
// ============================================================================

/**
 * Input for augmentation classification
 */
export interface ClassificationInput {
  /** Primary augmentation domain */
  primaryDomain: AugmentationType;

  /** Secondary domains (if hybrid) */
  secondaryDomains?: AugmentationType[];

  /** Target capabilities being enhanced */
  targetCapabilities: string[];

  /** Integration depth (1-10) */
  integrationDepth: number;

  /** Integration mode */
  integrationMode: IntegrationMode;

  /** Expected enhancement factor */
  enhancementFactor: number;
}

/**
 * Classification result
 */
export interface ClassificationResult {
  /** Determined augmentation type */
  type: AugmentationType;

  /** Enhancement level */
  level: EnhancementLevel;

  /** Integration mode */
  integrationMode: IntegrationMode;

  /** Integration depth score (0-10) */
  integrationDepth: number;

  /** Target capabilities */
  capabilities: string[];

  /** Expected enhancement ratio */
  enhancementRatio: number;

  /** Recommended safety level (per WIA-AUG-013) */
  safetyLevel: string;
}

// ============================================================================
// Capability Metrics
// ============================================================================

/**
 * Physical capability metrics
 */
export interface PhysicalMetrics {
  /** Strength measurements */
  strength?: { value: number; unit: string; test: string };

  /** Speed measurements */
  speed?: { value: number; unit: string; test: string };

  /** Endurance measurements */
  endurance?: { value: number; unit: string; test: string };

  /** Dexterity measurements */
  dexterity?: { value: number; unit: string; test: string };

  /** Flexibility measurements */
  flexibility?: { value: number; unit: string; test: string };
}

/**
 * Sensory capability metrics
 */
export interface SensoryMetrics {
  /** Visual acuity */
  visualAcuity?: { value: number; unit: string; test: string };

  /** Auditory range */
  auditoryRange?: { min: number; max: number; unit: string };

  /** Tactile sensitivity */
  tactileSensitivity?: { value: number; unit: string; test: string };

  /** Proprioception score */
  proprioception?: { value: number; unit: string; test: string };

  /** Additional sensory capabilities */
  other?: Record<string, { value: number; unit: string; test: string }>;
}

/**
 * Cognitive capability metrics
 */
export interface CognitiveMetrics {
  /** Memory capacity */
  memory?: { value: number; unit: string; test: string };

  /** Processing speed */
  processingSpeed?: { value: number; unit: string; test: string };

  /** Pattern recognition accuracy */
  patternRecognition?: { value: number; unit: string; test: string };

  /** Decision making speed/accuracy */
  decisionMaking?: { value: number; unit: string; test: string };

  /** Learning rate */
  learningRate?: { value: number; unit: string; test: string };
}

/**
 * Neural interface metrics
 */
export interface NeuralMetrics {
  /** Neural bandwidth (bits/sec) */
  bandwidth?: { value: number; unit: string };

  /** Signal fidelity (signal-to-noise ratio) */
  signalFidelity?: { value: number; unit: string };

  /** Latency (milliseconds) */
  latency?: { value: number; unit: string };

  /** Integration density (connection count) */
  integrationDensity?: { value: number; unit: string };

  /** Plasticity enhancement factor */
  plasticity?: { value: number; unit: string };
}

// ============================================================================
// Baseline Registry Types
// ============================================================================

/**
 * Subject information for baseline registry
 */
export interface SubjectInfo {
  /** Unique subject identifier */
  subjectId: string;

  /** Age at baseline measurement */
  age: number;

  /** Gender (if relevant to baseline) */
  gender?: 'M' | 'F' | 'Other' | 'Prefer not to say';

  /** Height (cm) */
  height?: number;

  /** Weight (kg) */
  weight?: number;

  /** General health status */
  healthStatus: 'excellent' | 'good' | 'fair' | 'poor';

  /** Relevant medical conditions */
  conditions?: string[];
}

/**
 * Comprehensive baseline measurement
 */
export interface BaselineMeasurement {
  /** Physical capabilities */
  physical?: PhysicalMetrics;

  /** Sensory capabilities */
  sensory?: SensoryMetrics;

  /** Cognitive capabilities */
  cognitive?: CognitiveMetrics;

  /** Neural capabilities (if applicable) */
  neural?: NeuralMetrics;

  /** Measurement conditions */
  conditions: MeasurementConditions;
}

/**
 * Measurement conditions
 */
export interface MeasurementConditions {
  /** Test date and time */
  timestamp: Date;

  /** Testing facility */
  facility?: string;

  /** Test administrator */
  administrator?: string;

  /** Environmental conditions */
  environment?: {
    temperature?: number;
    humidity?: number;
    altitude?: number;
  };

  /** Subject condition at time of test */
  subjectCondition?: {
    rested: boolean;
    fasted: boolean;
    medicationFree: boolean;
    notes?: string;
  };
}

/**
 * Baseline registry record
 */
export interface BaselineRecord {
  /** Unique registry identifier */
  registryId: string;

  /** Subject information */
  subject: SubjectInfo;

  /** Registration date */
  registrationDate: Date;

  /** Baseline measurements */
  baseline: BaselineMeasurement;

  /** Population percentiles */
  populationPercentile?: Record<string, number>;

  /** Update history */
  updates?: BaselineUpdate[];
}

/**
 * Baseline update record
 */
export interface BaselineUpdate {
  /** Update identifier */
  updateId: string;

  /** Update date */
  updateDate: Date;

  /** Reason for update */
  reason: 'annual' | 'pre-upgrade' | 'post-removal' | 'incident' | 'other';

  /** Updated measurements */
  measurements: BaselineMeasurement;

  /** Changes from previous */
  changes?: Record<string, { previous: number; current: number; delta: number }>;
}

// ============================================================================
// Enhancement Ratio Types
// ============================================================================

/**
 * Enhancement ratio calculation input
 */
export interface EnhancementRatioInput {
  /** Baseline value */
  baselineValue: number;

  /** Augmented value */
  augmentedValue: number;

  /** Metric name */
  metric: string;

  /** Unit of measurement */
  unit?: string;

  /** Whether lower values are better (e.g., reaction time) */
  lowerIsBetter?: boolean;
}

/**
 * Enhancement ratio result
 */
export interface EnhancementResult {
  /** Metric name */
  metric: string;

  /** Baseline value */
  baseline: number;

  /** Augmented value */
  augmented: number;

  /** Enhancement ratio */
  ratio: number;

  /** Enhancement level */
  level: EnhancementLevel;

  /** Percentage improvement */
  percentageImprovement: number;

  /** Normalized enhancement ratio (if applicable) */
  normalizedRatio?: number;
}

/**
 * Multi-metric enhancement score
 */
export interface MultiMetricEnhancement {
  /** Individual metric enhancements */
  metrics: EnhancementResult[];

  /** Overall enhancement score */
  overallScore: number;

  /** Weighted average enhancement ratio */
  weightedRatio: number;

  /** Overall enhancement level */
  overallLevel: EnhancementLevel;

  /** Metric weights used */
  weights: Record<string, number>;
}

// ============================================================================
// Compatibility Assessment Types
// ============================================================================

/**
 * Augmentation device information
 */
export interface AugmentationInfo {
  /** Unique augmentation identifier */
  id: string;

  /** Augmentation type */
  type: AugmentationType;

  /** Integration mode */
  integrationMode: IntegrationMode;

  /** Target capabilities */
  capabilities: string[];

  /** Power requirements */
  power?: {
    voltage: number;
    current: number;
    consumption: number;
  };

  /** Communication protocol */
  protocol?: string;

  /** Physical location/mounting */
  location?: string;

  /** Data format */
  dataFormat?: string;

  /** Operating signals */
  signals?: {
    frequency?: number;
    bandwidth?: number;
    modulation?: string;
  };

  /** Thermal characteristics */
  heat?: {
    dissipation: number;
    maxTemperature: number;
  };

  /** Mechanical forces */
  forces?: {
    maxForce: number;
    direction: string;
  };

  /** Resource requirements */
  resources?: string[];
}

/**
 * Technical interface compatibility
 */
export interface TechnicalInterface {
  /** Power compatibility score (0-1) */
  powerCompatibility: number;

  /** Communication protocol compatibility (0-1) */
  communicationProtocol: number;

  /** Physical interference score (0-1, 1 = no interference) */
  physicalInterference: number;

  /** Data format alignment (0-1) */
  dataFormatAlignment: number;

  /** Overall technical interface score */
  score: number;
}

/**
 * Safety interaction compatibility
 */
export interface SafetyInteraction {
  /** Biological conflict score (0-1, 1 = no conflict) */
  biologicalConflict: number;

  /** Electrical interference score (0-1, 1 = no interference) */
  electricalInterference: number;

  /** Thermal interaction score (0-1) */
  thermalInteraction: number;

  /** Mechanical stress score (0-1) */
  mechanicalStress: number;

  /** Overall safety interaction score */
  score: number;
}

/**
 * Performance synergy
 */
export interface PerformanceSynergy {
  /** Cooperative effect (-1 to 1, negative = interference) */
  cooperativeEffect: number;

  /** Resource sharing efficiency (0-1) */
  resourceSharing: number;

  /** Functional complementarity (0-1) */
  functionalComplementarity: number;

  /** Overall performance synergy score */
  score: number;
}

/**
 * Compatibility assessment result
 */
export interface CompatibilityResult {
  /** Pair or group being assessed */
  augmentations: string[];

  /** Technical interface assessment */
  technicalInterface: TechnicalInterface;

  /** Safety interaction assessment */
  safetyInteraction: SafetyInteraction;

  /** Performance synergy assessment */
  performanceSynergy: PerformanceSynergy;

  /** Overall compatibility score (0-1) */
  compatibilityScore: number;

  /** Compatibility level */
  level: CompatibilityLevel;

  /** Compatible or not */
  compatible: boolean;

  /** Warnings and recommendations */
  warnings: string[];

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Performance Evaluation Types
// ============================================================================

/**
 * Performance metrics
 */
export interface PerformanceMetrics {
  /** Effectiveness (0-100%) */
  effectiveness: number;

  /** Reliability (0-100%) */
  reliability: number;

  /** Efficiency (0-100%) */
  efficiency: number;

  /** Usability (0-100%) */
  usability: number;
}

/**
 * Performance characteristics
 */
export interface PerformanceCharacteristics {
  /** Peak performance level */
  peakPerformance: number;

  /** Sustained performance level */
  sustainedPerformance: number;

  /** Recovery time (seconds) */
  recoveryTime: number;

  /** Adaptation period (days) */
  adaptationPeriod: number;

  /** Performance degradation rate */
  degradationRate?: number;
}

/**
 * Performance evaluation
 */
export interface PerformanceEvaluation {
  /** Evaluation identifier */
  evaluationId: string;

  /** Augmentation being evaluated */
  augmentationId: string;

  /** Test date */
  testDate: Date;

  /** Test protocol used */
  testProtocol: string;

  /** Performance metrics */
  metrics: PerformanceMetrics;

  /** Enhancement ratio */
  enhancementRatio: number;

  /** Enhancement level */
  level: EnhancementLevel;

  /** Performance characteristics */
  performance: PerformanceCharacteristics;

  /** Test data */
  testData?: Record<string, any>;

  /** Pass/fail status */
  passed: boolean;

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Interoperability Types
// ============================================================================

/**
 * Message type for inter-augmentation communication
 */
export type MessageType = 'STATUS_UPDATE' | 'COMMAND' | 'RESPONSE' | 'ALERT' | 'SYNC';

/**
 * Command type
 */
export type CommandType =
  | 'ACTIVATE'
  | 'DEACTIVATE'
  | 'ADJUST_LEVEL'
  | 'SYNC_WITH'
  | 'REQUEST_STATUS'
  | 'EMERGENCY_STOP';

/**
 * Priority level
 */
export type Priority = 'LOW' | 'NORMAL' | 'HIGH' | 'CRITICAL';

/**
 * Message header
 */
export interface MessageHeader {
  /** Unique message identifier */
  messageId: string;

  /** Message timestamp */
  timestamp: Date;

  /** Source augmentation ID */
  sourceId: string;

  /** Destination augmentation ID */
  destinationId: string;

  /** Message type */
  messageType: MessageType;

  /** Priority */
  priority?: Priority;
}

/**
 * Status update payload
 */
export interface StatusUpdatePayload {
  /** Augmentation type */
  augmentationType: AugmentationType;

  /** Operational status */
  status: OperationalState;

  /** Performance metrics */
  performanceMetrics: {
    currentLoad: number;
    efficiency: number;
    powerLevel: number;
  };

  /** Active alerts */
  alerts: Alert[];
}

/**
 * Command payload
 */
export interface CommandPayload {
  /** Command type */
  command: CommandType;

  /** Command parameters */
  parameters?: Record<string, any>;

  /** Requires acknowledgment */
  requiresAcknowledgment: boolean;
}

/**
 * Data exchange message
 */
export interface DataExchangeMessage {
  /** Message header */
  header: MessageHeader;

  /** Message payload */
  payload: StatusUpdatePayload | CommandPayload | Record<string, any>;
}

/**
 * Augmentation status
 */
export interface AugmentationStatus {
  /** Augmentation identifier */
  augmentationId: string;

  /** Status timestamp */
  timestamp: Date;

  /** Operational state */
  operationalState: OperationalState;

  /** Health metrics */
  health: {
    batteryLevel: number; // 0-100%
    signalIntegrity: number; // 0-100%
    mechanicalIntegrity: number; // 0-100%
    biologicalInterface: number; // 0-100%
  };

  /** Performance data */
  performance: {
    currentEnhancement: number;
    efficiency: number;
    uptime: number; // hours
  };

  /** Active alerts */
  alerts: Alert[];

  /** Diagnostic data */
  diagnostics?: Record<string, any>;
}

/**
 * Alert
 */
export interface Alert {
  /** Alert identifier */
  id: string;

  /** Alert type */
  type: 'power' | 'signal' | 'temperature' | 'performance' | 'safety' | 'other';

  /** Severity */
  severity: 'info' | 'warning' | 'critical';

  /** Alert message */
  message: string;

  /** Alert timestamp */
  timestamp: Date;

  /** Acknowledged flag */
  acknowledged: boolean;
}

/**
 * Synchronization protocol configuration
 */
export interface SyncProtocol {
  /** Sync group identifier */
  syncGroupId: string;

  /** Coordinator augmentation ID */
  coordinator: string;

  /** Participant augmentation IDs */
  participants: string[];

  /** Synchronization settings */
  synchronization: {
    clockSync: boolean;
    dataSync: boolean;
    actionSync: boolean;
  };

  /** Coordination settings */
  coordination: {
    latencyTolerance: number; // milliseconds
    updateFrequency: number; // Hz
    conflictResolution: 'PRIORITY' | 'CONSENSUS' | 'COORDINATOR_DECIDES';
  };
}

/**
 * Synchronization result
 */
export interface SyncResult {
  /** Sync group ID */
  syncGroupId: string;

  /** Sync status */
  status: 'SUCCESS' | 'PARTIAL' | 'FAILED';

  /** Synchronized augmentations */
  synchronized: string[];

  /** Failed augmentations */
  failed: string[];

  /** Sync latency (ms) */
  latency: number;

  /** Error messages */
  errors?: string[];
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Enhancement thresholds
 */
export const ENHANCEMENT_THRESHOLDS = {
  MINIMAL_MIN: 1.1,
  MINIMAL_MAX: 2.0,
  MODERATE_MIN: 2.0,
  MODERATE_MAX: 5.0,
  SIGNIFICANT_MIN: 5.0,
  SIGNIFICANT_MAX: 10.0,
  TRANSFORMATIVE_MIN: 10.0,
} as const;

/**
 * Integration depth thresholds
 */
export const INTEGRATION_THRESHOLDS = {
  EXTERNAL_MAX: 3.0,
  SEMI_INVASIVE_MIN: 3.1,
  SEMI_INVASIVE_MAX: 6.5,
  FULLY_INVASIVE_MIN: 6.6,
} as const;

/**
 * Compatibility score thresholds
 */
export const COMPATIBILITY_THRESHOLDS = {
  HIGHLY_COMPATIBLE: 0.8,
  COMPATIBLE: 0.6,
  CONDITIONAL: 0.4,
} as const;

/**
 * Compatibility weights
 */
export const COMPATIBILITY_WEIGHTS = {
  TECHNICAL_INTERFACE: 0.4,
  SAFETY_INTERACTION: 0.35,
  PERFORMANCE_SYNERGY: 0.25,
} as const;

/**
 * Minimum performance requirements
 */
export const MINIMUM_PERFORMANCE = {
  EFFECTIVENESS: 80,
  RELIABILITY: 95,
  EFFICIENCY: 70,
  USABILITY: 75,
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Augmentation error codes
 */
export enum AugmentationErrorCode {
  CLASSIFICATION_INVALID_INPUT = 'AUG001',
  BASELINE_NOT_FOUND = 'AUG002',
  ENHANCEMENT_CALCULATION_FAILED = 'AUG003',
  COMPATIBILITY_ASSESSMENT_FAILED = 'AUG004',
  PERFORMANCE_BELOW_THRESHOLD = 'AUG005',
  INTEROPERABILITY_ERROR = 'AUG006',
  SYNC_FAILED = 'AUG007',
  INVALID_AUGMENTATION_TYPE = 'AUG008',
}

/**
 * Augmentation error class
 */
export class AugmentationError extends Error {
  constructor(
    public code: AugmentationErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'AugmentationError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  AugmentationType,
  IntegrationMode,
  EnhancementLevel,
  OperationalState,
  CompatibilityLevel,
  ClassificationInput,
  ClassificationResult,
  PhysicalMetrics,
  SensoryMetrics,
  CognitiveMetrics,
  NeuralMetrics,
  SubjectInfo,
  BaselineMeasurement,
  MeasurementConditions,
  BaselineRecord,
  BaselineUpdate,
  EnhancementRatioInput,
  EnhancementResult,
  MultiMetricEnhancement,
  AugmentationInfo,
  TechnicalInterface,
  SafetyInteraction,
  PerformanceSynergy,
  CompatibilityResult,
  PerformanceMetrics,
  PerformanceCharacteristics,
  PerformanceEvaluation,
  MessageType,
  CommandType,
  Priority,
  MessageHeader,
  StatusUpdatePayload,
  CommandPayload,
  DataExchangeMessage,
  AugmentationStatus,
  Alert,
  SyncProtocol,
  SyncResult,
};

export {
  ENHANCEMENT_THRESHOLDS,
  INTEGRATION_THRESHOLDS,
  COMPATIBILITY_THRESHOLDS,
  COMPATIBILITY_WEIGHTS,
  MINIMUM_PERFORMANCE,
  AugmentationErrorCode,
  AugmentationError,
};
