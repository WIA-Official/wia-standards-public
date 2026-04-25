/**
 * WIA-DEF-018: Military AI - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense AI Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Geographic coordinates
 */
export interface GeoCoordinate {
  latitude: number;
  longitude: number;
  altitude?: number;
}

/**
 * Geographic polygon for geofencing
 */
export interface GeoPolygon {
  coordinates: GeoCoordinate[];
  altitudeMin?: number;
  altitudeMax?: number;
}

/**
 * Time range
 */
export interface TimeRange {
  start: Date;
  end: Date;
}

// ============================================================================
// Autonomy Levels
// ============================================================================

/**
 * System autonomy levels per WIA-DEF-018
 */
export type AutonomyLevel =
  | 0  // Fully Manual - human operates all functions
  | 1  // AI-Assisted - AI provides information, human decides
  | 2  // AI-Recommended - AI suggests actions, human approves
  | 3  // Human-Supervised - AI acts, human monitors
  | 4; // Human-on-Loop - AI operates, human can override

/**
 * Autonomy level configuration
 */
export interface AutonomyConfig {
  /** Current autonomy level */
  level: AutonomyLevel;

  /** Maximum autonomy duration in seconds */
  maxDuration: number;

  /** Check-in interval in seconds */
  checkinInterval?: number;

  /** Lethal force permitted (only levels 0-1) */
  lethalForcePermitted: boolean;

  /** Human approval required */
  requiresHumanApproval: boolean;
}

// ============================================================================
// Safety Features
// ============================================================================

/**
 * Kill switch configuration
 */
export interface KillSwitch {
  /** Kill switch enabled (mandatory for levels 3-4) */
  enabled: true;

  /** Activation methods */
  activationMethods: ('remote' | 'automatic' | 'physical')[];

  /** Activation time in seconds */
  activationTime: number; // <1 second required

  /** Behavior on activation */
  failureMode: 'return-to-base' | 'land-immediately' | 'loiter' | 'shutdown';
}

/**
 * Geofencing configuration
 */
export interface Geofence {
  /** Geofence enabled (mandatory for levels 3-4) */
  enabled: true;

  /** Geographic boundary */
  boundary: GeoPolygon;

  /** Action on violation */
  violationAction: 'stop' | 'return' | 'alert' | 'shutdown';

  /** Buffer distance in meters */
  bufferDistance?: number;
}

/**
 * Safety features configuration
 */
export interface SafetyFeatures {
  /** Kill switch configuration */
  killSwitch: KillSwitch;

  /** Geofencing configuration */
  geofence: Geofence;

  /** Emergency stop capability */
  emergencyStop: boolean;

  /** Fail-safe defaults */
  failSafeDefaults: boolean;

  /** Audit logging enabled */
  auditLogging: boolean;

  /** Adversarial robustness validated */
  adversarialRobustness: boolean;
}

// ============================================================================
// Human Oversight
// ============================================================================

/**
 * Human oversight level
 */
export type OversightLevel =
  | 'operator'      // Active operator control
  | 'supervisor'    // Supervisory oversight
  | 'monitor';      // Monitoring only

/**
 * Human oversight configuration
 */
export interface HumanOversight {
  /** Oversight level */
  level: OversightLevel;

  /** Operator required */
  operatorRequired: boolean;

  /** Approval required for actions */
  approvalRequiredFor: string[];

  /** Real-time monitoring interface */
  monitoringInterface?: {
    updateRate: number; // Hz
    videoFeed: boolean;
    telemetry: boolean;
    aiDecisionsVisible: boolean;
  };

  /** Intervention capabilities */
  interventionCapabilities: {
    emergencyStop: boolean;
    manualControlTakeover: boolean;
    parameterAdjustment: boolean;
    missionModification: boolean;
  };
}

// ============================================================================
// AI/ML Models
// ============================================================================

/**
 * AI model types
 */
export type AIModelType =
  | 'classification'
  | 'detection'
  | 'segmentation'
  | 'prediction'
  | 'recommendation'
  | 'planning';

/**
 * Model explainability methods
 */
export type ExplainabilityMethod =
  | 'shap'
  | 'lime'
  | 'integrated-gradients'
  | 'attention'
  | 'grad-cam';

/**
 * AI model configuration
 */
export interface AIModel {
  /** Model identifier */
  id: string;

  /** Model name */
  name: string;

  /** Model version */
  version: string;

  /** Model type */
  type: AIModelType;

  /** Model architecture */
  architecture: string;

  /** Confidence threshold */
  confidenceThreshold: number;

  /** Explainability enabled */
  explainabilityEnabled: boolean;

  /** Explainability methods */
  explainabilityMethods?: ExplainabilityMethod[];

  /** Human review required below confidence */
  humanReviewBelowConfidence?: number;

  /** Performance metrics */
  performance: {
    accuracy: number;
    precision: number;
    recall: number;
    f1Score: number;
  };

  /** Robustness metrics */
  robustness?: {
    adversarialAccuracy: number;
    oodDetectionRate: number;
    sensorDegradationTolerance: number;
  };
}

/**
 * Model prediction result
 */
export interface ModelPrediction {
  /** Predicted class/value */
  prediction: any;

  /** Confidence score (0-1) */
  confidence: number;

  /** Uncertainty estimate */
  uncertainty?: number;

  /** Alternative predictions */
  alternatives?: Array<{
    prediction: any;
    confidence: number;
  }>;

  /** Explanation */
  explanation?: Explanation;

  /** Requires human review */
  requiresHumanReview: boolean;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Explanation data
 */
export interface Explanation {
  /** Feature importance scores */
  featureImportance: Array<{
    feature: string;
    importance: number;
  }>;

  /** Decision pathway */
  decisionPathway?: string[];

  /** Confidence factors */
  confidenceFactors: string[];

  /** Counterfactuals */
  counterfactuals?: Array<{
    change: string;
    resultingPrediction: any;
    probability: number;
  }>;

  /** Similar cases */
  similarCases?: Array<{
    caseId: string;
    similarity: number;
    outcome: any;
  }>;
}

// ============================================================================
// Autonomous Systems
// ============================================================================

/**
 * System types
 */
export type SystemType = 'UAV' | 'UGV' | 'USV' | 'UUV' | 'FIXED';

/**
 * Mission types
 */
export type MissionType =
  | 'reconnaissance'
  | 'surveillance'
  | 'logistics'
  | 'search-rescue'
  | 'combat'
  | 'training';

/**
 * Autonomous system configuration
 */
export interface AutonomousSystem {
  /** System identifier */
  id: string;

  /** System type */
  type: SystemType;

  /** System designation */
  designation: string;

  /** Autonomy configuration */
  autonomy: AutonomyConfig;

  /** Safety features */
  safety: SafetyFeatures;

  /** Human oversight */
  oversight: HumanOversight;

  /** AI systems */
  aiSystems: AIModel[];

  /** Sensor configuration */
  sensors: SensorConfig[];

  /** Mission configuration */
  mission?: MissionConfig;

  /** Current status */
  status: SystemStatus;
}

/**
 * Sensor configuration
 */
export interface SensorConfig {
  /** Sensor type */
  type: 'gps' | 'camera' | 'radar' | 'lidar' | 'imu' | 'comm' | 'other';

  /** Sensor identifier */
  id: string;

  /** Sensor status */
  status: 'operational' | 'degraded' | 'failed';

  /** Update rate in Hz */
  updateRate?: number;

  /** Accuracy metrics */
  accuracy?: number;
}

/**
 * Mission configuration
 */
export interface MissionConfig {
  /** Mission type */
  type: MissionType;

  /** Mission objective */
  objective: string;

  /** Target area */
  area?: GeoPolygon;

  /** Approved by */
  approvedBy: string;

  /** Authorization level */
  authorizationLevel: string;

  /** Start time */
  startTime: Date;

  /** Maximum duration in seconds */
  maxDuration: number;

  /** Civilian risk level */
  civilianRisk: 'none' | 'low' | 'medium' | 'high';

  /** Rules of engagement */
  rulesOfEngagement?: string[];
}

/**
 * System status
 */
export interface SystemStatus {
  /** Current state */
  state: 'idle' | 'active' | 'mission' | 'returning' | 'emergency' | 'maintenance';

  /** Current position */
  position?: GeoCoordinate;

  /** Current velocity */
  velocity?: {
    speed: number; // m/s
    heading: number; // degrees
  };

  /** Battery/fuel level (0-1) */
  powerLevel: number;

  /** Health score (0-1) */
  healthScore: number;

  /** Communication link quality (0-1) */
  commLinkQuality: number;

  /** Active alerts */
  alerts: Alert[];

  /** Last update timestamp */
  lastUpdate: Date;
}

/**
 * System alert
 */
export interface Alert {
  /** Alert level */
  level: 'info' | 'warning' | 'error' | 'critical';

  /** Alert message */
  message: string;

  /** Alert timestamp */
  timestamp: Date;

  /** Requires acknowledgment */
  requiresAck: boolean;

  /** Acknowledged */
  acknowledged: boolean;
}

// ============================================================================
// ISR Analysis
// ============================================================================

/**
 * Intelligence data types
 */
export type IntelligenceType =
  | 'IMINT'    // Imagery Intelligence
  | 'SIGINT'   // Signals Intelligence
  | 'HUMINT'   // Human Intelligence
  | 'MASINT'   // Measurement and Signature Intelligence
  | 'OSINT';   // Open Source Intelligence

/**
 * ISR analyzer configuration
 */
export interface ISRAnalyzer {
  /** Data types to analyze */
  dataTypes: IntelligenceType[];

  /** Confidence threshold */
  confidenceThreshold: number;

  /** Human review required */
  humanReviewRequired: boolean;

  /** Multi-INT fusion enabled */
  multiIntFusion: boolean;

  /** Privacy protection enabled */
  privacyProtection: boolean;
}

/**
 * ISR analysis result
 */
export interface ISRAnalysis {
  /** Analysis identifier */
  id: string;

  /** Data type analyzed */
  dataType: IntelligenceType;

  /** Detected objects/entities */
  detections: Detection[];

  /** Classifications */
  classifications: Classification[];

  /** Changes detected */
  changes?: Change[];

  /** Overall confidence */
  confidence: number;

  /** Requires human review */
  requiresHumanReview: boolean;

  /** Explanation */
  explanation: Explanation;

  /** Timestamp */
  timestamp: Date;

  /** Location */
  location?: GeoCoordinate;

  /** Source information */
  source: SourceInfo;
}

/**
 * Detection result
 */
export interface Detection {
  /** Object type */
  type: string;

  /** Confidence */
  confidence: number;

  /** Bounding box (for imagery) */
  boundingBox?: {
    x: number;
    y: number;
    width: number;
    height: number;
  };

  /** Location */
  location?: GeoCoordinate;

  /** Attributes */
  attributes?: Record<string, any>;
}

/**
 * Classification result
 */
export interface Classification {
  /** Class label */
  class: string;

  /** Confidence */
  confidence: number;

  /** Sub-classes */
  subClasses?: Classification[];

  /** Attributes */
  attributes?: Record<string, any>;
}

/**
 * Change detection result
 */
export interface Change {
  /** Change type */
  type: 'addition' | 'removal' | 'modification';

  /** Location */
  location: GeoCoordinate;

  /** Description */
  description: string;

  /** Confidence */
  confidence: number;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Source information
 */
export interface SourceInfo {
  /** Source type */
  type: 'satellite' | 'aerial' | 'ground' | 'intercept' | 'human' | 'other';

  /** Source identifier */
  id: string;

  /** Reliability score (1-6, NATO scale) */
  reliability?: number;

  /** Credibility score (1-6, NATO scale) */
  credibility?: number;

  /** Collection timestamp */
  collectedAt: Date;
}

// ============================================================================
// Predictive Maintenance
// ============================================================================

/**
 * Equipment health status
 */
export interface EquipmentHealth {
  /** Equipment identifier */
  equipmentId: string;

  /** Health score (0-100) */
  healthScore: number;

  /** Anomalies detected */
  anomalies: Anomaly[];

  /** Predicted failures */
  predictedFailures: FailurePrediction[];

  /** Remaining useful life in hours */
  remainingUsefulLife: number;

  /** Recommendations */
  recommendations: MaintenanceRecommendation[];

  /** Timestamp */
  timestamp: Date;

  /** Explanation */
  explanation: Explanation;
}

/**
 * Anomaly detection
 */
export interface Anomaly {
  /** Anomaly type */
  type: string;

  /** Severity */
  severity: 'low' | 'medium' | 'high' | 'critical';

  /** Description */
  description: string;

  /** Confidence */
  confidence: number;

  /** Detected at */
  detectedAt: Date;

  /** Affected components */
  affectedComponents: string[];
}

/**
 * Failure prediction
 */
export interface FailurePrediction {
  /** Failure type */
  type: string;

  /** Component */
  component: string;

  /** Probability */
  probability: number;

  /** Time to failure (hours) */
  timeToFailure: number;

  /** Severity */
  severity: 'low' | 'medium' | 'high' | 'critical';

  /** Impact */
  impact: string;
}

/**
 * Maintenance recommendation
 */
export interface MaintenanceRecommendation {
  /** Priority */
  priority: 'low' | 'medium' | 'high' | 'critical';

  /** Action */
  action: string;

  /** Timeframe */
  timeframe: string;

  /** Reason */
  reason: string;

  /** Estimated cost */
  estimatedCost?: number;

  /** Estimated duration (hours) */
  estimatedDuration?: number;
}

// ============================================================================
// Ethical Compliance
// ============================================================================

/**
 * Ethical assessment
 */
export interface EthicalAssessment {
  /** Is action ethical */
  isEthical: boolean;

  /** Complies with IHL */
  ihlCompliance: boolean;

  /** Proportionality check */
  proportionality: {
    militaryAdvantage: string;
    civilianHarm: string;
    isProportional: boolean;
  };

  /** Distinction capability */
  distinction: {
    canDistinguish: boolean;
    confidence: number;
  };

  /** Violations */
  violations: string[];

  /** Warnings */
  warnings: string[];

  /** Recommendation */
  recommendation: 'proceed' | 'proceed-with-caution' | 'requires-review' | 'prohibited';

  /** Required oversight */
  requiredOversight: OversightLevel[];
}

/**
 * Mission validation
 */
export interface MissionValidation {
  /** Is mission valid */
  isValid: boolean;

  /** Ethical compliance */
  ethicalCompliance: EthicalAssessment;

  /** Safety checks */
  safetyChecks: SafetyCheck[];

  /** Legal compliance */
  legalCompliance: {
    compliant: boolean;
    issues: string[];
  };

  /** Errors (blocking) */
  errors: string[];

  /** Warnings (non-blocking) */
  warnings: string[];

  /** Recommendations */
  recommendations: string[];
}

/**
 * Safety check result
 */
export interface SafetyCheck {
  /** Check name */
  name: string;

  /** Status */
  status: 'pass' | 'fail' | 'warning' | 'info';

  /** Description */
  description: string;

  /** Measured value */
  value?: number;

  /** Expected value */
  expected?: number;

  /** Threshold */
  threshold?: number;

  /** Corrective action */
  correctiveAction?: string;
}

// ============================================================================
// Testing & Validation
// ============================================================================

/**
 * Test result
 */
export interface TestResult {
  /** Test identifier */
  testId: string;

  /** Test name */
  name: string;

  /** Test type */
  type: 'unit' | 'integration' | 'system' | 'operational';

  /** Pass/fail status */
  passed: boolean;

  /** Metrics */
  metrics: Record<string, number>;

  /** Errors */
  errors: string[];

  /** Warnings */
  warnings: string[];

  /** Duration in milliseconds */
  duration: number;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Validation metrics
 */
export interface ValidationMetrics {
  /** Performance metrics */
  performance: {
    accuracy: number;
    precision: number;
    recall: number;
    f1Score: number;
    latency: number;
  };

  /** Robustness metrics */
  robustness: {
    adversarialAccuracy: number;
    oodDetection: number;
    sensorDegradation: number;
    environmentalRobustness: number;
  };

  /** Explainability metrics */
  explainability: {
    humanInterpretability: number;
    attributionCorrectness: number;
    explanationLatency: number;
  };

  /** Safety metrics */
  safety: {
    falsePositiveRate: number;
    falseNegativeRate: number;
    failSafeActivation: number;
    killSwitchLatency: number;
  };
}

// ============================================================================
// Audit & Compliance
// ============================================================================

/**
 * Audit report
 */
export interface AuditReport {
  /** Report identifier */
  id: string;

  /** System audited */
  systemId: string;

  /** Audit timestamp */
  timestamp: Date;

  /** Auditor */
  auditor: string;

  /** Safety audit */
  safety: {
    passed: boolean;
    findings: string[];
  };

  /** Oversight audit */
  oversight: {
    passed: boolean;
    findings: string[];
  };

  /** Explainability audit */
  explainability: {
    passed: boolean;
    findings: string[];
  };

  /** Security audit */
  security: {
    passed: boolean;
    findings: string[];
  };

  /** Performance audit */
  performance: {
    passed: boolean;
    findings: string[];
  };

  /** Legal compliance audit */
  legal: {
    passed: boolean;
    findings: string[];
  };

  /** Overall compliance */
  compliant: boolean;

  /** Recommendations */
  recommendations: string[];

  /** Required actions */
  requiredActions: string[];
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-DEF-018 error codes
 */
export enum DefenseAIErrorCode {
  INVALID_CONFIGURATION = 'DEF001',
  SAFETY_VIOLATION = 'DEF002',
  ETHICAL_VIOLATION = 'DEF003',
  AUTONOMY_EXCEEDED = 'DEF004',
  GEOFENCE_VIOLATION = 'DEF005',
  HUMAN_APPROVAL_REQUIRED = 'DEF006',
  MODEL_CONFIDENCE_LOW = 'DEF007',
  SENSOR_FAILURE = 'DEF008',
  COMMUNICATION_LOSS = 'DEF009',
  KILL_SWITCH_ACTIVATED = 'DEF010',
}

/**
 * Defense AI error
 */
export class DefenseAIError extends Error {
  constructor(
    public code: DefenseAIErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'DefenseAIError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core
  GeoCoordinate,
  GeoPolygon,
  TimeRange,

  // Autonomy
  AutonomyConfig,

  // Safety
  KillSwitch,
  Geofence,
  SafetyFeatures,

  // Oversight
  HumanOversight,

  // AI Models
  AIModel,
  ModelPrediction,
  Explanation,

  // Autonomous Systems
  AutonomousSystem,
  SensorConfig,
  MissionConfig,
  SystemStatus,
  Alert,

  // ISR
  ISRAnalyzer,
  ISRAnalysis,
  Detection,
  Classification,
  Change,
  SourceInfo,

  // Maintenance
  EquipmentHealth,
  Anomaly,
  FailurePrediction,
  MaintenanceRecommendation,

  // Ethics
  EthicalAssessment,
  MissionValidation,
  SafetyCheck,

  // Testing
  TestResult,
  ValidationMetrics,

  // Audit
  AuditReport,
};

export {
  DefenseAIErrorCode,
  DefenseAIError,
};
