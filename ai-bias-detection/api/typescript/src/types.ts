/**
 * WIA-AI-013: AI Bias Detection Standard
 * TypeScript Type Definitions
 *
 * Philosophy: 弘益人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @license CC BY 4.0
 */

/**
 * Protected attributes that can be used for bias detection
 */
export type ProtectedAttribute =
  | 'gender'
  | 'race'
  | 'ethnicity'
  | 'age'
  | 'nationality'
  | 'religion'
  | 'disability'
  | 'marital_status'
  | string;

/**
 * Fairness metrics supported by the standard
 */
export type FairnessMetric =
  | 'demographic_parity'
  | 'equalized_odds'
  | 'equal_opportunity'
  | 'predictive_parity'
  | 'disparate_impact';

/**
 * Status of a fairness check
 */
export type FairnessStatus = 'pass' | 'fail' | 'warning';

/**
 * Types of bias that can be detected
 */
export type BiasType =
  | 'selection'
  | 'measurement'
  | 'label'
  | 'algorithmic'
  | 'aggregation'
  | 'temporal';

/**
 * Severity levels for detected bias
 */
export type BiasSeverity = 'low' | 'medium' | 'high' | 'critical';

/**
 * Model types supported
 */
export type ModelType = 'classification' | 'regression' | 'ranking';

/**
 * Mitigation phases
 */
export type MitigationPhase = 'pre-processing' | 'in-processing' | 'post-processing';

/**
 * Report metadata
 */
export interface ReportMetadata {
  standardVersion: string;
  reportId: string;
  generatedAt: string;
  generatedBy: {
    tool: string;
    version: string;
    organization?: string;
  };
}

/**
 * Model information
 */
export interface ModelInfo {
  modelId: string;
  modelName: string;
  modelVersion: string;
  modelType: ModelType;
  taskDescription: string;
}

/**
 * Dataset information
 */
export interface DatasetInfo {
  datasetId: string;
  datasetName: string;
  totalSamples: number;
  dateRange?: {
    start: string;
    end: string;
  };
  protectedAttributes: ProtectedAttribute[];
}

/**
 * Group-specific metrics
 */
export interface GroupMetrics {
  positiveRate?: number;
  sampleSize: number;
  accuracy?: number;
  precision?: number;
  recall?: number;
  f1Score?: number;
  truePositiveRate?: number;
  falsePositiveRate?: number;
  trueNegativeRate?: number;
  falseNegativeRate?: number;
}

/**
 * Confusion matrix
 */
export interface ConfusionMatrix {
  truePositive: number;
  falsePositive: number;
  trueNegative: number;
  falseNegative: number;
}

/**
 * Demographic parity metric
 */
export interface DemographicParityMetric {
  metric: 'demographic_parity_ratio';
  value: number;
  threshold: number;
  status: FairnessStatus;
  byGroup: Record<string, GroupMetrics>;
}

/**
 * Equalized odds metric
 */
export interface EqualizedOddsMetric {
  metric: 'equalized_odds';
  tprDisparity: number;
  fprDisparity: number;
  threshold: number;
  status: FairnessStatus;
  byGroup: Record<string, GroupMetrics>;
}

/**
 * Equal opportunity metric
 */
export interface EqualOpportunityMetric {
  metric: 'equal_opportunity';
  value: number;
  threshold: number;
  status: FairnessStatus;
  byGroup: Record<string, GroupMetrics>;
}

/**
 * Predictive parity metric
 */
export interface PredictiveParityMetric {
  metric: 'predictive_parity';
  value: number;
  threshold: number;
  status: FairnessStatus;
  byGroup: Record<string, GroupMetrics>;
}

/**
 * Disparate impact metric
 */
export interface DisparateImpactMetric {
  metric: 'disparate_impact_ratio';
  value: number;
  threshold: number;
  status: FairnessStatus;
  referenceGroup: string;
  byGroup: Record<string, { ratio: number }>;
}

/**
 * All fairness metrics
 */
export interface FairnessMetrics {
  demographicParity?: DemographicParityMetric;
  equalizedOdds?: EqualizedOddsMetric;
  equalOpportunity?: EqualOpportunityMetric;
  predictiveParity?: PredictiveParityMetric;
  disparateImpact?: DisparateImpactMetric;
}

/**
 * Group performance metrics
 */
export interface GroupPerformanceMetrics {
  byProtectedAttribute: Record<ProtectedAttribute, Record<string, GroupMetrics & { confusionMatrix?: ConfusionMatrix }>>;
  intersectional?: Record<string, GroupMetrics>;
}

/**
 * Bias indicator evidence
 */
export interface BiasEvidence {
  metricName: string;
  observedValue: number;
  expectedValue: number;
  deviation: number;
}

/**
 * Detected bias indicator
 */
export interface BiasIndicator {
  type: BiasType;
  severity: BiasSeverity;
  confidence: number;
  description: string;
  affectedGroups: string[];
  evidence: BiasEvidence;
}

/**
 * Proxy feature detection
 */
export interface ProxyFeature {
  featureName: string;
  protectedAttribute: ProtectedAttribute;
  correlation: number;
  importance: number;
  recommendation: 'remove' | 'review' | 'retain';
}

/**
 * Bias indicators
 */
export interface BiasIndicators {
  biasTypes: BiasIndicator[];
  proxyFeatures: ProxyFeature[];
}

/**
 * Mitigation recommendation
 */
export interface MitigationRecommendation {
  id: string;
  category: 'data' | 'model' | 'deployment' | 'monitoring';
  phase: MitigationPhase;
  title: string;
  description: string;
  expectedImpact: string;
  implementationEffort: 'low' | 'medium' | 'high';
  resources: string[];
}

/**
 * Recommendations
 */
export interface Recommendations {
  priority: 'immediate' | 'high' | 'medium' | 'low';
  recommendations: MitigationRecommendation[];
}

/**
 * Complete bias detection report
 */
export interface BiasDetectionReport {
  reportMetadata: ReportMetadata;
  modelInfo: ModelInfo;
  datasetInfo: DatasetInfo;
  fairnessMetrics: FairnessMetrics;
  groupMetrics: GroupPerformanceMetrics;
  biasIndicators: BiasIndicators;
  recommendations: Recommendations;
}

/**
 * API response wrapper
 */
export interface APIResponse<T> {
  success: boolean;
  data?: T;
  errors: string[];
  warnings: string[];
}

/**
 * Job status
 */
export type JobStatus = 'pending' | 'processing' | 'completed' | 'failed' | 'cancelled';

/**
 * Analysis job
 */
export interface AnalysisJob {
  jobId: string;
  status: JobStatus;
  estimatedCompletionTime?: string;
  result?: BiasDetectionReport;
  completedAt?: string;
  error?: string;
  _links: {
    self: string;
    cancel?: string;
  };
}

/**
 * Bias detection configuration
 */
export interface BiasDetectorConfig {
  protectedAttributes: ProtectedAttribute[];
  fairnessCriteria?: Partial<Record<FairnessMetric, number>>;
  includeIntersectional?: boolean;
  customThresholds?: Record<string, number>;
}

/**
 * Prediction log entry
 */
export interface PredictionLog {
  predictionId: string;
  modelId: string;
  timestamp: string;
  input: Record<string, any>;
  output: {
    prediction: number | string;
    probability?: number;
  };
  protectedAttributes: Record<ProtectedAttribute, any>;
}

/**
 * Monitoring alert
 */
export interface MonitoringAlert {
  alertId: string;
  level: 'INFO' | 'WARNING' | 'CRITICAL' | 'EMERGENCY';
  type: string;
  modelId: string;
  timestamp: string;
  metric: FairnessMetric;
  currentValue: number;
  threshold: number;
  affectedGroups: string[];
  context: Record<string, any>;
  recommendations: Array<{
    action: string;
    priority: string;
  }>;
}

/**
 * Drift detection result
 */
export interface DriftDetectionResult {
  driftType: 'data_drift' | 'prediction_drift' | 'fairness_drift';
  timestamp: string;
  status: 'no_drift' | 'drift_detected' | 'significant_drift';
  metric?: string;
  currentValue?: number;
  baselineValue?: number;
  deviation?: number;
  threshold?: number;
}

/**
 * Audit log entry
 */
export interface AuditLogEntry {
  eventId: string;
  eventType: string;
  timestamp: string;
  actor: {
    type: 'system' | 'user';
    id: string;
    name: string;
  };
  resource: {
    type: string;
    id: string;
    version?: string;
  };
  action: string;
  result: string;
  details: Record<string, any>;
  metadata?: Record<string, any>;
}

/**
 * Export all types
 */
export type {
  ProtectedAttribute,
  FairnessMetric,
  FairnessStatus,
  BiasType,
  BiasSeverity,
  ModelType,
  MitigationPhase,
};
