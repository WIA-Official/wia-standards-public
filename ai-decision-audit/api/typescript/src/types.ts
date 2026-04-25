/**
 * WIA-AI-018: AI Decision Audit - TypeScript Type Definitions
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 *
 * Complete type definitions for AI decision audit systems implementing
 * the WIA-AI-018 standard.
 *
 * @version 1.0.0
 * @license MIT
 */

/**
 * Core decision audit log structure
 */
export interface DecisionAuditLog {
  /** Unique decision identifier (UUID recommended) */
  decision_id: string;

  /** Correlation ID for linking related decisions */
  correlation_id?: string;

  /** Temporal information */
  timestamp: string;  // ISO 8601 format
  timezone: string;   // IANA timezone identifier
  processing_duration_ms: number;

  /** System information */
  system: SystemInfo;

  /** AI model information */
  model: ModelInfo;

  /** Input data and context */
  input: InputData;

  /** Decision output */
  output: DecisionOutput;

  /** Reasoning and explainability */
  reasoning?: ReasoningInfo;

  /** Decision context */
  context: DecisionContext;

  /** Compliance metadata */
  compliance: ComplianceMetadata;

  /** Audit metadata */
  audit: AuditMetadata;
}

/**
 * System information
 */
export interface SystemInfo {
  name: string;
  version: string;
  environment: "production" | "staging" | "development";
  deployment_id: string;
}

/**
 * AI model information
 */
export interface ModelInfo {
  name: string;
  version: string;
  type: "neural_network" | "decision_tree" | "ensemble" | "rule_based" | "other";
  training_date: string;
  training_dataset_id?: string;
  documentation_url?: string;
  accuracy_metrics?: AccuracyMetrics;
  bias_testing_completed?: boolean;
}

/**
 * Model accuracy metrics
 */
export interface AccuracyMetrics {
  accuracy: number;
  precision: number;
  recall: number;
  f1_score: number;
  auc_roc?: number;
}

/**
 * Input data structure
 */
export interface InputData {
  raw_data: any;
  preprocessed_data?: any;
  feature_vector?: number[];
  data_sources: string[];
}

/**
 * Decision output
 */
export interface DecisionOutput {
  decision: any;
  confidence: number;  // 0.0 to 1.0
  alternatives?: Alternative[];
  flags: string[];
}

/**
 * Alternative decision option
 */
export interface Alternative {
  decision: any;
  confidence: number;
}

/**
 * Reasoning and explanation
 */
export interface ReasoningInfo {
  explanation: string;
  key_features: KeyFeature[];
  decision_path?: string;
  attention_weights?: number[];
  counterfactuals?: Counterfactual[];
}

/**
 * Key feature importance
 */
export interface KeyFeature {
  name: string;
  importance: number;
  value: any;
}

/**
 * Counterfactual explanation
 */
export interface Counterfactual {
  description: string;
  modified_features: Record<string, any>;
  resulting_decision: any;
}

/**
 * Decision context
 */
export interface DecisionContext {
  user_id?: string;
  session_id?: string;
  request_id?: string;
  geographic_location?: string;
  decision_type: string;
  business_impact: "low" | "medium" | "high" | "critical";
  risk_category?: "low" | "medium" | "high";
  affected_population?: number;
  financial_impact?: number;
  reversible?: boolean;
}

/**
 * Compliance metadata
 */
export interface ComplianceMetadata {
  data_subject_consent: boolean;
  consent_date?: string;
  purpose: string[];
  legal_basis: "consent" | "contract" | "legal_obligation" | "legitimate_interest";
  automated: boolean;
  human_review_required: boolean;
  human_reviewer_id?: string;
  human_review_timestamp?: string;
  disclosure_provided?: boolean;
  explanation_available?: boolean;
  data_shared_with_third_party?: boolean;
  transparency_notice_provided?: boolean;
  human_oversight_available?: boolean;
  adverse_action_notice_sent?: boolean;
}

/**
 * Audit metadata
 */
export interface AuditMetadata {
  log_version: string;
  integrity_hash: string;
  previous_hash?: string;
  signature?: string;
}

/**
 * Compliance rule definition
 */
export interface ComplianceRule {
  id: string;
  name: string;
  framework: ComplianceFramework;
  severity: "critical" | "high" | "medium" | "low";
  check: (log: DecisionAuditLog) => ComplianceResult;
  remediation?: (log: DecisionAuditLog) => Promise<void>;
}

/**
 * Supported compliance frameworks
 */
export type ComplianceFramework =
  | "GDPR"
  | "CCPA"
  | "EU_AI_Act"
  | "ECOA"
  | "HIPAA"
  | "SOC2"
  | "ISO27001"
  | "Custom";

/**
 * Compliance check result
 */
export interface ComplianceResult {
  compliant: boolean;
  message: string;
  evidence?: any;
  recommendation?: string;
}

/**
 * Compliance report
 */
export interface ComplianceReport {
  decision_id: string;
  overall_compliant: boolean;
  violations: ComplianceViolation[];
  results: ComplianceResult[];
  checked_at: string;
}

/**
 * Compliance violation
 */
export interface ComplianceViolation {
  rule_id: string;
  rule_name: string;
  severity: "critical" | "high" | "medium" | "low";
  message: string;
  evidence?: any;
  timestamp: string;
}

/**
 * Risk assessment score
 */
export interface RiskScore {
  overall: number;  // 0-100
  dimensions: RiskDimensions;
  flags: string[];
  severity: "low" | "medium" | "high" | "critical";
}

/**
 * Risk dimensions
 */
export interface RiskDimensions {
  confidence_risk: number;
  bias_risk: number;
  data_quality_risk: number;
  drift_risk: number;
  impact_risk: number;
  compliance_risk: number;
  fairness_risk: number;
  explainability_risk: number;
}

/**
 * Bias analysis result
 */
export interface BiasAnalysis {
  bias_score: number;  // 0-100
  disparate_impact_ratio: number;
  statistical_parity_difference: number;
  equal_opportunity_difference: number;
  demographic_parity: Record<string, number>;
  approval_rates_by_group: Record<string, number>;
  sample_size: number;
  confidence_interval: [number, number];
}

/**
 * Model drift detection result
 */
export interface DriftAnalysis {
  feature_drift: {
    kl_divergence: number;
    drifted_features: string[];
  };
  prediction_drift: {
    distribution_shift: number;
    mean_prediction_change: number;
  };
  accuracy_drift: {
    current_accuracy: number;
    baseline_accuracy: number;
    degradation: number;
  };
  recommended_action: "none" | "investigate" | "retrain";
}

/**
 * Anomaly detection result
 */
export interface AnomalyDetection {
  anomalies: Anomaly[];
  anomaly_score: number;
  requires_investigation: boolean;
}

/**
 * Individual anomaly
 */
export interface Anomaly {
  type: "unusual_confidence" | "slow_processing" | "unusual_input" |
        "suspicious_repetition" | "data_quality";
  severity: "low" | "medium" | "high";
  description: string;
  details?: any;
}

/**
 * Audit query parameters
 */
export interface AuditQuery {
  start_date?: string;
  end_date?: string;
  model_name?: string;
  model_version?: string;
  decision_type?: string;
  business_impact?: ("low" | "medium" | "high" | "critical")[];
  risk_severity?: ("low" | "medium" | "high" | "critical")[];
  compliance_status?: "compliant" | "non_compliant" | "all";
  limit?: number;
  offset?: number;
}

/**
 * Audit statistics
 */
export interface AuditStatistics {
  total_decisions: number;
  compliant_decisions: number;
  compliance_rate: number;
  high_risk_count: number;
  critical_risk_count: number;
  average_confidence: number;
  average_risk_score: number;
}

/**
 * Export format options
 */
export type ExportFormat = "JSON" | "CSV" | "PDF" | "EXCEL";

/**
 * Report generation options
 */
export interface ReportOptions {
  format: ExportFormat;
  include_sections: ReportSection[];
  date_range: DateRange;
  recipient_email?: string;
}

/**
 * Report sections
 */
export type ReportSection =
  | "executive_summary"
  | "decision_statistics"
  | "risk_analysis"
  | "compliance_status"
  | "detailed_logs"
  | "recommendations";

/**
 * Date range
 */
export interface DateRange {
  start: string;  // ISO 8601
  end: string;    // ISO 8601
}

/**
 * Audit logger configuration
 */
export interface AuditLoggerConfig {
  async_logging: boolean;
  batch_size: number;
  flush_interval_ms: number;
  enable_encryption: boolean;
  enable_hash_chaining: boolean;
  storage_backend: "postgresql" | "mongodb" | "s3" | "custom";
  connection_string?: string;
}

/**
 * Hash chain verification result
 */
export interface HashChainVerification {
  valid: boolean;
  verified_count: number;
  total_count: number;
  first_invalid_index?: number;
  integrity_score: number;
}

/**
 * Health check result
 */
export interface HealthCheckResult {
  status: "healthy" | "degraded" | "unhealthy";
  checks: HealthCheck[];
  timestamp: string;
}

/**
 * Individual health check
 */
export interface HealthCheck {
  name: string;
  status: "pass" | "fail";
  latency_ms?: number;
  message?: string;
}

/**
 * Federated audit log (Phase 4)
 */
export interface FederatedDecisionLog extends DecisionAuditLog {
  federation: {
    originating_node: string;
    participating_nodes: string[];
    cross_org_validation: boolean;
    consensus_signatures: ConsensusSignature[];
  };
}

/**
 * Consensus signature (Phase 4)
 */
export interface ConsensusSignature {
  node_id: string;
  signature: string;
  timestamp: string;
}

/**
 * Dashboard metrics
 */
export interface DashboardMetrics {
  overview: AuditStatistics;
  risk_distribution: Record<string, number>;
  compliance_by_framework: Record<string, number>;
  trend_data: TrendDataPoint[];
  top_violations: ComplianceViolation[];
  recent_alerts: Alert[];
}

/**
 * Trend data point
 */
export interface TrendDataPoint {
  date: string;
  compliance_rate: number;
  average_risk: number;
  decision_count: number;
}

/**
 * Alert
 */
export interface Alert {
  id: string;
  severity: "critical" | "high" | "medium" | "low";
  type: "compliance_violation" | "high_risk" | "bias_detected" | "drift_detected";
  message: string;
  decision_id?: string;
  timestamp: string;
  acknowledged: boolean;
}
