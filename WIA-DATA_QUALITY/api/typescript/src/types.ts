/**
 * WIA Data Quality Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Data Quality Types
// ============================================================================

export interface WIADataQuality {
  standard: 'WIA-DATA-QUALITY';
  version: string;
  profile: QualityProfile;
  dimensions: QualityDimension[];
  rules: QualityRule[];
  checks: QualityCheck[];
  reports?: QualityReport[];
  extensions?: Record<string, unknown>;
}

export interface QualityProfile {
  id: string;
  name: string;
  description?: string;
  owner: string;
  dataAsset: DataAssetReference;
  createdAt: string;
  updatedAt?: string;
  status: ProfileStatus;
  tags?: string[];
}

export type ProfileStatus =
  | 'draft'
  | 'active'
  | 'under-review'
  | 'deprecated';

export interface DataAssetReference {
  id: string;
  name: string;
  type: DataAssetType;
  location: AssetLocation;
  schema?: SchemaReference;
}

export type DataAssetType =
  | 'table'
  | 'view'
  | 'file'
  | 'stream'
  | 'api'
  | 'dataset';

export interface AssetLocation {
  system: string;
  database?: string;
  schema?: string;
  path?: string;
  uri?: string;
}

export interface SchemaReference {
  type: 'inline' | 'external';
  fields?: SchemaField[];
  externalRef?: string;
}

export interface SchemaField {
  name: string;
  type: string;
  nullable?: boolean;
  description?: string;
  businessName?: string;
}

// ============================================================================
// Quality Dimension Types
// ============================================================================

export interface QualityDimension {
  id: string;
  name: DimensionName;
  weight: number;
  targetScore: number;
  currentScore?: number;
  metrics: DimensionMetric[];
  thresholds: DimensionThreshold;
}

export type DimensionName =
  | 'completeness'
  | 'accuracy'
  | 'consistency'
  | 'validity'
  | 'timeliness'
  | 'uniqueness'
  | 'integrity';

export interface DimensionMetric {
  name: string;
  value: number;
  unit?: string;
  trend?: 'improving' | 'stable' | 'declining';
  lastMeasured?: string;
}

export interface DimensionThreshold {
  critical: number;
  warning: number;
  acceptable: number;
}

// ============================================================================
// Quality Rule Types
// ============================================================================

export interface QualityRule {
  id: string;
  name: string;
  description?: string;
  dimension: DimensionName;
  type: RuleType;
  scope: RuleScope;
  definition: RuleDefinition;
  severity: RuleSeverity;
  enabled: boolean;
  metadata?: RuleMetadata;
}

export type RuleType =
  | 'null-check'
  | 'range-check'
  | 'format-check'
  | 'reference-check'
  | 'uniqueness-check'
  | 'consistency-check'
  | 'freshness-check'
  | 'custom-sql'
  | 'custom-expression'
  | 'statistical';

export interface RuleScope {
  type: 'column' | 'row' | 'table' | 'cross-table';
  columns?: string[];
  tables?: string[];
  filter?: string;
}

export type RuleDefinition =
  | NullCheckDefinition
  | RangeCheckDefinition
  | FormatCheckDefinition
  | ReferenceCheckDefinition
  | UniquenessCheckDefinition
  | ConsistencyCheckDefinition
  | FreshnessCheckDefinition
  | CustomSQLDefinition
  | StatisticalDefinition;

export interface NullCheckDefinition {
  type: 'null-check';
  maxNullPercentage: number;
  treatEmptyAsNull?: boolean;
}

export interface RangeCheckDefinition {
  type: 'range-check';
  min?: number;
  max?: number;
  inclusive?: boolean;
}

export interface FormatCheckDefinition {
  type: 'format-check';
  pattern: string;
  patternType: 'regex' | 'glob' | 'predefined';
  predefinedFormat?: 'email' | 'phone' | 'url' | 'date' | 'uuid' | 'ip';
}

export interface ReferenceCheckDefinition {
  type: 'reference-check';
  referenceTable: string;
  referenceColumn: string;
  matchType: 'exact' | 'subset';
}

export interface UniquenessCheckDefinition {
  type: 'uniqueness-check';
  columns: string[];
  scope: 'table' | 'partition' | 'window';
}

export interface ConsistencyCheckDefinition {
  type: 'consistency-check';
  expression: string;
  compareWith?: {
    table: string;
    column: string;
    aggregation?: string;
  };
}

export interface FreshnessCheckDefinition {
  type: 'freshness-check';
  timestampColumn: string;
  maxAge: number;
  ageUnit: 'seconds' | 'minutes' | 'hours' | 'days';
}

export interface CustomSQLDefinition {
  type: 'custom-sql';
  sql: string;
  expectation: 'empty' | 'non-empty' | 'count';
  threshold?: number;
}

export interface StatisticalDefinition {
  type: 'statistical';
  metric: 'mean' | 'median' | 'stddev' | 'variance' | 'min' | 'max' | 'percentile';
  percentileValue?: number;
  expectedRange: { min: number; max: number };
  baseline?: number;
  allowedDeviation?: number;
}

export type RuleSeverity =
  | 'critical'
  | 'major'
  | 'minor'
  | 'info';

export interface RuleMetadata {
  owner?: string;
  createdAt?: string;
  lastModified?: string;
  documentation?: string;
  linkedIssues?: string[];
}

// ============================================================================
// Quality Check Types
// ============================================================================

export interface QualityCheck {
  id: string;
  name: string;
  type: CheckType;
  schedule: CheckSchedule;
  rules: string[];
  configuration: CheckConfiguration;
  notifications?: NotificationConfig;
  lastRun?: CheckRunSummary;
}

export type CheckType =
  | 'scheduled'
  | 'on-demand'
  | 'pipeline-triggered'
  | 'continuous';

export interface CheckSchedule {
  type: 'cron' | 'interval' | 'event';
  expression?: string;
  timezone?: string;
  interval?: number;
  eventTrigger?: string;
}

export interface CheckConfiguration {
  samplingStrategy?: SamplingStrategy;
  parallelism?: number;
  timeout?: number;
  retryPolicy?: RetryPolicy;
}

export interface SamplingStrategy {
  type: 'full' | 'random' | 'stratified' | 'time-based';
  sampleSize?: number;
  samplePercentage?: number;
  stratifyColumns?: string[];
  timePeriod?: string;
}

export interface RetryPolicy {
  maxRetries: number;
  retryDelay: number;
  exponentialBackoff?: boolean;
}

export interface NotificationConfig {
  channels: NotificationChannel[];
  conditions: NotificationCondition[];
}

export interface NotificationChannel {
  type: 'email' | 'slack' | 'pagerduty' | 'webhook' | 'sms';
  target: string;
  template?: string;
}

export interface NotificationCondition {
  trigger: 'failure' | 'warning' | 'recovery' | 'always';
  ruleIds?: string[];
  dimensions?: DimensionName[];
}

export interface CheckRunSummary {
  runId: string;
  status: RunStatus;
  startTime: string;
  endTime?: string;
  totalRules: number;
  passedRules: number;
  failedRules: number;
  overallScore: number;
}

export type RunStatus =
  | 'running'
  | 'completed'
  | 'failed'
  | 'cancelled'
  | 'timeout';

// ============================================================================
// Quality Report Types
// ============================================================================

export interface QualityReport {
  id: string;
  checkId: string;
  runId: string;
  generatedAt: string;
  summary: ReportSummary;
  dimensionScores: DimensionScore[];
  ruleResults: RuleResult[];
  anomalies?: Anomaly[];
  recommendations?: Recommendation[];
}

export interface ReportSummary {
  overallScore: number;
  trend: 'improving' | 'stable' | 'declining';
  totalRecords: number;
  sampledRecords: number;
  passRate: number;
  issueCount: number;
  criticalIssues: number;
}

export interface DimensionScore {
  dimension: DimensionName;
  score: number;
  previousScore?: number;
  trend: 'improving' | 'stable' | 'declining';
  weight: number;
  contribution: number;
}

export interface RuleResult {
  ruleId: string;
  ruleName: string;
  status: 'passed' | 'failed' | 'warning' | 'error' | 'skipped';
  score: number;
  affectedRecords: number;
  totalRecords: number;
  failureRate: number;
  executionTime: number;
  details?: RuleResultDetail[];
  samples?: FailureSample[];
}

export interface RuleResultDetail {
  column?: string;
  metric: string;
  expectedValue?: unknown;
  actualValue?: unknown;
  deviation?: number;
}

export interface FailureSample {
  recordId?: string;
  values: Record<string, unknown>;
  failureReason: string;
}

export interface Anomaly {
  id: string;
  type: AnomalyType;
  severity: RuleSeverity;
  dimension: DimensionName;
  description: string;
  detectedAt: string;
  affectedColumns?: string[];
  statisticalDetails?: StatisticalAnomaly;
}

export type AnomalyType =
  | 'sudden-change'
  | 'trend-deviation'
  | 'outlier'
  | 'missing-data-spike'
  | 'distribution-shift';

export interface StatisticalAnomaly {
  metric: string;
  expectedValue: number;
  actualValue: number;
  deviation: number;
  confidence: number;
}

export interface Recommendation {
  id: string;
  priority: 'high' | 'medium' | 'low';
  category: 'fix' | 'investigate' | 'monitor' | 'optimize';
  title: string;
  description: string;
  affectedRules?: string[];
  estimatedImpact?: string;
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface ProfileResponse {
  id: string;
  name: string;
  status: ProfileStatus;
  currentScore: number;
  lastCheckAt?: string;
  createdAt: string;
  links: {
    self: string;
    checks?: string;
    reports?: string;
  };
}

export interface CheckRunRequest {
  checkId: string;
  profileId?: string;
  ruleIds?: string[];
  samplingOverride?: SamplingStrategy;
  priority?: 'low' | 'normal' | 'high';
}

export interface CheckRunResponse {
  runId: string;
  status: RunStatus;
  startTime: string;
  estimatedCompletion?: string;
}

// ============================================================================
// Utility Types
// ============================================================================

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
  timestamp: string;
  requestId?: string;
}

export interface ValidationResult {
  valid: boolean;
  errors?: ValidationError[];
}

export interface ValidationError {
  path: string;
  message: string;
  value?: unknown;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    total: number;
    limit: number;
    offset: number;
    hasMore: boolean;
  };
  links: {
    first?: string;
    prev?: string;
    self: string;
    next?: string;
    last?: string;
  };
}
