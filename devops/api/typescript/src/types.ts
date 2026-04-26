/**
 * WIA-COMP-011: DevOps - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Computing Research Group
 *
 * 弘익人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Environment types
 */
export type Environment = 'development' | 'staging' | 'production' | 'test';

/**
 * Deployment status
 */
export type DeploymentStatus =
  | 'pending'
  | 'in_progress'
  | 'success'
  | 'failed'
  | 'rolled_back';

/**
 * Deployment strategy
 */
export type DeploymentStrategy =
  | 'rolling'
  | 'blue-green'
  | 'canary'
  | 'recreate';

/**
 * Severity level
 */
export type SeverityLevel = 'low' | 'medium' | 'high' | 'critical';

// ============================================================================
// Pipeline Types
// ============================================================================

/**
 * CI/CD Pipeline configuration
 */
export interface PipelineConfig {
  /** Pipeline name */
  name: string;

  /** Pipeline stages */
  stages: PipelineStage[];

  /** Trigger events */
  triggers: TriggerEvent[];

  /** Target environment */
  environment: Environment;

  /** Timeout in seconds */
  timeout?: number;

  /** Environment variables */
  env?: Record<string, string>;

  /** Notification settings */
  notifications?: NotificationConfig;
}

/**
 * Pipeline stage
 */
export interface PipelineStage {
  /** Stage name */
  name: string;

  /** Stage type */
  type: 'build' | 'test' | 'security' | 'deploy' | 'custom';

  /** Commands to execute */
  commands: string[];

  /** Dependencies (other stages) */
  dependencies?: string[];

  /** Retry configuration */
  retry?: {
    attempts: number;
    backoff: number;
  };

  /** Artifact configuration */
  artifacts?: {
    paths: string[];
    expire: number;
  };
}

/**
 * Trigger event
 */
export type TriggerEvent =
  | 'push'
  | 'pull_request'
  | 'tag'
  | 'schedule'
  | 'manual'
  | 'webhook';

/**
 * Notification configuration
 */
export interface NotificationConfig {
  /** Notification channels */
  channels: ('email' | 'slack' | 'webhook')[];

  /** On success notification */
  onSuccess?: boolean;

  /** On failure notification */
  onFailure?: boolean;

  /** Recipients */
  recipients?: string[];
}

/**
 * Pipeline execution result
 */
export interface PipelineResult {
  /** Execution ID */
  id: string;

  /** Pipeline name */
  name: string;

  /** Execution status */
  status: 'success' | 'failed' | 'running' | 'cancelled';

  /** Start time */
  startTime: Date;

  /** End time */
  endTime?: Date;

  /** Duration in seconds */
  duration: number;

  /** Stage results */
  stages: StageResult[];

  /** Logs */
  logs: string[];
}

/**
 * Stage execution result
 */
export interface StageResult {
  /** Stage name */
  name: string;

  /** Stage status */
  status: 'success' | 'failed' | 'skipped';

  /** Duration in seconds */
  duration: number;

  /** Output logs */
  output: string;

  /** Error message (if failed) */
  error?: string;
}

// ============================================================================
// Deployment Types
// ============================================================================

/**
 * Deployment configuration
 */
export interface DeploymentConfig {
  /** Application name */
  application: string;

  /** Environment */
  environment: Environment;

  /** Version/tag to deploy */
  version: string;

  /** Deployment strategy */
  strategy: DeploymentStrategy;

  /** Health check configuration */
  healthCheck?: HealthCheck;

  /** Rollback configuration */
  rollback?: RollbackConfig;

  /** Resource limits */
  resources?: ResourceConfig;
}

/**
 * Health check configuration
 */
export interface HealthCheck {
  /** Health check endpoint */
  endpoint: string;

  /** Check interval in seconds */
  interval: number;

  /** Timeout in seconds */
  timeout: number;

  /** Healthy threshold */
  healthyThreshold?: number;

  /** Unhealthy threshold */
  unhealthyThreshold?: number;
}

/**
 * Rollback configuration
 */
export interface RollbackConfig {
  /** Auto rollback on failure */
  automatic: boolean;

  /** Rollback threshold (error rate) */
  threshold?: number;

  /** Preserve database state */
  preserveData?: boolean;
}

/**
 * Resource configuration
 */
export interface ResourceConfig {
  /** CPU limit */
  cpu?: string;

  /** Memory limit */
  memory?: string;

  /** Replicas */
  replicas?: number;

  /** Auto-scaling */
  autoscaling?: {
    min: number;
    max: number;
    targetCPU: number;
  };
}

/**
 * Deployment result
 */
export interface DeploymentResult {
  /** Deployment ID */
  id: string;

  /** Application */
  application: string;

  /** Environment */
  environment: Environment;

  /** Version */
  version: string;

  /** Status */
  status: DeploymentStatus;

  /** Start time */
  startTime: Date;

  /** End time */
  endTime?: Date;

  /** Deployment URL */
  url?: string;

  /** Error message */
  error?: string;
}

// ============================================================================
// Infrastructure Types
// ============================================================================

/**
 * Infrastructure deployment configuration
 */
export interface InfrastructureConfig {
  /** Cloud provider */
  provider: 'aws' | 'gcp' | 'azure' | 'digitalocean' | 'custom';

  /** Template file path */
  template: string;

  /** Variables */
  variables?: Record<string, any>;

  /** Backend configuration */
  backend?: {
    type: string;
    config: Record<string, string>;
  };

  /** State management */
  state?: {
    location: string;
    locking: boolean;
  };
}

/**
 * Infrastructure deployment result
 */
export interface InfrastructureResult {
  /** Deployment ID */
  id: string;

  /** Provider */
  provider: string;

  /** Status */
  status: 'planned' | 'applying' | 'applied' | 'failed' | 'destroyed';

  /** Resources created/updated */
  resources: ResourceChange[];

  /** Output values */
  outputs?: Record<string, any>;

  /** Error message */
  error?: string;
}

/**
 * Resource change
 */
export interface ResourceChange {
  /** Resource type */
  type: string;

  /** Resource name */
  name: string;

  /** Action */
  action: 'create' | 'update' | 'delete' | 'no-change';

  /** Resource ID */
  id?: string;
}

// ============================================================================
// Monitoring Types
// ============================================================================

/**
 * System monitoring configuration
 */
export interface MonitoringConfig {
  /** Service name */
  service: string;

  /** Metrics to collect */
  metrics: MetricType[];

  /** Collection interval in ms */
  interval: number;

  /** Alert rules */
  alerts?: AlertRule[];

  /** Retention period in days */
  retention?: number;
}

/**
 * Metric type
 */
export type MetricType =
  | 'cpu'
  | 'memory'
  | 'disk'
  | 'network'
  | 'requests'
  | 'errors'
  | 'latency'
  | 'custom';

/**
 * Alert rule
 */
export interface AlertRule {
  /** Rule name */
  name: string;

  /** Metric to monitor */
  metric: MetricType;

  /** Condition */
  condition: 'above' | 'below' | 'equal';

  /** Threshold value */
  threshold: number;

  /** Evaluation period in seconds */
  period: number;

  /** Severity */
  severity: SeverityLevel;

  /** Notification channels */
  notify: string[];
}

/**
 * Monitoring result
 */
export interface MonitoringResult {
  /** Service name */
  service: string;

  /** Timestamp */
  timestamp: Date;

  /** System status */
  status: 'healthy' | 'degraded' | 'down';

  /** Metrics */
  metrics: MetricData[];

  /** Active alerts */
  alerts: Alert[];
}

/**
 * Metric data
 */
export interface MetricData {
  /** Metric type */
  type: MetricType;

  /** Metric value */
  value: number;

  /** Unit */
  unit: string;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Alert
 */
export interface Alert {
  /** Alert ID */
  id: string;

  /** Rule name */
  rule: string;

  /** Severity */
  severity: SeverityLevel;

  /** Message */
  message: string;

  /** Triggered at */
  triggeredAt: Date;

  /** Acknowledged */
  acknowledged: boolean;
}

// ============================================================================
// Security Types
// ============================================================================

/**
 * Security scan configuration
 */
export interface SecurityScanConfig {
  /** Scan target */
  target: string;

  /** Scan types */
  scanTypes: ScanType[];

  /** Minimum severity to report */
  minSeverity?: SeverityLevel;

  /** Fail on threshold */
  failOn?: SeverityLevel;

  /** Exclusions */
  exclude?: string[];
}

/**
 * Scan type
 */
export type ScanType =
  | 'sast'
  | 'dast'
  | 'sca'
  | 'container'
  | 'secrets'
  | 'iac';

/**
 * Security scan result
 */
export interface SecurityScanResult {
  /** Scan ID */
  id: string;

  /** Scan type */
  type: ScanType;

  /** Status */
  status: 'passed' | 'failed';

  /** Vulnerabilities found */
  vulnerabilities: Vulnerability[];

  /** Scan duration */
  duration: number;

  /** Summary */
  summary: {
    critical: number;
    high: number;
    medium: number;
    low: number;
  };
}

/**
 * Vulnerability
 */
export interface Vulnerability {
  /** Vulnerability ID */
  id: string;

  /** Title */
  title: string;

  /** Description */
  description: string;

  /** Severity */
  severity: SeverityLevel;

  /** Affected component */
  component: string;

  /** Version */
  version?: string;

  /** Fix version */
  fixVersion?: string;

  /** CVE ID */
  cve?: string;

  /** CVSS score */
  cvss?: number;
}

// ============================================================================
// Incident Types
// ============================================================================

/**
 * Incident
 */
export interface Incident {
  /** Incident ID */
  id: string;

  /** Title */
  title: string;

  /** Description */
  description: string;

  /** Severity */
  severity: SeverityLevel;

  /** Status */
  status: 'open' | 'investigating' | 'resolved' | 'closed';

  /** Affected service */
  service: string;

  /** Created at */
  createdAt: Date;

  /** Resolved at */
  resolvedAt?: Date;

  /** Assigned to */
  assignedTo?: string;

  /** Timeline */
  timeline: IncidentEvent[];
}

/**
 * Incident event
 */
export interface IncidentEvent {
  /** Event timestamp */
  timestamp: Date;

  /** Event type */
  type: 'created' | 'updated' | 'escalated' | 'resolved' | 'comment';

  /** Event message */
  message: string;

  /** User */
  user?: string;
}

/**
 * Incident creation parameters
 */
export interface CreateIncidentParams {
  /** Title */
  title: string;

  /** Description */
  description: string;

  /** Severity */
  severity: SeverityLevel;

  /** Service */
  service: string;

  /** Assignee */
  assignTo?: string;
}

// ============================================================================
// Metrics and KPIs
// ============================================================================

/**
 * DORA metrics
 */
export interface DORAMetrics {
  /** Deployment frequency */
  deploymentFrequency: {
    value: number;
    unit: 'per-day' | 'per-week' | 'per-month';
    level: 'elite' | 'high' | 'medium' | 'low';
  };

  /** Lead time for changes */
  leadTime: {
    value: number;
    unit: 'hours' | 'days' | 'weeks';
    level: 'elite' | 'high' | 'medium' | 'low';
  };

  /** Time to restore service */
  timeToRestore: {
    value: number;
    unit: 'hours' | 'days' | 'weeks';
    level: 'elite' | 'high' | 'medium' | 'low';
  };

  /** Change failure rate */
  changeFailureRate: {
    value: number;
    unit: 'percentage';
    level: 'elite' | 'high' | 'medium' | 'low';
  };
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * DevOps error codes
 */
export enum DevOpsErrorCode {
  PIPELINE_FAILED = 'D001',
  DEPLOYMENT_FAILED = 'D002',
  INFRASTRUCTURE_FAILED = 'D003',
  SECURITY_SCAN_FAILED = 'D004',
  MONITORING_FAILED = 'D005',
  INVALID_CONFIG = 'D006',
  RESOURCE_NOT_FOUND = 'D007',
  PERMISSION_DENIED = 'D008',
}

/**
 * DevOps error
 */
export class DevOpsError extends Error {
  constructor(
    public code: DevOpsErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'DevOpsError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  Environment,
  DeploymentStatus,
  DeploymentStrategy,
  SeverityLevel,
  PipelineConfig,
  PipelineStage,
  TriggerEvent,
  NotificationConfig,
  PipelineResult,
  StageResult,
  DeploymentConfig,
  HealthCheck,
  RollbackConfig,
  ResourceConfig,
  DeploymentResult,
  InfrastructureConfig,
  InfrastructureResult,
  ResourceChange,
  MonitoringConfig,
  MetricType,
  AlertRule,
  MonitoringResult,
  MetricData,
  Alert,
  SecurityScanConfig,
  ScanType,
  SecurityScanResult,
  Vulnerability,
  Incident,
  IncidentEvent,
  CreateIncidentParams,
  DORAMetrics,
};

export { DevOpsErrorCode, DevOpsError };
