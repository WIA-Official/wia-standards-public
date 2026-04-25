/**
 * WIA DDoS Protection Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core DDoS Protection Types
// ============================================================================

export interface WIADDoSProtection {
  standard: 'WIA-DDOS-PROTECTION';
  version: string;
  service: ServiceMetadata;
  protection: ProtectionConfiguration;
  detection: DetectionConfiguration;
  mitigation: MitigationConfiguration;
  monitoring?: MonitoringConfiguration;
  extensions?: Record<string, unknown>;
}

export interface ServiceMetadata {
  id: string;
  name: string;
  description?: string;
  owner: string;
  createdAt: string;
  updatedAt?: string;
  status: ServiceStatus;
  tier: ServiceTier;
}

export type ServiceStatus =
  | 'active'
  | 'standby'
  | 'under-attack'
  | 'maintenance'
  | 'disabled';

export type ServiceTier =
  | 'basic'
  | 'standard'
  | 'advanced'
  | 'enterprise';

// ============================================================================
// Protection Configuration Types
// ============================================================================

export interface ProtectionConfiguration {
  enabled: boolean;
  mode: ProtectionMode;
  targets: ProtectionTarget[];
  policies: ProtectionPolicy[];
  rateLimits: RateLimitConfig[];
  geoBlocking?: GeoBlockingConfig;
  ipLists?: IPListConfig;
}

export type ProtectionMode =
  | 'detection-only'
  | 'always-on'
  | 'auto-escalate'
  | 'manual';

export interface ProtectionTarget {
  id: string;
  type: TargetType;
  identifier: string;
  port?: number;
  protocol?: Protocol;
  priority: number;
}

export type TargetType =
  | 'domain'
  | 'ip-address'
  | 'ip-range'
  | 'network'
  | 'load-balancer'
  | 'cdn-origin';

export type Protocol =
  | 'tcp'
  | 'udp'
  | 'http'
  | 'https'
  | 'dns'
  | 'all';

export interface ProtectionPolicy {
  id: string;
  name: string;
  type: PolicyType;
  rules: PolicyRule[];
  actions: PolicyAction[];
  enabled: boolean;
}

export type PolicyType =
  | 'layer3'
  | 'layer4'
  | 'layer7'
  | 'hybrid';

export interface PolicyRule {
  id: string;
  condition: RuleCondition;
  threshold?: Threshold;
  duration?: number;
}

export interface RuleCondition {
  field: string;
  operator: ConditionOperator;
  value: string | number | string[];
}

export type ConditionOperator =
  | 'equals'
  | 'not-equals'
  | 'contains'
  | 'not-contains'
  | 'starts-with'
  | 'ends-with'
  | 'greater-than'
  | 'less-than'
  | 'in'
  | 'not-in'
  | 'regex';

export interface Threshold {
  value: number;
  unit: ThresholdUnit;
  window: number;
}

export type ThresholdUnit =
  | 'requests-per-second'
  | 'connections-per-second'
  | 'bytes-per-second'
  | 'packets-per-second';

export interface PolicyAction {
  type: ActionType;
  duration?: number;
  parameters?: Record<string, unknown>;
}

export type ActionType =
  | 'allow'
  | 'block'
  | 'challenge'
  | 'rate-limit'
  | 'redirect'
  | 'log'
  | 'alert';

export interface RateLimitConfig {
  id: string;
  name: string;
  scope: RateLimitScope;
  limit: number;
  window: number;
  action: ActionType;
  bypassConditions?: RuleCondition[];
}

export type RateLimitScope =
  | 'global'
  | 'per-ip'
  | 'per-session'
  | 'per-path'
  | 'per-country';

export interface GeoBlockingConfig {
  enabled: boolean;
  mode: 'allow-list' | 'block-list';
  countries: string[];
  regions?: string[];
  exceptIPs?: string[];
}

export interface IPListConfig {
  allowList: IPEntry[];
  blockList: IPEntry[];
  autoBlockDuration?: number;
}

export interface IPEntry {
  ip: string;
  description?: string;
  expiresAt?: string;
  addedAt: string;
  source: 'manual' | 'auto-detected' | 'threat-intel';
}

// ============================================================================
// Detection Configuration Types
// ============================================================================

export interface DetectionConfiguration {
  enabled: boolean;
  algorithms: DetectionAlgorithm[];
  baselines: BaselineConfig;
  anomalyDetection: AnomalyDetectionConfig;
  signatureDetection: SignatureDetectionConfig;
  machineLearning?: MLDetectionConfig;
}

export interface DetectionAlgorithm {
  id: string;
  type: AlgorithmType;
  enabled: boolean;
  sensitivity: Sensitivity;
  parameters?: Record<string, unknown>;
}

export type AlgorithmType =
  | 'rate-based'
  | 'pattern-based'
  | 'behavioral'
  | 'volumetric'
  | 'protocol-anomaly'
  | 'application-layer';

export type Sensitivity =
  | 'low'
  | 'medium'
  | 'high'
  | 'very-high';

export interface BaselineConfig {
  learningPeriod: number;
  updateInterval: number;
  metrics: BaselineMetric[];
}

export interface BaselineMetric {
  name: string;
  type: 'average' | 'percentile' | 'max';
  percentileValue?: number;
  deviationThreshold: number;
}

export interface AnomalyDetectionConfig {
  enabled: boolean;
  methods: AnomalyMethod[];
  alertThreshold: number;
  mitigationThreshold: number;
}

export type AnomalyMethod =
  | 'statistical'
  | 'clustering'
  | 'time-series'
  | 'entropy';

export interface SignatureDetectionConfig {
  enabled: boolean;
  signatureSets: string[];
  customSignatures?: CustomSignature[];
  updateFrequency: number;
}

export interface CustomSignature {
  id: string;
  name: string;
  pattern: string;
  protocol: Protocol;
  severity: Severity;
}

export type Severity =
  | 'info'
  | 'low'
  | 'medium'
  | 'high'
  | 'critical';

export interface MLDetectionConfig {
  enabled: boolean;
  modelType: 'supervised' | 'unsupervised' | 'hybrid';
  modelVersion?: string;
  confidenceThreshold: number;
  retrainingSchedule?: string;
}

// ============================================================================
// Mitigation Configuration Types
// ============================================================================

export interface MitigationConfiguration {
  automaticResponse: boolean;
  escalationLevels: EscalationLevel[];
  techniques: MitigationTechnique[];
  scrubbing?: ScrubbingConfig;
  cdn?: CDNConfig;
}

export interface EscalationLevel {
  level: number;
  name: string;
  triggers: EscalationTrigger[];
  actions: MitigationAction[];
  autoDeescalate?: boolean;
  deescalateAfter?: number;
}

export interface EscalationTrigger {
  metric: string;
  threshold: number;
  duration?: number;
}

export interface MitigationAction {
  type: MitigationActionType;
  parameters: Record<string, unknown>;
  priority: number;
}

export type MitigationActionType =
  | 'rate-limit'
  | 'blackhole'
  | 'scrubbing-center'
  | 'challenge'
  | 'geo-block'
  | 'protocol-filter'
  | 'application-firewall';

export interface MitigationTechnique {
  id: string;
  type: TechniqueType;
  enabled: boolean;
  config: Record<string, unknown>;
}

export type TechniqueType =
  | 'syn-cookies'
  | 'connection-limiting'
  | 'packet-filtering'
  | 'traffic-shaping'
  | 'dns-sinkhole'
  | 'http-challenge'
  | 'captcha';

export interface ScrubbingConfig {
  enabled: boolean;
  centers: ScrubbingCenter[];
  routingMode: 'bgp' | 'dns' | 'gre';
  capacityGbps: number;
}

export interface ScrubbingCenter {
  id: string;
  name: string;
  location: string;
  capacity: number;
  active: boolean;
}

export interface CDNConfig {
  enabled: boolean;
  provider: string;
  edgeNodes: number;
  cacheRules?: CacheRule[];
}

export interface CacheRule {
  path: string;
  ttl: number;
  bypassOnAttack?: boolean;
}

// ============================================================================
// Monitoring Configuration Types
// ============================================================================

export interface MonitoringConfiguration {
  realTimeMetrics: MetricsConfig;
  alerting: AlertingConfig;
  logging: LoggingConfig;
  reporting: ReportingConfig;
}

export interface MetricsConfig {
  enabled: boolean;
  interval: number;
  retention: number;
  metrics: string[];
}

export interface AlertingConfig {
  channels: AlertChannel[];
  rules: AlertRule[];
  escalation?: AlertEscalation;
}

export interface AlertChannel {
  type: 'email' | 'sms' | 'webhook' | 'slack' | 'pagerduty';
  target: string;
  severity: Severity[];
}

export interface AlertRule {
  id: string;
  name: string;
  condition: string;
  severity: Severity;
  cooldown?: number;
}

export interface AlertEscalation {
  enabled: boolean;
  levels: { delay: number; channels: string[] }[];
}

export interface LoggingConfig {
  enabled: boolean;
  level: 'debug' | 'info' | 'warn' | 'error';
  destination: string;
  retention: number;
  includeHeaders?: boolean;
  includeBody?: boolean;
}

export interface ReportingConfig {
  enabled: boolean;
  schedule: string;
  recipients: string[];
  format: 'pdf' | 'html' | 'json';
}

// ============================================================================
// Attack Types
// ============================================================================

export interface AttackEvent {
  id: string;
  type: AttackType;
  startTime: string;
  endTime?: string;
  status: AttackStatus;
  severity: Severity;
  source: AttackSource;
  target: ProtectionTarget;
  metrics: AttackMetrics;
  mitigation?: MitigationRecord;
}

export type AttackType =
  | 'syn-flood'
  | 'udp-flood'
  | 'icmp-flood'
  | 'http-flood'
  | 'slowloris'
  | 'dns-amplification'
  | 'ntp-amplification'
  | 'memcached-amplification'
  | 'ssdp-amplification'
  | 'application-layer'
  | 'mixed';

export type AttackStatus =
  | 'ongoing'
  | 'mitigated'
  | 'ended'
  | 'analyzing';

export interface AttackSource {
  type: 'single-ip' | 'botnet' | 'amplification' | 'unknown';
  ipCount?: number;
  topCountries?: { country: string; percentage: number }[];
  topASNs?: { asn: string; percentage: number }[];
}

export interface AttackMetrics {
  peakPps: number;
  peakBps: number;
  peakRps?: number;
  avgPps: number;
  avgBps: number;
  duration: number;
  packetsBlocked: number;
  bytesBlocked: number;
}

export interface MitigationRecord {
  techniques: string[];
  startTime: string;
  endTime?: string;
  effectiveness: number;
  falsePositiveRate?: number;
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface ServiceResponse {
  id: string;
  name: string;
  status: ServiceStatus;
  tier: ServiceTier;
  protectedTargets: number;
  lastAttack?: string;
  links: {
    self: string;
    attacks?: string;
    metrics?: string;
  };
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
