/**
 * WIA Data Lineage Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Lineage Types
// ============================================================================

export interface WIADataLineage {
  standard: 'WIA-DATA-LINEAGE';
  version: string;
  lineage: LineageMetadata;
  nodes: LineageNode[];
  edges: LineageEdge[];
  transformations: Transformation[];
  quality?: QualityMetrics;
  extensions?: Record<string, unknown>;
}

export interface LineageMetadata {
  id: string;
  name: string;
  description?: string;
  owner: string;
  createdAt: string;
  updatedAt?: string;
  tags?: string[];
  classification?: DataClassification;
}

export type DataClassification =
  | 'public'
  | 'internal'
  | 'confidential'
  | 'restricted'
  | 'pii'
  | 'phi';

// ============================================================================
// Node Types
// ============================================================================

export interface LineageNode {
  id: string;
  name: string;
  type: NodeType;
  system: SystemInfo;
  schema?: DataSchema;
  location?: DataLocation;
  metadata?: NodeMetadata;
}

export type NodeType =
  | 'source'
  | 'destination'
  | 'transformation'
  | 'staging'
  | 'archive'
  | 'external';

export interface SystemInfo {
  name: string;
  type: SystemType;
  version?: string;
  environment?: 'development' | 'staging' | 'production';
  connectionInfo?: ConnectionInfo;
}

export type SystemType =
  | 'database'
  | 'data-warehouse'
  | 'data-lake'
  | 'api'
  | 'file-system'
  | 'stream'
  | 'application'
  | 'etl-tool'
  | 'bi-tool';

export interface ConnectionInfo {
  host?: string;
  port?: number;
  database?: string;
  schema?: string;
  protocol?: string;
  encrypted?: boolean;
}

export interface DataSchema {
  format: SchemaFormat;
  fields: SchemaField[];
  primaryKeys?: string[];
  foreignKeys?: ForeignKey[];
}

export type SchemaFormat =
  | 'relational'
  | 'document'
  | 'columnar'
  | 'key-value'
  | 'graph'
  | 'time-series';

export interface SchemaField {
  name: string;
  type: string;
  nullable?: boolean;
  description?: string;
  sensitivity?: DataSensitivity;
  transformations?: string[];
}

export type DataSensitivity =
  | 'none'
  | 'low'
  | 'medium'
  | 'high'
  | 'critical';

export interface ForeignKey {
  column: string;
  references: {
    table: string;
    column: string;
  };
}

export interface DataLocation {
  path?: string;
  bucket?: string;
  container?: string;
  partition?: PartitionSpec;
}

export interface PartitionSpec {
  columns: string[];
  strategy: 'date' | 'hash' | 'range' | 'list';
  granularity?: 'hour' | 'day' | 'month' | 'year';
}

export interface NodeMetadata {
  owner?: string;
  steward?: string;
  domain?: string;
  dataProduct?: string;
  sla?: SLASpec;
  freshness?: FreshnessSpec;
}

export interface SLASpec {
  availability: number; // percentage
  latency?: number; // milliseconds
  errorRate?: number; // percentage
}

export interface FreshnessSpec {
  expectedDelay: number; // seconds
  alertThreshold: number; // seconds
  lastUpdated?: string;
}

// ============================================================================
// Edge Types
// ============================================================================

export interface LineageEdge {
  id: string;
  source: string; // node id
  target: string; // node id
  type: EdgeType;
  transformationId?: string;
  metadata?: EdgeMetadata;
}

export type EdgeType =
  | 'data-flow'
  | 'schema-derivation'
  | 'dependency'
  | 'reference'
  | 'aggregation';

export interface EdgeMetadata {
  description?: string;
  frequency?: DataFlowFrequency;
  volume?: DataVolume;
  latency?: LatencySpec;
}

export interface DataFlowFrequency {
  type: 'realtime' | 'batch' | 'micro-batch' | 'on-demand';
  schedule?: string; // cron expression
  interval?: number; // seconds
}

export interface DataVolume {
  avgRecords?: number;
  avgBytes?: number;
  peakRecords?: number;
  peakBytes?: number;
}

export interface LatencySpec {
  average: number; // milliseconds
  p95?: number;
  p99?: number;
}

// ============================================================================
// Transformation Types
// ============================================================================

export interface Transformation {
  id: string;
  name: string;
  type: TransformationType;
  description?: string;
  logic?: TransformationLogic;
  columnMappings?: ColumnMapping[];
  quality?: TransformationQuality;
}

export type TransformationType =
  | 'filter'
  | 'map'
  | 'aggregate'
  | 'join'
  | 'union'
  | 'pivot'
  | 'unpivot'
  | 'deduplicate'
  | 'enrich'
  | 'mask'
  | 'encrypt'
  | 'custom';

export interface TransformationLogic {
  language?: 'sql' | 'python' | 'scala' | 'spark' | 'dbt';
  code?: string;
  parameters?: Record<string, unknown>;
}

export interface ColumnMapping {
  source: ColumnReference;
  target: ColumnReference;
  transformation?: string;
  expression?: string;
}

export interface ColumnReference {
  node: string;
  column: string;
}

export interface TransformationQuality {
  tested?: boolean;
  testCoverage?: number;
  lastTestRun?: string;
  testResults?: TestResult[];
}

export interface TestResult {
  name: string;
  status: 'passed' | 'failed' | 'skipped';
  duration?: number;
  message?: string;
}

// ============================================================================
// Quality Metrics
// ============================================================================

export interface QualityMetrics {
  completeness?: CompletenessMetric;
  accuracy?: AccuracyMetric;
  consistency?: ConsistencyMetric;
  timeliness?: TimelinessMetric;
  validity?: ValidityMetric;
}

export interface CompletenessMetric {
  score: number; // 0-100
  nullRate?: number;
  missingFields?: string[];
}

export interface AccuracyMetric {
  score: number; // 0-100
  errorRate?: number;
  samplingMethod?: string;
}

export interface ConsistencyMetric {
  score: number; // 0-100
  inconsistencies?: Inconsistency[];
}

export interface Inconsistency {
  field: string;
  description: string;
  count?: number;
}

export interface TimelinessMetric {
  score: number; // 0-100
  avgDelay?: number; // seconds
  delayBreaches?: number;
}

export interface ValidityMetric {
  score: number; // 0-100
  validationRules?: ValidationRule[];
}

export interface ValidationRule {
  name: string;
  expression: string;
  passRate: number;
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface LineageResponse {
  id: string;
  name: string;
  nodeCount: number;
  edgeCount: number;
  createdAt: string;
  updatedAt?: string;
  links: {
    self: string;
    nodes?: string;
    edges?: string;
  };
}

export interface ImpactAnalysis {
  sourceNode: string;
  impactedNodes: ImpactedNode[];
  totalImpact: number;
}

export interface ImpactedNode {
  id: string;
  name: string;
  type: NodeType;
  distance: number; // hops from source
  path: string[];
  impact: 'direct' | 'indirect';
}

export interface LineageQuery {
  nodeId?: string;
  direction?: 'upstream' | 'downstream' | 'both';
  depth?: number;
  nodeTypes?: NodeType[];
  systems?: string[];
}

// ============================================================================
// Event Types
// ============================================================================

export interface LineageEvent {
  id: string;
  type: LineageEventType;
  timestamp: string;
  actor: string;
  nodeId?: string;
  edgeId?: string;
  details: Record<string, unknown>;
}

export type LineageEventType =
  | 'node-created'
  | 'node-updated'
  | 'node-deleted'
  | 'edge-created'
  | 'edge-deleted'
  | 'schema-changed'
  | 'data-flow-started'
  | 'data-flow-completed'
  | 'data-flow-failed';

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
