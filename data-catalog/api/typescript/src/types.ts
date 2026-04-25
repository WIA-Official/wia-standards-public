/**
 * WIA Data Catalog Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Data Catalog Types
// ============================================================================

export interface WIADataCatalogProject {
  standard: 'WIA-DATA-CATALOG';
  version: string;
  metadata: ProjectMetadata;
  catalog: CatalogConfiguration;
  assets: AssetManagement;
  discovery: DiscoveryConfig;
  lineage: LineageTracking;
  quality: DataQualityConfig;
  governance: GovernanceConfig;
  collaboration: CollaborationConfig;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  organization: Organization;
  domain: string;
  createdAt: string;
  updatedAt?: string;
  status: ProjectStatus;
}

export interface Organization {
  name: string;
  department?: string;
  contact: ContactInfo;
}

export interface ContactInfo {
  name: string;
  email: string;
  role: string;
}

export type ProjectStatus = 'active' | 'development' | 'archived';

// ============================================================================
// Catalog Configuration Types
// ============================================================================

export interface CatalogConfiguration {
  name: string;
  description?: string;
  namespaces: Namespace[];
  taxonomies: Taxonomy[];
  customAttributes: CustomAttribute[];
  searchConfig: SearchConfiguration;
}

export interface Namespace {
  id: string;
  name: string;
  description?: string;
  parent?: string;
  path: string;
  visibility: 'public' | 'internal' | 'restricted';
}

export interface Taxonomy {
  id: string;
  name: string;
  description?: string;
  categories: TaxonomyCategory[];
  hierarchical: boolean;
}

export interface TaxonomyCategory {
  id: string;
  name: string;
  description?: string;
  parent?: string;
  children?: string[];
  attributes?: Record<string, unknown>;
}

export interface CustomAttribute {
  id: string;
  name: string;
  type: 'string' | 'number' | 'boolean' | 'date' | 'enum' | 'json';
  required: boolean;
  enumValues?: string[];
  defaultValue?: unknown;
  validation?: string;
  assetTypes: AssetType[];
}

export interface SearchConfiguration {
  fullText: boolean;
  fuzzyMatching: boolean;
  synonyms: SynonymGroup[];
  boostFields: { field: string; boost: number }[];
  facets: string[];
}

export interface SynonymGroup {
  terms: string[];
  primary: string;
}

// ============================================================================
// Asset Management Types
// ============================================================================

export interface AssetManagement {
  assetTypes: AssetTypeDefinition[];
  schemas: SchemaRegistry;
  relationships: RelationshipConfig;
  versioning: VersioningConfig;
}

export interface AssetTypeDefinition {
  id: string;
  name: string;
  type: AssetType;
  icon?: string;
  color?: string;
  attributes: string[];
  requiredFields: string[];
  defaultTags?: string[];
}

export type AssetType = 'table' | 'view' | 'column' | 'database' | 'schema' | 'pipeline' | 'dashboard' | 'report' | 'model' | 'api' | 'file' | 'stream' | 'topic';

export interface SchemaRegistry {
  enabled: boolean;
  format: 'avro' | 'json-schema' | 'protobuf' | 'auto';
  compatibility: 'backward' | 'forward' | 'full' | 'none';
  evolution: boolean;
}

export interface RelationshipConfig {
  types: RelationshipType[];
  autoDetection: boolean;
  inference: InferenceConfig;
}

export interface RelationshipType {
  id: string;
  name: string;
  sourceTypes: AssetType[];
  targetTypes: AssetType[];
  cardinality: 'one-to-one' | 'one-to-many' | 'many-to-many';
  properties?: string[];
}

export interface InferenceConfig {
  enabled: boolean;
  methods: ('schema' | 'naming' | 'profiling' | 'ml')[];
  confidence: number;
}

export interface VersioningConfig {
  enabled: boolean;
  retentionPolicy: string;
  diffTracking: boolean;
  snapshotFrequency?: string;
}

// ============================================================================
// Data Asset Types
// ============================================================================

export interface DataAsset {
  id: string;
  qualifiedName: string;
  type: AssetType;
  name: string;
  description?: string;
  namespace: string;
  owner: string;
  steward?: string;
  tags: string[];
  classifications: string[];
  attributes: Record<string, unknown>;
  schema?: SchemaDefinition;
  statistics?: AssetStatistics;
  quality?: QualityScore;
  lineage?: AssetLineage;
  createdAt: string;
  updatedAt: string;
  status: 'active' | 'deprecated' | 'archived';
}

export interface SchemaDefinition {
  columns: ColumnDefinition[];
  primaryKey?: string[];
  foreignKeys?: ForeignKey[];
  indexes?: IndexDefinition[];
  partitioning?: PartitionConfig;
}

export interface ColumnDefinition {
  name: string;
  type: string;
  nullable: boolean;
  description?: string;
  defaultValue?: unknown;
  isPrimaryKey?: boolean;
  isForeignKey?: boolean;
  classifications?: string[];
  statistics?: ColumnStatistics;
}

export interface ForeignKey {
  columns: string[];
  referenceTable: string;
  referenceColumns: string[];
  name?: string;
}

export interface IndexDefinition {
  name: string;
  columns: string[];
  unique: boolean;
  type: 'btree' | 'hash' | 'fulltext' | 'spatial';
}

export interface PartitionConfig {
  columns: string[];
  type: 'range' | 'list' | 'hash';
  granularity?: string;
}

export interface ColumnStatistics {
  distinctCount?: number;
  nullCount?: number;
  minValue?: unknown;
  maxValue?: unknown;
  avgLength?: number;
  sampleValues?: unknown[];
}

export interface AssetStatistics {
  rowCount?: number;
  sizeBytes?: number;
  lastModified?: string;
  lastAccessed?: string;
  accessCount?: number;
  popularityScore?: number;
}

export interface QualityScore {
  overall: number;
  dimensions: { dimension: string; score: number }[];
  lastAssessed: string;
  issues: number;
}

export interface AssetLineage {
  upstream: LineageNode[];
  downstream: LineageNode[];
  transformations: Transformation[];
}

export interface LineageNode {
  assetId: string;
  qualifiedName: string;
  type: AssetType;
  relationship: string;
}

export interface Transformation {
  id: string;
  type: string;
  description?: string;
  logic?: string;
  columns?: { source: string; target: string }[];
}

// ============================================================================
// Discovery & Search Types
// ============================================================================

export interface DiscoveryConfig {
  connectors: Connector[];
  scanSchedule: ScanSchedule;
  profiling: ProfilingConfig;
  classification: ClassificationConfig;
}

export interface Connector {
  id: string;
  name: string;
  type: ConnectorType;
  config: Record<string, unknown>;
  credentials: CredentialRef;
  enabled: boolean;
  schedule?: string;
}

export type ConnectorType = 'database' | 'data-warehouse' | 'data-lake' | 'bi-tool' | 'api' | 'file-system' | 'cloud-storage' | 'streaming' | 'custom';

export interface CredentialRef {
  type: 'secret-manager' | 'vault' | 'env';
  reference: string;
}

export interface ScanSchedule {
  fullScan: string;
  incrementalScan: string;
  metadataRefresh: string;
  statisticsRefresh: string;
}

export interface ProfilingConfig {
  enabled: boolean;
  sampleSize: number;
  columns: 'all' | 'selected' | 'none';
  selectedColumns?: string[];
  metrics: ('cardinality' | 'nulls' | 'distribution' | 'patterns')[];
}

export interface ClassificationConfig {
  enabled: boolean;
  rules: ClassificationRule[];
  patterns: ClassificationPattern[];
  mlEnabled: boolean;
}

export interface ClassificationRule {
  id: string;
  name: string;
  classification: string;
  conditions: RuleCondition[];
}

export interface RuleCondition {
  field: 'name' | 'type' | 'pattern' | 'values';
  operator: 'equals' | 'contains' | 'matches' | 'in';
  value: string | string[];
}

export interface ClassificationPattern {
  id: string;
  name: string;
  classification: string;
  regex: string;
  confidence: number;
}

export interface SearchQuery {
  query: string;
  filters?: SearchFilter[];
  facets?: string[];
  sort?: { field: string; order: 'asc' | 'desc' };
  from?: number;
  size?: number;
}

export interface SearchFilter {
  field: string;
  operator: 'eq' | 'ne' | 'in' | 'range' | 'exists';
  value: unknown;
}

export interface SearchResult {
  assets: DataAsset[];
  total: number;
  facets: Record<string, FacetValue[]>;
  suggestions?: string[];
}

export interface FacetValue {
  value: string;
  count: number;
}

// ============================================================================
// Lineage Tracking Types
// ============================================================================

export interface LineageTracking {
  enabled: boolean;
  sources: LineageSource[];
  granularity: 'dataset' | 'column' | 'row';
  retention: string;
  visualization: LineageVisualization;
}

export interface LineageSource {
  type: 'sql' | 'spark' | 'airflow' | 'dbt' | 'custom';
  config: Record<string, unknown>;
  enabled: boolean;
}

export interface LineageVisualization {
  layout: 'hierarchical' | 'force-directed' | 'dagre';
  depth: number;
  expandable: boolean;
  impactAnalysis: boolean;
}

export interface LineageGraph {
  nodes: LineageGraphNode[];
  edges: LineageEdge[];
}

export interface LineageGraphNode {
  id: string;
  qualifiedName: string;
  type: AssetType;
  name: string;
  depth: number;
  metadata?: Record<string, unknown>;
}

export interface LineageEdge {
  source: string;
  target: string;
  type: string;
  transformation?: string;
  columns?: { source: string[]; target: string[] };
}

// ============================================================================
// Data Quality Types
// ============================================================================

export interface DataQualityConfig {
  dimensions: QualityDimension[];
  rules: QualityRule[];
  monitoring: QualityMonitoring;
  alerts: QualityAlert[];
}

export interface QualityDimension {
  id: string;
  name: string;
  description: string;
  weight: number;
  metrics: string[];
}

export interface QualityRule {
  id: string;
  name: string;
  dimension: string;
  type: 'completeness' | 'accuracy' | 'validity' | 'consistency' | 'timeliness' | 'uniqueness';
  expression: string;
  threshold: number;
  severity: 'low' | 'medium' | 'high' | 'critical';
  assetScope: string[];
}

export interface QualityMonitoring {
  enabled: boolean;
  schedule: string;
  trending: boolean;
  anomalyDetection: boolean;
}

export interface QualityAlert {
  id: string;
  name: string;
  condition: string;
  channels: string[];
  severity: string;
}

// ============================================================================
// Governance & Collaboration Types
// ============================================================================

export interface GovernanceConfig {
  policies: GovernancePolicy[];
  accessControl: AccessControl;
  dataPrivacy: DataPrivacy;
  compliance: ComplianceConfig;
}

export interface GovernancePolicy {
  id: string;
  name: string;
  type: 'access' | 'retention' | 'usage' | 'quality';
  scope: string[];
  rules: PolicyRule[];
  enforcement: 'strict' | 'advisory';
}

export interface PolicyRule {
  condition: string;
  action: string;
  priority: number;
}

export interface AccessControl {
  model: 'rbac' | 'abac' | 'tag-based';
  roles: AccessRole[];
  permissions: Permission[];
}

export interface AccessRole {
  id: string;
  name: string;
  permissions: string[];
  dataScope?: string[];
}

export interface Permission {
  id: string;
  name: string;
  operations: ('read' | 'write' | 'admin' | 'export')[];
  resource: string;
}

export interface DataPrivacy {
  piiFields: string[];
  masking: MaskingPolicy[];
  encryption: EncryptionPolicy[];
}

export interface MaskingPolicy {
  id: string;
  name: string;
  classifications: string[];
  method: 'full' | 'partial' | 'hash' | 'tokenize';
  config?: Record<string, unknown>;
}

export interface EncryptionPolicy {
  id: string;
  classifications: string[];
  algorithm: string;
  keyManagement: string;
}

export interface ComplianceConfig {
  frameworks: string[];
  requirements: ComplianceRequirement[];
  auditing: AuditConfig;
}

export interface ComplianceRequirement {
  id: string;
  framework: string;
  requirement: string;
  controls: string[];
  status: 'compliant' | 'partial' | 'non-compliant';
}

export interface AuditConfig {
  enabled: boolean;
  events: string[];
  retention: string;
}

export interface CollaborationConfig {
  comments: boolean;
  ratings: boolean;
  bookmarks: boolean;
  wikis: boolean;
  notifications: NotificationConfig;
}

export interface NotificationConfig {
  channels: ('email' | 'slack' | 'teams')[];
  events: string[];
  subscriptions: boolean;
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface ProjectResponse {
  id: string;
  name: string;
  status: ProjectStatus;
  createdAt: string;
  updatedAt?: string;
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
}
