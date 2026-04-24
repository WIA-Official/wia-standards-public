/**
 * WIA Data Lake Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIADataLakeProject, ProjectResponse, ValidationResult, PaginatedResponse,
  DataZone, DataLayer, IngestionSource, DataPipeline, PipelineStage, ProcessingEngine,
  DataTable, TableSchema, GovernancePolicy, AccessRole, AccessPolicy, AlertRule, LifecyclePolicy
} from './types';

// ============================================================================
// WIA Data Lake Client
// ============================================================================

export class WIADataLakeClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }),
      },
    });
  }

  // Project Management
  async createProject(project: WIADataLakeProject): Promise<ProjectResponse> {
    return (await this.axios.post<ProjectResponse>('/projects', project)).data;
  }

  async getProject(id: string): Promise<WIADataLakeProject> {
    return (await this.axios.get<WIADataLakeProject>(`/projects/${id}`)).data;
  }

  async listProjects(params?: { status?: string; environment?: string; limit?: number }): Promise<PaginatedResponse<ProjectResponse>> {
    return (await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', { params })).data;
  }

  async updateProject(id: string, updates: Partial<WIADataLakeProject>): Promise<ProjectResponse> {
    return (await this.axios.put<ProjectResponse>(`/projects/${id}`, updates)).data;
  }

  async deleteProject(id: string): Promise<void> {
    await this.axios.delete(`/projects/${id}`);
  }

  // Zone Management
  async listZones(projectId: string): Promise<DataZone[]> {
    return (await this.axios.get<DataZone[]>(`/projects/${projectId}/zones`)).data;
  }

  async createZone(projectId: string, zone: Partial<DataZone>): Promise<DataZone> {
    return (await this.axios.post<DataZone>(`/projects/${projectId}/zones`, zone)).data;
  }

  async getZone(projectId: string, zoneId: string): Promise<DataZone> {
    return (await this.axios.get<DataZone>(`/projects/${projectId}/zones/${zoneId}`)).data;
  }

  async updateZone(projectId: string, zoneId: string, updates: Partial<DataZone>): Promise<DataZone> {
    return (await this.axios.put<DataZone>(`/projects/${projectId}/zones/${zoneId}`, updates)).data;
  }

  async getZoneMetrics(projectId: string, zoneId: string): Promise<ZoneMetrics> {
    return (await this.axios.get<ZoneMetrics>(`/projects/${projectId}/zones/${zoneId}/metrics`)).data;
  }

  // Layer Management
  async listLayers(projectId: string, zoneId?: string): Promise<DataLayer[]> {
    return (await this.axios.get<DataLayer[]>(`/projects/${projectId}/layers`, { params: { zoneId } })).data;
  }

  async createLayer(projectId: string, layer: Partial<DataLayer>): Promise<DataLayer> {
    return (await this.axios.post<DataLayer>(`/projects/${projectId}/layers`, layer)).data;
  }

  async getLayer(projectId: string, layerId: string): Promise<DataLayer> {
    return (await this.axios.get<DataLayer>(`/projects/${projectId}/layers/${layerId}`)).data;
  }

  async updateLayer(projectId: string, layerId: string, updates: Partial<DataLayer>): Promise<DataLayer> {
    return (await this.axios.put<DataLayer>(`/projects/${projectId}/layers/${layerId}`, updates)).data;
  }

  // Table Management
  async listTables(projectId: string, params?: { zone?: string; layer?: string; limit?: number }): Promise<PaginatedResponse<DataTable>> {
    return (await this.axios.get<PaginatedResponse<DataTable>>(`/projects/${projectId}/tables`, { params })).data;
  }

  async createTable(projectId: string, table: Partial<DataTable>): Promise<DataTable> {
    return (await this.axios.post<DataTable>(`/projects/${projectId}/tables`, table)).data;
  }

  async getTable(projectId: string, tableId: string): Promise<DataTable> {
    return (await this.axios.get<DataTable>(`/projects/${projectId}/tables/${tableId}`)).data;
  }

  async updateTable(projectId: string, tableId: string, updates: Partial<DataTable>): Promise<DataTable> {
    return (await this.axios.put<DataTable>(`/projects/${projectId}/tables/${tableId}`, updates)).data;
  }

  async deleteTable(projectId: string, tableId: string): Promise<void> {
    await this.axios.delete(`/projects/${projectId}/tables/${tableId}`);
  }

  async getTableSchema(projectId: string, tableId: string): Promise<TableSchema> {
    return (await this.axios.get<TableSchema>(`/projects/${projectId}/tables/${tableId}/schema`)).data;
  }

  async updateTableSchema(projectId: string, tableId: string, schema: Partial<TableSchema>): Promise<TableSchema> {
    return (await this.axios.put<TableSchema>(`/projects/${projectId}/tables/${tableId}/schema`, schema)).data;
  }

  async previewTableData(projectId: string, tableId: string, limit?: number): Promise<DataPreviewResult> {
    return (await this.axios.get<DataPreviewResult>(`/projects/${projectId}/tables/${tableId}/preview`, { params: { limit } })).data;
  }

  async getTableStatistics(projectId: string, tableId: string): Promise<TableStatisticsResult> {
    return (await this.axios.get<TableStatisticsResult>(`/projects/${projectId}/tables/${tableId}/statistics`)).data;
  }

  async getTableHistory(projectId: string, tableId: string, limit?: number): Promise<TableVersion[]> {
    return (await this.axios.get<TableVersion[]>(`/projects/${projectId}/tables/${tableId}/history`, { params: { limit } })).data;
  }

  async restoreTableVersion(projectId: string, tableId: string, version: number): Promise<DataTable> {
    return (await this.axios.post<DataTable>(`/projects/${projectId}/tables/${tableId}/restore`, { version })).data;
  }

  // Ingestion Source Management
  async listIngestionSources(projectId: string, params?: { type?: string; status?: string }): Promise<IngestionSource[]> {
    return (await this.axios.get<IngestionSource[]>(`/projects/${projectId}/ingestion/sources`, { params })).data;
  }

  async createIngestionSource(projectId: string, source: Partial<IngestionSource>): Promise<IngestionSource> {
    return (await this.axios.post<IngestionSource>(`/projects/${projectId}/ingestion/sources`, source)).data;
  }

  async updateIngestionSource(projectId: string, sourceId: string, updates: Partial<IngestionSource>): Promise<IngestionSource> {
    return (await this.axios.put<IngestionSource>(`/projects/${projectId}/ingestion/sources/${sourceId}`, updates)).data;
  }

  async testIngestionSource(projectId: string, sourceId: string): Promise<ConnectionTestResult> {
    return (await this.axios.post<ConnectionTestResult>(`/projects/${projectId}/ingestion/sources/${sourceId}/test`)).data;
  }

  async triggerIngestion(projectId: string, sourceId: string, mode: 'full' | 'incremental'): Promise<IngestionJob> {
    return (await this.axios.post<IngestionJob>(`/projects/${projectId}/ingestion/sources/${sourceId}/ingest`, { mode })).data;
  }

  async getIngestionJobStatus(projectId: string, jobId: string): Promise<IngestionJob> {
    return (await this.axios.get<IngestionJob>(`/projects/${projectId}/ingestion/jobs/${jobId}`)).data;
  }

  async listIngestionJobs(projectId: string, params?: { sourceId?: string; status?: string; limit?: number }): Promise<PaginatedResponse<IngestionJob>> {
    return (await this.axios.get<PaginatedResponse<IngestionJob>>(`/projects/${projectId}/ingestion/jobs`, { params })).data;
  }

  // Pipeline Management
  async listPipelines(projectId: string, params?: { status?: string }): Promise<DataPipeline[]> {
    return (await this.axios.get<DataPipeline[]>(`/projects/${projectId}/pipelines`, { params })).data;
  }

  async createPipeline(projectId: string, pipeline: Partial<DataPipeline>): Promise<DataPipeline> {
    return (await this.axios.post<DataPipeline>(`/projects/${projectId}/pipelines`, pipeline)).data;
  }

  async getPipeline(projectId: string, pipelineId: string): Promise<DataPipeline> {
    return (await this.axios.get<DataPipeline>(`/projects/${projectId}/pipelines/${pipelineId}`)).data;
  }

  async updatePipeline(projectId: string, pipelineId: string, updates: Partial<DataPipeline>): Promise<DataPipeline> {
    return (await this.axios.put<DataPipeline>(`/projects/${projectId}/pipelines/${pipelineId}`, updates)).data;
  }

  async runPipeline(projectId: string, pipelineId: string): Promise<PipelineRun> {
    return (await this.axios.post<PipelineRun>(`/projects/${projectId}/pipelines/${pipelineId}/run`)).data;
  }

  async getPipelineRun(projectId: string, pipelineId: string, runId: string): Promise<PipelineRun> {
    return (await this.axios.get<PipelineRun>(`/projects/${projectId}/pipelines/${pipelineId}/runs/${runId}`)).data;
  }

  async listPipelineRuns(projectId: string, pipelineId: string, limit?: number): Promise<PipelineRun[]> {
    return (await this.axios.get<PipelineRun[]>(`/projects/${projectId}/pipelines/${pipelineId}/runs`, { params: { limit } })).data;
  }

  async addPipelineStage(projectId: string, pipelineId: string, stage: Partial<PipelineStage>): Promise<PipelineStage> {
    return (await this.axios.post<PipelineStage>(`/projects/${projectId}/pipelines/${pipelineId}/stages`, stage)).data;
  }

  // Processing Engine Management
  async listProcessingEngines(projectId: string): Promise<ProcessingEngine[]> {
    return (await this.axios.get<ProcessingEngine[]>(`/projects/${projectId}/processing/engines`)).data;
  }

  async createProcessingEngine(projectId: string, engine: Partial<ProcessingEngine>): Promise<ProcessingEngine> {
    return (await this.axios.post<ProcessingEngine>(`/projects/${projectId}/processing/engines`, engine)).data;
  }

  async getEngineStatus(projectId: string, engineId: string): Promise<EngineStatus> {
    return (await this.axios.get<EngineStatus>(`/projects/${projectId}/processing/engines/${engineId}/status`)).data;
  }

  async scaleEngine(projectId: string, engineId: string, nodes: number): Promise<ProcessingEngine> {
    return (await this.axios.post<ProcessingEngine>(`/projects/${projectId}/processing/engines/${engineId}/scale`, { nodes })).data;
  }

  // Catalog Operations
  async searchCatalog(projectId: string, query: { term: string; filters?: Record<string, unknown> }): Promise<CatalogSearchResult> {
    return (await this.axios.post<CatalogSearchResult>(`/projects/${projectId}/catalog/search`, query)).data;
  }

  async getTableLineage(projectId: string, tableId: string, depth?: number): Promise<LineageGraph> {
    return (await this.axios.get<LineageGraph>(`/projects/${projectId}/tables/${tableId}/lineage`, { params: { depth } })).data;
  }

  async classifyTable(projectId: string, tableId: string, classification: string): Promise<DataTable> {
    return (await this.axios.post<DataTable>(`/projects/${projectId}/tables/${tableId}/classify`, { classification })).data;
  }

  // Governance
  async listGovernancePolicies(projectId: string): Promise<GovernancePolicy[]> {
    return (await this.axios.get<GovernancePolicy[]>(`/projects/${projectId}/governance/policies`)).data;
  }

  async createGovernancePolicy(projectId: string, policy: Partial<GovernancePolicy>): Promise<GovernancePolicy> {
    return (await this.axios.post<GovernancePolicy>(`/projects/${projectId}/governance/policies`, policy)).data;
  }

  async listAccessRoles(projectId: string): Promise<AccessRole[]> {
    return (await this.axios.get<AccessRole[]>(`/projects/${projectId}/access/roles`)).data;
  }

  async createAccessRole(projectId: string, role: Partial<AccessRole>): Promise<AccessRole> {
    return (await this.axios.post<AccessRole>(`/projects/${projectId}/access/roles`, role)).data;
  }

  async listAccessPolicies(projectId: string): Promise<AccessPolicy[]> {
    return (await this.axios.get<AccessPolicy[]>(`/projects/${projectId}/access/policies`)).data;
  }

  async createAccessPolicy(projectId: string, policy: Partial<AccessPolicy>): Promise<AccessPolicy> {
    return (await this.axios.post<AccessPolicy>(`/projects/${projectId}/access/policies`, policy)).data;
  }

  async grantAccess(projectId: string, tableId: string, principal: string, permissions: string[]): Promise<void> {
    await this.axios.post(`/projects/${projectId}/tables/${tableId}/access/grant`, { principal, permissions });
  }

  async revokeAccess(projectId: string, tableId: string, principal: string): Promise<void> {
    await this.axios.post(`/projects/${projectId}/tables/${tableId}/access/revoke`, { principal });
  }

  // Data Quality
  async runQualityCheck(projectId: string, tableId: string): Promise<QualityCheckResult> {
    return (await this.axios.post<QualityCheckResult>(`/projects/${projectId}/tables/${tableId}/quality/check`)).data;
  }

  async getQualityScore(projectId: string, tableId: string): Promise<QualityScore> {
    return (await this.axios.get<QualityScore>(`/projects/${projectId}/tables/${tableId}/quality/score`)).data;
  }

  async getQualityTrend(projectId: string, tableId: string, period: string): Promise<QualityTrendPoint[]> {
    return (await this.axios.get<QualityTrendPoint[]>(`/projects/${projectId}/tables/${tableId}/quality/trend`, { params: { period } })).data;
  }

  // Storage & Lifecycle
  async listLifecyclePolicies(projectId: string): Promise<LifecyclePolicy[]> {
    return (await this.axios.get<LifecyclePolicy[]>(`/projects/${projectId}/storage/lifecycle`)).data;
  }

  async createLifecyclePolicy(projectId: string, policy: Partial<LifecyclePolicy>): Promise<LifecyclePolicy> {
    return (await this.axios.post<LifecyclePolicy>(`/projects/${projectId}/storage/lifecycle`, policy)).data;
  }

  async getStorageMetrics(projectId: string): Promise<StorageMetrics> {
    return (await this.axios.get<StorageMetrics>(`/projects/${projectId}/storage/metrics`)).data;
  }

  async optimizeStorage(projectId: string, tableId: string): Promise<OptimizationResult> {
    return (await this.axios.post<OptimizationResult>(`/projects/${projectId}/tables/${tableId}/optimize`)).data;
  }

  async compactTable(projectId: string, tableId: string): Promise<CompactionResult> {
    return (await this.axios.post<CompactionResult>(`/projects/${projectId}/tables/${tableId}/compact`)).data;
  }

  // Alerting
  async listAlertRules(projectId: string): Promise<AlertRule[]> {
    return (await this.axios.get<AlertRule[]>(`/projects/${projectId}/alerts/rules`)).data;
  }

  async createAlertRule(projectId: string, rule: Partial<AlertRule>): Promise<AlertRule> {
    return (await this.axios.post<AlertRule>(`/projects/${projectId}/alerts/rules`, rule)).data;
  }

  async getActiveAlerts(projectId: string): Promise<Alert[]> {
    return (await this.axios.get<Alert[]>(`/projects/${projectId}/alerts/active`)).data;
  }

  // Dashboard
  async getLakeDashboard(projectId: string): Promise<LakeDashboard> {
    return (await this.axios.get<LakeDashboard>(`/projects/${projectId}/dashboard`)).data;
  }

  // Validation
  validateProject(project: WIADataLakeProject): ValidationResult {
    const errors: { path: string; message: string }[] = [];

    if (!project.standard || project.standard !== 'WIA-DATA-LAKE') {
      errors.push({ path: 'standard', message: 'Standard must be "WIA-DATA-LAKE"' });
    }
    if (!project.version) {
      errors.push({ path: 'version', message: 'Version is required' });
    }
    if (!project.metadata?.id) {
      errors.push({ path: 'metadata.id', message: 'Project ID is required' });
    }
    if (!project.metadata?.name) {
      errors.push({ path: 'metadata.name', message: 'Project name is required' });
    }
    if (!project.architecture) {
      errors.push({ path: 'architecture', message: 'Lake architecture is required' });
    }
    if (!project.storage) {
      errors.push({ path: 'storage', message: 'Storage configuration is required' });
    }

    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

// ============================================================================
// Supporting Types
// ============================================================================

export interface ZoneMetrics {
  zoneId: string;
  tables: number;
  sizeBytes: number;
  rowCount: number;
  lastUpdated: string;
}

export interface DataPreviewResult {
  columns: { name: string; type: string }[];
  rows: unknown[][];
  totalRows: number;
}

export interface TableStatisticsResult {
  tableId: string;
  rowCount: number;
  sizeBytes: number;
  files: number;
  partitions: number;
  lastModified: string;
  columnStats: { column: string; nullCount: number; distinctCount: number }[];
}

export interface TableVersion {
  version: number;
  timestamp: string;
  operation: string;
  user?: string;
  changes: number;
}

export interface ConnectionTestResult {
  success: boolean;
  message?: string;
  latency?: number;
}

export interface IngestionJob {
  id: string;
  sourceId: string;
  mode: 'full' | 'incremental';
  status: 'pending' | 'running' | 'completed' | 'failed';
  startedAt: string;
  completedAt?: string;
  recordsIngested?: number;
  bytesIngested?: number;
  error?: string;
}

export interface PipelineRun {
  id: string;
  pipelineId: string;
  status: 'pending' | 'running' | 'completed' | 'failed';
  startedAt: string;
  completedAt?: string;
  stages: { stageId: string; status: string; duration?: number }[];
  metrics?: { recordsProcessed: number; bytesProcessed: number };
}

export interface EngineStatus {
  engineId: string;
  status: 'running' | 'stopped' | 'scaling';
  nodes: number;
  utilization: number;
  jobsRunning: number;
}

export interface CatalogSearchResult {
  tables: DataTable[];
  total: number;
  facets: Record<string, { value: string; count: number }[]>;
}

export interface LineageGraph {
  nodes: { id: string; type: string; name: string; layer?: string }[];
  edges: { source: string; target: string; type: string }[];
}

export interface QualityCheckResult {
  tableId: string;
  timestamp: string;
  score: number;
  checks: { rule: string; passed: boolean; value: number }[];
}

export interface QualityScore {
  overall: number;
  dimensions: { name: string; score: number }[];
  trend: 'up' | 'down' | 'stable';
}

export interface QualityTrendPoint {
  timestamp: string;
  score: number;
}

export interface StorageMetrics {
  totalSize: number;
  byZone: { zone: string; size: number }[];
  byTier: { tier: string; size: number; cost: number }[];
  growth: { period: string; added: number; deleted: number }[];
}

export interface OptimizationResult {
  tableId: string;
  beforeSize: number;
  afterSize: number;
  savings: number;
  duration: number;
}

export interface CompactionResult {
  tableId: string;
  filesBefore: number;
  filesAfter: number;
  duration: number;
}

export interface Alert {
  id: string;
  ruleId: string;
  severity: string;
  message: string;
  triggeredAt: string;
  tableId?: string;
}

export interface LakeDashboard {
  zones: { total: number; byPurpose: Record<string, number> };
  tables: { total: number; byLayer: Record<string, number> };
  storage: { total: number; byTier: Record<string, number> };
  ingestion: { last24h: number; recordsIngested: number };
  pipelines: { active: number; failed: number };
  quality: { avgScore: number; issues: number };
  cost: { current: number; projected: number };
}

// ============================================================================
// Utility Functions
// ============================================================================

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16);
  });
}

export function createMinimalProject(name: string, organization: string, cloudProvider: 'aws' | 'azure' | 'gcp' | 'hybrid' | 'on-premises' = 'aws'): WIADataLakeProject {
  return {
    standard: 'WIA-DATA-LAKE',
    version: '1.0.0',
    metadata: {
      id: generateUUID(),
      name,
      organization: { name: organization, cloudProvider, region: 'us-east-1', contact: { name: '', email: '', role: 'Data Lake Admin' } },
      environment: 'development',
      createdAt: new Date().toISOString(),
      status: 'active',
    },
    architecture: {
      pattern: { type: 'lakehouse', description: 'Modern lakehouse architecture', principles: [] },
      zones: [],
      layers: [],
      medallion: { enabled: true, bronze: { zone: 'raw', format: 'parquet', retention: '90d', transformations: [] }, silver: { zone: 'curated', format: 'delta', retention: '1y', transformations: [] }, gold: { zone: 'consumption', format: 'delta', retention: '2y', transformations: [] } },
    },
    storage: {
      primary: { type: 's3', bucket: '', credentials: { type: 'iam', reference: '' }, encryption: { enabled: true, type: 'sse-s3' }, versioning: true },
      tiering: { enabled: true, tiers: [{ name: 'hot', class: 'hot', transitionAfter: '0d', cost: 0.023 }, { name: 'warm', class: 'warm', transitionAfter: '90d', cost: 0.0125 }], automation: true },
      lifecycle: [],
    },
    ingestion: {
      sources: [],
      patterns: [],
      streaming: { enabled: false, platform: 'kafka', topics: [], processing: { engine: 'spark-streaming', checkpointing: true, watermark: '10 minutes', lateDataHandling: 'allow' } },
      batch: { scheduler: 'airflow', defaultSchedule: '0 0 * * *', parallelism: 4, retryPolicy: { maxAttempts: 3, backoff: 'exponential', initialDelay: '1m' } },
    },
    processing: {
      engines: [],
      pipelines: [],
      transformations: { builtin: [], custom: [], macros: [] },
      compute: { default: '', autoscaling: { enabled: true, minCapacity: 1, maxCapacity: 10, targetUtilization: 70, cooldown: '5m' }, scheduling: { fairScheduler: true, queues: [], priorities: [] } },
    },
    catalog: {
      platform: { type: 'unity-catalog', sync: true },
      discovery: { autoDiscovery: true, frequency: 'daily', profiling: true, classification: true },
      metadata: { businessMetadata: true, technicalMetadata: true, operationalMetadata: true, customAttributes: [] },
      search: { fullText: true, facets: ['zone', 'layer', 'owner', 'classification'], suggestions: true },
    },
    governance: {
      policies: [],
      classification: { levels: [], autoClassification: true, piiDetection: true },
      lineage: { enabled: true, granularity: 'column', sources: ['spark', 'dbt'], visualization: true },
      quality: { dimensions: [], rules: [], monitoring: { enabled: true, frequency: 'daily', alerting: true, trending: true } },
    },
    access: {
      model: { type: 'rbac', inheritance: true, defaultDeny: true },
      authentication: { providers: [], sso: true, mfa: true },
      authorization: { roles: [], policies: [], rowLevelSecurity: true, columnMasking: { enabled: true, rules: [] } },
      audit: { enabled: true, events: ['read', 'write', 'schema-change', 'access-grant'], retention: '7y', destination: '' },
    },
    monitoring: {
      metrics: { enabled: true, provider: 'cloudwatch', metrics: [], dashboards: [] },
      logging: { level: 'info', centralized: true, destination: '', retention: '90d' },
      alerting: { enabled: true, rules: [], channels: [] },
      costManagement: { tracking: true, budgets: [], optimization: { enabled: true, recommendations: true, autoImplement: false } },
    },
  };
}

export default { WIADataLakeClient, generateUUID, createMinimalProject };
