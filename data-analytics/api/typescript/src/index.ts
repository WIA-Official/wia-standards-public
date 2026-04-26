/**
 * WIA Data Analytics Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIADataAnalyticsProject, ProjectResponse, ValidationResult, PaginatedResponse,
  DataSource, ConnectionConfig, Pipeline, PipelineStage, DataCatalogEntry, KPIDefinition,
  AggregationConfig, Dashboard, Widget, ReportTemplate, ReportSchedule, PredictiveModel,
  ForecastConfig, AlertConfig, GovernancePolicy, AccessRole, FeatureSet
} from './types';

// ============================================================================
// WIA Data Analytics Client
// ============================================================================

export class WIADataAnalyticsClient {
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
  async createProject(project: WIADataAnalyticsProject): Promise<ProjectResponse> {
    return (await this.axios.post<ProjectResponse>('/projects', project)).data;
  }

  async getProject(id: string): Promise<WIADataAnalyticsProject> {
    return (await this.axios.get<WIADataAnalyticsProject>(`/projects/${id}`)).data;
  }

  async listProjects(params?: { status?: string; domain?: string; limit?: number }): Promise<PaginatedResponse<ProjectResponse>> {
    return (await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', { params })).data;
  }

  async updateProject(id: string, updates: Partial<WIADataAnalyticsProject>): Promise<ProjectResponse> {
    return (await this.axios.put<ProjectResponse>(`/projects/${id}`, updates)).data;
  }

  async deleteProject(id: string): Promise<void> {
    await this.axios.delete(`/projects/${id}`);
  }

  // Data Source Management
  async listDataSources(projectId: string, params?: { type?: string; status?: string }): Promise<DataSource[]> {
    return (await this.axios.get<DataSource[]>(`/projects/${projectId}/sources`, { params })).data;
  }

  async createDataSource(projectId: string, source: Partial<DataSource>): Promise<DataSource> {
    return (await this.axios.post<DataSource>(`/projects/${projectId}/sources`, source)).data;
  }

  async updateDataSource(projectId: string, sourceId: string, updates: Partial<DataSource>): Promise<DataSource> {
    return (await this.axios.put<DataSource>(`/projects/${projectId}/sources/${sourceId}`, updates)).data;
  }

  async testConnection(projectId: string, sourceId: string): Promise<{ success: boolean; message?: string; latency?: number }> {
    return (await this.axios.post<{ success: boolean; message?: string; latency?: number }>(`/projects/${projectId}/sources/${sourceId}/test`)).data;
  }

  async syncDataSource(projectId: string, sourceId: string): Promise<{ jobId: string; status: string }> {
    return (await this.axios.post<{ jobId: string; status: string }>(`/projects/${projectId}/sources/${sourceId}/sync`)).data;
  }

  // Connection Management
  async listConnections(projectId: string): Promise<ConnectionConfig[]> {
    return (await this.axios.get<ConnectionConfig[]>(`/projects/${projectId}/connections`)).data;
  }

  async createConnection(projectId: string, connection: Partial<ConnectionConfig>): Promise<ConnectionConfig> {
    return (await this.axios.post<ConnectionConfig>(`/projects/${projectId}/connections`, connection)).data;
  }

  async updateConnection(projectId: string, connectionId: string, updates: Partial<ConnectionConfig>): Promise<ConnectionConfig> {
    return (await this.axios.put<ConnectionConfig>(`/projects/${projectId}/connections/${connectionId}`, updates)).data;
  }

  async deleteConnection(projectId: string, connectionId: string): Promise<void> {
    await this.axios.delete(`/projects/${projectId}/connections/${connectionId}`);
  }

  // Data Catalog
  async listCatalogEntries(projectId: string, params?: { source?: string; tags?: string[] }): Promise<DataCatalogEntry[]> {
    return (await this.axios.get<DataCatalogEntry[]>(`/projects/${projectId}/catalog`, { params })).data;
  }

  async getCatalogEntry(projectId: string, entryId: string): Promise<DataCatalogEntry> {
    return (await this.axios.get<DataCatalogEntry>(`/projects/${projectId}/catalog/${entryId}`)).data;
  }

  async updateCatalogEntry(projectId: string, entryId: string, updates: Partial<DataCatalogEntry>): Promise<DataCatalogEntry> {
    return (await this.axios.put<DataCatalogEntry>(`/projects/${projectId}/catalog/${entryId}`, updates)).data;
  }

  async searchCatalog(projectId: string, query: { term: string; filters?: Record<string, unknown> }): Promise<DataCatalogEntry[]> {
    return (await this.axios.post<DataCatalogEntry[]>(`/projects/${projectId}/catalog/search`, query)).data;
  }

  async getDataLineage(projectId: string, entryId: string): Promise<{ upstream: DataCatalogEntry[]; downstream: DataCatalogEntry[] }> {
    return (await this.axios.get<{ upstream: DataCatalogEntry[]; downstream: DataCatalogEntry[] }>(`/projects/${projectId}/catalog/${entryId}/lineage`)).data;
  }

  // Pipeline Management
  async listPipelines(projectId: string, params?: { status?: string }): Promise<Pipeline[]> {
    return (await this.axios.get<Pipeline[]>(`/projects/${projectId}/pipelines`, { params })).data;
  }

  async createPipeline(projectId: string, pipeline: Partial<Pipeline>): Promise<Pipeline> {
    return (await this.axios.post<Pipeline>(`/projects/${projectId}/pipelines`, pipeline)).data;
  }

  async getPipeline(projectId: string, pipelineId: string): Promise<Pipeline> {
    return (await this.axios.get<Pipeline>(`/projects/${projectId}/pipelines/${pipelineId}`)).data;
  }

  async updatePipeline(projectId: string, pipelineId: string, updates: Partial<Pipeline>): Promise<Pipeline> {
    return (await this.axios.put<Pipeline>(`/projects/${projectId}/pipelines/${pipelineId}`, updates)).data;
  }

  async runPipeline(projectId: string, pipelineId: string, params?: Record<string, unknown>): Promise<{ runId: string; status: string }> {
    return (await this.axios.post<{ runId: string; status: string }>(`/projects/${projectId}/pipelines/${pipelineId}/run`, { params })).data;
  }

  async getPipelineRuns(projectId: string, pipelineId: string, limit?: number): Promise<PipelineRun[]> {
    return (await this.axios.get<PipelineRun[]>(`/projects/${projectId}/pipelines/${pipelineId}/runs`, { params: { limit } })).data;
  }

  async getPipelineRunStatus(projectId: string, pipelineId: string, runId: string): Promise<PipelineRun> {
    return (await this.axios.get<PipelineRun>(`/projects/${projectId}/pipelines/${pipelineId}/runs/${runId}`)).data;
  }

  async addPipelineStage(projectId: string, pipelineId: string, stage: Partial<PipelineStage>): Promise<PipelineStage> {
    return (await this.axios.post<PipelineStage>(`/projects/${projectId}/pipelines/${pipelineId}/stages`, stage)).data;
  }

  // KPI & Analytics
  async listKPIs(projectId: string): Promise<KPIDefinition[]> {
    return (await this.axios.get<KPIDefinition[]>(`/projects/${projectId}/analytics/kpis`)).data;
  }

  async createKPI(projectId: string, kpi: Partial<KPIDefinition>): Promise<KPIDefinition> {
    return (await this.axios.post<KPIDefinition>(`/projects/${projectId}/analytics/kpis`, kpi)).data;
  }

  async getKPIValue(projectId: string, kpiId: string, params?: { start?: string; end?: string; granularity?: string }): Promise<KPIValue> {
    return (await this.axios.get<KPIValue>(`/projects/${projectId}/analytics/kpis/${kpiId}/value`, { params })).data;
  }

  async createAggregation(projectId: string, aggregation: Partial<AggregationConfig>): Promise<AggregationConfig> {
    return (await this.axios.post<AggregationConfig>(`/projects/${projectId}/analytics/aggregations`, aggregation)).data;
  }

  async executeAggregation(projectId: string, aggregationId: string, params?: Record<string, unknown>): Promise<AggregationResult> {
    return (await this.axios.post<AggregationResult>(`/projects/${projectId}/analytics/aggregations/${aggregationId}/execute`, { params })).data;
  }

  async queryData(projectId: string, query: { sql?: string; dimensions?: string[]; measures?: string[]; filters?: Record<string, unknown> }): Promise<QueryResult> {
    return (await this.axios.post<QueryResult>(`/projects/${projectId}/analytics/query`, query)).data;
  }

  // Predictive Analytics
  async listPredictiveModels(projectId: string, params?: { type?: string; status?: string }): Promise<PredictiveModel[]> {
    return (await this.axios.get<PredictiveModel[]>(`/projects/${projectId}/analytics/predictive/models`, { params })).data;
  }

  async trainModel(projectId: string, config: { name: string; type: string; algorithm: string; features: string[]; target: string; data: string }): Promise<{ jobId: string; status: string }> {
    return (await this.axios.post<{ jobId: string; status: string }>(`/projects/${projectId}/analytics/predictive/train`, config)).data;
  }

  async getModelPrediction(projectId: string, modelId: string, input: Record<string, unknown>): Promise<{ prediction: unknown; confidence?: number }> {
    return (await this.axios.post<{ prediction: unknown; confidence?: number }>(`/projects/${projectId}/analytics/predictive/models/${modelId}/predict`, input)).data;
  }

  async listForecasts(projectId: string): Promise<ForecastConfig[]> {
    return (await this.axios.get<ForecastConfig[]>(`/projects/${projectId}/analytics/predictive/forecasts`)).data;
  }

  async createForecast(projectId: string, forecast: Partial<ForecastConfig>): Promise<ForecastConfig> {
    return (await this.axios.post<ForecastConfig>(`/projects/${projectId}/analytics/predictive/forecasts`, forecast)).data;
  }

  async getForecastResults(projectId: string, forecastId: string): Promise<ForecastResult> {
    return (await this.axios.get<ForecastResult>(`/projects/${projectId}/analytics/predictive/forecasts/${forecastId}/results`)).data;
  }

  // Dashboard & Visualization
  async listDashboards(projectId: string, params?: { visibility?: string }): Promise<Dashboard[]> {
    return (await this.axios.get<Dashboard[]>(`/projects/${projectId}/dashboards`, { params })).data;
  }

  async createDashboard(projectId: string, dashboard: Partial<Dashboard>): Promise<Dashboard> {
    return (await this.axios.post<Dashboard>(`/projects/${projectId}/dashboards`, dashboard)).data;
  }

  async getDashboard(projectId: string, dashboardId: string): Promise<Dashboard> {
    return (await this.axios.get<Dashboard>(`/projects/${projectId}/dashboards/${dashboardId}`)).data;
  }

  async updateDashboard(projectId: string, dashboardId: string, updates: Partial<Dashboard>): Promise<Dashboard> {
    return (await this.axios.put<Dashboard>(`/projects/${projectId}/dashboards/${dashboardId}`, updates)).data;
  }

  async addWidget(projectId: string, dashboardId: string, widget: Partial<Widget>): Promise<Widget> {
    return (await this.axios.post<Widget>(`/projects/${projectId}/dashboards/${dashboardId}/widgets`, widget)).data;
  }

  async updateWidget(projectId: string, dashboardId: string, widgetId: string, updates: Partial<Widget>): Promise<Widget> {
    return (await this.axios.put<Widget>(`/projects/${projectId}/dashboards/${dashboardId}/widgets/${widgetId}`, updates)).data;
  }

  async getEmbedToken(projectId: string, dashboardId: string): Promise<{ token: string; expiresAt: string }> {
    return (await this.axios.post<{ token: string; expiresAt: string }>(`/projects/${projectId}/dashboards/${dashboardId}/embed`)).data;
  }

  // Reporting
  async listReportTemplates(projectId: string): Promise<ReportTemplate[]> {
    return (await this.axios.get<ReportTemplate[]>(`/projects/${projectId}/reports/templates`)).data;
  }

  async createReportTemplate(projectId: string, template: Partial<ReportTemplate>): Promise<ReportTemplate> {
    return (await this.axios.post<ReportTemplate>(`/projects/${projectId}/reports/templates`, template)).data;
  }

  async generateReport(projectId: string, templateId: string, params?: Record<string, unknown>): Promise<{ reportId: string; status: string }> {
    return (await this.axios.post<{ reportId: string; status: string }>(`/projects/${projectId}/reports/generate`, { templateId, params })).data;
  }

  async getReportStatus(projectId: string, reportId: string): Promise<{ status: string; url?: string; error?: string }> {
    return (await this.axios.get<{ status: string; url?: string; error?: string }>(`/projects/${projectId}/reports/${reportId}`)).data;
  }

  async listReportSchedules(projectId: string): Promise<ReportSchedule[]> {
    return (await this.axios.get<ReportSchedule[]>(`/projects/${projectId}/reports/schedules`)).data;
  }

  async createReportSchedule(projectId: string, schedule: Partial<ReportSchedule>): Promise<ReportSchedule> {
    return (await this.axios.post<ReportSchedule>(`/projects/${projectId}/reports/schedules`, schedule)).data;
  }

  // Alerts
  async listAlerts(projectId: string, params?: { severity?: string; status?: string }): Promise<AlertConfig[]> {
    return (await this.axios.get<AlertConfig[]>(`/projects/${projectId}/alerts`, { params })).data;
  }

  async createAlert(projectId: string, alert: Partial<AlertConfig>): Promise<AlertConfig> {
    return (await this.axios.post<AlertConfig>(`/projects/${projectId}/alerts`, alert)).data;
  }

  async updateAlert(projectId: string, alertId: string, updates: Partial<AlertConfig>): Promise<AlertConfig> {
    return (await this.axios.put<AlertConfig>(`/projects/${projectId}/alerts/${alertId}`, updates)).data;
  }

  async getAlertHistory(projectId: string, alertId: string, limit?: number): Promise<AlertEvent[]> {
    return (await this.axios.get<AlertEvent[]>(`/projects/${projectId}/alerts/${alertId}/history`, { params: { limit } })).data;
  }

  // Governance
  async listGovernancePolicies(projectId: string): Promise<GovernancePolicy[]> {
    return (await this.axios.get<GovernancePolicy[]>(`/projects/${projectId}/governance/policies`)).data;
  }

  async createGovernancePolicy(projectId: string, policy: Partial<GovernancePolicy>): Promise<GovernancePolicy> {
    return (await this.axios.post<GovernancePolicy>(`/projects/${projectId}/governance/policies`, policy)).data;
  }

  async listAccessRoles(projectId: string): Promise<AccessRole[]> {
    return (await this.axios.get<AccessRole[]>(`/projects/${projectId}/governance/roles`)).data;
  }

  async createAccessRole(projectId: string, role: Partial<AccessRole>): Promise<AccessRole> {
    return (await this.axios.post<AccessRole>(`/projects/${projectId}/governance/roles`, role)).data;
  }

  async getAuditLog(projectId: string, params?: { start?: string; end?: string; event?: string; limit?: number }): Promise<PaginatedResponse<AuditLogEntry>> {
    return (await this.axios.get<PaginatedResponse<AuditLogEntry>>(`/projects/${projectId}/governance/audit`, { params })).data;
  }

  // Feature Store
  async listFeatureSets(projectId: string): Promise<FeatureSet[]> {
    return (await this.axios.get<FeatureSet[]>(`/projects/${projectId}/ml/features`)).data;
  }

  async createFeatureSet(projectId: string, featureSet: Partial<FeatureSet>): Promise<FeatureSet> {
    return (await this.axios.post<FeatureSet>(`/projects/${projectId}/ml/features`, featureSet)).data;
  }

  async getFeatureValues(projectId: string, featureSetId: string, entities: Record<string, unknown>[]): Promise<Record<string, unknown>[]> {
    return (await this.axios.post<Record<string, unknown>[]>(`/projects/${projectId}/ml/features/${featureSetId}/values`, { entities })).data;
  }

  // Validation
  validateProject(project: WIADataAnalyticsProject): ValidationResult {
    const errors: { path: string; message: string }[] = [];

    if (!project.standard || project.standard !== 'WIA-DATA-ANALYTICS') {
      errors.push({ path: 'standard', message: 'Standard must be "WIA-DATA-ANALYTICS"' });
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
    if (!project.dataSources) {
      errors.push({ path: 'dataSources', message: 'Data sources configuration is required' });
    }

    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

// ============================================================================
// Supporting Types
// ============================================================================

export interface PipelineRun {
  id: string;
  pipelineId: string;
  status: 'pending' | 'running' | 'completed' | 'failed' | 'cancelled';
  startedAt: string;
  completedAt?: string;
  stages: { stageId: string; status: string; duration?: number }[];
  metrics?: { rowsProcessed: number; bytesProcessed: number };
  error?: string;
}

export interface KPIValue {
  current: number;
  previous?: number;
  change?: number;
  changePercent?: number;
  trend?: 'up' | 'down' | 'stable';
  history?: { timestamp: string; value: number }[];
}

export interface AggregationResult {
  columns: string[];
  data: Record<string, unknown>[];
  rowCount: number;
  executionTime: number;
}

export interface QueryResult {
  columns: { name: string; type: string }[];
  data: unknown[][];
  rowCount: number;
  executionTime: number;
  truncated: boolean;
}

export interface ForecastResult {
  predictions: { timestamp: string; value: number; lowerBound: number; upperBound: number }[];
  accuracy: { mape: number; rmse: number };
  modelInfo: { algorithm: string; lastTrained: string };
}

export interface AlertEvent {
  id: string;
  alertId: string;
  timestamp: string;
  value: unknown;
  status: 'triggered' | 'resolved';
  notified: boolean;
}

export interface AuditLogEntry {
  id: string;
  timestamp: string;
  user: string;
  action: string;
  resource: string;
  details: Record<string, unknown>;
  ip?: string;
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

export function createMinimalProject(name: string, organization: string, domain: 'business-intelligence' | 'data-science' | 'operational' | 'customer' | 'financial' | 'marketing' | 'hr' | 'supply-chain' = 'business-intelligence'): WIADataAnalyticsProject {
  return {
    standard: 'WIA-DATA-ANALYTICS',
    version: '1.0.0',
    metadata: {
      id: generateUUID(),
      name,
      organization: { name: organization, industry: 'technology', contact: { name: '', email: '', role: 'Data Analyst' } },
      domain,
      createdAt: new Date().toISOString(),
      status: 'active',
    },
    dataSources: {
      sources: [],
      connections: [],
      ingestion: { mode: 'batch', validation: { schemaValidation: true, nullChecks: true, typeChecks: true, customRules: [] }, errorHandling: { strategy: 'quarantine', alerting: true } },
      catalog: [],
    },
    processing: {
      pipelines: [],
      transformations: { custom: [], sql: [], python: [] },
      scheduling: { scheduler: 'airflow', defaultTimezone: 'UTC', concurrencyLimit: 10 },
      orchestration: { platform: 'kubernetes' },
    },
    analytics: {
      descriptive: { kpis: [], aggregations: [], dimensions: [], measures: [] },
      diagnostic: { drillDown: [], correlation: [], segmentation: [], anomalyDetection: { enabled: true, methods: ['zscore'], sensitivity: 'medium', metrics: [] } },
      predictive: { models: [], forecasting: [], propensity: [] },
      prescriptive: { optimization: [], recommendations: [], simulations: [] },
      realtime: { enabled: false, streaming: { platform: 'kafka-streams', latency: '1s', windows: [] }, alerts: [], dashboardRefresh: '5m' },
    },
    visualization: {
      platform: { type: 'superset' },
      dashboards: [],
      themes: { primary: '#1890ff', secondary: '#52c41a', palette: [], fonts: { heading: 'Inter', body: 'Inter' } },
      embedding: { enabled: false, authentication: 'token', allowedDomains: [] },
    },
    reporting: {
      templates: [],
      scheduling: [],
      distribution: { channels: ['email'] },
      formats: ['pdf', 'excel'],
    },
    governance: {
      policies: [],
      accessControl: { model: 'rbac', roles: [], dataPermissions: [] },
      privacy: { piiFields: [], anonymization: { methods: [] }, consent: { required: false, purposes: [], tracking: false } },
      audit: { enabled: true, events: ['read', 'write', 'delete', 'export'], retention: '1 year', storage: '' },
    },
    mlIntegration: {
      platforms: [],
      featureStore: { enabled: false, platform: 'feast', features: [] },
      modelRegistry: { platform: '', versioning: true, lifecycle: ['development', 'staging', 'production'] },
      serving: { platform: 'kubernetes', scaling: { min: 1, max: 10 }, monitoring: true },
    },
  };
}

export default { WIADataAnalyticsClient, generateUUID, createMinimalProject };
