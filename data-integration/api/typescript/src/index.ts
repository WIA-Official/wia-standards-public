/**
 * WIA Data Integration Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIADataIntegrationProject, ProjectResponse, ValidationResult, PaginatedResponse,
  SourceConnector, TargetDestination, Pipeline, PipelineStage, Workflow, WorkflowTask,
  TriggerConfig, QualityCheck, FieldMapping, CustomTransform, PipelineRun, AlertRule
} from './types';

// ============================================================================
// WIA Data Integration Client
// ============================================================================

export class WIADataIntegrationClient {
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
  async createProject(project: WIADataIntegrationProject): Promise<ProjectResponse> {
    return (await this.axios.post<ProjectResponse>('/projects', project)).data;
  }

  async getProject(id: string): Promise<WIADataIntegrationProject> {
    return (await this.axios.get<WIADataIntegrationProject>(`/projects/${id}`)).data;
  }

  async listProjects(params?: { status?: string; environment?: string; limit?: number }): Promise<PaginatedResponse<ProjectResponse>> {
    return (await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', { params })).data;
  }

  async updateProject(id: string, updates: Partial<WIADataIntegrationProject>): Promise<ProjectResponse> {
    return (await this.axios.put<ProjectResponse>(`/projects/${id}`, updates)).data;
  }

  async deleteProject(id: string): Promise<void> {
    await this.axios.delete(`/projects/${id}`);
  }

  // Source Connector Management
  async listSourceConnectors(projectId: string, params?: { type?: string; status?: string }): Promise<SourceConnector[]> {
    return (await this.axios.get<SourceConnector[]>(`/projects/${projectId}/sources`, { params })).data;
  }

  async createSourceConnector(projectId: string, connector: Partial<SourceConnector>): Promise<SourceConnector> {
    return (await this.axios.post<SourceConnector>(`/projects/${projectId}/sources`, connector)).data;
  }

  async getSourceConnector(projectId: string, connectorId: string): Promise<SourceConnector> {
    return (await this.axios.get<SourceConnector>(`/projects/${projectId}/sources/${connectorId}`)).data;
  }

  async updateSourceConnector(projectId: string, connectorId: string, updates: Partial<SourceConnector>): Promise<SourceConnector> {
    return (await this.axios.put<SourceConnector>(`/projects/${projectId}/sources/${connectorId}`, updates)).data;
  }

  async deleteSourceConnector(projectId: string, connectorId: string): Promise<void> {
    await this.axios.delete(`/projects/${projectId}/sources/${connectorId}`);
  }

  async testSourceConnection(projectId: string, connectorId: string): Promise<ConnectionTestResult> {
    return (await this.axios.post<ConnectionTestResult>(`/projects/${projectId}/sources/${connectorId}/test`)).data;
  }

  async discoverSchema(projectId: string, connectorId: string): Promise<SchemaDiscoveryResult> {
    return (await this.axios.post<SchemaDiscoveryResult>(`/projects/${projectId}/sources/${connectorId}/discover`)).data;
  }

  async previewData(projectId: string, connectorId: string, table: string, limit?: number): Promise<DataPreviewResult> {
    return (await this.axios.get<DataPreviewResult>(`/projects/${projectId}/sources/${connectorId}/preview`, { params: { table, limit } })).data;
  }

  // Target Destination Management
  async listTargetDestinations(projectId: string): Promise<TargetDestination[]> {
    return (await this.axios.get<TargetDestination[]>(`/projects/${projectId}/targets`)).data;
  }

  async createTargetDestination(projectId: string, destination: Partial<TargetDestination>): Promise<TargetDestination> {
    return (await this.axios.post<TargetDestination>(`/projects/${projectId}/targets`, destination)).data;
  }

  async updateTargetDestination(projectId: string, targetId: string, updates: Partial<TargetDestination>): Promise<TargetDestination> {
    return (await this.axios.put<TargetDestination>(`/projects/${projectId}/targets/${targetId}`, updates)).data;
  }

  async testTargetConnection(projectId: string, targetId: string): Promise<ConnectionTestResult> {
    return (await this.axios.post<ConnectionTestResult>(`/projects/${projectId}/targets/${targetId}/test`)).data;
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

  async deletePipeline(projectId: string, pipelineId: string): Promise<void> {
    await this.axios.delete(`/projects/${projectId}/pipelines/${pipelineId}`);
  }

  async validatePipeline(projectId: string, pipelineId: string): Promise<PipelineValidationResult> {
    return (await this.axios.post<PipelineValidationResult>(`/projects/${projectId}/pipelines/${pipelineId}/validate`)).data;
  }

  async runPipeline(projectId: string, pipelineId: string, params?: Record<string, unknown>): Promise<PipelineRun> {
    return (await this.axios.post<PipelineRun>(`/projects/${projectId}/pipelines/${pipelineId}/run`, { params })).data;
  }

  async stopPipelineRun(projectId: string, pipelineId: string, runId: string): Promise<PipelineRun> {
    return (await this.axios.post<PipelineRun>(`/projects/${projectId}/pipelines/${pipelineId}/runs/${runId}/stop`)).data;
  }

  async retryPipelineRun(projectId: string, pipelineId: string, runId: string): Promise<PipelineRun> {
    return (await this.axios.post<PipelineRun>(`/projects/${projectId}/pipelines/${pipelineId}/runs/${runId}/retry`)).data;
  }

  async getPipelineRun(projectId: string, pipelineId: string, runId: string): Promise<PipelineRun> {
    return (await this.axios.get<PipelineRun>(`/projects/${projectId}/pipelines/${pipelineId}/runs/${runId}`)).data;
  }

  async listPipelineRuns(projectId: string, pipelineId: string, params?: { status?: string; limit?: number }): Promise<PaginatedResponse<PipelineRun>> {
    return (await this.axios.get<PaginatedResponse<PipelineRun>>(`/projects/${projectId}/pipelines/${pipelineId}/runs`, { params })).data;
  }

  async getPipelineMetrics(projectId: string, pipelineId: string, period: string): Promise<PipelineMetrics> {
    return (await this.axios.get<PipelineMetrics>(`/projects/${projectId}/pipelines/${pipelineId}/metrics`, { params: { period } })).data;
  }

  // Pipeline Stage Management
  async addPipelineStage(projectId: string, pipelineId: string, stage: Partial<PipelineStage>): Promise<PipelineStage> {
    return (await this.axios.post<PipelineStage>(`/projects/${projectId}/pipelines/${pipelineId}/stages`, stage)).data;
  }

  async updatePipelineStage(projectId: string, pipelineId: string, stageId: string, updates: Partial<PipelineStage>): Promise<PipelineStage> {
    return (await this.axios.put<PipelineStage>(`/projects/${projectId}/pipelines/${pipelineId}/stages/${stageId}`, updates)).data;
  }

  async deletePipelineStage(projectId: string, pipelineId: string, stageId: string): Promise<void> {
    await this.axios.delete(`/projects/${projectId}/pipelines/${pipelineId}/stages/${stageId}`);
  }

  async reorderPipelineStages(projectId: string, pipelineId: string, stageOrder: string[]): Promise<Pipeline> {
    return (await this.axios.post<Pipeline>(`/projects/${projectId}/pipelines/${pipelineId}/stages/reorder`, { stageOrder })).data;
  }

  // Field Mapping
  async getFieldMappings(projectId: string, pipelineId: string): Promise<FieldMapping[]> {
    return (await this.axios.get<FieldMapping[]>(`/projects/${projectId}/pipelines/${pipelineId}/mappings`)).data;
  }

  async setFieldMappings(projectId: string, pipelineId: string, mappings: FieldMapping[]): Promise<FieldMapping[]> {
    return (await this.axios.put<FieldMapping[]>(`/projects/${projectId}/pipelines/${pipelineId}/mappings`, { mappings })).data;
  }

  async autoMapFields(projectId: string, pipelineId: string): Promise<FieldMapping[]> {
    return (await this.axios.post<FieldMapping[]>(`/projects/${projectId}/pipelines/${pipelineId}/mappings/auto`)).data;
  }

  // Transformation Management
  async listCustomTransforms(projectId: string): Promise<CustomTransform[]> {
    return (await this.axios.get<CustomTransform[]>(`/projects/${projectId}/transforms`)).data;
  }

  async createCustomTransform(projectId: string, transform: Partial<CustomTransform>): Promise<CustomTransform> {
    return (await this.axios.post<CustomTransform>(`/projects/${projectId}/transforms`, transform)).data;
  }

  async testTransform(projectId: string, transformId: string, sampleData: Record<string, unknown>[]): Promise<TransformTestResult> {
    return (await this.axios.post<TransformTestResult>(`/projects/${projectId}/transforms/${transformId}/test`, { sampleData })).data;
  }

  // Workflow/Orchestration Management
  async listWorkflows(projectId: string, params?: { status?: string }): Promise<Workflow[]> {
    return (await this.axios.get<Workflow[]>(`/projects/${projectId}/workflows`, { params })).data;
  }

  async createWorkflow(projectId: string, workflow: Partial<Workflow>): Promise<Workflow> {
    return (await this.axios.post<Workflow>(`/projects/${projectId}/workflows`, workflow)).data;
  }

  async getWorkflow(projectId: string, workflowId: string): Promise<Workflow> {
    return (await this.axios.get<Workflow>(`/projects/${projectId}/workflows/${workflowId}`)).data;
  }

  async updateWorkflow(projectId: string, workflowId: string, updates: Partial<Workflow>): Promise<Workflow> {
    return (await this.axios.put<Workflow>(`/projects/${projectId}/workflows/${workflowId}`, updates)).data;
  }

  async addWorkflowTask(projectId: string, workflowId: string, task: Partial<WorkflowTask>): Promise<WorkflowTask> {
    return (await this.axios.post<WorkflowTask>(`/projects/${projectId}/workflows/${workflowId}/tasks`, task)).data;
  }

  async runWorkflow(projectId: string, workflowId: string): Promise<WorkflowRun> {
    return (await this.axios.post<WorkflowRun>(`/projects/${projectId}/workflows/${workflowId}/run`)).data;
  }

  async getWorkflowRun(projectId: string, workflowId: string, runId: string): Promise<WorkflowRun> {
    return (await this.axios.get<WorkflowRun>(`/projects/${projectId}/workflows/${workflowId}/runs/${runId}`)).data;
  }

  // Trigger Management
  async listTriggers(projectId: string): Promise<TriggerConfig[]> {
    return (await this.axios.get<TriggerConfig[]>(`/projects/${projectId}/triggers`)).data;
  }

  async createTrigger(projectId: string, trigger: Partial<TriggerConfig>): Promise<TriggerConfig> {
    return (await this.axios.post<TriggerConfig>(`/projects/${projectId}/triggers`, trigger)).data;
  }

  async updateTrigger(projectId: string, triggerId: string, updates: Partial<TriggerConfig>): Promise<TriggerConfig> {
    return (await this.axios.put<TriggerConfig>(`/projects/${projectId}/triggers/${triggerId}`, updates)).data;
  }

  async enableTrigger(projectId: string, triggerId: string): Promise<TriggerConfig> {
    return (await this.axios.post<TriggerConfig>(`/projects/${projectId}/triggers/${triggerId}/enable`)).data;
  }

  async disableTrigger(projectId: string, triggerId: string): Promise<TriggerConfig> {
    return (await this.axios.post<TriggerConfig>(`/projects/${projectId}/triggers/${triggerId}/disable`)).data;
  }

  // Quality Management
  async listQualityChecks(projectId: string): Promise<QualityCheck[]> {
    return (await this.axios.get<QualityCheck[]>(`/projects/${projectId}/quality/checks`)).data;
  }

  async createQualityCheck(projectId: string, check: Partial<QualityCheck>): Promise<QualityCheck> {
    return (await this.axios.post<QualityCheck>(`/projects/${projectId}/quality/checks`, check)).data;
  }

  async runQualityCheck(projectId: string, pipelineId: string): Promise<QualityCheckResult> {
    return (await this.axios.post<QualityCheckResult>(`/projects/${projectId}/pipelines/${pipelineId}/quality/run`)).data;
  }

  async getQualityReport(projectId: string, pipelineId: string, runId: string): Promise<QualityReport> {
    return (await this.axios.get<QualityReport>(`/projects/${projectId}/pipelines/${pipelineId}/runs/${runId}/quality`)).data;
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

  async acknowledgeAlert(projectId: string, alertId: string): Promise<void> {
    await this.axios.post(`/projects/${projectId}/alerts/${alertId}/acknowledge`);
  }

  // Lineage
  async getPipelineLineage(projectId: string, pipelineId: string): Promise<LineageGraph> {
    return (await this.axios.get<LineageGraph>(`/projects/${projectId}/pipelines/${pipelineId}/lineage`)).data;
  }

  async getDataLineage(projectId: string, table: string): Promise<LineageGraph> {
    return (await this.axios.get<LineageGraph>(`/projects/${projectId}/lineage`, { params: { table } })).data;
  }

  // Dashboard
  async getIntegrationDashboard(projectId: string): Promise<IntegrationDashboard> {
    return (await this.axios.get<IntegrationDashboard>(`/projects/${projectId}/dashboard`)).data;
  }

  // Validation
  validateProject(project: WIADataIntegrationProject): ValidationResult {
    const errors: { path: string; message: string }[] = [];

    if (!project.standard || project.standard !== 'WIA-DATA-INTEGRATION') {
      errors.push({ path: 'standard', message: 'Standard must be "WIA-DATA-INTEGRATION"' });
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

    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

// ============================================================================
// Supporting Types
// ============================================================================

export interface ConnectionTestResult {
  success: boolean;
  message?: string;
  latency?: number;
  details?: Record<string, unknown>;
}

export interface SchemaDiscoveryResult {
  tables: { name: string; columns: { name: string; type: string; nullable: boolean }[]; rowCount?: number }[];
  timestamp: string;
}

export interface DataPreviewResult {
  columns: string[];
  rows: unknown[][];
  totalRows: number;
  truncated: boolean;
}

export interface PipelineValidationResult {
  valid: boolean;
  errors: { stage: string; message: string }[];
  warnings: { stage: string; message: string }[];
}

export interface PipelineMetrics {
  pipelineId: string;
  period: string;
  runs: { total: number; successful: number; failed: number };
  avgDuration: number;
  recordsProcessed: number;
  bytesProcessed: number;
  errors: { type: string; count: number }[];
}

export interface TransformTestResult {
  success: boolean;
  input: Record<string, unknown>[];
  output: Record<string, unknown>[];
  errors?: string[];
}

export interface WorkflowRun {
  id: string;
  workflowId: string;
  status: 'pending' | 'running' | 'completed' | 'failed';
  startedAt: string;
  completedAt?: string;
  tasks: { taskId: string; status: string; duration?: number }[];
}

export interface QualityCheckResult {
  timestamp: string;
  checks: { checkId: string; passed: boolean; value: number; threshold: number }[];
  passed: number;
  failed: number;
}

export interface QualityReport {
  runId: string;
  pipelineId: string;
  timestamp: string;
  overall: 'pass' | 'fail';
  checks: { name: string; type: string; passed: boolean; details: string }[];
  recordsValidated: number;
  recordsRejected: number;
}

export interface Alert {
  id: string;
  ruleId: string;
  severity: string;
  message: string;
  triggeredAt: string;
  acknowledged: boolean;
  pipelineId?: string;
}

export interface LineageGraph {
  nodes: { id: string; type: string; name: string }[];
  edges: { source: string; target: string; type: string }[];
}

export interface IntegrationDashboard {
  pipelines: { total: number; active: number; failed: number };
  runsLast24h: { total: number; successful: number; failed: number };
  dataProcessed: { records: number; bytes: number };
  activeAlerts: number;
  topPipelines: { id: string; name: string; runs: number; successRate: number }[];
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

export function createMinimalProject(name: string, organization: string): WIADataIntegrationProject {
  return {
    standard: 'WIA-DATA-INTEGRATION',
    version: '1.0.0',
    metadata: {
      id: generateUUID(),
      name,
      organization: { name: organization, team: 'Data Engineering', contact: { name: '', email: '', role: 'Data Engineer' } },
      environment: 'development',
      createdAt: new Date().toISOString(),
      status: 'active',
    },
    sources: {
      connectors: [],
      discovery: { enabled: true, frequency: 'daily', autoSchema: true, filters: [] },
      schemas: { evolution: { strategy: 'flexible', allowedChanges: ['add-column'], notification: true }, validation: { enabled: true, rules: [], action: 'warn' }, versioning: true },
    },
    targets: {
      destinations: [],
      writeConfig: { mode: 'append', batchSize: 10000, parallelism: 4, errorHandling: { strategy: 'quarantine', maxErrors: 100, retryAttempts: 3, retryDelay: '10s' }, commit: { type: 'per-batch', checkpointing: true } },
      partitioning: [],
    },
    pipelines: {
      pipelines: [],
      templates: [],
      dependencies: { nodes: [], edges: [] },
    },
    transformations: {
      engine: { type: 'spark', config: {} },
      library: { builtin: [], custom: [], udfs: [] },
      mapping: { autoMapping: true, matchingStrategy: 'case-insensitive', defaultTransforms: [] },
    },
    orchestration: {
      platform: { type: 'airflow' },
      workflows: [],
      triggers: [],
      notifications: { channels: [], rules: [] },
    },
    quality: {
      checks: [],
      profiling: { enabled: true, frequency: 'per-run', metrics: ['cardinality', 'nulls'], sampling: 0.1 },
      anomalyDetection: { enabled: true, metrics: [], sensitivity: 'medium', baseline: '30d' },
    },
    monitoring: {
      metrics: { enabled: true, provider: 'prometheus', interval: '1m', dimensions: [], customMetrics: [] },
      logging: { level: 'info', format: 'json', destination: '', retention: '30d' },
      alerting: { enabled: true, rules: [], channels: [] },
      lineage: { enabled: true, granularity: 'table', storage: '' },
    },
  };
}

export default { WIADataIntegrationClient, generateUUID, createMinimalProject };
