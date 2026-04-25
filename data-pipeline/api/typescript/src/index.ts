/**
 * WIA Data Pipeline Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig,
  WIADataPipeline,
  PipelineResponse,
  PipelineStage,
  PipelineRun,
  RunRequest,
  StageRun,
  RunMetrics,
  ValidationResult,
  PaginatedResponse,
} from './types';

// ============================================================================
// WIA Data Pipeline Client
// ============================================================================

export class WIADataPipelineClient {
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

  // ========================================================================
  // Pipeline Management
  // ========================================================================

  /**
   * Create a new data pipeline
   */
  async createPipeline(pipeline: WIADataPipeline): Promise<PipelineResponse> {
    const response = await this.axios.post<PipelineResponse>('/pipelines', pipeline);
    return response.data;
  }

  /**
   * Get pipeline by ID
   */
  async getPipeline(id: string): Promise<WIADataPipeline> {
    const response = await this.axios.get<WIADataPipeline>(`/pipelines/${id}`);
    return response.data;
  }

  /**
   * List all pipelines
   */
  async listPipelines(params?: {
    status?: string;
    owner?: string;
    tag?: string;
    limit?: number;
    offset?: number;
  }): Promise<PaginatedResponse<PipelineResponse>> {
    const response = await this.axios.get<PaginatedResponse<PipelineResponse>>('/pipelines', {
      params,
    });
    return response.data;
  }

  /**
   * Update existing pipeline
   */
  async updatePipeline(id: string, updates: Partial<WIADataPipeline>): Promise<PipelineResponse> {
    const response = await this.axios.put<PipelineResponse>(`/pipelines/${id}`, updates);
    return response.data;
  }

  /**
   * Delete pipeline
   */
  async deletePipeline(id: string): Promise<void> {
    await this.axios.delete(`/pipelines/${id}`);
  }

  /**
   * Activate pipeline
   */
  async activatePipeline(id: string): Promise<void> {
    await this.axios.post(`/pipelines/${id}/activate`);
  }

  /**
   * Pause pipeline
   */
  async pausePipeline(id: string): Promise<void> {
    await this.axios.post(`/pipelines/${id}/pause`);
  }

  // ========================================================================
  // Stage Management
  // ========================================================================

  /**
   * Add stage to pipeline
   */
  async addStage(pipelineId: string, stage: PipelineStage): Promise<PipelineStage> {
    const response = await this.axios.post<PipelineStage>(
      `/pipelines/${pipelineId}/stages`,
      stage
    );
    return response.data;
  }

  /**
   * Get stage by ID
   */
  async getStage(pipelineId: string, stageId: string): Promise<PipelineStage> {
    const response = await this.axios.get<PipelineStage>(
      `/pipelines/${pipelineId}/stages/${stageId}`
    );
    return response.data;
  }

  /**
   * List all stages in pipeline
   */
  async listStages(
    pipelineId: string,
    params?: {
      type?: string;
      limit?: number;
      offset?: number;
    }
  ): Promise<PaginatedResponse<PipelineStage>> {
    const response = await this.axios.get<PaginatedResponse<PipelineStage>>(
      `/pipelines/${pipelineId}/stages`,
      { params }
    );
    return response.data;
  }

  /**
   * Update stage
   */
  async updateStage(
    pipelineId: string,
    stageId: string,
    updates: Partial<PipelineStage>
  ): Promise<PipelineStage> {
    const response = await this.axios.put<PipelineStage>(
      `/pipelines/${pipelineId}/stages/${stageId}`,
      updates
    );
    return response.data;
  }

  /**
   * Delete stage
   */
  async deleteStage(pipelineId: string, stageId: string): Promise<void> {
    await this.axios.delete(`/pipelines/${pipelineId}/stages/${stageId}`);
  }

  // ========================================================================
  // Pipeline Execution
  // ========================================================================

  /**
   * Trigger pipeline run
   */
  async triggerRun(request: RunRequest): Promise<PipelineRun> {
    const response = await this.axios.post<PipelineRun>('/runs', request);
    return response.data;
  }

  /**
   * Get run by ID
   */
  async getRun(runId: string): Promise<PipelineRun> {
    const response = await this.axios.get<PipelineRun>(`/runs/${runId}`);
    return response.data;
  }

  /**
   * List runs for pipeline
   */
  async listRuns(
    pipelineId: string,
    params?: {
      status?: string;
      startTime?: string;
      endTime?: string;
      limit?: number;
      offset?: number;
    }
  ): Promise<PaginatedResponse<PipelineRun>> {
    const response = await this.axios.get<PaginatedResponse<PipelineRun>>(
      `/pipelines/${pipelineId}/runs`,
      { params }
    );
    return response.data;
  }

  /**
   * Cancel running pipeline
   */
  async cancelRun(runId: string): Promise<void> {
    await this.axios.post(`/runs/${runId}/cancel`);
  }

  /**
   * Retry failed run
   */
  async retryRun(runId: string, fromStage?: string): Promise<PipelineRun> {
    const response = await this.axios.post<PipelineRun>(`/runs/${runId}/retry`, {
      fromStage,
    });
    return response.data;
  }

  /**
   * Get stage run details
   */
  async getStageRun(runId: string, stageId: string): Promise<StageRun> {
    const response = await this.axios.get<StageRun>(`/runs/${runId}/stages/${stageId}`);
    return response.data;
  }

  /**
   * Get stage run logs
   */
  async getStageRunLogs(
    runId: string,
    stageId: string,
    params?: {
      startTime?: string;
      endTime?: string;
      level?: string;
      limit?: number;
    }
  ): Promise<string[]> {
    const response = await this.axios.get<string[]>(
      `/runs/${runId}/stages/${stageId}/logs`,
      { params }
    );
    return response.data;
  }

  // ========================================================================
  // Metrics and Monitoring
  // ========================================================================

  /**
   * Get run metrics
   */
  async getRunMetrics(runId: string): Promise<RunMetrics> {
    const response = await this.axios.get<RunMetrics>(`/runs/${runId}/metrics`);
    return response.data;
  }

  /**
   * Get pipeline health
   */
  async getPipelineHealth(pipelineId: string): Promise<{
    status: 'healthy' | 'degraded' | 'unhealthy';
    lastCheck: string;
    checks: { name: string; status: string; message?: string }[];
  }> {
    const response = await this.axios.get(`/pipelines/${pipelineId}/health`);
    return response.data;
  }

  /**
   * Get pipeline statistics
   */
  async getPipelineStats(
    pipelineId: string,
    params?: {
      startTime?: string;
      endTime?: string;
      granularity?: 'hour' | 'day' | 'week' | 'month';
    }
  ): Promise<{
    totalRuns: number;
    successfulRuns: number;
    failedRuns: number;
    avgDuration: number;
    avgRecordsProcessed: number;
  }> {
    const response = await this.axios.get(`/pipelines/${pipelineId}/stats`, { params });
    return response.data;
  }

  // ========================================================================
  // Validation
  // ========================================================================

  /**
   * Validate pipeline configuration
   */
  async validatePipelineRemote(pipeline: WIADataPipeline): Promise<ValidationResult> {
    const response = await this.axios.post<ValidationResult>('/pipelines/validate', pipeline);
    return response.data;
  }

  /**
   * Validate pipeline configuration locally
   */
  validatePipeline(pipeline: WIADataPipeline): ValidationResult {
    const errors: any[] = [];

    if (!pipeline.standard || pipeline.standard !== 'WIA-DATA-PIPELINE') {
      errors.push({
        path: 'standard',
        message: 'Standard must be "WIA-DATA-PIPELINE"',
      });
    }

    if (!pipeline.version || !/^\d+\.\d+\.\d+$/.test(pipeline.version)) {
      errors.push({
        path: 'version',
        message: 'Version must follow semantic versioning (x.y.z)',
      });
    }

    if (!pipeline.pipeline || !pipeline.pipeline.id) {
      errors.push({
        path: 'pipeline.id',
        message: 'Pipeline ID is required',
      });
    }

    if (!pipeline.stages || pipeline.stages.length === 0) {
      errors.push({
        path: 'stages',
        message: 'At least one stage is required',
      });
    }

    // Validate stage dependencies
    if (pipeline.stages) {
      const stageIds = new Set(pipeline.stages.map((s) => s.id));
      pipeline.stages.forEach((stage, index) => {
        if (stage.dependencies) {
          stage.dependencies.forEach((dep) => {
            if (!stageIds.has(dep)) {
              errors.push({
                path: `stages[${index}].dependencies`,
                message: `Dependency stage "${dep}" not found`,
              });
            }
          });
        }
      });
    }

    // Validate connections
    if (pipeline.connections) {
      const stageIds = new Set(pipeline.stages?.map((s) => s.id) || []);
      pipeline.connections.forEach((conn, index) => {
        if (!stageIds.has(conn.source.stageId)) {
          errors.push({
            path: `connections[${index}].source.stageId`,
            message: `Source stage "${conn.source.stageId}" not found`,
          });
        }
        if (!stageIds.has(conn.target.stageId)) {
          errors.push({
            path: `connections[${index}].target.stageId`,
            message: `Target stage "${conn.target.stageId}" not found`,
          });
        }
      });
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined,
    };
  }

  // ========================================================================
  // Import/Export
  // ========================================================================

  /**
   * Export pipeline as YAML
   */
  async exportPipelineYAML(pipelineId: string): Promise<string> {
    const response = await this.axios.get(`/pipelines/${pipelineId}/export`, {
      headers: { Accept: 'application/yaml' },
    });
    return response.data;
  }

  /**
   * Import pipeline from YAML
   */
  async importPipelineYAML(yaml: string): Promise<PipelineResponse> {
    const response = await this.axios.post<PipelineResponse>('/pipelines/import', yaml, {
      headers: { 'Content-Type': 'application/yaml' },
    });
    return response.data;
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Generate a UUID v4
 */
export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    const v = c === 'x' ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
}

/**
 * Create a minimal valid data pipeline
 */
export function createMinimalPipeline(name: string): WIADataPipeline {
  return {
    standard: 'WIA-DATA-PIPELINE',
    version: '1.0.0',
    pipeline: {
      id: generateUUID(),
      name,
      owner: 'default',
      createdAt: new Date().toISOString(),
      status: 'draft',
    },
    stages: [],
    connections: [],
    triggers: [],
    configuration: {
      executionMode: 'batch',
      errorHandling: {
        strategy: 'fail-fast',
      },
      logging: {
        level: 'info',
        destination: {
          type: 'file',
          uri: '/var/log/pipeline',
        },
        format: 'json',
      },
    },
  };
}

/**
 * Create a source stage
 */
export function createSourceStage(
  id: string,
  name: string,
  location: { type: 's3' | 'gcs' | 'jdbc' | 'kafka'; uri: string },
  format: 'json' | 'parquet' | 'csv' = 'parquet'
): PipelineStage {
  return {
    id,
    name,
    type: 'source',
    configuration: {
      executor: {
        type: 'spark',
      },
      parameters: {},
    },
    inputs: [],
    outputs: [
      {
        id: `${id}-output`,
        name: `${name} Output`,
        format,
        location,
      },
    ],
  };
}

/**
 * Create a sink stage
 */
export function createSinkStage(
  id: string,
  name: string,
  location: { type: 's3' | 'gcs' | 'jdbc' | 'kafka'; uri: string },
  format: 'json' | 'parquet' | 'csv' = 'parquet'
): PipelineStage {
  return {
    id,
    name,
    type: 'sink',
    configuration: {
      executor: {
        type: 'spark',
      },
      parameters: {},
    },
    inputs: [
      {
        id: `${id}-input`,
        name: `${name} Input`,
        format,
        location,
      },
    ],
    outputs: [],
  };
}

/**
 * Create a transform stage
 */
export function createTransformStage(
  id: string,
  name: string,
  transformLogic: string
): PipelineStage {
  return {
    id,
    name,
    type: 'transform',
    configuration: {
      executor: {
        type: 'sql',
      },
      parameters: {
        sql: transformLogic,
      },
    },
    inputs: [
      {
        id: `${id}-input`,
        name: `${name} Input`,
        format: 'parquet',
      },
    ],
    outputs: [
      {
        id: `${id}-output`,
        name: `${name} Output`,
        format: 'parquet',
      },
    ],
  };
}

// ============================================================================
// Exports
// ============================================================================

export default {
  WIADataPipelineClient,
  generateUUID,
  createMinimalPipeline,
  createSourceStage,
  createSinkStage,
  createTransformStage,
};
