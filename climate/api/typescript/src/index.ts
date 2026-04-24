/**
 * WIA Climate Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig,
  WIAClimateProject,
  ClimateDataRequest,
  ClimateDataResponse,
  ProjectResponse,
  ValidationResult,
  PaginatedResponse,
  ClimateVariable,
  DataSource,
  MonitoringStation,
  ClimateScenario,
  DataPoint,
} from './types';

// ============================================================================
// WIA Climate Client
// ============================================================================

export class WIAClimateClient {
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
  // Project Management
  // ========================================================================

  async createProject(project: WIAClimateProject): Promise<ProjectResponse> {
    const response = await this.axios.post<ProjectResponse>('/projects', project);
    return response.data;
  }

  async getProject(id: string): Promise<WIAClimateProject> {
    const response = await this.axios.get<WIAClimateProject>(`/projects/${id}`);
    return response.data;
  }

  async listProjects(params?: {
    type?: string;
    region?: string;
    status?: string;
    limit?: number;
    offset?: number;
  }): Promise<PaginatedResponse<ProjectResponse>> {
    const response = await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', {
      params,
    });
    return response.data;
  }

  async updateProject(id: string, updates: Partial<WIAClimateProject>): Promise<ProjectResponse> {
    const response = await this.axios.put<ProjectResponse>(`/projects/${id}`, updates);
    return response.data;
  }

  async deleteProject(id: string): Promise<void> {
    await this.axios.delete(`/projects/${id}`);
  }

  // ========================================================================
  // Climate Data
  // ========================================================================

  async queryClimateData(request: ClimateDataRequest): Promise<ClimateDataResponse> {
    const response = await this.axios.post<ClimateDataResponse>('/data/query', request);
    return response.data;
  }

  async getLatestData(variables: string[], stationId?: string): Promise<DataPoint[]> {
    const response = await this.axios.get<DataPoint[]>('/data/latest', {
      params: { variables: variables.join(','), stationId },
    });
    return response.data;
  }

  async getHistoricalData(
    variables: string[],
    start: string,
    end: string,
    region?: string
  ): Promise<ClimateDataResponse> {
    const response = await this.axios.get<ClimateDataResponse>('/data/historical', {
      params: { variables: variables.join(','), start, end, region },
    });
    return response.data;
  }

  async getClimatology(
    variable: string,
    region: string,
    period: string
  ): Promise<any> {
    const response = await this.axios.get('/data/climatology', {
      params: { variable, region, period },
    });
    return response.data;
  }

  async getAnomalies(
    variable: string,
    region: string,
    baselinePeriod: string
  ): Promise<DataPoint[]> {
    const response = await this.axios.get<DataPoint[]>('/data/anomalies', {
      params: { variable, region, baselinePeriod },
    });
    return response.data;
  }

  // ========================================================================
  // Variables
  // ========================================================================

  async listVariables(): Promise<ClimateVariable[]> {
    const response = await this.axios.get<ClimateVariable[]>('/variables');
    return response.data;
  }

  async getVariable(id: string): Promise<ClimateVariable> {
    const response = await this.axios.get<ClimateVariable>(`/variables/${id}`);
    return response.data;
  }

  // ========================================================================
  // Data Sources
  // ========================================================================

  async listDataSources(): Promise<DataSource[]> {
    const response = await this.axios.get<DataSource[]>('/sources');
    return response.data;
  }

  async getDataSource(id: string): Promise<DataSource> {
    const response = await this.axios.get<DataSource>(`/sources/${id}`);
    return response.data;
  }

  // ========================================================================
  // Monitoring Stations
  // ========================================================================

  async listStations(params?: {
    type?: string;
    status?: string;
    region?: string;
    limit?: number;
    offset?: number;
  }): Promise<PaginatedResponse<MonitoringStation>> {
    const response = await this.axios.get<PaginatedResponse<MonitoringStation>>('/stations', {
      params,
    });
    return response.data;
  }

  async getStation(id: string): Promise<MonitoringStation> {
    const response = await this.axios.get<MonitoringStation>(`/stations/${id}`);
    return response.data;
  }

  async getStationData(
    stationId: string,
    variables: string[],
    start: string,
    end: string
  ): Promise<DataPoint[]> {
    const response = await this.axios.get<DataPoint[]>(`/stations/${stationId}/data`, {
      params: { variables: variables.join(','), start, end },
    });
    return response.data;
  }

  // ========================================================================
  // Scenarios
  // ========================================================================

  async listScenarios(): Promise<ClimateScenario[]> {
    const response = await this.axios.get<ClimateScenario[]>('/scenarios');
    return response.data;
  }

  async getScenario(id: string): Promise<ClimateScenario> {
    const response = await this.axios.get<ClimateScenario>(`/scenarios/${id}`);
    return response.data;
  }

  async getProjections(
    scenario: string,
    variables: string[],
    region: string,
    timeframe: { start: string; end: string }
  ): Promise<ClimateDataResponse> {
    const response = await this.axios.post<ClimateDataResponse>('/scenarios/projections', {
      scenario,
      variables,
      region,
      timeframe,
    });
    return response.data;
  }

  // ========================================================================
  // Analysis
  // ========================================================================

  async runTrendAnalysis(
    variable: string,
    region: string,
    start: string,
    end: string
  ): Promise<TrendAnalysisResult> {
    const response = await this.axios.post<TrendAnalysisResult>('/analysis/trend', {
      variable,
      region,
      start,
      end,
    });
    return response.data;
  }

  async runExtremeValueAnalysis(
    variable: string,
    region: string,
    returnPeriods: number[]
  ): Promise<ExtremeValueResult> {
    const response = await this.axios.post<ExtremeValueResult>('/analysis/extremes', {
      variable,
      region,
      returnPeriods,
    });
    return response.data;
  }

  async detectAnomalies(
    variable: string,
    region: string,
    method: string,
    threshold?: number
  ): Promise<AnomalyDetectionResult> {
    const response = await this.axios.post<AnomalyDetectionResult>('/analysis/anomalies', {
      variable,
      region,
      method,
      threshold,
    });
    return response.data;
  }

  // ========================================================================
  // Reports
  // ========================================================================

  async generateReport(
    projectId: string,
    templateId: string,
    options?: ReportOptions
  ): Promise<ReportResult> {
    const response = await this.axios.post<ReportResult>('/reports/generate', {
      projectId,
      templateId,
      options,
    });
    return response.data;
  }

  async getReport(reportId: string): Promise<any> {
    const response = await this.axios.get(`/reports/${reportId}`);
    return response.data;
  }

  // ========================================================================
  // Validation
  // ========================================================================

  validateProject(project: WIAClimateProject): ValidationResult {
    const errors: any[] = [];

    if (!project.standard || project.standard !== 'WIA-CLIMATE') {
      errors.push({
        path: 'standard',
        message: 'Standard must be "WIA-CLIMATE"',
      });
    }

    if (!project.version || !/^\d+\.\d+\.\d+$/.test(project.version)) {
      errors.push({
        path: 'version',
        message: 'Version must follow semantic versioning (x.y.z)',
      });
    }

    if (!project.metadata?.id) {
      errors.push({
        path: 'metadata.id',
        message: 'Project ID is required',
      });
    }

    if (!project.climateData?.variables?.length) {
      errors.push({
        path: 'climateData.variables',
        message: 'At least one climate variable is required',
      });
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined,
    };
  }
}

// ============================================================================
// Analysis Result Types
// ============================================================================

export interface TrendAnalysisResult {
  variable: string;
  region: string;
  period: { start: string; end: string };
  trend: {
    slope: number;
    intercept: number;
    unit: string;
    perDecade?: number;
  };
  significance: {
    pValue: number;
    confident: boolean;
    method: string;
  };
  statistics: {
    r2: number;
    rmse: number;
    n: number;
  };
}

export interface ExtremeValueResult {
  variable: string;
  region: string;
  distribution: string;
  parameters: Record<string, number>;
  returnLevels: ReturnLevel[];
  uncertainty: {
    method: string;
    confidenceLevel: number;
  };
}

export interface ReturnLevel {
  period: number;
  value: number;
  lower: number;
  upper: number;
}

export interface AnomalyDetectionResult {
  variable: string;
  region: string;
  method: string;
  anomalies: Anomaly[];
  statistics: {
    total: number;
    positive: number;
    negative: number;
    meanMagnitude: number;
  };
}

export interface Anomaly {
  timestamp: string;
  value: number;
  deviation: number;
  percentile?: number;
}

export interface ReportOptions {
  format?: 'pdf' | 'html' | 'docx';
  language?: string;
  sections?: string[];
  includeCharts?: boolean;
  includeMaps?: boolean;
}

export interface ReportResult {
  id: string;
  status: 'queued' | 'processing' | 'completed' | 'failed';
  url?: string;
  createdAt: string;
  completedAt?: string;
}

// ============================================================================
// Utility Functions
// ============================================================================

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    const v = c === 'x' ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
}

export function createMinimalProject(name: string, region: string): WIAClimateProject {
  return {
    standard: 'WIA-CLIMATE',
    version: '1.0.0',
    metadata: {
      id: generateUUID(),
      name,
      type: 'monitoring',
      region: {
        name: region,
        type: 'regional',
      },
      timeframe: {
        start: new Date().toISOString().split('T')[0],
        end: new Date(Date.now() + 365 * 24 * 60 * 60 * 1000).toISOString().split('T')[0],
        resolution: 'daily',
      },
    },
    climateData: {
      variables: [
        {
          id: 'temp',
          name: 'Temperature',
          type: 'temperature',
          unit: '°C',
        },
      ],
      sources: [],
      quality: {
        level: 'quality-controlled',
        checks: [],
      },
    },
    monitoring: {
      schedule: {
        frequency: 'hourly',
        timezone: 'UTC',
      },
    },
    analysis: {
      methods: [],
      outputs: [],
    },
    reporting: {
      reports: [],
    },
  };
}

export function celsiusToFahrenheit(celsius: number): number {
  return (celsius * 9) / 5 + 32;
}

export function fahrenheitToCelsius(fahrenheit: number): number {
  return ((fahrenheit - 32) * 5) / 9;
}

export function calculateHeatIndex(temperature: number, humidity: number): number {
  const T = temperature;
  const R = humidity;

  if (T < 27 || R < 40) {
    return T;
  }

  return (
    -8.784695 +
    1.61139411 * T +
    2.338549 * R -
    0.14611605 * T * R -
    0.012308094 * T * T -
    0.016424828 * R * R +
    0.002211732 * T * T * R +
    0.00072546 * T * R * R -
    0.000003582 * T * T * R * R
  );
}

export function calculateWindChill(temperature: number, windSpeed: number): number {
  if (temperature > 10 || windSpeed < 4.8) {
    return temperature;
  }

  return (
    13.12 +
    0.6215 * temperature -
    11.37 * Math.pow(windSpeed, 0.16) +
    0.3965 * temperature * Math.pow(windSpeed, 0.16)
  );
}

// ============================================================================
// Exports
// ============================================================================

export default {
  WIAClimateClient,
  generateUUID,
  createMinimalProject,
  celsiusToFahrenheit,
  fahrenheitToCelsius,
  calculateHeatIndex,
  calculateWindChill,
};
