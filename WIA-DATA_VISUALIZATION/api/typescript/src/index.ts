/**
 * WIA Data Visualization Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig,
  WIADataVisualization,
  VisualizationResponse,
  VisualizationType,
  RenderRequest,
  RenderResponse,
  DataSourceConfig,
  ChartConfiguration,
  StylingConfiguration,
  ValidationResult,
  PaginatedResponse,
} from './types';

// ============================================================================
// WIA Data Visualization Client
// ============================================================================

export class WIADataVisualizationClient {
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
  // Visualization Management
  // ========================================================================

  /**
   * Create a new visualization
   */
  async createVisualization(visualization: WIADataVisualization): Promise<VisualizationResponse> {
    const response = await this.axios.post<VisualizationResponse>('/visualizations', visualization);
    return response.data;
  }

  /**
   * Get visualization by ID
   */
  async getVisualization(id: string): Promise<WIADataVisualization> {
    const response = await this.axios.get<WIADataVisualization>(`/visualizations/${id}`);
    return response.data;
  }

  /**
   * List all visualizations
   */
  async listVisualizations(params?: {
    type?: VisualizationType;
    owner?: string;
    tag?: string;
    limit?: number;
    offset?: number;
  }): Promise<PaginatedResponse<VisualizationResponse>> {
    const response = await this.axios.get<PaginatedResponse<VisualizationResponse>>('/visualizations', {
      params,
    });
    return response.data;
  }

  /**
   * Update existing visualization
   */
  async updateVisualization(id: string, updates: Partial<WIADataVisualization>): Promise<VisualizationResponse> {
    const response = await this.axios.put<VisualizationResponse>(`/visualizations/${id}`, updates);
    return response.data;
  }

  /**
   * Delete visualization
   */
  async deleteVisualization(id: string): Promise<void> {
    await this.axios.delete(`/visualizations/${id}`);
  }

  /**
   * Duplicate visualization
   */
  async duplicateVisualization(id: string, newName?: string): Promise<VisualizationResponse> {
    const response = await this.axios.post<VisualizationResponse>(`/visualizations/${id}/duplicate`, {
      name: newName,
    });
    return response.data;
  }

  // ========================================================================
  // Rendering
  // ========================================================================

  /**
   * Render visualization to image
   */
  async render(request: RenderRequest): Promise<RenderResponse> {
    const response = await this.axios.post<RenderResponse>('/render', request);
    return response.data;
  }

  /**
   * Get embed code for visualization
   */
  async getEmbedCode(id: string, options?: {
    width?: number;
    height?: number;
    theme?: 'light' | 'dark';
    interactive?: boolean;
  }): Promise<{ html: string; script: string }> {
    const response = await this.axios.get(`/visualizations/${id}/embed`, { params: options });
    return response.data;
  }

  /**
   * Export visualization data
   */
  async exportData(id: string, format: 'csv' | 'json' | 'xlsx'): Promise<Blob> {
    const response = await this.axios.get(`/visualizations/${id}/export`, {
      params: { format },
      responseType: 'blob',
    });
    return response.data;
  }

  // ========================================================================
  // Data Source Management
  // ========================================================================

  /**
   * Test data source connection
   */
  async testDataSource(dataSource: DataSourceConfig): Promise<{
    success: boolean;
    message?: string;
    sampleData?: unknown[];
  }> {
    const response = await this.axios.post('/datasources/test', dataSource);
    return response.data;
  }

  /**
   * Preview data from source
   */
  async previewData(dataSource: DataSourceConfig, limit?: number): Promise<{
    columns: { name: string; type: string }[];
    rows: unknown[][];
    totalRows: number;
  }> {
    const response = await this.axios.post('/datasources/preview', {
      ...dataSource,
      limit: limit || 100,
    });
    return response.data;
  }

  /**
   * Refresh visualization data
   */
  async refreshData(id: string): Promise<{ success: boolean; lastRefreshed: string }> {
    const response = await this.axios.post(`/visualizations/${id}/refresh`);
    return response.data;
  }

  // ========================================================================
  // Templates and Themes
  // ========================================================================

  /**
   * List available chart templates
   */
  async listTemplates(params?: {
    type?: VisualizationType;
    category?: string;
  }): Promise<PaginatedResponse<{
    id: string;
    name: string;
    type: VisualizationType;
    preview: string;
  }>> {
    const response = await this.axios.get('/templates', { params });
    return response.data;
  }

  /**
   * Create visualization from template
   */
  async createFromTemplate(templateId: string, data: {
    name: string;
    dataSource: DataSourceConfig;
    overrides?: Partial<ChartConfiguration>;
  }): Promise<VisualizationResponse> {
    const response = await this.axios.post<VisualizationResponse>(`/templates/${templateId}/create`, data);
    return response.data;
  }

  /**
   * List available themes
   */
  async listThemes(): Promise<{
    id: string;
    name: string;
    mode: 'light' | 'dark';
    preview: string;
  }[]> {
    const response = await this.axios.get('/themes');
    return response.data;
  }

  /**
   * Apply theme to visualization
   */
  async applyTheme(visualizationId: string, themeId: string): Promise<void> {
    await this.axios.post(`/visualizations/${visualizationId}/theme`, { themeId });
  }

  // ========================================================================
  // Dashboard Integration
  // ========================================================================

  /**
   * Add visualization to dashboard
   */
  async addToDashboard(visualizationId: string, dashboardId: string, position?: {
    x: number;
    y: number;
    width: number;
    height: number;
  }): Promise<void> {
    await this.axios.post(`/dashboards/${dashboardId}/widgets`, {
      visualizationId,
      position,
    });
  }

  // ========================================================================
  // Sharing and Collaboration
  // ========================================================================

  /**
   * Share visualization
   */
  async shareVisualization(id: string, options: {
    users?: string[];
    groups?: string[];
    public?: boolean;
    permissions?: 'view' | 'edit';
  }): Promise<{ shareUrl?: string }> {
    const response = await this.axios.post(`/visualizations/${id}/share`, options);
    return response.data;
  }

  /**
   * Get share settings
   */
  async getShareSettings(id: string): Promise<{
    users: { id: string; name: string; permission: string }[];
    groups: { id: string; name: string; permission: string }[];
    public: boolean;
    shareUrl?: string;
  }> {
    const response = await this.axios.get(`/visualizations/${id}/share`);
    return response.data;
  }

  // ========================================================================
  // Validation
  // ========================================================================

  /**
   * Validate visualization configuration
   */
  validateVisualization(visualization: WIADataVisualization): ValidationResult {
    const errors: any[] = [];

    if (!visualization.standard || visualization.standard !== 'WIA-DATA-VISUALIZATION') {
      errors.push({
        path: 'standard',
        message: 'Standard must be "WIA-DATA-VISUALIZATION"',
      });
    }

    if (!visualization.version || !/^\d+\.\d+\.\d+$/.test(visualization.version)) {
      errors.push({
        path: 'version',
        message: 'Version must follow semantic versioning (x.y.z)',
      });
    }

    if (!visualization.visualization || !visualization.visualization.id) {
      errors.push({
        path: 'visualization.id',
        message: 'Visualization ID is required',
      });
    }

    if (!visualization.dataSource) {
      errors.push({
        path: 'dataSource',
        message: 'Data source configuration is required',
      });
    }

    if (!visualization.chart) {
      errors.push({
        path: 'chart',
        message: 'Chart configuration is required',
      });
    }

    if (!visualization.chart?.measures || visualization.chart.measures.length === 0) {
      errors.push({
        path: 'chart.measures',
        message: 'At least one measure is required',
      });
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined,
    };
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
 * Create a minimal valid visualization
 */
export function createMinimalVisualization(
  name: string,
  type: VisualizationType = 'bar'
): WIADataVisualization {
  return {
    standard: 'WIA-DATA-VISUALIZATION',
    version: '1.0.0',
    visualization: {
      id: generateUUID(),
      name,
      type,
      owner: 'default',
      createdAt: new Date().toISOString(),
    },
    dataSource: {
      type: 'inline',
      connection: {},
    },
    chart: {
      dimensions: [],
      measures: [],
      layout: {
        width: { type: 'responsive' },
        height: { type: 'fixed', value: 400 },
        padding: { top: 20, right: 20, bottom: 40, left: 60 },
      },
    },
    styling: {
      theme: {
        name: 'default',
        mode: 'light',
      },
      colors: {
        primary: '#3366CC',
        secondary: '#DC3912',
        background: '#FFFFFF',
        text: '#333333',
        accent: ['#FF9900', '#109618', '#990099', '#0099C6'],
      },
      typography: {
        fontFamily: 'system-ui, -apple-system, sans-serif',
        fontSize: {
          title: 18,
          label: 12,
          body: 14,
          caption: 10,
        },
      },
    },
  };
}

/**
 * Create a bar chart configuration
 */
export function createBarChart(
  dimension: string,
  measure: string,
  options?: Partial<ChartConfiguration>
): ChartConfiguration {
  return {
    dimensions: [
      {
        field: dimension,
        type: 'categorical',
      },
    ],
    measures: [
      {
        field: measure,
        aggregation: 'sum',
      },
    ],
    layout: {
      width: { type: 'responsive' },
      height: { type: 'fixed', value: 400 },
      padding: { top: 20, right: 20, bottom: 40, left: 60 },
      orientation: 'vertical',
    },
    axes: {
      x: { show: true },
      y: { show: true },
    },
    legend: {
      show: false,
      position: 'top',
    },
    tooltip: {
      show: true,
    },
    ...options,
  };
}

/**
 * Create a line chart configuration
 */
export function createLineChart(
  timeDimension: string,
  measure: string,
  options?: Partial<ChartConfiguration>
): ChartConfiguration {
  return {
    dimensions: [
      {
        field: timeDimension,
        type: 'temporal',
      },
    ],
    measures: [
      {
        field: measure,
        aggregation: 'sum',
      },
    ],
    layout: {
      width: { type: 'responsive' },
      height: { type: 'fixed', value: 400 },
      padding: { top: 20, right: 20, bottom: 40, left: 60 },
    },
    axes: {
      x: { show: true, scale: { type: 'time' } },
      y: { show: true },
    },
    legend: {
      show: false,
      position: 'top',
    },
    tooltip: {
      show: true,
    },
    ...options,
  };
}

/**
 * Create a pie chart configuration
 */
export function createPieChart(
  dimension: string,
  measure: string,
  options?: Partial<ChartConfiguration>
): ChartConfiguration {
  return {
    dimensions: [
      {
        field: dimension,
        type: 'categorical',
      },
    ],
    measures: [
      {
        field: measure,
        aggregation: 'sum',
      },
    ],
    layout: {
      width: { type: 'responsive' },
      height: { type: 'fixed', value: 400 },
      padding: { top: 20, right: 20, bottom: 20, left: 20 },
    },
    legend: {
      show: true,
      position: 'right',
      orientation: 'vertical',
    },
    tooltip: {
      show: true,
    },
    ...options,
  };
}

/**
 * Get color palette by name
 */
export function getColorPalette(name: 'default' | 'warm' | 'cool' | 'pastel' | 'bold'): string[] {
  const palettes: Record<string, string[]> = {
    default: ['#3366CC', '#DC3912', '#FF9900', '#109618', '#990099', '#0099C6'],
    warm: ['#E25822', '#F2A541', '#F5CB5C', '#E8D5B7', '#BC4B51', '#8B2635'],
    cool: ['#264653', '#2A9D8F', '#E9C46A', '#F4A261', '#E76F51', '#023047'],
    pastel: ['#FFB3BA', '#BAFFC9', '#BAE1FF', '#FFFFBA', '#FFDFBA', '#E0BBE4'],
    bold: ['#FF0000', '#00FF00', '#0000FF', '#FFFF00', '#FF00FF', '#00FFFF'],
  };
  return palettes[name] || palettes.default;
}

// ============================================================================
// Exports
// ============================================================================

export default {
  WIADataVisualizationClient,
  generateUUID,
  createMinimalVisualization,
  createBarChart,
  createLineChart,
  createPieChart,
  getColorPalette,
};
