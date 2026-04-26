/**
 * WIA-ENE-027: Indoor Air Quality Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import type {
  ApiResponse,
  SpaceRegistrationRequest,
  SpaceRegistrationResponse,
  ReadingSubmissionRequest,
  ReadingSubmissionResponse,
  AirQualityReading,
  IndoorSpace,
  AirQualityAlert,
  HistoricalDataQuery,
  TimeSeriesData,
  StatisticalSummary,
  AirQualityReport,
  ReportType,
  HealthRiskAssessment,
  ComplianceStatus,
  HVACStatus,
  Timestamp,
} from './types';

/**
 * Client configuration
 */
export interface IndoorAirQualityClientConfig {
  apiKey: string;
  endpoint?: string;
  timeout?: number;
  headers?: Record<string, string>;
}

/**
 * Default configuration
 */
const DEFAULT_CONFIG = {
  endpoint: 'https://api.wia.org/ene-027/v1',
  timeout: 30000,
};

/**
 * Indoor Air Quality Client
 *
 * @example
 * ```typescript
 * const client = new IndoorAirQualityClient({
 *   apiKey: 'your-api-key',
 *   endpoint: 'https://api.wia.org/ene-027/v1'
 * });
 *
 * // Register a space
 * const space = await client.registerSpace({
 *   space: {
 *     spaceId: 'OFFICE-SEL-001',
 *     name: '서울 본사 3층',
 *     type: 'OFFICE',
 *     location: { lat: 37.5665, lon: 126.9780 },
 *     floorArea_m2: 150,
 *     ceilingHeight_m: 2.7,
 *     volume_m3: 405,
 *     occupancy: { maxOccupancy: 20 }
 *   },
 *   sensors: [...]
 * });
 *
 * console.log('Space ID:', space.spaceId);
 * console.log('Dashboard URL:', space.dashboardUrl);
 * ```
 */
export class IndoorAirQualityClient {
  private config: Required<IndoorAirQualityClientConfig>;

  constructor(config: IndoorAirQualityClientConfig) {
    this.config = {
      ...DEFAULT_CONFIG,
      ...config,
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${config.apiKey}`,
        ...config.headers,
      },
    };
  }

  // ============================================================================
  // Space Management
  // ============================================================================

  /**
   * Register a new indoor space
   *
   * @param request - Space registration information
   * @returns Space registration response with ID and dashboard URL
   */
  async registerSpace(
    request: SpaceRegistrationRequest
  ): Promise<SpaceRegistrationResponse> {
    const response = await this.post<SpaceRegistrationResponse>(
      '/spaces/register',
      request
    );
    return response;
  }

  /**
   * Get space information by ID
   *
   * @param spaceId - Space ID
   * @returns Indoor space information
   */
  async getSpace(spaceId: string): Promise<IndoorSpace> {
    const response = await this.get<IndoorSpace>(`/spaces/${spaceId}`);
    return response;
  }

  /**
   * Update space information
   *
   * @param spaceId - Space ID
   * @param updates - Partial space updates
   * @returns Updated space information
   */
  async updateSpace(
    spaceId: string,
    updates: Partial<IndoorSpace>
  ): Promise<IndoorSpace> {
    const response = await this.put<IndoorSpace>(
      `/spaces/${spaceId}`,
      updates
    );
    return response;
  }

  /**
   * List all registered spaces
   *
   * @param limit - Maximum number of results
   * @param offset - Offset for pagination
   * @returns List of spaces
   */
  async listSpaces(limit: number = 100, offset: number = 0): Promise<{
    spaces: IndoorSpace[];
    total: number;
  }> {
    const response = await this.get<{
      spaces: IndoorSpace[];
      total: number;
    }>(`/spaces?limit=${limit}&offset=${offset}`);
    return response;
  }

  // ============================================================================
  // Air Quality Readings
  // ============================================================================

  /**
   * Submit air quality readings
   *
   * @param request - Readings data
   * @returns Submission response
   */
  async submitReadings(
    request: ReadingSubmissionRequest
  ): Promise<ReadingSubmissionResponse> {
    const response = await this.post<ReadingSubmissionResponse>(
      '/readings/submit',
      request
    );
    return response;
  }

  /**
   * Get latest air quality reading for a space
   *
   * @param spaceId - Space ID
   * @returns Latest air quality reading
   */
  async getLatestReading(spaceId: string): Promise<AirQualityReading> {
    const response = await this.get<AirQualityReading>(
      `/spaces/${spaceId}/readings/latest`
    );
    return response;
  }

  /**
   * Get current air quality status for a space
   *
   * @param spaceId - Space ID
   * @returns Current air quality reading with IAQ index
   */
  async getCurrentStatus(spaceId: string): Promise<{
    reading: AirQualityReading;
    alerts: AirQualityAlert[];
  }> {
    const response = await this.get<{
      reading: AirQualityReading;
      alerts: AirQualityAlert[];
    }>(`/spaces/${spaceId}/status`);
    return response;
  }

  // ============================================================================
  // Historical Data & Analytics
  // ============================================================================

  /**
   * Query historical air quality data
   *
   * @param query - Query parameters
   * @returns Time series data
   */
  async getHistoricalData(
    query: HistoricalDataQuery
  ): Promise<TimeSeriesData[]> {
    const params = new URLSearchParams({
      spaceId: query.spaceId,
      parameters: query.parameters.join(','),
      startDate: query.startDate,
      endDate: query.endDate,
    });

    if (query.resolution) {
      params.append('resolution', query.resolution);
    }
    if (query.aggregation) {
      params.append('aggregation', query.aggregation);
    }

    const response = await this.get<TimeSeriesData[]>(
      `/data/historical?${params.toString()}`
    );
    return response;
  }

  /**
   * Get statistical summary for a parameter
   *
   * @param spaceId - Space ID
   * @param parameter - Parameter name (e.g., 'co2_ppm', 'pm25_ugm3')
   * @param startDate - Start date (ISO 8601)
   * @param endDate - End date (ISO 8601)
   * @returns Statistical summary
   */
  async getStatistics(
    spaceId: string,
    parameter: string,
    startDate: Timestamp,
    endDate: Timestamp
  ): Promise<StatisticalSummary> {
    const response = await this.get<StatisticalSummary>(
      `/spaces/${spaceId}/statistics?parameter=${parameter}&start=${startDate}&end=${endDate}`
    );
    return response;
  }

  // ============================================================================
  // Alerts & Notifications
  // ============================================================================

  /**
   * Get active alerts for a space
   *
   * @param spaceId - Space ID
   * @returns List of active alerts
   */
  async getActiveAlerts(spaceId: string): Promise<AirQualityAlert[]> {
    const response = await this.get<AirQualityAlert[]>(
      `/spaces/${spaceId}/alerts/active`
    );
    return response;
  }

  /**
   * Acknowledge an alert
   *
   * @param alertId - Alert ID
   * @returns Updated alert
   */
  async acknowledgeAlert(alertId: string): Promise<AirQualityAlert> {
    const response = await this.post<AirQualityAlert>(
      `/alerts/${alertId}/acknowledge`,
      {}
    );
    return response;
  }

  /**
   * Subscribe to real-time alerts via WebSocket
   *
   * @param spaceId - Space ID
   * @param onAlert - Callback for incoming alerts
   * @returns WebSocket connection
   */
  subscribeToAlerts(
    spaceId: string,
    onAlert: (alert: AirQualityAlert) => void
  ): WebSocket {
    const wsEndpoint = this.config.endpoint
      .replace('http://', 'ws://')
      .replace('https://', 'wss://');
    const ws = new WebSocket(
      `${wsEndpoint}/spaces/${spaceId}/alerts/stream`
    );

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      onAlert(data);
    };

    ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    return ws;
  }

  /**
   * Subscribe to real-time air quality readings via WebSocket
   *
   * @param spaceId - Space ID
   * @param onReading - Callback for incoming readings
   * @returns WebSocket connection
   */
  subscribeToReadings(
    spaceId: string,
    onReading: (reading: AirQualityReading) => void
  ): WebSocket {
    const wsEndpoint = this.config.endpoint
      .replace('http://', 'ws://')
      .replace('https://', 'wss://');
    const ws = new WebSocket(
      `${wsEndpoint}/spaces/${spaceId}/readings/stream`
    );

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      onReading(data);
    };

    ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    return ws;
  }

  // ============================================================================
  // Reports
  // ============================================================================

  /**
   * Generate air quality report
   *
   * @param spaceId - Space ID
   * @param reportType - Report type
   * @param startDate - Start date (ISO 8601)
   * @param endDate - End date (ISO 8601)
   * @returns Air quality report
   */
  async generateReport(
    spaceId: string,
    reportType: ReportType,
    startDate: Timestamp,
    endDate: Timestamp
  ): Promise<AirQualityReport> {
    const response = await this.post<AirQualityReport>('/reports/generate', {
      spaceId,
      reportType,
      startDate,
      endDate,
    });
    return response;
  }

  /**
   * Get report by ID
   *
   * @param reportId - Report ID
   * @returns Air quality report
   */
  async getReport(reportId: string): Promise<AirQualityReport> {
    const response = await this.get<AirQualityReport>(`/reports/${reportId}`);
    return response;
  }

  /**
   * Export report as PDF
   *
   * @param reportId - Report ID
   * @returns PDF blob
   */
  async exportReportPDF(reportId: string): Promise<Blob> {
    const url = `${this.config.endpoint}/reports/${reportId}/export/pdf`;
    const response = await fetch(url, {
      headers: this.config.headers,
    });

    if (!response.ok) {
      throw new Error(`HTTP ${response.status}: ${response.statusText}`);
    }

    return response.blob();
  }

  // ============================================================================
  // Health & Compliance
  // ============================================================================

  /**
   * Get health risk assessment for a space
   *
   * @param spaceId - Space ID
   * @param startDate - Start date (ISO 8601)
   * @param endDate - End date (ISO 8601)
   * @returns Health risk assessment
   */
  async getHealthRiskAssessment(
    spaceId: string,
    startDate: Timestamp,
    endDate: Timestamp
  ): Promise<HealthRiskAssessment> {
    const response = await this.get<HealthRiskAssessment>(
      `/spaces/${spaceId}/health-risk?start=${startDate}&end=${endDate}`
    );
    return response;
  }

  /**
   * Check compliance status
   *
   * @param spaceId - Space ID
   * @param standard - Standard name (e.g., 'ASHRAE_62.1', 'WHO_2021')
   * @returns Compliance status
   */
  async checkCompliance(
    spaceId: string,
    standard: string
  ): Promise<ComplianceStatus> {
    const response = await this.get<ComplianceStatus>(
      `/spaces/${spaceId}/compliance?standard=${standard}`
    );
    return response;
  }

  // ============================================================================
  // HVAC Integration
  // ============================================================================

  /**
   * Get HVAC system status
   *
   * @param spaceId - Space ID
   * @returns HVAC status
   */
  async getHVACStatus(spaceId: string): Promise<HVACStatus[]> {
    const response = await this.get<HVACStatus[]>(
      `/spaces/${spaceId}/hvac/status`
    );
    return response;
  }

  /**
   * Update HVAC settings
   *
   * @param spaceId - Space ID
   * @param systemId - HVAC system ID
   * @param settings - Settings to update
   * @returns Updated HVAC status
   */
  async updateHVACSettings(
    spaceId: string,
    systemId: string,
    settings: Partial<HVACStatus>
  ): Promise<HVACStatus> {
    const response = await this.put<HVACStatus>(
      `/spaces/${spaceId}/hvac/${systemId}`,
      settings
    );
    return response;
  }

  /**
   * Get HVAC control recommendations based on air quality
   *
   * @param spaceId - Space ID
   * @returns HVAC control recommendations
   */
  async getHVACRecommendations(spaceId: string): Promise<{
    recommendations: Array<{
      systemId: string;
      action: string;
      reason: string;
      priority: string;
    }>;
  }> {
    const response = await this.get<{
      recommendations: Array<{
        systemId: string;
        action: string;
        reason: string;
        priority: string;
      }>;
    }>(`/spaces/${spaceId}/hvac/recommendations`);
    return response;
  }

  // ============================================================================
  // Utilities
  // ============================================================================

  /**
   * Calculate Indoor Air Quality Index
   *
   * @param reading - Air quality reading
   * @returns IAQ index and category
   */
  async calculateIAQ(reading: Partial<AirQualityReading>): Promise<{
    overall: number;
    category: string;
    dominantPollutant: string;
    healthAdvisory: string;
  }> {
    const response = await this.post<{
      overall: number;
      category: string;
      dominantPollutant: string;
      healthAdvisory: string;
    }>('/utilities/calculate-iaq', { reading });
    return response;
  }

  /**
   * Get ventilation recommendations
   *
   * @param spaceId - Space ID
   * @returns Ventilation recommendations
   */
  async getVentilationRecommendations(spaceId: string): Promise<{
    currentACH: number;
    recommendedACH: number;
    reason: string;
    actions: string[];
  }> {
    const response = await this.get<{
      currentACH: number;
      recommendedACH: number;
      reason: string;
      actions: string[];
    }>(`/spaces/${spaceId}/ventilation/recommendations`);
    return response;
  }

  // ============================================================================
  // HTTP Helpers
  // ============================================================================

  private async get<T>(path: string): Promise<T> {
    return this.request<T>('GET', path);
  }

  private async post<T>(path: string, body?: any): Promise<T> {
    return this.request<T>('POST', path, body);
  }

  private async put<T>(path: string, body?: any): Promise<T> {
    return this.request<T>('PUT', path, body);
  }

  private async delete<T>(path: string): Promise<T> {
    return this.request<T>('DELETE', path);
  }

  private async request<T>(
    method: string,
    path: string,
    body?: any
  ): Promise<T> {
    const url = `${this.config.endpoint}${path}`;
    const options: RequestInit = {
      method,
      headers: this.config.headers,
    };

    if (body) {
      options.body = JSON.stringify(body);
    }

    try {
      const response = await fetch(url, options);

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(
          errorData.error?.message || `HTTP ${response.status}: ${response.statusText}`
        );
      }

      const apiResponse: ApiResponse<T> = await response.json();

      if (!apiResponse.success) {
        throw new Error(
          apiResponse.error?.message || 'API request failed'
        );
      }

      return apiResponse.data as T;
    } catch (error) {
      if (error instanceof Error) {
        throw error;
      }
      throw new Error('Unknown error occurred');
    }
  }
}

// ============================================================================
// Export types
// ============================================================================

export * from './types';

// ============================================================================
// Re-export for convenience
// ============================================================================

export default IndoorAirQualityClient;
