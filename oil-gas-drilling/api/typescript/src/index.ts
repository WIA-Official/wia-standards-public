/**
 * WIA-ENE-036: Oil & Gas Drilling Standard - SDK Implementation
 *
 * @version 1.0.0
 * @license CC BY 4.0
 * @description Client SDK for oil and gas well management and monitoring
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

import type {
  ClientConfig,
  Well,
  CreateWellRequest,
  CreateWellResponse,
  UpdateWellStatusRequest,
  WellQueryParams,
  ProductionRecord,
  SubmitProductionRequest,
  ProductionQueryParams,
  SubmitEnvironmentalDataRequest,
  SpillReportRequest,
  ProductionForecastRequest,
  ProductionForecastResponse,
  EmissionsAlert,
  SafetyIncidentReport,
  ComplianceStatus,
  OperatorDashboard,
  PaginatedResponse,
  APIError,
} from './types';

export * from './types';

/**
 * Oil & Gas Drilling Client
 */
export class OilGasDrillingClient {
  private config: Required<ClientConfig>;
  private baseURL: string;

  constructor(config: ClientConfig) {
    this.config = {
      apiKey: config.apiKey,
      endpoint: config.endpoint || 'https://api.wia.org/ene-036/v1',
      timeout: config.timeout || 30000,
      retries: config.retries || 3,
      operatorId: config.operatorId,
    };
    this.baseURL = this.config.endpoint;
  }

  /**
   * Make HTTP request with retry logic
   */
  private async request<T>(
    method: string,
    path: string,
    body?: any,
    params?: Record<string, any>
  ): Promise<T> {
    const url = new URL(path, this.baseURL);

    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined && value !== null) {
          url.searchParams.append(key, String(value));
        }
      });
    }

    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
      'X-API-Key': this.config.apiKey,
      'User-Agent': 'WIA-ENE-036-SDK/1.0.0',
    };

    if (this.config.operatorId) {
      headers['X-Operator-ID'] = this.config.operatorId;
    }

    let lastError: Error | null = null;

    for (let attempt = 0; attempt <= this.config.retries; attempt++) {
      try {
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

        const response = await fetch(url.toString(), {
          method,
          headers,
          body: body ? JSON.stringify(body) : undefined,
          signal: controller.signal,
        });

        clearTimeout(timeoutId);

        if (!response.ok) {
          const error: APIError = await response.json();
          throw new Error(`API Error (${response.status}): ${error.message}`);
        }

        return await response.json();
      } catch (error) {
        lastError = error as Error;
        if (attempt < this.config.retries) {
          // Exponential backoff
          await new Promise(resolve => setTimeout(resolve, Math.pow(2, attempt) * 1000));
        }
      }
    }

    throw lastError || new Error('Request failed');
  }

  // ==================== Well Management ====================

  /**
   * Create a new well
   */
  async createWell(request: CreateWellRequest): Promise<CreateWellResponse> {
    return this.request<CreateWellResponse>('POST', '/wells', request);
  }

  /**
   * Get well details by ID
   */
  async getWell(wellId: string): Promise<Well> {
    return this.request<Well>('GET', `/wells/${wellId}`);
  }

  /**
   * Query wells with filters
   */
  async queryWells(params: WellQueryParams): Promise<PaginatedResponse<Well>> {
    return this.request<PaginatedResponse<Well>>('GET', '/wells', undefined, params);
  }

  /**
   * Update well status
   */
  async updateWellStatus(request: UpdateWellStatusRequest): Promise<void> {
    await this.request('PUT', `/wells/${request.wellId}/status`, request);
  }

  /**
   * Update well data (partial update)
   */
  async updateWell(wellId: string, updates: Partial<Well>): Promise<void> {
    await this.request('PATCH', `/wells/${wellId}`, updates);
  }

  /**
   * Delete a well record (admin only)
   */
  async deleteWell(wellId: string): Promise<void> {
    await this.request('DELETE', `/wells/${wellId}`);
  }

  // ==================== Production Data ====================

  /**
   * Submit daily production data
   */
  async submitProduction(request: SubmitProductionRequest): Promise<void> {
    await this.request('POST', '/production/daily', request);
  }

  /**
   * Get production data
   */
  async getProduction(params: ProductionQueryParams): Promise<PaginatedResponse<ProductionRecord>> {
    return this.request<PaginatedResponse<ProductionRecord>>('GET', '/production/daily', undefined, params);
  }

  /**
   * Get production forecast using ML models
   */
  async getProductionForecast(request: ProductionForecastRequest): Promise<ProductionForecastResponse> {
    return this.request<ProductionForecastResponse>('POST', '/production/forecast', request);
  }

  /**
   * Get cumulative production for a well
   */
  async getCumulativeProduction(wellId: string, asOfDate?: string): Promise<any> {
    const params = asOfDate ? { asOfDate } : {};
    return this.request('GET', `/wells/${wellId}/production/cumulative`, undefined, params);
  }

  /**
   * Batch submit production data for multiple wells
   */
  async batchSubmitProduction(records: SubmitProductionRequest[]): Promise<void> {
    await this.request('POST', '/production/batch', { records });
  }

  // ==================== Environmental Monitoring ====================

  /**
   * Submit environmental data (emissions, water usage, etc.)
   */
  async submitEnvironmentalData(request: SubmitEnvironmentalDataRequest): Promise<void> {
    await this.request('POST', '/environmental/emissions', request);
  }

  /**
   * Get emissions data
   */
  async getEmissionsData(params: {
    wellId?: string;
    operatorId?: string;
    startDate?: string;
    endDate?: string;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<any>> {
    return this.request<PaginatedResponse<any>>('GET', '/environmental/emissions', undefined, params);
  }

  /**
   * Report a spill incident
   */
  async reportSpill(request: SpillReportRequest): Promise<void> {
    await this.request('POST', '/environmental/spills', request);
  }

  /**
   * Get spill incidents
   */
  async getSpills(params: {
    wellId?: string;
    operatorId?: string;
    startDate?: string;
    endDate?: string;
    severity?: string;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<any>> {
    return this.request<PaginatedResponse<any>>('GET', '/environmental/spills', undefined, params);
  }

  /**
   * Get emissions alerts
   */
  async getEmissionsAlerts(params: {
    wellId?: string;
    operatorId?: string;
    severity?: 'info' | 'warning' | 'critical';
    status?: 'active' | 'acknowledged' | 'resolved';
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<EmissionsAlert>> {
    return this.request<PaginatedResponse<EmissionsAlert>>('GET', '/environmental/alerts', undefined, params);
  }

  /**
   * Acknowledge an emissions alert
   */
  async acknowledgeEmissionsAlert(alertId: string, acknowledgedBy: string): Promise<void> {
    await this.request('PUT', `/environmental/alerts/${alertId}/acknowledge`, { acknowledgedBy });
  }

  /**
   * Get flaring statistics
   */
  async getFlaringStats(params: {
    wellId?: string;
    operatorId?: string;
    startDate?: string;
    endDate?: string;
    aggregation?: 'daily' | 'monthly' | 'yearly';
  }): Promise<any> {
    return this.request('GET', '/environmental/flaring', undefined, params);
  }

  // ==================== Safety Monitoring ====================

  /**
   * Report a safety incident
   */
  async reportSafetyIncident(incident: SafetyIncidentReport): Promise<void> {
    await this.request('POST', '/safety/incidents', incident);
  }

  /**
   * Get safety incidents
   */
  async getSafetyIncidents(params: {
    wellId?: string;
    operatorId?: string;
    incidentType?: string;
    severity?: string;
    startDate?: string;
    endDate?: string;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<SafetyIncidentReport>> {
    return this.request<PaginatedResponse<SafetyIncidentReport>>('GET', '/safety/incidents', undefined, params);
  }

  /**
   * Submit BOP test results
   */
  async submitBOPTest(wellId: string, testData: any): Promise<void> {
    await this.request('POST', `/safety/bop-tests`, { wellId, ...testData });
  }

  /**
   * Get BOP test history
   */
  async getBOPTests(params: {
    wellId?: string;
    startDate?: string;
    endDate?: string;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<any>> {
    return this.request<PaginatedResponse<any>>('GET', '/safety/bop-tests', undefined, params);
  }

  /**
   * Submit H2S alert
   */
  async submitH2SAlert(wellId: string, alertData: any): Promise<void> {
    await this.request('POST', '/safety/h2s-alerts', { wellId, ...alertData });
  }

  // ==================== Regulatory Compliance ====================

  /**
   * Get compliance status for a well or operator
   */
  async getComplianceStatus(wellId?: string, operatorId?: string): Promise<ComplianceStatus> {
    const params = { wellId, operatorId };
    return this.request<ComplianceStatus>('GET', '/regulatory/compliance', undefined, params);
  }

  /**
   * Get permit information
   */
  async getPermits(params: {
    wellId?: string;
    operatorId?: string;
    permitType?: string;
    status?: string;
    expiringWithinDays?: number;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<any>> {
    return this.request<PaginatedResponse<any>>('GET', '/regulatory/permits', undefined, params);
  }

  /**
   * Submit regulatory report
   */
  async submitRegulatoryReport(reportData: any): Promise<void> {
    await this.request('POST', '/regulatory/reports', reportData);
  }

  // ==================== Operator Dashboard ====================

  /**
   * Get operator dashboard summary
   */
  async getOperatorDashboard(operatorId?: string): Promise<OperatorDashboard> {
    const id = operatorId || this.config.operatorId;
    if (!id) {
      throw new Error('Operator ID is required');
    }
    return this.request<OperatorDashboard>('GET', `/operators/${id}/dashboard`);
  }

  /**
   * Get production summary by operator
   */
  async getOperatorProductionSummary(params: {
    operatorId?: string;
    startDate?: string;
    endDate?: string;
    aggregation?: 'daily' | 'monthly' | 'yearly';
  }): Promise<any> {
    return this.request('GET', '/operators/production-summary', undefined, params);
  }

  /**
   * Get environmental performance by operator
   */
  async getOperatorEnvironmentalPerformance(params: {
    operatorId?: string;
    year?: number;
  }): Promise<any> {
    return this.request('GET', '/operators/environmental-performance', undefined, params);
  }

  // ==================== Analytics & Insights ====================

  /**
   * Get decline curve analysis
   */
  async getDeclineCurveAnalysis(wellId: string): Promise<any> {
    return this.request('GET', `/analytics/decline-curve/${wellId}`);
  }

  /**
   * Get reservoir pressure trends
   */
  async getReservoirPressureTrends(wellId: string, params?: {
    startDate?: string;
    endDate?: string;
  }): Promise<any> {
    return this.request('GET', `/analytics/reservoir-pressure/${wellId}`, undefined, params);
  }

  /**
   * Get well performance comparison
   */
  async compareWells(wellIds: string[], metric: 'production' | 'emissions' | 'efficiency'): Promise<any> {
    return this.request('POST', '/analytics/compare', { wellIds, metric });
  }

  /**
   * Get drilling performance metrics
   */
  async getDrillingPerformance(wellId: string): Promise<any> {
    return this.request('GET', `/analytics/drilling-performance/${wellId}`);
  }

  // ==================== Decommissioning ====================

  /**
   * Submit P&A (Plug and Abandonment) plan
   */
  async submitPAndAPlan(wellId: string, plan: any): Promise<void> {
    await this.request('POST', `/decommissioning/pa-plan`, { wellId, ...plan });
  }

  /**
   * Update P&A progress
   */
  async updatePAndAProgress(wellId: string, progress: any): Promise<void> {
    await this.request('PUT', `/decommissioning/${wellId}/progress`, progress);
  }

  /**
   * Submit site restoration report
   */
  async submitRestorationReport(wellId: string, report: any): Promise<void> {
    await this.request('POST', `/decommissioning/${wellId}/restoration`, report);
  }

  /**
   * Get decommissioning status
   */
  async getDecommissioningStatus(wellId: string): Promise<any> {
    return this.request('GET', `/decommissioning/${wellId}/status`);
  }

  // ==================== Data Export ====================

  /**
   * Export well data to CSV
   */
  async exportWellData(params: {
    wellIds?: string[];
    operatorId?: string;
    format: 'csv' | 'json' | 'excel';
    startDate?: string;
    endDate?: string;
  }): Promise<Blob> {
    const response = await fetch(`${this.baseURL}/export/wells`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'X-API-Key': this.config.apiKey,
      },
      body: JSON.stringify(params),
    });

    if (!response.ok) {
      throw new Error(`Export failed: ${response.statusText}`);
    }

    return await response.blob();
  }

  /**
   * Export production data
   */
  async exportProductionData(params: {
    wellIds?: string[];
    operatorId?: string;
    format: 'csv' | 'json' | 'excel';
    startDate: string;
    endDate: string;
    aggregation?: 'daily' | 'monthly';
  }): Promise<Blob> {
    const response = await fetch(`${this.baseURL}/export/production`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'X-API-Key': this.config.apiKey,
      },
      body: JSON.stringify(params),
    });

    if (!response.ok) {
      throw new Error(`Export failed: ${response.statusText}`);
    }

    return await response.blob();
  }

  // ==================== Webhooks ====================

  /**
   * Register a webhook
   */
  async registerWebhook(params: {
    url: string;
    events: string[];
    secret?: string;
  }): Promise<{ webhookId: string }> {
    return this.request('POST', '/webhooks', params);
  }

  /**
   * List webhooks
   */
  async listWebhooks(): Promise<any[]> {
    const response = await this.request<{ data: any[] }>('GET', '/webhooks');
    return response.data;
  }

  /**
   * Delete a webhook
   */
  async deleteWebhook(webhookId: string): Promise<void> {
    await this.request('DELETE', `/webhooks/${webhookId}`);
  }

  /**
   * Test a webhook
   */
  async testWebhook(webhookId: string): Promise<void> {
    await this.request('POST', `/webhooks/${webhookId}/test`);
  }

  // ==================== Utility Methods ====================

  /**
   * Calculate gas-oil ratio
   */
  static calculateGOR(gasVolume: number, oilVolume: number, gasUnit: string = 'scf', oilUnit: string = 'bbl'): number {
    // Returns GOR in scf/bbl
    let gas = gasVolume;
    let oil = oilVolume;

    // Convert to standard units if needed
    if (gasUnit === 'mcf') gas *= 1000;
    if (gasUnit === 'm3') gas *= 35.3147; // m3 to scf

    if (oilUnit === 'm3') oil *= 6.28981; // m3 to bbl

    return gas / oil;
  }

  /**
   * Calculate oil API gravity from specific gravity
   */
  static calculateAPIGravity(specificGravity: number): number {
    return (141.5 / specificGravity) - 131.5;
  }

  /**
   * Calculate specific gravity from API gravity
   */
  static calculateSpecificGravity(apiGravity: number): number {
    return 141.5 / (apiGravity + 131.5);
  }

  /**
   * Convert pressure units
   */
  static convertPressure(value: number, fromUnit: string, toUnit: string): number {
    const toPsi: Record<string, number> = {
      psi: 1,
      bar: 14.5038,
      kPa: 0.145038,
      MPa: 145.038,
    };

    const psiValue = value * toPsi[fromUnit];
    return psiValue / toPsi[toUnit];
  }

  /**
   * Convert volume units
   */
  static convertVolume(value: number, fromUnit: string, toUnit: string): number {
    const toBbl: Record<string, number> = {
      bbl: 1,
      m3: 6.28981,
      scf: 0.178107, // at standard conditions
      mcf: 178.107,
    };

    const bblValue = value * toBbl[fromUnit];
    return bblValue / toBbl[toUnit];
  }

  /**
   * Estimate CO2 equivalent from flared gas
   */
  static calculateFlaringEmissions(volumeFlared: number, unit: string = 'scf'): number {
    // Returns tonnes CO2e
    let scf = volumeFlared;
    if (unit === 'mcf') scf *= 1000;
    if (unit === 'm3') scf *= 35.3147;

    // Assuming natural gas with 98.5% combustion efficiency
    // 1 scf of natural gas ≈ 0.0551 kg CO2
    const co2Kg = scf * 0.0551;
    return co2Kg / 1000; // Convert to tonnes
  }
}

/**
 * Helper function to create client instance
 */
export function createClient(config: ClientConfig): OilGasDrillingClient {
  return new OilGasDrillingClient(config);
}

/**
 * Default export
 */
export default OilGasDrillingClient;
