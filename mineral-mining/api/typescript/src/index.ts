/**
 * WIA-ENE-038: Sustainable Mineral Mining Standard - SDK Implementation
 *
 * @version 1.0.0
 * @license CC BY 4.0
 * @description Client SDK for mineral mining operations, environmental management,
 *              and supply chain traceability
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

import type {
  ClientConfig,
  Mine,
  CreateMineRequest,
  CreateMineResponse,
  UpdateMineStatusRequest,
  MineQueryParams,
  ProductionRecord,
  SubmitProductionRequest,
  ProductionQueryParams,
  SubmitEnvironmentalDataRequest,
  SubmitCommunityEngagementRequest,
  RegisterSupplyChainRequest,
  SubmitESGReportRequest,
  ComplianceStatus,
  OperatorDashboard,
  PaginatedResponse,
  APIError,
  SafetyIncident,
  SupplyChainTracking,
} from './types';

export * from './types';

/**
 * Mineral Mining Client
 */
export class MineralMiningClient {
  private config: Required<ClientConfig>;
  private baseURL: string;

  constructor(config: ClientConfig) {
    this.config = {
      apiKey: config.apiKey,
      endpoint: config.endpoint || 'https://api.wia.org/ene-038/v1',
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
      'User-Agent': 'WIA-ENE-038-SDK/1.0.0',
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

  // ==================== Mine Management ====================

  /**
   * Create a new mine
   */
  async createMine(request: CreateMineRequest): Promise<CreateMineResponse> {
    return this.request<CreateMineResponse>('POST', '/mines', request);
  }

  /**
   * Get mine details by ID
   */
  async getMine(mineId: string): Promise<Mine> {
    return this.request<Mine>('GET', `/mines/${mineId}`);
  }

  /**
   * Query mines with filters
   */
  async queryMines(params: MineQueryParams): Promise<PaginatedResponse<Mine>> {
    return this.request<PaginatedResponse<Mine>>('GET', '/mines', undefined, params);
  }

  /**
   * Update mine status
   */
  async updateMineStatus(request: UpdateMineStatusRequest): Promise<void> {
    await this.request('PUT', `/mines/${request.mineId}/status`, request);
  }

  /**
   * Update mine data (partial update)
   */
  async updateMine(mineId: string, updates: Partial<Mine>): Promise<void> {
    await this.request('PATCH', `/mines/${mineId}`, updates);
  }

  /**
   * Delete a mine record (admin only)
   */
  async deleteMine(mineId: string): Promise<void> {
    await this.request('DELETE', `/mines/${mineId}`);
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
   * Get cumulative production for a mine
   */
  async getCumulativeProduction(mineId: string, asOfDate?: string): Promise<any> {
    const params = asOfDate ? { asOfDate } : {};
    return this.request('GET', `/mines/${mineId}/production/cumulative`, undefined, params);
  }

  /**
   * Batch submit production data for multiple mines
   */
  async batchSubmitProduction(records: SubmitProductionRequest[]): Promise<void> {
    await this.request('POST', '/production/batch', { records });
  }

  /**
   * Get mineral production trends
   */
  async getProductionTrends(params: {
    mineId?: string;
    mineralType?: string;
    startDate: string;
    endDate: string;
    aggregation?: 'daily' | 'monthly' | 'yearly';
  }): Promise<any> {
    return this.request('GET', '/production/trends', undefined, params);
  }

  // ==================== Environmental Monitoring ====================

  /**
   * Submit environmental data
   */
  async submitEnvironmentalData(request: SubmitEnvironmentalDataRequest): Promise<void> {
    await this.request('POST', '/environmental/data', request);
  }

  /**
   * Get environmental data
   */
  async getEnvironmentalData(params: {
    mineId?: string;
    operatorId?: string;
    startDate?: string;
    endDate?: string;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<any>> {
    return this.request<PaginatedResponse<any>>('GET', '/environmental/data', undefined, params);
  }

  /**
   * Get tailings monitoring data
   */
  async getTailingsMonitoring(mineId: string): Promise<any> {
    return this.request('GET', `/environmental/tailings/${mineId}`);
  }

  /**
   * Submit tailings stability report
   */
  async submitTailingsReport(mineId: string, report: any): Promise<void> {
    await this.request('POST', `/environmental/tailings/${mineId}`, report);
  }

  /**
   * Get water usage statistics
   */
  async getWaterUsage(params: {
    mineId?: string;
    operatorId?: string;
    startDate?: string;
    endDate?: string;
    aggregation?: 'daily' | 'monthly' | 'yearly';
  }): Promise<any> {
    return this.request('GET', '/environmental/water', undefined, params);
  }

  /**
   * Get emissions data
   */
  async getEmissionsData(params: {
    mineId?: string;
    operatorId?: string;
    year?: number;
  }): Promise<any> {
    return this.request('GET', '/environmental/emissions', undefined, params);
  }

  /**
   * Get land reclamation status
   */
  async getLandReclamationStatus(mineId: string): Promise<any> {
    return this.request('GET', `/environmental/reclamation/${mineId}`);
  }

  /**
   * Submit land reclamation progress
   */
  async submitReclamationProgress(mineId: string, progress: any): Promise<void> {
    await this.request('POST', `/environmental/reclamation/${mineId}`, progress);
  }

  // ==================== Safety Management ====================

  /**
   * Report a safety incident
   */
  async reportSafetyIncident(incident: SafetyIncident): Promise<void> {
    await this.request('POST', '/safety/incidents', incident);
  }

  /**
   * Get safety incidents
   */
  async getSafetyIncidents(params: {
    mineId?: string;
    operatorId?: string;
    incidentType?: string;
    severity?: string;
    startDate?: string;
    endDate?: string;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<SafetyIncident>> {
    return this.request<PaginatedResponse<SafetyIncident>>('GET', '/safety/incidents', undefined, params);
  }

  /**
   * Get safety metrics
   */
  async getSafetyMetrics(params: {
    mineId?: string;
    operatorId?: string;
    year?: number;
  }): Promise<any> {
    return this.request('GET', '/safety/metrics', undefined, params);
  }

  /**
   * Submit safety training record
   */
  async submitSafetyTraining(mineId: string, training: any): Promise<void> {
    await this.request('POST', `/safety/training`, { mineId, ...training });
  }

  // ==================== Community Relations ====================

  /**
   * Submit community engagement data
   */
  async submitCommunityEngagement(request: SubmitCommunityEngagementRequest): Promise<void> {
    await this.request('POST', '/community/engagement', request);
  }

  /**
   * Get community engagement history
   */
  async getCommunityEngagement(params: {
    mineId?: string;
    operatorId?: string;
    startDate?: string;
    endDate?: string;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<any>> {
    return this.request<PaginatedResponse<any>>('GET', '/community/engagement', undefined, params);
  }

  /**
   * Submit grievance
   */
  async submitGrievance(mineId: string, grievance: any): Promise<void> {
    await this.request('POST', '/community/grievances', { mineId, ...grievance });
  }

  /**
   * Get grievance status
   */
  async getGrievances(params: {
    mineId?: string;
    status?: 'open' | 'resolved' | 'closed';
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<any>> {
    return this.request<PaginatedResponse<any>>('GET', '/community/grievances', undefined, params);
  }

  /**
   * Update grievance status
   */
  async updateGrievance(grievanceId: string, updates: any): Promise<void> {
    await this.request('PUT', `/community/grievances/${grievanceId}`, updates);
  }

  /**
   * Get social impact data
   */
  async getSocialImpact(mineId: string): Promise<any> {
    return this.request('GET', `/community/impact/${mineId}`);
  }

  // ==================== Supply Chain Traceability ====================

  /**
   * Register supply chain data
   */
  async registerSupplyChain(request: RegisterSupplyChainRequest): Promise<void> {
    await this.request('POST', '/supply-chain/register', request);
  }

  /**
   * Track shipment
   */
  async trackShipment(shipmentId: string): Promise<SupplyChainTracking> {
    return this.request<SupplyChainTracking>('GET', `/supply-chain/track/${shipmentId}`);
  }

  /**
   * Verify conflict-free status
   */
  async verifyConflictFree(params: {
    mineId?: string;
    shipmentId?: string;
  }): Promise<any> {
    return this.request('GET', '/supply-chain/conflict-free', undefined, params);
  }

  /**
   * Get supply chain by mine
   */
  async getSupplyChain(mineId: string, params?: {
    startDate?: string;
    endDate?: string;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<SupplyChainTracking>> {
    return this.request<PaginatedResponse<SupplyChainTracking>>(
      'GET',
      `/supply-chain/mine/${mineId}`,
      undefined,
      params
    );
  }

  /**
   * Submit blockchain verification
   */
  async submitBlockchainVerification(shipmentId: string, blockchainHash: string): Promise<void> {
    await this.request('POST', `/supply-chain/blockchain`, { shipmentId, blockchainHash });
  }

  /**
   * Get certification status
   */
  async getCertificationStatus(params: {
    mineId?: string;
    certificationType?: string;
  }): Promise<any> {
    return this.request('GET', '/supply-chain/certifications', undefined, params);
  }

  // ==================== ESG Reporting ====================

  /**
   * Submit ESG report
   */
  async submitESGReport(request: SubmitESGReportRequest): Promise<void> {
    await this.request('POST', '/esg/report', request);
  }

  /**
   * Get ESG metrics
   */
  async getESGMetrics(params: {
    mineId?: string;
    operatorId?: string;
    year?: number;
  }): Promise<any> {
    return this.request('GET', '/esg/metrics', undefined, params);
  }

  /**
   * Get ESG dashboard
   */
  async getESGDashboard(operatorId?: string): Promise<any> {
    const id = operatorId || this.config.operatorId;
    if (!id) {
      throw new Error('Operator ID is required');
    }
    return this.request('GET', `/esg/dashboard/${id}`);
  }

  /**
   * Get ESG score trends
   */
  async getESGScoreTrends(params: {
    operatorId?: string;
    startYear: number;
    endYear: number;
  }): Promise<any> {
    return this.request('GET', '/esg/trends', undefined, params);
  }

  // ==================== Regulatory Compliance ====================

  /**
   * Get compliance status for a mine or operator
   */
  async getComplianceStatus(mineId?: string, operatorId?: string): Promise<ComplianceStatus> {
    const params = { mineId, operatorId };
    return this.request<ComplianceStatus>('GET', '/regulatory/compliance', undefined, params);
  }

  /**
   * Get permit information
   */
  async getPermits(params: {
    mineId?: string;
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
   * Get mining efficiency analysis
   */
  async getMiningEfficiency(mineId: string): Promise<any> {
    return this.request('GET', `/analytics/efficiency/${mineId}`);
  }

  /**
   * Get resource depletion forecast
   */
  async getResourceForecast(mineId: string, years?: number): Promise<any> {
    const params = years ? { years } : {};
    return this.request('GET', `/analytics/resource-forecast/${mineId}`, undefined, params);
  }

  /**
   * Compare mines
   */
  async compareMines(mineIds: string[], metric: 'production' | 'environmental' | 'safety' | 'esg'): Promise<any> {
    return this.request('POST', '/analytics/compare', { mineIds, metric });
  }

  /**
   * Get benchmark data
   */
  async getBenchmark(params: {
    mineralType?: string;
    mineType?: string;
    metric: string;
  }): Promise<any> {
    return this.request('GET', '/analytics/benchmark', undefined, params);
  }

  // ==================== Mine Closure ====================

  /**
   * Submit mine closure plan
   */
  async submitClosurePlan(mineId: string, plan: any): Promise<void> {
    await this.request('POST', `/closure/plan`, { mineId, ...plan });
  }

  /**
   * Update closure progress
   */
  async updateClosureProgress(mineId: string, progress: any): Promise<void> {
    await this.request('PUT', `/closure/${mineId}/progress`, progress);
  }

  /**
   * Get closure status
   */
  async getClosureStatus(mineId: string): Promise<any> {
    return this.request('GET', `/closure/${mineId}/status`);
  }

  /**
   * Submit post-closure monitoring
   */
  async submitPostClosureMonitoring(mineId: string, monitoring: any): Promise<void> {
    await this.request('POST', `/closure/${mineId}/monitoring`, monitoring);
  }

  // ==================== Data Export ====================

  /**
   * Export mine data
   */
  async exportMineData(params: {
    mineIds?: string[];
    operatorId?: string;
    format: 'csv' | 'json' | 'excel';
    startDate?: string;
    endDate?: string;
  }): Promise<Blob> {
    const response = await fetch(`${this.baseURL}/export/mines`, {
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
   * Export ESG report
   */
  async exportESGReport(params: {
    operatorId?: string;
    year: number;
    format: 'pdf' | 'excel' | 'json';
  }): Promise<Blob> {
    const response = await fetch(`${this.baseURL}/export/esg`, {
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
   * Calculate recovery rate
   */
  static calculateRecoveryRate(produced: number, theoretical: number): number {
    return (produced / theoretical) * 100;
  }

  /**
   * Calculate water recycling rate
   */
  static calculateWaterRecyclingRate(recycled: number, total: number): number {
    return (recycled / total) * 100;
  }

  /**
   * Calculate carbon intensity
   */
  static calculateCarbonIntensity(emissions: number, production: number): number {
    // Returns tonnes CO2e per tonne of product
    return emissions / production;
  }

  /**
   * Calculate strip ratio
   */
  static calculateStripRatio(waste: number, ore: number): number {
    return waste / ore;
  }

  /**
   * Convert grade units
   */
  static convertGrade(value: number, fromUnit: string, toUnit: string): number {
    // Convert between percent, ppm, g/t, oz/ton
    const toPpm: Record<string, number> = {
      percent: 10000,
      ppm: 1,
      'g/t': 1,
      'oz/ton': 34285.7, // Troy ounce per short ton
    };

    const ppmValue = value * toPpm[fromUnit];
    return ppmValue / toPpm[toUnit];
  }

  /**
   * Estimate mine life
   */
  static estimateMineLife(reserves: number, annualProduction: number): number {
    // Returns years
    return reserves / annualProduction;
  }

  /**
   * Calculate LTIFR (Lost Time Injury Frequency Rate)
   */
  static calculateLTIFR(lostTimeInjuries: number, hoursWorked: number): number {
    // Per million hours worked
    return (lostTimeInjuries / hoursWorked) * 1000000;
  }
}

/**
 * Helper function to create client instance
 */
export function createClient(config: ClientConfig): MineralMiningClient {
  return new MineralMiningClient(config);
}

/**
 * Default export
 */
export default MineralMiningClient;
