/**
 * WIA-ENE-037: Seabed Resource Development Standard - SDK Implementation
 *
 * @version 1.0.0
 * @license CC BY 4.0
 * @description Client SDK for seabed resource exploration, mining, and environmental monitoring
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

import type {
  ClientConfig,
  SeabedResourceSystem,
  CreateLicenseRequest,
  CreateLicenseResponse,
  ISALicense,
  SubmitExplorationSurveyRequest,
  SubmitSampleRequest,
  SubmitProductionRequest,
  SubmitEnvironmentalDataRequest,
  ReportIncidentRequest,
  QueryParams,
  PaginatedResponse,
  ComplianceStatus,
  OperatorDashboard,
  ExplorationSurvey,
  Sample,
  DailyProduction,
  ResourceEstimate,
  EmergencyResponse,
  AnnualReport,
  APIError,
} from './types';

export * from './types';

/**
 * Seabed Resource Client
 */
export class SeabedResourceClient {
  private config: Required<ClientConfig>;
  private baseURL: string;

  constructor(config: ClientConfig) {
    this.config = {
      apiKey: config.apiKey,
      endpoint: config.endpoint || 'https://api.wia.org/ene-037/v1',
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
      'User-Agent': 'WIA-ENE-037-SDK/1.0.0',
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
          await new Promise(resolve => setTimeout(resolve, Math.pow(2, attempt) * 1000));
        }
      }
    }

    throw lastError || new Error('Request failed');
  }

  // ==================== License Management ====================

  /**
   * Create a new license
   */
  async createLicense(request: CreateLicenseRequest): Promise<CreateLicenseResponse> {
    return this.request<CreateLicenseResponse>('POST', '/licenses', request);
  }

  /**
   * Get license details
   */
  async getLicense(licenseNumber: string): Promise<ISALicense> {
    return this.request<ISALicense>('GET', `/licenses/${licenseNumber}`);
  }

  /**
   * Query licenses
   */
  async queryLicenses(params: QueryParams): Promise<PaginatedResponse<ISALicense>> {
    return this.request<PaginatedResponse<ISALicense>>('GET', '/licenses', undefined, params);
  }

  /**
   * Update license
   */
  async updateLicense(licenseNumber: string, updates: Partial<ISALicense>): Promise<void> {
    await this.request('PATCH', `/licenses/${licenseNumber}`, updates);
  }

  // ==================== Exploration ====================

  /**
   * Submit exploration survey
   */
  async submitExplorationSurvey(request: SubmitExplorationSurveyRequest): Promise<void> {
    await this.request('POST', '/exploration/surveys', request);
  }

  /**
   * Get exploration surveys
   */
  async getExplorationSurveys(params: {
    licenseNumber?: string;
    surveyType?: string;
    startDate?: string;
    endDate?: string;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<ExplorationSurvey>> {
    return this.request<PaginatedResponse<ExplorationSurvey>>(
      'GET',
      '/exploration/surveys',
      undefined,
      params
    );
  }

  /**
   * Submit sample data
   */
  async submitSample(request: SubmitSampleRequest): Promise<void> {
    await this.request('POST', '/exploration/samples', request);
  }

  /**
   * Get samples
   */
  async getSamples(params: {
    licenseNumber?: string;
    sampleType?: string;
    startDate?: string;
    endDate?: string;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<Sample>> {
    return this.request<PaginatedResponse<Sample>>('GET', '/exploration/samples', undefined, params);
  }

  /**
   * Get resource estimate
   */
  async getResourceEstimate(licenseNumber: string): Promise<ResourceEstimate> {
    return this.request<ResourceEstimate>('GET', `/exploration/resources/${licenseNumber}`);
  }

  /**
   * Submit resource estimate
   */
  async submitResourceEstimate(licenseNumber: string, estimate: Partial<ResourceEstimate>): Promise<void> {
    await this.request('POST', '/exploration/resources', { licenseNumber, ...estimate });
  }

  // ==================== Mining Operations ====================

  /**
   * Register mining system
   */
  async registerMiningSystem(systemData: any): Promise<{ systemId: string }> {
    return this.request('POST', '/mining/systems', systemData);
  }

  /**
   * Get mining system
   */
  async getMiningSystem(systemId: string): Promise<any> {
    return this.request('GET', `/mining/systems/${systemId}`);
  }

  /**
   * Submit daily production
   */
  async submitProduction(request: SubmitProductionRequest): Promise<void> {
    await this.request('POST', '/mining/daily-production', request);
  }

  /**
   * Get production history
   */
  async getProductionHistory(params: {
    systemId?: string;
    licenseNumber?: string;
    startDate?: string;
    endDate?: string;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<DailyProduction>> {
    return this.request<PaginatedResponse<DailyProduction>>(
      'GET',
      '/mining/production-history',
      undefined,
      params
    );
  }

  /**
   * Get cumulative production
   */
  async getCumulativeProduction(systemId: string, asOfDate?: string): Promise<any> {
    const params = asOfDate ? { asOfDate } : {};
    return this.request('GET', `/mining/systems/${systemId}/cumulative`, undefined, params);
  }

  /**
   * Get mining statistics
   */
  async getMiningStatistics(params: {
    systemId?: string;
    operatorId?: string;
    resourceType?: string;
    startDate?: string;
    endDate?: string;
  }): Promise<any> {
    return this.request('GET', '/mining/statistics', undefined, params);
  }

  // ==================== Environmental Monitoring ====================

  /**
   * Submit baseline study
   */
  async submitBaselineStudy(licenseNumber: string, baselineData: any): Promise<void> {
    await this.request('POST', '/environmental/baseline', { licenseNumber, ...baselineData });
  }

  /**
   * Get baseline study
   */
  async getBaselineStudy(licenseNumber: string): Promise<any> {
    return this.request('GET', `/environmental/baseline/${licenseNumber}`);
  }

  /**
   * Submit environmental monitoring data
   */
  async submitEnvironmentalData(request: SubmitEnvironmentalDataRequest): Promise<void> {
    await this.request('POST', '/environmental/monitoring', request);
  }

  /**
   * Get environmental monitoring data
   */
  async getEnvironmentalMonitoring(params: {
    systemId?: string;
    licenseNumber?: string;
    monitoringType?: string;
    startDate?: string;
    endDate?: string;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<any>> {
    return this.request<PaginatedResponse<any>>('GET', '/environmental/monitoring', undefined, params);
  }

  /**
   * Report environmental incident
   */
  async reportIncident(request: ReportIncidentRequest): Promise<{ incidentId: string }> {
    return this.request('POST', '/environmental/incidents', request);
  }

  /**
   * Get environmental incidents
   */
  async getIncidents(params: {
    systemId?: string;
    licenseNumber?: string;
    incidentType?: string;
    severity?: string;
    startDate?: string;
    endDate?: string;
    resolved?: boolean;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<EmergencyResponse>> {
    return this.request<PaginatedResponse<EmergencyResponse>>('GET', '/environmental/incidents', undefined, params);
  }

  /**
   * Update incident status
   */
  async updateIncidentStatus(incidentId: string, updates: {
    resolved?: boolean;
    resolvedAt?: string;
    additionalActions?: any[];
  }): Promise<void> {
    await this.request('PATCH', `/environmental/incidents/${incidentId}`, updates);
  }

  /**
   * Get sediment plume data
   */
  async getSedimentPlumeData(params: {
    systemId?: string;
    startDate?: string;
    endDate?: string;
    exceedanceOnly?: boolean;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<any>> {
    return this.request<PaginatedResponse<any>>('GET', '/environmental/sediment-plumes', undefined, params);
  }

  /**
   * Get biological monitoring data
   */
  async getBiologicalMonitoring(params: {
    licenseNumber?: string;
    startDate?: string;
    endDate?: string;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<any>> {
    return this.request<PaginatedResponse<any>>('GET', '/environmental/biological', undefined, params);
  }

  /**
   * Get water quality data
   */
  async getWaterQualityData(params: {
    systemId?: string;
    licenseNumber?: string;
    startDate?: string;
    endDate?: string;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<any>> {
    return this.request<PaginatedResponse<any>>('GET', '/environmental/water-quality', undefined, params);
  }

  /**
   * Submit environmental management plan
   */
  async submitEnvironmentalManagementPlan(licenseNumber: string, empData: any): Promise<void> {
    await this.request('POST', '/environmental/management-plans', { licenseNumber, ...empData });
  }

  /**
   * Get environmental management plan
   */
  async getEnvironmentalManagementPlan(licenseNumber: string): Promise<any> {
    return this.request('GET', `/environmental/management-plans/${licenseNumber}`);
  }

  // ==================== ROV/AUV Operations ====================

  /**
   * Register vehicle mission
   */
  async registerVehicleMission(missionData: any): Promise<{ missionId: string }> {
    return this.request('POST', '/vehicles/missions', missionData);
  }

  /**
   * Get vehicle mission
   */
  async getVehicleMission(missionId: string): Promise<any> {
    return this.request('GET', `/vehicles/missions/${missionId}`);
  }

  /**
   * Query vehicle missions
   */
  async queryVehicleMissions(params: {
    licenseNumber?: string;
    vehicleType?: 'ROV' | 'AUV';
    missionType?: string;
    startDate?: string;
    endDate?: string;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<any>> {
    return this.request<PaginatedResponse<any>>('GET', '/vehicles/missions', undefined, params);
  }

  /**
   * Submit vehicle telemetry
   */
  async submitVehicleTelemetry(missionId: string, telemetryData: any): Promise<void> {
    await this.request('POST', `/vehicles/missions/${missionId}/telemetry`, telemetryData);
  }

  /**
   * Get vehicle status
   */
  async getVehicleStatus(vehicleId: string): Promise<any> {
    return this.request('GET', `/vehicles/${vehicleId}/status`);
  }

  // ==================== Regulatory & Compliance ====================

  /**
   * Submit annual report
   */
  async submitAnnualReport(reportData: AnnualReport): Promise<void> {
    await this.request('POST', '/regulatory/annual-reports', reportData);
  }

  /**
   * Get annual reports
   */
  async getAnnualReports(params: {
    operatorId?: string;
    licenseNumber?: string;
    year?: number;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<AnnualReport>> {
    return this.request<PaginatedResponse<AnnualReport>>('GET', '/regulatory/annual-reports', undefined, params);
  }

  /**
   * Get compliance status
   */
  async getComplianceStatus(systemId?: string, licenseNumber?: string): Promise<ComplianceStatus> {
    const params = { systemId, licenseNumber };
    return this.request<ComplianceStatus>('GET', '/regulatory/compliance', undefined, params);
  }

  /**
   * Get permits
   */
  async getPermits(params: {
    licenseNumber?: string;
    operatorId?: string;
    permitType?: string;
    jurisdiction?: string;
    status?: string;
    expiringWithinDays?: number;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<any>> {
    return this.request<PaginatedResponse<any>>('GET', '/regulatory/permits', undefined, params);
  }

  /**
   * Submit ISA report
   */
  async submitISAReport(reportData: any): Promise<void> {
    await this.request('POST', '/regulatory/isa-reports', reportData);
  }

  // ==================== Operator Dashboard ====================

  /**
   * Get operator dashboard
   */
  async getOperatorDashboard(operatorId?: string): Promise<OperatorDashboard> {
    const id = operatorId || this.config.operatorId;
    if (!id) {
      throw new Error('Operator ID is required');
    }
    return this.request<OperatorDashboard>('GET', `/operators/${id}/dashboard`);
  }

  /**
   * Get production summary
   */
  async getProductionSummary(params: {
    operatorId?: string;
    resourceType?: string;
    startDate?: string;
    endDate?: string;
    aggregation?: 'daily' | 'monthly' | 'yearly';
  }): Promise<any> {
    return this.request('GET', '/operators/production-summary', undefined, params);
  }

  /**
   * Get environmental performance
   */
  async getEnvironmentalPerformance(params: {
    operatorId?: string;
    licenseNumber?: string;
    year?: number;
  }): Promise<any> {
    return this.request('GET', '/operators/environmental-performance', undefined, params);
  }

  /**
   * Get resource portfolio
   */
  async getResourcePortfolio(operatorId?: string): Promise<any> {
    const id = operatorId || this.config.operatorId;
    if (!id) {
      throw new Error('Operator ID is required');
    }
    return this.request('GET', `/operators/${id}/portfolio`);
  }

  // ==================== Analytics & Insights ====================

  /**
   * Get resource assessment trends
   */
  async getResourceTrends(licenseNumber: string, params?: {
    startDate?: string;
    endDate?: string;
  }): Promise<any> {
    return this.request('GET', `/analytics/resource-trends/${licenseNumber}`, undefined, params);
  }

  /**
   * Get production efficiency
   */
  async getProductionEfficiency(systemId: string, params?: {
    startDate?: string;
    endDate?: string;
  }): Promise<any> {
    return this.request('GET', `/analytics/production-efficiency/${systemId}`, undefined, params);
  }

  /**
   * Get environmental impact analysis
   */
  async getEnvironmentalImpactAnalysis(systemId: string): Promise<any> {
    return this.request('GET', `/analytics/environmental-impact/${systemId}`);
  }

  /**
   * Compare systems
   */
  async compareSystems(systemIds: string[], metric: 'production' | 'efficiency' | 'environmental'): Promise<any> {
    return this.request('POST', '/analytics/compare', { systemIds, metric });
  }

  /**
   * Get biodiversity assessment
   */
  async getBiodiversityAssessment(licenseNumber: string): Promise<any> {
    return this.request('GET', `/analytics/biodiversity/${licenseNumber}`);
  }

  // ==================== Data Export ====================

  /**
   * Export exploration data
   */
  async exportExplorationData(params: {
    licenseNumber?: string;
    format: 'csv' | 'json' | 'excel';
    startDate?: string;
    endDate?: string;
  }): Promise<Blob> {
    const response = await fetch(`${this.baseURL}/export/exploration`, {
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
    systemId?: string;
    licenseNumber?: string;
    format: 'csv' | 'json' | 'excel';
    startDate: string;
    endDate: string;
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

  /**
   * Export environmental data
   */
  async exportEnvironmentalData(params: {
    systemId?: string;
    licenseNumber?: string;
    format: 'csv' | 'json' | 'excel';
    startDate: string;
    endDate: string;
    dataType?: 'all' | 'sediment_plumes' | 'biological' | 'water_quality';
  }): Promise<Blob> {
    const response = await fetch(`${this.baseURL}/export/environmental`, {
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
   * Calculate metal content from nodules
   */
  static calculateMetalContent(
    noduleWeight: number,
    metalGrade: number,
    weightUnit: string = 'tonnes',
    gradeUnit: string = 'percent'
  ): number {
    let weight = noduleWeight;
    let grade = metalGrade;

    // Convert to standard units
    if (weightUnit === 'kg') weight /= 1000;
    if (gradeUnit === 'fraction') grade *= 100;

    return (weight * grade) / 100;
  }

  /**
   * Convert depth to pressure
   */
  static depthToPressure(depth: number, depthUnit: string = 'meters'): number {
    // Returns pressure in bar
    let depthM = depth;
    if (depthUnit === 'feet') depthM *= 0.3048;

    // Hydrostatic pressure: P = ρgh
    // Seawater density ≈ 1025 kg/m³
    // g = 9.81 m/s²
    // Plus atmospheric pressure (≈ 1 bar)
    return (1025 * 9.81 * depthM) / 100000 + 1;
  }

  /**
   * Calculate gas volume from hydrate
   */
  static hydrateToGasVolume(hydrateVolume: number, hydrateFactor: number = 164): number {
    // Returns gas volume in m³ at STP
    // Default: 1 m³ hydrate → 164 m³ CH₄
    return hydrateVolume * hydrateFactor;
  }

  /**
   * Calculate plume dispersion distance
   */
  static calculatePlumeDispersion(
    particleSize: number, // micrometers
    currentSpeed: number, // m/s
    releaseHeight: number, // meters above seafloor
    settlingVelocity?: number // mm/s
  ): number {
    // Simplified calculation for plume dispersion distance (meters)
    // Stokes' law for settling velocity if not provided
    const sv = settlingVelocity || this.calculateSettlingVelocity(particleSize);

    // Time to settle (seconds)
    const settlingTime = (releaseHeight * 1000) / sv; // convert m to mm

    // Horizontal distance (meters)
    return currentSpeed * settlingTime;
  }

  /**
   * Calculate particle settling velocity (Stokes' law)
   */
  static calculateSettlingVelocity(particleSize: number): number {
    // Returns settling velocity in mm/s
    // particleSize in micrometers
    // Simplified Stokes' law for marine sediment
    const d = particleSize / 1000; // convert to mm
    const rho_p = 2650; // particle density (kg/m³) - quartz
    const rho_f = 1025; // fluid density (kg/m³) - seawater
    const mu = 0.00108; // dynamic viscosity (Pa·s) - seawater at 2°C
    const g = 9.81; // gravity (m/s²)

    // Stokes' law: v = (g * d² * (ρp - ρf)) / (18 * μ)
    const v = (g * Math.pow(d / 1000, 2) * (rho_p - rho_f)) / (18 * mu);

    return v * 1000; // convert m/s to mm/s
  }

  /**
   * Estimate recovery time for benthic habitat
   */
  static estimateRecoveryTime(
    impactType: 'direct' | 'sedimentation',
    habitatType: 'abyssal_plain' | 'seamount' | 'ridge',
    depth: number
  ): number {
    // Returns estimated recovery time in years
    // Based on literature and deep-sea ecology research
    const baseRecoveryTimes: Record<string, Record<string, number>> = {
      direct: {
        abyssal_plain: 50,
        seamount: 100,
        ridge: 75,
      },
      sedimentation: {
        abyssal_plain: 25,
        seamount: 50,
        ridge: 35,
      },
    };

    let recoveryTime = baseRecoveryTimes[impactType]?.[habitatType] || 50;

    // Adjust for depth (slower recovery at greater depths)
    if (depth > 5000) recoveryTime *= 1.5;
    else if (depth > 4000) recoveryTime *= 1.25;

    return Math.round(recoveryTime);
  }
}

/**
 * Helper function to create client instance
 */
export function createClient(config: ClientConfig): SeabedResourceClient {
  return new SeabedResourceClient(config);
}

/**
 * Default export
 */
export default SeabedResourceClient;
