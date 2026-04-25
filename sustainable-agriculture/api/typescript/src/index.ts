/**
 * WIA-ENE-016: Sustainable Agriculture Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Export all types
export * from './types';

import {
  FarmProfile,
  CropRotationPlan,
  CertificationInfo,
  WaterUsageRecord,
  WaterConservationMetrics,
  SoilTestResults,
  SoilAmendment,
  BiodiversityAssessment,
  CarbonFootprint,
  IPMRecord,
  HarvestRecord,
  ProductionMetrics,
  SustainabilityKPIs,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
  EventType,
  EventPayload,
  FarmType,
  CertificationStandard,
  IrrigationType,
  SoilHealthIndicator,
  PlantingRecord,
  ClimateAdaptation,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface SustainableAgricultureSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// Event Handler Type
// ============================================================================

export type EventHandler = (event: EventPayload) => void;

// ============================================================================
// SDK Client
// ============================================================================

export class WIASustainableAgriculture {
  private config: Required<SustainableAgricultureSDKConfig>;
  private headers: Record<string, string>;
  private eventHandlers: Map<EventType, EventHandler[]>;
  private websocket?: WebSocket;

  constructor(config: SustainableAgricultureSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'ENE-016',
      'X-WIA-Version': '1.0.0',
    };

    this.eventHandlers = new Map();
  }

  // ==========================================================================
  // Farm Management APIs
  // ==========================================================================

  /**
   * Register a new farm
   */
  async registerFarm(
    farm: Omit<FarmProfile, 'farmId' | 'certifications'>
  ): Promise<ApiResponse<FarmProfile>> {
    return this.post<FarmProfile>('/api/v1/farm/register', farm);
  }

  /**
   * Get farm details
   */
  async getFarm(farmId: string): Promise<ApiResponse<FarmProfile>> {
    return this.get<FarmProfile>(`/api/v1/farm/${farmId}`);
  }

  /**
   * Update farm information
   */
  async updateFarm(
    farmId: string,
    updates: Partial<FarmProfile>
  ): Promise<ApiResponse<FarmProfile>> {
    return this.put<FarmProfile>(`/api/v1/farm/${farmId}`, updates);
  }

  /**
   * List all farms
   */
  async listFarms(params?: {
    pagination?: PaginationParams;
    farmType?: FarmType;
    region?: string;
  }): Promise<ApiResponse<PaginatedResponse<FarmProfile>>> {
    const queryParams = new URLSearchParams();

    if (params?.pagination) {
      Object.entries(params.pagination).forEach(([key, value]) => {
        if (value !== undefined) queryParams.append(key, String(value));
      });
    }

    if (params?.farmType) queryParams.append('farmType', params.farmType);
    if (params?.region) queryParams.append('region', params.region);

    return this.get<PaginatedResponse<FarmProfile>>(
      `/api/v1/farms?${queryParams.toString()}`
    );
  }

  /**
   * Delete farm
   */
  async deleteFarm(farmId: string): Promise<ApiResponse<void>> {
    return this.delete<void>(`/api/v1/farm/${farmId}`);
  }

  // ==========================================================================
  // Crop Planning APIs
  // ==========================================================================

  /**
   * Create crop rotation plan
   */
  async createRotationPlan(
    plan: Omit<CropRotationPlan, 'planId'>
  ): Promise<ApiResponse<CropRotationPlan>> {
    return this.post<CropRotationPlan>('/api/v1/crop/rotation', plan);
  }

  /**
   * Get rotation plan
   */
  async getRotationPlan(planId: string): Promise<ApiResponse<CropRotationPlan>> {
    return this.get<CropRotationPlan>(`/api/v1/crop/rotation/${planId}`);
  }

  /**
   * Update rotation plan
   */
  async updateRotationPlan(
    planId: string,
    updates: Partial<CropRotationPlan>
  ): Promise<ApiResponse<CropRotationPlan>> {
    return this.put<CropRotationPlan>(`/api/v1/crop/rotation/${planId}`, updates);
  }

  /**
   * Get crop recommendations
   */
  async getCropRecommendations(params: {
    farmId: string;
    season: string;
    previousCrops?: string[];
    soilHealth?: SoilHealthIndicator;
  }): Promise<ApiResponse<any[]>> {
    return this.post<any[]>('/api/v1/crop/recommendations', params);
  }

  /**
   * Record planting
   */
  async recordPlanting(
    planting: Omit<PlantingRecord, 'recordId'>
  ): Promise<ApiResponse<PlantingRecord>> {
    return this.post<PlantingRecord>('/api/v1/crop/planting', planting);
  }

  // ==========================================================================
  // Certification Tracking APIs
  // ==========================================================================

  /**
   * Apply for certification
   */
  async applyCertification(params: {
    farmId: string;
    standard: CertificationStandard;
    scope: string[];
  }): Promise<ApiResponse<CertificationInfo>> {
    return this.post<CertificationInfo>('/api/v1/certification/apply', params);
  }

  /**
   * Get certification status
   */
  async getCertification(certificationId: string): Promise<ApiResponse<CertificationInfo>> {
    return this.get<CertificationInfo>(`/api/v1/certification/${certificationId}`);
  }

  /**
   * List farm certifications
   */
  async listCertifications(farmId: string): Promise<ApiResponse<CertificationInfo[]>> {
    return this.get<CertificationInfo[]>(`/api/v1/farm/${farmId}/certifications`);
  }

  /**
   * Update certification status
   */
  async updateCertification(
    certificationId: string,
    updates: Partial<CertificationInfo>
  ): Promise<ApiResponse<CertificationInfo>> {
    return this.put<CertificationInfo>(`/api/v1/certification/${certificationId}`, updates);
  }

  /**
   * Schedule audit
   */
  async scheduleAudit(params: {
    certificationId: string;
    auditDate: string;
    auditor: string;
    auditType: 'annual' | 'surveillance' | 'unannounced';
  }): Promise<ApiResponse<any>> {
    return this.post<any>('/api/v1/certification/audit/schedule', params);
  }

  // ==========================================================================
  // Water Management APIs
  // ==========================================================================

  /**
   * Record water usage
   */
  async recordWaterUsage(
    usage: Omit<WaterUsageRecord, 'recordId'>
  ): Promise<ApiResponse<WaterUsageRecord>> {
    return this.post<WaterUsageRecord>('/api/v1/water/usage', usage);
  }

  /**
   * Get water usage history
   */
  async getWaterUsage(params: {
    farmId: string;
    dateRange?: DateRangeFilter;
  }): Promise<ApiResponse<PaginatedResponse<WaterUsageRecord>>> {
    const queryParams = new URLSearchParams({ farmId: params.farmId });

    if (params.dateRange) {
      queryParams.append('startDate', params.dateRange.startDate);
      queryParams.append('endDate', params.dateRange.endDate);
    }

    return this.get<PaginatedResponse<WaterUsageRecord>>(
      `/api/v1/water/usage?${queryParams.toString()}`
    );
  }

  /**
   * Get water conservation metrics
   */
  async getWaterMetrics(params: {
    farmId: string;
    period: DateRangeFilter;
  }): Promise<ApiResponse<WaterConservationMetrics>> {
    return this.post<WaterConservationMetrics>('/api/v1/water/metrics', params);
  }

  /**
   * Optimize irrigation schedule
   */
  async optimizeIrrigation(params: {
    farmId: string;
    fieldId: string;
    cropType: string;
    weatherForecast?: any;
  }): Promise<ApiResponse<any>> {
    return this.post<any>('/api/v1/water/optimize', params);
  }

  // ==========================================================================
  // Soil Health APIs
  // ==========================================================================

  /**
   * Submit soil test results
   */
  async submitSoilTest(
    test: Omit<SoilTestResults, 'testId'>
  ): Promise<ApiResponse<SoilTestResults>> {
    return this.post<SoilTestResults>('/api/v1/soil/test', test);
  }

  /**
   * Get soil test results
   */
  async getSoilTest(testId: string): Promise<ApiResponse<SoilTestResults>> {
    return this.get<SoilTestResults>(`/api/v1/soil/test/${testId}`);
  }

  /**
   * Get soil health history
   */
  async getSoilHealthHistory(params: {
    farmId: string;
    fieldId?: string;
    dateRange?: DateRangeFilter;
  }): Promise<ApiResponse<SoilTestResults[]>> {
    const queryParams = new URLSearchParams({ farmId: params.farmId });

    if (params.fieldId) queryParams.append('fieldId', params.fieldId);
    if (params.dateRange) {
      queryParams.append('startDate', params.dateRange.startDate);
      queryParams.append('endDate', params.dateRange.endDate);
    }

    return this.get<SoilTestResults[]>(`/api/v1/soil/history?${queryParams.toString()}`);
  }

  /**
   * Record soil amendment
   */
  async recordAmendment(
    amendment: Omit<SoilAmendment, 'amendmentId'>
  ): Promise<ApiResponse<SoilAmendment>> {
    return this.post<SoilAmendment>('/api/v1/soil/amendment', amendment);
  }

  /**
   * Get amendment recommendations
   */
  async getAmendmentRecommendations(params: {
    soilTestId: string;
    targetCrop?: string;
  }): Promise<ApiResponse<any[]>> {
    return this.post<any[]>('/api/v1/soil/recommendations', params);
  }

  // ==========================================================================
  // Carbon Tracking APIs
  // ==========================================================================

  /**
   * Calculate carbon footprint
   */
  async calculateCarbon(params: {
    farmId: string;
    period: DateRangeFilter;
    includeOffset?: boolean;
  }): Promise<ApiResponse<CarbonFootprint>> {
    return this.post<CarbonFootprint>('/api/v1/carbon/calculate', params);
  }

  /**
   * Get carbon footprint
   */
  async getCarbonFootprint(assessmentId: string): Promise<ApiResponse<CarbonFootprint>> {
    return this.get<CarbonFootprint>(`/api/v1/carbon/${assessmentId}`);
  }

  /**
   * Get carbon trends
   */
  async getCarbonTrends(params: {
    farmId: string;
    startYear: number;
    endYear: number;
  }): Promise<ApiResponse<any>> {
    const queryParams = new URLSearchParams({
      farmId: params.farmId,
      startYear: String(params.startYear),
      endYear: String(params.endYear),
    });

    return this.get<any>(`/api/v1/carbon/trends?${queryParams.toString()}`);
  }

  /**
   * Register carbon offset
   */
  async registerCarbonOffset(params: {
    farmId: string;
    amount: number;
    methodology: string;
    verificationBody?: string;
  }): Promise<ApiResponse<any>> {
    return this.post<any>('/api/v1/carbon/offset', params);
  }

  // ==========================================================================
  // Additional Management APIs
  // ==========================================================================

  /**
   * Record harvest
   */
  async recordHarvest(
    harvest: Omit<HarvestRecord, 'recordId'>
  ): Promise<ApiResponse<HarvestRecord>> {
    return this.post<HarvestRecord>('/api/v1/harvest/record', harvest);
  }

  /**
   * Get production metrics
   */
  async getProductionMetrics(params: {
    farmId: string;
    period: DateRangeFilter;
  }): Promise<ApiResponse<ProductionMetrics>> {
    return this.post<ProductionMetrics>('/api/v1/production/metrics', params);
  }

  /**
   * Record IPM activity
   */
  async recordIPM(
    ipm: Omit<IPMRecord, 'recordId'>
  ): Promise<ApiResponse<IPMRecord>> {
    return this.post<IPMRecord>('/api/v1/ipm/record', ipm);
  }

  /**
   * Assess biodiversity
   */
  async assessBiodiversity(
    assessment: Omit<BiodiversityAssessment, 'assessmentId'>
  ): Promise<ApiResponse<BiodiversityAssessment>> {
    return this.post<BiodiversityAssessment>('/api/v1/biodiversity/assess', assessment);
  }

  /**
   * Get sustainability KPIs
   */
  async getSustainabilityKPIs(params: {
    farmId: string;
    period: DateRangeFilter;
  }): Promise<ApiResponse<SustainabilityKPIs>> {
    return this.post<SustainabilityKPIs>('/api/v1/sustainability/kpi', params);
  }

  /**
   * Get climate adaptation plan
   */
  async getClimateAdaptation(farmId: string): Promise<ApiResponse<ClimateAdaptation>> {
    return this.get<ClimateAdaptation>(`/api/v1/climate/adaptation/${farmId}`);
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  /**
   * Register event handler
   */
  on(eventType: EventType, handler: EventHandler): void {
    if (!this.eventHandlers.has(eventType)) {
      this.eventHandlers.set(eventType, []);
    }
    this.eventHandlers.get(eventType)!.push(handler);
  }

  /**
   * Unregister event handler
   */
  off(eventType: EventType, handler: EventHandler): void {
    const handlers = this.eventHandlers.get(eventType);
    if (handlers) {
      const index = handlers.indexOf(handler);
      if (index > -1) {
        handlers.splice(index, 1);
      }
    }
  }

  /**
   * Connect to event stream
   */
  async connectEventStream(farmId?: string): Promise<void> {
    const wsUrl = this.config.endpoint.replace(/^http/, 'ws') + '/api/v1/events';
    const url = farmId ? `${wsUrl}?farmId=${farmId}` : wsUrl;

    this.websocket = new WebSocket(url);

    this.websocket.onmessage = (event) => {
      try {
        const payload: EventPayload = JSON.parse(event.data);
        this.handleEvent(payload);
      } catch (error) {
        if (this.config.debug) {
          console.error('[WIA-ENE-016] Failed to parse event:', error);
        }
      }
    };

    this.websocket.onerror = (error) => {
      if (this.config.debug) {
        console.error('[WIA-ENE-016] WebSocket error:', error);
      }
    };

    return new Promise((resolve, reject) => {
      if (!this.websocket) return reject(new Error('WebSocket not initialized'));

      this.websocket.onopen = () => resolve();
      this.websocket.onerror = reject;
    });
  }

  /**
   * Disconnect from event stream
   */
  disconnectEventStream(): void {
    if (this.websocket) {
      this.websocket.close();
      this.websocket = undefined;
    }
  }

  /**
   * Handle incoming event
   */
  private handleEvent(event: EventPayload): void {
    const handlers = this.eventHandlers.get(event.type);
    if (handlers) {
      handlers.forEach(handler => {
        try {
          handler(event);
        } catch (error) {
          if (this.config.debug) {
            console.error('[WIA-ENE-016] Event handler error:', error);
          }
        }
      });
    }
  }

  // ==========================================================================
  // HTTP Helper Methods
  // ==========================================================================

  private async request<T>(
    method: string,
    path: string,
    body?: any
  ): Promise<ApiResponse<T>> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA-ENE-016] ${method} ${url}`, body || '');
    }

    try {
      const response = await fetch(url, {
        method,
        headers: this.headers,
        body: body ? JSON.stringify(body) : undefined,
        signal: AbortSignal.timeout(this.config.timeout),
      });

      const data = await response.json();

      if (!response.ok) {
        return {
          success: false,
          error: {
            code: `HTTP_${response.status}`,
            message: data.message || response.statusText,
            details: data,
          },
          metadata: {
            timestamp: new Date().toISOString(),
            version: '1.0.0',
          },
        };
      }

      return {
        success: true,
        data: data as T,
        metadata: {
          timestamp: new Date().toISOString(),
          version: '1.0.0',
        },
      };
    } catch (error) {
      if (this.config.debug) {
        console.error('[WIA-ENE-016] Request failed:', error);
      }

      return {
        success: false,
        error: {
          code: 'NETWORK_ERROR',
          message: error instanceof Error ? error.message : 'Unknown error',
          details: error,
        },
        metadata: {
          timestamp: new Date().toISOString(),
          version: '1.0.0',
        },
      };
    }
  }

  private async get<T>(path: string): Promise<ApiResponse<T>> {
    return this.request<T>('GET', path);
  }

  private async post<T>(path: string, body: any): Promise<ApiResponse<T>> {
    return this.request<T>('POST', path, body);
  }

  private async put<T>(path: string, body: any): Promise<ApiResponse<T>> {
    return this.request<T>('PUT', path, body);
  }

  private async delete<T>(path: string): Promise<ApiResponse<T>> {
    return this.request<T>('DELETE', path);
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate water use efficiency
 */
export function calculateWaterEfficiency(
  waterUsed: number,        // cubic meters
  yieldProduced: number     // kg
): number {
  if (yieldProduced === 0) return 0;
  return waterUsed / yieldProduced;  // cubic meters per kg
}

/**
 * Calculate soil organic matter change
 */
export function calculateSOMChange(
  current: number,          // percentage
  previous: number,         // percentage
  years: number
): number {
  return ((current - previous) / previous) * 100 / years;  // % change per year
}

/**
 * Estimate carbon sequestration from cover crops
 */
export function estimateCoverCropSequestration(
  areaHectares: number,
  coverCropType: 'legume' | 'grass' | 'brassica',
  duration: number          // months
): number {
  // Simplified estimates in tons CO2e per hectare per year
  const rates = {
    legume: 2.5,
    grass: 2.0,
    brassica: 1.5,
  };

  return (rates[coverCropType] * areaHectares * duration) / 12;
}

/**
 * Calculate crop diversity index (Shannon index for crops)
 */
export function calculateCropDiversity(crops: Array<{ name: string; area: number }>): number {
  const totalArea = crops.reduce((sum, crop) => sum + crop.area, 0);
  if (totalArea === 0) return 0;

  return -1 * crops.reduce((sum, crop) => {
    const proportion = crop.area / totalArea;
    return sum + (proportion > 0 ? proportion * Math.log(proportion) : 0);
  }, 0);
}

/**
 * Assess certification readiness
 */
export function assessCertificationReadiness(params: {
  organicInputsPercent: number;
  chemicalFreeYears: number;
  recordKeeping: boolean;
  bufferZones: boolean;
  soilTests: boolean;
}): { ready: boolean; score: number; requirements: string[] } {
  const requirements: string[] = [];
  let score = 0;

  if (params.organicInputsPercent >= 95) {
    score += 30;
  } else {
    requirements.push(`Increase organic inputs to 95% (currently ${params.organicInputsPercent}%)`);
  }

  if (params.chemicalFreeYears >= 3) {
    score += 30;
  } else {
    requirements.push(`Complete ${3 - params.chemicalFreeYears} more years chemical-free`);
  }

  if (params.recordKeeping) {
    score += 20;
  } else {
    requirements.push('Implement comprehensive record keeping system');
  }

  if (params.bufferZones) {
    score += 10;
  } else {
    requirements.push('Establish buffer zones around fields');
  }

  if (params.soilTests) {
    score += 10;
  } else {
    requirements.push('Conduct annual soil testing');
  }

  return {
    ready: score >= 90,
    score,
    requirements,
  };
}

/**
 * Calculate net carbon balance
 */
export function calculateNetCarbon(
  emissions: number,        // tons CO2e
  sequestration: number     // tons CO2e
): { net: number; status: 'positive' | 'neutral' | 'negative' } {
  const net = emissions - sequestration;

  let status: 'positive' | 'neutral' | 'negative';
  if (net < -0.5) status = 'positive';      // More sequestration than emissions
  else if (net <= 0.5) status = 'neutral';  // Balanced
  else status = 'negative';                 // More emissions than sequestration

  return { net, status };
}

/**
 * Validate organic certification period
 */
export function isInTransition(
  conventionalEndDate: string,
  currentDate: string = new Date().toISOString()
): { inTransition: boolean; monthsRemaining: number } {
  const end = new Date(conventionalEndDate);
  const now = new Date(currentDate);
  const transitionEnd = new Date(end);
  transitionEnd.setFullYear(transitionEnd.getFullYear() + 3);  // 3-year transition

  const monthsRemaining = Math.max(
    0,
    (transitionEnd.getTime() - now.getTime()) / (1000 * 60 * 60 * 24 * 30)
  );

  return {
    inTransition: monthsRemaining > 0,
    monthsRemaining: Math.round(monthsRemaining),
  };
}

// ============================================================================
// Factory Function
// ============================================================================

/**
 * Create a new WIA Sustainable Agriculture SDK instance
 */
export function createSustainableAgricultureSDK(
  config: SustainableAgricultureSDKConfig
): WIASustainableAgriculture {
  return new WIASustainableAgriculture(config);
}

// ============================================================================
// Default Export
// ============================================================================

export default WIASustainableAgriculture;
