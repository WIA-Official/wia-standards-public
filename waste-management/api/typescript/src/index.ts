/**
 * WIA-ENE-022: Waste Management Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Export all types
export * from './types';

import {
  WasteGenerationEvent,
  WasteCollectionEvent,
  WasteTreatmentFacility,
  RecyclingPerformance,
  SmartBin,
  MonitoringData,
  PerformanceReport,
  CollectionSchedule,
  KPIDashboard,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
  Certification,
  WasteCategoryCode,
  TreatmentMethod,
  FacilityType,
  OperationalStatus,
  ReportType,
  CollectionFrequency,
  CertificationLevel,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WasteManagementSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class WasteManagementSDK {
  private config: Required<WasteManagementSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: WasteManagementSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'ENE-022',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Waste Generation APIs
  // ==========================================================================

  /**
   * Create a new waste generation event
   */
  async createWasteEvent(
    event: Omit<WasteGenerationEvent, 'eventId' | 'timestamp' | 'metadata'>
  ): Promise<ApiResponse<WasteGenerationEvent>> {
    return this.post<WasteGenerationEvent>('/api/v1/waste/generate', event);
  }

  /**
   * Get waste event by ID
   */
  async getWasteEvent(eventId: string): Promise<ApiResponse<WasteGenerationEvent>> {
    return this.get<WasteGenerationEvent>(`/api/v1/waste/${eventId}`);
  }

  /**
   * Update waste event
   */
  async updateWasteEvent(
    eventId: string,
    updates: Partial<WasteGenerationEvent>
  ): Promise<ApiResponse<WasteGenerationEvent>> {
    return this.put<WasteGenerationEvent>(`/api/v1/waste/${eventId}`, updates);
  }

  /**
   * Delete waste event
   */
  async deleteWasteEvent(eventId: string): Promise<ApiResponse<void>> {
    return this.delete<void>(`/api/v1/waste/${eventId}`);
  }

  /**
   * List waste events with pagination
   */
  async listWasteEvents(
    params?: PaginationParams & { categoryCode?: WasteCategoryCode }
  ): Promise<ApiResponse<PaginatedResponse<WasteGenerationEvent>>> {
    const queryParams = new URLSearchParams();
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<WasteGenerationEvent>>(
      `/api/v1/waste?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Collection Management APIs
  // ==========================================================================

  /**
   * Create collection schedule
   */
  async createCollectionSchedule(
    schedule: Omit<CollectionSchedule, 'scheduleId'>
  ): Promise<ApiResponse<CollectionSchedule>> {
    return this.post<CollectionSchedule>('/api/v1/collection/schedule', schedule);
  }

  /**
   * Get collection schedule
   */
  async getCollectionSchedule(scheduleId: string): Promise<ApiResponse<CollectionSchedule>> {
    return this.get<CollectionSchedule>(`/api/v1/collection/schedule/${scheduleId}`);
  }

  /**
   * Get collection route
   */
  async getCollectionRoute(routeId: string): Promise<ApiResponse<WasteCollectionEvent>> {
    return this.get<WasteCollectionEvent>(`/api/v1/collection/route/${routeId}`);
  }

  /**
   * Report collection completion
   */
  async reportCollectionComplete(
    collection: WasteCollectionEvent
  ): Promise<ApiResponse<WasteCollectionEvent>> {
    return this.post<WasteCollectionEvent>('/api/v1/collection/complete', collection);
  }

  /**
   * List collection schedules by region
   */
  async listCollectionSchedules(
    region: string,
    categoryCode?: WasteCategoryCode
  ): Promise<ApiResponse<CollectionSchedule[]>> {
    const queryParams = new URLSearchParams({ region });
    if (categoryCode) {
      queryParams.append('categoryCode', categoryCode);
    }
    return this.get<CollectionSchedule[]>(
      `/api/v1/collection/schedules?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Smart Bin Management APIs
  // ==========================================================================

  /**
   * Get smart bin status
   */
  async getSmartBin(binId: string): Promise<ApiResponse<SmartBin>> {
    return this.get<SmartBin>(`/api/v1/smartbin/${binId}`);
  }

  /**
   * List smart bins needing collection
   */
  async getSmartBinsNeedingCollection(
    region?: string
  ): Promise<ApiResponse<SmartBin[]>> {
    const queryParams = region ? new URLSearchParams({ region }) : '';
    return this.get<SmartBin[]>(
      `/api/v1/smartbin/needs-collection${queryParams ? '?' + queryParams : ''}`
    );
  }

  /**
   * Update smart bin sensor data
   */
  async updateSmartBinData(
    binId: string,
    sensorData: SmartBin['sensorData']
  ): Promise<ApiResponse<SmartBin>> {
    return this.put<SmartBin>(`/api/v1/smartbin/${binId}/sensor`, { sensorData });
  }

  // ==========================================================================
  // Facility Management APIs
  // ==========================================================================

  /**
   * Get facility status
   */
  async getFacilityStatus(facilityId: string): Promise<ApiResponse<WasteTreatmentFacility>> {
    return this.get<WasteTreatmentFacility>(`/api/v1/facility/${facilityId}/status`);
  }

  /**
   * Get facility performance
   */
  async getFacilityPerformance(
    facilityId: string,
    dateRange?: DateRangeFilter
  ): Promise<ApiResponse<RecyclingPerformance>> {
    const queryParams = dateRange ? new URLSearchParams({
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
    }) : '';
    return this.get<RecyclingPerformance>(
      `/api/v1/facility/${facilityId}/performance${queryParams ? '?' + queryParams : ''}`
    );
  }

  /**
   * Submit monitoring data
   */
  async submitMonitoringData(
    facilityId: string,
    data: MonitoringData
  ): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/facility/${facilityId}/monitoring`, data);
  }

  /**
   * List facilities by type
   */
  async listFacilities(
    type?: FacilityType,
    status?: OperationalStatus
  ): Promise<ApiResponse<WasteTreatmentFacility[]>> {
    const queryParams = new URLSearchParams();
    if (type) queryParams.append('type', type);
    if (status) queryParams.append('status', status);
    return this.get<WasteTreatmentFacility[]>(
      `/api/v1/facilities${queryParams.toString() ? '?' + queryParams.toString() : ''}`
    );
  }

  // ==========================================================================
  // Analytics & Reporting APIs
  // ==========================================================================

  /**
   * Get recycling performance
   */
  async getRecyclingPerformance(
    region: string,
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<RecyclingPerformance>> {
    const queryParams = new URLSearchParams({
      region,
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
    });
    return this.get<RecyclingPerformance>(
      `/api/v1/analytics/performance?${queryParams.toString()}`
    );
  }

  /**
   * Get KPI dashboard
   */
  async getKPIDashboard(
    region?: string,
    date?: string
  ): Promise<ApiResponse<KPIDashboard>> {
    const queryParams = new URLSearchParams();
    if (region) queryParams.append('region', region);
    if (date) queryParams.append('date', date);
    return this.get<KPIDashboard>(
      `/api/v1/analytics/kpi${queryParams.toString() ? '?' + queryParams.toString() : ''}`
    );
  }

  /**
   * Generate performance report
   */
  async generateReport(
    reportType: ReportType,
    region: string,
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<PerformanceReport>> {
    return this.post<PerformanceReport>('/api/v1/analytics/report', {
      reportType,
      region,
      period: dateRange,
    });
  }

  /**
   * Get generated report
   */
  async getReport(reportId: string): Promise<ApiResponse<PerformanceReport>> {
    return this.get<PerformanceReport>(`/api/v1/analytics/report/${reportId}`);
  }

  // ==========================================================================
  // Certification APIs
  // ==========================================================================

  /**
   * Get facility certification
   */
  async getCertification(facilityId: string): Promise<ApiResponse<Certification>> {
    return this.get<Certification>(`/api/v1/certification/${facilityId}`);
  }

  /**
   * Apply for certification
   */
  async applyCertification(
    facilityId: string,
    level: CertificationLevel
  ): Promise<ApiResponse<Certification>> {
    return this.post<Certification>('/api/v1/certification/apply', {
      facilityId,
      level,
    });
  }

  /**
   * Renew certification
   */
  async renewCertification(certificateId: string): Promise<ApiResponse<Certification>> {
    return this.post<Certification>(`/api/v1/certification/${certificateId}/renew`, {});
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
      console.log(`[WIA-ENE-022] ${method} ${url}`, body || '');
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
        console.error('[WIA-ENE-022] Request failed:', error);
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
// Convenience Functions
// ============================================================================

/**
 * Calculate recycling rate
 */
export function calculateRecyclingRate(
  totalGenerated: number,
  totalRecycled: number
): number {
  if (totalGenerated === 0) return 0;
  return (totalRecycled / totalGenerated) * 100;
}

/**
 * Calculate diversion rate (waste diverted from landfill)
 */
export function calculateDiversionRate(
  totalGenerated: number,
  landfilled: number
): number {
  if (totalGenerated === 0) return 0;
  return ((totalGenerated - landfilled) / totalGenerated) * 100;
}

/**
 * Estimate CO2 reduction from recycling
 * Based on average emission factors for different materials
 */
export function estimateCO2Reduction(
  categoryCode: WasteCategoryCode,
  quantity: number // in kg
): number {
  // CO2 savings per kg of material (kg CO2)
  const emissionFactors: Record<WasteCategoryCode, number> = {
    [WasteCategoryCode.PAPER]: 1.5,
    [WasteCategoryCode.PLASTIC]: 2.5,
    [WasteCategoryCode.GLASS]: 0.5,
    [WasteCategoryCode.METAL]: 3.0,
    [WasteCategoryCode.FOOD]: 0.3,
    [WasteCategoryCode.TEXTILE]: 2.0,
    [WasteCategoryCode.WOOD]: 1.0,
    [WasteCategoryCode.ELECTRONICS]: 2.5,
    [WasteCategoryCode.HAZARDOUS]: 0.0,
    [WasteCategoryCode.OTHER]: 1.0,
  };

  return (quantity / 1000) * (emissionFactors[categoryCode] || 0); // Convert kg to tons
}

/**
 * Check if smart bin needs collection
 */
export function needsCollection(bin: SmartBin): boolean {
  return bin.sensorData.fillLevel >= 80;
}

/**
 * Calculate collection priority
 */
export function calculateCollectionPriority(bin: SmartBin): SmartBin['priority'] {
  const { fillLevel, temperature } = bin.sensorData;

  if (fillLevel >= 95 || temperature > 40) {
    return 'high';
  } else if (fillLevel >= 85 || temperature > 30) {
    return 'medium';
  } else {
    return 'low';
  }
}

/**
 * Validate waste category code
 */
export function isValidWasteCategory(code: string): code is WasteCategoryCode {
  return Object.values(WasteCategoryCode).includes(code as WasteCategoryCode);
}

/**
 * Validate treatment method
 */
export function isValidTreatmentMethod(method: string): method is TreatmentMethod {
  return Object.values(TreatmentMethod).includes(method as TreatmentMethod);
}

// ============================================================================
// Default Export
// ============================================================================

export default WasteManagementSDK;
