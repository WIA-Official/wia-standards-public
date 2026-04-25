/**
 * WIA-ENE-034: Drought Monitoring Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Export all types
export * from './types';

import {
  SPIData,
  PDSIData,
  SPEIData,
  SoilMoistureData,
  GroundwaterData,
  ReservoirStorageData,
  PrecipitationDeficitData,
  VegetationStressData,
  AgriculturalImpactData,
  WaterRestrictionData,
  DroughtDeclarationData,
  DroughtMonitoringReport,
  DroughtForecastData,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
  DroughtSeverity,
  DroughtType,
  AlertLevel,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface DroughtMonitoringSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class DroughtMonitoringSDK {
  private config: Required<DroughtMonitoringSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: DroughtMonitoringSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'ENE-034',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Drought Indices APIs
  // ==========================================================================

  /**
   * Get Standardized Precipitation Index (SPI) data
   */
  async getSPI(
    location: { latitude: number; longitude: number },
    dateRange?: DateRangeFilter
  ): Promise<ApiResponse<SPIData[]>> {
    const queryParams = new URLSearchParams({
      latitude: location.latitude.toString(),
      longitude: location.longitude.toString(),
    });
    if (dateRange) {
      queryParams.append('startDate', dateRange.startDate);
      queryParams.append('endDate', dateRange.endDate);
    }
    return this.get<SPIData[]>(`/api/v1/indices/spi?${queryParams.toString()}`);
  }

  /**
   * Get Palmer Drought Severity Index (PDSI) data
   */
  async getPDSI(
    location: { latitude: number; longitude: number },
    dateRange?: DateRangeFilter
  ): Promise<ApiResponse<PDSIData[]>> {
    const queryParams = new URLSearchParams({
      latitude: location.latitude.toString(),
      longitude: location.longitude.toString(),
    });
    if (dateRange) {
      queryParams.append('startDate', dateRange.startDate);
      queryParams.append('endDate', dateRange.endDate);
    }
    return this.get<PDSIData[]>(`/api/v1/indices/pdsi?${queryParams.toString()}`);
  }

  /**
   * Get Standardized Precipitation Evapotranspiration Index (SPEI) data
   */
  async getSPEI(
    location: { latitude: number; longitude: number },
    dateRange?: DateRangeFilter
  ): Promise<ApiResponse<SPEIData[]>> {
    const queryParams = new URLSearchParams({
      latitude: location.latitude.toString(),
      longitude: location.longitude.toString(),
    });
    if (dateRange) {
      queryParams.append('startDate', dateRange.startDate);
      queryParams.append('endDate', dateRange.endDate);
    }
    return this.get<SPEIData[]>(`/api/v1/indices/spei?${queryParams.toString()}`);
  }

  // ==========================================================================
  // Soil Moisture APIs
  // ==========================================================================

  /**
   * Get soil moisture data
   */
  async getSoilMoisture(
    location: { latitude: number; longitude: number },
    dateRange?: DateRangeFilter
  ): Promise<ApiResponse<SoilMoistureData[]>> {
    const queryParams = new URLSearchParams({
      latitude: location.latitude.toString(),
      longitude: location.longitude.toString(),
    });
    if (dateRange) {
      queryParams.append('startDate', dateRange.startDate);
      queryParams.append('endDate', dateRange.endDate);
    }
    return this.get<SoilMoistureData[]>(`/api/v1/soil-moisture?${queryParams.toString()}`);
  }

  /**
   * Submit soil moisture measurement
   */
  async submitSoilMoisture(
    data: Omit<SoilMoistureData, 'monitoringId'>
  ): Promise<ApiResponse<SoilMoistureData>> {
    return this.post<SoilMoistureData>('/api/v1/soil-moisture', data);
  }

  // ==========================================================================
  // Groundwater APIs
  // ==========================================================================

  /**
   * List groundwater monitoring wells
   */
  async listGroundwaterWells(
    params?: PaginationParams & { region?: string }
  ): Promise<ApiResponse<PaginatedResponse<GroundwaterData>>> {
    const queryParams = new URLSearchParams();
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<GroundwaterData>>(
      `/api/v1/groundwater/wells?${queryParams.toString()}`
    );
  }

  /**
   * Get groundwater level data
   */
  async getGroundwaterLevel(
    wellId: string,
    dateRange?: DateRangeFilter
  ): Promise<ApiResponse<GroundwaterData[]>> {
    const queryParams = dateRange
      ? new URLSearchParams({
          startDate: dateRange.startDate,
          endDate: dateRange.endDate,
        })
      : '';
    return this.get<GroundwaterData[]>(
      `/api/v1/groundwater/wells/${wellId}/levels${queryParams ? '?' + queryParams : ''}`
    );
  }

  /**
   * Submit groundwater level measurement
   */
  async submitGroundwaterLevel(
    data: Omit<GroundwaterData, 'monitoringId'>
  ): Promise<ApiResponse<GroundwaterData>> {
    return this.post<GroundwaterData>('/api/v1/groundwater/levels', data);
  }

  // ==========================================================================
  // Reservoir Storage APIs
  // ==========================================================================

  /**
   * List reservoirs
   */
  async listReservoirs(
    params?: PaginationParams & { region?: string }
  ): Promise<ApiResponse<PaginatedResponse<ReservoirStorageData>>> {
    const queryParams = new URLSearchParams();
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<ReservoirStorageData>>(
      `/api/v1/reservoirs?${queryParams.toString()}`
    );
  }

  /**
   * Get reservoir storage data
   */
  async getReservoirStorage(
    reservoirId: string,
    dateRange?: DateRangeFilter
  ): Promise<ApiResponse<ReservoirStorageData[]>> {
    const queryParams = dateRange
      ? new URLSearchParams({
          startDate: dateRange.startDate,
          endDate: dateRange.endDate,
        })
      : '';
    return this.get<ReservoirStorageData[]>(
      `/api/v1/reservoirs/${reservoirId}/storage${queryParams ? '?' + queryParams : ''}`
    );
  }

  /**
   * Submit reservoir storage data
   */
  async submitReservoirStorage(
    data: Omit<ReservoirStorageData, 'monitoringId'>
  ): Promise<ApiResponse<ReservoirStorageData>> {
    return this.post<ReservoirStorageData>('/api/v1/reservoirs/storage', data);
  }

  // ==========================================================================
  // Precipitation APIs
  // ==========================================================================

  /**
   * Get precipitation deficit analysis
   */
  async getPrecipitationDeficit(
    location: { latitude: number; longitude: number },
    dateRange?: DateRangeFilter
  ): Promise<ApiResponse<PrecipitationDeficitData[]>> {
    const queryParams = new URLSearchParams({
      latitude: location.latitude.toString(),
      longitude: location.longitude.toString(),
    });
    if (dateRange) {
      queryParams.append('startDate', dateRange.startDate);
      queryParams.append('endDate', dateRange.endDate);
    }
    return this.get<PrecipitationDeficitData[]>(
      `/api/v1/precipitation/deficit?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Vegetation Stress APIs
  // ==========================================================================

  /**
   * Get vegetation stress data from satellite
   */
  async getVegetationStress(
    location: { latitude: number; longitude: number },
    dateRange?: DateRangeFilter
  ): Promise<ApiResponse<VegetationStressData[]>> {
    const queryParams = new URLSearchParams({
      latitude: location.latitude.toString(),
      longitude: location.longitude.toString(),
    });
    if (dateRange) {
      queryParams.append('startDate', dateRange.startDate);
      queryParams.append('endDate', dateRange.endDate);
    }
    return this.get<VegetationStressData[]>(
      `/api/v1/vegetation/stress?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Agricultural Impact APIs
  // ==========================================================================

  /**
   * Get agricultural drought impact
   */
  async getAgriculturalImpact(
    location: { latitude: number; longitude: number },
    dateRange?: DateRangeFilter
  ): Promise<ApiResponse<AgriculturalImpactData[]>> {
    const queryParams = new URLSearchParams({
      latitude: location.latitude.toString(),
      longitude: location.longitude.toString(),
    });
    if (dateRange) {
      queryParams.append('startDate', dateRange.startDate);
      queryParams.append('endDate', dateRange.endDate);
    }
    return this.get<AgriculturalImpactData[]>(
      `/api/v1/agriculture/impact?${queryParams.toString()}`
    );
  }

  /**
   * Submit agricultural impact assessment
   */
  async submitAgriculturalImpact(
    data: Omit<AgriculturalImpactData, 'monitoringId'>
  ): Promise<ApiResponse<AgriculturalImpactData>> {
    return this.post<AgriculturalImpactData>('/api/v1/agriculture/impact', data);
  }

  // ==========================================================================
  // Water Restrictions APIs
  // ==========================================================================

  /**
   * Get current water restrictions
   */
  async getWaterRestrictions(
    region: string
  ): Promise<ApiResponse<WaterRestrictionData[]>> {
    const queryParams = new URLSearchParams({ region });
    return this.get<WaterRestrictionData[]>(
      `/api/v1/restrictions?${queryParams.toString()}`
    );
  }

  /**
   * Update water restriction level
   */
  async updateWaterRestrictions(
    restrictionId: string,
    updates: Partial<WaterRestrictionData>
  ): Promise<ApiResponse<WaterRestrictionData>> {
    return this.put<WaterRestrictionData>(
      `/api/v1/restrictions/${restrictionId}`,
      updates
    );
  }

  // ==========================================================================
  // Drought Declaration APIs
  // ==========================================================================

  /**
   * List drought declarations
   */
  async listDroughtDeclarations(
    params?: PaginationParams & { region?: string; status?: string }
  ): Promise<ApiResponse<PaginatedResponse<DroughtDeclarationData>>> {
    const queryParams = new URLSearchParams();
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<DroughtDeclarationData>>(
      `/api/v1/declarations?${queryParams.toString()}`
    );
  }

  /**
   * Get drought declaration details
   */
  async getDroughtDeclaration(
    declarationId: string
  ): Promise<ApiResponse<DroughtDeclarationData>> {
    return this.get<DroughtDeclarationData>(`/api/v1/declarations/${declarationId}`);
  }

  /**
   * Create drought declaration
   */
  async createDroughtDeclaration(
    data: Omit<DroughtDeclarationData, 'declarationId'>
  ): Promise<ApiResponse<DroughtDeclarationData>> {
    return this.post<DroughtDeclarationData>('/api/v1/declarations', data);
  }

  /**
   * Update drought declaration
   */
  async updateDroughtDeclaration(
    declarationId: string,
    updates: Partial<DroughtDeclarationData>
  ): Promise<ApiResponse<DroughtDeclarationData>> {
    return this.put<DroughtDeclarationData>(
      `/api/v1/declarations/${declarationId}`,
      updates
    );
  }

  // ==========================================================================
  // Integrated Monitoring APIs
  // ==========================================================================

  /**
   * Get comprehensive drought monitoring report
   */
  async getDroughtMonitoringReport(
    location: { latitude: number; longitude: number },
    date?: string
  ): Promise<ApiResponse<DroughtMonitoringReport>> {
    const queryParams = new URLSearchParams({
      latitude: location.latitude.toString(),
      longitude: location.longitude.toString(),
    });
    if (date) {
      queryParams.append('date', date);
    }
    return this.get<DroughtMonitoringReport>(
      `/api/v1/monitoring/report?${queryParams.toString()}`
    );
  }

  /**
   * Get drought status for a region
   */
  async getRegionalDroughtStatus(
    region: string,
    date?: string
  ): Promise<ApiResponse<DroughtMonitoringReport[]>> {
    const queryParams = new URLSearchParams({ region });
    if (date) {
      queryParams.append('date', date);
    }
    return this.get<DroughtMonitoringReport[]>(
      `/api/v1/monitoring/regional?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Forecast and Early Warning APIs
  // ==========================================================================

  /**
   * Get drought forecast
   */
  async getDroughtForecast(
    location: { latitude: number; longitude: number }
  ): Promise<ApiResponse<DroughtForecastData>> {
    const queryParams = new URLSearchParams({
      latitude: location.latitude.toString(),
      longitude: location.longitude.toString(),
    });
    return this.get<DroughtForecastData>(
      `/api/v1/forecast?${queryParams.toString()}`
    );
  }

  /**
   * Get early warning alerts
   */
  async getEarlyWarningAlerts(
    region: string
  ): Promise<ApiResponse<DroughtForecastData[]>> {
    const queryParams = new URLSearchParams({ region });
    return this.get<DroughtForecastData[]>(
      `/api/v1/alerts/early-warning?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Analytics APIs
  // ==========================================================================

  /**
   * Get drought trend analysis
   */
  async getDroughtTrends(
    location: { latitude: number; longitude: number },
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<any>> {
    const queryParams = new URLSearchParams({
      latitude: location.latitude.toString(),
      longitude: location.longitude.toString(),
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
    });
    return this.get<any>(`/api/v1/analytics/trends?${queryParams.toString()}`);
  }

  /**
   * Generate drought impact report
   */
  async generateImpactReport(
    region: string,
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<any>> {
    return this.post<any>('/api/v1/analytics/impact-report', {
      region,
      period: dateRange,
    });
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
      console.log(`[WIA-ENE-034] ${method} ${url}`, body || '');
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
        console.error('[WIA-ENE-034] Request failed:', error);
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
 * Calculate drought severity from SPI value
 */
export function getDroughtSeverityFromSPI(spi: number): DroughtSeverity {
  if (spi >= -0.5) return DroughtSeverity.NORMAL;
  if (spi >= -0.79) return DroughtSeverity.ABNORMALLY_DRY;
  if (spi >= -1.29) return DroughtSeverity.MODERATE_DROUGHT;
  if (spi >= -1.59) return DroughtSeverity.SEVERE_DROUGHT;
  if (spi >= -1.99) return DroughtSeverity.EXTREME_DROUGHT;
  return DroughtSeverity.EXCEPTIONAL_DROUGHT;
}

/**
 * Calculate drought severity from PDSI value
 */
export function getDroughtSeverityFromPDSI(pdsi: number): DroughtSeverity {
  if (pdsi >= -1.0) return DroughtSeverity.NORMAL;
  if (pdsi >= -2.0) return DroughtSeverity.MODERATE_DROUGHT;
  if (pdsi >= -3.0) return DroughtSeverity.SEVERE_DROUGHT;
  if (pdsi >= -4.0) return DroughtSeverity.EXTREME_DROUGHT;
  return DroughtSeverity.EXCEPTIONAL_DROUGHT;
}

/**
 * Calculate Vegetation Health Index (VHI)
 * VHI = 0.5 * VCI + 0.5 * TCI
 */
export function calculateVHI(vci: number, tci: number): number {
  return 0.5 * vci + 0.5 * tci;
}

/**
 * Calculate Crop Water Stress Index (CWSI)
 */
export function calculateCWSI(
  canopyTemp: number,
  airTemp: number,
  vpd: number
): number {
  // Simplified CWSI calculation
  // CWSI = (Tc - Ta) / (Tc_ul - Tc_ll)
  // where Tc = canopy temp, Ta = air temp
  const lowerBaseline = -2.0; // well-watered
  const upperBaseline = 5.0; // non-transpiring
  const tempDiff = canopyTemp - airTemp;

  const cwsi = (tempDiff - lowerBaseline) / (upperBaseline - lowerBaseline);
  return Math.max(0, Math.min(1, cwsi)); // Clamp to 0-1
}

/**
 * Calculate days of water supply remaining
 */
export function calculateDaysOfSupply(
  currentVolume: number,
  dailyDemand: number,
  inflow: number
): number {
  if (dailyDemand <= inflow) {
    return Infinity; // Sustainable
  }
  const netDailyLoss = dailyDemand - inflow;
  return Math.floor(currentVolume / netDailyLoss);
}

/**
 * Estimate crop yield loss from drought stress
 */
export function estimateYieldLoss(
  cwsi: number,
  growthStage: string,
  cropType: string
): number {
  // Simplified yield loss model
  // Sensitivity varies by growth stage and crop type
  const stageSensitivity: Record<string, number> = {
    vegetative: 0.3,
    flowering: 1.0,
    grain_filling: 0.8,
    maturity: 0.2,
  };

  const sensitivity = stageSensitivity[growthStage] || 0.5;
  const stressImpact = cwsi * sensitivity;

  // Yield loss as percentage
  return Math.min(100, stressImpact * 100);
}

/**
 * Validate drought severity level
 */
export function isValidDroughtSeverity(severity: string): severity is DroughtSeverity {
  return Object.values(DroughtSeverity).includes(severity as DroughtSeverity);
}

/**
 * Validate drought type
 */
export function isValidDroughtType(type: string): type is DroughtType {
  return Object.values(DroughtType).includes(type as DroughtType);
}

/**
 * Calculate composite drought index from multiple indicators
 */
export function calculateCompositeDroughtIndex(
  spi: number,
  pdsi: number,
  soilMoisturePercentile: number,
  vhi: number
): number {
  // Normalize all indices to 0-100 scale (100 = no drought, 0 = severe drought)
  const spiNorm = Math.max(0, Math.min(100, ((spi + 3) / 6) * 100));
  const pdsiNorm = Math.max(0, Math.min(100, ((pdsi + 10) / 20) * 100));
  const smNorm = soilMoisturePercentile;
  const vhiNorm = vhi;

  // Weighted average
  const weights = {
    spi: 0.3,
    pdsi: 0.3,
    soilMoisture: 0.25,
    vhi: 0.15,
  };

  return (
    spiNorm * weights.spi +
    pdsiNorm * weights.pdsi +
    smNorm * weights.soilMoisture +
    vhiNorm * weights.vhi
  );
}

// ============================================================================
// Default Export
// ============================================================================

export default DroughtMonitoringSDK;
