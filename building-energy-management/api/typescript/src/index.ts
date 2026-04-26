/**
 * WIA-CITY-011: Building Energy Management Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Export all types
export * from './types';

import {
  BuildingInfo,
  EnergyMeterReading,
  HVACOperationalData,
  SolarPVData,
  ESSData,
  GeothermalData,
  CarbonFootprintReport,
  BenchmarkingComparison,
  BenchmarkingRecommendation,
  CertificationStatus,
  RealtimeDashboard,
  AlertInfo,
  EnergyForecast,
  PeakShavingEvent,
  DemandResponseEvent,
  HVACSetpointCommand,
  LightingControlCommand,
  ESSControlCommand,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
  AggregationType,
  MeterType,
  BuildingType,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface BuildingEnergySDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class BuildingEnergySDK {
  private config: Required<BuildingEnergySDKConfig>;
  private headers: Record<string, string>;

  constructor(config: BuildingEnergySDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'CITY-011',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Building Information APIs
  // ==========================================================================

  /**
   * Get building information
   */
  async getBuildingInfo(buildingId: string): Promise<ApiResponse<BuildingInfo>> {
    return this.get<BuildingInfo>(`/api/v1/buildings/${buildingId}`);
  }

  /**
   * List all buildings
   */
  async listBuildings(
    params?: PaginationParams
  ): Promise<ApiResponse<PaginatedResponse<BuildingInfo>>> {
    const queryParams = this.buildQueryParams(params);
    return this.get<PaginatedResponse<BuildingInfo>>(
      `/api/v1/buildings${queryParams}`
    );
  }

  /**
   * Update building information
   */
  async updateBuildingInfo(
    buildingId: string,
    updates: Partial<BuildingInfo>
  ): Promise<ApiResponse<BuildingInfo>> {
    return this.put<BuildingInfo>(`/api/v1/buildings/${buildingId}`, updates);
  }

  // ==========================================================================
  // Energy Metering APIs
  // ==========================================================================

  /**
   * Get real-time energy meter reading
   */
  async getMeterReading(meterId: string): Promise<ApiResponse<EnergyMeterReading>> {
    return this.get<EnergyMeterReading>(`/api/v1/meters/${meterId}/latest`);
  }

  /**
   * Get historical meter readings
   */
  async getMeterHistory(
    meterId: string,
    dateRange: DateRangeFilter,
    aggregation: AggregationType = '15min'
  ): Promise<ApiResponse<EnergyMeterReading[]>> {
    const queryParams = new URLSearchParams({
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
      aggregation,
    });
    return this.get<EnergyMeterReading[]>(
      `/api/v1/meters/${meterId}/history?${queryParams.toString()}`
    );
  }

  /**
   * Get energy consumption by type
   */
  async getEnergyConsumption(
    buildingId: string,
    meterType: MeterType,
    dateRange: DateRangeFilter,
    aggregation: AggregationType = 'daily'
  ): Promise<ApiResponse<any>> {
    const queryParams = new URLSearchParams({
      buildingId,
      meterType,
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
      aggregation,
    });
    return this.get<any>(`/api/v1/energy/consumption?${queryParams.toString()}`);
  }

  /**
   * Get energy breakdown by end-use
   */
  async getEnergyBreakdown(
    buildingId: string,
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<any>> {
    const queryParams = new URLSearchParams({
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
    });
    return this.get<any>(
      `/api/v1/energy/${buildingId}/breakdown?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // HVAC Monitoring APIs
  // ==========================================================================

  /**
   * Get HVAC system status
   */
  async getHVACStatus(systemId: string): Promise<ApiResponse<HVACOperationalData>> {
    return this.get<HVACOperationalData>(`/api/v1/hvac/${systemId}/status`);
  }

  /**
   * List all HVAC systems in building
   */
  async listHVACSystems(buildingId: string): Promise<ApiResponse<HVACOperationalData[]>> {
    return this.get<HVACOperationalData[]>(`/api/v1/hvac?buildingId=${buildingId}`);
  }

  /**
   * Get HVAC historical data
   */
  async getHVACHistory(
    systemId: string,
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<HVACOperationalData[]>> {
    const queryParams = new URLSearchParams({
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
    });
    return this.get<HVACOperationalData[]>(
      `/api/v1/hvac/${systemId}/history?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // HVAC Control APIs
  // ==========================================================================

  /**
   * Set HVAC temperature setpoint
   */
  async setHVACSetpoint(
    command: HVACSetpointCommand
  ): Promise<ApiResponse<void>> {
    return this.post<void>('/api/v1/hvac/setpoint', command);
  }

  /**
   * Control lighting
   */
  async controlLighting(
    command: LightingControlCommand
  ): Promise<ApiResponse<void>> {
    return this.post<void>('/api/v1/lighting/control', command);
  }

  // ==========================================================================
  // Renewable Energy APIs
  // ==========================================================================

  /**
   * Get solar PV real-time data
   */
  async getSolarPVData(pvSystemId: string): Promise<ApiResponse<SolarPVData>> {
    return this.get<SolarPVData>(`/api/v1/renewable/solar/${pvSystemId}`);
  }

  /**
   * Get ESS real-time data
   */
  async getESSData(essId: string): Promise<ApiResponse<ESSData>> {
    return this.get<ESSData>(`/api/v1/renewable/ess/${essId}`);
  }

  /**
   * Control ESS operation
   */
  async controlESS(command: ESSControlCommand): Promise<ApiResponse<void>> {
    return this.post<void>('/api/v1/renewable/ess/control', command);
  }

  /**
   * Get geothermal system data
   */
  async getGeothermalData(ghpId: string): Promise<ApiResponse<GeothermalData>> {
    return this.get<GeothermalData>(`/api/v1/renewable/geothermal/${ghpId}`);
  }

  /**
   * List all renewable energy systems
   */
  async listRenewableSystems(
    buildingId: string
  ): Promise<ApiResponse<any>> {
    return this.get<any>(`/api/v1/renewable?buildingId=${buildingId}`);
  }

  // ==========================================================================
  // Load Management APIs
  // ==========================================================================

  /**
   * Get peak shaving configuration
   */
  async getPeakShavingConfig(
    buildingId: string
  ): Promise<ApiResponse<any>> {
    return this.get<any>(`/api/v1/load-management/${buildingId}/peak-shaving`);
  }

  /**
   * Update peak shaving configuration
   */
  async updatePeakShavingConfig(
    buildingId: string,
    config: any
  ): Promise<ApiResponse<any>> {
    return this.put<any>(
      `/api/v1/load-management/${buildingId}/peak-shaving`,
      config
    );
  }

  /**
   * Get peak shaving events
   */
  async getPeakShavingEvents(
    buildingId: string,
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<PeakShavingEvent[]>> {
    const queryParams = new URLSearchParams({
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
    });
    return this.get<PeakShavingEvent[]>(
      `/api/v1/load-management/${buildingId}/peak-shaving/events?${queryParams.toString()}`
    );
  }

  /**
   * Trigger peak shaving manually
   */
  async triggerPeakShaving(
    buildingId: string,
    targetReduction: number
  ): Promise<ApiResponse<PeakShavingEvent>> {
    return this.post<PeakShavingEvent>(
      `/api/v1/load-management/${buildingId}/peak-shaving/trigger`,
      { targetReduction }
    );
  }

  // ==========================================================================
  // Demand Response APIs
  // ==========================================================================

  /**
   * Get demand response configuration
   */
  async getDRConfig(buildingId: string): Promise<ApiResponse<any>> {
    return this.get<any>(`/api/v1/demand-response/${buildingId}/config`);
  }

  /**
   * Update demand response configuration
   */
  async updateDRConfig(
    buildingId: string,
    config: any
  ): Promise<ApiResponse<any>> {
    return this.put<any>(
      `/api/v1/demand-response/${buildingId}/config`,
      config
    );
  }

  /**
   * Get active DR events
   */
  async getActiveDREvents(
    buildingId: string
  ): Promise<ApiResponse<DemandResponseEvent[]>> {
    return this.get<DemandResponseEvent[]>(
      `/api/v1/demand-response/${buildingId}/events/active`
    );
  }

  /**
   * Get DR event history
   */
  async getDREventHistory(
    buildingId: string,
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<DemandResponseEvent[]>> {
    const queryParams = new URLSearchParams({
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
    });
    return this.get<DemandResponseEvent[]>(
      `/api/v1/demand-response/${buildingId}/events/history?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Carbon Footprint APIs
  // ==========================================================================

  /**
   * Get carbon emissions report
   */
  async getCarbonReport(
    buildingId: string,
    period: 'daily' | 'weekly' | 'monthly' | 'annual',
    date: string
  ): Promise<ApiResponse<CarbonFootprintReport>> {
    return this.get<CarbonFootprintReport>(
      `/api/v1/carbon/${buildingId}/report?period=${period}&date=${date}`
    );
  }

  /**
   * Get real-time carbon emissions
   */
  async getRealtimeCarbon(
    buildingId: string
  ): Promise<ApiResponse<any>> {
    return this.get<any>(`/api/v1/carbon/${buildingId}/realtime`);
  }

  /**
   * Get carbon intensity trends
   */
  async getCarbonTrends(
    buildingId: string,
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<any>> {
    const queryParams = new URLSearchParams({
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
    });
    return this.get<any>(
      `/api/v1/carbon/${buildingId}/trends?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Benchmarking APIs
  // ==========================================================================

  /**
   * Compare building energy performance
   */
  async compareBenchmark(
    buildingId: string,
    period: 'monthly' | 'annual' = 'annual'
  ): Promise<ApiResponse<BenchmarkingComparison>> {
    return this.get<BenchmarkingComparison>(
      `/api/v1/benchmarking/${buildingId}/compare?period=${period}`
    );
  }

  /**
   * Get benchmarking recommendations
   */
  async getBenchmarkingRecommendations(
    buildingId: string
  ): Promise<ApiResponse<BenchmarkingRecommendation[]>> {
    return this.get<BenchmarkingRecommendation[]>(
      `/api/v1/benchmarking/${buildingId}/recommendations`
    );
  }

  /**
   * Get peer buildings for comparison
   */
  async getPeerBuildings(
    buildingType: BuildingType,
    params?: {
      location?: string;
      floorArea?: { min: number; max: number };
      limit?: number;
    }
  ): Promise<ApiResponse<BuildingInfo[]>> {
    const queryParams = new URLSearchParams({ buildingType });
    if (params?.location) queryParams.append('location', params.location);
    if (params?.floorArea) {
      queryParams.append('floorAreaMin', params.floorArea.min.toString());
      queryParams.append('floorAreaMax', params.floorArea.max.toString());
    }
    if (params?.limit) queryParams.append('limit', params.limit.toString());

    return this.get<BuildingInfo[]>(
      `/api/v1/benchmarking/peers?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Certification APIs
  // ==========================================================================

  /**
   * Get certification status
   */
  async getCertificationStatus(
    buildingId: string
  ): Promise<ApiResponse<CertificationStatus[]>> {
    return this.get<CertificationStatus[]>(
      `/api/v1/certification/${buildingId}/status`
    );
  }

  /**
   * Get LEED energy performance data
   */
  async getLEEDData(buildingId: string): Promise<ApiResponse<any>> {
    return this.get<any>(`/api/v1/certification/${buildingId}/leed`);
  }

  /**
   * Get BREEAM energy data
   */
  async getBREEAMData(buildingId: string): Promise<ApiResponse<any>> {
    return this.get<any>(`/api/v1/certification/${buildingId}/breeam`);
  }

  /**
   * Export certification report
   */
  async exportCertificationReport(
    buildingId: string,
    certificationType: string,
    format: 'pdf' | 'excel' = 'pdf'
  ): Promise<ApiResponse<Blob>> {
    return this.get<Blob>(
      `/api/v1/certification/${buildingId}/export?type=${certificationType}&format=${format}`
    );
  }

  // ==========================================================================
  // Monitoring & Dashboard APIs
  // ==========================================================================

  /**
   * Get real-time dashboard data
   */
  async getRealtimeDashboard(
    buildingId: string
  ): Promise<ApiResponse<RealtimeDashboard>> {
    return this.get<RealtimeDashboard>(`/api/v1/dashboard/${buildingId}/realtime`);
  }

  /**
   * Get active alerts
   */
  async getActiveAlerts(buildingId: string): Promise<ApiResponse<AlertInfo[]>> {
    return this.get<AlertInfo[]>(`/api/v1/alerts/${buildingId}/active`);
  }

  /**
   * Acknowledge alert
   */
  async acknowledgeAlert(alertId: string): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/alerts/${alertId}/acknowledge`, {});
  }

  /**
   * Get alert history
   */
  async getAlertHistory(
    buildingId: string,
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<AlertInfo[]>> {
    const queryParams = new URLSearchParams({
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
    });
    return this.get<AlertInfo[]>(
      `/api/v1/alerts/${buildingId}/history?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Analytics & Forecasting APIs
  // ==========================================================================

  /**
   * Get energy forecast
   */
  async getEnergyForecast(
    buildingId: string,
    horizon: '1day' | '7day' | '30day' = '1day'
  ): Promise<ApiResponse<EnergyForecast>> {
    return this.get<EnergyForecast>(
      `/api/v1/analytics/${buildingId}/forecast?horizon=${horizon}`
    );
  }

  /**
   * Get energy efficiency trends
   */
  async getEfficiencyTrends(
    buildingId: string,
    dateRange: DateRangeFilter,
    metric: 'eui' | 'load-factor' | 'peak-demand'
  ): Promise<ApiResponse<any>> {
    const queryParams = new URLSearchParams({
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
      metric,
    });
    return this.get<any>(
      `/api/v1/analytics/${buildingId}/trends?${queryParams.toString()}`
    );
  }

  /**
   * Generate custom report
   */
  async generateReport(
    buildingId: string,
    reportType: 'energy' | 'carbon' | 'cost' | 'comprehensive',
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<any>> {
    return this.post<any>(`/api/v1/reports/${buildingId}/generate`, {
      reportType,
      dateRange,
    });
  }

  // ==========================================================================
  // HTTP Helper Methods
  // ==========================================================================

  private buildQueryParams(params?: any): string {
    if (!params) return '';
    const queryParams = new URLSearchParams();
    Object.entries(params).forEach(([key, value]) => {
      if (value !== undefined && value !== null) {
        queryParams.append(key, String(value));
      }
    });
    const qs = queryParams.toString();
    return qs ? `?${qs}` : '';
  }

  private async request<T>(
    method: string,
    path: string,
    body?: any
  ): Promise<ApiResponse<T>> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA-CITY-011] ${method} ${url}`, body || '');
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
        console.error('[WIA-CITY-011] Request failed:', error);
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
 * Calculate Energy Use Intensity (EUI)
 */
export function calculateEUI(
  annualEnergyConsumption: number,  // kWh
  floorArea: number                 // m²
): number {
  return annualEnergyConsumption / floorArea;
}

/**
 * Calculate load factor
 */
export function calculateLoadFactor(
  averageLoad: number,              // kW
  peakLoad: number                  // kW
): number {
  return averageLoad / peakLoad;
}

/**
 * Calculate COP (Coefficient of Performance)
 */
export function calculateCOP(
  thermalOutput: number,            // kW
  electricalInput: number           // kW
): number {
  return thermalOutput / electricalInput;
}

/**
 * Calculate carbon emissions
 */
export function calculateCarbonEmissions(
  energyConsumption: number,        // kWh or m³
  emissionFactor: number            // kg CO₂/kWh or kg CO₂/m³
): number {
  return energyConsumption * emissionFactor;
}

/**
 * Calculate ESS round-trip efficiency
 */
export function calculateESSEfficiency(
  energyDischarged: number,         // kWh
  energyCharged: number             // kWh
): number {
  return (energyDischarged / energyCharged) * 100;
}

/**
 * Calculate solar PV performance ratio
 */
export function calculatePVPerformanceRatio(
  actualGeneration: number,         // kWh
  theoreticalGeneration: number     // kWh
): number {
  return (actualGeneration / theoreticalGeneration) * 100;
}

/**
 * Estimate peak shaving potential
 */
export function estimatePeakShavingPotential(
  peakDemand: number,               // kW
  targetReduction: number,          // %
  essCapacity: number,              // kW
  sheddableLoad: number             // kW
): {
  targetPeak: number;
  essContribution: number;
  loadSheddingRequired: number;
  achievable: boolean;
} {
  const targetPeak = peakDemand * (1 - targetReduction / 100);
  const reductionNeeded = peakDemand - targetPeak;
  const essContribution = Math.min(essCapacity, reductionNeeded);
  const loadSheddingRequired = Math.max(0, reductionNeeded - essContribution);
  const achievable = loadSheddingRequired <= sheddableLoad;

  return {
    targetPeak,
    essContribution,
    loadSheddingRequired,
    achievable,
  };
}

/**
 * Calculate cost savings from energy reduction
 */
export function calculateCostSavings(
  energySavings: number,            // kWh
  energyRate: number,               // currency/kWh
  demandSavings?: number,           // kW
  demandCharge?: number             // currency/kW
): number {
  let savings = energySavings * energyRate;
  if (demandSavings && demandCharge) {
    savings += demandSavings * demandCharge;
  }
  return savings;
}

/**
 * Validate building type
 */
export function isValidBuildingType(type: string): type is BuildingType {
  return Object.values(BuildingType).includes(type as BuildingType);
}

// ============================================================================
// Default Export
// ============================================================================

export default BuildingEnergySDK;
