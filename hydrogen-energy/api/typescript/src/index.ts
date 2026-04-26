/**
 * WIA-ENE-007: Hydrogen Energy Standard - TypeScript SDK
 * Main SDK Implementation
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

import type {
  SDKConfig,
  ApiResponse,
  PaginatedResponse,
  Electrolyzer,
  ProductionMetrics,
  StorageVessel,
  StorageMetrics,
  RefuelingStation,
  HydrogenQuality,
  GreenCertification,
  SafetySensor,
  SafetyIncident,
  CostMetrics,
  PerformanceKPIs,
  TimeSeriesDataPoint,
  TimeSeriesQuery,
  FacilityConfig
} from './types';

export * from './types';

/**
 * Main SDK class for WIA-ENE-007 Hydrogen Energy Standard
 */
export class HydrogenEnergySDK {
  private config: Required<SDKConfig>;
  private baseUrl: string;

  constructor(config: SDKConfig) {
    this.config = {
      apiKey: config.apiKey,
      baseUrl: config.baseUrl || 'https://api.wia.org/v1/hydrogen-energy',
      timeout: config.timeout || 30000,
      retryAttempts: config.retryAttempts || 3,
      logLevel: config.logLevel || 'INFO'
    };
    this.baseUrl = this.config.baseUrl;
  }

  // =========================================================================
  // Production Management
  // =========================================================================

  /**
   * Get electrolyzer information
   */
  async getElectrolyzer(id: string): Promise<ApiResponse<Electrolyzer>> {
    return this.request<Electrolyzer>(`/electrolyzers/${id}`);
  }

  /**
   * List all electrolyzers
   */
  async listElectrolyzers(): Promise<ApiResponse<Electrolyzer[]>> {
    return this.request<Electrolyzer[]>('/electrolyzers');
  }

  /**
   * Get production metrics for a time period
   */
  async getProductionMetrics(
    query: TimeSeriesQuery
  ): Promise<ApiResponse<ProductionMetrics[]>> {
    return this.request<ProductionMetrics[]>('/production/metrics', {
      method: 'POST',
      body: JSON.stringify(query)
    });
  }

  /**
   * Start electrolyzer operation
   */
  async startElectrolyzer(
    id: string,
    targetLoad: number
  ): Promise<ApiResponse<{ success: boolean; message: string }>> {
    return this.request(`/electrolyzers/${id}/start`, {
      method: 'POST',
      body: JSON.stringify({ targetLoad })
    });
  }

  /**
   * Stop electrolyzer operation
   */
  async stopElectrolyzer(id: string): Promise<ApiResponse<{ success: boolean }>> {
    return this.request(`/electrolyzers/${id}/stop`, {
      method: 'POST'
    });
  }

  /**
   * Set electrolyzer load
   */
  async setElectrolyzerLoad(
    id: string,
    load: number
  ): Promise<ApiResponse<{ currentLoad: number }>> {
    return this.request(`/electrolyzers/${id}/load`, {
      method: 'PUT',
      body: JSON.stringify({ load })
    });
  }

  // =========================================================================
  // Storage Management
  // =========================================================================

  /**
   * Get storage vessel information
   */
  async getStorageVessel(id: string): Promise<ApiResponse<StorageVessel>> {
    return this.request<StorageVessel>(`/storage/vessels/${id}`);
  }

  /**
   * List all storage vessels
   */
  async listStorageVessels(): Promise<ApiResponse<StorageVessel[]>> {
    return this.request<StorageVessel[]>('/storage/vessels');
  }

  /**
   * Get aggregated storage metrics
   */
  async getStorageMetrics(): Promise<ApiResponse<StorageMetrics>> {
    return this.request<StorageMetrics>('/storage/metrics');
  }

  /**
   * Get storage inventory history
   */
  async getStorageHistory(
    query: TimeSeriesQuery
  ): Promise<ApiResponse<TimeSeriesDataPoint[]>> {
    return this.request<TimeSeriesDataPoint[]>('/storage/history', {
      method: 'POST',
      body: JSON.stringify(query)
    });
  }

  // =========================================================================
  // Distribution and Refueling
  // =========================================================================

  /**
   * Get refueling station information
   */
  async getRefuelingStation(id: string): Promise<ApiResponse<RefuelingStation>> {
    return this.request<RefuelingStation>(`/refueling-stations/${id}`);
  }

  /**
   * List all refueling stations
   */
  async listRefuelingStations(): Promise<ApiResponse<RefuelingStation[]>> {
    return this.request<RefuelingStation[]>('/refueling-stations');
  }

  /**
   * Find nearest refueling station
   */
  async findNearestStation(
    latitude: number,
    longitude: number,
    radius?: number
  ): Promise<ApiResponse<RefuelingStation[]>> {
    const params = new URLSearchParams({
      lat: latitude.toString(),
      lng: longitude.toString(),
      radius: (radius || 50).toString()
    });
    return this.request<RefuelingStation[]>(`/refueling-stations/nearby?${params}`);
  }

  // =========================================================================
  // Quality and Compliance
  // =========================================================================

  /**
   * Get hydrogen quality analysis
   */
  async getHydrogenQuality(
    sampleId?: string
  ): Promise<ApiResponse<HydrogenQuality>> {
    const endpoint = sampleId
      ? `/quality/samples/${sampleId}`
      : '/quality/latest';
    return this.request<HydrogenQuality>(endpoint);
  }

  /**
   * Get quality history
   */
  async getQualityHistory(
    query: TimeSeriesQuery
  ): Promise<ApiResponse<HydrogenQuality[]>> {
    return this.request<HydrogenQuality[]>('/quality/history', {
      method: 'POST',
      body: JSON.stringify(query)
    });
  }

  /**
   * Get green hydrogen certification
   */
  async getGreenCertification(): Promise<ApiResponse<GreenCertification>> {
    return this.request<GreenCertification>('/certification/green');
  }

  /**
   * Verify certification
   */
  async verifyCertification(
    certificateNumber: string
  ): Promise<ApiResponse<{ valid: boolean; details: GreenCertification }>> {
    return this.request(`/certification/verify/${certificateNumber}`);
  }

  // =========================================================================
  // Safety and Monitoring
  // =========================================================================

  /**
   * Get safety sensor readings
   */
  async getSafetySensors(): Promise<ApiResponse<SafetySensor[]>> {
    return this.request<SafetySensor[]>('/safety/sensors');
  }

  /**
   * Get specific sensor data
   */
  async getSensor(id: string): Promise<ApiResponse<SafetySensor>> {
    return this.request<SafetySensor>(`/safety/sensors/${id}`);
  }

  /**
   * Get safety incidents
   */
  async getSafetyIncidents(
    status?: string
  ): Promise<ApiResponse<SafetyIncident[]>> {
    const endpoint = status
      ? `/safety/incidents?status=${status}`
      : '/safety/incidents';
    return this.request<SafetyIncident[]>(endpoint);
  }

  /**
   * Report a safety incident
   */
  async reportIncident(
    incident: Omit<SafetyIncident, 'id' | 'status'>
  ): Promise<ApiResponse<SafetyIncident>> {
    return this.request<SafetyIncident>('/safety/incidents', {
      method: 'POST',
      body: JSON.stringify(incident)
    });
  }

  // =========================================================================
  // Performance and Economics
  // =========================================================================

  /**
   * Get cost metrics
   */
  async getCostMetrics(period: string): Promise<ApiResponse<CostMetrics>> {
    return this.request<CostMetrics>(`/economics/costs?period=${period}`);
  }

  /**
   * Get performance KPIs
   */
  async getPerformanceKPIs(period: string): Promise<ApiResponse<PerformanceKPIs>> {
    return this.request<PerformanceKPIs>(`/performance/kpis?period=${period}`);
  }

  /**
   * Calculate LCOH (Levelized Cost of Hydrogen)
   */
  async calculateLCOH(parameters: {
    capitalCost: number;
    operatingCost: number;
    annualProduction: number;
    projectLife: number;
    discountRate: number;
  }): Promise<ApiResponse<{ lcoh: number; breakdown: any }>> {
    return this.request('/economics/lcoh', {
      method: 'POST',
      body: JSON.stringify(parameters)
    });
  }

  /**
   * Get efficiency trends
   */
  async getEfficiencyTrends(
    query: TimeSeriesQuery
  ): Promise<ApiResponse<TimeSeriesDataPoint[]>> {
    return this.request<TimeSeriesDataPoint[]>('/performance/efficiency', {
      method: 'POST',
      body: JSON.stringify(query)
    });
  }

  // =========================================================================
  // Facility Management
  // =========================================================================

  /**
   * Get facility configuration
   */
  async getFacilityConfig(): Promise<ApiResponse<FacilityConfig>> {
    return this.request<FacilityConfig>('/facility/config');
  }

  /**
   * Update facility configuration
   */
  async updateFacilityConfig(
    config: Partial<FacilityConfig>
  ): Promise<ApiResponse<FacilityConfig>> {
    return this.request<FacilityConfig>('/facility/config', {
      method: 'PUT',
      body: JSON.stringify(config)
    });
  }

  /**
   * Get facility status dashboard
   */
  async getFacilityDashboard(): Promise<ApiResponse<{
    production: ProductionMetrics;
    storage: StorageMetrics;
    safety: { alarmCount: number; warningCount: number };
    performance: PerformanceKPIs;
  }>> {
    return this.request('/facility/dashboard');
  }

  // =========================================================================
  // Utility Methods
  // =========================================================================

  /**
   * Calculate hydrogen mass from volume
   */
  static volumeToMass(
    volumeNm3: number,
    temperature: number = 15,
    pressure: number = 1
  ): number {
    // Using ideal gas law: PV = nRT
    // H₂ density at NTP: 0.08988 kg/m³
    const densityNTP = 0.08988; // kg/Nm³
    return volumeNm3 * densityNTP;
  }

  /**
   * Calculate hydrogen volume from mass
   */
  static massToVolume(massKg: number): number {
    const densityNTP = 0.08988; // kg/Nm³
    return massKg / densityNTP;
  }

  /**
   * Calculate energy content (LHV basis)
   */
  static calculateEnergyLHV(massKg: number): number {
    const lhv = 33.3; // kWh/kg
    return massKg * lhv;
  }

  /**
   * Calculate energy content (HHV basis)
   */
  static calculateEnergyHHV(massKg: number): number {
    const hhv = 39.4; // kWh/kg
    return massKg * hhv;
  }

  /**
   * Validate ISO 14687 compliance
   */
  static validateISO14687(quality: HydrogenQuality): {
    compliant: boolean;
    violations: string[];
  } {
    const violations: string[] = [];

    if (quality.totalNonHydrogen > 2) {
      violations.push('Total non-H₂ gases exceeds 2%');
    }
    if (quality.waterContent > 5) {
      violations.push('Water content exceeds 5 μmol/mol');
    }
    if (quality.totalHydrocarbons > 2) {
      violations.push('Total hydrocarbons exceeds 2 μmol/mol');
    }
    if (quality.oxygen > 5) {
      violations.push('Oxygen exceeds 5 μmol/mol');
    }
    if (quality.carbonMonoxide > 0.2) {
      violations.push('CO exceeds 0.2 μmol/mol');
    }
    if (quality.carbonDioxide > 2) {
      violations.push('CO₂ exceeds 2 μmol/mol');
    }
    if (quality.totalSulfur > 0.004) {
      violations.push('Total sulfur exceeds 0.004 μmol/mol');
    }
    if (quality.ammonia > 0.1) {
      violations.push('Ammonia exceeds 0.1 μmol/mol');
    }

    return {
      compliant: violations.length === 0,
      violations
    };
  }

  // =========================================================================
  // Private Methods
  // =========================================================================

  private async request<T>(
    endpoint: string,
    options: RequestInit = {}
  ): Promise<ApiResponse<T>> {
    const url = `${this.baseUrl}${endpoint}`;
    const headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'User-Agent': 'WIA-ENE-007-SDK/1.0.0',
      ...options.headers
    };

    let lastError: Error | null = null;

    for (let attempt = 0; attempt < this.config.retryAttempts; attempt++) {
      try {
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

        const response = await fetch(url, {
          ...options,
          headers,
          signal: controller.signal
        });

        clearTimeout(timeoutId);

        if (!response.ok) {
          const errorData = await response.json().catch(() => ({}));
          return {
            success: false,
            error: {
              code: errorData.code || `HTTP_${response.status}`,
              message: errorData.message || response.statusText,
              details: errorData.details
            },
            timestamp: new Date()
          };
        }

        const data = await response.json();

        return {
          success: true,
          data: data as T,
          timestamp: new Date()
        };
      } catch (error) {
        lastError = error as Error;

        if (error instanceof Error && error.name === 'AbortError') {
          this.log('WARN', `Request timeout on attempt ${attempt + 1}`);
        } else {
          this.log('WARN', `Request failed on attempt ${attempt + 1}: ${error}`);
        }

        if (attempt < this.config.retryAttempts - 1) {
          await this.delay(1000 * Math.pow(2, attempt)); // Exponential backoff
        }
      }
    }

    return {
      success: false,
      error: {
        code: 'REQUEST_FAILED',
        message: lastError?.message || 'Unknown error',
        details: lastError
      },
      timestamp: new Date()
    };
  }

  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  private log(level: string, message: string): void {
    const levels = ['DEBUG', 'INFO', 'WARN', 'ERROR'];
    const configLevel = levels.indexOf(this.config.logLevel);
    const messageLevel = levels.indexOf(level);

    if (messageLevel >= configLevel) {
      console.log(`[WIA-ENE-007 SDK] [${level}] ${message}`);
    }
  }
}

/**
 * Create SDK instance
 */
export function createHydrogenEnergyClient(config: SDKConfig): HydrogenEnergySDK {
  return new HydrogenEnergySDK(config);
}

// Default export
export default HydrogenEnergySDK;
