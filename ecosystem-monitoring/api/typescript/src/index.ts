/**
 * WIA-ENE-031: Ecosystem Monitoring Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Export all types
export * from './types';

import {
  EcosystemBasicInfo,
  BiodiversityData,
  VegetationHealthData,
  SoilHealthData,
  WaterQualityData,
  CarbonSequestrationData,
  HydrologicalData,
  NitrogenCycleData,
  PhosphorusCycleData,
  SatelliteImageryData,
  WeatherSensorStation,
  SoilSensorArray,
  WaterQualitySensor,
  AutomatedCO2Chamber,
  RestorationProject,
  ProtectedArea,
  EcosystemHealthIndex,
  CarbonCreditCalculation,
  KPIDashboard,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
  EcosystemType,
  EcosystemCode,
  HealthGrade,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface EcosystemMonitoringSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class EcosystemMonitoringSDK {
  private config: Required<EcosystemMonitoringSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: EcosystemMonitoringSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'ENE-031',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Ecosystem Management APIs
  // ==========================================================================

  /**
   * Get ecosystem list
   */
  async listEcosystems(
    params?: PaginationParams & { type?: EcosystemType }
  ): Promise<ApiResponse<PaginatedResponse<EcosystemBasicInfo>>> {
    const queryParams = new URLSearchParams();
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<EcosystemBasicInfo>>(
      `/api/v1/ecosystems?${queryParams.toString()}`
    );
  }

  /**
   * Get ecosystem details
   */
  async getEcosystem(ecosystemId: string): Promise<ApiResponse<EcosystemBasicInfo>> {
    return this.get<EcosystemBasicInfo>(`/api/v1/ecosystems/${ecosystemId}`);
  }

  /**
   * Register new ecosystem
   */
  async createEcosystem(
    ecosystem: Omit<EcosystemBasicInfo, 'ecosystemId' | 'metadata'>
  ): Promise<ApiResponse<EcosystemBasicInfo>> {
    return this.post<EcosystemBasicInfo>('/api/v1/ecosystems', ecosystem);
  }

  /**
   * Update ecosystem information
   */
  async updateEcosystem(
    ecosystemId: string,
    updates: Partial<EcosystemBasicInfo>
  ): Promise<ApiResponse<EcosystemBasicInfo>> {
    return this.put<EcosystemBasicInfo>(`/api/v1/ecosystems/${ecosystemId}`, updates);
  }

  /**
   * Delete ecosystem
   */
  async deleteEcosystem(ecosystemId: string): Promise<ApiResponse<void>> {
    return this.delete<void>(`/api/v1/ecosystems/${ecosystemId}`);
  }

  // ==========================================================================
  // Ecosystem Health APIs
  // ==========================================================================

  /**
   * Get ecosystem health indicators
   */
  async getEcosystemHealth(
    ecosystemId: string
  ): Promise<ApiResponse<EcosystemHealthIndex>> {
    return this.get<EcosystemHealthIndex>(`/api/v1/ecosystems/${ecosystemId}/health`);
  }

  /**
   * Get biodiversity data
   */
  async getBiodiversity(
    ecosystemId: string,
    dateRange?: DateRangeFilter
  ): Promise<ApiResponse<BiodiversityData[]>> {
    const queryParams = dateRange
      ? new URLSearchParams({
          startDate: dateRange.startDate,
          endDate: dateRange.endDate,
        })
      : '';
    return this.get<BiodiversityData[]>(
      `/api/v1/ecosystems/${ecosystemId}/biodiversity${queryParams ? '?' + queryParams : ''}`
    );
  }

  /**
   * Get carbon storage and sequestration data
   */
  async getCarbonData(
    ecosystemId: string,
    dateRange?: DateRangeFilter
  ): Promise<ApiResponse<CarbonSequestrationData[]>> {
    const queryParams = dateRange
      ? new URLSearchParams({
          startDate: dateRange.startDate,
          endDate: dateRange.endDate,
        })
      : '';
    return this.get<CarbonSequestrationData[]>(
      `/api/v1/ecosystems/${ecosystemId}/carbon${queryParams ? '?' + queryParams : ''}`
    );
  }

  /**
   * Get water quality and hydrology data
   */
  async getWaterData(
    ecosystemId: string,
    dateRange?: DateRangeFilter
  ): Promise<ApiResponse<WaterQualityData[]>> {
    const queryParams = dateRange
      ? new URLSearchParams({
          startDate: dateRange.startDate,
          endDate: dateRange.endDate,
        })
      : '';
    return this.get<WaterQualityData[]>(
      `/api/v1/ecosystems/${ecosystemId}/water${queryParams ? '?' + queryParams : ''}`
    );
  }

  // ==========================================================================
  // Monitoring Data Submission APIs
  // ==========================================================================

  /**
   * Submit biodiversity monitoring data
   */
  async submitBiodiversityData(
    data: Omit<BiodiversityData, 'monitoringId'>
  ): Promise<ApiResponse<BiodiversityData>> {
    return this.post<BiodiversityData>('/api/v1/monitoring/biodiversity', data);
  }

  /**
   * Submit vegetation health data
   */
  async submitVegetationData(
    data: Omit<VegetationHealthData, 'monitoringId'>
  ): Promise<ApiResponse<VegetationHealthData>> {
    return this.post<VegetationHealthData>('/api/v1/monitoring/vegetation', data);
  }

  /**
   * Submit soil health data
   */
  async submitSoilData(
    data: Omit<SoilHealthData, 'monitoringId'>
  ): Promise<ApiResponse<SoilHealthData>> {
    return this.post<SoilHealthData>('/api/v1/monitoring/soil', data);
  }

  /**
   * Submit water quality data
   */
  async submitWaterQualityData(
    data: Omit<WaterQualityData, 'monitoringId'>
  ): Promise<ApiResponse<WaterQualityData>> {
    return this.post<WaterQualityData>('/api/v1/monitoring/water', data);
  }

  /**
   * Submit carbon sequestration data
   */
  async submitCarbonData(
    data: Omit<CarbonSequestrationData, 'monitoringId'>
  ): Promise<ApiResponse<CarbonSequestrationData>> {
    return this.post<CarbonSequestrationData>('/api/v1/monitoring/carbon', data);
  }

  // ==========================================================================
  // Satellite Imagery APIs
  // ==========================================================================

  /**
   * Register satellite imagery data
   */
  async registerSatelliteImagery(
    data: Omit<SatelliteImageryData, 'imageId'>
  ): Promise<ApiResponse<SatelliteImageryData>> {
    return this.post<SatelliteImageryData>('/api/v1/satellite/imagery', data);
  }

  /**
   * Get time series satellite data
   */
  async getSatelliteTimeSeries(
    ecosystemId: string,
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<SatelliteImageryData[]>> {
    const queryParams = new URLSearchParams({
      ecosystemId,
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
    });
    return this.get<SatelliteImageryData[]>(
      `/api/v1/satellite/timeseries?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // IoT Sensor APIs
  // ==========================================================================

  /**
   * Register IoT sensor
   */
  async registerSensor(
    sensor:
      | Omit<WeatherSensorStation, 'stationId'>
      | Omit<SoilSensorArray, 'arrayId'>
      | Omit<WaterQualitySensor, 'sensorId'>
      | Omit<AutomatedCO2Chamber, 'chamberId'>
  ): Promise<ApiResponse<any>> {
    return this.post<any>('/api/v1/iot/sensor', sensor);
  }

  /**
   * Submit IoT sensor data
   */
  async submitSensorData(
    sensorId: string,
    data: any
  ): Promise<ApiResponse<void>> {
    return this.post<void>('/api/v1/iot/data', {
      sensorId,
      ...data,
    });
  }

  /**
   * Get sensor data
   */
  async getSensorData(
    sensorId: string,
    dateRange?: DateRangeFilter
  ): Promise<ApiResponse<any[]>> {
    const queryParams = dateRange
      ? new URLSearchParams({
          startDate: dateRange.startDate,
          endDate: dateRange.endDate,
        })
      : '';
    return this.get<any[]>(
      `/api/v1/iot/sensors/${sensorId}/data${queryParams ? '?' + queryParams : ''}`
    );
  }

  // ==========================================================================
  // Restoration Project APIs
  // ==========================================================================

  /**
   * Create restoration project
   */
  async createRestorationProject(
    project: Omit<RestorationProject, 'projectId'>
  ): Promise<ApiResponse<RestorationProject>> {
    return this.post<RestorationProject>('/api/v1/restoration/projects', project);
  }

  /**
   * Get restoration project
   */
  async getRestorationProject(projectId: string): Promise<ApiResponse<RestorationProject>> {
    return this.get<RestorationProject>(`/api/v1/restoration/projects/${projectId}`);
  }

  /**
   * Update restoration project
   */
  async updateRestorationProject(
    projectId: string,
    updates: Partial<RestorationProject>
  ): Promise<ApiResponse<RestorationProject>> {
    return this.put<RestorationProject>(
      `/api/v1/restoration/projects/${projectId}`,
      updates
    );
  }

  /**
   * Submit restoration monitoring data
   */
  async submitRestorationMonitoring(
    projectId: string,
    monitoringData: any
  ): Promise<ApiResponse<void>> {
    return this.post<void>('/api/v1/restoration/monitoring', {
      projectId,
      ...monitoringData,
    });
  }

  // ==========================================================================
  // Protected Area APIs
  // ==========================================================================

  /**
   * List protected areas
   */
  async listProtectedAreas(
    params?: PaginationParams
  ): Promise<ApiResponse<PaginatedResponse<ProtectedArea>>> {
    const queryParams = new URLSearchParams();
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<ProtectedArea>>(
      `/api/v1/protected-areas?${queryParams.toString()}`
    );
  }

  /**
   * Get protected area details
   */
  async getProtectedArea(protectedAreaId: string): Promise<ApiResponse<ProtectedArea>> {
    return this.get<ProtectedArea>(`/api/v1/protected-areas/${protectedAreaId}`);
  }

  /**
   * Submit patrol record
   */
  async submitPatrolRecord(
    protectedAreaId: string,
    patrolData: any
  ): Promise<ApiResponse<void>> {
    return this.post<void>('/api/v1/protected-areas/patrols', {
      protectedAreaId,
      ...patrolData,
    });
  }

  // ==========================================================================
  // Analytics APIs
  // ==========================================================================

  /**
   * Get KPI dashboard
   */
  async getKPIDashboard(
    ecosystemId: string,
    date?: string
  ): Promise<ApiResponse<KPIDashboard>> {
    const queryParams = date ? new URLSearchParams({ date }) : '';
    return this.get<KPIDashboard>(
      `/api/v1/analytics/kpi/${ecosystemId}${queryParams ? '?' + queryParams : ''}`
    );
  }

  /**
   * Get trend analysis
   */
  async getTrendAnalysis(
    ecosystemId: string,
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<any>> {
    const queryParams = new URLSearchParams({
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
    });
    return this.get<any>(
      `/api/v1/analytics/trends/${ecosystemId}?${queryParams.toString()}`
    );
  }

  /**
   * Generate report
   */
  async generateReport(
    ecosystemId: string,
    reportType: 'monthly' | 'quarterly' | 'annual',
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<any>> {
    return this.post<any>('/api/v1/analytics/report', {
      ecosystemId,
      reportType,
      period: dateRange,
    });
  }

  // ==========================================================================
  // Carbon Credit APIs
  // ==========================================================================

  /**
   * Calculate carbon credits
   */
  async calculateCarbonCredits(
    projectId: string,
    calculation: Omit<CarbonCreditCalculation, 'projectId'>
  ): Promise<ApiResponse<CarbonCreditCalculation>> {
    return this.post<CarbonCreditCalculation>('/api/v1/carbon-credits/calculate', {
      projectId,
      ...calculation,
    });
  }

  /**
   * Get carbon credit verification
   */
  async getCarbonCreditVerification(
    projectId: string
  ): Promise<ApiResponse<CarbonCreditCalculation>> {
    return this.get<CarbonCreditCalculation>(
      `/api/v1/carbon-credits/verification/${projectId}`
    );
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
      console.log(`[WIA-ENE-031] ${method} ${url}`, body || '');
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
        console.error('[WIA-ENE-031] Request failed:', error);
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
 * Calculate biodiversity index
 */
export function calculateBiodiversityIndex(
  shannonIndex: number,
  speciesRichness: number,
  endemicSpecies: number,
  baselineRichness: number = 50
): number {
  // Shannon 지수 정규화 (0-5 → 0-100)
  const shannonScore = Math.min((shannonIndex / 5) * 100, 100);

  // 종 풍부도 점수
  const richnessScore = Math.min((speciesRichness / baselineRichness) * 100, 100);

  // 고유종 점수 (고유종 수에 따라)
  const endemicScore = Math.min(endemicSpecies * 10, 100);

  // 가중 평균
  return shannonScore * 0.4 + richnessScore * 0.3 + endemicScore * 0.3;
}

/**
 * Calculate vegetation health index
 */
export function calculateVegetationHealthIndex(
  ndviMean: number,
  biomass: number,
  npp: number,
  stressScore: number,
  baselineBiomass: number = 100,
  baselineNPP: number = 10
): number {
  // NDVI 점수 (0.8 이상 = 100점, 0.5 이하 = 0점)
  const ndviScore = ndviMean >= 0.8 ? 100 : ndviMean <= 0.5 ? 0 : ((ndviMean - 0.5) / 0.3) * 100;

  // 바이오매스 점수
  const biomassScore = Math.min((biomass / baselineBiomass) * 100, 100);

  // 생산성 점수
  const nppScore = Math.min((npp / baselineNPP) * 100, 100);

  // 스트레스 점수 역산 (낮을수록 좋음)
  const healthStressScore = 100 - stressScore;

  // 가중 평균
  return ndviScore * 0.35 + biomassScore * 0.25 + nppScore * 0.25 + healthStressScore * 0.15;
}

/**
 * Estimate carbon stock from DBH measurements
 */
export function estimateCarbonStockFromDBH(
  dbhArray: number[],
  speciesCoefficients: { a: number; b: number } = { a: 0.0509, b: 2.4257 },
  carbonContent: number = 0.48,
  plotArea: number = 400
): number {
  // 상대생장식: AGB = a × DBH^b
  const totalBiomass = dbhArray.reduce((sum, dbh) => {
    const agb = speciesCoefficients.a * Math.pow(dbh, speciesCoefficients.b);
    return sum + agb;
  }, 0);

  // 탄소량 (C) = AGB × 탄소 함량
  const carbonStock = totalBiomass * carbonContent;

  // 헥타르당 탄소 저장량 (Mg C/ha)
  const carbonStockPerHa = (carbonStock / plotArea) * 10000;

  return carbonStockPerHa;
}

/**
 * Validate ecosystem code
 */
export function isValidEcosystemCode(code: string): code is EcosystemCode {
  return Object.values(EcosystemCode).includes(code as EcosystemCode);
}

/**
 * Validate ecosystem type
 */
export function isValidEcosystemType(type: string): type is EcosystemType {
  return Object.values(EcosystemType).includes(type as EcosystemType);
}

/**
 * Calculate NDVI from red and NIR bands
 */
export function calculateNDVI(red: number, nir: number): number {
  if (red + nir === 0) return 0;
  return (nir - red) / (nir + red);
}

/**
 * Calculate EVI (Enhanced Vegetation Index)
 */
export function calculateEVI(red: number, nir: number, blue: number): number {
  const G = 2.5;
  const C1 = 6;
  const C2 = 7.5;
  const L = 1;

  const numerator = nir - red;
  const denominator = nir + C1 * red - C2 * blue + L;

  if (denominator === 0) return 0;
  return G * (numerator / denominator);
}

// ============================================================================
// Default Export
// ============================================================================

export default EcosystemMonitoringSDK;
