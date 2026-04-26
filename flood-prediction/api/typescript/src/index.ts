/**
 * WIA-ENE-033: Flood Prediction Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Export all types
export * from './types';

import {
  FloodPredictionRequest,
  FloodPredictionResult,
  FloodAlert,
  EvacuationShelter,
  EvacuationRoute,
  RiverLevelData,
  DamStatusData,
  DrainageData,
  MonitoringDashboard,
  FloodResponseReport,
  KPIDashboard,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
  Coordinates,
  RiskLevel,
  TimeHorizon,
  SpatialResolution,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface FloodPredictionSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class FloodPredictionSDK {
  private config: Required<FloodPredictionSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: FloodPredictionSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'ENE-033',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Flood Prediction APIs
  // ==========================================================================

  /**
   * Predict flood risk for a location
   */
  async predictFloodRisk(
    request: FloodPredictionRequest
  ): Promise<ApiResponse<FloodPredictionResult>> {
    return this.post<FloodPredictionResult>('/api/v1/flood/predict', request);
  }

  /**
   * Get flood prediction by ID
   */
  async getPrediction(predictionId: string): Promise<ApiResponse<FloodPredictionResult>> {
    return this.get<FloodPredictionResult>(`/api/v1/flood/prediction/${predictionId}`);
  }

  /**
   * Run flood inundation simulation
   */
  async runSimulation(
    request: FloodPredictionRequest & { scenarios: string[] }
  ): Promise<ApiResponse<FloodPredictionResult>> {
    return this.post<FloodPredictionResult>('/api/v1/flood/simulation', request);
  }

  /**
   * Get flood risk map for a region
   */
  async getRiskMap(regionId: string): Promise<ApiResponse<any>> {
    return this.get<any>(`/api/v1/flood/risk-map/${regionId}`);
  }

  /**
   * List flood predictions with pagination
   */
  async listPredictions(
    params?: PaginationParams & { regionId?: string; riskLevel?: RiskLevel }
  ): Promise<ApiResponse<PaginatedResponse<FloodPredictionResult>>> {
    const queryParams = new URLSearchParams();
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<FloodPredictionResult>>(
      `/api/v1/flood/predictions?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Real-time Monitoring APIs
  // ==========================================================================

  /**
   * Get river level monitoring data
   */
  async monitorRivers(regionId?: string): Promise<ApiResponse<RiverLevelData[]>> {
    const queryParams = regionId ? new URLSearchParams({ regionId }) : '';
    return this.get<RiverLevelData[]>(
      `/api/v1/monitoring/rivers${queryParams ? '?' + queryParams : ''}`
    );
  }

  /**
   * Get specific river status
   */
  async getRiverStatus(riverId: string): Promise<ApiResponse<RiverLevelData>> {
    return this.get<RiverLevelData>(`/api/v1/monitoring/rivers/${riverId}`);
  }

  /**
   * Get dam/reservoir status
   */
  async monitorDams(regionId?: string): Promise<ApiResponse<DamStatusData[]>> {
    const queryParams = regionId ? new URLSearchParams({ regionId }) : '';
    return this.get<DamStatusData[]>(
      `/api/v1/monitoring/dams${queryParams ? '?' + queryParams : ''}`
    );
  }

  /**
   * Get specific dam status
   */
  async getDamStatus(damId: string): Promise<ApiResponse<DamStatusData>> {
    return this.get<DamStatusData>(`/api/v1/monitoring/dams/${damId}`);
  }

  /**
   * Get rainfall monitoring data
   */
  async monitorRainfall(regionId?: string): Promise<ApiResponse<any>> {
    const queryParams = regionId ? new URLSearchParams({ regionId }) : '';
    return this.get<any>(
      `/api/v1/monitoring/rainfall${queryParams ? '?' + queryParams : ''}`
    );
  }

  /**
   * Get drainage facility status
   */
  async monitorDrainage(regionId?: string): Promise<ApiResponse<DrainageData[]>> {
    const queryParams = regionId ? new URLSearchParams({ regionId }) : '';
    return this.get<DrainageData[]>(
      `/api/v1/monitoring/drainage${queryParams ? '?' + queryParams : ''}`
    );
  }

  /**
   * Get monitoring dashboard
   */
  async getMonitoringDashboard(regionId?: string): Promise<ApiResponse<MonitoringDashboard>> {
    const queryParams = regionId ? new URLSearchParams({ regionId }) : '';
    return this.get<MonitoringDashboard>(
      `/api/v1/monitoring/dashboard${queryParams ? '?' + queryParams : ''}`
    );
  }

  // ==========================================================================
  // Alert Management APIs
  // ==========================================================================

  /**
   * Create flood alert
   */
  async createAlert(
    alert: Omit<FloodAlert, 'alertId' | 'timestamp'>
  ): Promise<ApiResponse<FloodAlert>> {
    return this.post<FloodAlert>('/api/v1/alert/create', alert);
  }

  /**
   * Get active alerts
   */
  async getActiveAlerts(regionId?: string): Promise<ApiResponse<FloodAlert[]>> {
    const queryParams = regionId ? new URLSearchParams({ regionId }) : '';
    return this.get<FloodAlert[]>(
      `/api/v1/alert/active${queryParams ? '?' + queryParams : ''}`
    );
  }

  /**
   * Get alert by ID
   */
  async getAlert(alertId: string): Promise<ApiResponse<FloodAlert>> {
    return this.get<FloodAlert>(`/api/v1/alert/${alertId}`);
  }

  /**
   * Update alert level
   */
  async updateAlert(
    alertId: string,
    updates: Partial<FloodAlert>
  ): Promise<ApiResponse<FloodAlert>> {
    return this.put<FloodAlert>(`/api/v1/alert/${alertId}/update`, updates);
  }

  /**
   * Cancel alert
   */
  async cancelAlert(alertId: string, reason?: string): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/alert/${alertId}/cancel`, { reason });
  }

  // ==========================================================================
  // Evacuation APIs
  // ==========================================================================

  /**
   * Get evacuation routes
   */
  async getEvacuationRoutes(
    origin: Coordinates,
    destinationType?: 'shelter' | 'high-ground'
  ): Promise<ApiResponse<EvacuationRoute[]>> {
    const queryParams = new URLSearchParams({
      latitude: origin.latitude.toString(),
      longitude: origin.longitude.toString(),
    });
    if (destinationType) {
      queryParams.append('type', destinationType);
    }
    return this.get<EvacuationRoute[]>(
      `/api/v1/evacuation/routes?${queryParams.toString()}`
    );
  }

  /**
   * Get specific evacuation route
   */
  async getEvacuationRoute(routeId: string): Promise<ApiResponse<EvacuationRoute>> {
    return this.get<EvacuationRoute>(`/api/v1/evacuation/routes/${routeId}`);
  }

  /**
   * Get evacuation shelters
   */
  async getEvacuationShelters(regionId?: string): Promise<ApiResponse<EvacuationShelter[]>> {
    const queryParams = regionId ? new URLSearchParams({ regionId }) : '';
    return this.get<EvacuationShelter[]>(
      `/api/v1/evacuation/shelters${queryParams ? '?' + queryParams : ''}`
    );
  }

  /**
   * Get shelter capacity status
   */
  async getShelterCapacity(shelterId: string): Promise<ApiResponse<EvacuationShelter>> {
    return this.get<EvacuationShelter>(`/api/v1/evacuation/shelters/${shelterId}`);
  }

  /**
   * Update shelter status
   */
  async updateShelterStatus(
    shelterId: string,
    updates: Partial<EvacuationShelter>
  ): Promise<ApiResponse<EvacuationShelter>> {
    return this.put<EvacuationShelter>(
      `/api/v1/evacuation/shelters/${shelterId}`,
      updates
    );
  }

  // ==========================================================================
  // Analytics & Reporting APIs
  // ==========================================================================

  /**
   * Get KPI dashboard
   */
  async getKPIDashboard(
    regionId?: string,
    dateRange?: DateRangeFilter
  ): Promise<ApiResponse<KPIDashboard>> {
    const queryParams = new URLSearchParams();
    if (regionId) queryParams.append('regionId', regionId);
    if (dateRange) {
      queryParams.append('startDate', dateRange.startDate);
      queryParams.append('endDate', dateRange.endDate);
    }
    return this.get<KPIDashboard>(
      `/api/v1/analytics/kpi${queryParams.toString() ? '?' + queryParams.toString() : ''}`
    );
  }

  /**
   * Generate performance report
   */
  async generateReport(
    regionId: string,
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<FloodResponseReport>> {
    return this.post<FloodResponseReport>('/api/v1/analytics/report', {
      regionId,
      period: dateRange,
    });
  }

  /**
   * Get report by ID
   */
  async getReport(reportId: string): Promise<ApiResponse<FloodResponseReport>> {
    return this.get<FloodResponseReport>(`/api/v1/analytics/report/${reportId}`);
  }

  /**
   * List historical events
   */
  async listHistoricalEvents(
    params?: PaginationParams & DateRangeFilter & { regionId?: string }
  ): Promise<ApiResponse<PaginatedResponse<FloodPredictionResult>>> {
    const queryParams = new URLSearchParams();
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<FloodPredictionResult>>(
      `/api/v1/analytics/events?${queryParams.toString()}`
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
      console.log(`[WIA-ENE-033] ${method} ${url}`, body || '');
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
        console.error('[WIA-ENE-033] Request failed:', error);
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
 * Calculate flood risk score (0-100)
 */
export function calculateRiskScore(
  precipitationScore: number,
  riverLevelScore: number,
  damScore: number,
  drainageScore: number
): number {
  return (
    precipitationScore * 0.3 +
    riverLevelScore * 0.4 +
    damScore * 0.2 +
    drainageScore * 0.1
  );
}

/**
 * Determine risk level from score
 */
export function getRiskLevelFromScore(score: number): RiskLevel {
  if (score >= 80) return RiskLevel.LEVEL_4;
  if (score >= 60) return RiskLevel.LEVEL_3;
  if (score >= 40) return RiskLevel.LEVEL_2;
  if (score >= 20) return RiskLevel.LEVEL_1;
  return RiskLevel.LEVEL_0;
}

/**
 * Check if evacuation is required
 */
export function isEvacuationRequired(
  riskLevel: RiskLevel,
  probability: number
): boolean {
  return riskLevel >= RiskLevel.LEVEL_3 && probability >= 60;
}

/**
 * Estimate time to overflow
 */
export function estimateTimeToOverflow(
  currentLevel: number,
  dangerLevel: number,
  rateOfRise: number
): number | null {
  if (rateOfRise <= 0 || currentLevel >= dangerLevel) {
    return null;
  }
  return (dangerLevel - currentLevel) / rateOfRise;
}

/**
 * Calculate distance between two coordinates (Haversine formula)
 */
export function calculateDistance(
  coord1: Coordinates,
  coord2: Coordinates
): number {
  const R = 6371e3; // Earth radius in meters
  const φ1 = (coord1.latitude * Math.PI) / 180;
  const φ2 = (coord2.latitude * Math.PI) / 180;
  const Δφ = ((coord2.latitude - coord1.latitude) * Math.PI) / 180;
  const Δλ = ((coord2.longitude - coord1.longitude) * Math.PI) / 180;

  const a =
    Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
    Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) * Math.sin(Δλ / 2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

  return R * c; // Distance in meters
}

/**
 * Find nearest shelter
 */
export function findNearestShelter(
  location: Coordinates,
  shelters: EvacuationShelter[]
): EvacuationShelter | null {
  if (shelters.length === 0) return null;

  let nearest = shelters[0];
  let minDistance = calculateDistance(location, nearest.location);

  for (const shelter of shelters.slice(1)) {
    const distance = calculateDistance(location, shelter.location);
    if (distance < minDistance) {
      minDistance = distance;
      nearest = shelter;
    }
  }

  return nearest;
}

/**
 * Validate time horizon
 */
export function isValidTimeHorizon(horizon: string): horizon is TimeHorizon {
  return ['1h', '6h', '24h', '72h'].includes(horizon);
}

/**
 * Validate spatial resolution
 */
export function isValidResolution(resolution: string): resolution is SpatialResolution {
  return ['coarse', 'medium', 'fine'].includes(resolution);
}

/**
 * Format risk level for display
 */
export function formatRiskLevel(level: RiskLevel): { ko: string; en: string; color: string } {
  const formats: Record<RiskLevel, { ko: string; en: string; color: string }> = {
    [RiskLevel.LEVEL_0]: { ko: '정상', en: 'Normal', color: 'green' },
    [RiskLevel.LEVEL_1]: { ko: '관심', en: 'Attention', color: 'blue' },
    [RiskLevel.LEVEL_2]: { ko: '주의', en: 'Caution', color: 'yellow' },
    [RiskLevel.LEVEL_3]: { ko: '경계', en: 'Warning', color: 'orange' },
    [RiskLevel.LEVEL_4]: { ko: '심각', en: 'Severe', color: 'red' },
  };
  return formats[level];
}

// ============================================================================
// Default Export
// ============================================================================

export default FloodPredictionSDK;
