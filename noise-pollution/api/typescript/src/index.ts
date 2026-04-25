/**
 * WIA-ENE-028: Noise Pollution Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Export all types
export * from './types';

import {
  NoiseMeasurement,
  MonitoringStation,
  NoiseSource,
  TimePeriodNoise,
  HealthImpactAssessment,
  KPIDashboard,
  NoiseAlert,
  NoiseComplaint,
  QuietZone,
  MitigationStrategy,
  NoiseBarrier,
  RealtimeNoiseUpdate,
  NoiseHeatmapPoint,
  StatisticalSummary,
  Certification,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
  NoiseSourceCategory,
  TimePeriod,
  StationType,
  OperationalStatus,
  AlertType,
  ComplaintStatus,
  CertificationLevel,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface NoisePollutionSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class NoisePollutionSDK {
  private config: Required<NoisePollutionSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: NoisePollutionSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'ENE-028',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Measurement APIs
  // ==========================================================================

  /**
   * Get noise measurement by ID
   */
  async getMeasurement(measurementId: string): Promise<ApiResponse<NoiseMeasurement>> {
    return this.get<NoiseMeasurement>(`/api/v1/noise/measurement/${measurementId}`);
  }

  /**
   * Create new measurement
   */
  async createMeasurement(
    measurement: Omit<NoiseMeasurement, 'measurementId' | 'timestamp'>
  ): Promise<ApiResponse<NoiseMeasurement>> {
    return this.post<NoiseMeasurement>('/api/v1/noise/measurement', measurement);
  }

  /**
   * List measurements with filters
   */
  async listMeasurements(params?: {
    stationId?: string;
    dateRange?: DateRangeFilter;
    pagination?: PaginationParams;
  }): Promise<ApiResponse<PaginatedResponse<NoiseMeasurement>>> {
    const queryParams = new URLSearchParams();

    if (params?.stationId) {
      queryParams.append('stationId', params.stationId);
    }
    if (params?.dateRange) {
      queryParams.append('startDate', params.dateRange.startDate);
      queryParams.append('endDate', params.dateRange.endDate);
    }
    if (params?.pagination) {
      queryParams.append('page', String(params.pagination.page));
      queryParams.append('limit', String(params.pagination.limit));
      if (params.pagination.sortBy) {
        queryParams.append('sortBy', params.pagination.sortBy);
        queryParams.append('sortOrder', params.pagination.sortOrder || 'asc');
      }
    }

    return this.get<PaginatedResponse<NoiseMeasurement>>(
      `/api/v1/noise/measurement?${queryParams.toString()}`
    );
  }

  /**
   * Get statistical summary for a station
   */
  async getStationStatistics(
    stationId: string,
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<StatisticalSummary>> {
    const queryParams = new URLSearchParams({
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
    });
    return this.get<StatisticalSummary>(
      `/api/v1/noise/station/${stationId}/statistics?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Station Management APIs
  // ==========================================================================

  /**
   * Get station information
   */
  async getStation(stationId: string): Promise<ApiResponse<MonitoringStation>> {
    return this.get<MonitoringStation>(`/api/v1/noise/station/${stationId}`);
  }

  /**
   * List all monitoring stations
   */
  async listStations(params?: {
    type?: StationType;
    status?: OperationalStatus;
    region?: string;
  }): Promise<ApiResponse<MonitoringStation[]>> {
    const queryParams = new URLSearchParams();
    if (params?.type) queryParams.append('type', params.type);
    if (params?.status) queryParams.append('status', params.status);
    if (params?.region) queryParams.append('region', params.region);

    return this.get<MonitoringStation[]>(
      `/api/v1/noise/station${queryParams.toString() ? '?' + queryParams.toString() : ''}`
    );
  }

  /**
   * Update station information
   */
  async updateStation(
    stationId: string,
    updates: Partial<MonitoringStation>
  ): Promise<ApiResponse<MonitoringStation>> {
    return this.put<MonitoringStation>(`/api/v1/noise/station/${stationId}`, updates);
  }

  /**
   * Get station status (operational info)
   */
  async getStationStatus(stationId: string): Promise<ApiResponse<{
    status: OperationalStatus;
    lastMeasurement: string;
    dataQuality: number;
    uptime: number;
  }>> {
    return this.get(`/api/v1/noise/station/${stationId}/status`);
  }

  // ==========================================================================
  // Realtime Data APIs
  // ==========================================================================

  /**
   * Get realtime noise levels for all active stations
   */
  async getRealtimeData(region?: string): Promise<ApiResponse<RealtimeNoiseUpdate[]>> {
    const queryParams = region ? new URLSearchParams({ region }) : '';
    return this.get<RealtimeNoiseUpdate[]>(
      `/api/v1/noise/realtime${queryParams ? '?' + queryParams : ''}`
    );
  }

  /**
   * Subscribe to realtime updates (returns WebSocket URL)
   */
  getRealtimeWebSocketURL(stationId?: string): string {
    const wsEndpoint = this.config.endpoint.replace('https://', 'wss://').replace('http://', 'ws://');
    return stationId
      ? `${wsEndpoint}/api/v1/noise/realtime/ws?stationId=${stationId}`
      : `${wsEndpoint}/api/v1/noise/realtime/ws`;
  }

  // ==========================================================================
  // Noise Source APIs
  // ==========================================================================

  /**
   * Get noise source information
   */
  async getNoiseSource(sourceId: string): Promise<ApiResponse<NoiseSource>> {
    return this.get<NoiseSource>(`/api/v1/noise/source/${sourceId}`);
  }

  /**
   * List noise sources
   */
  async listNoiseSources(params?: {
    category?: NoiseSourceCategory;
    region?: string;
    compliant?: boolean;
  }): Promise<ApiResponse<NoiseSource[]>> {
    const queryParams = new URLSearchParams();
    if (params?.category) queryParams.append('category', params.category);
    if (params?.region) queryParams.append('region', params.region);
    if (params?.compliant !== undefined) queryParams.append('compliant', String(params.compliant));

    return this.get<NoiseSource[]>(
      `/api/v1/noise/source${queryParams.toString() ? '?' + queryParams.toString() : ''}`
    );
  }

  /**
   * Create or register new noise source
   */
  async createNoiseSource(
    source: Omit<NoiseSource, 'sourceId'>
  ): Promise<ApiResponse<NoiseSource>> {
    return this.post<NoiseSource>('/api/v1/noise/source', source);
  }

  /**
   * Update noise source
   */
  async updateNoiseSource(
    sourceId: string,
    updates: Partial<NoiseSource>
  ): Promise<ApiResponse<NoiseSource>> {
    return this.put<NoiseSource>(`/api/v1/noise/source/${sourceId}`, updates);
  }

  // ==========================================================================
  // Time Period Analysis APIs
  // ==========================================================================

  /**
   * Get time period analysis (day/evening/night)
   */
  async getTimePeriodAnalysis(
    location: string,
    date: string
  ): Promise<ApiResponse<TimePeriodNoise>> {
    const queryParams = new URLSearchParams({ location, date });
    return this.get<TimePeriodNoise>(`/api/v1/noise/time-period?${queryParams.toString()}`);
  }

  /**
   * Get time period analysis for a date range
   */
  async getTimePeriodTrends(
    location: string,
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<TimePeriodNoise[]>> {
    const queryParams = new URLSearchParams({
      location,
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
    });
    return this.get<TimePeriodNoise[]>(`/api/v1/noise/time-period/trends?${queryParams.toString()}`);
  }

  // ==========================================================================
  // Health Impact APIs
  // ==========================================================================

  /**
   * Get health impact assessment
   */
  async getHealthImpact(
    region: string,
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<HealthImpactAssessment>> {
    const queryParams = new URLSearchParams({
      region,
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
    });
    return this.get<HealthImpactAssessment>(
      `/api/v1/noise/analytics/health?${queryParams.toString()}`
    );
  }

  /**
   * Calculate exposed population
   */
  async calculateExposedPopulation(
    region: string,
    thresholdLevel_dBA: number
  ): Promise<ApiResponse<{ population: number; percentage: number }>> {
    const queryParams = new URLSearchParams({
      region,
      threshold: String(thresholdLevel_dBA),
    });
    return this.get(
      `/api/v1/noise/analytics/exposed-population?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Heatmap & Visualization APIs
  // ==========================================================================

  /**
   * Get noise heatmap data
   */
  async getNoiseHeatmap(params: {
    region: string;
    date?: string;
    timePeriod?: TimePeriod;
    resolution?: number;  // grid resolution in meters
  }): Promise<ApiResponse<NoiseHeatmapPoint[]>> {
    const queryParams = new URLSearchParams({ region: params.region });
    if (params.date) queryParams.append('date', params.date);
    if (params.timePeriod) queryParams.append('timePeriod', params.timePeriod);
    if (params.resolution) queryParams.append('resolution', String(params.resolution));

    return this.get<NoiseHeatmapPoint[]>(`/api/v1/noise/heatmap?${queryParams.toString()}`);
  }

  // ==========================================================================
  // Alert Management APIs
  // ==========================================================================

  /**
   * Get active alerts
   */
  async getActiveAlerts(severity?: 'low' | 'medium' | 'high' | 'critical'): Promise<ApiResponse<NoiseAlert[]>> {
    const queryParams = severity ? new URLSearchParams({ severity }) : '';
    return this.get<NoiseAlert[]>(
      `/api/v1/noise/alerts${queryParams ? '?' + queryParams : ''}`
    );
  }

  /**
   * Get alert by ID
   */
  async getAlert(alertId: string): Promise<ApiResponse<NoiseAlert>> {
    return this.get<NoiseAlert>(`/api/v1/noise/alerts/${alertId}`);
  }

  /**
   * Acknowledge alert
   */
  async acknowledgeAlert(alertId: string): Promise<ApiResponse<NoiseAlert>> {
    return this.post<NoiseAlert>(`/api/v1/noise/alerts/${alertId}/acknowledge`, {});
  }

  // ==========================================================================
  // Complaint Management APIs
  // ==========================================================================

  /**
   * Submit noise complaint
   */
  async submitComplaint(
    complaint: Omit<NoiseComplaint, 'complaintId' | 'submittedDate' | 'status'>
  ): Promise<ApiResponse<NoiseComplaint>> {
    return this.post<NoiseComplaint>('/api/v1/noise/complaint', complaint);
  }

  /**
   * Get complaint by ID
   */
  async getComplaint(complaintId: string): Promise<ApiResponse<NoiseComplaint>> {
    return this.get<NoiseComplaint>(`/api/v1/noise/complaint/${complaintId}`);
  }

  /**
   * List complaints with filters
   */
  async listComplaints(params?: {
    status?: ComplaintStatus;
    region?: string;
    dateRange?: DateRangeFilter;
    pagination?: PaginationParams;
  }): Promise<ApiResponse<PaginatedResponse<NoiseComplaint>>> {
    const queryParams = new URLSearchParams();
    if (params?.status) queryParams.append('status', params.status);
    if (params?.region) queryParams.append('region', params.region);
    if (params?.dateRange) {
      queryParams.append('startDate', params.dateRange.startDate);
      queryParams.append('endDate', params.dateRange.endDate);
    }
    if (params?.pagination) {
      queryParams.append('page', String(params.pagination.page));
      queryParams.append('limit', String(params.pagination.limit));
    }

    return this.get<PaginatedResponse<NoiseComplaint>>(
      `/api/v1/noise/complaint?${queryParams.toString()}`
    );
  }

  /**
   * Update complaint status
   */
  async updateComplaint(
    complaintId: string,
    updates: Partial<NoiseComplaint>
  ): Promise<ApiResponse<NoiseComplaint>> {
    return this.put<NoiseComplaint>(`/api/v1/noise/complaint/${complaintId}`, updates);
  }

  // ==========================================================================
  // Quiet Zone APIs
  // ==========================================================================

  /**
   * Get quiet zone information
   */
  async getQuietZone(zoneId: string): Promise<ApiResponse<QuietZone>> {
    return this.get<QuietZone>(`/api/v1/noise/quiet-zone/${zoneId}`);
  }

  /**
   * List quiet zones
   */
  async listQuietZones(region?: string): Promise<ApiResponse<QuietZone[]>> {
    const queryParams = region ? new URLSearchParams({ region }) : '';
    return this.get<QuietZone[]>(
      `/api/v1/noise/quiet-zone${queryParams ? '?' + queryParams : ''}`
    );
  }

  /**
   * Check if location is in quiet zone
   */
  async checkQuietZone(latitude: number, longitude: number): Promise<ApiResponse<{
    inQuietZone: boolean;
    zones: QuietZone[];
  }>> {
    const queryParams = new URLSearchParams({
      latitude: String(latitude),
      longitude: String(longitude),
    });
    return this.get(`/api/v1/noise/quiet-zone/check?${queryParams.toString()}`);
  }

  // ==========================================================================
  // Mitigation & Barriers APIs
  // ==========================================================================

  /**
   * Get mitigation strategies
   */
  async getMitigationStrategies(region?: string): Promise<ApiResponse<MitigationStrategy[]>> {
    const queryParams = region ? new URLSearchParams({ region }) : '';
    return this.get<MitigationStrategy[]>(
      `/api/v1/noise/mitigation${queryParams ? '?' + queryParams : ''}`
    );
  }

  /**
   * Get noise barriers
   */
  async getNoiseBarriers(region?: string): Promise<ApiResponse<NoiseBarrier[]>> {
    const queryParams = region ? new URLSearchParams({ region }) : '';
    return this.get<NoiseBarrier[]>(
      `/api/v1/noise/barrier${queryParams ? '?' + queryParams : ''}`
    );
  }

  // ==========================================================================
  // KPI & Analytics APIs
  // ==========================================================================

  /**
   * Get KPI dashboard
   */
  async getKPIDashboard(
    region: string,
    date?: string
  ): Promise<ApiResponse<KPIDashboard>> {
    const queryParams = new URLSearchParams({ region });
    if (date) queryParams.append('date', date);
    return this.get<KPIDashboard>(`/api/v1/noise/analytics/kpi?${queryParams.toString()}`);
  }

  /**
   * Get compliance report
   */
  async getComplianceReport(
    region: string,
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<{
    totalStations: number;
    compliantStations: number;
    complianceRate: number;
    violations: { stationId: string; violations: string[] }[];
  }>> {
    const queryParams = new URLSearchParams({
      region,
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
    });
    return this.get(`/api/v1/noise/compliance?${queryParams.toString()}`);
  }

  // ==========================================================================
  // Certification APIs
  // ==========================================================================

  /**
   * Get station certification
   */
  async getCertification(stationId: string): Promise<ApiResponse<Certification>> {
    return this.get<Certification>(`/api/v1/noise/certification/${stationId}`);
  }

  /**
   * Apply for certification
   */
  async applyCertification(
    stationId: string,
    level: CertificationLevel
  ): Promise<ApiResponse<Certification>> {
    return this.post<Certification>('/api/v1/noise/certification/apply', {
      stationId,
      level,
    });
  }

  /**
   * Renew certification
   */
  async renewCertification(certificateId: string): Promise<ApiResponse<Certification>> {
    return this.post<Certification>(`/api/v1/noise/certification/${certificateId}/renew`, {});
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
      console.log(`[WIA-ENE-028] ${method} ${url}`, body || '');
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
        console.error('[WIA-ENE-028] Request failed:', error);
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
 * Calculate Lden (Day-Evening-Night Level)
 * Lden = 10 × log10[(12×10^(Ld/10) + 4×10^((Le+5)/10) + 8×10^((Ln+10)/10))/24]
 */
export function calculateLden(ld: number, le: number, ln: number): number {
  const term1 = 12 * Math.pow(10, ld / 10);
  const term2 = 4 * Math.pow(10, (le + 5) / 10);
  const term3 = 8 * Math.pow(10, (ln + 10) / 10);
  return 10 * Math.log10((term1 + term2 + term3) / 24);
}

/**
 * Calculate highly annoyed percentage from Lden
 * HA(%) = 78.9270 - 3.1172 × Lden + 0.0244 × Lden²
 */
export function calculateHighlyAnnoyed(lden: number): number {
  return Math.max(0, 78.9270 - 3.1172 * lden + 0.0244 * lden * lden);
}

/**
 * Calculate high sleep disturbance percentage from Lnight
 * HSD(%) = 20.8 - 1.05 × Lnight + 0.01486 × Lnight²
 */
export function calculateSleepDisturbance(lnight: number): number {
  return Math.max(0, 20.8 - 1.05 * lnight + 0.01486 * lnight * lnight);
}

/**
 * Estimate noise reduction from distance doubling
 * Point source: -6 dB per distance doubling
 * Line source: -3 dB per distance doubling
 */
export function estimateNoiseReduction(
  sourceType: 'point' | 'line',
  currentDistance: number,
  newDistance: number
): number {
  const ratio = newDistance / currentDistance;
  const attenuationPerDoubling = sourceType === 'point' ? 6 : 3;
  return attenuationPerDoubling * Math.log2(ratio);
}

/**
 * Check if noise level exceeds regulation
 */
export function isRegulationExceeded(
  measured_dBA: number,
  limit_dBA: number,
  tolerance_dB: number = 0
): boolean {
  return measured_dBA > (limit_dBA + tolerance_dB);
}

/**
 * Convert dB to linear scale
 */
export function dbToLinear(db: number): number {
  return Math.pow(10, db / 10);
}

/**
 * Convert linear scale to dB
 */
export function linearToDb(linear: number): number {
  return 10 * Math.log10(linear);
}

/**
 * Add noise levels (energetic addition)
 * L_total = 10 × log10(10^(L1/10) + 10^(L2/10) + ...)
 */
export function addNoiseLevels(...levels_dB: number[]): number {
  const sum = levels_dB.reduce((acc, level) => acc + dbToLinear(level), 0);
  return linearToDb(sum);
}

/**
 * Subtract noise levels (background noise correction)
 * L_source = 10 × log10(10^(L_total/10) - 10^(L_background/10))
 */
export function subtractNoiseLevels(total_dB: number, background_dB: number): number {
  const difference = dbToLinear(total_dB) - dbToLinear(background_dB);
  return difference > 0 ? linearToDb(difference) : 0;
}

/**
 * Validate noise level range
 */
export function isValidNoiseLevel(level_dB: number): boolean {
  return level_dB >= 0 && level_dB <= 140;  // 0 dB to 140 dB (pain threshold)
}

/**
 * Get noise category based on level
 */
export function getNoiseCategory(leq_dBA: number): 'low' | 'moderate' | 'high' | 'very_high' {
  if (leq_dBA < 55) return 'low';
  if (leq_dBA < 65) return 'moderate';
  if (leq_dBA < 75) return 'high';
  return 'very_high';
}

// ============================================================================
// Default Export
// ============================================================================

export default NoisePollutionSDK;
