/**
 * WIA-ENE-029: Light Pollution Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Export all types
export * from './types';

import {
  LightPollutionMeasurement,
  LightingFixture,
  LightPollutionAlert,
  LightPollutionStatistics,
  PerformanceReport,
  ComplianceStatus,
  ZoneLightingLimits,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
  DarkSkyZone,
  BortleScale,
  PollutionSeverity,
  AlertType,
  ReportType,
  ViolationType,
  ShieldingType,
  FixtureType,
  SkyBrightness,
  Illuminance,
  Luminance,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface LightPollutionSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class LightPollutionSDK {
  private config: Required<LightPollutionSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: LightPollutionSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'ENE-029',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Measurement APIs
  // ==========================================================================

  /**
   * Submit a light pollution measurement
   */
  async submitMeasurement(
    measurement: Omit<LightPollutionMeasurement, 'measurementId' | 'timestamp'>
  ): Promise<ApiResponse<LightPollutionMeasurement>> {
    return this.post<LightPollutionMeasurement>('/api/v1/measurements', measurement);
  }

  /**
   * Get measurement by ID
   */
  async getMeasurement(measurementId: string): Promise<ApiResponse<LightPollutionMeasurement>> {
    return this.get<LightPollutionMeasurement>(`/api/v1/measurements/${measurementId}`);
  }

  /**
   * List measurements with pagination
   */
  async listMeasurements(
    params?: PaginationParams & { zone?: DarkSkyZone; startDate?: string; endDate?: string }
  ): Promise<ApiResponse<PaginatedResponse<LightPollutionMeasurement>>> {
    const queryParams = new URLSearchParams();
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<LightPollutionMeasurement>>(
      `/api/v1/measurements?${queryParams.toString()}`
    );
  }

  /**
   * Get sky brightness data for a location
   */
  async getSkyBrightness(
    latitude: number,
    longitude: number
  ): Promise<ApiResponse<SkyBrightness>> {
    return this.get<SkyBrightness>(
      `/api/v1/measurements/sky-brightness?lat=${latitude}&lon=${longitude}`
    );
  }

  /**
   * Get real-time illuminance data
   */
  async getIlluminance(
    latitude: number,
    longitude: number
  ): Promise<ApiResponse<Illuminance>> {
    return this.get<Illuminance>(
      `/api/v1/measurements/illuminance?lat=${latitude}&lon=${longitude}`
    );
  }

  // ==========================================================================
  // Lighting Fixture APIs
  // ==========================================================================

  /**
   * Register a lighting fixture
   */
  async registerFixture(
    fixture: Omit<LightingFixture, 'fixtureId' | 'installDate'>
  ): Promise<ApiResponse<LightingFixture>> {
    return this.post<LightingFixture>('/api/v1/fixtures', fixture);
  }

  /**
   * Get fixture by ID
   */
  async getFixture(fixtureId: string): Promise<ApiResponse<LightingFixture>> {
    return this.get<LightingFixture>(`/api/v1/fixtures/${fixtureId}`);
  }

  /**
   * Update fixture
   */
  async updateFixture(
    fixtureId: string,
    updates: Partial<LightingFixture>
  ): Promise<ApiResponse<LightingFixture>> {
    return this.put<LightingFixture>(`/api/v1/fixtures/${fixtureId}`, updates);
  }

  /**
   * Delete fixture
   */
  async deleteFixture(fixtureId: string): Promise<ApiResponse<void>> {
    return this.delete<void>(`/api/v1/fixtures/${fixtureId}`);
  }

  /**
   * List fixtures in an area
   */
  async listFixtures(
    params?: PaginationParams & { zone?: DarkSkyZone; type?: FixtureType }
  ): Promise<ApiResponse<PaginatedResponse<LightingFixture>>> {
    const queryParams = new URLSearchParams();
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<LightingFixture>>(
      `/api/v1/fixtures?${queryParams.toString()}`
    );
  }

  /**
   * Check fixture compliance
   */
  async checkFixtureCompliance(fixtureId: string): Promise<ApiResponse<ComplianceStatus>> {
    return this.get<ComplianceStatus>(`/api/v1/fixtures/${fixtureId}/compliance`);
  }

  // ==========================================================================
  // Zone Management APIs
  // ==========================================================================

  /**
   * Get lighting limits for a zone
   */
  async getZoneLimits(zone: DarkSkyZone): Promise<ApiResponse<ZoneLightingLimits>> {
    return this.get<ZoneLightingLimits>(`/api/v1/zones/${zone}/limits`);
  }

  /**
   * Get zone classification for a location
   */
  async getZoneClassification(
    latitude: number,
    longitude: number
  ): Promise<ApiResponse<{ zone: DarkSkyZone; limits: ZoneLightingLimits }>> {
    return this.get<{ zone: DarkSkyZone; limits: ZoneLightingLimits }>(
      `/api/v1/zones/classify?lat=${latitude}&lon=${longitude}`
    );
  }

  // ==========================================================================
  // Alert & Monitoring APIs
  // ==========================================================================

  /**
   * Create an alert
   */
  async createAlert(
    alert: Omit<LightPollutionAlert, 'alertId' | 'timestamp' | 'status'>
  ): Promise<ApiResponse<LightPollutionAlert>> {
    return this.post<LightPollutionAlert>('/api/v1/alerts', alert);
  }

  /**
   * Get alert by ID
   */
  async getAlert(alertId: string): Promise<ApiResponse<LightPollutionAlert>> {
    return this.get<LightPollutionAlert>(`/api/v1/alerts/${alertId}`);
  }

  /**
   * List active alerts
   */
  async listAlerts(
    params?: PaginationParams & { type?: AlertType; severity?: PollutionSeverity }
  ): Promise<ApiResponse<PaginatedResponse<LightPollutionAlert>>> {
    const queryParams = new URLSearchParams();
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<LightPollutionAlert>>(
      `/api/v1/alerts?${queryParams.toString()}`
    );
  }

  /**
   * Acknowledge an alert
   */
  async acknowledgeAlert(alertId: string): Promise<ApiResponse<LightPollutionAlert>> {
    return this.post<LightPollutionAlert>(`/api/v1/alerts/${alertId}/acknowledge`, {});
  }

  /**
   * Resolve an alert
   */
  async resolveAlert(alertId: string, resolution: string): Promise<ApiResponse<LightPollutionAlert>> {
    return this.post<LightPollutionAlert>(`/api/v1/alerts/${alertId}/resolve`, { resolution });
  }

  // ==========================================================================
  // Analytics & Reporting APIs
  // ==========================================================================

  /**
   * Get light pollution statistics
   */
  async getStatistics(
    region: string,
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<LightPollutionStatistics>> {
    const queryParams = new URLSearchParams({
      region,
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
    });
    return this.get<LightPollutionStatistics>(
      `/api/v1/analytics/statistics?${queryParams.toString()}`
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

  /**
   * Get compliance dashboard
   */
  async getComplianceDashboard(
    region: string,
    zone?: DarkSkyZone
  ): Promise<ApiResponse<{
    totalFixtures: number;
    compliantFixtures: number;
    violations: number;
    complianceRate: number;
  }>> {
    const queryParams = new URLSearchParams({ region });
    if (zone) queryParams.append('zone', zone);
    return this.get<{
      totalFixtures: number;
      compliantFixtures: number;
      violations: number;
      complianceRate: number;
    }>(`/api/v1/analytics/compliance?${queryParams.toString()}`);
  }

  // ==========================================================================
  // Violation Management APIs
  // ==========================================================================

  /**
   * Report a violation
   */
  async reportViolation(
    fixtureId: string,
    violationType: ViolationType,
    description: string
  ): Promise<ApiResponse<{ violationId: string }>> {
    return this.post<{ violationId: string }>('/api/v1/violations', {
      fixtureId,
      violationType,
      description,
    });
  }

  /**
   * List violations
   */
  async listViolations(
    params?: PaginationParams & { resolved?: boolean; type?: ViolationType }
  ): Promise<ApiResponse<PaginatedResponse<any>>> {
    const queryParams = new URLSearchParams();
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined) {
          queryParams.append(key, String(value));
        }
      });
    }
    return this.get<PaginatedResponse<any>>(
      `/api/v1/violations?${queryParams.toString()}`
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
      console.log(`[WIA-ENE-029] ${method} ${url}`, body || '');
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
        console.error('[WIA-ENE-029] Request failed:', error);
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
 * Calculate Bortle scale from sky brightness (mag/arcsec²)
 */
export function calculateBortleScale(skyBrightness: number): BortleScale {
  if (skyBrightness >= 21.7) return BortleScale.EXCELLENT_DARK_SKY;
  if (skyBrightness >= 21.5) return BortleScale.TYPICAL_DARK_SKY;
  if (skyBrightness >= 21.3) return BortleScale.RURAL_SKY;
  if (skyBrightness >= 20.5) return BortleScale.RURAL_SUBURBAN_TRANSITION;
  if (skyBrightness >= 19.5) return BortleScale.SUBURBAN_SKY;
  if (skyBrightness >= 18.5) return BortleScale.BRIGHT_SUBURBAN_SKY;
  if (skyBrightness >= 18.0) return BortleScale.SUBURBAN_URBAN_TRANSITION;
  if (skyBrightness >= 17.0) return BortleScale.CITY_SKY;
  return BortleScale.INNER_CITY_SKY;
}

/**
 * Estimate visible stars from Bortle scale
 */
export function estimateVisibleStars(bortle: BortleScale): number {
  const starsMap: Record<BortleScale, number> = {
    [BortleScale.EXCELLENT_DARK_SKY]: 5000,
    [BortleScale.TYPICAL_DARK_SKY]: 4000,
    [BortleScale.RURAL_SKY]: 2500,
    [BortleScale.RURAL_SUBURBAN_TRANSITION]: 1000,
    [BortleScale.SUBURBAN_SKY]: 500,
    [BortleScale.BRIGHT_SUBURBAN_SKY]: 250,
    [BortleScale.SUBURBAN_URBAN_TRANSITION]: 100,
    [BortleScale.CITY_SKY]: 50,
    [BortleScale.INNER_CITY_SKY]: 10,
  };
  return starsMap[bortle] || 0;
}

/**
 * Calculate melatonin suppression from melanopic lux
 */
export function calculateMelatoninSuppression(
  melanopicLux: number,
  exposureDuration: number // minutes
): number {
  // Simplified model: suppression increases with melanopic lux and duration
  // Based on research: 50% suppression at ~100 melanopic lux for 1 hour
  const baseSupression = Math.min((melanopicLux / 100) * 50, 100);
  const durationFactor = Math.min(exposureDuration / 60, 1);
  return Math.min(baseSupression * durationFactor, 100);
}

/**
 * Check if fixture is compliant for zone
 */
export function isFixtureCompliant(
  fixture: LightingFixture,
  zoneLimits: ZoneLightingLimits
): boolean {
  const { specifications } = fixture;

  // Check color temperature
  if (specifications.colorTemperature > zoneLimits.maxColorTemperature) {
    return false;
  }

  // Check upward light ratio
  if (specifications.upwardLightRatio > zoneLimits.maxUpwardLightRatio) {
    return false;
  }

  // Check shielding requirements
  if (zoneLimits.zone === DarkSkyZone.E1_INTRINSICALLY_DARK ||
      zoneLimits.zone === DarkSkyZone.E2_DARK) {
    if (specifications.shielding !== ShieldingType.FULLY_SHIELDED) {
      return false;
    }
  }

  return true;
}

/**
 * Convert illuminance to approximate sky brightness
 * Rough estimation for ground illuminance to sky brightness conversion
 */
export function estimateSkyBrightness(illuminance: number): number {
  // Very rough logarithmic relationship
  // Higher illuminance → brighter sky → lower mag/arcsec²
  // This is a simplified model
  const baseSkyBrightness = 22.0; // Dark sky baseline
  const reduction = Math.log10(Math.max(illuminance, 0.001)) * 0.5;
  return Math.max(baseSkyBrightness - reduction, 15.0);
}

/**
 * Calculate energy waste from upward light
 */
export function calculateEnergyWaste(
  wattage: number,
  upwardLightRatio: number,
  hoursPerNight: number,
  nightsPerYear: number = 365
): number {
  // Energy wasted (kWh/year) = wattage × upward ratio × hours × nights / 1000
  return (wattage * (upwardLightRatio / 100) * hoursPerNight * nightsPerYear) / 1000;
}

/**
 * Assess pollution severity from sky brightness
 */
export function assessPollutionSeverity(skyBrightness: number): PollutionSeverity {
  if (skyBrightness >= 21.5) return PollutionSeverity.NONE;
  if (skyBrightness >= 20.5) return PollutionSeverity.MINIMAL;
  if (skyBrightness >= 19.5) return PollutionSeverity.LOW;
  if (skyBrightness >= 18.5) return PollutionSeverity.MODERATE;
  if (skyBrightness >= 17.5) return PollutionSeverity.HIGH;
  if (skyBrightness >= 16.5) return PollutionSeverity.SEVERE;
  return PollutionSeverity.EXTREME;
}

/**
 * Validate dark sky zone
 */
export function isValidDarkSkyZone(zone: string): zone is DarkSkyZone {
  return Object.values(DarkSkyZone).includes(zone as DarkSkyZone);
}

/**
 * Get zone lighting limits (default values)
 */
export function getDefaultZoneLimits(zone: DarkSkyZone): ZoneLightingLimits {
  const limitsMap: Record<DarkSkyZone, ZoneLightingLimits> = {
    [DarkSkyZone.E1_INTRINSICALLY_DARK]: {
      zone,
      maxVerticalIlluminance: 0.1,
      maxUpwardLightRatio: 0,
      maxColorTemperature: 2200,
      curfewRequired: true,
      curfewHours: { start: '22:00', end: '06:00' },
    },
    [DarkSkyZone.E2_DARK]: {
      zone,
      maxVerticalIlluminance: 1,
      maxUpwardLightRatio: 0,
      maxColorTemperature: 2700,
      curfewRequired: true,
      curfewHours: { start: '23:00', end: '06:00' },
    },
    [DarkSkyZone.E3_RURAL]: {
      zone,
      maxVerticalIlluminance: 2,
      maxUpwardLightRatio: 5,
      maxColorTemperature: 3000,
      curfewRequired: false,
      curfewHours: null,
    },
    [DarkSkyZone.E4_SUBURBAN]: {
      zone,
      maxVerticalIlluminance: 5,
      maxUpwardLightRatio: 10,
      maxColorTemperature: 4000,
      curfewRequired: false,
      curfewHours: null,
    },
    [DarkSkyZone.E5_URBAN]: {
      zone,
      maxVerticalIlluminance: 10,
      maxUpwardLightRatio: 15,
      maxColorTemperature: 5000,
      curfewRequired: false,
      curfewHours: null,
    },
  };

  return limitsMap[zone];
}

// ============================================================================
// Default Export
// ============================================================================

export default LightPollutionSDK;
