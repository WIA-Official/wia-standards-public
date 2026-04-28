/**
 * WIA-ENE-039: Resource Depletion Response Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Export all types
export * from './types';

import {
  ResourceStatus,
  DepletionForecast,
  MitigationStrategy,
  EarlyWarningIndicator,
  StrategicReserve,
  AlternativeMaterial,
  ResourceReport,
  KPIDashboard,
  ResourceDigitalTwin,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
  ResourceCategoryCode,
  DepletionRiskLevel,
  StrategyType,
  ForecastModelType,
  AlertLevel,
  ReportType,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface ResourceDepletionSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class ResourceDepletionSDK {
  private config: Required<ResourceDepletionSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: ResourceDepletionSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'ENE-039',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Resource Status APIs
  // ==========================================================================

  /**
   * Get resource status by ID
   */
  async getResourceStatus(resourceId: string): Promise<ApiResponse<ResourceStatus>> {
    return this.get<ResourceStatus>(`/api/v1/resources/${resourceId}/status`);
  }

  /**
   * List all resources with optional filtering
   */
  async listResources(
    params?: PaginationParams & {
      categoryCode?: ResourceCategoryCode;
      riskLevel?: DepletionRiskLevel;
      country?: string;
    }
  ): Promise<ApiResponse<PaginatedResponse<ResourceStatus>>> {
    const queryParams = this.buildQueryParams(params);
    return this.get<PaginatedResponse<ResourceStatus>>(
      `/api/v1/resources${queryParams ? '?' + queryParams : ''}`
    );
  }

  /**
   * Get critical resources (high risk)
   */
  async getCriticalResources(): Promise<ApiResponse<ResourceStatus[]>> {
    return this.get<ResourceStatus[]>('/api/v1/resources/critical');
  }

  /**
   * Update resource status
   */
  async updateResourceStatus(
    resourceId: string,
    updates: Partial<ResourceStatus>
  ): Promise<ApiResponse<ResourceStatus>> {
    return this.put<ResourceStatus>(`/api/v1/resources/${resourceId}/status`, updates);
  }

  // ==========================================================================
  // Production & Consumption APIs
  // ==========================================================================

  /**
   * Submit production data
   */
  async submitProductionData(
    resourceId: string,
    data: {
      quantity: number;
      period: string;
      country: string;
      facility?: string;
    }
  ): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/resources/${resourceId}/production`, data);
  }

  /**
   * Submit consumption data
   */
  async submitConsumptionData(
    resourceId: string,
    data: {
      quantity: number;
      period: string;
      country: string;
      sector?: string;
    }
  ): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/resources/${resourceId}/consumption`, data);
  }

  // ==========================================================================
  // Depletion Forecast APIs
  // ==========================================================================

  /**
   * Get depletion forecast
   */
  async getDepletionForecast(
    resourceId: string,
    modelType?: ForecastModelType
  ): Promise<ApiResponse<DepletionForecast>> {
    const queryParams = modelType ? `?modelType=${modelType}` : '';
    return this.get<DepletionForecast>(
      `/api/v1/resources/${resourceId}/forecast${queryParams}`
    );
  }

  /**
   * Generate new forecast
   */
  async generateForecast(
    resourceId: string,
    request: {
      modelType: ForecastModelType;
      scenarios: any[];
      timeHorizon: number; // years
    }
  ): Promise<ApiResponse<DepletionForecast>> {
    return this.post<DepletionForecast>(
      `/api/v1/resources/${resourceId}/forecast`,
      request
    );
  }

  /**
   * Compare multiple scenarios
   */
  async compareScenarios(
    resourceId: string,
    scenarioIds: string[]
  ): Promise<ApiResponse<any>> {
    return this.post<any>(`/api/v1/resources/${resourceId}/forecast/compare`, {
      scenarioIds,
    });
  }

  // ==========================================================================
  // Early Warning System APIs
  // ==========================================================================

  /**
   * Get active alerts
   */
  async getActiveAlerts(
    resourceId?: string,
    level?: AlertLevel
  ): Promise<ApiResponse<EarlyWarningIndicator[]>> {
    const queryParams = new URLSearchParams();
    if (resourceId) queryParams.append('resourceId', resourceId);
    if (level) queryParams.append('level', level);

    return this.get<EarlyWarningIndicator[]>(
      `/api/v1/alerts/active${queryParams.toString() ? '?' + queryParams.toString() : ''}`
    );
  }

  /**
   * Get indicator by ID
   */
  async getIndicator(indicatorId: string): Promise<ApiResponse<EarlyWarningIndicator>> {
    return this.get<EarlyWarningIndicator>(`/api/v1/alerts/indicators/${indicatorId}`);
  }

  /**
   * Subscribe to alert notifications
   */
  async subscribeToAlerts(
    subscription: {
      resourceIds?: string[];
      alertLevels?: AlertLevel[];
      channels: ('email' | 'sms' | 'webhook')[];
      endpoint?: string;
    }
  ): Promise<ApiResponse<{ subscriptionId: string }>> {
    return this.post<{ subscriptionId: string }>('/api/v1/alerts/subscribe', subscription);
  }

  /**
   * Acknowledge alert
   */
  async acknowledgeAlert(
    indicatorId: string,
    acknowledgment: {
      acknowledgedBy: string;
      notes?: string;
    }
  ): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/alerts/indicators/${indicatorId}/acknowledge`, acknowledgment);
  }

  // ==========================================================================
  // Mitigation Strategies APIs
  // ==========================================================================

  /**
   * Create mitigation strategy
   */
  async createStrategy(
    strategy: Omit<MitigationStrategy, 'strategyId'>
  ): Promise<ApiResponse<MitigationStrategy>> {
    return this.post<MitigationStrategy>('/api/v1/strategies', strategy);
  }

  /**
   * Get strategy by ID
   */
  async getStrategy(strategyId: string): Promise<ApiResponse<MitigationStrategy>> {
    return this.get<MitigationStrategy>(`/api/v1/strategies/${strategyId}`);
  }

  /**
   * List strategies for a resource
   */
  async listStrategies(
    resourceId: string,
    type?: StrategyType
  ): Promise<ApiResponse<MitigationStrategy[]>> {
    const queryParams = type ? `?type=${type}` : '';
    return this.get<MitigationStrategy[]>(
      `/api/v1/strategies?resourceId=${resourceId}${queryParams}`
    );
  }

  /**
   * Update strategy status
   */
  async updateStrategyStatus(
    strategyId: string,
    status: MitigationStrategy['status'],
    notes?: string
  ): Promise<ApiResponse<MitigationStrategy>> {
    return this.put<MitigationStrategy>(`/api/v1/strategies/${strategyId}/status`, {
      status,
      notes,
    });
  }

  /**
   * Evaluate strategy impact
   */
  async evaluateStrategyImpact(
    strategyId: string
  ): Promise<ApiResponse<{ actualImpact: any; variance: any }>> {
    return this.get<{ actualImpact: any; variance: any }>(
      `/api/v1/strategies/${strategyId}/evaluate`
    );
  }

  // ==========================================================================
  // Strategic Reserves APIs
  // ==========================================================================

  /**
   * Get strategic reserve
   */
  async getStrategicReserve(reserveId: string): Promise<ApiResponse<StrategicReserve>> {
    return this.get<StrategicReserve>(`/api/v1/reserves/${reserveId}`);
  }

  /**
   * List reserves by resource
   */
  async listReserves(resourceId?: string): Promise<ApiResponse<StrategicReserve[]>> {
    const queryParams = resourceId ? `?resourceId=${resourceId}` : '';
    return this.get<StrategicReserve[]>(`/api/v1/reserves${queryParams}`);
  }

  /**
   * Update reserve inventory
   */
  async updateReserveInventory(
    reserveId: string,
    update: {
      quantity: number;
      operation: 'add' | 'release';
      reason: string;
    }
  ): Promise<ApiResponse<StrategicReserve>> {
    return this.post<StrategicReserve>(`/api/v1/reserves/${reserveId}/inventory`, update);
  }

  /**
   * Check release criteria
   */
  async checkReleaseCriteria(reserveId: string): Promise<ApiResponse<{
    shouldRelease: boolean;
    triggeredCriteria: string[];
    recommendedAmount: number;
  }>> {
    return this.get<{
      shouldRelease: boolean;
      triggeredCriteria: string[];
      recommendedAmount: number;
    }>(`/api/v1/reserves/${reserveId}/check-criteria`);
  }

  // ==========================================================================
  // Alternative Materials APIs
  // ==========================================================================

  /**
   * Search alternative materials
   */
  async searchAlternatives(
    resourceId: string,
    application?: string
  ): Promise<ApiResponse<AlternativeMaterial[]>> {
    const queryParams = application ? `?application=${application}` : '';
    return this.get<AlternativeMaterial[]>(
      `/api/v1/alternatives?resourceId=${resourceId}${queryParams}`
    );
  }

  /**
   * Get alternative material details
   */
  async getAlternative(materialId: string): Promise<ApiResponse<AlternativeMaterial>> {
    return this.get<AlternativeMaterial>(`/api/v1/alternatives/${materialId}`);
  }

  /**
   * Compare alternatives
   */
  async compareAlternatives(
    materialIds: string[],
    criteria?: ('cost' | 'performance' | 'environmental' | 'availability')[]
  ): Promise<ApiResponse<any>> {
    return this.post<any>('/api/v1/alternatives/compare', {
      materialIds,
      criteria,
    });
  }

  // ==========================================================================
  // Analytics & Reporting APIs
  // ==========================================================================

  /**
   * Get KPI dashboard
   */
  async getKPIDashboard(
    region?: string,
    dateRange?: DateRangeFilter
  ): Promise<ApiResponse<KPIDashboard>> {
    const queryParams = new URLSearchParams();
    if (region) queryParams.append('region', region);
    if (dateRange) {
      queryParams.append('startDate', dateRange.startDate);
      queryParams.append('endDate', dateRange.endDate);
    }

    return this.get<KPIDashboard>(
      `/api/v1/analytics/dashboard${queryParams.toString() ? '?' + queryParams.toString() : ''}`
    );
  }

  /**
   * Generate resource report
   */
  async generateReport(
    reportType: ReportType,
    dateRange: DateRangeFilter,
    resourceIds?: string[]
  ): Promise<ApiResponse<ResourceReport>> {
    return this.post<ResourceReport>('/api/v1/analytics/report', {
      reportType,
      period: dateRange,
      resourceIds,
    });
  }

  /**
   * Get generated report
   */
  async getReport(reportId: string): Promise<ApiResponse<ResourceReport>> {
    return this.get<ResourceReport>(`/api/v1/analytics/report/${reportId}`);
  }

  /**
   * Export data
   */
  async exportData(
    format: 'json' | 'csv' | 'excel',
    query: {
      resourceIds?: string[];
      dateRange?: DateRangeFilter;
      fields?: string[];
    }
  ): Promise<ApiResponse<{ downloadUrl: string }>> {
    return this.post<{ downloadUrl: string }>('/api/v1/analytics/export', {
      format,
      ...query,
    });
  }

  // ==========================================================================
  // Digital Twin APIs
  // ==========================================================================

  /**
   * Get digital twin
   */
  async getDigitalTwin(resourceId: string): Promise<ApiResponse<ResourceDigitalTwin>> {
    return this.get<ResourceDigitalTwin>(`/api/v1/digitaltwin/${resourceId}`);
  }

  /**
   * Run simulation
   */
  async runSimulation(
    resourceId: string,
    scenario: {
      name: string;
      parameters: Record<string, number>;
      timeHorizon: number;
    }
  ): Promise<ApiResponse<ResourceDigitalTwin['simulation'][0]>> {
    return this.post<ResourceDigitalTwin['simulation'][0]>(
      `/api/v1/digitaltwin/${resourceId}/simulate`,
      scenario
    );
  }

  /**
   * Get optimization recommendations
   */
  async getOptimizations(
    resourceId: string,
    objective?: string
  ): Promise<ApiResponse<ResourceDigitalTwin['optimization']>> {
    const queryParams = objective ? `?objective=${objective}` : '';
    return this.get<ResourceDigitalTwin['optimization']>(
      `/api/v1/digitaltwin/${resourceId}/optimize${queryParams}`
    );
  }

  // ==========================================================================
  // HTTP Helper Methods
  // ==========================================================================

  private buildQueryParams(params?: Record<string, any>): string {
    if (!params) return '';
    const queryParams = new URLSearchParams();
    Object.entries(params).forEach(([key, value]) => {
      if (value !== undefined && value !== null) {
        queryParams.append(key, String(value));
      }
    });
    return queryParams.toString();
  }

  private async request<T>(
    method: string,
    path: string,
    body?: any
  ): Promise<ApiResponse<T>> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA-ENE-039] ${method} ${url}`, body || '');
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
        console.error('[WIA-ENE-039] Request failed:', error);
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
 * Calculate R/P ratio
 */
export function calculateRPRatio(reserves: number, annualProduction: number): number {
  if (annualProduction === 0) return Infinity;
  return reserves / annualProduction;
}

/**
 * Calculate depletion rate
 */
export function calculateDepletionRate(
  currentReserves: number,
  previousReserves: number
): number {
  if (previousReserves === 0) return 0;
  return ((previousReserves - currentReserves) / previousReserves) * 100;
}

/**
 * Calculate circularity rate
 */
export function calculateCircularityRate(
  recycledInput: number,
  reusedInput: number,
  totalConsumption: number
): number {
  if (totalConsumption === 0) return 0;
  return ((recycledInput + reusedInput) / totalConsumption) * 100;
}

/**
 * Estimate lifetime extension from strategy
 */
export function estimateLifetimeExtension(
  currentRPRatio: number,
  resourceSavedPerYear: number,
  annualProduction: number
): number {
  if (annualProduction === 0) return 0;
  return resourceSavedPerYear / annualProduction;
}

/**
 * Calculate supply concentration (HHI index)
 */
export function calculateHHI(marketShares: number[]): number {
  return marketShares.reduce((sum, share) => sum + share * share, 0);
}

/**
 * Assess depletion risk level
 */
export function assessRiskLevel(rpRatio: number): DepletionRiskLevel {
  if (rpRatio > 100) return DepletionRiskLevel.SAFE;
  if (rpRatio > 50) return DepletionRiskLevel.CAUTION;
  if (rpRatio > 25) return DepletionRiskLevel.WARNING;
  if (rpRatio > 10) return DepletionRiskLevel.DANGER;
  return DepletionRiskLevel.CRITICAL;
}

/**
 * Calculate resource productivity
 */
export function calculateResourceProductivity(gdp: number, resourceConsumption: number): number {
  if (resourceConsumption === 0) return 0;
  return gdp / resourceConsumption;
}

/**
 * Calculate material intensity
 */
export function calculateMaterialIntensity(resourceConsumption: number, gdp: number): number {
  if (gdp === 0) return 0;
  return resourceConsumption / gdp;
}

/**
 * Validate resource category code
 */
export function isValidResourceCategory(code: string): code is ResourceCategoryCode {
  return Object.values(ResourceCategoryCode).includes(code as ResourceCategoryCode);
}

/**
 * Format R/P ratio for display
 */
export function formatRPRatio(rpRatio: number): string {
  if (rpRatio === Infinity) return '∞ (infinite)';
  if (rpRatio > 1000) return '1000+ years';
  return `${Math.round(rpRatio)} years`;
}

// ============================================================================
// Default Export
// ============================================================================

export default ResourceDepletionSDK;
