/**
 * Zero Energy Building City Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import {
  ApiResponse,
  EnergyDistrict,
  DistrictId,
  BuildingId,
  ZEBBuilding,
  CityGrid,
  Microgrid,
  GridConnection,
  EnergyFlow,
  RenewableSource,
  SolarFarm,
  WindTurbine,
  GeothermalPlant,
  StorageSystem,
  BatteryBank,
  ThermalStorage,
  CityStorage,
  SmartMeter,
  DataHub,
  ControlCenter,
  CarbonFootprint,
  NetZeroScore,
  Certification,
  DemandForecast,
  ProductionForecast,
  OptimizationResult,
  DistrictEnergyProfile,
  BuildingEnergyProfile,
  ZEBCityError,
  DistrictNotFoundError,
  GridBalanceError,
  StorageCapacityError,
  CertificationError,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

/**
 * SDK configuration options
 */
export interface ZEBCitySDKConfig {
  apiKey: string;
  endpoint?: string;
  timeout?: number;
  retries?: number;
  enableEventEmitter?: boolean;
}

/**
 * Default configuration
 */
const DEFAULT_CONFIG = {
  endpoint: 'https://api.wia.org/zero-energy-city/v1',
  timeout: 30000,
  retries: 3,
  enableEventEmitter: true,
};

// ============================================================================
// Event Types
// ============================================================================

/**
 * SDK event types
 */
export enum ZEBCityEventType {
  DISTRICT_UPDATED = 'district:updated',
  GRID_BALANCED = 'grid:balanced',
  STORAGE_CHARGED = 'storage:charged',
  STORAGE_DISCHARGED = 'storage:discharged',
  DEMAND_PEAK = 'demand:peak',
  PRODUCTION_PEAK = 'production:peak',
  CERTIFICATION_AWARDED = 'certification:awarded',
  CARBON_MILESTONE = 'carbon:milestone',
  SYSTEM_ERROR = 'system:error',
}

/**
 * Event payload
 */
export interface ZEBCityEvent {
  type: ZEBCityEventType;
  timestamp: string;
  data: any;
}

/**
 * Event listener callback
 */
export type EventListener = (event: ZEBCityEvent) => void;

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA Zero Energy Building City SDK
 *
 * Comprehensive SDK for managing city-wide net-zero energy districts,
 * buildings, renewable sources, energy storage, and smart grid integration.
 */
export class WIAZeroEnergyBuildingCitySDK {
  private config: Required<ZEBCitySDKConfig>;
  private eventListeners: Map<ZEBCityEventType, Set<EventListener>> = new Map();

  constructor(config: ZEBCitySDKConfig) {
    this.config = {
      ...DEFAULT_CONFIG,
      ...config,
    };

    // Initialize event listener maps
    if (this.config.enableEventEmitter) {
      Object.values(ZEBCityEventType).forEach(eventType => {
        this.eventListeners.set(eventType, new Set());
      });
    }
  }

  // ========================================================================
  // Event Emitter Pattern
  // ========================================================================

  /**
   * Register event listener
   */
  on(eventType: ZEBCityEventType, listener: EventListener): void {
    if (!this.config.enableEventEmitter) {
      throw new ZEBCityError('Event emitter is disabled', 'EVENT_EMITTER_DISABLED');
    }

    const listeners = this.eventListeners.get(eventType);
    if (listeners) {
      listeners.add(listener);
    }
  }

  /**
   * Remove event listener
   */
  off(eventType: ZEBCityEventType, listener: EventListener): void {
    const listeners = this.eventListeners.get(eventType);
    if (listeners) {
      listeners.delete(listener);
    }
  }

  /**
   * Emit event to all listeners
   */
  private emit(event: ZEBCityEvent): void {
    if (!this.config.enableEventEmitter) return;

    const listeners = this.eventListeners.get(event.type);
    if (listeners) {
      listeners.forEach(listener => {
        try {
          listener(event);
        } catch (error) {
          console.error('Event listener error:', error);
        }
      });
    }
  }

  // ========================================================================
  // HTTP Request Handler
  // ========================================================================

  /**
   * Make HTTP request with retry logic
   */
  private async request<T>(
    method: string,
    path: string,
    data?: any,
    retryCount = 0
  ): Promise<ApiResponse<T>> {
    const url = `${this.config.endpoint}${path}`;
    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-SDK-Version': '1.0.0',
    };

    try {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

      const response = await fetch(url, {
        method,
        headers,
        body: data ? JSON.stringify(data) : undefined,
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      const result = await response.json();

      if (!response.ok && retryCount < this.config.retries) {
        // Retry on server errors
        if (response.status >= 500) {
          await new Promise(resolve => setTimeout(resolve, 1000 * (retryCount + 1)));
          return this.request<T>(method, path, data, retryCount + 1);
        }
      }

      return result as ApiResponse<T>;
    } catch (error: any) {
      if (retryCount < this.config.retries) {
        await new Promise(resolve => setTimeout(resolve, 1000 * (retryCount + 1)));
        return this.request<T>(method, path, data, retryCount + 1);
      }

      this.emit({
        type: ZEBCityEventType.SYSTEM_ERROR,
        timestamp: new Date().toISOString(),
        data: { error: error.message, path },
      });

      return {
        success: false,
        error: {
          code: 'REQUEST_FAILED',
          message: error.message,
          details: error,
        },
      };
    }
  }

  // ========================================================================
  // District Energy Management
  // ========================================================================

  /**
   * Get district energy profile and real-time metrics
   */
  async getDistrictEnergy(districtId: DistrictId): Promise<ApiResponse<{
    district: EnergyDistrict;
    profile: DistrictEnergyProfile;
    realtime: {
      current_demand_kW: number;
      current_production_kW: number;
      current_storage_kWh: number;
      grid_flow_kW: number;
    };
  }>> {
    const response = await this.request<any>('GET', `/districts/${districtId}/energy`);

    if (response.success) {
      this.emit({
        type: ZEBCityEventType.DISTRICT_UPDATED,
        timestamp: new Date().toISOString(),
        data: { districtId, ...response.data },
      });
    }

    return response;
  }

  /**
   * Get all districts
   */
  async listDistricts(filters?: {
    class?: string;
    status?: string;
    certified?: boolean;
  }): Promise<ApiResponse<EnergyDistrict[]>> {
    const params = new URLSearchParams(filters as any);
    return this.request<EnergyDistrict[]>('GET', `/districts?${params}`);
  }

  /**
   * Register new energy district
   */
  async registerDistrict(
    district: Omit<EnergyDistrict, 'districtId' | 'created_at' | 'updated_at'>
  ): Promise<ApiResponse<EnergyDistrict>> {
    return this.request<EnergyDistrict>('POST', '/districts', district);
  }

  /**
   * Update district configuration
   */
  async updateDistrict(
    districtId: DistrictId,
    updates: Partial<EnergyDistrict>
  ): Promise<ApiResponse<EnergyDistrict>> {
    return this.request<EnergyDistrict>('PUT', `/districts/${districtId}`, updates);
  }

  // ========================================================================
  // Grid Balancing & Optimization
  // ========================================================================

  /**
   * Balance energy grid in real-time
   *
   * Automatically adjusts energy flows between production, consumption,
   * storage, and grid import/export to maintain grid stability.
   */
  async balanceGrid(districtId: DistrictId, options?: {
    priority?: 'COST' | 'RELIABILITY' | 'SUSTAINABILITY';
    max_grid_import_kW?: number;
    target_soc_percent?: number;
  }): Promise<ApiResponse<{
    balanced: boolean;
    actions_taken: Array<{
      action: string;
      asset_id: string;
      value_kW: number;
    }>;
    grid_status: {
      frequency_Hz: number;
      voltage_stability: number;
      load_balance_percent: number;
    };
  }>> {
    const response = await this.request<any>('POST', `/districts/${districtId}/balance`, options);

    if (response.success && response.data?.balanced) {
      this.emit({
        type: ZEBCityEventType.GRID_BALANCED,
        timestamp: new Date().toISOString(),
        data: { districtId, ...response.data },
      });
    }

    return response;
  }

  /**
   * Optimize energy flow across district
   *
   * Uses predictive algorithms to optimize energy routing for cost,
   * emissions, or reliability.
   */
  async optimizeFlow(
    districtId: DistrictId,
    horizon_hours: number = 24,
    objective: 'MINIMIZE_COST' | 'MINIMIZE_EMISSIONS' | 'MAXIMIZE_RELIABILITY' | 'BALANCED' = 'BALANCED'
  ): Promise<ApiResponse<OptimizationResult>> {
    return this.request<OptimizationResult>('POST', `/districts/${districtId}/optimize`, {
      horizon_hours,
      objective,
    });
  }

  /**
   * Get grid connection status
   */
  async getGridConnection(districtId: DistrictId): Promise<ApiResponse<GridConnection>> {
    return this.request<GridConnection>('GET', `/districts/${districtId}/grid-connection`);
  }

  /**
   * Get city-wide grid status
   */
  async getCityGrid(city: string): Promise<ApiResponse<CityGrid>> {
    return this.request<CityGrid>('GET', `/cities/${city}/grid`);
  }

  /**
   * Get microgrid details
   */
  async getMicrogrid(microgridId: string): Promise<ApiResponse<Microgrid>> {
    return this.request<Microgrid>('GET', `/microgrids/${microgridId}`);
  }

  // ========================================================================
  // Demand Forecasting
  // ========================================================================

  /**
   * Forecast energy demand
   *
   * Predicts future energy demand using ML models based on historical
   * data, weather forecasts, and occupancy patterns.
   */
  async forecastDemand(
    districtId: DistrictId,
    horizon_hours: number = 24
  ): Promise<ApiResponse<DemandForecast>> {
    const response = await this.request<DemandForecast>('POST', `/districts/${districtId}/forecast/demand`, {
      horizon_hours,
    });

    if (response.success && response.data) {
      // Check for peak demand events
      const forecast = response.data;
      const maxDemand = Math.max(...forecast.hourly_forecast_kW);
      const avgDemand = forecast.hourly_forecast_kW.reduce((a, b) => a + b, 0) / forecast.hourly_forecast_kW.length;

      if (maxDemand > avgDemand * 1.5) {
        this.emit({
          type: ZEBCityEventType.DEMAND_PEAK,
          timestamp: new Date().toISOString(),
          data: { districtId, peak_kW: maxDemand, average_kW: avgDemand },
        });
      }
    }

    return response;
  }

  /**
   * Forecast renewable energy production
   */
  async forecastProduction(
    sourceId: string,
    horizon_hours: number = 24
  ): Promise<ApiResponse<ProductionForecast>> {
    const response = await this.request<ProductionForecast>('POST', `/sources/${sourceId}/forecast`, {
      horizon_hours,
    });

    if (response.success && response.data) {
      const forecast = response.data;
      const maxProduction = Math.max(...forecast.hourly_forecast_kW);
      const avgProduction = forecast.hourly_forecast_kW.reduce((a, b) => a + b, 0) / forecast.hourly_forecast_kW.length;

      if (maxProduction > avgProduction * 1.8) {
        this.emit({
          type: ZEBCityEventType.PRODUCTION_PEAK,
          timestamp: new Date().toISOString(),
          data: { sourceId, peak_kW: maxProduction, average_kW: avgProduction },
        });
      }
    }

    return response;
  }

  // ========================================================================
  // Storage Management
  // ========================================================================

  /**
   * Manage energy storage systems
   *
   * Controls charging/discharging of batteries and thermal storage
   * based on grid conditions and optimization objectives.
   */
  async manageStorage(
    districtId: DistrictId,
    command: {
      action: 'CHARGE' | 'DISCHARGE' | 'IDLE' | 'OPTIMIZE';
      target_power_kW?: number;
      target_soc_percent?: number;
      duration_hours?: number;
    }
  ): Promise<ApiResponse<{
    executed: boolean;
    storage_systems: Array<{
      storageId: string;
      action: string;
      power_kW: number;
      new_soc_percent: number;
    }>;
  }>> {
    const response = await this.request<any>('POST', `/districts/${districtId}/storage/manage`, command);

    if (response.success && response.data?.executed) {
      // Emit storage events
      response.data.storage_systems.forEach((system: any) => {
        if (system.action === 'CHARGE') {
          this.emit({
            type: ZEBCityEventType.STORAGE_CHARGED,
            timestamp: new Date().toISOString(),
            data: { districtId, ...system },
          });
        } else if (system.action === 'DISCHARGE') {
          this.emit({
            type: ZEBCityEventType.STORAGE_DISCHARGED,
            timestamp: new Date().toISOString(),
            data: { districtId, ...system },
          });
        }
      });
    }

    return response;
  }

  /**
   * Get city-wide storage status
   */
  async getCityStorage(districtId: DistrictId): Promise<ApiResponse<CityStorage>> {
    return this.request<CityStorage>('GET', `/districts/${districtId}/storage`);
  }

  /**
   * Get battery bank details
   */
  async getBatteryBank(batteryId: string): Promise<ApiResponse<BatteryBank>> {
    return this.request<BatteryBank>('GET', `/storage/battery/${batteryId}`);
  }

  /**
   * Get thermal storage details
   */
  async getThermalStorage(storageId: string): Promise<ApiResponse<ThermalStorage>> {
    return this.request<ThermalStorage>('GET', `/storage/thermal/${storageId}`);
  }

  // ========================================================================
  // Carbon Tracking & Sustainability
  // ========================================================================

  /**
   * Track carbon footprint
   *
   * Calculates and tracks CO2 emissions for districts and buildings,
   * including offsets from renewable energy.
   */
  async trackCarbon(
    entityId: string,
    entity_type: 'DISTRICT' | 'BUILDING',
    period?: {
      start_date: string;
      end_date: string;
    }
  ): Promise<ApiResponse<CarbonFootprint>> {
    const response = await this.request<CarbonFootprint>('POST', `/carbon/track`, {
      entityId,
      entity_type,
      period,
    });

    if (response.success && response.data) {
      const footprint = response.data;
      // Check for carbon neutrality milestone
      if (footprint.net_emissions_kg_CO2e <= 0) {
        this.emit({
          type: ZEBCityEventType.CARBON_MILESTONE,
          timestamp: new Date().toISOString(),
          data: {
            entityId,
            entity_type,
            milestone: 'CARBON_NEUTRAL',
            net_emissions: footprint.net_emissions_kg_CO2e,
          },
        });
      }
    }

    return response;
  }

  /**
   * Get net-zero energy score
   */
  async getNetZeroScore(
    entityId: string,
    entity_type: 'DISTRICT' | 'BUILDING'
  ): Promise<ApiResponse<NetZeroScore>> {
    return this.request<NetZeroScore>('GET', `/netzero/score/${entity_type}/${entityId}`);
  }

  /**
   * Calculate carbon offset credits
   */
  async calculateCarbonOffsets(
    districtId: DistrictId,
    period_months: number = 12
  ): Promise<ApiResponse<{
    total_offset_kg_CO2e: number;
    carbon_credits_eligible: number;
    market_value_USD: number;
  }>> {
    return this.request<any>('POST', `/districts/${districtId}/carbon/offsets`, {
      period_months,
    });
  }

  // ========================================================================
  // City Metrics & Analytics
  // ========================================================================

  /**
   * Get comprehensive city metrics
   *
   * Returns aggregate metrics across all districts including energy,
   * carbon, financial, and operational data.
   */
  async getCityMetrics(city: string, period?: {
    start_date: string;
    end_date: string;
  }): Promise<ApiResponse<{
    city: string;
    period: { start: string; end: string };
    districts_count: number;
    buildings_count: number;
    population: number;
    energy: {
      total_consumption_kWh: number;
      total_production_kWh: number;
      net_balance_kWh: number;
      renewable_percent: number;
      self_sufficiency_percent: number;
    };
    carbon: {
      total_emissions_kg_CO2e: number;
      emissions_per_capita_kg_CO2e: number;
      net_zero_districts_count: number;
      carbon_neutral: boolean;
    };
    financial: {
      energy_cost_savings_USD: number;
      carbon_credit_value_USD: number;
      roi_percent: number;
    };
    grid: {
      uptime_percent: number;
      reliability_score: number;
      peak_demand_kW: number;
      peak_production_kW: number;
    };
  }>> {
    return this.request<any>('GET', `/cities/${city}/metrics`, period);
  }

  /**
   * Get district performance comparison
   */
  async compareDistricts(districtIds: DistrictId[]): Promise<ApiResponse<Array<{
    districtId: string;
    name: string;
    net_zero_score: number;
    carbon_intensity: number;
    self_sufficiency: number;
    ranking: number;
  }>>> {
    return this.request<any>('POST', '/districts/compare', { districtIds });
  }

  // ========================================================================
  // Certification & Compliance
  // ========================================================================

  /**
   * Certify district as net-zero energy
   *
   * Initiates certification process for district or building to achieve
   * official net-zero energy designation.
   */
  async certifyDistrict(
    entityId: string,
    entity_type: 'DISTRICT' | 'BUILDING',
    standard: string,
    verification_data: {
      verification_period_years: number;
      verification_method: 'MEASURED' | 'MODELED' | 'HYBRID';
      supporting_documents: string[];
    }
  ): Promise<ApiResponse<Certification>> {
    const response = await this.request<Certification>('POST', '/certification/apply', {
      entityId,
      entity_type,
      standard,
      verification_data,
    });

    if (response.success && response.data?.status === 'CERTIFIED') {
      this.emit({
        type: ZEBCityEventType.CERTIFICATION_AWARDED,
        timestamp: new Date().toISOString(),
        data: {
          entityId,
          entity_type,
          certification: response.data,
        },
      });
    }

    return response;
  }

  /**
   * Get certification status
   */
  async getCertification(certificationId: string): Promise<ApiResponse<Certification>> {
    return this.request<Certification>('GET', `/certification/${certificationId}`);
  }

  /**
   * List all certifications for an entity
   */
  async listCertifications(
    entityId: string,
    entity_type: 'DISTRICT' | 'BUILDING'
  ): Promise<ApiResponse<Certification[]>> {
    return this.request<Certification[]>('GET', `/certification/${entity_type}/${entityId}/list`);
  }

  // ========================================================================
  // Urban Planning & Expansion
  // ========================================================================

  /**
   * Plan district expansion
   *
   * Simulates and plans expansion of zero-energy districts including
   * new buildings, renewable sources, and grid infrastructure.
   */
  async planExpansion(
    districtId: DistrictId,
    expansion_plan: {
      new_buildings: number;
      additional_area_km2: number;
      target_population_increase: number;
      renewable_capacity_increase_MW: number;
      storage_capacity_increase_MWh: number;
      timeline_years: number;
    }
  ): Promise<ApiResponse<{
    feasible: boolean;
    total_investment_USD: number;
    roi_years: number;
    projected_metrics: {
      total_capacity_MW: number;
      total_storage_MWh: number;
      self_sufficiency_percent: number;
      carbon_reduction_kg_CO2e_per_year: number;
    };
    recommendations: Array<{
      category: string;
      recommendation: string;
      priority: 'HIGH' | 'MEDIUM' | 'LOW';
    }>;
    timeline: Array<{
      phase: string;
      duration_months: number;
      deliverables: string[];
      cost_USD: number;
    }>;
  }>> {
    return this.request<any>('POST', `/districts/${districtId}/plan-expansion`, expansion_plan);
  }

  /**
   * Simulate new district
   */
  async simulateDistrict(simulation: {
    location: { lat: number; lon: number; city: string };
    area_km2: number;
    population: number;
    building_mix: Record<string, number>;
    renewable_mix: Record<string, number>;
  }): Promise<ApiResponse<{
    feasibility_score: number;
    estimated_cost_USD: number;
    payback_period_years: number;
    energy_metrics: any;
    environmental_impact: any;
  }>> {
    return this.request<any>('POST', '/districts/simulate', simulation);
  }

  // ========================================================================
  // Building Management
  // ========================================================================

  /**
   * Get building details
   */
  async getBuilding(buildingId: BuildingId): Promise<ApiResponse<ZEBBuilding>> {
    return this.request<ZEBBuilding>('GET', `/buildings/${buildingId}`);
  }

  /**
   * List buildings in district
   */
  async listBuildings(districtId: DistrictId, filters?: {
    class?: string;
    building_type?: string;
  }): Promise<ApiResponse<ZEBBuilding[]>> {
    const params = new URLSearchParams({ districtId, ...filters } as any);
    return this.request<ZEBBuilding[]>('GET', `/buildings?${params}`);
  }

  /**
   * Register new ZEB building
   */
  async registerBuilding(
    building: Omit<ZEBBuilding, 'buildingId'>
  ): Promise<ApiResponse<ZEBBuilding>> {
    return this.request<ZEBBuilding>('POST', '/buildings', building);
  }

  // ========================================================================
  // Renewable Energy Sources
  // ========================================================================

  /**
   * Get renewable source details
   */
  async getRenewableSource(sourceId: string): Promise<ApiResponse<RenewableSource>> {
    return this.request<RenewableSource>('GET', `/sources/${sourceId}`);
  }

  /**
   * List renewable sources in district
   */
  async listRenewableSources(districtId: DistrictId, type?: string): Promise<ApiResponse<RenewableSource[]>> {
    const params = new URLSearchParams({ districtId, type } as any);
    return this.request<RenewableSource[]>('GET', `/sources?${params}`);
  }

  /**
   * Register solar farm
   */
  async registerSolarFarm(
    solarFarm: Omit<SolarFarm, 'sourceId'>
  ): Promise<ApiResponse<SolarFarm>> {
    return this.request<SolarFarm>('POST', '/sources/solar', solarFarm);
  }

  /**
   * Register wind turbine
   */
  async registerWindTurbine(
    windTurbine: Omit<WindTurbine, 'sourceId'>
  ): Promise<ApiResponse<WindTurbine>> {
    return this.request<WindTurbine>('POST', '/sources/wind', windTurbine);
  }

  /**
   * Register geothermal plant
   */
  async registerGeothermalPlant(
    geothermalPlant: Omit<GeothermalPlant, 'sourceId'>
  ): Promise<ApiResponse<GeothermalPlant>> {
    return this.request<GeothermalPlant>('POST', '/sources/geothermal', geothermalPlant);
  }

  // ========================================================================
  // Smart City Integration
  // ========================================================================

  /**
   * Get data hub status
   */
  async getDataHub(districtId: DistrictId): Promise<ApiResponse<DataHub>> {
    return this.request<DataHub>('GET', `/districts/${districtId}/data-hub`);
  }

  /**
   * Get control center status
   */
  async getControlCenter(city: string): Promise<ApiResponse<ControlCenter>> {
    return this.request<ControlCenter>('GET', `/cities/${city}/control-center`);
  }

  /**
   * Get smart meter data
   */
  async getSmartMeterData(
    meterId: string,
    period?: { start_date: string; end_date: string }
  ): Promise<ApiResponse<{
    meter: SmartMeter;
    readings: Array<{
      timestamp: string;
      power_kW: number;
      energy_kWh: number;
    }>;
  }>> {
    return this.request<any>('GET', `/meters/${meterId}/data`, period);
  }

  // ========================================================================
  // Utilities & Helpers
  // ========================================================================

  /**
   * Validate district configuration
   */
  async validateDistrict(districtId: DistrictId): Promise<ApiResponse<{
    valid: boolean;
    issues: Array<{
      severity: 'ERROR' | 'WARNING' | 'INFO';
      category: string;
      message: string;
    }>;
    recommendations: string[];
  }>> {
    return this.request<any>('POST', `/districts/${districtId}/validate`);
  }

  /**
   * Get health check
   */
  async healthCheck(): Promise<ApiResponse<{
    status: 'HEALTHY' | 'DEGRADED' | 'DOWN';
    version: string;
    uptime_seconds: number;
  }>> {
    return this.request<any>('GET', '/health');
  }
}

// ============================================================================
// Factory Function
// ============================================================================

/**
 * Create WIA Zero Energy Building City SDK instance
 *
 * @param config - SDK configuration
 * @returns SDK instance
 *
 * @example
 * ```typescript
 * const sdk = createZEBCitySDK({
 *   apiKey: 'your-api-key',
 *   endpoint: 'https://api.wia.org/zero-energy-city/v1'
 * });
 *
 * // Get district energy
 * const energy = await sdk.getDistrictEnergy('district-123');
 *
 * // Balance grid
 * const result = await sdk.balanceGrid('district-123', {
 *   priority: 'SUSTAINABILITY'
 * });
 * ```
 */
export function createZEBCitySDK(config: ZEBCitySDKConfig): WIAZeroEnergyBuildingCitySDK {
  return new WIAZeroEnergyBuildingCitySDK(config);
}

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Calculate energy self-sufficiency percentage
 */
export function calculateSelfSufficiency(
  production_kWh: number,
  consumption_kWh: number
): number {
  if (consumption_kWh === 0) return 100;
  return Math.min((production_kWh / consumption_kWh) * 100, 100);
}

/**
 * Calculate net-zero energy balance
 */
export function calculateNetZeroBalance(
  production_kWh: number,
  consumption_kWh: number
): number {
  return production_kWh - consumption_kWh;
}

/**
 * Determine if entity achieves net-zero
 */
export function isNetZero(
  annual_production_kWh: number,
  annual_consumption_kWh: number,
  tolerance_percent: number = 5
): boolean {
  const balance = annual_production_kWh - annual_consumption_kWh;
  const tolerance_kWh = annual_consumption_kWh * (tolerance_percent / 100);
  return balance >= -tolerance_kWh;
}

/**
 * Calculate carbon intensity
 */
export function calculateCarbonIntensity(
  emissions_kg_CO2e: number,
  energy_kWh: number
): number {
  if (energy_kWh === 0) return 0;
  return emissions_kg_CO2e / energy_kWh;
}

/**
 * Calculate storage round-trip efficiency
 */
export function calculateStorageEfficiency(
  energy_out_kWh: number,
  energy_in_kWh: number
): number {
  if (energy_in_kWh === 0) return 0;
  return (energy_out_kWh / energy_in_kWh) * 100;
}

/**
 * Format energy value with appropriate unit
 */
export function formatEnergy(kWh: number): string {
  if (kWh >= 1000000) {
    return `${(kWh / 1000000).toFixed(2)} GWh`;
  } else if (kWh >= 1000) {
    return `${(kWh / 1000).toFixed(2)} MWh`;
  } else {
    return `${kWh.toFixed(2)} kWh`;
  }
}

/**
 * Format power value with appropriate unit
 */
export function formatPower(kW: number): string {
  if (kW >= 1000) {
    return `${(kW / 1000).toFixed(2)} MW`;
  } else {
    return `${kW.toFixed(2)} kW`;
  }
}

// ============================================================================
// Export default SDK class
// ============================================================================

export default WIAZeroEnergyBuildingCitySDK;
