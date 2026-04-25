/**
 * WIA-CITY-016: Urban Planning Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import type {
  LandUse,
  ZoningArea,
  ZoningType,
  BuildingRegulation,
  GreenSpace,
  RoadNetwork,
  PublicTransit,
  TransitStation,
  WaterSupply,
  Sewerage,
  PowerInfrastructure,
  PopulationData,
  UrbanPlan,
  GrowthSimulationParams,
  SimulationResult,
  TrafficSimulationParams,
  TrafficSimulationResult,
  ZoningCompatibilityAnalysis,
  AccessibilityAnalysis,
  DensityAnalysis,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  FilterParams,
  DateRangeFilter,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface UrbanPlanningSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;                     // ms (default: 30000)
  retryAttempts?: number;               // default: 3
  retryDelay?: number;                  // ms (default: 1000)
}

// ============================================================================
// Main SDK Class
// ============================================================================

export class UrbanPlanningSDK {
  private config: Required<UrbanPlanningSDKConfig>;
  private baseUrl: string;

  // Sub-modules
  public landUse: LandUseAPI;
  public zoning: ZoningAPI;
  public greenSpace: GreenSpaceAPI;
  public infrastructure: InfrastructureAPI;
  public population: PopulationAPI;
  public plans: PlansAPI;
  public simulation: SimulationAPI;
  public analysis: AnalysisAPI;

  constructor(config: UrbanPlanningSDKConfig) {
    this.config = {
      ...config,
      timeout: config.timeout || 30000,
      retryAttempts: config.retryAttempts || 3,
      retryDelay: config.retryDelay || 1000,
    };

    this.baseUrl = config.endpoint.replace(/\/$/, '');

    // Initialize sub-modules
    this.landUse = new LandUseAPI(this);
    this.zoning = new ZoningAPI(this);
    this.greenSpace = new GreenSpaceAPI(this);
    this.infrastructure = new InfrastructureAPI(this);
    this.population = new PopulationAPI(this);
    this.plans = new PlansAPI(this);
    this.simulation = new SimulationAPI(this);
    this.analysis = new AnalysisAPI(this);
  }

  /**
   * Make HTTP request with retry logic
   */
  async request<T>(
    method: 'GET' | 'POST' | 'PUT' | 'DELETE',
    path: string,
    data?: any,
    params?: Record<string, any>
  ): Promise<ApiResponse<T>> {
    const url = new URL(`${this.baseUrl}${path}`);

    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined && value !== null) {
          url.searchParams.append(key, String(value));
        }
      });
    }

    let lastError: Error | null = null;

    for (let attempt = 0; attempt < this.config.retryAttempts; attempt++) {
      try {
        const options: RequestInit = {
          method,
          headers: {
            'Authorization': `Bearer ${this.config.apiKey}`,
            'Content-Type': 'application/json',
          },
          signal: AbortSignal.timeout(this.config.timeout),
        };

        if (data) {
          options.body = JSON.stringify(data);
        }

        const response = await fetch(url.toString(), options);

        if (!response.ok) {
          throw new Error(`HTTP ${response.status}: ${response.statusText}`);
        }

        const result = await response.json();
        return result;
      } catch (error) {
        lastError = error as Error;
        if (attempt < this.config.retryAttempts - 1) {
          await new Promise((resolve) =>
            setTimeout(resolve, this.config.retryDelay * (attempt + 1))
          );
        }
      }
    }

    return {
      success: false,
      error: {
        code: 'REQUEST_FAILED',
        message: lastError?.message || 'Unknown error',
      },
    };
  }
}

// ============================================================================
// Land Use API
// ============================================================================

class LandUseAPI {
  constructor(private sdk: UrbanPlanningSDK) {}

  /**
   * Get land use data for a parcel
   */
  async get(parcelId: string): Promise<ApiResponse<LandUse>> {
    return this.sdk.request<LandUse>('GET', `/api/v1/land-use/${parcelId}`);
  }

  /**
   * List land use parcels
   */
  async list(
    params?: PaginationParams & FilterParams
  ): Promise<ApiResponse<PaginatedResponse<LandUse>>> {
    return this.sdk.request<PaginatedResponse<LandUse>>(
      'GET',
      '/api/v1/land-use',
      undefined,
      params
    );
  }

  /**
   * Create new land use parcel
   */
  async create(landUse: Omit<LandUse, 'parcelId' | 'lastUpdated'>): Promise<ApiResponse<LandUse>> {
    return this.sdk.request<LandUse>('POST', '/api/v1/land-use', landUse);
  }

  /**
   * Update land use parcel
   */
  async update(parcelId: string, updates: Partial<LandUse>): Promise<ApiResponse<LandUse>> {
    return this.sdk.request<LandUse>('PUT', `/api/v1/land-use/${parcelId}`, updates);
  }

  /**
   * Delete land use parcel
   */
  async delete(parcelId: string): Promise<ApiResponse<void>> {
    return this.sdk.request<void>('DELETE', `/api/v1/land-use/${parcelId}`);
  }

  /**
   * Analyze land use distribution
   */
  async analyze(params: {
    area?: string;
    district?: string;
  }): Promise<ApiResponse<{
    totalArea: number;
    distribution: Record<string, number>;
    averageFAR: number;
    averageBCR: number;
  }>> {
    return this.sdk.request('POST', '/api/v1/land-use/analyze', params);
  }
}

// ============================================================================
// Zoning API
// ============================================================================

class ZoningAPI {
  constructor(private sdk: UrbanPlanningSDK) {}

  /**
   * Get zoning area
   */
  async get(zoneId: string): Promise<ApiResponse<ZoningArea>> {
    return this.sdk.request<ZoningArea>('GET', `/api/v1/zoning/${zoneId}`);
  }

  /**
   * List zoning areas
   */
  async list(params?: PaginationParams): Promise<ApiResponse<PaginatedResponse<ZoningArea>>> {
    return this.sdk.request<PaginatedResponse<ZoningArea>>('GET', '/api/v1/zoning', undefined, params);
  }

  /**
   * Create new zoning area
   */
  async create(zone: Omit<ZoningArea, 'zoneId'>): Promise<ApiResponse<ZoningArea>> {
    return this.sdk.request<ZoningArea>('POST', '/api/v1/zoning', zone);
  }

  /**
   * Update zoning area
   */
  async update(zoneId: string, updates: Partial<ZoningArea>): Promise<ApiResponse<ZoningArea>> {
    return this.sdk.request<ZoningArea>('PUT', `/api/v1/zoning/${zoneId}`, updates);
  }

  /**
   * Check zoning compatibility
   */
  async checkCompatibility(
    zone1: ZoningType,
    zone2: ZoningType
  ): Promise<ApiResponse<ZoningCompatibilityAnalysis>> {
    return this.sdk.request<ZoningCompatibilityAnalysis>('POST', '/api/v1/zoning/compatibility', {
      zone1,
      zone2,
    });
  }

  /**
   * Calculate development capacity
   */
  async calculateCapacity(zoneId: string): Promise<ApiResponse<{
    currentFAR: number;
    allowedFAR: number;
    remainingCapacity: number;
    potentialUnits: number;
    potentialPopulation: number;
  }>> {
    return this.sdk.request('POST', `/api/v1/zoning/${zoneId}/capacity`);
  }
}

// ============================================================================
// Green Space API
// ============================================================================

class GreenSpaceAPI {
  constructor(private sdk: UrbanPlanningSDK) {}

  /**
   * Get green space
   */
  async get(greenSpaceId: string): Promise<ApiResponse<GreenSpace>> {
    return this.sdk.request<GreenSpace>('GET', `/api/v1/green-space/${greenSpaceId}`);
  }

  /**
   * List green spaces
   */
  async list(params?: PaginationParams): Promise<ApiResponse<PaginatedResponse<GreenSpace>>> {
    return this.sdk.request<PaginatedResponse<GreenSpace>>(
      'GET',
      '/api/v1/green-space',
      undefined,
      params
    );
  }

  /**
   * Create new green space
   */
  async create(greenSpace: Omit<GreenSpace, 'greenSpaceId'>): Promise<ApiResponse<GreenSpace>> {
    return this.sdk.request<GreenSpace>('POST', '/api/v1/green-space', greenSpace);
  }

  /**
   * Calculate green space per capita
   */
  async calculatePerCapita(params: {
    zoneId?: string;
    district?: string;
  }): Promise<ApiResponse<{
    totalGreenSpace: number;
    population: number;
    perCapita: number;
    target: number;
    gap: number;
  }>> {
    return this.sdk.request('POST', '/api/v1/green-space/per-capita', params);
  }

  /**
   * Analyze accessibility
   */
  async analyzeAccessibility(params: {
    zoneId: string;
    maxDistance?: number;
  }): Promise<ApiResponse<{
    nearestPark: string;
    distance: number;
    walkingTime: number;
    coverage: number;
  }>> {
    return this.sdk.request('POST', '/api/v1/green-space/accessibility', params);
  }
}

// ============================================================================
// Infrastructure API
// ============================================================================

class InfrastructureAPI {
  constructor(private sdk: UrbanPlanningSDK) {}

  /**
   * Get road network
   */
  async getRoadNetwork(roadId: string): Promise<ApiResponse<RoadNetwork>> {
    return this.sdk.request<RoadNetwork>('GET', `/api/v1/infrastructure/roads/${roadId}`);
  }

  /**
   * List road networks
   */
  async listRoads(params?: PaginationParams): Promise<ApiResponse<PaginatedResponse<RoadNetwork>>> {
    return this.sdk.request<PaginatedResponse<RoadNetwork>>(
      'GET',
      '/api/v1/infrastructure/roads',
      undefined,
      params
    );
  }

  /**
   * Get public transit
   */
  async getTransit(transitId: string): Promise<ApiResponse<PublicTransit>> {
    return this.sdk.request<PublicTransit>('GET', `/api/v1/infrastructure/transit/${transitId}`);
  }

  /**
   * List public transit
   */
  async listTransit(
    params?: PaginationParams
  ): Promise<ApiResponse<PaginatedResponse<PublicTransit>>> {
    return this.sdk.request<PaginatedResponse<PublicTransit>>(
      'GET',
      '/api/v1/infrastructure/transit',
      undefined,
      params
    );
  }

  /**
   * Plan infrastructure
   */
  async planInfrastructure(params: {
    planId: string;
    population: number;
    area: number;
    landUse: Record<string, number>;
  }): Promise<ApiResponse<{
    roads: { totalLength: number; byType: Record<string, number> };
    transit: { stations: number; routes: number };
    water: { capacity: number; facilities: number };
    sewerage: { capacity: number; facilities: number };
    power: { capacity: number; substations: number };
  }>> {
    return this.sdk.request('POST', '/api/v1/infrastructure/plan', params);
  }

  /**
   * Analyze capacity
   */
  async analyzeCapacity(params: {
    zoneId: string;
    infrastructureType: 'road' | 'transit' | 'water' | 'sewerage' | 'power';
  }): Promise<ApiResponse<{
    currentCapacity: number;
    currentDemand: number;
    utilizationRate: number;
    futureCapacity: number;
    futureGap: number;
  }>> {
    return this.sdk.request('POST', '/api/v1/infrastructure/capacity', params);
  }
}

// ============================================================================
// Population API
// ============================================================================

class PopulationAPI {
  constructor(private sdk: UrbanPlanningSDK) {}

  /**
   * Get population data
   */
  async get(zoneId: string): Promise<ApiResponse<PopulationData>> {
    return this.sdk.request<PopulationData>('GET', `/api/v1/population/${zoneId}`);
  }

  /**
   * List population data
   */
  async list(
    params?: PaginationParams
  ): Promise<ApiResponse<PaginatedResponse<PopulationData>>> {
    return this.sdk.request<PaginatedResponse<PopulationData>>(
      'GET',
      '/api/v1/population',
      undefined,
      params
    );
  }

  /**
   * Calculate density
   */
  async calculateDensity(params: {
    zoneId: string;
    type: 'gross' | 'net';
  }): Promise<ApiResponse<{
    population: number;
    area: number;
    density: number;
    households: number;
    householdDensity: number;
  }>> {
    return this.sdk.request('POST', '/api/v1/population/density', params);
  }

  /**
   * Project population
   */
  async project(params: {
    zoneId: string;
    targetYear: number;
    growthRate?: number;
  }): Promise<ApiResponse<{
    currentPopulation: number;
    projectedPopulation: number;
    growthRate: number;
    households: number;
  }>> {
    return this.sdk.request('POST', '/api/v1/population/project', params);
  }

  /**
   * Analyze demographics
   */
  async analyzeDemographics(zoneId: string): Promise<ApiResponse<{
    ageDistribution: Record<string, number>;
    dependencyRatio: number;
    agingIndex: number;
    medianAge: number;
  }>> {
    return this.sdk.request('POST', `/api/v1/population/${zoneId}/demographics`);
  }
}

// ============================================================================
// Plans API
// ============================================================================

class PlansAPI {
  constructor(private sdk: UrbanPlanningSDK) {}

  /**
   * Get urban plan
   */
  async get(planId: string): Promise<ApiResponse<UrbanPlan>> {
    return this.sdk.request<UrbanPlan>('GET', `/api/v1/plans/${planId}`);
  }

  /**
   * List urban plans
   */
  async list(params?: PaginationParams): Promise<ApiResponse<PaginatedResponse<UrbanPlan>>> {
    return this.sdk.request<PaginatedResponse<UrbanPlan>>(
      'GET',
      '/api/v1/plans',
      undefined,
      params
    );
  }

  /**
   * Create new plan
   */
  async create(plan: Omit<UrbanPlan, 'planId'>): Promise<ApiResponse<UrbanPlan>> {
    return this.sdk.request<UrbanPlan>('POST', '/api/v1/plans', plan);
  }

  /**
   * Update plan
   */
  async update(planId: string, updates: Partial<UrbanPlan>): Promise<ApiResponse<UrbanPlan>> {
    return this.sdk.request<UrbanPlan>('PUT', `/api/v1/plans/${planId}`, updates);
  }

  /**
   * Delete plan
   */
  async delete(planId: string): Promise<ApiResponse<void>> {
    return this.sdk.request<void>('DELETE', `/api/v1/plans/${planId}`);
  }

  /**
   * Get plan status
   */
  async getStatus(planId: string): Promise<ApiResponse<{
    status: string;
    progress: number;
    completedPhases: number;
    totalPhases: number;
    budget: {
      allocated: number;
      spent: number;
      remaining: number;
    };
  }>> {
    return this.sdk.request('GET', `/api/v1/plans/${planId}/status`);
  }
}

// ============================================================================
// Simulation API
// ============================================================================

class SimulationAPI {
  constructor(private sdk: UrbanPlanningSDK) {}

  /**
   * Run growth simulation
   */
  async simulateGrowth(params: GrowthSimulationParams): Promise<ApiResponse<SimulationResult>> {
    return this.sdk.request<SimulationResult>('POST', '/api/v1/simulation/growth', params);
  }

  /**
   * Get simulation result
   */
  async getSimulation(simulationId: string): Promise<ApiResponse<SimulationResult>> {
    return this.sdk.request<SimulationResult>('GET', `/api/v1/simulation/${simulationId}`);
  }

  /**
   * Run traffic simulation
   */
  async simulateTraffic(
    params: TrafficSimulationParams
  ): Promise<ApiResponse<TrafficSimulationResult>> {
    return this.sdk.request<TrafficSimulationResult>('POST', '/api/v1/simulation/traffic', params);
  }

  /**
   * Run environmental impact simulation
   */
  async simulateEnvironmentalImpact(params: {
    planId: string;
    scenario: string;
  }): Promise<ApiResponse<{
    airQuality: { pm25: number; pm10: number; no2: number };
    greenSpaceRatio: number;
    carbonEmissions: number;
    waterQuality: { bod: number; cod: number };
    noiseLevel: number;
  }>> {
    return this.sdk.request('POST', '/api/v1/simulation/environmental', params);
  }

  /**
   * Compare scenarios
   */
  async compareScenarios(params: {
    simulationIds: string[];
  }): Promise<ApiResponse<{
    comparison: Record<string, any>[];
    recommendations: string[];
  }>> {
    return this.sdk.request('POST', '/api/v1/simulation/compare', params);
  }
}

// ============================================================================
// Analysis API
// ============================================================================

class AnalysisAPI {
  constructor(private sdk: UrbanPlanningSDK) {}

  /**
   * Analyze accessibility
   */
  async analyzeAccessibility(zoneId: string): Promise<ApiResponse<AccessibilityAnalysis>> {
    return this.sdk.request<AccessibilityAnalysis>(
      'POST',
      `/api/v1/analysis/accessibility`,
      { zoneId }
    );
  }

  /**
   * Analyze density
   */
  async analyzeDensity(zoneId: string): Promise<ApiResponse<DensityAnalysis>> {
    return this.sdk.request<DensityAnalysis>('POST', `/api/v1/analysis/density`, { zoneId });
  }

  /**
   * Analyze land use efficiency
   */
  async analyzeLandUseEfficiency(params: {
    zoneId: string;
  }): Promise<ApiResponse<{
    utilizationRate: number;
    vacancyRate: number;
    underutilizedParcels: string[];
    recommendations: string[];
  }>> {
    return this.sdk.request('POST', '/api/v1/analysis/land-use-efficiency', params);
  }

  /**
   * Analyze sustainability
   */
  async analyzeSustainability(params: {
    planId: string;
  }): Promise<ApiResponse<{
    greenSpaceRatio: number;
    publicTransitShare: number;
    renewableEnergyRatio: number;
    wasteRecyclingRate: number;
    carbonFootprint: number;
    sustainabilityScore: number;
  }>> {
    return this.sdk.request('POST', '/api/v1/analysis/sustainability', params);
  }

  /**
   * Analyze equity
   */
  async analyzeEquity(params: {
    zoneIds: string[];
  }): Promise<ApiResponse<{
    amenityAccessGap: number;
    incomeInequality: number;
    spatialSegregation: number;
    recommendations: string[];
  }>> {
    return this.sdk.request('POST', '/api/v1/analysis/equity', params);
  }
}

// ============================================================================
// Export all types
// ============================================================================

export * from './types';
