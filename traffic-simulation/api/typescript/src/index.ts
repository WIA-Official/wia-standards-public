/**
 * WIA-CITY-017: Traffic Simulation Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import type {
  RoadLink,
  TrafficNode,
  TrafficState,
  TrafficSignal,
  SignalTimingPlan,
  Vehicle,
  VehicleCharacteristics,
  Pedestrian,
  TransitRoute,
  TransitVehicle,
  TrafficIncident,
  IncidentImpact,
  SimulationConfig,
  SimulationState,
  SimulationResults,
  TrafficForecast,
  CongestionData,
  ODMatrix,
  TrafficZone,
  Detector,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface TrafficSimulationSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;                     // ms (default: 60000 for long simulations)
  retryAttempts?: number;               // default: 3
  retryDelay?: number;                  // ms (default: 1000)
}

// ============================================================================
// Main SDK Class
// ============================================================================

export class TrafficSimulationSDK {
  private config: Required<TrafficSimulationSDKConfig>;
  private baseUrl: string;

  // Sub-modules
  public network: NetworkAPI;
  public simulation: SimulationAPI;
  public traffic: TrafficAPI;
  public signals: SignalsAPI;
  public vehicles: VehiclesAPI;
  public transit: TransitAPI;
  public incidents: IncidentsAPI;
  public forecasting: ForecastingAPI;
  public congestion: CongestionAPI;
  public od: OriginDestinationAPI;

  constructor(config: TrafficSimulationSDKConfig) {
    this.config = {
      ...config,
      timeout: config.timeout || 60000,
      retryAttempts: config.retryAttempts || 3,
      retryDelay: config.retryDelay || 1000,
    };

    this.baseUrl = config.endpoint.replace(/\/$/, '');

    // Initialize sub-modules
    this.network = new NetworkAPI(this);
    this.simulation = new SimulationAPI(this);
    this.traffic = new TrafficAPI(this);
    this.signals = new SignalsAPI(this);
    this.vehicles = new VehiclesAPI(this);
    this.transit = new TransitAPI(this);
    this.incidents = new IncidentsAPI(this);
    this.forecasting = new ForecastingAPI(this);
    this.congestion = new CongestionAPI(this);
    this.od = new OriginDestinationAPI(this);
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

    const headers: Record<string, string> = {
      'Authorization': `Bearer ${this.config.apiKey}`,
      'Content-Type': 'application/json',
      'User-Agent': 'WIA-CITY-017-SDK/1.0.0',
    };

    let lastError: Error | null = null;

    for (let attempt = 0; attempt < this.config.retryAttempts; attempt++) {
      try {
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

        const response = await fetch(url.toString(), {
          method,
          headers,
          body: data ? JSON.stringify(data) : undefined,
          signal: controller.signal,
        });

        clearTimeout(timeoutId);

        const result = await response.json();

        if (!response.ok) {
          throw new Error(result.error?.message || `HTTP ${response.status}`);
        }

        return result;
      } catch (error) {
        lastError = error as Error;

        // Don't retry on client errors (4xx)
        if (error instanceof Error && error.message.includes('4')) {
          break;
        }

        // Wait before retrying
        if (attempt < this.config.retryAttempts - 1) {
          await new Promise((resolve) =>
            setTimeout(resolve, this.config.retryDelay * (attempt + 1))
          );
        }
      }
    }

    throw lastError || new Error('Request failed');
  }
}

// ============================================================================
// Network API
// ============================================================================

class NetworkAPI {
  constructor(private sdk: TrafficSimulationSDK) {}

  /**
   * Get road network
   */
  async getNetwork(networkId: string): Promise<ApiResponse<{
    links: RoadLink[];
    nodes: TrafficNode[];
  }>> {
    return this.sdk.request('GET', `/api/v1/network/${networkId}`);
  }

  /**
   * Get link by ID
   */
  async getLink(linkId: string): Promise<ApiResponse<RoadLink>> {
    return this.sdk.request('GET', `/api/v1/network/links/${linkId}`);
  }

  /**
   * Get node by ID
   */
  async getNode(nodeId: string): Promise<ApiResponse<TrafficNode>> {
    return this.sdk.request('GET', `/api/v1/network/nodes/${nodeId}`);
  }

  /**
   * List links
   */
  async listLinks(params?: {
    linkType?: string;
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<RoadLink>>> {
    return this.sdk.request('GET', '/api/v1/network/links', undefined, params);
  }

  /**
   * Create link
   */
  async createLink(link: Omit<RoadLink, 'linkId'>): Promise<ApiResponse<{ linkId: string }>> {
    return this.sdk.request('POST', '/api/v1/network/links', link);
  }

  /**
   * Update link
   */
  async updateLink(linkId: string, link: Partial<RoadLink>): Promise<ApiResponse<RoadLink>> {
    return this.sdk.request('PUT', `/api/v1/network/links/${linkId}`, link);
  }

  /**
   * Delete link
   */
  async deleteLink(linkId: string): Promise<ApiResponse<void>> {
    return this.sdk.request('DELETE', `/api/v1/network/links/${linkId}`);
  }
}

// ============================================================================
// Simulation API
// ============================================================================

class SimulationAPI {
  constructor(private sdk: TrafficSimulationSDK) {}

  /**
   * Create simulation
   */
  async create(config: Omit<SimulationConfig, 'simulationId'>): Promise<ApiResponse<{
    simulationId: string;
    status: string;
  }>> {
    return this.sdk.request('POST', '/api/v1/simulations', config);
  }

  /**
   * Get simulation status
   */
  async getStatus(simulationId: string): Promise<ApiResponse<SimulationState>> {
    return this.sdk.request('GET', `/api/v1/simulations/${simulationId}/status`);
  }

  /**
   * Get simulation results
   */
  async getResults(simulationId: string): Promise<ApiResponse<SimulationResults>> {
    return this.sdk.request('GET', `/api/v1/simulations/${simulationId}/results`);
  }

  /**
   * Start simulation
   */
  async start(simulationId: string): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/simulations/${simulationId}/start`);
  }

  /**
   * Stop simulation
   */
  async stop(simulationId: string): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/simulations/${simulationId}/stop`);
  }

  /**
   * Delete simulation
   */
  async delete(simulationId: string): Promise<ApiResponse<void>> {
    return this.sdk.request('DELETE', `/api/v1/simulations/${simulationId}`);
  }

  /**
   * List simulations
   */
  async list(params?: {
    status?: string;
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<SimulationConfig>>> {
    return this.sdk.request('GET', '/api/v1/simulations', undefined, params);
  }

  /**
   * Run simulation and wait for completion
   */
  async runAndWait(
    config: Omit<SimulationConfig, 'simulationId'>,
    pollInterval: number = 5000
  ): Promise<ApiResponse<SimulationResults>> {
    // Create simulation
    const createResponse = await this.create(config);
    if (!createResponse.success || !createResponse.data) {
      throw new Error('Failed to create simulation');
    }

    const simulationId = createResponse.data.simulationId;

    // Start simulation
    await this.start(simulationId);

    // Poll for completion
    while (true) {
      const statusResponse = await this.getStatus(simulationId);
      if (!statusResponse.success || !statusResponse.data) {
        throw new Error('Failed to get simulation status');
      }

      const status = statusResponse.data.status;

      if (status === 'completed') {
        return this.getResults(simulationId);
      } else if (status === 'failed') {
        throw new Error(statusResponse.data.error?.message || 'Simulation failed');
      }

      // Wait before next poll
      await new Promise((resolve) => setTimeout(resolve, pollInterval));
    }
  }
}

// ============================================================================
// Traffic API
// ============================================================================

class TrafficAPI {
  constructor(private sdk: TrafficSimulationSDK) {}

  /**
   * Get real-time traffic data
   */
  async getRealtime(params?: {
    linkIds?: string[];
    bbox?: { minLat: number; minLon: number; maxLat: number; maxLon: number };
  }): Promise<ApiResponse<{ states: TrafficState[] }>> {
    return this.sdk.request('GET', '/api/v1/traffic/realtime', undefined, params);
  }

  /**
   * Get traffic data for a link
   */
  async getLinkTraffic(linkId: string, params?: {
    start?: string;
    end?: string;
    interval?: number;
  }): Promise<ApiResponse<{ states: TrafficState[] }>> {
    return this.sdk.request('GET', `/api/v1/traffic/links/${linkId}`, undefined, params);
  }

  /**
   * Get historical traffic data
   */
  async getHistorical(params: {
    linkIds?: string[];
    start: string;
    end: string;
    aggregation?: 'raw' | '5min' | '15min' | '1hour';
  }): Promise<ApiResponse<{ states: TrafficState[] }>> {
    return this.sdk.request('GET', '/api/v1/traffic/historical', undefined, params);
  }

  /**
   * Get detector data
   */
  async getDetectorData(detectorId: string, params?: {
    start?: string;
    end?: string;
  }): Promise<ApiResponse<{ measurements: any[] }>> {
    return this.sdk.request('GET', `/api/v1/traffic/detectors/${detectorId}`, undefined, params);
  }
}

// ============================================================================
// Signals API
// ============================================================================

class SignalsAPI {
  constructor(private sdk: TrafficSimulationSDK) {}

  /**
   * List signals
   */
  async list(params?: {
    bbox?: { minLat: number; minLon: number; maxLat: number; maxLon: number };
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<TrafficSignal>>> {
    return this.sdk.request('GET', '/api/v1/signals', undefined, params);
  }

  /**
   * Get signal by ID
   */
  async get(signalId: string): Promise<ApiResponse<TrafficSignal>> {
    return this.sdk.request('GET', `/api/v1/signals/${signalId}`);
  }

  /**
   * Update signal timing
   */
  async updateTiming(
    signalId: string,
    timingPlan: SignalTimingPlan
  ): Promise<ApiResponse<TrafficSignal>> {
    return this.sdk.request('PUT', `/api/v1/signals/${signalId}/timing`, timingPlan);
  }

  /**
   * Optimize signal timing
   */
  async optimize(signalIds: string[], params?: {
    objective?: 'minimize_delay' | 'maximize_throughput' | 'green_wave';
    constraints?: any;
  }): Promise<ApiResponse<{
    optimizedTimings: SignalTimingPlan[];
    improvement: {
      delayReduction: number;
      throughputIncrease: number;
    };
  }>> {
    return this.sdk.request('POST', '/api/v1/signals/optimize', { signalIds, ...params });
  }

  /**
   * Get signal coordination plan
   */
  async getCoordination(corridorId: string): Promise<ApiResponse<{
    signals: TrafficSignal[];
    bandwidth: number;
    speed: number;
  }>> {
    return this.sdk.request('GET', `/api/v1/signals/coordination/${corridorId}`);
  }
}

// ============================================================================
// Vehicles API
// ============================================================================

class VehiclesAPI {
  constructor(private sdk: TrafficSimulationSDK) {}

  /**
   * Get vehicle by ID
   */
  async get(vehicleId: string): Promise<ApiResponse<Vehicle>> {
    return this.sdk.request('GET', `/api/v1/vehicles/${vehicleId}`);
  }

  /**
   * List active vehicles
   */
  async listActive(params?: {
    simulationId?: string;
    linkId?: string;
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<Vehicle>>> {
    return this.sdk.request('GET', '/api/v1/vehicles', undefined, params);
  }

  /**
   * Get vehicle trajectory
   */
  async getTrajectory(vehicleId: string): Promise<ApiResponse<{
    vehicleId: string;
    trajectory: Array<{
      timestamp: string;
      linkId: string;
      position: number;
      speed: number;
    }>;
  }>> {
    return this.sdk.request('GET', `/api/v1/vehicles/${vehicleId}/trajectory`);
  }

  /**
   * Inject vehicle into simulation
   */
  async inject(vehicle: Omit<Vehicle, 'vehicleId' | 'currentState'>): Promise<ApiResponse<{
    vehicleId: string;
  }>> {
    return this.sdk.request('POST', '/api/v1/vehicles', vehicle);
  }
}

// ============================================================================
// Transit API
// ============================================================================

class TransitAPI {
  constructor(private sdk: TrafficSimulationSDK) {}

  /**
   * List transit routes
   */
  async listRoutes(params?: {
    routeType?: string;
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<TransitRoute>>> {
    return this.sdk.request('GET', '/api/v1/transit/routes', undefined, params);
  }

  /**
   * Get route by ID
   */
  async getRoute(routeId: string): Promise<ApiResponse<TransitRoute>> {
    return this.sdk.request('GET', `/api/v1/transit/routes/${routeId}`);
  }

  /**
   * Get transit vehicle
   */
  async getVehicle(vehicleId: string): Promise<ApiResponse<TransitVehicle>> {
    return this.sdk.request('GET', `/api/v1/transit/vehicles/${vehicleId}`);
  }

  /**
   * List transit vehicles on route
   */
  async listVehiclesOnRoute(routeId: string): Promise<ApiResponse<{
    vehicles: TransitVehicle[];
  }>> {
    return this.sdk.request('GET', `/api/v1/transit/routes/${routeId}/vehicles`);
  }

  /**
   * Get transit performance
   */
  async getPerformance(routeId: string, params: {
    start: string;
    end: string;
  }): Promise<ApiResponse<{
    routeId: string;
    onTimePerformance: number;
    avgDelay: number;
    avgSpeed: number;
    passengerLoad: number;
  }>> {
    return this.sdk.request('GET', `/api/v1/transit/routes/${routeId}/performance`, undefined, params);
  }
}

// ============================================================================
// Incidents API
// ============================================================================

class IncidentsAPI {
  constructor(private sdk: TrafficSimulationSDK) {}

  /**
   * List incidents
   */
  async list(params?: {
    active?: boolean;
    type?: string;
    severity?: string;
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<TrafficIncident>>> {
    return this.sdk.request('GET', '/api/v1/incidents', undefined, params);
  }

  /**
   * Get incident by ID
   */
  async get(incidentId: string): Promise<ApiResponse<TrafficIncident>> {
    return this.sdk.request('GET', `/api/v1/incidents/${incidentId}`);
  }

  /**
   * Create incident
   */
  async create(incident: Omit<TrafficIncident, 'incidentId'>): Promise<ApiResponse<{
    incidentId: string;
  }>> {
    return this.sdk.request('POST', '/api/v1/incidents', incident);
  }

  /**
   * Update incident
   */
  async update(incidentId: string, incident: Partial<TrafficIncident>): Promise<ApiResponse<TrafficIncident>> {
    return this.sdk.request('PUT', `/api/v1/incidents/${incidentId}`, incident);
  }

  /**
   * Close incident
   */
  async close(incidentId: string): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/incidents/${incidentId}/close`);
  }

  /**
   * Analyze incident impact
   */
  async analyzeImpact(incidentId: string): Promise<ApiResponse<IncidentImpact>> {
    return this.sdk.request('POST', `/api/v1/incidents/${incidentId}/analyze`);
  }
}

// ============================================================================
// Forecasting API
// ============================================================================

class ForecastingAPI {
  constructor(private sdk: TrafficSimulationSDK) {}

  /**
   * Get traffic forecast
   */
  async getForecast(linkId: string, params: {
    horizon: 'short' | 'medium' | 'long';
    forecastTime?: string;
  }): Promise<ApiResponse<TrafficForecast>> {
    return this.sdk.request('GET', `/api/v1/forecast/links/${linkId}`, undefined, params);
  }

  /**
   * Get forecasts for multiple links
   */
  async getForecasts(linkIds: string[], params: {
    horizon: 'short' | 'medium' | 'long';
    forecastTime?: string;
  }): Promise<ApiResponse<{ forecasts: TrafficForecast[] }>> {
    return this.sdk.request('POST', '/api/v1/forecast/batch', { linkIds, ...params });
  }

  /**
   * Predict congestion
   */
  async predictCongestion(params: {
    bbox?: { minLat: number; minLon: number; maxLat: number; maxLon: number };
    linkIds?: string[];
    time: string;
    duration?: number;
  }): Promise<ApiResponse<{
    predictions: Array<{
      linkId: string;
      congestionLevel: string;
      probability: number;
    }>;
  }>> {
    return this.sdk.request('POST', '/api/v1/forecast/congestion', params);
  }
}

// ============================================================================
// Congestion API
// ============================================================================

class CongestionAPI {
  constructor(private sdk: TrafficSimulationSDK) {}

  /**
   * Get current congestion
   */
  async getCurrent(params?: {
    linkIds?: string[];
    bbox?: { minLat: number; minLon: number; maxLat: number; maxLon: number };
  }): Promise<ApiResponse<{ congestion: CongestionData[] }>> {
    return this.sdk.request('GET', '/api/v1/congestion/current', undefined, params);
  }

  /**
   * Get congestion analytics
   */
  async getAnalytics(params: {
    start: string;
    end: string;
    linkIds?: string[];
  }): Promise<ApiResponse<{
    avgTCI: number;
    avgTTI: number;
    avgPTI: number;
    recurrentCongestion: number;
    nonRecurrentCongestion: number;
  }>> {
    return this.sdk.request('GET', '/api/v1/congestion/analytics', undefined, params);
  }

  /**
   * Identify bottlenecks
   */
  async identifyBottlenecks(params?: {
    threshold?: number;
    minDuration?: number;
  }): Promise<ApiResponse<{
    bottlenecks: Array<{
      linkId: string;
      intensity: number;
      frequency: number;
      avgDuration: number;
    }>;
  }>> {
    return this.sdk.request('GET', '/api/v1/congestion/bottlenecks', undefined, params);
  }
}

// ============================================================================
// Origin-Destination API
// ============================================================================

class OriginDestinationAPI {
  constructor(private sdk: TrafficSimulationSDK) {}

  /**
   * Get OD matrix
   */
  async getMatrix(matrixId: string): Promise<ApiResponse<ODMatrix>> {
    return this.sdk.request('GET', `/api/v1/od/matrices/${matrixId}`);
  }

  /**
   * List OD matrices
   */
  async listMatrices(params?: {
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<ODMatrix>>> {
    return this.sdk.request('GET', '/api/v1/od/matrices', undefined, params);
  }

  /**
   * Create OD matrix
   */
  async createMatrix(matrix: Omit<ODMatrix, 'matrixId'>): Promise<ApiResponse<{
    matrixId: string;
  }>> {
    return this.sdk.request('POST', '/api/v1/od/matrices', matrix);
  }

  /**
   * Estimate OD matrix from traffic counts
   */
  async estimateFromCounts(params: {
    linkCounts: Array<{ linkId: string; count: number }>;
    seedMatrix?: string;
  }): Promise<ApiResponse<{ matrixId: string; accuracy: number }>> {
    return this.sdk.request('POST', '/api/v1/od/estimate', params);
  }

  /**
   * List traffic zones
   */
  async listZones(): Promise<ApiResponse<{ zones: TrafficZone[] }>> {
    return this.sdk.request('GET', '/api/v1/od/zones');
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate traffic flow parameters
 */
export function calculateFlow(speed: number, density: number): {
  volume: number;
  vcRatio: number;
  los: string;
} {
  const volume = speed * density;
  const capacity = 2200; // veh/h/lane (typical)
  const vcRatio = volume / capacity;

  let los: string;
  if (vcRatio <= 0.35) los = 'A';
  else if (vcRatio <= 0.54) los = 'B';
  else if (vcRatio <= 0.77) los = 'C';
  else if (vcRatio <= 0.90) los = 'D';
  else if (vcRatio <= 1.00) los = 'E';
  else los = 'F';

  return { volume, vcRatio, los };
}

/**
 * Calculate GEH statistic for traffic count comparison
 */
export function calculateGEH(observed: number, modeled: number): number {
  return Math.sqrt(2 * Math.pow(modeled - observed, 2) / (modeled + observed));
}

/**
 * Calculate queue length from demand and capacity
 */
export function calculateQueueLength(
  demand: number,        // veh/h
  capacity: number,      // veh/h
  duration: number,      // hours
  jamDensity: number = 140  // veh/km
): number {
  if (demand <= capacity) return 0;
  const queueVehicles = (demand - capacity) * duration;
  return queueVehicles / jamDensity;
}

/**
 * Calculate travel time index (TTI)
 */
export function calculateTTI(
  actualTravelTime: number,
  freeFlowTravelTime: number
): number {
  return actualTravelTime / freeFlowTravelTime;
}

/**
 * Calculate traffic congestion index (TCI)
 */
export function calculateTCI(
  actualTravelTime: number,
  freeFlowTravelTime: number
): number {
  return ((actualTravelTime - freeFlowTravelTime) / freeFlowTravelTime) * 100;
}

// ============================================================================
// Exports
// ============================================================================

export * from './types';

export default TrafficSimulationSDK;
