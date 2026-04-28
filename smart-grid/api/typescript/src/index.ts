/**
 * WIA-ENERGY-004: Smart Grid Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Export all types
export * from './types';

import {
  GridNode,
  TransmissionLine,
  PowerPlant,
  GenerationData,
  LoadDistribution,
  SmartMeter,
  SmartMeterReading,
  DemandResponseEvent,
  DRParticipant,
  GridFrequencyStatus,
  VoltageStability,
  GridInertia,
  RenewableForecast,
  CurtailmentEvent,
  GridIntegrationMetrics,
  OutageEvent,
  OutageDetection,
  RestorationPlan,
  BatteryStorageSystem,
  GridEvent,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
  TimeSeriesDataPoint,
  GridEventType,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface SmartGridSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
  region?: string;
}

// ============================================================================
// Event Handler Types
// ============================================================================

export type EventHandler = (event: GridEvent) => void | Promise<void>;

// ============================================================================
// SDK Client
// ============================================================================

export class WIASmartGrid {
  private config: Required<SmartGridSDKConfig>;
  private headers: Record<string, string>;
  private eventHandlers: Map<GridEventType, EventHandler[]>;
  private wsConnection?: WebSocket;

  constructor(config: SmartGridSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      region: 'default',
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'ENERGY-004',
      'X-WIA-Version': '1.0.0',
    };

    this.eventHandlers = new Map();
  }

  // ==========================================================================
  // Grid Topology APIs
  // ==========================================================================

  /**
   * Get grid node information
   */
  async getGridNode(nodeId: string): Promise<ApiResponse<GridNode>> {
    return this.get<GridNode>(`/api/v1/grid/nodes/${nodeId}`);
  }

  /**
   * List all grid nodes in a zone
   */
  async listGridNodes(
    zoneId: string,
    params?: PaginationParams
  ): Promise<ApiResponse<PaginatedResponse<GridNode>>> {
    const queryParams = this.buildQueryParams({ zoneId, ...params });
    return this.get<PaginatedResponse<GridNode>>(`/api/v1/grid/nodes${queryParams}`);
  }

  /**
   * Get transmission line details
   */
  async getTransmissionLine(lineId: string): Promise<ApiResponse<TransmissionLine>> {
    return this.get<TransmissionLine>(`/api/v1/grid/lines/${lineId}`);
  }

  /**
   * Get grid topology for a zone
   */
  async getGridTopology(zoneId: string): Promise<ApiResponse<{
    nodes: GridNode[];
    lines: TransmissionLine[];
  }>> {
    return this.get(`/api/v1/grid/topology?zoneId=${zoneId}`);
  }

  // ==========================================================================
  // Grid Monitoring APIs
  // ==========================================================================

  /**
   * Get real-time grid frequency status
   */
  async getFrequencyStatus(zoneId: string): Promise<ApiResponse<GridFrequencyStatus>> {
    return this.get<GridFrequencyStatus>(`/api/v1/grid/monitoring/frequency?zoneId=${zoneId}`);
  }

  /**
   * Get voltage stability metrics
   */
  async getVoltageStability(nodeId: string): Promise<ApiResponse<VoltageStability>> {
    return this.get<VoltageStability>(`/api/v1/grid/monitoring/voltage/${nodeId}`);
  }

  /**
   * Get grid inertia metrics
   */
  async getGridInertia(zoneId: string): Promise<ApiResponse<GridInertia>> {
    return this.get<GridInertia>(`/api/v1/grid/monitoring/inertia?zoneId=${zoneId}`);
  }

  /**
   * Monitor grid health across all metrics
   */
  async getGridHealth(zoneId: string): Promise<ApiResponse<{
    frequency: GridFrequencyStatus;
    voltage: VoltageStability[];
    inertia: GridInertia;
    overallStatus: 'healthy' | 'degraded' | 'critical';
  }>> {
    return this.get(`/api/v1/grid/monitoring/health?zoneId=${zoneId}`);
  }

  // ==========================================================================
  // Generation Management APIs
  // ==========================================================================

  /**
   * Get power plant information
   */
  async getPowerPlant(plantId: string): Promise<ApiResponse<PowerPlant>> {
    return this.get<PowerPlant>(`/api/v1/generation/plants/${plantId}`);
  }

  /**
   * List all power plants
   */
  async listPowerPlants(
    filters?: {
      zoneId?: string;
      type?: string;
      isRenewable?: boolean;
    }
  ): Promise<ApiResponse<PowerPlant[]>> {
    const queryParams = this.buildQueryParams(filters);
    return this.get<PowerPlant[]>(`/api/v1/generation/plants${queryParams}`);
  }

  /**
   * Get real-time generation data
   */
  async getGenerationData(plantId: string): Promise<ApiResponse<GenerationData>> {
    return this.get<GenerationData>(`/api/v1/generation/plants/${plantId}/data`);
  }

  /**
   * Get aggregated generation by zone
   */
  async getZoneGeneration(zoneId: string): Promise<ApiResponse<{
    totalGeneration: number;
    byType: Record<string, number>;
    renewablePercentage: number;
  }>> {
    return this.get(`/api/v1/generation/zones/${zoneId}`);
  }

  // ==========================================================================
  // Load Management & Balancing APIs
  // ==========================================================================

  /**
   * Get load distribution for a zone
   */
  async getLoadDistribution(zoneId: string): Promise<ApiResponse<LoadDistribution>> {
    return this.get<LoadDistribution>(`/api/v1/load/distribution?zoneId=${zoneId}`);
  }

  /**
   * Get load forecast
   */
  async getLoadForecast(
    zoneId: string,
    horizon: '15min' | '1hour' | '6hour' | '24hour' | '7day'
  ): Promise<ApiResponse<TimeSeriesDataPoint[]>> {
    return this.get<TimeSeriesDataPoint[]>(
      `/api/v1/load/forecast?zoneId=${zoneId}&horizon=${horizon}`
    );
  }

  /**
   * Perform load balancing optimization
   */
  async optimizeLoadBalancing(
    zoneId: string,
    constraints?: {
      maxFrequencyDeviation?: number;
      renewablePreference?: boolean;
      costOptimization?: boolean;
    }
  ): Promise<ApiResponse<{
    recommendedDispatch: { plantId: string; power: number }[];
    expectedFrequency: number;
    estimatedCost: number;
    renewableUtilization: number;
  }>> {
    return this.post(`/api/v1/load/optimize`, { zoneId, constraints });
  }

  /**
   * Get real-time load-generation balance
   */
  async getLoadGenerationBalance(zoneId: string): Promise<ApiResponse<{
    totalLoad: number;
    totalGeneration: number;
    balance: number;
    status: 'balanced' | 'surplus' | 'deficit';
  }>> {
    return this.get(`/api/v1/load/balance?zoneId=${zoneId}`);
  }

  // ==========================================================================
  // Demand Response APIs
  // ==========================================================================

  /**
   * Create demand response event
   */
  async createDREvent(event: Partial<DemandResponseEvent>): Promise<ApiResponse<DemandResponseEvent>> {
    return this.post<DemandResponseEvent>('/api/v1/demand-response/events', event);
  }

  /**
   * Get active demand response events
   */
  async getActiveDREvents(zoneId?: string): Promise<ApiResponse<DemandResponseEvent[]>> {
    const queryParams = zoneId ? `?zoneId=${zoneId}` : '';
    return this.get<DemandResponseEvent[]>(`/api/v1/demand-response/events/active${queryParams}`);
  }

  /**
   * Get DR event history
   */
  async getDREventHistory(
    dateRange: DateRangeFilter,
    zoneId?: string
  ): Promise<ApiResponse<DemandResponseEvent[]>> {
    const queryParams = this.buildQueryParams({ ...dateRange, zoneId });
    return this.get<DemandResponseEvent[]>(`/api/v1/demand-response/events/history${queryParams}`);
  }

  /**
   * Register DR participant
   */
  async registerDRParticipant(
    participant: Partial<DRParticipant>
  ): Promise<ApiResponse<DRParticipant>> {
    return this.post<DRParticipant>('/api/v1/demand-response/participants', participant);
  }

  /**
   * Get DR participant information
   */
  async getDRParticipant(participantId: string): Promise<ApiResponse<DRParticipant>> {
    return this.get<DRParticipant>(`/api/v1/demand-response/participants/${participantId}`);
  }

  /**
   * Calculate DR potential
   */
  async calculateDRPotential(zoneId: string): Promise<ApiResponse<{
    totalEnrolledCapacity: number;
    availableCapacity: number;
    participants: number;
    averageResponseTime: number;
  }>> {
    return this.get(`/api/v1/demand-response/potential?zoneId=${zoneId}`);
  }

  // ==========================================================================
  // Smart Meter Management APIs
  // ==========================================================================

  /**
   * Register smart meter
   */
  async registerSmartMeter(meter: Partial<SmartMeter>): Promise<ApiResponse<SmartMeter>> {
    return this.post<SmartMeter>('/api/v1/meters', meter);
  }

  /**
   * Get smart meter information
   */
  async getSmartMeter(meterId: string): Promise<ApiResponse<SmartMeter>> {
    return this.get<SmartMeter>(`/api/v1/meters/${meterId}`);
  }

  /**
   * Get latest smart meter reading
   */
  async getLatestReading(meterId: string): Promise<ApiResponse<SmartMeterReading>> {
    return this.get<SmartMeterReading>(`/api/v1/meters/${meterId}/readings/latest`);
  }

  /**
   * Get smart meter reading history
   */
  async getMeterReadingHistory(
    meterId: string,
    dateRange: DateRangeFilter,
    interval?: '1min' | '15min' | '1hour' | '1day'
  ): Promise<ApiResponse<SmartMeterReading[]>> {
    const queryParams = this.buildQueryParams({ ...dateRange, interval });
    return this.get<SmartMeterReading[]>(`/api/v1/meters/${meterId}/readings${queryParams}`);
  }

  /**
   * Get aggregated meter data for a zone
   */
  async getZoneMeterData(
    zoneId: string,
    aggregation: 'sum' | 'average' | 'max' | 'min' = 'sum'
  ): Promise<ApiResponse<{
    totalMeters: number;
    totalLoad: number;
    averageVoltage: number;
    averageFrequency: number;
    powerQualityIssues: number;
  }>> {
    return this.get(`/api/v1/meters/zones/${zoneId}/aggregate?aggregation=${aggregation}`);
  }

  /**
   * Detect meter anomalies
   */
  async detectMeterAnomalies(
    zoneId?: string
  ): Promise<ApiResponse<{
    meterId: string;
    anomalyType: string;
    severity: string;
    timestamp: string;
  }[]>> {
    const queryParams = zoneId ? `?zoneId=${zoneId}` : '';
    return this.get(`/api/v1/meters/anomalies${queryParams}`);
  }

  // ==========================================================================
  // Renewable Integration APIs
  // ==========================================================================

  /**
   * Get renewable generation forecast
   */
  async getRenewableForecast(
    sourceId: string,
    horizon: '15min' | '1hour' | '6hour' | '24hour' | '7day'
  ): Promise<ApiResponse<RenewableForecast>> {
    return this.get<RenewableForecast>(
      `/api/v1/renewable/forecast?sourceId=${sourceId}&horizon=${horizon}`
    );
  }

  /**
   * Get curtailment events
   */
  async getCurtailmentEvents(
    dateRange: DateRangeFilter,
    sourceId?: string
  ): Promise<ApiResponse<CurtailmentEvent[]>> {
    const queryParams = this.buildQueryParams({ ...dateRange, sourceId });
    return this.get<CurtailmentEvent[]>(`/api/v1/renewable/curtailment${queryParams}`);
  }

  /**
   * Request renewable curtailment
   */
  async requestCurtailment(
    sourceId: string,
    power: number,
    reason: string
  ): Promise<ApiResponse<CurtailmentEvent>> {
    return this.post<CurtailmentEvent>('/api/v1/renewable/curtailment', {
      sourceId,
      power,
      reason,
    });
  }

  /**
   * Get grid integration metrics
   */
  async getGridIntegrationMetrics(
    zoneId: string
  ): Promise<ApiResponse<GridIntegrationMetrics>> {
    return this.get<GridIntegrationMetrics>(
      `/api/v1/renewable/integration?zoneId=${zoneId}`
    );
  }

  /**
   * Optimize renewable dispatch
   */
  async optimizeRenewableDispatch(
    zoneId: string,
    targetPenetration?: number
  ): Promise<ApiResponse<{
    recommendedCurtailment: { sourceId: string; reduction: number }[];
    achievedPenetration: number;
    gridStability: 'stable' | 'marginal' | 'unstable';
  }>> {
    return this.post('/api/v1/renewable/optimize', { zoneId, targetPenetration });
  }

  // ==========================================================================
  // Outage Detection & Management APIs
  // ==========================================================================

  /**
   * Report outage detection
   */
  async reportOutage(detection: Partial<OutageDetection>): Promise<ApiResponse<OutageEvent>> {
    return this.post<OutageEvent>('/api/v1/outages/detect', detection);
  }

  /**
   * Get active outages
   */
  async getActiveOutages(zoneId?: string): Promise<ApiResponse<OutageEvent[]>> {
    const queryParams = zoneId ? `?zoneId=${zoneId}` : '';
    return this.get<OutageEvent[]>(`/api/v1/outages/active${queryParams}`);
  }

  /**
   * Get outage details
   */
  async getOutageDetails(outageId: string): Promise<ApiResponse<OutageEvent>> {
    return this.get<OutageEvent>(`/api/v1/outages/${outageId}`);
  }

  /**
   * Get outage history
   */
  async getOutageHistory(
    dateRange: DateRangeFilter,
    zoneId?: string
  ): Promise<ApiResponse<OutageEvent[]>> {
    const queryParams = this.buildQueryParams({ ...dateRange, zoneId });
    return this.get<OutageEvent[]>(`/api/v1/outages/history${queryParams}`);
  }

  /**
   * Create restoration plan
   */
  async createRestorationPlan(
    outageId: string,
    plan?: Partial<RestorationPlan>
  ): Promise<ApiResponse<RestorationPlan>> {
    return this.post<RestorationPlan>(`/api/v1/outages/${outageId}/restoration`, plan);
  }

  /**
   * Get restoration plan
   */
  async getRestorationPlan(planId: string): Promise<ApiResponse<RestorationPlan>> {
    return this.get<RestorationPlan>(`/api/v1/outages/restoration/${planId}`);
  }

  /**
   * Update restoration status
   */
  async updateRestorationStatus(
    planId: string,
    stepNumber: number,
    status: string
  ): Promise<ApiResponse<void>> {
    return this.put(`/api/v1/outages/restoration/${planId}/steps/${stepNumber}`, { status });
  }

  /**
   * Predict outage risk
   */
  async predictOutageRisk(
    zoneId: string,
    weatherData?: any
  ): Promise<ApiResponse<{
    riskLevel: 'low' | 'medium' | 'high' | 'critical';
    factors: string[];
    recommendedActions: string[];
  }>> {
    return this.post('/api/v1/outages/predict', { zoneId, weatherData });
  }

  // ==========================================================================
  // Energy Storage APIs
  // ==========================================================================

  /**
   * Get battery storage system info
   */
  async getBatteryStorage(storageId: string): Promise<ApiResponse<BatteryStorageSystem>> {
    return this.get<BatteryStorageSystem>(`/api/v1/storage/${storageId}`);
  }

  /**
   * List all storage systems
   */
  async listStorageSystems(zoneId?: string): Promise<ApiResponse<BatteryStorageSystem[]>> {
    const queryParams = zoneId ? `?zoneId=${zoneId}` : '';
    return this.get<BatteryStorageSystem[]>(`/api/v1/storage${queryParams}`);
  }

  /**
   * Control storage system
   */
  async controlStorage(
    storageId: string,
    command: {
      mode: 'charging' | 'discharging' | 'idle';
      power?: number;
      targetSOC?: number;
    }
  ): Promise<ApiResponse<void>> {
    return this.post(`/api/v1/storage/${storageId}/control`, command);
  }

  /**
   * Optimize storage dispatch
   */
  async optimizeStorageDispatch(
    zoneId: string,
    objectives?: ('cost' | 'frequency' | 'voltage' | 'peak-shaving')[]
  ): Promise<ApiResponse<{
    recommendations: { storageId: string; mode: string; power: number }[];
    expectedBenefit: number;
  }>> {
    return this.post('/api/v1/storage/optimize', { zoneId, objectives });
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  /**
   * Subscribe to grid events
   */
  on(eventType: GridEventType, handler: EventHandler): void {
    if (!this.eventHandlers.has(eventType)) {
      this.eventHandlers.set(eventType, []);
    }
    this.eventHandlers.get(eventType)!.push(handler);
  }

  /**
   * Unsubscribe from grid events
   */
  off(eventType: GridEventType, handler: EventHandler): void {
    const handlers = this.eventHandlers.get(eventType);
    if (handlers) {
      const index = handlers.indexOf(handler);
      if (index > -1) {
        handlers.splice(index, 1);
      }
    }
  }

  /**
   * Connect to real-time event stream
   */
  async connectEventStream(zoneId?: string): Promise<void> {
    const wsUrl = this.config.endpoint.replace(/^http/, 'ws');
    const url = `${wsUrl}/api/v1/events/stream${zoneId ? `?zoneId=${zoneId}` : ''}`;

    this.wsConnection = new WebSocket(url);

    this.wsConnection.onmessage = async (message) => {
      try {
        const event: GridEvent = JSON.parse(message.data);
        const handlers = this.eventHandlers.get(event.type);
        if (handlers) {
          for (const handler of handlers) {
            await handler(event);
          }
        }
      } catch (error) {
        if (this.config.debug) {
          console.error('[WIA-ENERGY-004] Event handling error:', error);
        }
      }
    };

    return new Promise((resolve, reject) => {
      this.wsConnection!.onopen = () => resolve();
      this.wsConnection!.onerror = (error) => reject(error);
    });
  }

  /**
   * Disconnect from event stream
   */
  disconnectEventStream(): void {
    if (this.wsConnection) {
      this.wsConnection.close();
      this.wsConnection = undefined;
    }
  }

  /**
   * Get event history
   */
  async getEventHistory(
    dateRange: DateRangeFilter,
    filters?: {
      eventType?: GridEventType;
      severity?: string;
      zoneId?: string;
    }
  ): Promise<ApiResponse<GridEvent[]>> {
    const queryParams = this.buildQueryParams({ ...dateRange, ...filters });
    return this.get<GridEvent[]>(`/api/v1/events/history${queryParams}`);
  }

  /**
   * Acknowledge event
   */
  async acknowledgeEvent(eventId: string, acknowledgedBy: string): Promise<ApiResponse<void>> {
    return this.post(`/api/v1/events/${eventId}/acknowledge`, { acknowledgedBy });
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
      console.log(`[WIA-ENERGY-004] ${method} ${url}`, body || '');
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
        console.error('[WIA-ENERGY-004] Request failed:', error);
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

  private async post<T>(path: string, body?: any): Promise<ApiResponse<T>> {
    return this.request<T>('POST', path, body);
  }

  private async put<T>(path: string, body?: any): Promise<ApiResponse<T>> {
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
 * Calculate grid frequency deviation
 */
export function calculateFrequencyDeviation(
  actual: number,
  nominal: number
): { deviation: number; percentage: number } {
  const deviation = actual - nominal;
  const percentage = (deviation / nominal) * 100;
  return { deviation, percentage };
}

/**
 * Calculate voltage deviation
 */
export function calculateVoltageDeviation(
  actual: number,
  nominal: number
): { deviation: number; percentage: number } {
  const deviation = actual - nominal;
  const percentage = (deviation / nominal) * 100;
  return { deviation, percentage };
}

/**
 * Calculate power factor
 */
export function calculatePowerFactor(
  activePower: number,
  apparentPower: number
): number {
  return activePower / apparentPower;
}

/**
 * Calculate system losses
 */
export function calculateSystemLosses(
  generation: number,
  load: number
): { losses: number; percentage: number } {
  const losses = generation - load;
  const percentage = (losses / generation) * 100;
  return { losses, percentage };
}

/**
 * Estimate restoration time
 */
export function estimateRestorationTime(
  customersAffected: number,
  crewsAvailable: number,
  averageRepairTime: number
): number {
  const workUnits = customersAffected / 100; // 1 work unit per 100 customers
  const timePerCrew = (workUnits / crewsAvailable) * averageRepairTime;
  return Math.ceil(timePerCrew);
}

/**
 * Calculate renewable penetration
 */
export function calculateRenewablePenetration(
  renewableGeneration: number,
  totalGeneration: number
): number {
  return (renewableGeneration / totalGeneration) * 100;
}

/**
 * Calculate battery efficiency
 */
export function calculateBatteryEfficiency(
  energyOut: number,
  energyIn: number
): number {
  return (energyOut / energyIn) * 100;
}

// ============================================================================
// Factory Function
// ============================================================================

/**
 * Create a new WIA Smart Grid SDK instance
 */
export function createSmartGridSDK(config: SmartGridSDKConfig): WIASmartGrid {
  return new WIASmartGrid(config);
}

// ============================================================================
// Default Export
// ============================================================================

export default WIASmartGrid;
