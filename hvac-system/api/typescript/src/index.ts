/**
 * WIA-CITY-010: HVAC System Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import type {
  ApiResponse,
  SystemRegistrationRequest,
  SystemRegistrationResponse,
  TemperatureControlRequest,
  StatusQueryResponse,
  HVACSystem,
  Zone,
  ZoneSchedule,
  PerformanceMetrics,
  EnergyMetrics,
  VentilationControl,
  FilterStatus,
  Alarm,
  ConditionMonitoring,
  FailurePrediction,
  MaintenanceRecord,
  BACnetObject,
  ModbusRegister,
  Timestamp,
  OperatingMode,
  FanSpeed,
} from './types';

/**
 * Client configuration
 */
export interface HVACClientConfig {
  apiKey: string;
  endpoint?: string;
  timeout?: number;
  headers?: Record<string, string>;
}

/**
 * Default configuration
 */
const DEFAULT_CONFIG = {
  endpoint: 'https://api.wia.org/city-010/v1',
  timeout: 30000,
};

/**
 * HVAC System Client
 *
 * @example
 * ```typescript
 * const client = new HVACClient({
 *   apiKey: 'your-api-key',
 *   endpoint: 'https://api.wia.org/city-010/v1'
 * });
 *
 * // Register a system
 * const system = await client.registerSystem({
 *   system: {
 *     system_id: 'HVAC-BLDG-A-01',
 *     name: '본관 VRF 시스템',
 *     system_type: 'VRF',
 *     building: '본관',
 *     location: { lat: 37.5665, lon: 126.9780 },
 *     total_cooling_capacity_kw: 50.0,
 *     total_heating_capacity_kw: 55.0,
 *     equipment: [],
 *     zones: [],
 *     status: 'RUNNING'
 *   }
 * });
 *
 * console.log('System ID:', system.system_id);
 * console.log('Dashboard URL:', system.dashboard_url);
 * ```
 */
export class HVACClient {
  private config: Required<HVACClientConfig>;

  constructor(config: HVACClientConfig) {
    this.config = {
      ...DEFAULT_CONFIG,
      ...config,
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${config.apiKey}`,
        ...config.headers,
      },
    };
  }

  // ============================================================================
  // System Management
  // ============================================================================

  /**
   * Register a new HVAC system
   *
   * @param request - System registration information
   * @returns System registration response with ID and dashboard URL
   */
  async registerSystem(
    request: SystemRegistrationRequest
  ): Promise<SystemRegistrationResponse> {
    const response = await this.post<SystemRegistrationResponse>(
      '/hvac/systems/register',
      request
    );
    return response;
  }

  /**
   * Get HVAC system information by ID
   *
   * @param systemId - System ID
   * @returns HVAC system information
   */
  async getSystem(systemId: string): Promise<HVACSystem> {
    const response = await this.get<HVACSystem>(`/hvac/systems/${systemId}`);
    return response;
  }

  /**
   * Update HVAC system information
   *
   * @param systemId - System ID
   * @param updates - Partial system updates
   * @returns Updated system information
   */
  async updateSystem(
    systemId: string,
    updates: Partial<HVACSystem>
  ): Promise<HVACSystem> {
    const response = await this.put<HVACSystem>(
      `/hvac/systems/${systemId}`,
      updates
    );
    return response;
  }

  /**
   * List all registered HVAC systems
   *
   * @param limit - Maximum number of results
   * @param offset - Offset for pagination
   * @returns List of systems
   */
  async listSystems(limit: number = 100, offset: number = 0): Promise<{
    systems: HVACSystem[];
    total: number;
  }> {
    const response = await this.get<{
      systems: HVACSystem[];
      total: number;
    }>(`/hvac/systems?limit=${limit}&offset=${offset}`);
    return response;
  }

  /**
   * Delete an HVAC system
   *
   * @param systemId - System ID
   * @returns Deletion confirmation
   */
  async deleteSystem(systemId: string): Promise<{ deleted: boolean }> {
    const response = await this.delete<{ deleted: boolean }>(
      `/hvac/systems/${systemId}`
    );
    return response;
  }

  // ============================================================================
  // Zone Management
  // ============================================================================

  /**
   * Add a zone to the system
   *
   * @param systemId - System ID
   * @param zone - Zone information
   * @returns Created zone
   */
  async addZone(systemId: string, zone: Zone): Promise<Zone> {
    const response = await this.post<Zone>(
      `/hvac/systems/${systemId}/zones`,
      zone
    );
    return response;
  }

  /**
   * Get zone information
   *
   * @param systemId - System ID
   * @param zoneId - Zone ID
   * @returns Zone information
   */
  async getZone(systemId: string, zoneId: string): Promise<Zone> {
    const response = await this.get<Zone>(
      `/hvac/systems/${systemId}/zones/${zoneId}`
    );
    return response;
  }

  /**
   * Update zone information
   *
   * @param systemId - System ID
   * @param zoneId - Zone ID
   * @param updates - Partial zone updates
   * @returns Updated zone
   */
  async updateZone(
    systemId: string,
    zoneId: string,
    updates: Partial<Zone>
  ): Promise<Zone> {
    const response = await this.put<Zone>(
      `/hvac/systems/${systemId}/zones/${zoneId}`,
      updates
    );
    return response;
  }

  /**
   * List all zones in a system
   *
   * @param systemId - System ID
   * @returns List of zones
   */
  async listZones(systemId: string): Promise<Zone[]> {
    const response = await this.get<Zone[]>(
      `/hvac/systems/${systemId}/zones`
    );
    return response;
  }

  // ============================================================================
  // Temperature & Humidity Control
  // ============================================================================

  /**
   * Set zone temperature setpoint
   *
   * @param systemId - System ID
   * @param zoneId - Zone ID
   * @param request - Temperature control request
   * @returns Updated zone status
   */
  async setTemperature(
    systemId: string,
    zoneId: string,
    request: TemperatureControlRequest
  ): Promise<{
    zone_id: string;
    setpoint_updated: Timestamp;
    effective_immediately: boolean;
  }> {
    const response = await this.put<{
      zone_id: string;
      setpoint_updated: Timestamp;
      effective_immediately: boolean;
    }>(`/hvac/systems/${systemId}/zones/${zoneId}/setpoint`, request);
    return response;
  }

  /**
   * Set zone operating mode
   *
   * @param systemId - System ID
   * @param zoneId - Zone ID
   * @param mode - Operating mode
   * @returns Updated zone status
   */
  async setMode(
    systemId: string,
    zoneId: string,
    mode: OperatingMode
  ): Promise<{ zone_id: string; mode: OperatingMode }> {
    const response = await this.put<{ zone_id: string; mode: OperatingMode }>(
      `/hvac/systems/${systemId}/zones/${zoneId}/mode`,
      { mode }
    );
    return response;
  }

  /**
   * Set fan speed
   *
   * @param systemId - System ID
   * @param zoneId - Zone ID
   * @param fanSpeed - Fan speed
   * @returns Updated zone status
   */
  async setFanSpeed(
    systemId: string,
    zoneId: string,
    fanSpeed: FanSpeed
  ): Promise<{ zone_id: string; fan_speed: FanSpeed }> {
    const response = await this.put<{ zone_id: string; fan_speed: FanSpeed }>(
      `/hvac/systems/${systemId}/zones/${zoneId}/fan`,
      { fan_speed: fanSpeed }
    );
    return response;
  }

  /**
   * Set humidity setpoint
   *
   * @param systemId - System ID
   * @param zoneId - Zone ID
   * @param humidityPercent - Humidity setpoint (%)
   * @returns Updated zone status
   */
  async setHumidity(
    systemId: string,
    zoneId: string,
    humidityPercent: number
  ): Promise<{
    zone_id: string;
    humidity_setpoint_percent: number;
  }> {
    const response = await this.put<{
      zone_id: string;
      humidity_setpoint_percent: number;
    }>(`/hvac/systems/${systemId}/zones/${zoneId}/humidity`, {
      humidity_percent: humidityPercent,
    });
    return response;
  }

  // ============================================================================
  // Status & Monitoring
  // ============================================================================

  /**
   * Get current system status
   *
   * @param systemId - System ID
   * @returns Current system status
   */
  async getStatus(systemId: string): Promise<StatusQueryResponse> {
    const response = await this.get<StatusQueryResponse>(
      `/hvac/systems/${systemId}/status`
    );
    return response;
  }

  /**
   * Get zone current status
   *
   * @param systemId - System ID
   * @param zoneId - Zone ID
   * @returns Zone status
   */
  async getZoneStatus(systemId: string, zoneId: string): Promise<{
    zone_id: string;
    temperature_c: number;
    setpoint_c: number;
    humidity_percent: number;
    mode: OperatingMode;
    fan_speed: FanSpeed;
    comfort_level: string;
  }> {
    const response = await this.get<{
      zone_id: string;
      temperature_c: number;
      setpoint_c: number;
      humidity_percent: number;
      mode: OperatingMode;
      fan_speed: FanSpeed;
      comfort_level: string;
    }>(`/hvac/systems/${systemId}/zones/${zoneId}/status`);
    return response;
  }

  /**
   * Subscribe to real-time system status via WebSocket
   *
   * @param systemId - System ID
   * @param onStatus - Callback for incoming status updates
   * @returns WebSocket connection
   */
  subscribeToStatus(
    systemId: string,
    onStatus: (status: StatusQueryResponse) => void
  ): WebSocket {
    const wsEndpoint = this.config.endpoint
      .replace('http://', 'ws://')
      .replace('https://', 'wss://');
    const ws = new WebSocket(
      `${wsEndpoint}/hvac/systems/${systemId}/status/stream`
    );

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      onStatus(data);
    };

    ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    return ws;
  }

  // ============================================================================
  // Scheduling
  // ============================================================================

  /**
   * Set zone schedule
   *
   * @param systemId - System ID
   * @param zoneId - Zone ID
   * @param schedule - Zone schedule
   * @returns Updated schedule
   */
  async setSchedule(
    systemId: string,
    zoneId: string,
    schedule: ZoneSchedule
  ): Promise<ZoneSchedule> {
    const response = await this.put<ZoneSchedule>(
      `/hvac/systems/${systemId}/zones/${zoneId}/schedule`,
      schedule
    );
    return response;
  }

  /**
   * Get zone schedule
   *
   * @param systemId - System ID
   * @param zoneId - Zone ID
   * @returns Zone schedule
   */
  async getSchedule(systemId: string, zoneId: string): Promise<ZoneSchedule> {
    const response = await this.get<ZoneSchedule>(
      `/hvac/systems/${systemId}/zones/${zoneId}/schedule`
    );
    return response;
  }

  // ============================================================================
  // Performance & Energy
  // ============================================================================

  /**
   * Get system performance metrics
   *
   * @param systemId - System ID
   * @param startDate - Start date (ISO 8601)
   * @param endDate - End date (ISO 8601)
   * @returns Performance metrics
   */
  async getPerformance(
    systemId: string,
    startDate: Timestamp,
    endDate: Timestamp
  ): Promise<PerformanceMetrics> {
    const response = await this.get<PerformanceMetrics>(
      `/hvac/systems/${systemId}/performance?start=${startDate}&end=${endDate}`
    );
    return response;
  }

  /**
   * Get energy consumption data
   *
   * @param systemId - System ID
   * @param startDate - Start date (ISO 8601)
   * @param endDate - End date (ISO 8601)
   * @returns Energy metrics
   */
  async getEnergyData(
    systemId: string,
    startDate: Timestamp,
    endDate: Timestamp
  ): Promise<EnergyMetrics> {
    const response = await this.get<EnergyMetrics>(
      `/hvac/systems/${systemId}/energy?start=${startDate}&end=${endDate}`
    );
    return response;
  }

  /**
   * Get energy efficiency recommendations
   *
   * @param systemId - System ID
   * @returns Energy saving recommendations
   */
  async getEnergyRecommendations(systemId: string): Promise<{
    recommendations: Array<{
      strategy: string;
      description: string;
      potential_savings_percent: number;
      implementation_cost: string;
      payback_period_months: number;
    }>;
  }> {
    const response = await this.get<{
      recommendations: Array<{
        strategy: string;
        description: string;
        potential_savings_percent: number;
        implementation_cost: string;
        payback_period_months: number;
      }>;
    }>(`/hvac/systems/${systemId}/energy/recommendations`);
    return response;
  }

  // ============================================================================
  // Ventilation & Air Quality
  // ============================================================================

  /**
   * Get ventilation status
   *
   * @param systemId - System ID
   * @returns Ventilation control status
   */
  async getVentilation(systemId: string): Promise<VentilationControl> {
    const response = await this.get<VentilationControl>(
      `/hvac/systems/${systemId}/ventilation`
    );
    return response;
  }

  /**
   * Update ventilation settings
   *
   * @param systemId - System ID
   * @param settings - Ventilation settings
   * @returns Updated ventilation control
   */
  async setVentilation(
    systemId: string,
    settings: Partial<VentilationControl>
  ): Promise<VentilationControl> {
    const response = await this.put<VentilationControl>(
      `/hvac/systems/${systemId}/ventilation`,
      settings
    );
    return response;
  }

  /**
   * Get filter status
   *
   * @param systemId - System ID
   * @returns List of filter statuses
   */
  async getFilterStatus(systemId: string): Promise<FilterStatus[]> {
    const response = await this.get<FilterStatus[]>(
      `/hvac/systems/${systemId}/filters`
    );
    return response;
  }

  /**
   * Record filter replacement
   *
   * @param systemId - System ID
   * @param filterId - Filter ID
   * @returns Updated filter status
   */
  async recordFilterReplacement(
    systemId: string,
    filterId: string
  ): Promise<FilterStatus> {
    const response = await this.post<FilterStatus>(
      `/hvac/systems/${systemId}/filters/${filterId}/replace`,
      { replaced_at: new Date().toISOString() }
    );
    return response;
  }

  // ============================================================================
  // Alarms & Notifications
  // ============================================================================

  /**
   * Get active alarms
   *
   * @param systemId - System ID
   * @returns List of active alarms
   */
  async getActiveAlarms(systemId: string): Promise<Alarm[]> {
    const response = await this.get<Alarm[]>(
      `/hvac/systems/${systemId}/alarms/active`
    );
    return response;
  }

  /**
   * Acknowledge an alarm
   *
   * @param systemId - System ID
   * @param alarmId - Alarm ID
   * @returns Updated alarm
   */
  async acknowledgeAlarm(systemId: string, alarmId: string): Promise<Alarm> {
    const response = await this.post<Alarm>(
      `/hvac/systems/${systemId}/alarms/${alarmId}/acknowledge`,
      {}
    );
    return response;
  }

  /**
   * Subscribe to real-time alarms via WebSocket
   *
   * @param systemId - System ID
   * @param onAlarm - Callback for incoming alarms
   * @returns WebSocket connection
   */
  subscribeToAlarms(
    systemId: string,
    onAlarm: (alarm: Alarm) => void
  ): WebSocket {
    const wsEndpoint = this.config.endpoint
      .replace('http://', 'ws://')
      .replace('https://', 'wss://');
    const ws = new WebSocket(
      `${wsEndpoint}/hvac/systems/${systemId}/alarms/stream`
    );

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      onAlarm(data);
    };

    ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    return ws;
  }

  // ============================================================================
  // Predictive Maintenance
  // ============================================================================

  /**
   * Get condition monitoring data
   *
   * @param systemId - System ID
   * @param equipmentId - Equipment ID
   * @returns Condition monitoring data
   */
  async getConditionMonitoring(
    systemId: string,
    equipmentId: string
  ): Promise<ConditionMonitoring> {
    const response = await this.get<ConditionMonitoring>(
      `/hvac/systems/${systemId}/equipment/${equipmentId}/condition`
    );
    return response;
  }

  /**
   * Get failure predictions
   *
   * @param systemId - System ID
   * @returns List of failure predictions
   */
  async getFailurePredictions(systemId: string): Promise<FailurePrediction[]> {
    const response = await this.get<FailurePrediction[]>(
      `/hvac/systems/${systemId}/predictions`
    );
    return response;
  }

  /**
   * Get maintenance history
   *
   * @param systemId - System ID
   * @param limit - Maximum number of records
   * @returns List of maintenance records
   */
  async getMaintenanceHistory(
    systemId: string,
    limit: number = 50
  ): Promise<MaintenanceRecord[]> {
    const response = await this.get<MaintenanceRecord[]>(
      `/hvac/systems/${systemId}/maintenance?limit=${limit}`
    );
    return response;
  }

  /**
   * Record maintenance activity
   *
   * @param systemId - System ID
   * @param record - Maintenance record
   * @returns Created maintenance record
   */
  async recordMaintenance(
    systemId: string,
    record: MaintenanceRecord
  ): Promise<MaintenanceRecord> {
    const response = await this.post<MaintenanceRecord>(
      `/hvac/systems/${systemId}/maintenance`,
      record
    );
    return response;
  }

  // ============================================================================
  // Protocol Integration
  // ============================================================================

  /**
   * Get BACnet objects
   *
   * @param systemId - System ID
   * @returns List of BACnet objects
   */
  async getBACnetObjects(systemId: string): Promise<BACnetObject[]> {
    const response = await this.get<BACnetObject[]>(
      `/hvac/systems/${systemId}/bacnet/objects`
    );
    return response;
  }

  /**
   * Read BACnet property
   *
   * @param systemId - System ID
   * @param objectType - Object type
   * @param objectInstance - Object instance
   * @param property - Property name
   * @returns Property value
   */
  async readBACnetProperty(
    systemId: string,
    objectType: string,
    objectInstance: number,
    property: string
  ): Promise<{ value: any }> {
    const response = await this.get<{ value: any }>(
      `/hvac/systems/${systemId}/bacnet/${objectType}/${objectInstance}/${property}`
    );
    return response;
  }

  /**
   * Write BACnet property
   *
   * @param systemId - System ID
   * @param objectType - Object type
   * @param objectInstance - Object instance
   * @param property - Property name
   * @param value - Property value
   * @returns Write confirmation
   */
  async writeBACnetProperty(
    systemId: string,
    objectType: string,
    objectInstance: number,
    property: string,
    value: any
  ): Promise<{ success: boolean }> {
    const response = await this.put<{ success: boolean }>(
      `/hvac/systems/${systemId}/bacnet/${objectType}/${objectInstance}/${property}`,
      { value }
    );
    return response;
  }

  /**
   * Get Modbus registers
   *
   * @param systemId - System ID
   * @returns List of Modbus registers
   */
  async getModbusRegisters(systemId: string): Promise<ModbusRegister[]> {
    const response = await this.get<ModbusRegister[]>(
      `/hvac/systems/${systemId}/modbus/registers`
    );
    return response;
  }

  /**
   * Read Modbus register
   *
   * @param systemId - System ID
   * @param address - Register address
   * @returns Register value
   */
  async readModbusRegister(
    systemId: string,
    address: number
  ): Promise<{ value: any }> {
    const response = await this.get<{ value: any }>(
      `/hvac/systems/${systemId}/modbus/${address}`
    );
    return response;
  }

  /**
   * Write Modbus register
   *
   * @param systemId - System ID
   * @param address - Register address
   * @param value - Register value
   * @returns Write confirmation
   */
  async writeModbusRegister(
    systemId: string,
    address: number,
    value: any
  ): Promise<{ success: boolean }> {
    const response = await this.put<{ success: boolean }>(
      `/hvac/systems/${systemId}/modbus/${address}`,
      { value }
    );
    return response;
  }

  // ============================================================================
  // HTTP Helpers
  // ============================================================================

  private async get<T>(path: string): Promise<T> {
    return this.request<T>('GET', path);
  }

  private async post<T>(path: string, body?: any): Promise<T> {
    return this.request<T>('POST', path, body);
  }

  private async put<T>(path: string, body?: any): Promise<T> {
    return this.request<T>('PUT', path, body);
  }

  private async delete<T>(path: string): Promise<T> {
    return this.request<T>('DELETE', path);
  }

  private async request<T>(
    method: string,
    path: string,
    body?: any
  ): Promise<T> {
    const url = `${this.config.endpoint}${path}`;
    const options: RequestInit = {
      method,
      headers: this.config.headers,
    };

    if (body) {
      options.body = JSON.stringify(body);
    }

    try {
      const response = await fetch(url, options);

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(
          errorData.error?.message || `HTTP ${response.status}: ${response.statusText}`
        );
      }

      const apiResponse: ApiResponse<T> = await response.json();

      if (!apiResponse.success) {
        throw new Error(
          apiResponse.error?.message || 'API request failed'
        );
      }

      return apiResponse.data as T;
    } catch (error) {
      if (error instanceof Error) {
        throw error;
      }
      throw new Error('Unknown error occurred');
    }
  }
}

// ============================================================================
// Export types
// ============================================================================

export * from './types';

// ============================================================================
// Re-export for convenience
// ============================================================================

export default HVACClient;
