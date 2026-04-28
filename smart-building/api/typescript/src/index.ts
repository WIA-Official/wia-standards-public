/**
 * WIA Smart Building Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Export all types
export * from './types';

import {
  BuildingInfo,
  BuildingDashboard,
  Zone,
  HVACStatus,
  HVACControlCommand,
  HVACSchedule,
  LightingStatus,
  LightingControlCommand,
  LightingSchedule,
  DaylightHarvesting,
  AccessControlStatus,
  AccessEvent,
  AccessControlCommand,
  SecurityZone,
  EnergyConsumption,
  EnergyOptimization,
  DemandResponseEvent,
  OccupancyStatus,
  OccupancySensor,
  EnvironmentalSensor,
  AirQuality,
  BACnetDevice,
  BACnetObject,
  ModbusDevice,
  ModbusRegister,
  Alarm,
  EventLog,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
  AlarmSeverity,
  AlarmType,
  HVACMode,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIASmartBuildingConfig {
  apiKey: string;
  endpoint: string;
  buildingId?: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// Event Handler Types
// ============================================================================

export type AlarmHandler = (alarm: Alarm) => void;
export type OccupancyHandler = (occupancy: OccupancyStatus) => void;
export type EnergyHandler = (energy: EnergyConsumption) => void;
export type AccessHandler = (event: AccessEvent) => void;

// ============================================================================
// WIA Smart Building SDK Client
// ============================================================================

export class WIASmartBuilding {
  private config: Required<WIASmartBuildingConfig>;
  private headers: Record<string, string>;
  private eventHandlers: Map<string, Set<Function>>;

  constructor(config: WIASmartBuildingConfig) {
    this.config = {
      buildingId: '',
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'SMART-BUILDING',
      'X-WIA-Version': '1.0.0',
    };

    this.eventHandlers = new Map();
  }

  // ==========================================================================
  // Building Information APIs
  // ==========================================================================

  /**
   * Get building information
   */
  async getBuildingInfo(buildingId?: string): Promise<ApiResponse<BuildingInfo>> {
    const id = buildingId || this.config.buildingId;
    return this.get<BuildingInfo>(`/api/v1/buildings/${id}`);
  }

  /**
   * Get building dashboard
   */
  async getBuildingDashboard(buildingId?: string): Promise<ApiResponse<BuildingDashboard>> {
    const id = buildingId || this.config.buildingId;
    return this.get<BuildingDashboard>(`/api/v1/buildings/${id}/dashboard`);
  }

  /**
   * List all buildings
   */
  async listBuildings(
    params?: PaginationParams
  ): Promise<ApiResponse<PaginatedResponse<BuildingInfo>>> {
    const queryParams = this.buildQueryParams(params);
    return this.get<PaginatedResponse<BuildingInfo>>(`/api/v1/buildings${queryParams}`);
  }

  // ==========================================================================
  // Zone Management APIs
  // ==========================================================================

  /**
   * Get zone information
   */
  async getZone(zoneId: string): Promise<ApiResponse<Zone>> {
    return this.get<Zone>(`/api/v1/zones/${zoneId}`);
  }

  /**
   * List all zones in building
   */
  async listZones(buildingId?: string): Promise<ApiResponse<Zone[]>> {
    const id = buildingId || this.config.buildingId;
    return this.get<Zone[]>(`/api/v1/zones?buildingId=${id}`);
  }

  /**
   * Update zone information
   */
  async updateZone(zoneId: string, updates: Partial<Zone>): Promise<ApiResponse<Zone>> {
    return this.put<Zone>(`/api/v1/zones/${zoneId}`, updates);
  }

  // ==========================================================================
  // HVAC Control and Scheduling APIs
  // ==========================================================================

  /**
   * Get HVAC status for zone
   */
  async getHVACStatus(zoneId: string): Promise<ApiResponse<HVACStatus>> {
    return this.get<HVACStatus>(`/api/v1/hvac/zones/${zoneId}/status`);
  }

  /**
   * Get HVAC status for all zones
   */
  async getAllHVACStatus(buildingId?: string): Promise<ApiResponse<HVACStatus[]>> {
    const id = buildingId || this.config.buildingId;
    return this.get<HVACStatus[]>(`/api/v1/hvac/status?buildingId=${id}`);
  }

  /**
   * Set HVAC temperature and mode
   */
  async setHVAC(command: HVACControlCommand): Promise<ApiResponse<void>> {
    return this.post<void>('/api/v1/hvac/control', command);
  }

  /**
   * Set HVAC temperature for zone
   */
  async setTemperature(
    zoneId: string,
    temperature: number,
    mode?: HVACMode
  ): Promise<ApiResponse<void>> {
    return this.setHVAC({
      zoneId,
      setpointTemperature: temperature,
      mode: mode || HVACMode.AUTO,
    });
  }

  /**
   * Get HVAC schedule
   */
  async getHVACSchedule(zoneId: string): Promise<ApiResponse<HVACSchedule>> {
    return this.get<HVACSchedule>(`/api/v1/hvac/zones/${zoneId}/schedule`);
  }

  /**
   * Update HVAC schedule
   */
  async updateHVACSchedule(
    zoneId: string,
    schedule: Partial<HVACSchedule>
  ): Promise<ApiResponse<HVACSchedule>> {
    return this.put<HVACSchedule>(`/api/v1/hvac/zones/${zoneId}/schedule`, schedule);
  }

  /**
   * Enable/disable HVAC schedule
   */
  async toggleHVACSchedule(
    scheduleId: string,
    enabled: boolean
  ): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/hvac/schedules/${scheduleId}/toggle`, { enabled });
  }

  // ==========================================================================
  // Lighting Control APIs
  // ==========================================================================

  /**
   * Get lighting status for zone
   */
  async getLightingStatus(zoneId: string): Promise<ApiResponse<LightingStatus>> {
    return this.get<LightingStatus>(`/api/v1/lighting/zones/${zoneId}/status`);
  }

  /**
   * Get lighting status for all zones
   */
  async getAllLightingStatus(buildingId?: string): Promise<ApiResponse<LightingStatus[]>> {
    const id = buildingId || this.config.buildingId;
    return this.get<LightingStatus[]>(`/api/v1/lighting/status?buildingId=${id}`);
  }

  /**
   * Control lighting
   */
  async setLighting(command: LightingControlCommand): Promise<ApiResponse<void>> {
    return this.post<void>('/api/v1/lighting/control', command);
  }

  /**
   * Turn lights on
   */
  async lightsOn(zoneId: string, brightness?: number): Promise<ApiResponse<void>> {
    return this.setLighting({
      zoneId,
      action: 'on',
      brightness,
    });
  }

  /**
   * Turn lights off
   */
  async lightsOff(zoneId: string): Promise<ApiResponse<void>> {
    return this.setLighting({
      zoneId,
      action: 'off',
    });
  }

  /**
   * Dim lights
   */
  async dimLights(zoneId: string, brightness: number): Promise<ApiResponse<void>> {
    return this.setLighting({
      zoneId,
      action: 'dim',
      brightness,
    });
  }

  /**
   * Get lighting schedule
   */
  async getLightingSchedule(zoneId: string): Promise<ApiResponse<LightingSchedule>> {
    return this.get<LightingSchedule>(`/api/v1/lighting/zones/${zoneId}/schedule`);
  }

  /**
   * Update lighting schedule
   */
  async updateLightingSchedule(
    zoneId: string,
    schedule: Partial<LightingSchedule>
  ): Promise<ApiResponse<LightingSchedule>> {
    return this.put<LightingSchedule>(`/api/v1/lighting/zones/${zoneId}/schedule`, schedule);
  }

  /**
   * Get daylight harvesting configuration
   */
  async getDaylightHarvesting(zoneId: string): Promise<ApiResponse<DaylightHarvesting>> {
    return this.get<DaylightHarvesting>(`/api/v1/lighting/zones/${zoneId}/daylight`);
  }

  /**
   * Update daylight harvesting configuration
   */
  async updateDaylightHarvesting(
    zoneId: string,
    config: Partial<DaylightHarvesting>
  ): Promise<ApiResponse<DaylightHarvesting>> {
    return this.put<DaylightHarvesting>(`/api/v1/lighting/zones/${zoneId}/daylight`, config);
  }

  // ==========================================================================
  // Energy Monitoring and Optimization APIs
  // ==========================================================================

  /**
   * Get current energy consumption
   */
  async getEnergyConsumption(
    buildingId?: string,
    zoneId?: string
  ): Promise<ApiResponse<EnergyConsumption>> {
    const id = buildingId || this.config.buildingId;
    const params = zoneId ? `?zoneId=${zoneId}` : '';
    return this.get<EnergyConsumption>(`/api/v1/energy/${id}/current${params}`);
  }

  /**
   * Get historical energy consumption
   */
  async getEnergyHistory(
    buildingId: string,
    dateRange: DateRangeFilter,
    interval: '1min' | '5min' | '15min' | '1hour' | '1day' = '1hour'
  ): Promise<ApiResponse<EnergyConsumption[]>> {
    const queryParams = new URLSearchParams({
      startDate: dateRange.startDate,
      endDate: dateRange.endDate,
      interval,
    });
    return this.get<EnergyConsumption[]>(
      `/api/v1/energy/${buildingId}/history?${queryParams.toString()}`
    );
  }

  /**
   * Get energy optimization configuration
   */
  async getEnergyOptimization(
    buildingId?: string
  ): Promise<ApiResponse<EnergyOptimization>> {
    const id = buildingId || this.config.buildingId;
    return this.get<EnergyOptimization>(`/api/v1/energy/${id}/optimization`);
  }

  /**
   * Update energy optimization configuration
   */
  async updateEnergyOptimization(
    buildingId: string,
    config: Partial<EnergyOptimization>
  ): Promise<ApiResponse<EnergyOptimization>> {
    return this.put<EnergyOptimization>(`/api/v1/energy/${buildingId}/optimization`, config);
  }

  /**
   * Get demand response events
   */
  async getDemandResponseEvents(
    buildingId: string,
    dateRange?: DateRangeFilter
  ): Promise<ApiResponse<DemandResponseEvent[]>> {
    const queryParams = dateRange ? this.buildQueryParams(dateRange) : '';
    return this.get<DemandResponseEvent[]>(
      `/api/v1/energy/${buildingId}/demand-response${queryParams}`
    );
  }

  // ==========================================================================
  // Occupancy Tracking APIs
  // ==========================================================================

  /**
   * Get occupancy status for zone
   */
  async getOccupancy(zoneId: string): Promise<ApiResponse<OccupancyStatus>> {
    return this.get<OccupancyStatus>(`/api/v1/occupancy/zones/${zoneId}`);
  }

  /**
   * Get occupancy status for all zones
   */
  async getAllOccupancy(buildingId?: string): Promise<ApiResponse<OccupancyStatus[]>> {
    const id = buildingId || this.config.buildingId;
    return this.get<OccupancyStatus[]>(`/api/v1/occupancy?buildingId=${id}`);
  }

  /**
   * Get occupancy sensors
   */
  async getOccupancySensors(zoneId: string): Promise<ApiResponse<OccupancySensor[]>> {
    return this.get<OccupancySensor[]>(`/api/v1/occupancy/zones/${zoneId}/sensors`);
  }

  /**
   * Get air quality
   */
  async getAirQuality(zoneId: string): Promise<ApiResponse<AirQuality>> {
    return this.get<AirQuality>(`/api/v1/air-quality/zones/${zoneId}`);
  }

  /**
   * Get environmental sensors
   */
  async getEnvironmentalSensors(zoneId: string): Promise<ApiResponse<EnvironmentalSensor[]>> {
    return this.get<EnvironmentalSensor[]>(`/api/v1/sensors/zones/${zoneId}/environmental`);
  }

  // ==========================================================================
  // Access Control APIs
  // ==========================================================================

  /**
   * Get access control status
   */
  async getAccessControlStatus(zoneId: string): Promise<ApiResponse<AccessControlStatus>> {
    return this.get<AccessControlStatus>(`/api/v1/access/zones/${zoneId}/status`);
  }

  /**
   * Control access point
   */
  async controlAccess(command: AccessControlCommand): Promise<ApiResponse<void>> {
    return this.post<void>('/api/v1/access/control', command);
  }

  /**
   * Lock access point
   */
  async lock(accessPointId: string): Promise<ApiResponse<void>> {
    return this.controlAccess({ accessPointId, action: 'lock' });
  }

  /**
   * Unlock access point
   */
  async unlock(accessPointId: string, duration?: number): Promise<ApiResponse<void>> {
    return this.controlAccess({ accessPointId, action: 'unlock', duration });
  }

  /**
   * Get access events
   */
  async getAccessEvents(
    buildingId: string,
    dateRange?: DateRangeFilter
  ): Promise<ApiResponse<AccessEvent[]>> {
    const queryParams = dateRange ? this.buildQueryParams(dateRange) : '';
    return this.get<AccessEvent[]>(`/api/v1/access/${buildingId}/events${queryParams}`);
  }

  /**
   * Get security zones
   */
  async getSecurityZones(buildingId?: string): Promise<ApiResponse<SecurityZone[]>> {
    const id = buildingId || this.config.buildingId;
    return this.get<SecurityZone[]>(`/api/v1/security/${id}/zones`);
  }

  // ==========================================================================
  // Alarm and Event Management APIs
  // ==========================================================================

  /**
   * Get active alarms
   */
  async getActiveAlarms(buildingId?: string): Promise<ApiResponse<Alarm[]>> {
    const id = buildingId || this.config.buildingId;
    return this.get<Alarm[]>(`/api/v1/alarms/${id}/active`);
  }

  /**
   * Get alarm history
   */
  async getAlarmHistory(
    buildingId: string,
    dateRange: DateRangeFilter
  ): Promise<ApiResponse<Alarm[]>> {
    const queryParams = this.buildQueryParams(dateRange);
    return this.get<Alarm[]>(`/api/v1/alarms/${buildingId}/history${queryParams}`);
  }

  /**
   * Acknowledge alarm
   */
  async acknowledgeAlarm(alarmId: string, userId?: string): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/alarms/${alarmId}/acknowledge`, { userId });
  }

  /**
   * Resolve alarm
   */
  async resolveAlarm(alarmId: string): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/alarms/${alarmId}/resolve`, {});
  }

  /**
   * Get event logs
   */
  async getEventLogs(
    buildingId: string,
    dateRange?: DateRangeFilter,
    category?: string
  ): Promise<ApiResponse<EventLog[]>> {
    const params: any = dateRange || {};
    if (category) params.category = category;
    const queryParams = this.buildQueryParams(params);
    return this.get<EventLog[]>(`/api/v1/events/${buildingId}${queryParams}`);
  }

  // ==========================================================================
  // Building Automation Protocol APIs
  // ==========================================================================

  /**
   * Get BACnet devices
   */
  async getBACnetDevices(buildingId?: string): Promise<ApiResponse<BACnetDevice[]>> {
    const id = buildingId || this.config.buildingId;
    return this.get<BACnetDevice[]>(`/api/v1/bacnet/${id}/devices`);
  }

  /**
   * Get BACnet object value
   */
  async getBACnetObject(
    deviceId: number,
    objectType: string,
    objectInstance: number
  ): Promise<ApiResponse<BACnetObject>> {
    return this.get<BACnetObject>(
      `/api/v1/bacnet/devices/${deviceId}/objects/${objectType}/${objectInstance}`
    );
  }

  /**
   * Write BACnet object value
   */
  async writeBACnetObject(
    deviceId: number,
    objectType: string,
    objectInstance: number,
    value: any
  ): Promise<ApiResponse<void>> {
    return this.post<void>(
      `/api/v1/bacnet/devices/${deviceId}/objects/${objectType}/${objectInstance}`,
      { value }
    );
  }

  /**
   * Get Modbus devices
   */
  async getModbusDevices(buildingId?: string): Promise<ApiResponse<ModbusDevice[]>> {
    const id = buildingId || this.config.buildingId;
    return this.get<ModbusDevice[]>(`/api/v1/modbus/${id}/devices`);
  }

  /**
   * Read Modbus register
   */
  async readModbusRegister(
    deviceId: string,
    address: number
  ): Promise<ApiResponse<ModbusRegister>> {
    return this.get<ModbusRegister>(`/api/v1/modbus/devices/${deviceId}/registers/${address}`);
  }

  /**
   * Write Modbus register
   */
  async writeModbusRegister(
    deviceId: string,
    address: number,
    value: number
  ): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/modbus/devices/${deviceId}/registers/${address}`, { value });
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  /**
   * Subscribe to alarms
   */
  onAlarm(handler: AlarmHandler): () => void {
    return this.addEventListener('alarm', handler);
  }

  /**
   * Subscribe to occupancy changes
   */
  onOccupancy(handler: OccupancyHandler): () => void {
    return this.addEventListener('occupancy', handler);
  }

  /**
   * Subscribe to energy updates
   */
  onEnergy(handler: EnergyHandler): () => void {
    return this.addEventListener('energy', handler);
  }

  /**
   * Subscribe to access events
   */
  onAccess(handler: AccessHandler): () => void {
    return this.addEventListener('access', handler);
  }

  /**
   * Add event listener
   */
  private addEventListener(eventType: string, handler: Function): () => void {
    if (!this.eventHandlers.has(eventType)) {
      this.eventHandlers.set(eventType, new Set());
    }
    this.eventHandlers.get(eventType)!.add(handler);

    // Return unsubscribe function
    return () => {
      const handlers = this.eventHandlers.get(eventType);
      if (handlers) {
        handlers.delete(handler);
      }
    };
  }

  /**
   * Emit event to handlers
   */
  private emitEvent(eventType: string, data: any): void {
    const handlers = this.eventHandlers.get(eventType);
    if (handlers) {
      handlers.forEach((handler) => handler(data));
    }
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

  private async request<T>(method: string, path: string, body?: any): Promise<ApiResponse<T>> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA-SMART-BUILDING] ${method} ${url}`, body || '');
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
        console.error('[WIA-SMART-BUILDING] Request failed:', error);
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
// Factory Function
// ============================================================================

/**
 * Create a new WIA Smart Building SDK instance
 */
export function createSmartBuildingSDK(config: WIASmartBuildingConfig): WIASmartBuilding {
  return new WIASmartBuilding(config);
}

// ============================================================================
// Default Export
// ============================================================================

export default WIASmartBuilding;
