/**
 * WIA-CITY-013: Fire Safety System Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import type {
  ApiResponse,
  FireSensor,
  SmokeDetector,
  HeatDetector,
  FlameDetector,
  CODetector,
  SprinklerSystem,
  SprinklerHead,
  FireAlarm,
  Notification,
  EvacuationRoute,
  EmergencyExit,
  FireExtinguisher,
  FireHydrant,
  FireDoor,
  FireZone,
  FireIncident,
  FireDetectionEvent,
  SystemHealthCheck,
  EvacuationStatus,
  HVACStatus,
  ElevatorStatus,
  BMSIntegration,
  Timestamp,
  Location,
} from './types';

/**
 * Client configuration
 */
export interface FireSafetyClientConfig {
  apiKey: string;
  endpoint?: string;
  timeout?: number;
  headers?: Record<string, string>;
}

/**
 * Default configuration
 */
const DEFAULT_CONFIG = {
  endpoint: 'https://api.wia.org/city-013/v1',
  timeout: 30000,
};

/**
 * Fire Safety System Client
 *
 * @example
 * ```typescript
 * const client = new FireSafetySDK({
 *   apiKey: 'your-api-key',
 *   endpoint: 'https://api.wia.org/city-013/v1'
 * });
 *
 * // Detect fire
 * const event = await client.detectFire({
 *   sensorId: 'SMOKE-FL2-A001',
 *   sensorType: 'SMOKE',
 *   location: { lat: 37.5665, lon: 126.9780, floor: '2F', zone: 'A' },
 *   readings: {
 *     smokeLevel_ppm: 250,
 *     temperature_C: 85
 *   }
 * });
 *
 * console.log('Fire detected:', event.eventId);
 * ```
 */
export class FireSafetySDK {
  private config: Required<FireSafetyClientConfig>;

  constructor(config: FireSafetyClientConfig) {
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
  // Fire Detection
  // ============================================================================

  /**
   * Report fire detection from sensor
   *
   * @param event - Fire detection event data
   * @returns Fire detection event with ID
   */
  async detectFire(
    event: Omit<FireDetectionEvent, 'eventId'>
  ): Promise<FireDetectionEvent> {
    const response = await this.post<FireDetectionEvent>(
      '/fire/detect',
      event
    );
    return response;
  }

  /**
   * Get sensor status
   *
   * @param sensorId - Sensor ID
   * @returns Sensor information
   */
  async getSensor(sensorId: string): Promise<FireSensor> {
    const response = await this.get<FireSensor>(`/sensors/${sensorId}`);
    return response;
  }

  /**
   * Get all sensors in a zone
   *
   * @param zoneId - Zone ID
   * @returns List of sensors
   */
  async getSensorsByZone(zoneId: string): Promise<FireSensor[]> {
    const response = await this.get<FireSensor[]>(
      `/zones/${zoneId}/sensors`
    );
    return response;
  }

  /**
   * Update sensor status
   *
   * @param sensorId - Sensor ID
   * @param status - New status
   * @returns Updated sensor
   */
  async updateSensorStatus(
    sensorId: string,
    status: any
  ): Promise<FireSensor> {
    const response = await this.put<FireSensor>(
      `/sensors/${sensorId}/status`,
      { status }
    );
    return response;
  }

  // ============================================================================
  // Sprinkler System
  // ============================================================================

  /**
   * Activate sprinkler system
   *
   * @param systemId - Sprinkler system ID
   * @param zoneId - Optional zone ID to activate specific zone
   * @returns Activation confirmation
   */
  async activateSprinkler(
    systemId: string,
    zoneId?: string
  ): Promise<{
    systemId: string;
    activated: boolean;
    activatedHeads: string[];
    timestamp: Timestamp;
  }> {
    const response = await this.post<{
      systemId: string;
      activated: boolean;
      activatedHeads: string[];
      timestamp: Timestamp;
    }>('/sprinklers/activate', { systemId, zoneId });
    return response;
  }

  /**
   * Get sprinkler system status
   *
   * @param systemId - Sprinkler system ID
   * @returns Sprinkler system information
   */
  async getSprinklerSystem(systemId: string): Promise<SprinklerSystem> {
    const response = await this.get<SprinklerSystem>(
      `/sprinklers/${systemId}`
    );
    return response;
  }

  /**
   * Deactivate sprinkler system
   *
   * @param systemId - Sprinkler system ID
   * @returns Deactivation confirmation
   */
  async deactivateSprinkler(
    systemId: string
  ): Promise<{
    systemId: string;
    deactivated: boolean;
    timestamp: Timestamp;
  }> {
    const response = await this.post<{
      systemId: string;
      deactivated: boolean;
      timestamp: Timestamp;
    }>('/sprinklers/deactivate', { systemId });
    return response;
  }

  // ============================================================================
  // Fire Alarm System
  // ============================================================================

  /**
   * Trigger fire alarm
   *
   * @param alarm - Alarm data
   * @returns Triggered alarm
   */
  async triggerAlarm(
    alarm: Omit<FireAlarm, 'alarmId' | 'triggeredAt' | 'acknowledged' | 'silenced'>
  ): Promise<FireAlarm> {
    const response = await this.post<FireAlarm>('/alarms/trigger', alarm);
    return response;
  }

  /**
   * Acknowledge alarm
   *
   * @param alarmId - Alarm ID
   * @param acknowledgedBy - Person acknowledging the alarm
   * @returns Updated alarm
   */
  async acknowledgeAlarm(
    alarmId: string,
    acknowledgedBy: string
  ): Promise<FireAlarm> {
    const response = await this.post<FireAlarm>(
      `/alarms/${alarmId}/acknowledge`,
      { acknowledgedBy }
    );
    return response;
  }

  /**
   * Silence alarm
   *
   * @param alarmId - Alarm ID
   * @returns Updated alarm
   */
  async silenceAlarm(alarmId: string): Promise<FireAlarm> {
    const response = await this.post<FireAlarm>(
      `/alarms/${alarmId}/silence`,
      {}
    );
    return response;
  }

  /**
   * Get active alarms
   *
   * @returns List of active alarms
   */
  async getActiveAlarms(): Promise<FireAlarm[]> {
    const response = await this.get<FireAlarm[]>('/alarms/active');
    return response;
  }

  /**
   * Send notification
   *
   * @param notification - Notification data
   * @returns Sent notification
   */
  async sendNotification(
    notification: Omit<Notification, 'notificationId' | 'sentAt' | 'delivered'>
  ): Promise<Notification> {
    const response = await this.post<Notification>(
      '/notifications/send',
      notification
    );
    return response;
  }

  // ============================================================================
  // Evacuation Management
  // ============================================================================

  /**
   * Get evacuation route
   *
   * @param startLocation - Starting location
   * @param accessible - Whether route must be accessible
   * @returns Best evacuation route
   */
  async getEvacuationRoute(
    startLocation: Location,
    accessible: boolean = false
  ): Promise<EvacuationRoute> {
    const response = await this.post<EvacuationRoute>(
      '/evacuation/route',
      { startLocation, accessible }
    );
    return response;
  }

  /**
   * Initiate evacuation
   *
   * @param zoneIds - Zones to evacuate
   * @returns Evacuation status
   */
  async initiateEvacuation(zoneIds: string[]): Promise<EvacuationStatus> {
    const response = await this.post<EvacuationStatus>(
      '/evacuation/initiate',
      { zoneIds }
    );
    return response;
  }

  /**
   * Get evacuation status
   *
   * @returns Current evacuation status
   */
  async getEvacuationStatus(): Promise<EvacuationStatus> {
    const response = await this.get<EvacuationStatus>('/evacuation/status');
    return response;
  }

  /**
   * Update exit status
   *
   * @param exitId - Exit ID
   * @param status - New status
   * @returns Updated exit
   */
  async updateExitStatus(
    exitId: string,
    status: any
  ): Promise<EmergencyExit> {
    const response = await this.put<EmergencyExit>(
      `/exits/${exitId}/status`,
      { status }
    );
    return response;
  }

  // ============================================================================
  // Equipment Management
  // ============================================================================

  /**
   * Check fire extinguisher
   *
   * @param extinguisherId - Extinguisher ID
   * @returns Extinguisher information
   */
  async checkExtinguisher(
    extinguisherId: string
  ): Promise<FireExtinguisher> {
    const response = await this.get<FireExtinguisher>(
      `/equipment/extinguishers/${extinguisherId}`
    );
    return response;
  }

  /**
   * Get nearby extinguishers
   *
   * @param location - Current location
   * @param radius_m - Search radius in meters
   * @returns List of nearby extinguishers
   */
  async getNearbyExtinguishers(
    location: Location,
    radius_m: number = 50
  ): Promise<FireExtinguisher[]> {
    const response = await this.post<FireExtinguisher[]>(
      '/equipment/extinguishers/nearby',
      { location, radius_m }
    );
    return response;
  }

  /**
   * Record extinguisher inspection
   *
   * @param extinguisherId - Extinguisher ID
   * @param inspector - Inspector name
   * @param notes - Inspection notes
   * @returns Updated extinguisher
   */
  async inspectExtinguisher(
    extinguisherId: string,
    inspector: string,
    notes?: string
  ): Promise<FireExtinguisher> {
    const response = await this.post<FireExtinguisher>(
      `/equipment/extinguishers/${extinguisherId}/inspect`,
      { inspector, notes }
    );
    return response;
  }

  /**
   * Get fire hydrant status
   *
   * @param hydrantId - Hydrant ID
   * @returns Hydrant information
   */
  async getHydrant(hydrantId: string): Promise<FireHydrant> {
    const response = await this.get<FireHydrant>(
      `/equipment/hydrants/${hydrantId}`
    );
    return response;
  }

  // ============================================================================
  // Fire Doors & Zones
  // ============================================================================

  /**
   * Control fire door
   *
   * @param doorId - Fire door ID
   * @param action - Action to perform (open/close)
   * @returns Updated door status
   */
  async controlFireDoor(
    doorId: string,
    action: 'OPEN' | 'CLOSE' | 'AUTO'
  ): Promise<FireDoor> {
    const response = await this.post<FireDoor>(
      `/doors/${doorId}/control`,
      { action }
    );
    return response;
  }

  /**
   * Get fire zone information
   *
   * @param zoneId - Zone ID
   * @returns Zone information
   */
  async getZone(zoneId: string): Promise<FireZone> {
    const response = await this.get<FireZone>(`/zones/${zoneId}`);
    return response;
  }

  /**
   * Get all zones in building
   *
   * @param buildingId - Building ID
   * @returns List of zones
   */
  async getZonesByBuilding(buildingId: string): Promise<FireZone[]> {
    const response = await this.get<FireZone[]>(
      `/buildings/${buildingId}/zones`
    );
    return response;
  }

  // ============================================================================
  // Incident Management
  // ============================================================================

  /**
   * Create fire incident
   *
   * @param incident - Incident data
   * @returns Created incident
   */
  async createIncident(
    incident: Omit<FireIncident, 'incidentId' | 'detectedAt'>
  ): Promise<FireIncident> {
    const response = await this.post<FireIncident>(
      '/incidents/create',
      incident
    );
    return response;
  }

  /**
   * Get incident details
   *
   * @param incidentId - Incident ID
   * @returns Incident information
   */
  async getIncident(incidentId: string): Promise<FireIncident> {
    const response = await this.get<FireIncident>(
      `/incidents/${incidentId}`
    );
    return response;
  }

  /**
   * Update incident status
   *
   * @param incidentId - Incident ID
   * @param status - New status
   * @param notes - Optional notes
   * @returns Updated incident
   */
  async updateIncident(
    incidentId: string,
    status: any,
    notes?: string
  ): Promise<FireIncident> {
    const response = await this.put<FireIncident>(
      `/incidents/${incidentId}`,
      { status, notes }
    );
    return response;
  }

  /**
   * Get incident history
   *
   * @param startDate - Start date
   * @param endDate - End date
   * @returns List of incidents
   */
  async getIncidentHistory(
    startDate: Timestamp,
    endDate: Timestamp
  ): Promise<FireIncident[]> {
    const response = await this.get<FireIncident[]>(
      `/incidents/history?start=${startDate}&end=${endDate}`
    );
    return response;
  }

  // ============================================================================
  // System Integration
  // ============================================================================

  /**
   * Control HVAC system
   *
   * @param systemId - HVAC system ID
   * @param action - Action to perform
   * @returns Updated HVAC status
   */
  async controlHVAC(
    systemId: string,
    action: 'SHUTDOWN' | 'SMOKE_CONTROL' | 'PRESSURIZE' | 'NORMAL'
  ): Promise<HVACStatus> {
    const response = await this.post<HVACStatus>(
      '/integration/hvac/control',
      { systemId, action }
    );
    return response;
  }

  /**
   * Control elevator
   *
   * @param elevatorId - Elevator ID
   * @param action - Action to perform
   * @param targetFloor - Target floor for recall
   * @returns Updated elevator status
   */
  async controlElevator(
    elevatorId: string,
    action: 'FIRE_MODE' | 'EMERGENCY_STOP' | 'NORMAL',
    targetFloor?: string
  ): Promise<ElevatorStatus> {
    const response = await this.post<ElevatorStatus>(
      '/integration/elevators/control',
      { elevatorId, action, targetFloor }
    );
    return response;
  }

  /**
   * Get BMS integration status
   *
   * @param bmsId - BMS ID
   * @returns BMS integration status
   */
  async getBMSStatus(bmsId: string): Promise<BMSIntegration> {
    const response = await this.get<BMSIntegration>(
      `/integration/bms/${bmsId}`
    );
    return response;
  }

  // ============================================================================
  // System Monitoring
  // ============================================================================

  /**
   * Perform system health check
   *
   * @returns System health status
   */
  async healthCheck(): Promise<SystemHealthCheck> {
    const response = await this.get<SystemHealthCheck>('/system/health');
    return response;
  }

  /**
   * Subscribe to real-time events via WebSocket
   *
   * @param eventTypes - Types of events to subscribe to
   * @param onMessage - Callback for incoming events
   * @returns WebSocket connection
   */
  subscribeToEvents(
    eventTypes: string[],
    onMessage: (event: any) => void
  ): WebSocket {
    const wsEndpoint = this.config.endpoint
      .replace('http://', 'ws://')
      .replace('https://', 'wss://');
    const ws = new WebSocket(
      `${wsEndpoint}/events/subscribe?types=${eventTypes.join(',')}`
    );

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      onMessage(data);
    };

    ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    return ws;
  }

  /**
   * Get system statistics
   *
   * @param period - Time period (24h, 7d, 30d)
   * @returns System statistics
   */
  async getStatistics(
    period: '24h' | '7d' | '30d'
  ): Promise<{
    totalSensors: number;
    activeSensors: number;
    totalAlarms: number;
    falseAlarms: number;
    incidents: number;
    evacuations: number;
    avgResponseTime_sec: number;
  }> {
    const response = await this.get<{
      totalSensors: number;
      activeSensors: number;
      totalAlarms: number;
      falseAlarms: number;
      incidents: number;
      evacuations: number;
      avgResponseTime_sec: number;
    }>(`/system/statistics?period=${period}`);
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

export default FireSafetySDK;
