/**
 * WIA-CITY-009: Smart Lighting Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import type {
  LightingFixture,
  FixtureState,
  LightingScene,
  TimeBasedSchedule,
  AstronomicalSchedule,
  Sensor,
  Zone,
  EnergyDashboard,
  PowerBudget,
  LightingEvent,
  LightingAlert,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
  AutomationRule,
  CircadianLightingSchedule,
  DaylightHarvestingControl,
  PowerMeasurement,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface SmartLightingSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;                     // ms (default: 30000)
  retryAttempts?: number;               // default: 3
  retryDelay?: number;                  // ms (default: 1000)
}

// ============================================================================
// Main SDK Class
// ============================================================================

export class SmartLightingSDK {
  private config: Required<SmartLightingSDKConfig>;
  private baseUrl: string;

  // Sub-modules
  public fixtures: FixturesAPI;
  public scenes: ScenesAPI;
  public schedules: SchedulesAPI;
  public sensors: SensorsAPI;
  public zones: ZonesAPI;
  public energy: EnergyAPI;
  public automation: AutomationAPI;
  public events: EventsAPI;
  public alerts: AlertsAPI;

  constructor(config: SmartLightingSDKConfig) {
    this.config = {
      ...config,
      timeout: config.timeout || 30000,
      retryAttempts: config.retryAttempts || 3,
      retryDelay: config.retryDelay || 1000,
    };

    this.baseUrl = config.endpoint.replace(/\/$/, '');

    // Initialize sub-modules
    this.fixtures = new FixturesAPI(this);
    this.scenes = new ScenesAPI(this);
    this.schedules = new SchedulesAPI(this);
    this.sensors = new SensorsAPI(this);
    this.zones = new ZonesAPI(this);
    this.energy = new EnergyAPI(this);
    this.automation = new AutomationAPI(this);
    this.events = new EventsAPI(this);
    this.alerts = new AlertsAPI(this);
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
      'User-Agent': 'WIA-CITY-009-SDK/1.0.0',
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
// Fixtures API
// ============================================================================

class FixturesAPI {
  constructor(private sdk: SmartLightingSDK) {}

  /**
   * List all fixtures
   */
  async list(params?: {
    zone?: string;
    floor?: string;
    type?: string;
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<LightingFixture>>> {
    return this.sdk.request('GET', '/api/v1/fixtures', undefined, params);
  }

  /**
   * Get fixture by ID
   */
  async get(fixtureId: string): Promise<ApiResponse<LightingFixture>> {
    return this.sdk.request('GET', `/api/v1/fixtures/${fixtureId}`);
  }

  /**
   * Create new fixture
   */
  async create(fixture: Omit<LightingFixture, 'fixtureId'>): Promise<ApiResponse<{ fixtureId: string }>> {
    return this.sdk.request('POST', '/api/v1/fixtures', fixture);
  }

  /**
   * Update fixture
   */
  async update(fixtureId: string, fixture: Partial<LightingFixture>): Promise<ApiResponse<LightingFixture>> {
    return this.sdk.request('PUT', `/api/v1/fixtures/${fixtureId}`, fixture);
  }

  /**
   * Delete fixture
   */
  async delete(fixtureId: string): Promise<ApiResponse<void>> {
    return this.sdk.request('DELETE', `/api/v1/fixtures/${fixtureId}`);
  }

  /**
   * Set fixture state
   */
  async setState(
    fixtureId: string,
    state: {
      on?: boolean;
      brightness?: number;
      colorTemperature?: number;
      color?: { hue: number; saturation: number };
      fadeTime?: number;
    }
  ): Promise<ApiResponse<{ success: boolean; state: FixtureState }>> {
    return this.sdk.request('PUT', `/api/v1/fixtures/${fixtureId}/state`, state);
  }

  /**
   * Turn on fixture
   */
  async turnOn(fixtureId: string, fadeTime?: number): Promise<ApiResponse<{ success: boolean }>> {
    return this.setState(fixtureId, { on: true, fadeTime });
  }

  /**
   * Turn off fixture
   */
  async turnOff(fixtureId: string, fadeTime?: number): Promise<ApiResponse<{ success: boolean }>> {
    return this.setState(fixtureId, { on: false, fadeTime });
  }

  /**
   * Dim fixture
   */
  async dim(fixtureId: string, brightness: number, fadeTime?: number): Promise<ApiResponse<{ success: boolean }>> {
    return this.setState(fixtureId, { brightness, fadeTime });
  }

  /**
   * Set color temperature
   */
  async setColorTemperature(
    fixtureId: string,
    colorTemperature: number,
    fadeTime?: number
  ): Promise<ApiResponse<{ success: boolean }>> {
    return this.setState(fixtureId, { colorTemperature, fadeTime });
  }

  /**
   * Set color
   */
  async setColor(
    fixtureId: string,
    color: { hue: number; saturation: number },
    fadeTime?: number
  ): Promise<ApiResponse<{ success: boolean }>> {
    return this.setState(fixtureId, { color, fadeTime });
  }
}

// ============================================================================
// Scenes API
// ============================================================================

class ScenesAPI {
  constructor(private sdk: SmartLightingSDK) {}

  /**
   * List all scenes
   */
  async list(): Promise<ApiResponse<{ scenes: LightingScene[] }>> {
    return this.sdk.request('GET', '/api/v1/scenes');
  }

  /**
   * Get scene by ID
   */
  async get(sceneId: string): Promise<ApiResponse<LightingScene>> {
    return this.sdk.request('GET', `/api/v1/scenes/${sceneId}`);
  }

  /**
   * Create new scene
   */
  async create(scene: Omit<LightingScene, 'sceneId'>): Promise<ApiResponse<{ sceneId: string }>> {
    return this.sdk.request('POST', '/api/v1/scenes', scene);
  }

  /**
   * Update scene
   */
  async update(sceneId: string, scene: Partial<LightingScene>): Promise<ApiResponse<LightingScene>> {
    return this.sdk.request('PUT', `/api/v1/scenes/${sceneId}`, scene);
  }

  /**
   * Delete scene
   */
  async delete(sceneId: string): Promise<ApiResponse<void>> {
    return this.sdk.request('DELETE', `/api/v1/scenes/${sceneId}`);
  }

  /**
   * Activate scene
   */
  async activate(sceneId: string, fadeTime?: number): Promise<ApiResponse<{ success: boolean; activatedAt: string }>> {
    return this.sdk.request('POST', `/api/v1/scenes/${sceneId}/activate`, { fadeTime });
  }
}

// ============================================================================
// Schedules API
// ============================================================================

class SchedulesAPI {
  constructor(private sdk: SmartLightingSDK) {}

  /**
   * List all schedules
   */
  async list(): Promise<ApiResponse<{ schedules: (TimeBasedSchedule | AstronomicalSchedule)[] }>> {
    return this.sdk.request('GET', '/api/v1/schedules');
  }

  /**
   * Get schedule by ID
   */
  async get(scheduleId: string): Promise<ApiResponse<TimeBasedSchedule | AstronomicalSchedule>> {
    return this.sdk.request('GET', `/api/v1/schedules/${scheduleId}`);
  }

  /**
   * Create new schedule
   */
  async create(
    schedule: Omit<TimeBasedSchedule, 'scheduleId'> | Omit<AstronomicalSchedule, 'scheduleId'>
  ): Promise<ApiResponse<{ scheduleId: string }>> {
    return this.sdk.request('POST', '/api/v1/schedules', schedule);
  }

  /**
   * Update schedule
   */
  async update(
    scheduleId: string,
    schedule: Partial<TimeBasedSchedule | AstronomicalSchedule>
  ): Promise<ApiResponse<TimeBasedSchedule | AstronomicalSchedule>> {
    return this.sdk.request('PUT', `/api/v1/schedules/${scheduleId}`, schedule);
  }

  /**
   * Delete schedule
   */
  async delete(scheduleId: string): Promise<ApiResponse<void>> {
    return this.sdk.request('DELETE', `/api/v1/schedules/${scheduleId}`);
  }

  /**
   * Enable/disable schedule
   */
  async setEnabled(scheduleId: string, enabled: boolean): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('PUT', `/api/v1/schedules/${scheduleId}/enabled`, { enabled });
  }
}

// ============================================================================
// Sensors API
// ============================================================================

class SensorsAPI {
  constructor(private sdk: SmartLightingSDK) {}

  /**
   * List all sensors
   */
  async list(params?: {
    zone?: string;
    type?: string;
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<Sensor>>> {
    return this.sdk.request('GET', '/api/v1/sensors', undefined, params);
  }

  /**
   * Get sensor by ID
   */
  async get(sensorId: string): Promise<ApiResponse<Sensor>> {
    return this.sdk.request('GET', `/api/v1/sensors/${sensorId}`);
  }

  /**
   * Get sensor measurements (time series)
   */
  async getMeasurements(
    sensorId: string,
    params: {
      start: string;
      end: string;
      interval?: '1m' | '5m' | '15m' | '1h' | '1d';
    }
  ): Promise<ApiResponse<{ measurements: { timestamp: string; value: number }[] }>> {
    return this.sdk.request('GET', `/api/v1/sensors/${sensorId}/measurements`, undefined, params);
  }

  /**
   * Create new sensor
   */
  async create(sensor: Omit<Sensor, 'sensorId'>): Promise<ApiResponse<{ sensorId: string }>> {
    return this.sdk.request('POST', '/api/v1/sensors', sensor);
  }

  /**
   * Update sensor
   */
  async update(sensorId: string, sensor: Partial<Sensor>): Promise<ApiResponse<Sensor>> {
    return this.sdk.request('PUT', `/api/v1/sensors/${sensorId}`, sensor);
  }

  /**
   * Delete sensor
   */
  async delete(sensorId: string): Promise<ApiResponse<void>> {
    return this.sdk.request('DELETE', `/api/v1/sensors/${sensorId}`);
  }
}

// ============================================================================
// Zones API
// ============================================================================

class ZonesAPI {
  constructor(private sdk: SmartLightingSDK) {}

  /**
   * List all zones
   */
  async list(params?: {
    building?: string;
    floor?: string;
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<Zone>>> {
    return this.sdk.request('GET', '/api/v1/zones', undefined, params);
  }

  /**
   * Get zone by ID
   */
  async get(zoneId: string): Promise<ApiResponse<Zone>> {
    return this.sdk.request('GET', `/api/v1/zones/${zoneId}`);
  }

  /**
   * Create new zone
   */
  async create(zone: Omit<Zone, 'zoneId'>): Promise<ApiResponse<{ zoneId: string }>> {
    return this.sdk.request('POST', '/api/v1/zones', zone);
  }

  /**
   * Update zone
   */
  async update(zoneId: string, zone: Partial<Zone>): Promise<ApiResponse<Zone>> {
    return this.sdk.request('PUT', `/api/v1/zones/${zoneId}`, zone);
  }

  /**
   * Delete zone
   */
  async delete(zoneId: string): Promise<ApiResponse<void>> {
    return this.sdk.request('DELETE', `/api/v1/zones/${zoneId}`);
  }
}

// ============================================================================
// Energy API
// ============================================================================

class EnergyAPI {
  constructor(private sdk: SmartLightingSDK) {}

  /**
   * Get energy dashboard
   */
  async getDashboard(params: {
    start: string;
    end: string;
    zone?: string;
  }): Promise<ApiResponse<EnergyDashboard>> {
    return this.sdk.request('GET', '/api/v1/energy/dashboard', undefined, params);
  }

  /**
   * Get energy report
   */
  async getReport(params: {
    type: 'daily' | 'weekly' | 'monthly';
    date: string;
  }): Promise<ApiResponse<{ report: any }>> {
    return this.sdk.request('GET', '/api/v1/energy/report', undefined, params);
  }

  /**
   * Get power budget
   */
  async getBudget(zoneId: string): Promise<ApiResponse<PowerBudget>> {
    return this.sdk.request('GET', `/api/v1/energy/budget/${zoneId}`);
  }

  /**
   * Set power budget
   */
  async setBudget(zoneId: string, budget: Partial<PowerBudget>): Promise<ApiResponse<PowerBudget>> {
    return this.sdk.request('PUT', `/api/v1/energy/budget/${zoneId}`, budget);
  }

  /**
   * Get power measurements
   */
  async getMeasurements(params: {
    fixtureId?: string;
    zoneId?: string;
    start: string;
    end: string;
    interval?: '1m' | '5m' | '15m' | '1h' | '1d';
  }): Promise<ApiResponse<{ measurements: PowerMeasurement[] }>> {
    return this.sdk.request('GET', '/api/v1/energy/measurements', undefined, params);
  }
}

// ============================================================================
// Automation API
// ============================================================================

class AutomationAPI {
  constructor(private sdk: SmartLightingSDK) {}

  /**
   * List all automation rules
   */
  async list(): Promise<ApiResponse<{ rules: AutomationRule[] }>> {
    return this.sdk.request('GET', '/api/v1/automation/rules');
  }

  /**
   * Get automation rule by ID
   */
  async get(ruleId: string): Promise<ApiResponse<AutomationRule>> {
    return this.sdk.request('GET', `/api/v1/automation/rules/${ruleId}`);
  }

  /**
   * Create new automation rule
   */
  async create(rule: Omit<AutomationRule, 'ruleId'>): Promise<ApiResponse<{ ruleId: string }>> {
    return this.sdk.request('POST', '/api/v1/automation/rules', rule);
  }

  /**
   * Update automation rule
   */
  async update(ruleId: string, rule: Partial<AutomationRule>): Promise<ApiResponse<AutomationRule>> {
    return this.sdk.request('PUT', `/api/v1/automation/rules/${ruleId}`, rule);
  }

  /**
   * Delete automation rule
   */
  async delete(ruleId: string): Promise<ApiResponse<void>> {
    return this.sdk.request('DELETE', `/api/v1/automation/rules/${ruleId}`);
  }

  /**
   * Enable/disable automation rule
   */
  async setEnabled(ruleId: string, enabled: boolean): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('PUT', `/api/v1/automation/rules/${ruleId}/enabled`, { enabled });
  }

  /**
   * Trigger automation rule manually
   */
  async trigger(ruleId: string): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/automation/rules/${ruleId}/trigger`);
  }
}

// ============================================================================
// Events API
// ============================================================================

class EventsAPI {
  constructor(private sdk: SmartLightingSDK) {}

  /**
   * List events
   */
  async list(params?: {
    type?: string;
    start?: string;
    end?: string;
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<LightingEvent>>> {
    return this.sdk.request('GET', '/api/v1/events', undefined, params);
  }

  /**
   * Get event by ID
   */
  async get(eventId: string): Promise<ApiResponse<LightingEvent>> {
    return this.sdk.request('GET', `/api/v1/events/${eventId}`);
  }
}

// ============================================================================
// Alerts API
// ============================================================================

class AlertsAPI {
  constructor(private sdk: SmartLightingSDK) {}

  /**
   * List alerts
   */
  async list(params?: {
    severity?: string;
    status?: string;
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<LightingAlert>>> {
    return this.sdk.request('GET', '/api/v1/alerts', undefined, params);
  }

  /**
   * Get alert by ID
   */
  async get(alertId: string): Promise<ApiResponse<LightingAlert>> {
    return this.sdk.request('GET', `/api/v1/alerts/${alertId}`);
  }

  /**
   * Acknowledge alert
   */
  async acknowledge(alertId: string, acknowledgedBy: string): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/alerts/${alertId}/acknowledge`, { acknowledgedBy });
  }

  /**
   * Resolve alert
   */
  async resolve(alertId: string): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/alerts/${alertId}/resolve`);
  }

  /**
   * Close alert
   */
  async close(alertId: string): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/alerts/${alertId}/close`);
  }
}

// ============================================================================
// WebSocket Client (Optional)
// ============================================================================

export interface WebSocketMessage {
  type: string;
  [key: string]: any;
}

export class SmartLightingWebSocket {
  private ws: WebSocket | null = null;
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 5;
  private reconnectDelay = 1000;
  private listeners: Map<string, Set<(data: any) => void>> = new Map();

  constructor(
    private endpoint: string,
    private apiKey: string
  ) {}

  /**
   * Connect to WebSocket
   */
  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      const wsUrl = this.endpoint.replace(/^http/, 'ws');
      this.ws = new WebSocket(`${wsUrl}?apiKey=${this.apiKey}`);

      this.ws.onopen = () => {
        console.log('WebSocket connected');
        this.reconnectAttempts = 0;
        resolve();
      };

      this.ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        reject(error);
      };

      this.ws.onclose = () => {
        console.log('WebSocket disconnected');
        this.handleReconnect();
      };

      this.ws.onmessage = (event) => {
        try {
          const message: WebSocketMessage = JSON.parse(event.data);
          this.handleMessage(message);
        } catch (error) {
          console.error('Failed to parse WebSocket message:', error);
        }
      };
    });
  }

  /**
   * Disconnect from WebSocket
   */
  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }

  /**
   * Subscribe to channels
   */
  subscribe(channels: string[]): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error('WebSocket not connected');
    }

    this.ws.send(JSON.stringify({
      type: 'subscribe',
      channels,
    }));
  }

  /**
   * Unsubscribe from channels
   */
  unsubscribe(channels: string[]): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error('WebSocket not connected');
    }

    this.ws.send(JSON.stringify({
      type: 'unsubscribe',
      channels,
    }));
  }

  /**
   * Listen to message type
   */
  on(type: string, callback: (data: any) => void): void {
    if (!this.listeners.has(type)) {
      this.listeners.set(type, new Set());
    }
    this.listeners.get(type)!.add(callback);
  }

  /**
   * Remove listener
   */
  off(type: string, callback: (data: any) => void): void {
    const listeners = this.listeners.get(type);
    if (listeners) {
      listeners.delete(callback);
    }
  }

  private handleMessage(message: WebSocketMessage): void {
    const listeners = this.listeners.get(message.type);
    if (listeners) {
      listeners.forEach((callback) => callback(message));
    }
  }

  private handleReconnect(): void {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++;
      const delay = this.reconnectDelay * this.reconnectAttempts;

      console.log(`Reconnecting in ${delay}ms (attempt ${this.reconnectAttempts}/${this.maxReconnectAttempts})`);

      setTimeout(() => {
        this.connect().catch((error) => {
          console.error('Reconnect failed:', error);
        });
      }, delay);
    } else {
      console.error('Max reconnect attempts reached');
    }
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate tunable white LED channels
 */
export function calculateTunableWhiteChannels(
  targetCCT: number,
  brightness: number,
  warmCCT: number = 2700,
  coolCCT: number = 6500
): { warm: number; cool: number } {
  // Calculate CCT ratio
  const ratio = (targetCCT - warmCCT) / (coolCCT - warmCCT);
  const coolRatio = Math.max(0, Math.min(1, ratio));
  const warmRatio = 1 - coolRatio;

  // Apply brightness
  const warm = warmRatio * (brightness / 100) * 255;
  const cool = coolRatio * (brightness / 100) * 255;

  return {
    warm: Math.round(warm),
    cool: Math.round(cool),
  };
}

/**
 * Calculate EML (Equivalent Melanopic Lux) from illuminance and CCT
 */
export function calculateEML(illuminance: number, cct: number): number {
  // Simplified melanopic calculation
  // Higher CCT = more blue light = higher melanopic effect
  const melanopicFactor = 0.3 + (cct - 2700) / (6500 - 2700) * 0.7;
  return illuminance * melanopicFactor;
}

/**
 * Calculate sun times for location and date
 */
export function calculateSunTimes(
  date: Date,
  latitude: number,
  longitude: number
): {
  sunrise: Date;
  sunset: Date;
  dawn: Date;
  dusk: Date;
  solarNoon: Date;
} {
  // This is a simplified implementation
  // For production, use a library like suncalc
  const dayOfYear = Math.floor((date.getTime() - new Date(date.getFullYear(), 0, 0).getTime()) / 86400000);
  const solarNoonHour = 12 - longitude / 15;

  const sunrise = new Date(date);
  sunrise.setHours(solarNoonHour - 6, 0, 0, 0);

  const sunset = new Date(date);
  sunset.setHours(solarNoonHour + 6, 0, 0, 0);

  const dawn = new Date(sunrise);
  dawn.setMinutes(dawn.getMinutes() - 30);

  const dusk = new Date(sunset);
  dusk.setMinutes(dusk.getMinutes() + 30);

  const solarNoon = new Date(date);
  solarNoon.setHours(solarNoonHour, 0, 0, 0);

  return { sunrise, sunset, dawn, dusk, solarNoon };
}

// ============================================================================
// Exports
// ============================================================================

export * from './types';

export default SmartLightingSDK;
