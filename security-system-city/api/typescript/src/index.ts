/**
 * WIA-CITY-014: Security System Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import type {
  CCTVCamera,
  SecuritySensor,
  SecurityZone,
  Guard,
  PatrolRoute,
  PatrolCheckpoint,
  EmergencyAlert,
  SecurityEvent,
  SecurityAlert,
  ControlCenter,
  VideoWall,
  ObjectDetection,
  BehaviorAnalysis,
  FaceRecognition,
  LicensePlateRecognition,
  CrowdAnalytics,
  IncidentInvestigation,
  VideoStream,
  NVR,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface SecuritySystemSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;                     // ms (default: 30000)
  retryAttempts?: number;               // default: 3
  retryDelay?: number;                  // ms (default: 1000)
}

// ============================================================================
// Main SDK Class
// ============================================================================

export class SecuritySystemSDK {
  private config: Required<SecuritySystemSDKConfig>;
  private baseUrl: string;

  // Sub-modules
  public cameras: CamerasAPI;
  public sensors: SensorsAPI;
  public zones: ZonesAPI;
  public guards: GuardsAPI;
  public patrols: PatrolsAPI;
  public emergency: EmergencyAPI;
  public alerts: AlertsAPI;
  public events: EventsAPI;
  public analytics: AnalyticsAPI;
  public control: ControlCenterAPI;
  public investigations: InvestigationsAPI;

  constructor(config: SecuritySystemSDKConfig) {
    this.config = {
      ...config,
      timeout: config.timeout || 30000,
      retryAttempts: config.retryAttempts || 3,
      retryDelay: config.retryDelay || 1000,
    };

    this.baseUrl = config.endpoint.replace(/\/$/, '');

    // Initialize sub-modules
    this.cameras = new CamerasAPI(this);
    this.sensors = new SensorsAPI(this);
    this.zones = new ZonesAPI(this);
    this.guards = new GuardsAPI(this);
    this.patrols = new PatrolsAPI(this);
    this.emergency = new EmergencyAPI(this);
    this.alerts = new AlertsAPI(this);
    this.events = new EventsAPI(this);
    this.analytics = new AnalyticsAPI(this);
    this.control = new ControlCenterAPI(this);
    this.investigations = new InvestigationsAPI(this);
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
      'User-Agent': 'WIA-CITY-014-SDK/1.0.0',
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
// Cameras API
// ============================================================================

class CamerasAPI {
  constructor(private sdk: SecuritySystemSDK) {}

  /**
   * List all cameras
   */
  async list(params?: {
    zone?: string;
    type?: string;
    online?: boolean;
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<CCTVCamera>>> {
    return this.sdk.request('GET', '/api/v1/cameras', undefined, params);
  }

  /**
   * Get camera by ID
   */
  async get(cameraId: string): Promise<ApiResponse<CCTVCamera>> {
    return this.sdk.request('GET', `/api/v1/cameras/${cameraId}`);
  }

  /**
   * Create new camera
   */
  async create(camera: Omit<CCTVCamera, 'camera_id'>): Promise<ApiResponse<{ camera_id: string }>> {
    return this.sdk.request('POST', '/api/v1/cameras', camera);
  }

  /**
   * Update camera
   */
  async update(cameraId: string, camera: Partial<CCTVCamera>): Promise<ApiResponse<CCTVCamera>> {
    return this.sdk.request('PUT', `/api/v1/cameras/${cameraId}`, camera);
  }

  /**
   * Delete camera
   */
  async delete(cameraId: string): Promise<ApiResponse<void>> {
    return this.sdk.request('DELETE', `/api/v1/cameras/${cameraId}`);
  }

  /**
   * Get camera live stream
   */
  async getStream(cameraId: string, protocol?: 'rtsp' | 'hls' | 'webrtc'): Promise<ApiResponse<VideoStream>> {
    return this.sdk.request('GET', `/api/v1/cameras/${cameraId}/stream`, undefined, { protocol });
  }

  /**
   * Control PTZ camera
   */
  async ptzControl(
    cameraId: string,
    command: {
      action: 'pan' | 'tilt' | 'zoom' | 'preset' | 'tour' | 'stop';
      value?: number;
      speed?: number;
    }
  ): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/cameras/${cameraId}/ptz`, command);
  }

  /**
   * Take snapshot
   */
  async snapshot(cameraId: string): Promise<ApiResponse<{ image_url: string }>> {
    return this.sdk.request('POST', `/api/v1/cameras/${cameraId}/snapshot`);
  }

  /**
   * Get camera recordings
   */
  async getRecordings(
    cameraId: string,
    params: {
      start: string;
      end: string;
      page?: number;
      limit?: number;
    }
  ): Promise<ApiResponse<PaginatedResponse<{ start: string; end: string; url: string }>>> {
    return this.sdk.request('GET', `/api/v1/cameras/${cameraId}/recordings`, undefined, params);
  }
}

// ============================================================================
// Sensors API
// ============================================================================

class SensorsAPI {
  constructor(private sdk: SecuritySystemSDK) {}

  /**
   * List all sensors
   */
  async list(params?: {
    zone?: string;
    type?: string;
    armed?: boolean;
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<SecuritySensor>>> {
    return this.sdk.request('GET', '/api/v1/sensors', undefined, params);
  }

  /**
   * Get sensor by ID
   */
  async get(sensorId: string): Promise<ApiResponse<SecuritySensor>> {
    return this.sdk.request('GET', `/api/v1/sensors/${sensorId}`);
  }

  /**
   * Create new sensor
   */
  async create(sensor: Omit<SecuritySensor, 'sensor_id'>): Promise<ApiResponse<{ sensor_id: string }>> {
    return this.sdk.request('POST', '/api/v1/sensors', sensor);
  }

  /**
   * Update sensor
   */
  async update(sensorId: string, sensor: Partial<SecuritySensor>): Promise<ApiResponse<SecuritySensor>> {
    return this.sdk.request('PUT', `/api/v1/sensors/${sensorId}`, sensor);
  }

  /**
   * Delete sensor
   */
  async delete(sensorId: string): Promise<ApiResponse<void>> {
    return this.sdk.request('DELETE', `/api/v1/sensors/${sensorId}`);
  }

  /**
   * Arm sensor
   */
  async arm(sensorId: string): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/sensors/${sensorId}/arm`);
  }

  /**
   * Disarm sensor
   */
  async disarm(sensorId: string): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/sensors/${sensorId}/disarm`);
  }

  /**
   * Test sensor
   */
  async test(sensorId: string): Promise<ApiResponse<{ success: boolean; result: string }>> {
    return this.sdk.request('POST', `/api/v1/sensors/${sensorId}/test`);
  }
}

// ============================================================================
// Zones API
// ============================================================================

class ZonesAPI {
  constructor(private sdk: SecuritySystemSDK) {}

  /**
   * List all zones
   */
  async list(params?: {
    type?: string;
    armed?: boolean;
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<SecurityZone>>> {
    return this.sdk.request('GET', '/api/v1/zones', undefined, params);
  }

  /**
   * Get zone by ID
   */
  async get(zoneId: string): Promise<ApiResponse<SecurityZone>> {
    return this.sdk.request('GET', `/api/v1/zones/${zoneId}`);
  }

  /**
   * Create new zone
   */
  async create(zone: Omit<SecurityZone, 'zone_id'>): Promise<ApiResponse<{ zone_id: string }>> {
    return this.sdk.request('POST', '/api/v1/zones', zone);
  }

  /**
   * Update zone
   */
  async update(zoneId: string, zone: Partial<SecurityZone>): Promise<ApiResponse<SecurityZone>> {
    return this.sdk.request('PUT', `/api/v1/zones/${zoneId}`, zone);
  }

  /**
   * Delete zone
   */
  async delete(zoneId: string): Promise<ApiResponse<void>> {
    return this.sdk.request('DELETE', `/api/v1/zones/${zoneId}`);
  }

  /**
   * Arm zone
   */
  async arm(zoneId: string, mode?: 'away' | 'stay' | 'night'): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/zones/${zoneId}/arm`, { mode });
  }

  /**
   * Disarm zone
   */
  async disarm(zoneId: string, code?: string): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/zones/${zoneId}/disarm`, { code });
  }
}

// ============================================================================
// Guards API
// ============================================================================

class GuardsAPI {
  constructor(private sdk: SecuritySystemSDK) {}

  /**
   * List all guards
   */
  async list(params?: {
    shift?: string;
    status?: string;
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<Guard>>> {
    return this.sdk.request('GET', '/api/v1/guards', undefined, params);
  }

  /**
   * Get guard by ID
   */
  async get(guardId: string): Promise<ApiResponse<Guard>> {
    return this.sdk.request('GET', `/api/v1/guards/${guardId}`);
  }

  /**
   * Update guard location
   */
  async updateLocation(
    guardId: string,
    location: {
      latitude: number;
      longitude: number;
      building?: string;
      floor?: string;
    }
  ): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/guards/${guardId}/location`, location);
  }

  /**
   * Update guard status
   */
  async updateStatus(
    guardId: string,
    status: 'on_patrol' | 'at_post' | 'break' | 'responding' | 'off_duty'
  ): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/guards/${guardId}/status`, { status });
  }

  /**
   * Trigger panic button
   */
  async panic(guardId: string, location?: { latitude: number; longitude: number }): Promise<ApiResponse<{ alert_id: string }>> {
    return this.sdk.request('POST', `/api/v1/guards/${guardId}/panic`, { location });
  }
}

// ============================================================================
// Patrols API
// ============================================================================

class PatrolsAPI {
  constructor(private sdk: SecuritySystemSDK) {}

  /**
   * List patrol routes
   */
  async listRoutes(): Promise<ApiResponse<{ routes: PatrolRoute[] }>> {
    return this.sdk.request('GET', '/api/v1/patrols/routes');
  }

  /**
   * Get patrol route
   */
  async getRoute(routeId: string): Promise<ApiResponse<PatrolRoute>> {
    return this.sdk.request('GET', `/api/v1/patrols/routes/${routeId}`);
  }

  /**
   * Start patrol
   */
  async start(routeId: string, guardId: string): Promise<ApiResponse<{ patrol_id: string }>> {
    return this.sdk.request('POST', `/api/v1/patrols/start`, { route_id: routeId, guard_id: guardId });
  }

  /**
   * Scan checkpoint
   */
  async scanCheckpoint(
    checkpointId: string,
    guardId: string,
    data: {
      scan_code: string;
      tasks_completed: string[];
      notes?: string;
    }
  ): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/patrols/checkpoints/${checkpointId}/scan`, { ...data, guard_id: guardId });
  }

  /**
   * Complete patrol
   */
  async complete(patrolId: string): Promise<ApiResponse<{ success: boolean; report: any }>> {
    return this.sdk.request('POST', `/api/v1/patrols/${patrolId}/complete`);
  }

  /**
   * Get patrol history
   */
  async getHistory(params: {
    guard_id?: string;
    route_id?: string;
    start?: string;
    end?: string;
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<any>>> {
    return this.sdk.request('GET', '/api/v1/patrols/history', undefined, params);
  }
}

// ============================================================================
// Emergency API
// ============================================================================

class EmergencyAPI {
  constructor(private sdk: SecuritySystemSDK) {}

  /**
   * Trigger emergency alert
   */
  async trigger(alert: Omit<EmergencyAlert, 'alert_id'>): Promise<ApiResponse<{ alert_id: string }>> {
    return this.sdk.request('POST', '/api/v1/emergency/alert', alert);
  }

  /**
   * Get active emergencies
   */
  async getActive(): Promise<ApiResponse<{ emergencies: EmergencyAlert[] }>> {
    return this.sdk.request('GET', '/api/v1/emergency/active');
  }

  /**
   * Get emergency by ID
   */
  async get(alertId: string): Promise<ApiResponse<EmergencyAlert>> {
    return this.sdk.request('GET', `/api/v1/emergency/${alertId}`);
  }

  /**
   * Update emergency status
   */
  async updateStatus(
    alertId: string,
    status: 'dispatched' | 'en_route' | 'on_scene' | 'resolved'
  ): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/emergency/${alertId}/status`, { status });
  }

  /**
   * Resolve emergency
   */
  async resolve(
    alertId: string,
    resolution: {
      resolved_by: string;
      outcome: string;
      report?: string;
    }
  ): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/emergency/${alertId}/resolve`, resolution);
  }

  /**
   * Broadcast PA announcement
   */
  async broadcast(message: string, zones?: string[]): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', '/api/v1/emergency/broadcast', { message, zones });
  }
}

// ============================================================================
// Alerts API
// ============================================================================

class AlertsAPI {
  constructor(private sdk: SecuritySystemSDK) {}

  /**
   * List alerts
   */
  async list(params?: {
    severity?: string;
    status?: string;
    type?: string;
    start?: string;
    end?: string;
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<SecurityAlert>>> {
    return this.sdk.request('GET', '/api/v1/alerts', undefined, params);
  }

  /**
   * Get alert by ID
   */
  async get(alertId: string): Promise<ApiResponse<SecurityAlert>> {
    return this.sdk.request('GET', `/api/v1/alerts/${alertId}`);
  }

  /**
   * Acknowledge alert
   */
  async acknowledge(alertId: string, acknowledgedBy: string): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/alerts/${alertId}/acknowledge`, { acknowledged_by: acknowledgedBy });
  }

  /**
   * Resolve alert
   */
  async resolve(alertId: string, resolvedBy: string, notes?: string): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/alerts/${alertId}/resolve`, { resolved_by: resolvedBy, notes });
  }

  /**
   * Close alert
   */
  async close(alertId: string): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/alerts/${alertId}/close`);
  }
}

// ============================================================================
// Events API
// ============================================================================

class EventsAPI {
  constructor(private sdk: SecuritySystemSDK) {}

  /**
   * List events
   */
  async list(params?: {
    type?: string;
    severity?: string;
    source_type?: string;
    start?: string;
    end?: string;
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<SecurityEvent>>> {
    return this.sdk.request('GET', '/api/v1/events', undefined, params);
  }

  /**
   * Get event by ID
   */
  async get(eventId: string): Promise<ApiResponse<SecurityEvent>> {
    return this.sdk.request('GET', `/api/v1/events/${eventId}`);
  }

  /**
   * Search events
   */
  async search(query: {
    keyword?: string;
    filters?: Record<string, any>;
    date_range?: DateRangeFilter;
  }): Promise<ApiResponse<{ events: SecurityEvent[] }>> {
    return this.sdk.request('POST', '/api/v1/events/search', query);
  }

  /**
   * Export events
   */
  async export(params: {
    format: 'csv' | 'json' | 'pdf';
    start: string;
    end: string;
    filters?: Record<string, any>;
  }): Promise<ApiResponse<{ download_url: string }>> {
    return this.sdk.request('POST', '/api/v1/events/export', params);
  }
}

// ============================================================================
// Analytics API
// ============================================================================

class AnalyticsAPI {
  constructor(private sdk: SecuritySystemSDK) {}

  /**
   * Detect objects in camera feed
   */
  async detectObjects(cameraId: string): Promise<ApiResponse<{ detections: ObjectDetection[] }>> {
    return this.sdk.request('GET', `/api/v1/analytics/cameras/${cameraId}/objects`);
  }

  /**
   * Analyze behavior
   */
  async analyzeBehavior(cameraId: string): Promise<ApiResponse<BehaviorAnalysis>> {
    return this.sdk.request('GET', `/api/v1/analytics/cameras/${cameraId}/behavior`);
  }

  /**
   * Recognize faces
   */
  async recognizeFaces(cameraId: string): Promise<ApiResponse<{ faces: FaceRecognition[] }>> {
    return this.sdk.request('GET', `/api/v1/analytics/cameras/${cameraId}/faces`);
  }

  /**
   * Recognize license plates
   */
  async recognizePlates(cameraId: string): Promise<ApiResponse<{ plates: LicensePlateRecognition[] }>> {
    return this.sdk.request('GET', `/api/v1/analytics/cameras/${cameraId}/plates`);
  }

  /**
   * Analyze crowd
   */
  async analyzeCrowd(cameraId: string, zoneId?: string): Promise<ApiResponse<CrowdAnalytics>> {
    return this.sdk.request('GET', `/api/v1/analytics/cameras/${cameraId}/crowd`, undefined, { zone_id: zoneId });
  }

  /**
   * Get analytics report
   */
  async getReport(params: {
    type: 'daily' | 'weekly' | 'monthly';
    date: string;
    zone?: string;
  }): Promise<ApiResponse<{ report: any }>> {
    return this.sdk.request('GET', '/api/v1/analytics/report', undefined, params);
  }
}

// ============================================================================
// Control Center API
// ============================================================================

class ControlCenterAPI {
  constructor(private sdk: SecuritySystemSDK) {}

  /**
   * Get control center status
   */
  async getStatus(): Promise<ApiResponse<ControlCenter>> {
    return this.sdk.request('GET', '/api/v1/control/status');
  }

  /**
   * Get dashboard data
   */
  async getDashboard(): Promise<ApiResponse<{
    active_alerts: number;
    cameras_online: number;
    sensors_online: number;
    guards_on_duty: number;
    recent_events: SecurityEvent[];
  }>> {
    return this.sdk.request('GET', '/api/v1/control/dashboard');
  }

  /**
   * Get video wall configuration
   */
  async getVideoWall(wallId: string): Promise<ApiResponse<VideoWall>> {
    return this.sdk.request('GET', `/api/v1/control/video-wall/${wallId}`);
  }

  /**
   * Set video wall preset
   */
  async setVideoWallPreset(wallId: string, presetId: string): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/control/video-wall/${wallId}/preset`, { preset_id: presetId });
  }
}

// ============================================================================
// Investigations API
// ============================================================================

class InvestigationsAPI {
  constructor(private sdk: SecuritySystemSDK) {}

  /**
   * List investigations
   */
  async list(params?: {
    status?: string;
    severity?: string;
    page?: number;
    limit?: number;
  }): Promise<ApiResponse<PaginatedResponse<IncidentInvestigation>>> {
    return this.sdk.request('GET', '/api/v1/investigations', undefined, params);
  }

  /**
   * Get investigation by ID
   */
  async get(incidentId: string): Promise<ApiResponse<IncidentInvestigation>> {
    return this.sdk.request('GET', `/api/v1/investigations/${incidentId}`);
  }

  /**
   * Create investigation
   */
  async create(
    investigation: Omit<IncidentInvestigation, 'incident_id' | 'created_at'>
  ): Promise<ApiResponse<{ incident_id: string }>> {
    return this.sdk.request('POST', '/api/v1/investigations', investigation);
  }

  /**
   * Update investigation
   */
  async update(
    incidentId: string,
    investigation: Partial<IncidentInvestigation>
  ): Promise<ApiResponse<IncidentInvestigation>> {
    return this.sdk.request('PUT', `/api/v1/investigations/${incidentId}`, investigation);
  }

  /**
   * Add evidence
   */
  async addEvidence(
    incidentId: string,
    evidence: {
      type: 'video' | 'photo' | 'sensor_log' | 'other';
      data: any;
    }
  ): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/investigations/${incidentId}/evidence`, evidence);
  }

  /**
   * Close investigation
   */
  async close(incidentId: string): Promise<ApiResponse<{ success: boolean }>> {
    return this.sdk.request('POST', `/api/v1/investigations/${incidentId}/close`);
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate detection confidence threshold
 */
export function isHighConfidence(confidence: number, threshold: number = 0.8): boolean {
  return confidence >= threshold;
}

/**
 * Format alert severity for display
 */
export function formatSeverity(severity: 'critical' | 'high' | 'medium' | 'low'): string {
  const severityMap: Record<string, string> = {
    critical: '🔴 Critical',
    high: '🟠 High',
    medium: '🟡 Medium',
    low: '🟢 Low',
  };
  return severityMap[severity] || severity;
}

/**
 * Calculate estimated response time based on priority
 */
export function getEstimatedResponseTime(priority: 'critical' | 'high' | 'medium' | 'low'): number {
  const responseTimeMap: Record<string, number> = {
    critical: 0,    // Immediate
    high: 1,        // 1 minute
    medium: 5,      // 5 minutes
    low: 30,        // 30 minutes
  };
  return responseTimeMap[priority] || 60;
}

// ============================================================================
// Exports
// ============================================================================

export * from './types';

export default SecuritySystemSDK;
