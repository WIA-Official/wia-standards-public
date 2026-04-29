/**
 * WIA-MARITIME_SAFETY TypeScript SDK
 * Version: 1.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 *
 * Comprehensive SDK for maritime safety operations including:
 * - Vessel tracking and AIS integration
 * - Safety alerts and emergency response
 * - Weather monitoring and route planning
 * - Port operations and berth management
 * - SOLAS/IMO compliance tracking
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import WebSocket from 'ws';
import {
  MaritimeSafetyConfig,
  APIResponse,
  PaginatedResponse,
  VesselIdentity,
  AISPositionReport,
  Position,
  WeatherData,
  SafetyAlert,
  AlertSeverity,
  AlertCategory,
  Route,
  RouteCalculationRequest,
  RouteCalculationResponse,
  PortInfo,
  BerthRequest,
  BerthAssignment,
  VesselTrackRequest,
  VesselTrackResponse,
  VesselSearchRequest,
  CollisionRisk,
  SOLASCompliance,
  MARPOLCompliance,
  SARCoordination,
  StreamSubscription,
  StreamUpdate,
  StreamChannel,
  DistressCall,
  GMDSSMessage,
  VHFMessage,
} from './types';

/**
 * Main Maritime Safety SDK Client
 */
export class MaritimeSafetyClient {
  private client: AxiosInstance;
  private config: MaritimeSafetyConfig;
  private ws?: WebSocket;
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 5;

  constructor(config: MaritimeSafetyConfig) {
    this.config = {
      timeout: 30000,
      retryAttempts: 3,
      websocket: {
        enabled: false,
        reconnect: true,
      },
      ...config,
    };

    this.client = axios.create({
      baseURL: this.config.apiEndpoint,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        ...(this.config.apiKey && { Authorization: `Bearer ${this.config.apiKey}` }),
      },
    });

    // Add response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      (error: AxiosError) => this.handleError(error)
    );
  }

  /**
   * Error handler for API requests
   */
  private handleError(error: AxiosError): never {
    if (error.response) {
      const apiError = error.response.data as APIResponse<never>;
      throw new Error(
        apiError.error?.message || `API Error: ${error.response.status}`
      );
    } else if (error.request) {
      throw new Error('No response received from API');
    } else {
      throw new Error(`Request error: ${error.message}`);
    }
  }

  // ============================================================================
  // Vessel Tracking & AIS Methods
  // ============================================================================

  /**
   * Get current position of a vessel by MMSI
   */
  async getVesselPosition(mmsi: string): Promise<AISPositionReport> {
    const response = await this.client.get<APIResponse<AISPositionReport>>(
      `/vessels/${mmsi}/position`
    );
    return response.data.data!;
  }

  /**
   * Get vessel identity information
   */
  async getVesselIdentity(mmsi: string): Promise<VesselIdentity> {
    const response = await this.client.get<APIResponse<VesselIdentity>>(
      `/vessels/${mmsi}/identity`
    );
    return response.data.data!;
  }

  /**
   * Track multiple vessels simultaneously
   */
  async trackVessels(request: VesselTrackRequest): Promise<VesselTrackResponse[]> {
    const response = await this.client.post<APIResponse<{ vessels: VesselTrackResponse[] }>>(
      '/vessels/track',
      request
    );
    return response.data.data!.vessels;
  }

  /**
   * Search for vessels in a geographic area
   */
  async searchVessels(
    request: VesselSearchRequest
  ): Promise<PaginatedResponse<VesselTrackResponse>> {
    const response = await this.client.post<APIResponse<PaginatedResponse<VesselTrackResponse>>>(
      '/vessels/search',
      request
    );
    return response.data.data!;
  }

  /**
   * Get vessel historical track (voyage history)
   */
  async getVesselHistory(
    mmsi: string,
    startTime: string,
    endTime: string
  ): Promise<AISPositionReport[]> {
    const response = await this.client.get<APIResponse<{ positions: AISPositionReport[] }>>(
      `/vessels/${mmsi}/history`,
      {
        params: { startTime, endTime },
      }
    );
    return response.data.data!.positions;
  }

  // ============================================================================
  // Weather & Environmental Methods
  // ============================================================================

  /**
   * Get current marine weather for a location
   */
  async getWeather(
    latitude: number,
    longitude: number,
    includeForecast = false
  ): Promise<WeatherData> {
    const response = await this.client.get<APIResponse<WeatherData>>('/weather', {
      params: { latitude, longitude, forecast: includeForecast },
    });
    return response.data.data!;
  }

  /**
   * Get weather along a planned route
   */
  async getRouteWeather(waypoints: Position[], departureTime?: string): Promise<WeatherData[]> {
    const response = await this.client.post<APIResponse<{ weather: WeatherData[] }>>(
      '/weather/route',
      {
        waypoints,
        departureTime,
      }
    );
    return response.data.data!.weather;
  }

  /**
   * Get weather warnings for an area
   */
  async getWeatherWarnings(area: {
    center: Position;
    radius: number;
  }): Promise<string[]> {
    const response = await this.client.post<APIResponse<{ warnings: string[] }>>(
      '/weather/warnings',
      area
    );
    return response.data.data!.warnings;
  }

  // ============================================================================
  // Safety Alert Methods
  // ============================================================================

  /**
   * Create a safety alert
   */
  async createAlert(alert: {
    severity: AlertSeverity;
    category: AlertCategory;
    position: Position;
    vessel: { mmsi: string; name: string };
    description: string;
    autoNotify?: boolean;
  }): Promise<SafetyAlert> {
    const response = await this.client.post<APIResponse<SafetyAlert>>('/alerts', alert);
    return response.data.data!;
  }

  /**
   * Get active safety alerts
   */
  async getActiveAlerts(filters?: {
    severity?: AlertSeverity;
    category?: AlertCategory;
    area?: { center: Position; radius: number };
  }): Promise<PaginatedResponse<SafetyAlert>> {
    const response = await this.client.get<APIResponse<PaginatedResponse<SafetyAlert>>>(
      '/alerts',
      {
        params: filters,
      }
    );
    return response.data.data!;
  }

  /**
   * Update alert status
   */
  async updateAlertStatus(
    alertId: string,
    status: 'acknowledged' | 'resolved',
    notes?: string
  ): Promise<SafetyAlert> {
    const response = await this.client.patch<APIResponse<SafetyAlert>>(
      `/alerts/${alertId}`,
      {
        status,
        notes,
      }
    );
    return response.data.data!;
  }

  /**
   * Get alert details
   */
  async getAlert(alertId: string): Promise<SafetyAlert> {
    const response = await this.client.get<APIResponse<SafetyAlert>>(`/alerts/${alertId}`);
    return response.data.data!;
  }

  // ============================================================================
  // Emergency & Distress Methods
  // ============================================================================

  /**
   * Send a distress call
   */
  async sendDistressCall(distress: DistressCall): Promise<{ alertId: string }> {
    const response = await this.client.post<APIResponse<{ alertId: string }>>(
      '/emergency/distress',
      distress
    );
    return response.data.data!;
  }

  /**
   * Send GMDSS message
   */
  async sendGMDSSMessage(message: GMDSSMessage): Promise<{ messageId: string }> {
    const response = await this.client.post<APIResponse<{ messageId: string }>>(
      '/communication/gmdss',
      message
    );
    return response.data.data!;
  }

  /**
   * Send VHF radio message
   */
  async sendVHFMessage(message: VHFMessage): Promise<{ messageId: string }> {
    const response = await this.client.post<APIResponse<{ messageId: string }>>(
      '/communication/vhf',
      message
    );
    return response.data.data!;
  }

  /**
   * Initiate SAR operation
   */
  async initiateSAR(incident: {
    distressType: AlertCategory;
    position: Position;
    pob: number;
    vessel: VesselIdentity;
  }): Promise<SARCoordination> {
    const response = await this.client.post<APIResponse<SARCoordination>>(
      '/emergency/sar/initiate',
      incident
    );
    return response.data.data!;
  }

  /**
   * Get SAR mission details
   */
  async getSARMission(missionId: string): Promise<SARCoordination> {
    const response = await this.client.get<APIResponse<SARCoordination>>(
      `/emergency/sar/${missionId}`
    );
    return response.data.data!;
  }

  // ============================================================================
  // Route Planning Methods
  // ============================================================================

  /**
   * Calculate optimal route
   */
  async calculateRoute(request: RouteCalculationRequest): Promise<RouteCalculationResponse> {
    const response = await this.client.post<APIResponse<RouteCalculationResponse>>(
      '/routes/calculate',
      request
    );
    return response.data.data!;
  }

  /**
   * Get route details
   */
  async getRoute(routeId: string): Promise<Route> {
    const response = await this.client.get<APIResponse<Route>>(`/routes/${routeId}`);
    return response.data.data!;
  }

  /**
   * Validate route safety
   */
  async validateRoute(waypoints: Position[]): Promise<{
    safe: boolean;
    warnings: string[];
    hazards: Array<{
      position: Position;
      type: string;
      description: string;
    }>;
  }> {
    const response = await this.client.post<
      APIResponse<{
        safe: boolean;
        warnings: string[];
        hazards: Array<{ position: Position; type: string; description: string }>;
      }>
    >('/routes/validate', { waypoints });
    return response.data.data!;
  }

  /**
   * Optimize route for fuel efficiency
   */
  async optimizeRouteForFuel(routeId: string): Promise<Route> {
    const response = await this.client.post<APIResponse<Route>>(
      `/routes/${routeId}/optimize/fuel`
    );
    return response.data.data!;
  }

  // ============================================================================
  // Port & Berth Methods
  // ============================================================================

  /**
   * Get port information
   */
  async getPortInfo(portId: string): Promise<PortInfo> {
    const response = await this.client.get<APIResponse<PortInfo>>(`/ports/${portId}`);
    return response.data.data!;
  }

  /**
   * Request berth assignment
   */
  async requestBerth(portId: string, request: BerthRequest): Promise<BerthAssignment> {
    const response = await this.client.post<APIResponse<BerthAssignment>>(
      `/ports/${portId}/berth-request`,
      request
    );
    return response.data.data!;
  }

  /**
   * Get berth availability
   */
  async getBerthAvailability(
    portId: string,
    startTime: string,
    endTime: string
  ): Promise<{ available: number; total: number; berths: Array<{ number: string; available: boolean }> }> {
    const response = await this.client.get<
      APIResponse<{ available: number; total: number; berths: Array<{ number: string; available: boolean }> }>
    >(`/ports/${portId}/berth-availability`, {
      params: { startTime, endTime },
    });
    return response.data.data!;
  }

  /**
   * Search for nearby ports
   */
  async findNearbyPorts(position: Position, radius: number): Promise<PortInfo[]> {
    const response = await this.client.post<APIResponse<{ ports: PortInfo[] }>>(
      '/ports/search',
      {
        position,
        radius,
      }
    );
    return response.data.data!.ports;
  }

  // ============================================================================
  // Collision Avoidance Methods
  // ============================================================================

  /**
   * Assess collision risk with nearby vessels
   */
  async assessCollisionRisk(
    ownPosition: Position,
    ownCourse: number,
    ownSpeed: number,
    searchRadius: number
  ): Promise<CollisionRisk[]> {
    const response = await this.client.post<APIResponse<{ risks: CollisionRisk[] }>>(
      '/collision/assess',
      {
        ownPosition,
        ownCourse,
        ownSpeed,
        searchRadius,
      }
    );
    return response.data.data!.risks;
  }

  /**
   * Get COLREGS recommendation for a situation
   */
  async getCOLREGSAdvice(situation: {
    ownVessel: { position: Position; course: number; speed: number };
    targetVessel: { position: Position; course: number; speed: number };
  }): Promise<{
    rule: number;
    action: string;
    soundSignal?: string;
  }> {
    const response = await this.client.post<
      APIResponse<{ rule: number; action: string; soundSignal?: string }>
    >('/collision/colregs', situation);
    return response.data.data!;
  }

  // ============================================================================
  // Compliance & Certification Methods
  // ============================================================================

  /**
   * Get SOLAS compliance status
   */
  async getSOLASCompliance(mmsi: string): Promise<SOLASCompliance> {
    const response = await this.client.get<APIResponse<SOLASCompliance>>(
      `/compliance/solas/${mmsi}`
    );
    return response.data.data!;
  }

  /**
   * Get MARPOL compliance status
   */
  async getMARPOLCompliance(mmsi: string): Promise<MARPOLCompliance> {
    const response = await this.client.get<APIResponse<MARPOLCompliance>>(
      `/compliance/marpol/${mmsi}`
    );
    return response.data.data!;
  }

  /**
   * Update certificate information
   */
  async updateCertificate(
    mmsi: string,
    certificateType: string,
    certificateData: {
      number: string;
      issued: string;
      expires: string;
      issuer: string;
    }
  ): Promise<{ success: boolean }> {
    const response = await this.client.post<APIResponse<{ success: boolean }>>(
      `/compliance/certificates/${mmsi}`,
      {
        type: certificateType,
        ...certificateData,
      }
    );
    return response.data.data!;
  }

  // ============================================================================
  // Real-time Streaming Methods (WebSocket)
  // ============================================================================

  /**
   * Connect to real-time data stream
   */
  connectStream(
    onMessage: (update: StreamUpdate<unknown>) => void,
    onError?: (error: Error) => void
  ): void {
    if (!this.config.websocket?.enabled) {
      throw new Error('WebSocket is not enabled in configuration');
    }

    const wsEndpoint = this.config.websocket.endpoint || this.config.apiEndpoint.replace('http', 'ws') + '/stream';

    this.ws = new WebSocket(wsEndpoint, {
      headers: this.config.apiKey ? { Authorization: `Bearer ${this.config.apiKey}` } : {},
    });

    this.ws.on('open', () => {
      console.log('WebSocket connected');
      this.reconnectAttempts = 0;
    });

    this.ws.on('message', (data: WebSocket.Data) => {
      try {
        const update = JSON.parse(data.toString()) as StreamUpdate<unknown>;
        onMessage(update);
      } catch (error) {
        console.error('Failed to parse WebSocket message:', error);
      }
    });

    this.ws.on('error', (error: Error) => {
      console.error('WebSocket error:', error);
      if (onError) onError(error);
    });

    this.ws.on('close', () => {
      console.log('WebSocket disconnected');
      if (this.config.websocket?.reconnect && this.reconnectAttempts < this.maxReconnectAttempts) {
        this.reconnectAttempts++;
        setTimeout(() => {
          console.log(`Reconnecting... Attempt ${this.reconnectAttempts}`);
          this.connectStream(onMessage, onError);
        }, 5000);
      }
    });
  }

  /**
   * Subscribe to specific channels
   */
  subscribe(subscription: StreamSubscription): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error('WebSocket is not connected');
    }
    this.ws.send(JSON.stringify(subscription));
  }

  /**
   * Unsubscribe from channels
   */
  unsubscribe(channels: StreamChannel[]): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error('WebSocket is not connected');
    }
    this.ws.send(
      JSON.stringify({
        action: 'unsubscribe',
        channels,
      })
    );
  }

  /**
   * Close WebSocket connection
   */
  disconnectStream(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = undefined;
    }
  }

  // ============================================================================
  // Utility Methods
  // ============================================================================

  /**
   * Calculate distance between two positions (Great Circle)
   */
  static calculateDistance(pos1: Position, pos2: Position): number {
    const R = 3440.065; // Earth radius in nautical miles
    const lat1 = (pos1.latitude * Math.PI) / 180;
    const lat2 = (pos2.latitude * Math.PI) / 180;
    const deltaLat = ((pos2.latitude - pos1.latitude) * Math.PI) / 180;
    const deltaLon = ((pos2.longitude - pos1.longitude) * Math.PI) / 180;

    const a =
      Math.sin(deltaLat / 2) * Math.sin(deltaLat / 2) +
      Math.cos(lat1) * Math.cos(lat2) * Math.sin(deltaLon / 2) * Math.sin(deltaLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return R * c;
  }

  /**
   * Calculate bearing between two positions
   */
  static calculateBearing(pos1: Position, pos2: Position): number {
    const lat1 = (pos1.latitude * Math.PI) / 180;
    const lat2 = (pos2.latitude * Math.PI) / 180;
    const deltaLon = ((pos2.longitude - pos1.longitude) * Math.PI) / 180;

    const y = Math.sin(deltaLon) * Math.cos(lat2);
    const x =
      Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(deltaLon);
    const bearing = (Math.atan2(y, x) * 180) / Math.PI;

    return (bearing + 360) % 360;
  }

  /**
   * Validate MMSI format
   */
  static validateMMSI(mmsi: string): boolean {
    return /^[0-9]{9}$/.test(mmsi);
  }

  /**
   * Validate IMO number (including checksum)
   */
  static validateIMO(imo: string): boolean {
    if (!/^[0-9]{7}$/.test(imo)) return false;

    const digits = imo.split('').map(Number);
    let sum = 0;
    for (let i = 0; i < 6; i++) {
      sum += digits[i] * (7 - i);
    }
    const checkDigit = sum % 10;
    return checkDigit === digits[6];
  }
}

// ============================================================================
// Export everything
// ============================================================================

export * from './types';
export default MaritimeSafetyClient;
