/**
 * WIA-UNI-008 Transportation Network Standard
 * TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 */

import axios, { AxiosInstance, AxiosRequestConfig } from 'axios';
import WebSocket from 'ws';
import {
  TransportationRoute,
  TransportationBooking,
  CargoShipment,
  TransportVehicle,
  VehicleTracking,
  APIResponse,
  PaginatedResponse,
  QueryParams,
  APIError,
  WebhookConfig,
  Webhook,
  AvailabilityParams,
  AvailabilityResult,
} from './types';

/**
 * SDK Configuration options
 */
export interface WIATransportationConfig {
  /**
   * API base URL
   */
  baseURL: string;

  /**
   * OAuth 2.0 access token
   */
  accessToken: string;

  /**
   * Region identifier
   */
  region?: string;

  /**
   * API version
   */
  apiVersion?: string;

  /**
   * Request timeout in milliseconds
   */
  timeout?: number;
}

/**
 * Main SDK class for WIA-UNI-008 Transportation Network
 */
export class WIATransportationSDK {
  private client: AxiosInstance;
  private config: WIATransportationConfig;
  private wsConnections: Map<string, WebSocket>;

  /**
   * Create a new SDK instance
   * @param config - SDK configuration
   */
  constructor(config: WIATransportationConfig) {
    this.config = {
      apiVersion: 'v1',
      region: 'seoul',
      timeout: 30000,
      ...config,
    };

    this.client = axios.create({
      baseURL: `${this.config.baseURL}/${this.config.region}/${this.config.apiVersion}`,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.accessToken}`,
        'Content-Type': 'application/json',
        'Accept': 'application/json',
      },
    });

    this.wsConnections = new Map();

    // Add response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      (error) => {
        if (error.response?.data) {
          throw error.response.data as APIError;
        }
        throw error;
      }
    );
  }

  // ==================== Routes API ====================

  /**
   * List transportation routes
   * @param params - Query parameters
   * @returns Paginated list of routes
   */
  async listRoutes(params?: QueryParams): Promise<PaginatedResponse<TransportationRoute>> {
    const response = await this.client.get<PaginatedResponse<TransportationRoute>>('/routes', {
      params,
    });
    return response.data;
  }

  /**
   * Get route details by ID
   * @param routeId - Route identifier
   * @returns Route details
   */
  async getRoute(routeId: string): Promise<TransportationRoute> {
    const response = await this.client.get<TransportationRoute>(`/routes/${routeId}`);
    return response.data;
  }

  /**
   * Create a new transportation route
   * @param route - Route data
   * @returns Created route with ID
   */
  async createRoute(route: Omit<TransportationRoute, 'id' | '@context'>): Promise<APIResponse<TransportationRoute>> {
    const payload = {
      '@context': 'https://wiastandards.com/contexts/uni-008/v1',
      ...route,
    };
    const response = await this.client.post<APIResponse<TransportationRoute>>('/routes', payload);
    return response.data;
  }

  /**
   * Update an existing route
   * @param routeId - Route identifier
   * @param updates - Partial route updates
   * @returns Updated route
   */
  async updateRoute(
    routeId: string,
    updates: Partial<TransportationRoute>
  ): Promise<TransportationRoute> {
    const response = await this.client.put<TransportationRoute>(`/routes/${routeId}`, updates);
    return response.data;
  }

  /**
   * Delete a route
   * @param routeId - Route identifier
   */
  async deleteRoute(routeId: string): Promise<void> {
    await this.client.delete(`/routes/${routeId}`);
  }

  // ==================== Bookings API ====================

  /**
   * Search for available routes and seats
   * @param params - Availability search parameters
   * @returns Available routes and pricing
   */
  async searchAvailability(params: AvailabilityParams): Promise<AvailabilityResult> {
    const response = await this.client.get<AvailabilityResult>('/bookings/availability', {
      params,
    });
    return response.data;
  }

  /**
   * Create a new booking
   * @param booking - Booking data
   * @returns Created booking with confirmation
   */
  async createBooking(
    booking: Omit<TransportationBooking, 'id' | '@context' | 'bookingReference' | 'status'>
  ): Promise<APIResponse<TransportationBooking>> {
    const payload = {
      '@context': 'https://wiastandards.com/contexts/uni-008/v1',
      ...booking,
    };
    const response = await this.client.post<APIResponse<TransportationBooking>>('/bookings', payload);
    return response.data;
  }

  /**
   * Get booking details
   * @param bookingId - Booking identifier
   * @returns Booking details
   */
  async getBooking(bookingId: string): Promise<TransportationBooking> {
    const response = await this.client.get<TransportationBooking>(`/bookings/${bookingId}`);
    return response.data;
  }

  /**
   * Cancel a booking
   * @param bookingId - Booking identifier
   * @returns Cancellation confirmation with refund details
   */
  async cancelBooking(bookingId: string): Promise<{
    status: string;
    refund?: {
      amount: number;
      currency: string;
      method: string;
      estimatedDays: number;
    };
  }> {
    const response = await this.client.delete(`/bookings/${bookingId}`);
    return response.data;
  }

  /**
   * Track booking in real-time
   * @param bookingId - Booking identifier
   * @returns Current booking status and vehicle location
   */
  async trackBooking(bookingId: string): Promise<{
    bookingReference: string;
    status: string;
    vehicle?: {
      vehicleId: string;
      currentLocation: {
        latitude: number;
        longitude: number;
        timestamp: string;
      };
    };
    journey?: {
      origin: string;
      destination: string;
      progress: string;
      estimatedArrival: string;
    };
  }> {
    const response = await this.client.get(`/tracking/bookings/${bookingId}`);
    return response.data;
  }

  // ==================== Tracking API ====================

  /**
   * Track vehicle location and status
   * @param vehicleId - Vehicle identifier
   * @returns Real-time tracking data
   */
  async trackVehicle(vehicleId: string): Promise<VehicleTracking> {
    const response = await this.client.get<VehicleTracking>(`/tracking/vehicles/${vehicleId}`);
    return response.data;
  }

  /**
   * Subscribe to real-time vehicle tracking updates via WebSocket
   * @param vehicleId - Vehicle identifier
   * @param onUpdate - Callback function for tracking updates
   * @param onError - Callback function for errors
   * @returns WebSocket connection ID for later unsubscribe
   */
  subscribeToTracking(
    vehicleId: string,
    onUpdate: (data: VehicleTracking) => void,
    onError?: (error: Error) => void
  ): string {
    const wsUrl = this.config.baseURL.replace('http', 'ws') + `/${this.config.apiVersion}/tracking/stream`;
    const ws = new WebSocket(wsUrl);
    const connectionId = `tracking-${vehicleId}-${Date.now()}`;

    ws.on('open', () => {
      ws.send(JSON.stringify({
        action: 'subscribe',
        vehicleId,
        token: this.config.accessToken,
      }));
    });

    ws.on('message', (data: WebSocket.Data) => {
      try {
        const update = JSON.parse(data.toString()) as VehicleTracking;
        onUpdate(update);
      } catch (error) {
        if (onError) {
          onError(error as Error);
        }
      }
    });

    ws.on('error', (error) => {
      if (onError) {
        onError(error);
      }
    });

    this.wsConnections.set(connectionId, ws);
    return connectionId;
  }

  /**
   * Unsubscribe from real-time tracking updates
   * @param connectionId - Connection ID returned from subscribeToTracking
   */
  unsubscribeFromTracking(connectionId: string): void {
    const ws = this.wsConnections.get(connectionId);
    if (ws) {
      ws.close();
      this.wsConnections.delete(connectionId);
    }
  }

  // ==================== Cargo API ====================

  /**
   * Create a new cargo shipment
   * @param cargo - Cargo shipment data
   * @returns Created shipment with tracking number
   */
  async createCargoShipment(
    cargo: Omit<CargoShipment, 'id' | '@context' | 'trackingNumber' | 'status'>
  ): Promise<APIResponse<CargoShipment>> {
    const payload = {
      '@context': 'https://wiastandards.com/contexts/uni-008/v1',
      ...cargo,
    };
    const response = await this.client.post<APIResponse<CargoShipment>>('/cargo', payload);
    return response.data;
  }

  /**
   * Track cargo shipment
   * @param trackingNumber - Shipment tracking number
   * @returns Current shipment status and location
   */
  async trackCargo(trackingNumber: string): Promise<CargoShipment> {
    const response = await this.client.get<CargoShipment>(`/cargo/${trackingNumber}`);
    return response.data;
  }

  /**
   * List cargo shipments
   * @param params - Query parameters
   * @returns Paginated list of shipments
   */
  async listCargo(params?: QueryParams): Promise<PaginatedResponse<CargoShipment>> {
    const response = await this.client.get<PaginatedResponse<CargoShipment>>('/cargo', {
      params,
    });
    return response.data;
  }

  // ==================== Vehicles API ====================

  /**
   * Register a new vehicle
   * @param vehicle - Vehicle data
   * @returns Registered vehicle
   */
  async registerVehicle(
    vehicle: Omit<TransportVehicle, 'id' | '@context'>
  ): Promise<APIResponse<TransportVehicle>> {
    const payload = {
      '@context': 'https://wiastandards.com/contexts/uni-008/v1',
      ...vehicle,
    };
    const response = await this.client.post<APIResponse<TransportVehicle>>('/vehicles', payload);
    return response.data;
  }

  /**
   * Get vehicle details
   * @param vehicleId - Vehicle identifier
   * @returns Vehicle information
   */
  async getVehicle(vehicleId: string): Promise<TransportVehicle> {
    const response = await this.client.get<TransportVehicle>(`/vehicles/${vehicleId}`);
    return response.data;
  }

  /**
   * List vehicles
   * @param params - Query parameters
   * @returns Paginated list of vehicles
   */
  async listVehicles(params?: QueryParams): Promise<PaginatedResponse<TransportVehicle>> {
    const response = await this.client.get<PaginatedResponse<TransportVehicle>>('/vehicles', {
      params,
    });
    return response.data;
  }

  // ==================== Webhooks API ====================

  /**
   * Register a webhook
   * @param config - Webhook configuration
   * @returns Registered webhook
   */
  async registerWebhook(config: WebhookConfig): Promise<Webhook> {
    const response = await this.client.post<Webhook>('/webhooks', config);
    return response.data;
  }

  /**
   * List registered webhooks
   * @returns List of webhooks
   */
  async listWebhooks(): Promise<Webhook[]> {
    const response = await this.client.get<Webhook[]>('/webhooks');
    return response.data;
  }

  /**
   * Delete a webhook
   * @param webhookId - Webhook identifier
   */
  async deleteWebhook(webhookId: string): Promise<void> {
    await this.client.delete(`/webhooks/${webhookId}`);
  }

  // ==================== Utility Methods ====================

  /**
   * Close all WebSocket connections
   */
  closeAllConnections(): void {
    this.wsConnections.forEach((ws) => ws.close());
    this.wsConnections.clear();
  }

  /**
   * Update access token
   * @param newToken - New OAuth 2.0 access token
   */
  updateAccessToken(newToken: string): void {
    this.config.accessToken = newToken;
    this.client.defaults.headers['Authorization'] = `Bearer ${newToken}`;
  }
}

// Export all types
export * from './types';
