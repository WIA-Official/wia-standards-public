/**
 * WIA-SOC-007: Public Transportation Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import * as types from './types';

export * from './types';

export class WiaPublicTransit {
  private apiKey: string;
  private baseUrl: string;

  constructor(config: { apiKey: string; baseUrl?: string }) {
    this.apiKey = config.apiKey;
    this.baseUrl = config.baseUrl || 'https://api.transit.example.com/v1';
  }

  /**
   * Get list of all transit agencies
   */
  async getAgencies(): Promise<types.TransitAgency[]> {
    return this.request('/agencies');
  }

  /**
   * Get specific agency by ID
   */
  async getAgency(agencyId: string): Promise<types.TransitAgency> {
    return this.request(`/agencies/${agencyId}`);
  }

  /**
   * Get all routes
   */
  async getRoutes(params?: {
    agencyId?: string;
    routeType?: types.RouteType;
  }): Promise<types.TransitRoute[]> {
    return this.request('/routes', params);
  }

  /**
   * Get specific route by ID
   */
  async getRoute(routeId: string): Promise<types.TransitRoute> {
    return this.request(`/routes/${routeId}`);
  }

  /**
   * Get all stops
   */
  async getStops(params?: {
    lat?: number;
    lon?: number;
    radius?: number;
  }): Promise<types.TransitStop[]> {
    return this.request('/stops', params);
  }

  /**
   * Get specific stop by ID
   */
  async getStop(stopId: string): Promise<types.TransitStop> {
    return this.request(`/stops/${stopId}`);
  }

  /**
   * Get next arrivals at a stop
   */
  async getStopArrivals(
    stopId: string,
    params?: {
      routeId?: string;
      limit?: number;
      timeframe?: number;
    }
  ): Promise<types.StopArrivalsResponse> {
    return this.request(`/stops/${stopId}/arrivals`, params);
  }

  /**
   * Plan a trip from origin to destination
   */
  async planTrip(request: types.TripPlanRequest): Promise<types.Itinerary[]> {
    return this.request('/trip-planner', undefined, {
      method: 'POST',
      body: JSON.stringify(request)
    });
  }

  /**
   * Get current vehicle position
   */
  async getVehiclePosition(vehicleId: string): Promise<types.VehiclePosition> {
    return this.request(`/vehicles/${vehicleId}/position`);
  }

  /**
   * Get all vehicles on a route
   */
  async getRouteVehicles(routeId: string): Promise<types.VehiclePosition[]> {
    return this.request(`/routes/${routeId}/vehicles`);
  }

  /**
   * Get active service alerts
   */
  async getAlerts(params?: {
    routeId?: string;
    stopId?: string;
    agencyId?: string;
  }): Promise<types.ServiceAlert[]> {
    return this.request('/alerts', params);
  }

  /**
   * Subscribe to real-time updates via WebSocket
   */
  subscribeToRealtime(channels: string[]): WebSocket {
    const wsUrl = this.baseUrl.replace(/^http/, 'ws') + '/realtime';
    const ws = new WebSocket(wsUrl);

    ws.onopen = () => {
      ws.send(JSON.stringify({
        type: 'subscribe',
        channels,
        apiKey: this.apiKey
      }));
    };

    return ws;
  }

  /**
   * Make HTTP request to API
   */
  private async request(
    path: string,
    params?: Record<string, any>,
    options?: RequestInit
  ): Promise<any> {
    const url = new URL(path, this.baseUrl);

    if (params) {
      Object.keys(params).forEach(key => {
        if (params[key] !== undefined) {
          url.searchParams.append(key, params[key]);
        }
      });
    }

    const response = await fetch(url.toString(), {
      ...options,
      headers: {
        'X-API-Key': this.apiKey,
        'Content-Type': 'application/json',
        ...options?.headers
      }
    });

    if (!response.ok) {
      const error: types.APIError = await response.json();
      throw new Error(`API Error: ${error.message} (${error.code})`);
    }

    return response.json();
  }
}
