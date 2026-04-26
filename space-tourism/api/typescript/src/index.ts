/**
 * WIA-SPACE-014: Space Tourism Standard - TypeScript SDK
 */

import { Booking, FlightType, APIResponse } from './types';

export class SpaceTourismSDK {
  private baseURL: string;
  private apiKey: string;

  constructor(baseURL: string, apiKey: string) {
    this.baseURL = baseURL.replace(/\/$/, '');
    this.apiKey = apiKey;
  }

  private async request<T>(endpoint: string, options: RequestInit = {}): Promise<APIResponse<T>> {
    try {
      const response = await fetch(`${this.baseURL}${endpoint}`, {
        ...options,
        headers: { 'Authorization': `Bearer ${this.apiKey}`, 'Content-Type': 'application/json', ...options.headers }
      });
      const data = await response.json();
      return { status: response.ok ? 'success' : 'error', data: response.ok ? data : undefined, error: response.ok ? undefined : data.message, timestamp: new Date().toISOString() };
    } catch (error) {
      return { status: 'error', error: error instanceof Error ? error.message : 'Unknown error', timestamp: new Date().toISOString() };
    }
  }

  async getBookings(): Promise<APIResponse<Booking[]>> {
    return this.request<Booking[]>('/api/v1/tourism/bookings');
  }

  async createBooking(booking: Booking): Promise<APIResponse<{ booking_id: string }>> {
    return this.request<{ booking_id: string }>('/api/v1/tourism/bookings', {
      method: 'POST',
      body: JSON.stringify(booking)
    });
  }

  calculateCost(type: FlightType, duration_days: number): number {
    const baseCosts: Record<FlightType, number> = {
      [FlightType.SUBORBITAL]: 250000,
      [FlightType.ORBITAL]: 50000000,
      [FlightType.SPACE_STATION]: 55000000,
      [FlightType.LUNAR_FLYBY]: 100000000
    };
    const baseCost = baseCosts[type];
    return baseCost + (duration_days > 1 ? (duration_days - 1) * baseCost * 0.2 : 0);
  }
}

export * from './types';
export default SpaceTourismSDK;
