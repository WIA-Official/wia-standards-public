/**
 * WIA Digital Twin City Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIADigitalTwinCity, TwinResponse, DataLayer, SimulationConfig,
  SensorCategory, Dashboard, ValidationResult, PaginatedResponse,
} from './types';

export class WIADigitalTwinCityClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: { 'Content-Type': 'application/json', ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }) },
    });
  }

  async createTwin(twin: WIADigitalTwinCity): Promise<TwinResponse> {
    const response = await this.axios.post<TwinResponse>('/twins', twin);
    return response.data;
  }

  async getTwin(id: string): Promise<WIADigitalTwinCity> {
    const response = await this.axios.get<WIADigitalTwinCity>(`/twins/${id}`);
    return response.data;
  }

  async listTwins(params?: { status?: string; limit?: number }): Promise<PaginatedResponse<TwinResponse>> {
    const response = await this.axios.get<PaginatedResponse<TwinResponse>>('/twins', { params });
    return response.data;
  }

  async updateTwin(id: string, updates: Partial<WIADigitalTwinCity>): Promise<TwinResponse> {
    const response = await this.axios.put<TwinResponse>(`/twins/${id}`, updates);
    return response.data;
  }

  async syncTwin(id: string): Promise<{ status: string; syncedLayers: number }> {
    const response = await this.axios.post(`/twins/${id}/sync`);
    return response.data;
  }

  async addLayer(twinId: string, layer: DataLayer): Promise<DataLayer> {
    const response = await this.axios.post<DataLayer>(`/twins/${twinId}/layers`, layer);
    return response.data;
  }

  async getLayers(twinId: string, type?: string): Promise<DataLayer[]> {
    const response = await this.axios.get<DataLayer[]>(`/twins/${twinId}/layers`, { params: { type } });
    return response.data;
  }

  async queryLayer(twinId: string, layerId: string, query: { bbox?: number[]; filter?: string; limit?: number }): Promise<any[]> {
    const response = await this.axios.post(`/twins/${twinId}/layers/${layerId}/query`, query);
    return response.data;
  }

  async getSensorData(twinId: string, sensorType: string, params?: { from?: string; to?: string }): Promise<{ timestamp: string; values: Record<string, number> }[]> {
    const response = await this.axios.get(`/twins/${twinId}/sensors/${sensorType}/data`, { params });
    return response.data;
  }

  async getSensorStatus(twinId: string): Promise<SensorCategory[]> {
    const response = await this.axios.get<SensorCategory[]>(`/twins/${twinId}/sensors/status`);
    return response.data;
  }

  async runSimulation(twinId: string, simulation: SimulationConfig): Promise<{ simulationId: string; status: string }> {
    const response = await this.axios.post(`/twins/${twinId}/simulations`, simulation);
    return response.data;
  }

  async getSimulationResults(twinId: string, simulationId: string): Promise<{ status: string; progress: number; results?: any }> {
    const response = await this.axios.get(`/twins/${twinId}/simulations/${simulationId}`);
    return response.data;
  }

  async get3DView(twinId: string, viewport: { center: number[]; zoom: number }): Promise<{ url: string; format: string }> {
    const response = await this.axios.post(`/twins/${twinId}/view/3d`, viewport);
    return response.data;
  }

  async getAnalytics(twinId: string, metric: string, params?: { from?: string; to?: string; aggregation?: string }): Promise<{ data: { timestamp: string; value: number }[] }> {
    const response = await this.axios.get(`/twins/${twinId}/analytics/${metric}`, { params });
    return response.data;
  }

  async getDashboard(twinId: string, dashboardId: string): Promise<Dashboard> {
    const response = await this.axios.get<Dashboard>(`/twins/${twinId}/dashboards/${dashboardId}`);
    return response.data;
  }

  validateTwin(twin: WIADigitalTwinCity): ValidationResult {
    const errors: any[] = [];
    if (twin.standard !== 'WIA-DIGITAL-TWIN-CITY') errors.push({ path: 'standard', message: 'Invalid standard' });
    if (!twin.twin?.id) errors.push({ path: 'twin.id', message: 'Twin ID required' });
    if (!twin.city?.name) errors.push({ path: 'city.name', message: 'City name required' });
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16);
  });
}

export function createMinimalTwin(cityName: string, lat: number, lng: number): WIADigitalTwinCity {
  return {
    standard: 'WIA-DIGITAL-TWIN-CITY',
    version: '1.0.0',
    twin: { id: generateUUID(), name: `${cityName} Digital Twin`, status: 'building', fidelity: 'medium', createdAt: new Date().toISOString(), owner: 'default' },
    city: { name: cityName, country: '', population: 0, area: 0, timezone: 'UTC', coordinates: { latitude: lat, longitude: lng }, boundary: { type: 'Polygon', coordinates: [] } },
    layers: [],
    sensors: { totalSensors: 0, categories: [], protocols: [] },
    simulations: [],
    integrations: [],
  };
}

export default { WIADigitalTwinCityClient, generateUUID, createMinimalTwin };
