/**
 * WIA Distributed Energy Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIADistributedEnergy, NetworkResponse, EnergyResource, StorageSystem,
  LoadProfile, EnergyContract, ValidationResult, PaginatedResponse,
} from './types';

export class WIADistributedEnergyClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: { 'Content-Type': 'application/json', ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }) },
    });
  }

  async createNetwork(network: WIADistributedEnergy): Promise<NetworkResponse> {
    const response = await this.axios.post<NetworkResponse>('/networks', network);
    return response.data;
  }

  async getNetwork(id: string): Promise<WIADistributedEnergy> {
    const response = await this.axios.get<WIADistributedEnergy>(`/networks/${id}`);
    return response.data;
  }

  async listNetworks(params?: { type?: string; status?: string; limit?: number }): Promise<PaginatedResponse<NetworkResponse>> {
    const response = await this.axios.get<PaginatedResponse<NetworkResponse>>('/networks', { params });
    return response.data;
  }

  async updateNetwork(id: string, updates: Partial<WIADistributedEnergy>): Promise<NetworkResponse> {
    const response = await this.axios.put<NetworkResponse>(`/networks/${id}`, updates);
    return response.data;
  }

  async addResource(networkId: string, resource: EnergyResource): Promise<EnergyResource> {
    const response = await this.axios.post<EnergyResource>(`/networks/${networkId}/resources`, resource);
    return response.data;
  }

  async listResources(networkId: string, type?: string): Promise<EnergyResource[]> {
    const response = await this.axios.get<EnergyResource[]>(`/networks/${networkId}/resources`, { params: { type } });
    return response.data;
  }

  async getResourceOutput(networkId: string, resourceId: string, params?: { from?: string; to?: string }): Promise<{ timestamp: string; output: number }[]> {
    const response = await this.axios.get(`/networks/${networkId}/resources/${resourceId}/output`, { params });
    return response.data;
  }

  async addStorageSystem(networkId: string, storage: StorageSystem): Promise<StorageSystem> {
    const response = await this.axios.post<StorageSystem>(`/networks/${networkId}/storage`, storage);
    return response.data;
  }

  async getStorageStatus(networkId: string, storageId: string): Promise<{ soc: number; power: number; status: string }> {
    const response = await this.axios.get(`/networks/${networkId}/storage/${storageId}/status`);
    return response.data;
  }

  async controlStorage(networkId: string, storageId: string, command: { action: 'charge' | 'discharge' | 'idle'; power?: number }): Promise<void> {
    await this.axios.post(`/networks/${networkId}/storage/${storageId}/control`, command);
  }

  async getLoadProfiles(networkId: string): Promise<LoadProfile[]> {
    const response = await this.axios.get<LoadProfile[]>(`/networks/${networkId}/loads`);
    return response.data;
  }

  async getCurrentDemand(networkId: string): Promise<{ total: number; breakdown: { type: string; demand: number }[] }> {
    const response = await this.axios.get(`/networks/${networkId}/demand`);
    return response.data;
  }

  async createContract(networkId: string, contract: Omit<EnergyContract, 'id' | 'status'>): Promise<EnergyContract> {
    const response = await this.axios.post<EnergyContract>(`/networks/${networkId}/contracts`, contract);
    return response.data;
  }

  async listContracts(networkId: string): Promise<EnergyContract[]> {
    const response = await this.axios.get<EnergyContract[]>(`/networks/${networkId}/contracts`);
    return response.data;
  }

  async getEnergyBalance(networkId: string): Promise<{ generation: number; consumption: number; storage: number; gridExchange: number }> {
    const response = await this.axios.get(`/networks/${networkId}/balance`);
    return response.data;
  }

  async getForecast(networkId: string, type: 'generation' | 'load' | 'price', horizon: number): Promise<{ timestamp: string; value: number }[]> {
    const response = await this.axios.get(`/networks/${networkId}/forecast/${type}`, { params: { horizon } });
    return response.data;
  }

  async triggerIsland(networkId: string): Promise<{ success: boolean; transitionTime: number }> {
    const response = await this.axios.post(`/networks/${networkId}/island`);
    return response.data;
  }

  async reconnectGrid(networkId: string): Promise<{ success: boolean; syncTime: number }> {
    const response = await this.axios.post(`/networks/${networkId}/reconnect`);
    return response.data;
  }

  validateNetwork(network: WIADistributedEnergy): ValidationResult {
    const errors: any[] = [];
    if (network.standard !== 'WIA-DISTRIBUTED-ENERGY') errors.push({ path: 'standard', message: 'Invalid standard' });
    if (!network.network?.id) errors.push({ path: 'network.id', message: 'Network ID required' });
    if (!network.resources?.length) errors.push({ path: 'resources', message: 'At least one resource required' });
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16);
  });
}

export function createMinimalNetwork(name: string, lat: number, lng: number): WIADistributedEnergy {
  return {
    standard: 'WIA-DISTRIBUTED-ENERGY',
    version: '1.0.0',
    network: {
      id: generateUUID(), name, type: 'microgrid', status: 'planning', createdAt: new Date().toISOString(),
      operator: { id: generateUUID(), name: 'Operator', contact: { email: 'contact@example.com' } },
      location: { latitude: lat, longitude: lng, country: '', timezone: 'UTC' },
      capacity: { generation: 0, storage: 0, peakLoad: 0, unit: 'kW' },
    },
    resources: [],
    storage: [],
    loads: [],
    trading: {
      enabled: false, platform: { type: 'p2p' }, participants: [], contracts: [],
      settlements: { frequency: 'hourly', currency: 'USD', paymentMethod: 'fiat', reconciliation: true },
    },
    grid: {
      connectionPoint: { id: generateUUID(), voltage: 400, frequency: 50, phases: 3, maxImport: 100, maxExport: 100 },
      interconnection: { utility: '', agreementId: '', tariff: '', netMetering: true },
      services: [], islanding: { capable: true, automatic: false, criticalLoadsSupported: [] },
    },
    monitoring: {
      scada: { enabled: true, protocol: 'modbus', pollInterval: 1000, redundancy: false },
      analytics: { enabled: true, metrics: ['generation', 'consumption', 'storage'], dashboard: true, retention: 365 },
      forecasting: { generation: true, load: true, price: false, horizon: 24, models: ['persistence'] },
      alerts: [],
    },
  };
}

export default { WIADistributedEnergyClient, generateUUID, createMinimalNetwork };
