/**
 * WIA Direct Air Capture Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIADirectAirCapture, FacilityResponse, CarbonMetrics,
  EnergyConsumption, SensorConfig, ValidationResult, PaginatedResponse,
} from './types';

export class WIADirectAirCaptureClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: { 'Content-Type': 'application/json', ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }) },
    });
  }

  async createFacility(facility: WIADirectAirCapture): Promise<FacilityResponse> {
    const response = await this.axios.post<FacilityResponse>('/facilities', facility);
    return response.data;
  }

  async getFacility(id: string): Promise<WIADirectAirCapture> {
    const response = await this.axios.get<WIADirectAirCapture>(`/facilities/${id}`);
    return response.data;
  }

  async listFacilities(params?: { status?: string; country?: string; limit?: number }): Promise<PaginatedResponse<FacilityResponse>> {
    const response = await this.axios.get<PaginatedResponse<FacilityResponse>>('/facilities', { params });
    return response.data;
  }

  async updateFacility(id: string, updates: Partial<WIADirectAirCapture>): Promise<FacilityResponse> {
    const response = await this.axios.put<FacilityResponse>(`/facilities/${id}`, updates);
    return response.data;
  }

  async getCarbonMetrics(facilityId: string, period?: { from?: string; to?: string }): Promise<CarbonMetrics> {
    const response = await this.axios.get<CarbonMetrics>(`/facilities/${facilityId}/carbon`, { params: period });
    return response.data;
  }

  async getEnergyConsumption(facilityId: string, period?: { from?: string; to?: string }): Promise<EnergyConsumption> {
    const response = await this.axios.get<EnergyConsumption>(`/facilities/${facilityId}/energy`, { params: period });
    return response.data;
  }

  async getSensorData(facilityId: string, sensorId: string, params?: { from?: string; to?: string; resolution?: string }): Promise<{ timestamp: string; value: number }[]> {
    const response = await this.axios.get(`/facilities/${facilityId}/sensors/${sensorId}/data`, { params });
    return response.data;
  }

  async listSensors(facilityId: string): Promise<SensorConfig[]> {
    const response = await this.axios.get<SensorConfig[]>(`/facilities/${facilityId}/sensors`);
    return response.data;
  }

  async getOperationalStatus(facilityId: string): Promise<{ status: string; uptime: number; efficiency: number; alerts: number }> {
    const response = await this.axios.get(`/facilities/${facilityId}/status`);
    return response.data;
  }

  async reportCapture(facilityId: string, data: { amount: number; startTime: string; endTime: string; verification?: string }): Promise<{ reportId: string }> {
    const response = await this.axios.post(`/facilities/${facilityId}/capture-reports`, data);
    return response.data;
  }

  async getVerificationCertificate(facilityId: string, period: string): Promise<{ certificateId: string; url: string; validUntil: string }> {
    const response = await this.axios.get(`/facilities/${facilityId}/certificates`, { params: { period } });
    return response.data;
  }

  async getNetRemovalCalculation(facilityId: string): Promise<{ gross: number; processEmissions: number; supplyChain: number; net: number; efficiency: number }> {
    const response = await this.axios.get(`/facilities/${facilityId}/net-removal`);
    return response.data;
  }

  async scheduleMaintenance(facilityId: string, maintenance: { type: string; scheduledDate: string; duration: number }): Promise<{ maintenanceId: string }> {
    const response = await this.axios.post(`/facilities/${facilityId}/maintenance`, maintenance);
    return response.data;
  }

  validateFacility(facility: WIADirectAirCapture): ValidationResult {
    const errors: any[] = [];
    if (facility.standard !== 'WIA-DIRECT-AIR-CAPTURE') errors.push({ path: 'standard', message: 'Invalid standard' });
    if (!facility.facility?.id) errors.push({ path: 'facility.id', message: 'Facility ID required' });
    if (!facility.technology?.type) errors.push({ path: 'technology.type', message: 'Technology type required' });
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16);
  });
}

export function createMinimalFacility(name: string, lat: number, lng: number, capacityTons: number): WIADirectAirCapture {
  return {
    standard: 'WIA-DIRECT-AIR-CAPTURE',
    version: '1.0.0',
    facility: {
      id: generateUUID(), name, status: 'planning',
      location: { latitude: lat, longitude: lng, altitude: 0, country: '', timezone: 'UTC' },
      operator: { id: generateUUID(), name: 'Operator', type: 'private', contact: { email: 'contact@example.com' } },
      capacity: { designCapacity: capacityTons, currentCapacity: 0, unit: 'tCO2/year' },
    },
    technology: {
      type: 'solid-sorbent', specifications: {
        captureEfficiency: 90, energyRequirement: { thermal: 5, electrical: 0.5, unit: 'GJ/tCO2' },
        operatingTemperature: { min: 20, max: 100, optimal: 80, unit: 'celsius' },
        operatingPressure: { min: 1, max: 2, unit: 'bar' }, cycleTime: 3600, lifetime: 20,
      },
      equipment: [],
    },
    operations: {
      mode: 'continuous', schedule: { hoursPerDay: 24, daysPerYear: 350, plannedDowntime: 15 },
      maintenance: { preventive: [], predictive: true, averageDowntimePercent: 5 },
      automation: 'fully-automated', safety: { emergencyShutdown: true, leakDetection: true, fireSupression: true, safetyZone: 100, permits: [] },
    },
    carbon: {
      methodology: { standard: 'iso-14064', boundaries: ['scope-1', 'scope-2'], reportingPeriod: 'annual' },
      captured: { totalCaptured: 0, periodCaptured: 0, cumulativeCaptured: 0, unit: 'tCO2', measurementUncertainty: 5 },
      stored: { method: 'geological-saline', capacity: capacityTons * 30, currentStored: 0, permanence: { expectedDuration: 1000, unit: 'years', monitoringRequired: true, leakageRisk: 'low' } },
      netRemoval: { grossCapture: 0, processEmissions: 0, supplyChainEmissions: 0, netRemoval: 0, removalEfficiency: 85 },
      verification: { verifier: '', standard: '', lastVerified: '', nextVerification: '' },
    },
    energy: { sources: [], consumption: { thermal: 0, electrical: 0, total: 0, unit: 'MWh/year' }, carbonIntensity: 0, renewablePercent: 100 },
    monitoring: { realtime: true, sensors: [], dataFrequency: 60, storage: { local: true, cloud: true, retention: 365, encryption: true }, alerts: [] },
  };
}

export default { WIADirectAirCaptureClient, generateUUID, createMinimalFacility };
