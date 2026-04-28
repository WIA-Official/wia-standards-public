/**
 * WIA-AGRI-006 Smart Irrigation Standard - TypeScript SDK
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';
import type {
  SDKConfig,
  SmartIrrigationConfig,
  IrrigationSchedule,
  IrrigationEvent,
  WaterMeterReading,
  SoilMoistureData,
  EvapotranspirationData,
  IrrigationRecommendation,
  ValveControl,
  PumpControl,
  FertigationRecord,
  SystemMaintenance,
  WaterUsageAnalytics,
  AlertConfig,
  SystemAlert,
  ListParams,
} from './types';

export * from './types';

/**
 * Main SDK Client for WIA-AGRI-006 Smart Irrigation
 */
export class SmartIrrigationClient {
  private axios: AxiosInstance;
  private config: SDKConfig;

  constructor(config: SDKConfig) {
    this.config = {
      baseURL: config.baseURL || 'https://api.wiastandards.com/v1/smart-irrigation',
      timeout: config.timeout || 30000,
      ...config,
    };

    this.axios = axios.create({
      baseURL: this.config.baseURL,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'X-API-Key': this.config.apiKey || '',
      },
    });
  }

  /**
   * Get system configuration
   */
  async getSystem(systemId?: string): Promise<SmartIrrigationConfig> {
    const id = systemId || this.config.systemId;
    if (!id) throw new Error('System ID is required');
    const response = await this.axios.get(`/systems/${id}`);
    return response.data;
  }

  /**
   * Create irrigation system
   */
  async createSystem(
    system: Omit<SmartIrrigationConfig, 'systemId' | 'status'>
  ): Promise<SmartIrrigationConfig> {
    const response = await this.axios.post('/systems', system);
    return response.data;
  }

  /**
   * Update system
   */
  async updateSystem(
    systemId: string,
    updates: Partial<SmartIrrigationConfig>
  ): Promise<SmartIrrigationConfig> {
    const response = await this.axios.patch(`/systems/${systemId}`, updates);
    return response.data;
  }

  /**
   * Get schedules
   */
  async getSchedules(systemId?: string, params?: ListParams): Promise<IrrigationSchedule[]> {
    const id = systemId || this.config.systemId;
    const response = await this.axios.get('/schedules', {
      params: { systemId: id, ...params },
    });
    return response.data;
  }

  /**
   * Create schedule
   */
  async createSchedule(
    schedule: Omit<IrrigationSchedule, 'scheduleId'>
  ): Promise<IrrigationSchedule> {
    const response = await this.axios.post('/schedules', schedule);
    return response.data;
  }

  /**
   * Update schedule
   */
  async updateSchedule(
    scheduleId: string,
    updates: Partial<IrrigationSchedule>
  ): Promise<IrrigationSchedule> {
    const response = await this.axios.patch(`/schedules/${scheduleId}`, updates);
    return response.data;
  }

  /**
   * Delete schedule
   */
  async deleteSchedule(scheduleId: string): Promise<void> {
    await this.axios.delete(`/schedules/${scheduleId}`);
  }

  /**
   * Get irrigation events
   */
  async getEvents(systemId?: string, params?: ListParams): Promise<IrrigationEvent[]> {
    const id = systemId || this.config.systemId;
    const response = await this.axios.get('/events', {
      params: { systemId: id, ...params },
    });
    return response.data;
  }

  /**
   * Start irrigation
   */
  async startIrrigation(
    systemId: string,
    params: { zoneId?: string; duration?: number; waterAmount?: number }
  ): Promise<IrrigationEvent> {
    const response = await this.axios.post(`/systems/${systemId}/start`, params);
    return response.data;
  }

  /**
   * Stop irrigation
   */
  async stopIrrigation(systemId: string, zoneId?: string): Promise<void> {
    await this.axios.post(`/systems/${systemId}/stop`, { zoneId });
  }

  /**
   * Get water meter readings
   */
  async getMeterReadings(
    systemId?: string,
    params?: ListParams
  ): Promise<WaterMeterReading[]> {
    const id = systemId || this.config.systemId;
    if (!id) throw new Error('System ID is required');
    const response = await this.axios.get(`/systems/${id}/meter-readings`, {
      params,
    });
    return response.data;
  }

  /**
   * Get soil moisture data
   */
  async getSoilMoisture(
    zoneId: string,
    params?: ListParams
  ): Promise<SoilMoistureData[]> {
    const response = await this.axios.get(`/zones/${zoneId}/moisture`, { params });
    return response.data;
  }

  /**
   * Get ET data
   */
  async getETData(
    location: { latitude: number; longitude: number },
    params?: { startDate?: string; endDate?: string }
  ): Promise<EvapotranspirationData[]> {
    const response = await this.axios.get('/evapotranspiration', {
      params: { ...location, ...params },
    });
    return response.data;
  }

  /**
   * Get recommendations
   */
  async getRecommendations(
    systemId?: string,
    zoneId?: string
  ): Promise<IrrigationRecommendation[]> {
    const id = systemId || this.config.systemId;
    const response = await this.axios.get('/recommendations', {
      params: { systemId: id, zoneId },
    });
    return response.data;
  }

  /**
   * Get valve status
   */
  async getValves(systemId?: string): Promise<ValveControl[]> {
    const id = systemId || this.config.systemId;
    if (!id) throw new Error('System ID is required');
    const response = await this.axios.get(`/systems/${id}/valves`);
    return response.data;
  }

  /**
   * Control valve
   */
  async controlValve(
    valveId: string,
    action: 'open' | 'close' | 'partial',
    position?: number
  ): Promise<ValveControl> {
    const response = await this.axios.post(`/valves/${valveId}/control`, {
      action,
      position,
    });
    return response.data;
  }

  /**
   * Get pumps
   */
  async getPumps(systemId?: string): Promise<PumpControl[]> {
    const id = systemId || this.config.systemId;
    if (!id) throw new Error('System ID is required');
    const response = await this.axios.get(`/systems/${id}/pumps`);
    return response.data;
  }

  /**
   * Control pump
   */
  async controlPump(
    pumpId: string,
    action: 'on' | 'off',
    frequency?: number
  ): Promise<PumpControl> {
    const response = await this.axios.post(`/pumps/${pumpId}/control`, {
      action,
      frequency,
    });
    return response.data;
  }

  /**
   * Get fertigation records
   */
  async getFertigationRecords(
    systemId?: string,
    params?: ListParams
  ): Promise<FertigationRecord[]> {
    const id = systemId || this.config.systemId;
    const response = await this.axios.get('/fertigation', {
      params: { systemId: id, ...params },
    });
    return response.data;
  }

  /**
   * Record fertigation
   */
  async recordFertigation(
    record: Omit<FertigationRecord, 'recordId'>
  ): Promise<FertigationRecord> {
    const response = await this.axios.post('/fertigation', record);
    return response.data;
  }

  /**
   * Get maintenance records
   */
  async getMaintenanceRecords(
    systemId?: string,
    params?: ListParams
  ): Promise<SystemMaintenance[]> {
    const id = systemId || this.config.systemId;
    if (!id) throw new Error('System ID is required');
    const response = await this.axios.get(`/systems/${id}/maintenance`, { params });
    return response.data;
  }

  /**
   * Add maintenance record
   */
  async addMaintenanceRecord(
    record: Omit<SystemMaintenance, 'maintenanceId'>
  ): Promise<SystemMaintenance> {
    const response = await this.axios.post('/maintenance', record);
    return response.data;
  }

  /**
   * Get water usage analytics
   */
  async getWaterUsageAnalytics(
    systemId?: string,
    params?: { startDate?: string; endDate?: string }
  ): Promise<WaterUsageAnalytics> {
    const id = systemId || this.config.systemId;
    if (!id) throw new Error('System ID is required');
    const response = await this.axios.get(`/systems/${id}/analytics`, { params });
    return response.data;
  }

  /**
   * Get alerts
   */
  async getAlerts(systemId?: string, params?: ListParams): Promise<SystemAlert[]> {
    const id = systemId || this.config.systemId;
    const response = await this.axios.get('/alerts', {
      params: { systemId: id, ...params },
    });
    return response.data;
  }

  /**
   * Configure alert
   */
  async configureAlert(
    alert: Omit<AlertConfig, 'alertId'>
  ): Promise<AlertConfig> {
    const response = await this.axios.post('/alert-configs', alert);
    return response.data;
  }
}

/**
 * Schedule Manager for irrigation scheduling
 */
export class ScheduleManager {
  private client: SmartIrrigationClient;

  constructor(config: SDKConfig) {
    this.client = new SmartIrrigationClient(config);
  }

  /**
   * Create time-based schedule
   */
  async createTimeBasedSchedule(
    systemId: string,
    zoneId: string,
    time: string,
    duration: number
  ): Promise<IrrigationSchedule> {
    return this.client.createSchedule({
      systemId,
      zoneId,
      name: `Daily irrigation at ${time}`,
      type: 'time_based',
      enabled: true,
      timing: {
        type: 'daily',
        time,
      },
      duration,
    });
  }

  /**
   * Get active schedules
   */
  async getActiveSchedules(systemId?: string): Promise<IrrigationSchedule[]> {
    const schedules = await this.client.getSchedules(systemId);
    return schedules.filter((s) => s.enabled);
  }
}

/**
 * Zone Manager for irrigation zones
 */
export class ZoneManager {
  private client: SmartIrrigationClient;

  constructor(config: SDKConfig) {
    this.client = new SmartIrrigationClient(config);
  }

  /**
   * Get zone moisture status
   */
  async getZoneMoistureStatus(zoneId: string): Promise<{
    current: number;
    status: 'dry' | 'optimal' | 'wet';
  }> {
    const moistureData = await this.client.getSoilMoisture(zoneId, { limit: 1 });
    const current = moistureData[0]?.moisture || 0;

    let status: 'dry' | 'optimal' | 'wet';
    if (current < 25) status = 'dry';
    else if (current > 50) status = 'wet';
    else status = 'optimal';

    return { current, status };
  }
}

/**
 * Utility functions
 */
export const utils = {
  /**
   * Calculate irrigation time from water amount
   */
  calculateIrrigationTime(waterAmount: number, flowRate: number): number {
    return (waterAmount / flowRate) * 60; // Returns minutes
  },

  /**
   * Calculate water requirement from ET
   */
  calculateWaterRequirement(etc: number, area: number, efficiency: number = 0.9): number {
    return (etc * area * 10000) / efficiency; // Returns liters
  },

  /**
   * Estimate cost
   */
  estimateCost(waterUsed: number, costPerUnit: number): number {
    return waterUsed * costPerUnit;
  },
};

/**
 * Default export
 */
export default {
  SmartIrrigationClient,
  ScheduleManager,
  ZoneManager,
  utils,
};
