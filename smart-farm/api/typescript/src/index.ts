/**
 * WIA-AGRI-001 Smart Farm Standard - TypeScript SDK
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';
import type {
  SDKConfig,
  SmartFarmConfig,
  SmartFarmDevice,
  SensorReading,
  EnvironmentalData,
  FarmAlert,
  AutomationRule,
  DeviceControlCommand,
  DeviceControlResponse,
  FarmAnalytics,
  FarmZone,
  HarvestData,
  ListParams,
} from './types';

export * from './types';

/**
 * Main SDK Client for WIA-AGRI-001 Smart Farm
 */
export class SmartFarmClient {
  private axios: AxiosInstance;
  private config: SDKConfig;

  constructor(config: SDKConfig) {
    this.config = {
      baseURL: config.baseURL || 'https://api.wiastandards.com/v1/smart-farm',
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
   * Get farm configuration
   */
  async getFarm(farmId?: string): Promise<SmartFarmConfig> {
    const id = farmId || this.config.farmId;
    if (!id) throw new Error('Farm ID is required');
    const response = await this.axios.get(`/farms/${id}`);
    return response.data;
  }

  /**
   * Create a new farm
   */
  async createFarm(farm: Omit<SmartFarmConfig, 'farmId'>): Promise<SmartFarmConfig> {
    const response = await this.axios.post('/farms', farm);
    return response.data;
  }

  /**
   * Update farm configuration
   */
  async updateFarm(
    farmId: string,
    updates: Partial<SmartFarmConfig>
  ): Promise<SmartFarmConfig> {
    const response = await this.axios.patch(`/farms/${farmId}`, updates);
    return response.data;
  }

  /**
   * Delete a farm
   */
  async deleteFarm(farmId: string): Promise<void> {
    await this.axios.delete(`/farms/${farmId}`);
  }

  /**
   * List all devices in a farm
   */
  async listDevices(farmId?: string, params?: ListParams): Promise<SmartFarmDevice[]> {
    const id = farmId || this.config.farmId;
    if (!id) throw new Error('Farm ID is required');
    const response = await this.axios.get(`/farms/${id}/devices`, { params });
    return response.data;
  }

  /**
   * Get device details
   */
  async getDevice(deviceId: string): Promise<SmartFarmDevice> {
    const response = await this.axios.get(`/devices/${deviceId}`);
    return response.data;
  }

  /**
   * Register a new device
   */
  async registerDevice(
    farmId: string,
    device: Omit<SmartFarmDevice, 'deviceId' | 'status' | 'lastSeen'>
  ): Promise<SmartFarmDevice> {
    const response = await this.axios.post(`/farms/${farmId}/devices`, device);
    return response.data;
  }

  /**
   * Update device configuration
   */
  async updateDevice(
    deviceId: string,
    updates: Partial<SmartFarmDevice>
  ): Promise<SmartFarmDevice> {
    const response = await this.axios.patch(`/devices/${deviceId}`, updates);
    return response.data;
  }

  /**
   * Remove a device
   */
  async removeDevice(deviceId: string): Promise<void> {
    await this.axios.delete(`/devices/${deviceId}`);
  }

  /**
   * Get latest sensor readings
   */
  async getSensorReadings(
    farmId?: string,
    params?: ListParams & { deviceId?: string; sensorType?: string }
  ): Promise<SensorReading[]> {
    const id = farmId || this.config.farmId;
    if (!id) throw new Error('Farm ID is required');
    const response = await this.axios.get(`/farms/${id}/sensors/readings`, { params });
    return response.data;
  }

  /**
   * Submit sensor reading
   */
  async submitSensorReading(reading: SensorReading): Promise<void> {
    await this.axios.post('/sensors/readings', reading);
  }

  /**
   * Get environmental data
   */
  async getEnvironmentalData(
    farmId?: string,
    params?: ListParams
  ): Promise<EnvironmentalData[]> {
    const id = farmId || this.config.farmId;
    if (!id) throw new Error('Farm ID is required');
    const response = await this.axios.get(`/farms/${id}/environmental`, { params });
    return response.data;
  }

  /**
   * Get current environmental snapshot
   */
  async getCurrentEnvironment(farmId?: string): Promise<EnvironmentalData> {
    const id = farmId || this.config.farmId;
    if (!id) throw new Error('Farm ID is required');
    const response = await this.axios.get(`/farms/${id}/environmental/current`);
    return response.data;
  }

  /**
   * List farm alerts
   */
  async listAlerts(
    farmId?: string,
    params?: ListParams & { severity?: string; acknowledged?: boolean }
  ): Promise<FarmAlert[]> {
    const id = farmId || this.config.farmId;
    if (!id) throw new Error('Farm ID is required');
    const response = await this.axios.get(`/farms/${id}/alerts`, { params });
    return response.data;
  }

  /**
   * Acknowledge an alert
   */
  async acknowledgeAlert(alertId: string, userId: string): Promise<FarmAlert> {
    const response = await this.axios.post(`/alerts/${alertId}/acknowledge`, { userId });
    return response.data;
  }

  /**
   * Resolve an alert
   */
  async resolveAlert(alertId: string): Promise<FarmAlert> {
    const response = await this.axios.post(`/alerts/${alertId}/resolve`);
    return response.data;
  }

  /**
   * List automation rules
   */
  async listRules(farmId?: string, params?: ListParams): Promise<AutomationRule[]> {
    const id = farmId || this.config.farmId;
    if (!id) throw new Error('Farm ID is required');
    const response = await this.axios.get(`/farms/${id}/rules`, { params });
    return response.data;
  }

  /**
   * Create automation rule
   */
  async createRule(
    farmId: string,
    rule: Omit<AutomationRule, 'ruleId' | 'createdAt'>
  ): Promise<AutomationRule> {
    const response = await this.axios.post(`/farms/${farmId}/rules`, rule);
    return response.data;
  }

  /**
   * Update automation rule
   */
  async updateRule(
    ruleId: string,
    updates: Partial<AutomationRule>
  ): Promise<AutomationRule> {
    const response = await this.axios.patch(`/rules/${ruleId}`, updates);
    return response.data;
  }

  /**
   * Delete automation rule
   */
  async deleteRule(ruleId: string): Promise<void> {
    await this.axios.delete(`/rules/${ruleId}`);
  }

  /**
   * Control a device
   */
  async controlDevice(command: DeviceControlCommand): Promise<DeviceControlResponse> {
    const response = await this.axios.post('/devices/control', command);
    return response.data;
  }

  /**
   * Get farm analytics
   */
  async getAnalytics(
    farmId?: string,
    params?: { startDate?: string; endDate?: string; metrics?: string[] }
  ): Promise<FarmAnalytics> {
    const id = farmId || this.config.farmId;
    if (!id) throw new Error('Farm ID is required');
    const response = await this.axios.get(`/farms/${id}/analytics`, { params });
    return response.data;
  }

  /**
   * List farm zones
   */
  async listZones(farmId?: string): Promise<FarmZone[]> {
    const id = farmId || this.config.farmId;
    if (!id) throw new Error('Farm ID is required');
    const response = await this.axios.get(`/farms/${id}/zones`);
    return response.data;
  }

  /**
   * Create farm zone
   */
  async createZone(
    farmId: string,
    zone: Omit<FarmZone, 'zoneId' | 'farmId'>
  ): Promise<FarmZone> {
    const response = await this.axios.post(`/farms/${farmId}/zones`, zone);
    return response.data;
  }

  /**
   * Update farm zone
   */
  async updateZone(zoneId: string, updates: Partial<FarmZone>): Promise<FarmZone> {
    const response = await this.axios.patch(`/zones/${zoneId}`, updates);
    return response.data;
  }

  /**
   * Delete farm zone
   */
  async deleteZone(zoneId: string): Promise<void> {
    await this.axios.delete(`/zones/${zoneId}`);
  }

  /**
   * Record harvest data
   */
  async recordHarvest(harvest: Omit<HarvestData, 'harvestId'>): Promise<HarvestData> {
    const response = await this.axios.post('/harvests', harvest);
    return response.data;
  }

  /**
   * Get harvest history
   */
  async getHarvestHistory(
    farmId?: string,
    params?: ListParams
  ): Promise<HarvestData[]> {
    const id = farmId || this.config.farmId;
    if (!id) throw new Error('Farm ID is required');
    const response = await this.axios.get(`/farms/${id}/harvests`, { params });
    return response.data;
  }
}

/**
 * Device Manager for managing farm devices
 */
export class DeviceManager {
  private client: SmartFarmClient;

  constructor(config: SDKConfig) {
    this.client = new SmartFarmClient(config);
  }

  /**
   * List all online devices
   */
  async listOnlineDevices(farmId?: string): Promise<SmartFarmDevice[]> {
    const devices = await this.client.listDevices(farmId);
    return devices.filter((d) => d.status === 'online');
  }

  /**
   * List all offline devices
   */
  async listOfflineDevices(farmId?: string): Promise<SmartFarmDevice[]> {
    const devices = await this.client.listDevices(farmId);
    return devices.filter((d) => d.status === 'offline');
  }

  /**
   * Get devices by type
   */
  async getDevicesByType(type: string, farmId?: string): Promise<SmartFarmDevice[]> {
    const devices = await this.client.listDevices(farmId);
    return devices.filter((d) => d.type === type);
  }

  /**
   * Check device health
   */
  async checkDeviceHealth(deviceId: string): Promise<{
    healthy: boolean;
    lastSeen: string;
    status: string;
  }> {
    const device = await this.client.getDevice(deviceId);
    const lastSeenTime = device.lastSeen ? new Date(device.lastSeen).getTime() : 0;
    const now = Date.now();
    const minutesSinceLastSeen = (now - lastSeenTime) / 1000 / 60;

    return {
      healthy: device.status === 'online' && minutesSinceLastSeen < 10,
      lastSeen: device.lastSeen || 'never',
      status: device.status,
    };
  }
}

/**
 * Alert Manager for handling farm alerts
 */
export class AlertManager {
  private client: SmartFarmClient;

  constructor(config: SDKConfig) {
    this.client = new SmartFarmClient(config);
  }

  /**
   * Get critical alerts
   */
  async getCriticalAlerts(farmId?: string): Promise<FarmAlert[]> {
    const alerts = await this.client.listAlerts(farmId, { severity: 'critical' });
    return alerts.filter((a) => !a.resolved);
  }

  /**
   * Get unacknowledged alerts
   */
  async getUnacknowledgedAlerts(farmId?: string): Promise<FarmAlert[]> {
    const alerts = await this.client.listAlerts(farmId, { acknowledged: false });
    return alerts;
  }

  /**
   * Acknowledge all alerts
   */
  async acknowledgeAll(alertIds: string[], userId: string): Promise<void> {
    await Promise.all(
      alertIds.map((id) => this.client.acknowledgeAlert(id, userId))
    );
  }
}

/**
 * Automation Manager for managing automation rules
 */
export class AutomationManager {
  private client: SmartFarmClient;

  constructor(config: SDKConfig) {
    this.client = new SmartFarmClient(config);
  }

  /**
   * Get active rules
   */
  async getActiveRules(farmId?: string): Promise<AutomationRule[]> {
    const rules = await this.client.listRules(farmId);
    return rules.filter((r) => r.status === 'active');
  }

  /**
   * Pause a rule
   */
  async pauseRule(ruleId: string): Promise<AutomationRule> {
    return this.client.updateRule(ruleId, { status: 'paused' });
  }

  /**
   * Resume a rule
   */
  async resumeRule(ruleId: string): Promise<AutomationRule> {
    return this.client.updateRule(ruleId, { status: 'active' });
  }

  /**
   * Disable a rule
   */
  async disableRule(ruleId: string): Promise<AutomationRule> {
    return this.client.updateRule(ruleId, { status: 'inactive' });
  }
}

/**
 * Utility functions
 */
export const utils = {
  /**
   * Calculate average sensor reading
   */
  calculateAverage(readings: SensorReading[]): number {
    if (readings.length === 0) return 0;
    const sum = readings.reduce((acc, r) => acc + r.value, 0);
    return sum / readings.length;
  },

  /**
   * Filter readings by time range
   */
  filterByTimeRange(
    readings: SensorReading[],
    startDate: string,
    endDate: string
  ): SensorReading[] {
    const start = new Date(startDate).getTime();
    const end = new Date(endDate).getTime();
    return readings.filter((r) => {
      const time = new Date(r.timestamp).getTime();
      return time >= start && time <= end;
    });
  },

  /**
   * Group readings by sensor type
   */
  groupBySensorType(
    readings: SensorReading[]
  ): Record<string, SensorReading[]> {
    return readings.reduce(
      (acc, reading) => {
        if (!acc[reading.sensorType]) {
          acc[reading.sensorType] = [];
        }
        acc[reading.sensorType].push(reading);
        return acc;
      },
      {} as Record<string, SensorReading[]>
    );
  },

  /**
   * Format environmental data for display
   */
  formatEnvironmentalData(data: EnvironmentalData): string {
    const parts: string[] = [];
    if (data.temperature !== undefined)
      parts.push(`Temp: ${data.temperature}°C`);
    if (data.humidity !== undefined) parts.push(`Humidity: ${data.humidity}%`);
    if (data.soilMoisture !== undefined)
      parts.push(`Soil: ${data.soilMoisture}%`);
    return parts.join(' | ');
  },
};

/**
 * Default export
 */
export default {
  SmartFarmClient,
  DeviceManager,
  AlertManager,
  AutomationManager,
  utils,
};
