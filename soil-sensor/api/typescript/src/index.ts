/**
 * WIA-AGRI-004 Soil Sensor Standard - TypeScript SDK
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';
import type {
  SDKConfig,
  SoilSensorConfig,
  SoilMeasurement,
  AggregatedSoilData,
  SoilProfile,
  AlertConfig,
  SensorAlert,
  SensorNetwork,
  IrrigationRecommendation,
  SensorMaintenance,
  DataExportRequest,
  DataExportResponse,
  ListParams,
} from './types';

export * from './types';

/**
 * Main SDK Client for WIA-AGRI-004 Soil Sensor
 */
export class SoilSensorClient {
  private axios: AxiosInstance;
  private config: SDKConfig;

  constructor(config: SDKConfig) {
    this.config = {
      baseURL: config.baseURL || 'https://api.wiastandards.com/v1/soil-sensor',
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
   * Get sensor configuration
   */
  async getSensor(sensorId?: string): Promise<SoilSensorConfig> {
    const id = sensorId || this.config.sensorId;
    if (!id) throw new Error('Sensor ID is required');
    const response = await this.axios.get(`/sensors/${id}`);
    return response.data;
  }

  /**
   * Register a new sensor
   */
  async registerSensor(
    sensor: Omit<SoilSensorConfig, 'sensorId' | 'status'>
  ): Promise<SoilSensorConfig> {
    const response = await this.axios.post('/sensors', sensor);
    return response.data;
  }

  /**
   * Update sensor configuration
   */
  async updateSensor(
    sensorId: string,
    updates: Partial<SoilSensorConfig>
  ): Promise<SoilSensorConfig> {
    const response = await this.axios.patch(`/sensors/${sensorId}`, updates);
    return response.data;
  }

  /**
   * List all sensors
   */
  async listSensors(params?: ListParams): Promise<SoilSensorConfig[]> {
    const response = await this.axios.get('/sensors', { params });
    return response.data;
  }

  /**
   * Delete sensor
   */
  async deleteSensor(sensorId: string): Promise<void> {
    await this.axios.delete(`/sensors/${sensorId}`);
  }

  /**
   * Get measurements
   */
  async getMeasurements(
    sensorId?: string,
    params?: ListParams
  ): Promise<SoilMeasurement[]> {
    const id = sensorId || this.config.sensorId;
    if (!id) throw new Error('Sensor ID is required');
    const response = await this.axios.get(`/sensors/${id}/measurements`, { params });
    return response.data;
  }

  /**
   * Get latest measurement
   */
  async getLatestMeasurement(sensorId?: string): Promise<SoilMeasurement> {
    const id = sensorId || this.config.sensorId;
    if (!id) throw new Error('Sensor ID is required');
    const response = await this.axios.get(`/sensors/${id}/measurements/latest`);
    return response.data;
  }

  /**
   * Submit measurement
   */
  async submitMeasurement(
    measurement: Omit<SoilMeasurement, 'measurementId'>
  ): Promise<SoilMeasurement> {
    const response = await this.axios.post('/measurements', measurement);
    return response.data;
  }

  /**
   * Get aggregated data
   */
  async getAggregatedData(
    sensorId: string,
    params: { startDate: string; endDate: string; period?: string }
  ): Promise<AggregatedSoilData> {
    const response = await this.axios.get(`/sensors/${sensorId}/aggregated`, {
      params,
    });
    return response.data;
  }

  /**
   * Get soil profile
   */
  async getSoilProfile(location: {
    latitude: number;
    longitude: number;
  }): Promise<SoilProfile[]> {
    const response = await this.axios.get('/soil-profiles', { params: location });
    return response.data;
  }

  /**
   * Create soil profile
   */
  async createSoilProfile(
    profile: Omit<SoilProfile, 'profileId'>
  ): Promise<SoilProfile> {
    const response = await this.axios.post('/soil-profiles', profile);
    return response.data;
  }

  /**
   * Get alert configurations
   */
  async getAlertConfigs(sensorId?: string): Promise<AlertConfig[]> {
    const id = sensorId || this.config.sensorId;
    const response = await this.axios.get('/alert-configs', {
      params: { sensorId: id },
    });
    return response.data;
  }

  /**
   * Create alert configuration
   */
  async createAlertConfig(
    config: Omit<AlertConfig, 'alertId' | 'createdAt'>
  ): Promise<AlertConfig> {
    const response = await this.axios.post('/alert-configs', config);
    return response.data;
  }

  /**
   * Update alert configuration
   */
  async updateAlertConfig(
    alertId: string,
    updates: Partial<AlertConfig>
  ): Promise<AlertConfig> {
    const response = await this.axios.patch(`/alert-configs/${alertId}`, updates);
    return response.data;
  }

  /**
   * Delete alert configuration
   */
  async deleteAlertConfig(alertId: string): Promise<void> {
    await this.axios.delete(`/alert-configs/${alertId}`);
  }

  /**
   * Get active alerts
   */
  async getAlerts(sensorId?: string, params?: ListParams): Promise<SensorAlert[]> {
    const response = await this.axios.get('/alerts', {
      params: { sensorId: sensorId || this.config.sensorId, ...params },
    });
    return response.data;
  }

  /**
   * Acknowledge alert
   */
  async acknowledgeAlert(alertId: string, userId: string): Promise<SensorAlert> {
    const response = await this.axios.post(`/alerts/${alertId}/acknowledge`, {
      userId,
    });
    return response.data;
  }

  /**
   * Get sensor network
   */
  async getSensorNetwork(networkId: string): Promise<SensorNetwork> {
    const response = await this.axios.get(`/networks/${networkId}`);
    return response.data;
  }

  /**
   * Create sensor network
   */
  async createSensorNetwork(
    network: Omit<SensorNetwork, 'networkId' | 'createdAt'>
  ): Promise<SensorNetwork> {
    const response = await this.axios.post('/networks', network);
    return response.data;
  }

  /**
   * Get irrigation recommendations
   */
  async getIrrigationRecommendations(
    sensorId?: string
  ): Promise<IrrigationRecommendation[]> {
    const id = sensorId || this.config.sensorId;
    const response = await this.axios.get('/irrigation-recommendations', {
      params: { sensorId: id },
    });
    return response.data;
  }

  /**
   * Get maintenance records
   */
  async getMaintenanceRecords(
    sensorId?: string,
    params?: ListParams
  ): Promise<SensorMaintenance[]> {
    const id = sensorId || this.config.sensorId;
    if (!id) throw new Error('Sensor ID is required');
    const response = await this.axios.get(`/sensors/${id}/maintenance`, { params });
    return response.data;
  }

  /**
   * Add maintenance record
   */
  async addMaintenanceRecord(
    record: Omit<SensorMaintenance, 'maintenanceId'>
  ): Promise<SensorMaintenance> {
    const response = await this.axios.post('/maintenance', record);
    return response.data;
  }

  /**
   * Request data export
   */
  async requestDataExport(request: DataExportRequest): Promise<DataExportResponse> {
    const response = await this.axios.post('/data-export', request);
    return response.data;
  }

  /**
   * Get export status
   */
  async getExportStatus(exportId: string): Promise<DataExportResponse> {
    const response = await this.axios.get(`/data-export/${exportId}`);
    return response.data;
  }
}

/**
 * Measurement Manager for handling sensor data
 */
export class MeasurementManager {
  private client: SoilSensorClient;

  constructor(config: SDKConfig) {
    this.client = new SoilSensorClient(config);
  }

  /**
   * Get average moisture for a field
   */
  async getFieldAverageMoisture(fieldId: string): Promise<number> {
    const sensors = await this.client.listSensors({ fieldId });
    let totalMoisture = 0;
    let count = 0;

    for (const sensor of sensors) {
      if (sensor.type === 'moisture') {
        try {
          const latest = await this.client.getLatestMeasurement(sensor.sensorId);
          totalMoisture += latest.value;
          count++;
        } catch {
          // Skip sensors with no data
        }
      }
    }

    return count > 0 ? totalMoisture / count : 0;
  }

  /**
   * Check if irrigation is needed
   */
  async checkIrrigationNeeded(
    sensorId: string,
    threshold: number
  ): Promise<{ needed: boolean; currentValue: number; deficit: number }> {
    const latest = await this.client.getLatestMeasurement(sensorId);
    const needed = latest.value < threshold;
    return {
      needed,
      currentValue: latest.value,
      deficit: needed ? threshold - latest.value : 0,
    };
  }
}

/**
 * Alert Manager for monitoring conditions
 */
export class AlertManager {
  private client: SoilSensorClient;

  constructor(config: SDKConfig) {
    this.client = new SoilSensorClient(config);
  }

  /**
   * Create moisture alert
   */
  async createMoistureAlert(
    sensorId: string,
    minThreshold: number,
    maxThreshold: number
  ): Promise<AlertConfig> {
    return this.client.createAlertConfig({
      sensorId,
      name: 'Moisture Alert',
      enabled: true,
      condition: {
        metric: 'moisture',
        operator: 'outside',
        threshold: [minThreshold, maxThreshold],
      },
      notification: {
        channels: ['email', 'push'],
      },
    });
  }

  /**
   * Get critical alerts
   */
  async getCriticalAlerts(sensorId?: string): Promise<SensorAlert[]> {
    const alerts = await this.client.getAlerts(sensorId);
    return alerts.filter((a) => !a.acknowledged);
  }
}

/**
 * Utility functions
 */
export const utils = {
  /**
   * Classify soil moisture level
   */
  classifyMoisture(value: number): 'dry' | 'optimal' | 'wet' | 'saturated' {
    if (value < 20) return 'dry';
    if (value < 40) return 'optimal';
    if (value < 60) return 'wet';
    return 'saturated';
  },

  /**
   * Classify pH level
   */
  classifyPH(value: number): 'acidic' | 'neutral' | 'alkaline' {
    if (value < 6.5) return 'acidic';
    if (value <= 7.5) return 'neutral';
    return 'alkaline';
  },

  /**
   * Calculate VPD (Vapor Pressure Deficit)
   */
  calculateVPD(temperature: number, humidity: number): number {
    const svp = 610.7 * Math.pow(10, (7.5 * temperature) / (237.3 + temperature));
    const vpd = ((100 - humidity) / 100) * svp;
    return vpd / 1000; // Convert to kPa
  },
};

/**
 * Default export
 */
export default {
  SoilSensorClient,
  MeasurementManager,
  AlertManager,
  utils,
};
