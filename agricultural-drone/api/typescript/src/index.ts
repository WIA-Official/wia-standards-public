/**
 * WIA-AGRI-003 Agricultural Drone Standard - TypeScript SDK
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';
import type {
  SDKConfig,
  AgriculturalDroneConfig,
  FlightMission,
  FlightTelemetry,
  ImageCapture,
  SprayOperation,
  MaintenanceRecord,
  FlightLog,
  OrthomosaicMap,
  AnomalyDetection,
  BatteryInfo,
  ListParams,
} from './types';

export * from './types';

/**
 * Main SDK Client for WIA-AGRI-003 Agricultural Drone
 */
export class AgriculturalDroneClient {
  private axios: AxiosInstance;
  private config: SDKConfig;

  constructor(config: SDKConfig) {
    this.config = {
      baseURL: config.baseURL || 'https://api.wiastandards.com/v1/agricultural-drone',
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
   * Get drone configuration
   */
  async getDrone(droneId?: string): Promise<AgriculturalDroneConfig> {
    const id = droneId || this.config.droneId;
    if (!id) throw new Error('Drone ID is required');
    const response = await this.axios.get(`/drones/${id}`);
    return response.data;
  }

  /**
   * Register a new drone
   */
  async registerDrone(
    drone: Omit<AgriculturalDroneConfig, 'droneId'>
  ): Promise<AgriculturalDroneConfig> {
    const response = await this.axios.post('/drones', drone);
    return response.data;
  }

  /**
   * Update drone configuration
   */
  async updateDrone(
    droneId: string,
    updates: Partial<AgriculturalDroneConfig>
  ): Promise<AgriculturalDroneConfig> {
    const response = await this.axios.patch(`/drones/${droneId}`, updates);
    return response.data;
  }

  /**
   * List all missions
   */
  async listMissions(droneId?: string, params?: ListParams): Promise<FlightMission[]> {
    const id = droneId || this.config.droneId;
    const response = await this.axios.get('/missions', {
      params: { droneId: id, ...params },
    });
    return response.data;
  }

  /**
   * Get mission details
   */
  async getMission(missionId: string): Promise<FlightMission> {
    const response = await this.axios.get(`/missions/${missionId}`);
    return response.data;
  }

  /**
   * Create flight mission
   */
  async createMission(
    mission: Omit<FlightMission, 'missionId' | 'status'>
  ): Promise<FlightMission> {
    const response = await this.axios.post('/missions', mission);
    return response.data;
  }

  /**
   * Update mission
   */
  async updateMission(
    missionId: string,
    updates: Partial<FlightMission>
  ): Promise<FlightMission> {
    const response = await this.axios.patch(`/missions/${missionId}`, updates);
    return response.data;
  }

  /**
   * Start mission
   */
  async startMission(missionId: string): Promise<FlightMission> {
    const response = await this.axios.post(`/missions/${missionId}/start`);
    return response.data;
  }

  /**
   * Complete mission
   */
  async completeMission(missionId: string): Promise<FlightMission> {
    const response = await this.axios.post(`/missions/${missionId}/complete`);
    return response.data;
  }

  /**
   * Cancel mission
   */
  async cancelMission(missionId: string): Promise<FlightMission> {
    const response = await this.axios.post(`/missions/${missionId}/cancel`);
    return response.data;
  }

  /**
   * Get live telemetry
   */
  async getTelemetry(droneId?: string): Promise<FlightTelemetry> {
    const id = droneId || this.config.droneId;
    if (!id) throw new Error('Drone ID is required');
    const response = await this.axios.get(`/drones/${id}/telemetry`);
    return response.data;
  }

  /**
   * Stream telemetry (returns telemetry history)
   */
  async getTelemetryHistory(
    missionId: string,
    params?: ListParams
  ): Promise<FlightTelemetry[]> {
    const response = await this.axios.get(`/missions/${missionId}/telemetry`, {
      params,
    });
    return response.data;
  }

  /**
   * Get captured images
   */
  async getImages(missionId: string, params?: ListParams): Promise<ImageCapture[]> {
    const response = await this.axios.get(`/missions/${missionId}/images`, { params });
    return response.data;
  }

  /**
   * Upload image
   */
  async uploadImage(image: Omit<ImageCapture, 'imageId'>): Promise<ImageCapture> {
    const response = await this.axios.post('/images', image);
    return response.data;
  }

  /**
   * Get spray operations
   */
  async getSprayOperations(
    droneId?: string,
    params?: ListParams
  ): Promise<SprayOperation[]> {
    const id = droneId || this.config.droneId;
    const response = await this.axios.get('/spray-operations', {
      params: { droneId: id, ...params },
    });
    return response.data;
  }

  /**
   * Record spray operation
   */
  async recordSprayOperation(
    operation: Omit<SprayOperation, 'operationId'>
  ): Promise<SprayOperation> {
    const response = await this.axios.post('/spray-operations', operation);
    return response.data;
  }

  /**
   * Get maintenance records
   */
  async getMaintenanceRecords(
    droneId?: string,
    params?: ListParams
  ): Promise<MaintenanceRecord[]> {
    const id = droneId || this.config.droneId;
    if (!id) throw new Error('Drone ID is required');
    const response = await this.axios.get(`/drones/${id}/maintenance`, { params });
    return response.data;
  }

  /**
   * Add maintenance record
   */
  async addMaintenanceRecord(
    record: Omit<MaintenanceRecord, 'recordId'>
  ): Promise<MaintenanceRecord> {
    const response = await this.axios.post('/maintenance', record);
    return response.data;
  }

  /**
   * Get flight logs
   */
  async getFlightLogs(droneId?: string, params?: ListParams): Promise<FlightLog[]> {
    const id = droneId || this.config.droneId;
    const response = await this.axios.get('/flight-logs', {
      params: { droneId: id, ...params },
    });
    return response.data;
  }

  /**
   * Get orthomosaic maps
   */
  async getOrthomosaics(
    missionId?: string,
    params?: ListParams
  ): Promise<OrthomosaicMap[]> {
    const response = await this.axios.get('/orthomosaics', {
      params: { missionId, ...params },
    });
    return response.data;
  }

  /**
   * Create orthomosaic map
   */
  async createOrthomosaic(
    map: Omit<OrthomosaicMap, 'mapId'>
  ): Promise<OrthomosaicMap> {
    const response = await this.axios.post('/orthomosaics', map);
    return response.data;
  }

  /**
   * Get anomaly detections
   */
  async getAnomalies(
    missionId: string,
    params?: ListParams
  ): Promise<AnomalyDetection[]> {
    const response = await this.axios.get(`/missions/${missionId}/anomalies`, {
      params,
    });
    return response.data;
  }

  /**
   * Get battery information
   */
  async getBatteryInfo(droneId?: string): Promise<BatteryInfo> {
    const id = droneId || this.config.droneId;
    if (!id) throw new Error('Drone ID is required');
    const response = await this.axios.get(`/drones/${id}/battery`);
    return response.data;
  }
}

/**
 * Mission Manager for flight operations
 */
export class MissionManager {
  private client: AgriculturalDroneClient;

  constructor(config: SDKConfig) {
    this.client = new AgriculturalDroneClient(config);
  }

  /**
   * Get active missions
   */
  async getActiveMissions(droneId?: string): Promise<FlightMission[]> {
    const missions = await this.client.listMissions(droneId);
    return missions.filter((m) => m.status === 'in_progress');
  }

  /**
   * Get completed missions
   */
  async getCompletedMissions(
    droneId?: string,
    params?: ListParams
  ): Promise<FlightMission[]> {
    const missions = await this.client.listMissions(droneId, {
      ...params,
      status: 'completed',
    });
    return missions;
  }

  /**
   * Validate mission plan
   */
  async validateMission(mission: FlightMission): Promise<{
    valid: boolean;
    warnings: string[];
    errors: string[];
  }> {
    const warnings: string[] = [];
    const errors: string[] = [];

    // Check flight plan
    if (!mission.flightPlan.waypoints || mission.flightPlan.waypoints.length < 2) {
      errors.push('Flight plan must have at least 2 waypoints');
    }

    // Check altitude
    if (mission.flightPlan.altitude > 120) {
      warnings.push('Altitude exceeds recommended maximum of 120m');
    }

    // Check weather conditions
    if (mission.weather?.windSpeed && mission.weather.windSpeed > 10) {
      warnings.push('High wind speed detected');
    }

    return {
      valid: errors.length === 0,
      warnings,
      errors,
    };
  }
}

/**
 * Fleet Manager for managing multiple drones
 */
export class FleetManager {
  private client: AgriculturalDroneClient;

  constructor(config: SDKConfig) {
    this.client = new AgriculturalDroneClient(config);
  }

  /**
   * Get fleet status
   */
  async getFleetStatus(droneIds: string[]): Promise<
    Record<
      string,
      {
        drone: AgriculturalDroneConfig;
        telemetry?: FlightTelemetry;
        battery?: BatteryInfo;
      }
    >
  > {
    const status: Record<string, any> = {};

    for (const droneId of droneIds) {
      const [drone, battery] = await Promise.all([
        this.client.getDrone(droneId),
        this.client.getBatteryInfo(droneId).catch(() => undefined),
      ]);

      let telemetry;
      try {
        telemetry = await this.client.getTelemetry(droneId);
      } catch {
        // Drone may be offline
      }

      status[droneId] = { drone, telemetry, battery };
    }

    return status;
  }
}

/**
 * Utility functions
 */
export const utils = {
  /**
   * Calculate mission duration estimate
   */
  calculateMissionDuration(mission: FlightMission): number {
    const waypoints = mission.flightPlan.waypoints;
    let totalDistance = 0;

    for (let i = 0; i < waypoints.length - 1; i++) {
      const dist = this.calculateDistance(
        waypoints[i].location,
        waypoints[i + 1].location
      );
      totalDistance += dist;
    }

    const speed = mission.flightPlan.speed || 10; // m/s
    return (totalDistance / speed / 60) * 1.2; // Add 20% buffer, return in minutes
  },

  /**
   * Calculate distance between two locations
   */
  calculateDistance(
    loc1: { latitude: number; longitude: number },
    loc2: { latitude: number; longitude: number }
  ): number {
    const R = 6371000; // Earth radius in meters
    const lat1 = (loc1.latitude * Math.PI) / 180;
    const lat2 = (loc2.latitude * Math.PI) / 180;
    const dLat = ((loc2.latitude - loc1.latitude) * Math.PI) / 180;
    const dLon = ((loc2.longitude - loc1.longitude) * Math.PI) / 180;

    const a =
      Math.sin(dLat / 2) * Math.sin(dLat / 2) +
      Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return R * c;
  },

  /**
   * Check if battery is sufficient for mission
   */
  checkBatterySufficiency(
    battery: BatteryInfo,
    missionDuration: number,
    droneFlightTime: number
  ): boolean {
    const availableTime = (battery.currentCharge / 100) * droneFlightTime;
    return availableTime >= missionDuration * 1.3; // 30% safety margin
  },
};

/**
 * Default export
 */
export default {
  AgriculturalDroneClient,
  MissionManager,
  FleetManager,
  utils,
};
