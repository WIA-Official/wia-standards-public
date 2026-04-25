/**
 * WIA-AGRI-008 Livestock Tracking Standard - TypeScript SDK
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';
import type {
  SDKConfig,
  LivestockTrackingConfig,
  LocationHistory,
  HealthRecord,
  WeightRecord,
  ActivityData,
  VitalSigns,
  ReproductionRecord,
  FeedingRecord,
  MilkProductionRecord,
  LivestockAlert,
  Herd,
  MovementRecord,
  SlaughterRecord,
  AnalyticsReport,
  Geofence,
  ListParams,
} from './types';

export * from './types';

/**
 * Main SDK Client for WIA-AGRI-008 Livestock Tracking
 */
export class LivestockTrackingClient {
  private axios: AxiosInstance;
  private config: SDKConfig;

  constructor(config: SDKConfig) {
    this.config = {
      baseURL: config.baseURL || 'https://api.wiastandards.com/v1/livestock-tracking',
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
   * Get animal details
   */
  async getAnimal(animalId: string): Promise<LivestockTrackingConfig> {
    const response = await this.axios.get(`/animals/${animalId}`);
    return response.data;
  }

  /**
   * Register new animal
   */
  async registerAnimal(
    animal: Omit<LivestockTrackingConfig, 'animalId' | 'status'>
  ): Promise<LivestockTrackingConfig> {
    const response = await this.axios.post('/animals', animal);
    return response.data;
  }

  /**
   * Update animal
   */
  async updateAnimal(
    animalId: string,
    updates: Partial<LivestockTrackingConfig>
  ): Promise<LivestockTrackingConfig> {
    const response = await this.axios.patch(`/animals/${animalId}`, updates);
    return response.data;
  }

  /**
   * List animals
   */
  async listAnimals(params?: ListParams): Promise<LivestockTrackingConfig[]> {
    const response = await this.axios.get('/animals', {
      params: { farmId: this.config.farmId, ...params },
    });
    return response.data;
  }

  /**
   * Delete animal
   */
  async deleteAnimal(animalId: string): Promise<void> {
    await this.axios.delete(`/animals/${animalId}`);
  }

  /**
   * Get current location
   */
  async getCurrentLocation(animalId: string): Promise<LocationHistory> {
    const response = await this.axios.get(`/animals/${animalId}/location/current`);
    return response.data;
  }

  /**
   * Get location history
   */
  async getLocationHistory(
    animalId: string,
    params?: { startDate?: string; endDate?: string }
  ): Promise<LocationHistory> {
    const response = await this.axios.get(`/animals/${animalId}/location/history`, {
      params,
    });
    return response.data;
  }

  /**
   * Get health records
   */
  async getHealthRecords(
    animalId: string,
    params?: ListParams
  ): Promise<HealthRecord[]> {
    const response = await this.axios.get(`/animals/${animalId}/health`, { params });
    return response.data;
  }

  /**
   * Add health record
   */
  async addHealthRecord(
    record: Omit<HealthRecord, 'recordId'>
  ): Promise<HealthRecord> {
    const response = await this.axios.post('/health-records', record);
    return response.data;
  }

  /**
   * Get weight records
   */
  async getWeightRecords(
    animalId: string,
    params?: ListParams
  ): Promise<WeightRecord[]> {
    const response = await this.axios.get(`/animals/${animalId}/weight`, { params });
    return response.data;
  }

  /**
   * Record weight
   */
  async recordWeight(record: Omit<WeightRecord, 'recordId'>): Promise<WeightRecord> {
    const response = await this.axios.post('/weight-records', record);
    return response.data;
  }

  /**
   * Get activity data
   */
  async getActivityData(
    animalId: string,
    params?: ListParams
  ): Promise<ActivityData[]> {
    const response = await this.axios.get(`/animals/${animalId}/activity`, {
      params,
    });
    return response.data;
  }

  /**
   * Get vital signs
   */
  async getVitalSigns(
    animalId: string,
    params?: ListParams
  ): Promise<VitalSigns[]> {
    const response = await this.axios.get(`/animals/${animalId}/vitals`, { params });
    return response.data;
  }

  /**
   * Get reproduction records
   */
  async getReproductionRecords(
    animalId: string,
    params?: ListParams
  ): Promise<ReproductionRecord[]> {
    const response = await this.axios.get(`/animals/${animalId}/reproduction`, {
      params,
    });
    return response.data;
  }

  /**
   * Add reproduction record
   */
  async addReproductionRecord(
    record: Omit<ReproductionRecord, 'recordId'>
  ): Promise<ReproductionRecord> {
    const response = await this.axios.post('/reproduction-records', record);
    return response.data;
  }

  /**
   * Get feeding records
   */
  async getFeedingRecords(
    animalId?: string,
    params?: ListParams
  ): Promise<FeedingRecord[]> {
    const response = await this.axios.get('/feeding-records', {
      params: { animalId, ...params },
    });
    return response.data;
  }

  /**
   * Record feeding
   */
  async recordFeeding(
    record: Omit<FeedingRecord, 'recordId'>
  ): Promise<FeedingRecord> {
    const response = await this.axios.post('/feeding-records', record);
    return response.data;
  }

  /**
   * Get milk production records
   */
  async getMilkProduction(
    animalId: string,
    params?: ListParams
  ): Promise<MilkProductionRecord[]> {
    const response = await this.axios.get(`/animals/${animalId}/milk-production`, {
      params,
    });
    return response.data;
  }

  /**
   * Record milk production
   */
  async recordMilkProduction(
    record: Omit<MilkProductionRecord, 'recordId'>
  ): Promise<MilkProductionRecord> {
    const response = await this.axios.post('/milk-production', record);
    return response.data;
  }

  /**
   * Get alerts
   */
  async getAlerts(animalId?: string, params?: ListParams): Promise<LivestockAlert[]> {
    const response = await this.axios.get('/alerts', {
      params: { animalId, ...params },
    });
    return response.data;
  }

  /**
   * Acknowledge alert
   */
  async acknowledgeAlert(alertId: string): Promise<LivestockAlert> {
    const response = await this.axios.post(`/alerts/${alertId}/acknowledge`);
    return response.data;
  }

  /**
   * Get herds
   */
  async getHerds(farmId?: string): Promise<Herd[]> {
    const response = await this.axios.get('/herds', {
      params: { farmId: farmId || this.config.farmId },
    });
    return response.data;
  }

  /**
   * Create herd
   */
  async createHerd(herd: Omit<Herd, 'herdId'>): Promise<Herd> {
    const response = await this.axios.post('/herds', herd);
    return response.data;
  }

  /**
   * Update herd
   */
  async updateHerd(herdId: string, updates: Partial<Herd>): Promise<Herd> {
    const response = await this.axios.patch(`/herds/${herdId}`, updates);
    return response.data;
  }

  /**
   * Record movement
   */
  async recordMovement(
    movement: Omit<MovementRecord, 'recordId'>
  ): Promise<MovementRecord> {
    const response = await this.axios.post('/movements', movement);
    return response.data;
  }

  /**
   * Get movement records
   */
  async getMovements(animalId?: string, params?: ListParams): Promise<MovementRecord[]> {
    const response = await this.axios.get('/movements', {
      params: { animalId, ...params },
    });
    return response.data;
  }

  /**
   * Record slaughter
   */
  async recordSlaughter(
    record: Omit<SlaughterRecord, 'recordId'>
  ): Promise<SlaughterRecord> {
    const response = await this.axios.post('/slaughter-records', record);
    return response.data;
  }

  /**
   * Get analytics
   */
  async getAnalytics(
    params?: { farmId?: string; herdId?: string; startDate?: string; endDate?: string }
  ): Promise<AnalyticsReport> {
    const response = await this.axios.get('/analytics', { params });
    return response.data;
  }

  /**
   * Get geofences
   */
  async getGeofences(farmId?: string): Promise<Geofence[]> {
    const response = await this.axios.get('/geofences', {
      params: { farmId: farmId || this.config.farmId },
    });
    return response.data;
  }

  /**
   * Create geofence
   */
  async createGeofence(fence: Omit<Geofence, 'fenceId'>): Promise<Geofence> {
    const response = await this.axios.post('/geofences', fence);
    return response.data;
  }
}

/**
 * Health Monitor for animal health tracking
 */
export class HealthMonitor {
  private client: LivestockTrackingClient;

  constructor(config: SDKConfig) {
    this.client = new LivestockTrackingClient(config);
  }

  /**
   * Get health status
   */
  async getHealthStatus(animalId: string): Promise<{
    status: string;
    recentIssues: HealthRecord[];
    needsAttention: boolean;
  }> {
    const [animal, records] = await Promise.all([
      this.client.getAnimal(animalId),
      this.client.getHealthRecords(animalId, { limit: 5 }),
    ]);

    const recentIssues = records.filter((r) => r.type === 'illness' || r.type === 'injury');

    return {
      status: animal.status,
      recentIssues,
      needsAttention: animal.status === 'sick' || recentIssues.length > 0,
    };
  }

  /**
   * Get animals needing attention
   */
  async getAnimalsNeedingAttention(farmId?: string): Promise<LivestockTrackingConfig[]> {
    const animals = await this.client.listAnimals({ status: 'sick' });
    return animals;
  }
}

/**
 * Herd Manager for group management
 */
export class HerdManager {
  private client: LivestockTrackingClient;

  constructor(config: SDKConfig) {
    this.client = new LivestockTrackingClient(config);
  }

  /**
   * Get herd summary
   */
  async getHerdSummary(herdId: string): Promise<{
    herd: Herd;
    animalCount: number;
    healthyCount: number;
    averageWeight: number;
  }> {
    const herd = (await this.client.getHerds()).find((h) => h.herdId === herdId);
    if (!herd) throw new Error('Herd not found');

    const animals = await Promise.all(
      herd.animalIds.map((id) => this.client.getAnimal(id))
    );

    const healthyCount = animals.filter((a) => a.status === 'healthy').length;

    // Get latest weights
    const weights = await Promise.all(
      animals.map(async (a) => {
        const records = await this.client.getWeightRecords(a.animalId, { limit: 1 });
        return records[0]?.weight || 0;
      })
    );

    const averageWeight = weights.reduce((a, b) => a + b, 0) / (weights.length || 1);

    return {
      herd,
      animalCount: animals.length,
      healthyCount,
      averageWeight,
    };
  }

  /**
   * Add animals to herd
   */
  async addAnimalsToHerd(herdId: string, animalIds: string[]): Promise<Herd> {
    const herds = await this.client.getHerds();
    const herd = herds.find((h) => h.herdId === herdId);
    if (!herd) throw new Error('Herd not found');

    const updatedAnimalIds = [...new Set([...herd.animalIds, ...animalIds])];
    return this.client.updateHerd(herdId, { animalIds: updatedAnimalIds });
  }
}

/**
 * Utility functions
 */
export const utils = {
  /**
   * Calculate age in months
   */
  calculateAge(birthDate: string): number {
    const birth = new Date(birthDate);
    const now = new Date();
    const months =
      (now.getFullYear() - birth.getFullYear()) * 12 +
      (now.getMonth() - birth.getMonth());
    return months;
  },

  /**
   * Calculate average daily gain
   */
  calculateADG(startWeight: number, endWeight: number, days: number): number {
    return (endWeight - startWeight) / days;
  },

  /**
   * Estimate animal value
   */
  estimateValue(weight: number, pricePerKg: number): number {
    return weight * pricePerKg;
  },

  /**
   * Check vaccination due
   */
  isVaccinationDue(lastVaccinationDate: string, intervalMonths: number): boolean {
    const lastDate = new Date(lastVaccinationDate);
    const dueDate = new Date(lastDate);
    dueDate.setMonth(dueDate.getMonth() + intervalMonths);
    return new Date() >= dueDate;
  },
};

/**
 * Default export
 */
export default {
  LivestockTrackingClient,
  HealthMonitor,
  HerdManager,
  utils,
};
