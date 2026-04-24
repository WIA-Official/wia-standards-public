/**
 * WIA-AGRI-002 Precision Agriculture Standard - TypeScript SDK
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';
import type {
  SDKConfig,
  PrecisionAgricultureConfig,
  FieldData,
  SoilSample,
  VRAMap,
  YieldMap,
  NDVIData,
  EquipmentData,
  OperationRecord,
  ManagementZone,
  WeatherData,
  PrescriptionRecommendation,
  AnalyticsReport,
  ListParams,
} from './types';

export * from './types';

/**
 * Main SDK Client for WIA-AGRI-002 Precision Agriculture
 */
export class PrecisionAgricultureClient {
  private axios: AxiosInstance;
  private config: SDKConfig;

  constructor(config: SDKConfig) {
    this.config = {
      baseURL: config.baseURL || 'https://api.wiastandards.com/v1/precision-agriculture',
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
  async getFarm(farmId?: string): Promise<PrecisionAgricultureConfig> {
    const id = farmId || this.config.farmId;
    if (!id) throw new Error('Farm ID is required');
    const response = await this.axios.get(`/farms/${id}`);
    return response.data;
  }

  /**
   * Create a new farm
   */
  async createFarm(
    farm: Omit<PrecisionAgricultureConfig, 'farmId'>
  ): Promise<PrecisionAgricultureConfig> {
    const response = await this.axios.post('/farms', farm);
    return response.data;
  }

  /**
   * Update farm configuration
   */
  async updateFarm(
    farmId: string,
    updates: Partial<PrecisionAgricultureConfig>
  ): Promise<PrecisionAgricultureConfig> {
    const response = await this.axios.patch(`/farms/${farmId}`, updates);
    return response.data;
  }

  /**
   * List all fields
   */
  async listFields(farmId?: string, params?: ListParams): Promise<FieldData[]> {
    const id = farmId || this.config.farmId;
    if (!id) throw new Error('Farm ID is required');
    const response = await this.axios.get(`/farms/${id}/fields`, { params });
    return response.data;
  }

  /**
   * Get field details
   */
  async getField(fieldId: string): Promise<FieldData> {
    const response = await this.axios.get(`/fields/${fieldId}`);
    return response.data;
  }

  /**
   * Create a new field
   */
  async createField(field: Omit<FieldData, 'fieldId'>): Promise<FieldData> {
    const response = await this.axios.post('/fields', field);
    return response.data;
  }

  /**
   * Update field
   */
  async updateField(fieldId: string, updates: Partial<FieldData>): Promise<FieldData> {
    const response = await this.axios.patch(`/fields/${fieldId}`, updates);
    return response.data;
  }

  /**
   * Delete field
   */
  async deleteField(fieldId: string): Promise<void> {
    await this.axios.delete(`/fields/${fieldId}`);
  }

  /**
   * Get soil samples for a field
   */
  async getSoilSamples(fieldId: string, params?: ListParams): Promise<SoilSample[]> {
    const response = await this.axios.get(`/fields/${fieldId}/soil-samples`, { params });
    return response.data;
  }

  /**
   * Create soil sample
   */
  async createSoilSample(sample: Omit<SoilSample, 'sampleId'>): Promise<SoilSample> {
    const response = await this.axios.post('/soil-samples', sample);
    return response.data;
  }

  /**
   * Get VRA maps for a field
   */
  async getVRAMaps(fieldId: string, params?: ListParams): Promise<VRAMap[]> {
    const response = await this.axios.get(`/fields/${fieldId}/vra-maps`, { params });
    return response.data;
  }

  /**
   * Create VRA map
   */
  async createVRAMap(map: Omit<VRAMap, 'mapId'>): Promise<VRAMap> {
    const response = await this.axios.post('/vra-maps', map);
    return response.data;
  }

  /**
   * Update VRA map
   */
  async updateVRAMap(mapId: string, updates: Partial<VRAMap>): Promise<VRAMap> {
    const response = await this.axios.patch(`/vra-maps/${mapId}`, updates);
    return response.data;
  }

  /**
   * Get yield maps for a field
   */
  async getYieldMaps(fieldId: string, params?: ListParams): Promise<YieldMap[]> {
    const response = await this.axios.get(`/fields/${fieldId}/yield-maps`, { params });
    return response.data;
  }

  /**
   * Upload yield data
   */
  async uploadYieldData(yieldMap: Omit<YieldMap, 'mapId'>): Promise<YieldMap> {
    const response = await this.axios.post('/yield-maps', yieldMap);
    return response.data;
  }

  /**
   * Get NDVI data for a field
   */
  async getNDVIData(fieldId: string, params?: ListParams): Promise<NDVIData[]> {
    const response = await this.axios.get(`/fields/${fieldId}/ndvi`, { params });
    return response.data;
  }

  /**
   * Upload NDVI data
   */
  async uploadNDVIData(data: Omit<NDVIData, 'dataId'>): Promise<NDVIData> {
    const response = await this.axios.post('/ndvi', data);
    return response.data;
  }

  /**
   * List equipment
   */
  async listEquipment(farmId?: string): Promise<EquipmentData[]> {
    const id = farmId || this.config.farmId;
    if (!id) throw new Error('Farm ID is required');
    const response = await this.axios.get(`/farms/${id}/equipment`);
    return response.data;
  }

  /**
   * Register equipment
   */
  async registerEquipment(
    equipment: Omit<EquipmentData, 'equipmentId'>
  ): Promise<EquipmentData> {
    const response = await this.axios.post('/equipment', equipment);
    return response.data;
  }

  /**
   * Get operation records
   */
  async getOperationRecords(
    fieldId: string,
    params?: ListParams
  ): Promise<OperationRecord[]> {
    const response = await this.axios.get(`/fields/${fieldId}/operations`, { params });
    return response.data;
  }

  /**
   * Record operation
   */
  async recordOperation(
    operation: Omit<OperationRecord, 'operationId'>
  ): Promise<OperationRecord> {
    const response = await this.axios.post('/operations', operation);
    return response.data;
  }

  /**
   * Get management zones
   */
  async getManagementZones(fieldId: string): Promise<ManagementZone[]> {
    const response = await this.axios.get(`/fields/${fieldId}/management-zones`);
    return response.data;
  }

  /**
   * Create management zone
   */
  async createManagementZone(
    zone: Omit<ManagementZone, 'zoneId'>
  ): Promise<ManagementZone> {
    const response = await this.axios.post('/management-zones', zone);
    return response.data;
  }

  /**
   * Get weather data
   */
  async getWeatherData(
    location: { latitude: number; longitude: number },
    params?: { startDate?: string; endDate?: string }
  ): Promise<WeatherData[]> {
    const response = await this.axios.get('/weather', {
      params: { ...location, ...params },
    });
    return response.data;
  }

  /**
   * Get prescription recommendations
   */
  async getPrescriptionRecommendations(
    fieldId: string
  ): Promise<PrescriptionRecommendation[]> {
    const response = await this.axios.get(`/fields/${fieldId}/recommendations`);
    return response.data;
  }

  /**
   * Generate prescription recommendation
   */
  async generateRecommendation(
    fieldId: string,
    params: { applicationType: string; basedOn: string[] }
  ): Promise<PrescriptionRecommendation> {
    const response = await this.axios.post(`/fields/${fieldId}/recommendations`, params);
    return response.data;
  }

  /**
   * Get analytics report
   */
  async getAnalyticsReport(
    farmId?: string,
    params?: { reportType?: string; period?: string }
  ): Promise<AnalyticsReport> {
    const id = farmId || this.config.farmId;
    if (!id) throw new Error('Farm ID is required');
    const response = await this.axios.get(`/farms/${id}/analytics`, { params });
    return response.data;
  }
}

/**
 * Field Manager for managing field operations
 */
export class FieldManager {
  private client: PrecisionAgricultureClient;

  constructor(config: SDKConfig) {
    this.client = new PrecisionAgricultureClient(config);
  }

  /**
   * Get field summary with latest data
   */
  async getFieldSummary(fieldId: string): Promise<{
    field: FieldData;
    latestNDVI?: NDVIData;
    latestYield?: YieldMap;
    soilSampleCount: number;
  }> {
    const [field, ndviData, yieldMaps, soilSamples] = await Promise.all([
      this.client.getField(fieldId),
      this.client.getNDVIData(fieldId, { limit: 1 }),
      this.client.getYieldMaps(fieldId, { limit: 1 }),
      this.client.getSoilSamples(fieldId),
    ]);

    return {
      field,
      latestNDVI: ndviData[0],
      latestYield: yieldMaps[0],
      soilSampleCount: soilSamples.length,
    };
  }

  /**
   * Get fields by crop type
   */
  async getFieldsByCropType(cropType: string, farmId?: string): Promise<FieldData[]> {
    const fields = await this.client.listFields(farmId);
    return fields.filter((f) => f.currentCrop === cropType);
  }
}

/**
 * VRA Manager for Variable Rate Application
 */
export class VRAManager {
  private client: PrecisionAgricultureClient;

  constructor(config: SDKConfig) {
    this.client = new PrecisionAgricultureClient(config);
  }

  /**
   * Create VRA map from soil samples
   */
  async createFromSoilSamples(
    fieldId: string,
    applicationType: string,
    product: string
  ): Promise<VRAMap> {
    const soilSamples = await this.client.getSoilSamples(fieldId);
    // In production, this would include logic to interpolate zones
    // For now, we'll just create a basic structure
    const zones = soilSamples.map((sample, idx) => ({
      zoneId: `zone-${idx}`,
      geometry: {
        type: 'Polygon' as const,
        coordinates: [
          [
            [sample.location.longitude, sample.location.latitude],
            [sample.location.longitude + 0.001, sample.location.latitude],
            [sample.location.longitude + 0.001, sample.location.latitude + 0.001],
            [sample.location.longitude, sample.location.latitude + 0.001],
            [sample.location.longitude, sample.location.latitude],
          ],
        ],
      },
      rate: sample.analysis.nitrogen || 100,
      area: 1,
    }));

    return this.client.createVRAMap({
      fieldId,
      name: `${applicationType} - ${product}`,
      applicationType: applicationType as any,
      product,
      createdDate: new Date().toISOString(),
      status: 'draft',
      zones,
      unit: 'kg/ha',
    });
  }

  /**
   * Approve VRA map
   */
  async approveMap(mapId: string): Promise<VRAMap> {
    return this.client.updateVRAMap(mapId, { status: 'approved' });
  }

  /**
   * Mark VRA map as applied
   */
  async markAsApplied(mapId: string): Promise<VRAMap> {
    return this.client.updateVRAMap(mapId, { status: 'applied' });
  }
}

/**
 * Analytics Manager for data analysis
 */
export class AnalyticsManager {
  private client: PrecisionAgricultureClient;

  constructor(config: SDKConfig) {
    this.client = new PrecisionAgricultureClient(config);
  }

  /**
   * Calculate field productivity index
   */
  async calculateProductivityIndex(fieldId: string): Promise<number> {
    const yieldMaps = await this.client.getYieldMaps(fieldId);
    if (yieldMaps.length === 0) return 0;

    const avgYields = yieldMaps.map((ym) => ym.statistics.averageYield);
    const overallAvg = avgYields.reduce((a, b) => a + b, 0) / avgYields.length;
    return overallAvg;
  }

  /**
   * Compare field performance
   */
  async compareFields(fieldIds: string[]): Promise<
    Record<
      string,
      {
        avgYield: number;
        avgNDVI: number;
        operationCount: number;
      }
    >
  > {
    const results: Record<string, any> = {};

    for (const fieldId of fieldIds) {
      const [yieldMaps, ndviData, operations] = await Promise.all([
        this.client.getYieldMaps(fieldId),
        this.client.getNDVIData(fieldId),
        this.client.getOperationRecords(fieldId),
      ]);

      results[fieldId] = {
        avgYield:
          yieldMaps.reduce((sum, ym) => sum + ym.statistics.averageYield, 0) /
            (yieldMaps.length || 1),
        avgNDVI:
          ndviData.reduce((sum, nd) => sum + nd.statistics.mean, 0) /
            (ndviData.length || 1),
        operationCount: operations.length,
      };
    }

    return results;
  }
}

/**
 * Utility functions
 */
export const utils = {
  /**
   * Calculate area of polygon in hectares
   */
  calculatePolygonArea(coordinates: number[][][]): number {
    // Simplified area calculation (real implementation would use proper geodesic calculation)
    const ring = coordinates[0];
    let area = 0;
    for (let i = 0; i < ring.length - 1; i++) {
      area += ring[i][0] * ring[i + 1][1] - ring[i + 1][0] * ring[i][1];
    }
    return Math.abs(area / 2) * 111000 * 111000 / 10000; // Rough conversion to hectares
  },

  /**
   * Interpolate NDVI value at location
   */
  interpolateNDVI(
    ndviData: NDVIData,
    location: { latitude: number; longitude: number }
  ): number {
    // Simple nearest neighbor interpolation
    let minDist = Infinity;
    let nearestValue = 0;

    for (const point of ndviData.values) {
      const dist = Math.sqrt(
        Math.pow(point.location.latitude - location.latitude, 2) +
          Math.pow(point.location.longitude - location.longitude, 2)
      );
      if (dist < minDist) {
        minDist = dist;
        nearestValue = point.ndvi;
      }
    }

    return nearestValue;
  },

  /**
   * Classify soil fertility
   */
  classifySoilFertility(analysis: any): 'high' | 'medium' | 'low' {
    const nScore = (analysis.nitrogen || 0) > 50 ? 1 : 0;
    const pScore = (analysis.phosphorus || 0) > 30 ? 1 : 0;
    const kScore = (analysis.potassium || 0) > 150 ? 1 : 0;
    const total = nScore + pScore + kScore;

    if (total >= 2) return 'high';
    if (total === 1) return 'medium';
    return 'low';
  },
};

/**
 * Default export
 */
export default {
  PrecisionAgricultureClient,
  FieldManager,
  VRAManager,
  AnalyticsManager,
  utils,
};
