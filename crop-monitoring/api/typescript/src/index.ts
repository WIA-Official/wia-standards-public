/**
 * WIA-AGRI-005 Crop Monitoring Standard - TypeScript SDK
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';
import type {
  SDKConfig,
  CropMonitoringConfig,
  GrowthObservation,
  HealthAssessment,
  PestDetection,
  DiseaseDetection,
  WeedDetection,
  PhenologyStage,
  ScoutingReport,
  ImageryData,
  YieldEstimate,
  TreatmentRecord,
  WeatherImpact,
  AnalyticsReport,
  ListParams,
} from './types';

export * from './types';

/**
 * Main SDK Client for WIA-AGRI-005 Crop Monitoring
 */
export class CropMonitoringClient {
  private axios: AxiosInstance;
  private config: SDKConfig;

  constructor(config: SDKConfig) {
    this.config = {
      baseURL: config.baseURL || 'https://api.wiastandards.com/v1/crop-monitoring',
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
   * Get crop configuration
   */
  async getCrop(cropId?: string): Promise<CropMonitoringConfig> {
    const id = cropId || this.config.cropId;
    if (!id) throw new Error('Crop ID is required');
    const response = await this.axios.get(`/crops/${id}`);
    return response.data;
  }

  /**
   * Create crop
   */
  async createCrop(
    crop: Omit<CropMonitoringConfig, 'cropId'>
  ): Promise<CropMonitoringConfig> {
    const response = await this.axios.post('/crops', crop);
    return response.data;
  }

  /**
   * Update crop
   */
  async updateCrop(
    cropId: string,
    updates: Partial<CropMonitoringConfig>
  ): Promise<CropMonitoringConfig> {
    const response = await this.axios.patch(`/crops/${cropId}`, updates);
    return response.data;
  }

  /**
   * List crops
   */
  async listCrops(params?: ListParams): Promise<CropMonitoringConfig[]> {
    const response = await this.axios.get('/crops', { params });
    return response.data;
  }

  /**
   * Add growth observation
   */
  async addObservation(
    observation: Omit<GrowthObservation, 'observationId'>
  ): Promise<GrowthObservation> {
    const response = await this.axios.post('/observations', observation);
    return response.data;
  }

  /**
   * Get observations
   */
  async getObservations(
    cropId?: string,
    params?: ListParams
  ): Promise<GrowthObservation[]> {
    const id = cropId || this.config.cropId;
    const response = await this.axios.get('/observations', {
      params: { cropId: id, ...params },
    });
    return response.data;
  }

  /**
   * Get health assessment
   */
  async getHealthAssessment(cropId?: string): Promise<HealthAssessment> {
    const id = cropId || this.config.cropId;
    if (!id) throw new Error('Crop ID is required');
    const response = await this.axios.get(`/crops/${id}/health`);
    return response.data;
  }

  /**
   * Get pest detections
   */
  async getPestDetections(
    cropId?: string,
    params?: ListParams
  ): Promise<PestDetection[]> {
    const id = cropId || this.config.cropId;
    const response = await this.axios.get('/pests', {
      params: { cropId: id, ...params },
    });
    return response.data;
  }

  /**
   * Report pest detection
   */
  async reportPest(
    pest: Omit<PestDetection, 'detectionId'>
  ): Promise<PestDetection> {
    const response = await this.axios.post('/pests', pest);
    return response.data;
  }

  /**
   * Get disease detections
   */
  async getDiseaseDetections(
    cropId?: string,
    params?: ListParams
  ): Promise<DiseaseDetection[]> {
    const id = cropId || this.config.cropId;
    const response = await this.axios.get('/diseases', {
      params: { cropId: id, ...params },
    });
    return response.data;
  }

  /**
   * Report disease detection
   */
  async reportDisease(
    disease: Omit<DiseaseDetection, 'detectionId'>
  ): Promise<DiseaseDetection> {
    const response = await this.axios.post('/diseases', disease);
    return response.data;
  }

  /**
   * Get weed detections
   */
  async getWeedDetections(
    cropId?: string,
    params?: ListParams
  ): Promise<WeedDetection[]> {
    const id = cropId || this.config.cropId;
    const response = await this.axios.get('/weeds', {
      params: { cropId: id, ...params },
    });
    return response.data;
  }

  /**
   * Report weed detection
   */
  async reportWeed(weed: Omit<WeedDetection, 'detectionId'>): Promise<WeedDetection> {
    const response = await this.axios.post('/weeds', weed);
    return response.data;
  }

  /**
   * Get phenology stages
   */
  async getPhenologyStages(cropId?: string): Promise<PhenologyStage[]> {
    const id = cropId || this.config.cropId;
    if (!id) throw new Error('Crop ID is required');
    const response = await this.axios.get(`/crops/${id}/phenology`);
    return response.data;
  }

  /**
   * Update phenology stage
   */
  async updatePhenologyStage(
    stage: Omit<PhenologyStage, 'stageId'>
  ): Promise<PhenologyStage> {
    const response = await this.axios.post('/phenology', stage);
    return response.data;
  }

  /**
   * Create scouting report
   */
  async createScoutingReport(
    report: Omit<ScoutingReport, 'reportId'>
  ): Promise<ScoutingReport> {
    const response = await this.axios.post('/scouting-reports', report);
    return response.data;
  }

  /**
   * Get scouting reports
   */
  async getScoutingReports(
    cropId?: string,
    params?: ListParams
  ): Promise<ScoutingReport[]> {
    const id = cropId || this.config.cropId;
    const response = await this.axios.get('/scouting-reports', {
      params: { cropId: id, ...params },
    });
    return response.data;
  }

  /**
   * Upload imagery
   */
  async uploadImagery(
    imagery: Omit<ImageryData, 'imageId'>
  ): Promise<ImageryData> {
    const response = await this.axios.post('/imagery', imagery);
    return response.data;
  }

  /**
   * Get imagery
   */
  async getImagery(cropId?: string, params?: ListParams): Promise<ImageryData[]> {
    const id = cropId || this.config.cropId;
    const response = await this.axios.get('/imagery', {
      params: { cropId: id, ...params },
    });
    return response.data;
  }

  /**
   * Get yield estimates
   */
  async getYieldEstimates(cropId?: string): Promise<YieldEstimate[]> {
    const id = cropId || this.config.cropId;
    if (!id) throw new Error('Crop ID is required');
    const response = await this.axios.get(`/crops/${id}/yield-estimates`);
    return response.data;
  }

  /**
   * Add yield estimate
   */
  async addYieldEstimate(
    estimate: Omit<YieldEstimate, 'estimateId'>
  ): Promise<YieldEstimate> {
    const response = await this.axios.post('/yield-estimates', estimate);
    return response.data;
  }

  /**
   * Get treatment records
   */
  async getTreatmentRecords(
    cropId?: string,
    params?: ListParams
  ): Promise<TreatmentRecord[]> {
    const id = cropId || this.config.cropId;
    const response = await this.axios.get('/treatments', {
      params: { cropId: id, ...params },
    });
    return response.data;
  }

  /**
   * Record treatment
   */
  async recordTreatment(
    treatment: Omit<TreatmentRecord, 'treatmentId'>
  ): Promise<TreatmentRecord> {
    const response = await this.axios.post('/treatments', treatment);
    return response.data;
  }

  /**
   * Get weather impacts
   */
  async getWeatherImpacts(
    cropId?: string,
    params?: ListParams
  ): Promise<WeatherImpact[]> {
    const id = cropId || this.config.cropId;
    const response = await this.axios.get('/weather-impacts', {
      params: { cropId: id, ...params },
    });
    return response.data;
  }

  /**
   * Record weather impact
   */
  async recordWeatherImpact(
    impact: Omit<WeatherImpact, 'impactId'>
  ): Promise<WeatherImpact> {
    const response = await this.axios.post('/weather-impacts', impact);
    return response.data;
  }

  /**
   * Get analytics report
   */
  async getAnalyticsReport(
    cropId?: string,
    params?: { period?: string }
  ): Promise<AnalyticsReport> {
    const id = cropId || this.config.cropId;
    if (!id) throw new Error('Crop ID is required');
    const response = await this.axios.get(`/crops/${id}/analytics`, { params });
    return response.data;
  }
}

/**
 * Health Monitor for crop health tracking
 */
export class HealthMonitor {
  private client: CropMonitoringClient;

  constructor(config: SDKConfig) {
    this.client = new CropMonitoringClient(config);
  }

  /**
   * Get overall crop health score
   */
  async getHealthScore(cropId?: string): Promise<number> {
    const assessment = await this.client.getHealthAssessment(cropId);
    const statusScores = {
      healthy: 100,
      stressed: 60,
      diseased: 30,
      critical: 10,
    };
    return statusScores[assessment.overallHealth];
  }

  /**
   * Check for critical issues
   */
  async checkCriticalIssues(cropId?: string): Promise<{
    hasCriticalIssues: boolean;
    issues: string[];
  }> {
    const [pests, diseases, health] = await Promise.all([
      this.client.getPestDetections(cropId, { limit: 10 }),
      this.client.getDiseaseDetections(cropId, { limit: 10 }),
      this.client.getHealthAssessment(cropId),
    ]);

    const issues: string[] = [];
    const criticalPests = pests.filter((p) => p.severity === 'severe');
    const criticalDiseases = diseases.filter((d) => d.severity === 'severe');

    if (criticalPests.length > 0) {
      issues.push(`${criticalPests.length} severe pest infestations detected`);
    }
    if (criticalDiseases.length > 0) {
      issues.push(`${criticalDiseases.length} severe diseases detected`);
    }
    if (health.overallHealth === 'critical') {
      issues.push('Crop in critical health condition');
    }

    return {
      hasCriticalIssues: issues.length > 0,
      issues,
    };
  }
}

/**
 * Issue Tracker for pest, disease, and weed management
 */
export class IssueTracker {
  private client: CropMonitoringClient;

  constructor(config: SDKConfig) {
    this.client = new CropMonitoringClient(config);
  }

  /**
   * Get all active issues
   */
  async getAllIssues(cropId?: string): Promise<{
    pests: PestDetection[];
    diseases: DiseaseDetection[];
    weeds: WeedDetection[];
  }> {
    const [pests, diseases, weeds] = await Promise.all([
      this.client.getPestDetections(cropId),
      this.client.getDiseaseDetections(cropId),
      this.client.getWeedDetections(cropId),
    ]);

    return { pests, diseases, weeds };
  }

  /**
   * Get high severity issues
   */
  async getHighSeverityIssues(cropId?: string): Promise<{
    pests: PestDetection[];
    diseases: DiseaseDetection[];
  }> {
    const [pests, diseases] = await Promise.all([
      this.client.getPestDetections(cropId),
      this.client.getDiseaseDetections(cropId),
    ]);

    return {
      pests: pests.filter((p) => p.severity === 'high' || p.severity === 'severe'),
      diseases: diseases.filter((d) => d.severity === 'high' || d.severity === 'severe'),
    };
  }
}

/**
 * Utility functions
 */
export const utils = {
  /**
   * Calculate crop age in days
   */
  calculateCropAge(plantingDate: string): number {
    const planted = new Date(plantingDate);
    const now = new Date();
    const diff = now.getTime() - planted.getTime();
    return Math.floor(diff / (1000 * 60 * 60 * 24));
  },

  /**
   * Calculate days to harvest
   */
  calculateDaysToHarvest(expectedHarvestDate: string): number {
    const harvest = new Date(expectedHarvestDate);
    const now = new Date();
    const diff = harvest.getTime() - now.getTime();
    return Math.floor(diff / (1000 * 60 * 60 * 24));
  },

  /**
   * Classify NDVI value
   */
  classifyNDVI(ndvi: number): 'poor' | 'moderate' | 'good' | 'excellent' {
    if (ndvi < 0.3) return 'poor';
    if (ndvi < 0.5) return 'moderate';
    if (ndvi < 0.7) return 'good';
    return 'excellent';
  },
};

/**
 * Default export
 */
export default {
  CropMonitoringClient,
  HealthMonitor,
  IssueTracker,
  utils,
};
