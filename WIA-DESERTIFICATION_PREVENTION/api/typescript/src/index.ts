/**
 * WIA Desertification Prevention Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig,
  WIADesertificationPrevention,
  ProjectResponse,
  Intervention,
  LandAssessment,
  MonitoringIndicator,
  ValidationResult,
  PaginatedResponse,
} from './types';

export class WIADesertificationPreventionClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }),
      },
    });
  }

  async createProject(project: WIADesertificationPrevention): Promise<ProjectResponse> {
    const response = await this.axios.post<ProjectResponse>('/projects', project);
    return response.data;
  }

  async getProject(id: string): Promise<WIADesertificationPrevention> {
    const response = await this.axios.get<WIADesertificationPrevention>(`/projects/${id}`);
    return response.data;
  }

  async listProjects(params?: { status?: string; country?: string; limit?: number; offset?: number }): Promise<PaginatedResponse<ProjectResponse>> {
    const response = await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', { params });
    return response.data;
  }

  async updateProject(id: string, updates: Partial<WIADesertificationPrevention>): Promise<ProjectResponse> {
    const response = await this.axios.put<ProjectResponse>(`/projects/${id}`, updates);
    return response.data;
  }

  async deleteProject(id: string): Promise<void> {
    await this.axios.delete(`/projects/${id}`);
  }

  async addIntervention(projectId: string, intervention: Intervention): Promise<Intervention> {
    const response = await this.axios.post<Intervention>(`/projects/${projectId}/interventions`, intervention);
    return response.data;
  }

  async listInterventions(projectId: string): Promise<Intervention[]> {
    const response = await this.axios.get<Intervention[]>(`/projects/${projectId}/interventions`);
    return response.data;
  }

  async updateIntervention(projectId: string, interventionId: string, updates: Partial<Intervention>): Promise<Intervention> {
    const response = await this.axios.put<Intervention>(`/projects/${projectId}/interventions/${interventionId}`, updates);
    return response.data;
  }

  async submitAssessment(projectId: string, assessment: LandAssessment): Promise<LandAssessment> {
    const response = await this.axios.post<LandAssessment>(`/projects/${projectId}/assessments`, assessment);
    return response.data;
  }

  async getLatestAssessment(projectId: string): Promise<LandAssessment> {
    const response = await this.axios.get<LandAssessment>(`/projects/${projectId}/assessments/latest`);
    return response.data;
  }

  async recordIndicatorValue(projectId: string, indicatorId: string, value: number, date?: string): Promise<void> {
    await this.axios.post(`/projects/${projectId}/indicators/${indicatorId}/values`, { value, date: date || new Date().toISOString() });
  }

  async getIndicatorHistory(projectId: string, indicatorId: string, params?: { startDate?: string; endDate?: string }): Promise<{ date: string; value: number }[]> {
    const response = await this.axios.get(`/projects/${projectId}/indicators/${indicatorId}/history`, { params });
    return response.data;
  }

  async getProjectStats(projectId: string): Promise<{
    areaRestored: number;
    treesPlanted: number;
    vegetationCoverChange: number;
    communityMembersEngaged: number;
  }> {
    const response = await this.axios.get(`/projects/${projectId}/stats`);
    return response.data;
  }

  async getSatelliteImagery(projectId: string, date?: string): Promise<{ url: string; capturedAt: string; ndvi?: number }> {
    const response = await this.axios.get(`/projects/${projectId}/imagery`, { params: { date } });
    return response.data;
  }

  validateProject(project: WIADesertificationPrevention): ValidationResult {
    const errors: any[] = [];
    if (project.standard !== 'WIA-DESERTIFICATION-PREVENTION') {
      errors.push({ path: 'standard', message: 'Invalid standard identifier' });
    }
    if (!project.project?.id) {
      errors.push({ path: 'project.id', message: 'Project ID is required' });
    }
    if (!project.region?.coordinates) {
      errors.push({ path: 'region.coordinates', message: 'Region coordinates are required' });
    }
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    const v = c === 'x' ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
}

export function createMinimalProject(name: string, country: string, lat: number, lng: number): WIADesertificationPrevention {
  return {
    standard: 'WIA-DESERTIFICATION-PREVENTION',
    version: '1.0.0',
    project: { id: generateUUID(), name, organization: 'Default', startDate: new Date().toISOString(), status: 'planning' },
    region: {
      id: generateUUID(),
      name: `${name} Region`,
      country,
      coordinates: { latitude: lat, longitude: lng },
      area: { value: 100, unit: 'hectares' },
      climate: { type: 'semi-arid', avgTemperature: 25, annualRainfall: 300 },
      soil: { type: 'sandy', degradationLevel: 'moderate', organicMatter: 1.5 },
    },
    assessment: {
      date: new Date().toISOString(),
      assessor: 'Initial',
      methodology: 'WIA Standard Assessment v1.0',
      desertificationRisk: 'moderate',
      vegetationCover: { coverPercentage: 30, dominantSpecies: [], healthStatus: 'stressed', nativeSpeciesRatio: 0.6 },
      waterResources: { availability: 'limited', sources: [] },
      biodiversity: { speciesRichness: 50, endemicSpecies: 5, threatenedSpecies: 3, ecosystemHealth: 'degraded' },
      humanImpact: { populationDensity: 50, landUse: ['agriculture', 'pastoral'], overgrazing: true, deforestation: false, unsustainableFarming: true, livelihoodsAtRisk: 1000 },
      overallScore: 45,
    },
    interventions: [],
    monitoring: { frequency: 'monthly', indicators: [], methods: [], reports: [] },
  };
}

export default { WIADesertificationPreventionClient, generateUUID, createMinimalProject };
