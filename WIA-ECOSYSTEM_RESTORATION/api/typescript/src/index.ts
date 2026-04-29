/**
 * WIA Ecosystem Restoration Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIAEcosystemRestoration, ProjectResponse, SpeciesData, RestorationIntervention,
  MonitoringRecord, BiodiversityMetrics, Stakeholder, ValidationResult, PaginatedResponse,
} from './types';

export class WIAEcosystemRestorationClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: { 'Content-Type': 'application/json', ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }) },
    });
  }

  async createProject(project: WIAEcosystemRestoration): Promise<ProjectResponse> {
    const response = await this.axios.post<ProjectResponse>('/projects', project);
    return response.data;
  }

  async getProject(id: string): Promise<WIAEcosystemRestoration> {
    const response = await this.axios.get<WIAEcosystemRestoration>(`/projects/${id}`);
    return response.data;
  }

  async listProjects(params?: { type?: string; status?: string; limit?: number }): Promise<PaginatedResponse<ProjectResponse>> {
    const response = await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', { params });
    return response.data;
  }

  async updateProject(id: string, updates: Partial<WIAEcosystemRestoration>): Promise<ProjectResponse> {
    const response = await this.axios.put<ProjectResponse>(`/projects/${id}`, updates);
    return response.data;
  }

  async addSpecies(projectId: string, species: SpeciesData): Promise<SpeciesData> {
    const response = await this.axios.post<SpeciesData>(`/projects/${projectId}/species`, species);
    return response.data;
  }

  async listSpecies(projectId: string, params?: { type?: string; status?: string }): Promise<SpeciesData[]> {
    const response = await this.axios.get<SpeciesData[]>(`/projects/${projectId}/species`, { params });
    return response.data;
  }

  async updateSpeciesPopulation(projectId: string, speciesId: string, surveyData: { count: number; method: string }): Promise<SpeciesData> {
    const response = await this.axios.post<SpeciesData>(`/projects/${projectId}/species/${speciesId}/surveys`, surveyData);
    return response.data;
  }

  async createIntervention(projectId: string, intervention: Omit<RestorationIntervention, 'id' | 'status'>): Promise<RestorationIntervention> {
    const response = await this.axios.post<RestorationIntervention>(`/projects/${projectId}/interventions`, intervention);
    return response.data;
  }

  async listInterventions(projectId: string): Promise<RestorationIntervention[]> {
    const response = await this.axios.get<RestorationIntervention[]>(`/projects/${projectId}/interventions`);
    return response.data;
  }

  async updateInterventionStatus(projectId: string, interventionId: string, status: string, outcomes?: any[]): Promise<RestorationIntervention> {
    const response = await this.axios.patch<RestorationIntervention>(`/projects/${projectId}/interventions/${interventionId}`, { status, outcomes });
    return response.data;
  }

  async addMonitoringRecord(projectId: string, record: Omit<MonitoringRecord, 'id'>): Promise<MonitoringRecord> {
    const response = await this.axios.post<MonitoringRecord>(`/projects/${projectId}/monitoring`, record);
    return response.data;
  }

  async listMonitoringRecords(projectId: string, params?: { indicator?: string; from?: string; to?: string }): Promise<MonitoringRecord[]> {
    const response = await this.axios.get<MonitoringRecord[]>(`/projects/${projectId}/monitoring`, { params });
    return response.data;
  }

  async getBiodiversityMetrics(projectId: string): Promise<BiodiversityMetrics> {
    const response = await this.axios.get<BiodiversityMetrics>(`/projects/${projectId}/biodiversity`);
    return response.data;
  }

  async calculateBiodiversity(projectId: string): Promise<BiodiversityMetrics> {
    const response = await this.axios.post<BiodiversityMetrics>(`/projects/${projectId}/biodiversity/calculate`);
    return response.data;
  }

  async addStakeholder(projectId: string, stakeholder: Omit<Stakeholder, 'id'>): Promise<Stakeholder> {
    const response = await this.axios.post<Stakeholder>(`/projects/${projectId}/stakeholders`, stakeholder);
    return response.data;
  }

  async listStakeholders(projectId: string): Promise<Stakeholder[]> {
    const response = await this.axios.get<Stakeholder[]>(`/projects/${projectId}/stakeholders`);
    return response.data;
  }

  async getCarbonSequestration(projectId: string): Promise<{ total: number; annual: number; verified: number; credits: number }> {
    const response = await this.axios.get(`/projects/${projectId}/carbon`);
    return response.data;
  }

  async getProjectProgress(projectId: string): Promise<{ objectives: any[]; milestones: any[]; overall: number }> {
    const response = await this.axios.get(`/projects/${projectId}/progress`);
    return response.data;
  }

  async generateReport(projectId: string, type: 'progress' | 'annual' | 'donor' | 'public'): Promise<{ url: string; generatedAt: string }> {
    const response = await this.axios.post(`/projects/${projectId}/reports`, { type });
    return response.data;
  }

  async getSiteConditions(projectId: string): Promise<{ soil: any; water: any; vegetation: any; wildlife: any }> {
    const response = await this.axios.get(`/projects/${projectId}/site/conditions`);
    return response.data;
  }

  validateProject(project: WIAEcosystemRestoration): ValidationResult {
    const errors: { path: string; message: string }[] = [];
    if (project.standard !== 'WIA-ECOSYSTEM-RESTORATION') errors.push({ path: 'standard', message: 'Invalid standard' });
    if (!project.project?.id) errors.push({ path: 'project.id', message: 'Project ID required' });
    if (!project.site?.location) errors.push({ path: 'site.location', message: 'Site location required' });
    if (!project.species?.length) errors.push({ path: 'species', message: 'At least one species required' });
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16);
  });
}

export function createMinimalProject(name: string, lat: number, lng: number, type: string = 'forest'): WIAEcosystemRestoration {
  return {
    standard: 'WIA-ECOSYSTEM-RESTORATION',
    version: '1.0.0',
    project: {
      id: generateUUID(), name, type: type as any, status: 'planning', objectives: [], createdAt: new Date().toISOString(),
      timeline: { startDate: new Date().toISOString(), expectedEndDate: '', phases: [], milestones: [] },
      lead: { id: generateUUID(), name: 'Lead Organization', type: 'ngo', contact: { email: 'contact@example.com' } },
      certifications: [],
    },
    site: {
      id: generateUUID(), name: `${name} Site`,
      location: { latitude: lat, longitude: lng, country: '' },
      area: { total: 0, restored: 0, unit: 'hectares', zones: [] },
      ecosystem: { type: type, biome: '', keySpecies: [] },
      landUse: { previous: '', duration: 0, impacts: [], degradationCauses: [] },
      degradation: { level: 'moderate', factors: [], recoveryPotential: 'medium' },
      climate: { zone: '', annualRainfall: 0, temperature: { min: 0, max: 0, average: 0 }, growingSeason: { start: '', end: '' } },
    },
    species: [],
    interventions: [],
    monitoring: {
      id: generateUUID(), frequency: 'monthly', indicators: [], protocols: [], data: [],
      reporting: { frequency: 'quarterly', recipients: [], format: 'pdf', public: false },
    },
    biodiversity: { speciesRichness: 0, shannonIndex: 0, simpsonIndex: 0, evenness: 0, functionalDiversity: 0, trendAnalysis: { period: '', direction: 'stable', rate: 0, confidence: 0 }, comparisons: [] },
    stakeholders: [],
    funding: { totalBudget: 0, secured: 0, currency: 'USD', sources: [], expenditure: [] },
  };
}

export default { WIAEcosystemRestorationClient, generateUUID, createMinimalProject };
