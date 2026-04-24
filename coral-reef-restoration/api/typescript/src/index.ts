/**
 * WIA Coral Reef Restoration Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig,
  WIACoralReefProject,
  ProjectResponse,
  ValidationResult,
  PaginatedResponse,
  NurserySpecies,
  TransplantSite,
  TransplantEvent,
  MonitoringIndicator,
  BaselineSurvey,
  SpeciesRecord,
  WaterParameter,
} from './types';

// ============================================================================
// WIA Coral Reef Client
// ============================================================================

export class WIACoralReefClient {
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

  // ========================================================================
  // Project Management
  // ========================================================================

  async createProject(project: WIACoralReefProject): Promise<ProjectResponse> {
    const response = await this.axios.post<ProjectResponse>('/projects', project);
    return response.data;
  }

  async getProject(id: string): Promise<WIACoralReefProject> {
    const response = await this.axios.get<WIACoralReefProject>(`/projects/${id}`);
    return response.data;
  }

  async listProjects(params?: {
    type?: string;
    country?: string;
    status?: string;
    limit?: number;
    offset?: number;
  }): Promise<PaginatedResponse<ProjectResponse>> {
    const response = await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', {
      params,
    });
    return response.data;
  }

  async updateProject(id: string, updates: Partial<WIACoralReefProject>): Promise<ProjectResponse> {
    const response = await this.axios.put<ProjectResponse>(`/projects/${id}`, updates);
    return response.data;
  }

  async deleteProject(id: string): Promise<void> {
    await this.axios.delete(`/projects/${id}`);
  }

  // ========================================================================
  // Site Assessment
  // ========================================================================

  async getSiteAssessment(projectId: string): Promise<any> {
    const response = await this.axios.get(`/projects/${projectId}/assessment`);
    return response.data;
  }

  async submitSurvey(projectId: string, survey: BaselineSurvey): Promise<SurveyResult> {
    const response = await this.axios.post<SurveyResult>(
      `/projects/${projectId}/surveys`,
      survey
    );
    return response.data;
  }

  async listSurveys(projectId: string, params?: {
    type?: string;
    start?: string;
    end?: string;
  }): Promise<SurveyResult[]> {
    const response = await this.axios.get<SurveyResult[]>(
      `/projects/${projectId}/surveys`,
      { params }
    );
    return response.data;
  }

  async recordSpecies(
    projectId: string,
    surveyId: string,
    species: SpeciesRecord
  ): Promise<SpeciesRecord> {
    const response = await this.axios.post<SpeciesRecord>(
      `/projects/${projectId}/surveys/${surveyId}/species`,
      species
    );
    return response.data;
  }

  async getSpeciesList(projectId: string): Promise<SpeciesRecord[]> {
    const response = await this.axios.get<SpeciesRecord[]>(
      `/projects/${projectId}/species`
    );
    return response.data;
  }

  async submitWaterQuality(
    projectId: string,
    parameters: WaterParameter[]
  ): Promise<WaterQualityResult> {
    const response = await this.axios.post<WaterQualityResult>(
      `/projects/${projectId}/water-quality`,
      { parameters }
    );
    return response.data;
  }

  async getWaterQualityHistory(
    projectId: string,
    params?: { start?: string; end?: string }
  ): Promise<WaterQualityRecord[]> {
    const response = await this.axios.get<WaterQualityRecord[]>(
      `/projects/${projectId}/water-quality/history`,
      { params }
    );
    return response.data;
  }

  // ========================================================================
  // Coral Nursery
  // ========================================================================

  async getNurseryStatus(projectId: string): Promise<NurseryStatus> {
    const response = await this.axios.get<NurseryStatus>(
      `/projects/${projectId}/nursery`
    );
    return response.data;
  }

  async addNurserySpecies(
    projectId: string,
    species: NurserySpecies
  ): Promise<NurserySpecies> {
    const response = await this.axios.post<NurserySpecies>(
      `/projects/${projectId}/nursery/species`,
      species
    );
    return response.data;
  }

  async listNurserySpecies(projectId: string): Promise<NurserySpecies[]> {
    const response = await this.axios.get<NurserySpecies[]>(
      `/projects/${projectId}/nursery/species`
    );
    return response.data;
  }

  async updateFragmentCount(
    projectId: string,
    speciesId: string,
    count: number,
    notes?: string
  ): Promise<FragmentUpdate> {
    const response = await this.axios.put<FragmentUpdate>(
      `/projects/${projectId}/nursery/species/${speciesId}/fragments`,
      { count, notes }
    );
    return response.data;
  }

  async recordNurseryMaintenance(
    projectId: string,
    maintenance: MaintenanceRecord
  ): Promise<MaintenanceResult> {
    const response = await this.axios.post<MaintenanceResult>(
      `/projects/${projectId}/nursery/maintenance`,
      maintenance
    );
    return response.data;
  }

  async getNurseryMetrics(projectId: string): Promise<NurseryMetrics> {
    const response = await this.axios.get<NurseryMetrics>(
      `/projects/${projectId}/nursery/metrics`
    );
    return response.data;
  }

  // ========================================================================
  // Transplantation
  // ========================================================================

  async listTransplantSites(projectId: string): Promise<TransplantSite[]> {
    const response = await this.axios.get<TransplantSite[]>(
      `/projects/${projectId}/transplant/sites`
    );
    return response.data;
  }

  async createTransplantSite(
    projectId: string,
    site: Partial<TransplantSite>
  ): Promise<TransplantSite> {
    const response = await this.axios.post<TransplantSite>(
      `/projects/${projectId}/transplant/sites`,
      site
    );
    return response.data;
  }

  async scheduleTransplant(
    projectId: string,
    event: Partial<TransplantEvent>
  ): Promise<TransplantEvent> {
    const response = await this.axios.post<TransplantEvent>(
      `/projects/${projectId}/transplant/events`,
      event
    );
    return response.data;
  }

  async listTransplantEvents(
    projectId: string,
    params?: { site?: string; status?: string }
  ): Promise<TransplantEvent[]> {
    const response = await this.axios.get<TransplantEvent[]>(
      `/projects/${projectId}/transplant/events`,
      { params }
    );
    return response.data;
  }

  async recordTransplant(
    projectId: string,
    eventId: string,
    data: TransplantRecord
  ): Promise<TransplantResult> {
    const response = await this.axios.post<TransplantResult>(
      `/projects/${projectId}/transplant/events/${eventId}/record`,
      data
    );
    return response.data;
  }

  async getTransplantSurvival(
    projectId: string,
    siteId: string,
    params?: { period?: string }
  ): Promise<SurvivalData> {
    const response = await this.axios.get<SurvivalData>(
      `/projects/${projectId}/transplant/sites/${siteId}/survival`,
      { params }
    );
    return response.data;
  }

  // ========================================================================
  // Monitoring
  // ========================================================================

  async getIndicators(projectId: string): Promise<MonitoringIndicator[]> {
    const response = await this.axios.get<MonitoringIndicator[]>(
      `/projects/${projectId}/monitoring/indicators`
    );
    return response.data;
  }

  async updateIndicator(
    projectId: string,
    indicatorId: string,
    value: number,
    metadata?: Record<string, unknown>
  ): Promise<MonitoringIndicator> {
    const response = await this.axios.put<MonitoringIndicator>(
      `/projects/${projectId}/monitoring/indicators/${indicatorId}`,
      { value, metadata }
    );
    return response.data;
  }

  async submitMonitoringData(
    projectId: string,
    data: MonitoringSubmission
  ): Promise<MonitoringResult> {
    const response = await this.axios.post<MonitoringResult>(
      `/projects/${projectId}/monitoring/data`,
      data
    );
    return response.data;
  }

  async getMonitoringHistory(
    projectId: string,
    params?: { indicator?: string; start?: string; end?: string }
  ): Promise<MonitoringRecord[]> {
    const response = await this.axios.get<MonitoringRecord[]>(
      `/projects/${projectId}/monitoring/history`,
      { params }
    );
    return response.data;
  }

  async uploadPhoto(
    projectId: string,
    photo: PhotoUpload
  ): Promise<PhotoResult> {
    const response = await this.axios.post<PhotoResult>(
      `/projects/${projectId}/monitoring/photos`,
      photo
    );
    return response.data;
  }

  async getPhotoTimeline(
    projectId: string,
    location: string
  ): Promise<PhotoRecord[]> {
    const response = await this.axios.get<PhotoRecord[]>(
      `/projects/${projectId}/monitoring/photos/timeline`,
      { params: { location } }
    );
    return response.data;
  }

  // ========================================================================
  // Reporting
  // ========================================================================

  async generateReport(
    projectId: string,
    type: string,
    options?: ReportOptions
  ): Promise<ReportResult> {
    const response = await this.axios.post<ReportResult>(
      `/projects/${projectId}/reports`,
      { type, options }
    );
    return response.data;
  }

  async listReports(projectId: string): Promise<ReportSummary[]> {
    const response = await this.axios.get<ReportSummary[]>(
      `/projects/${projectId}/reports`
    );
    return response.data;
  }

  async getDashboard(projectId: string): Promise<DashboardData> {
    const response = await this.axios.get<DashboardData>(
      `/projects/${projectId}/dashboard`
    );
    return response.data;
  }

  // ========================================================================
  // Validation
  // ========================================================================

  validateProject(project: WIACoralReefProject): ValidationResult {
    const errors: any[] = [];

    if (!project.standard || project.standard !== 'WIA-CORAL-REEF-RESTORATION') {
      errors.push({
        path: 'standard',
        message: 'Standard must be "WIA-CORAL-REEF-RESTORATION"',
      });
    }

    if (!project.metadata?.id) {
      errors.push({ path: 'metadata.id', message: 'Project ID is required' });
    }

    if (!project.metadata?.location?.coordinates) {
      errors.push({
        path: 'metadata.location.coordinates',
        message: 'Site coordinates are required',
      });
    }

    if (!project.siteAssessment?.baseline) {
      errors.push({
        path: 'siteAssessment.baseline',
        message: 'Baseline survey is required',
      });
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined,
    };
  }
}

// ============================================================================
// Supporting Types
// ============================================================================

export interface SurveyResult {
  id: string;
  date: string;
  type: string;
  status: 'completed' | 'pending-review';
  summary: Record<string, number>;
}

export interface WaterQualityResult {
  id: string;
  timestamp: string;
  overall: string;
  parameters: { name: string; value: number; status: string }[];
}

export interface WaterQualityRecord {
  id: string;
  timestamp: string;
  parameters: Record<string, number>;
  overall: string;
}

export interface NurseryStatus {
  operational: boolean;
  totalFragments: number;
  species: number;
  readyForOutplanting: number;
  survivalRate: number;
  lastMaintenance: string;
}

export interface FragmentUpdate {
  speciesId: string;
  previousCount: number;
  newCount: number;
  change: number;
  timestamp: string;
}

export interface MaintenanceRecord {
  type: 'cleaning' | 'predator-removal' | 'disease-treatment' | 'inspection';
  date: string;
  personnel: string[];
  observations: string;
  actions: string[];
}

export interface MaintenanceResult {
  id: string;
  recorded: boolean;
  timestamp: string;
}

export interface NurseryMetrics {
  totalFragments: number;
  survivalRate: number;
  averageGrowthRate: number;
  speciesCount: number;
  productionRate: number;
  readyForOutplanting: number;
  outplantedTotal: number;
}

export interface TransplantRecord {
  speciesTransplanted: { species: string; quantity: number }[];
  attachmentMethod: string;
  conditions: { temperature: number; visibility: number; current: number };
  personnel: string[];
  duration: number;
  notes?: string;
  photos?: string[];
}

export interface TransplantResult {
  eventId: string;
  recorded: boolean;
  totalTransplanted: number;
  timestamp: string;
}

export interface SurvivalData {
  siteId: string;
  period: string;
  totalTransplanted: number;
  surviving: number;
  survivalRate: number;
  bySpecies: { species: string; transplanted: number; surviving: number; rate: number }[];
  trends: { date: string; rate: number }[];
}

export interface MonitoringSubmission {
  date: string;
  site?: string;
  indicators: { id: string; value: number }[];
  observations?: string;
  surveyor: string;
}

export interface MonitoringResult {
  id: string;
  submitted: boolean;
  timestamp: string;
}

export interface MonitoringRecord {
  id: string;
  date: string;
  site?: string;
  indicators: { id: string; name: string; value: number }[];
  surveyor: string;
}

export interface PhotoUpload {
  location: { latitude: number; longitude: number; depth?: number };
  timestamp: string;
  description?: string;
  tags?: string[];
  base64Data?: string;
  url?: string;
}

export interface PhotoResult {
  id: string;
  uploaded: boolean;
  url?: string;
  timestamp: string;
}

export interface PhotoRecord {
  id: string;
  location: { latitude: number; longitude: number };
  timestamp: string;
  url: string;
  description?: string;
}

export interface ReportOptions {
  format?: 'pdf' | 'docx' | 'html';
  period?: { start: string; end: string };
  sections?: string[];
  language?: string;
}

export interface ReportResult {
  id: string;
  status: 'queued' | 'processing' | 'completed' | 'failed';
  url?: string;
  createdAt: string;
}

export interface ReportSummary {
  id: string;
  type: string;
  createdAt: string;
  status: string;
  url?: string;
}

export interface DashboardData {
  project: { name: string; status: string; progress: number };
  nursery: { fragments: number; survival: number; ready: number };
  transplant: { sites: number; total: number; survival: number };
  reefHealth: { coralCover: number; change: number; health: string };
  indicators: { id: string; name: string; current: number; target: number }[];
  recentActivity: { date: string; type: string; description: string }[];
}

// ============================================================================
// Utility Functions
// ============================================================================

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    const v = c === 'x' ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
}

export function calculateSurvivalRate(initial: number, surviving: number): number {
  if (initial <= 0) return 0;
  return Math.round((surviving / initial) * 100 * 100) / 100;
}

export function calculateGrowthRate(initial: number, final: number, months: number): number {
  if (initial <= 0 || months <= 0) return 0;
  return Math.round(((final - initial) / initial / months) * 100 * 100) / 100;
}

export function createMinimalProject(name: string, country: string): WIACoralReefProject {
  return {
    standard: 'WIA-CORAL-REEF-RESTORATION',
    version: '1.0.0',
    metadata: {
      id: generateUUID(),
      name,
      type: 'active-restoration',
      location: { name: '', country, region: '', coordinates: { latitude: 0, longitude: 0 }, reefArea: { total: 0, targetRestoration: 0, unit: 'hectares' }, depth: { min: 0, max: 0, unit: 'meters' } },
      timeframe: { startDate: new Date().toISOString().split('T')[0], phases: [], monitoringPeriod: 5 },
      leadOrganization: { name: 'Organization', type: 'ngo', country },
      funding: { total: { amount: 0, currency: 'USD' }, sources: [], secured: 0 },
      status: 'planning',
    },
    siteAssessment: { baseline: { date: '', methodology: '', coralCover: { live: 0, dead: 0, algae: 0, sand: 0, rubble: 0, other: 0 }, reefHealth: { score: 0, category: 'fair', indicators: [] }, speciesInventory: [], substrate: { type: 'natural-reef', stability: 'stable', suitability: 'good' } }, waterQuality: { parameters: [], overall: 'good' }, threats: { threats: [], overallRisk: 'moderate', climateProjections: [] }, suitability: { score: 0, category: 'suitable', factors: [], recommendations: [] }, biodiversity: { coralDiversity: { speciesRichness: 0 }, fishDiversity: { speciesRichness: 0 }, invertebrateCount: 0, keyStoneSpecies: [], endangeredSpecies: [], ecologicalFunction: '' } },
    restoration: { objectives: [], methods: [], targets: [], timeline: { activities: [], milestones: [], dependencies: [] }, resources: { personnel: { roles: [], volunteers: { count: 0, roles: [], training: '' }, training: { topics: [], duration: '' } }, equipment: { diving: [], scientific: [], restoration: [], boats: [] }, materials: { substrates: [], nurseryMaterials: [], markers: [], other: [] }, budget: { total: { amount: 0, currency: 'USD' }, categories: [], contingency: 10 } }, risks: [] },
    coralNursery: { type: 'in-situ-rope', locations: [], species: [], capacity: { current: 0, maximum: 0, annualProduction: 0 }, maintenance: { cleaning: { frequency: '', procedure: '', responsible: '' }, monitoring: { frequency: '', procedure: '', responsible: '' }, predatorControl: { frequency: '', procedure: '', responsible: '' }, diseaseManagement: { monitoring: '', identification: [], treatment: [], quarantine: '' } }, production: { fragmentsProduced: 0, survivalRate: 0, growthRate: 0, readyForOutplanting: 0, outplantedToDate: 0 } },
    transplantation: { sites: [], methods: [], schedule: { events: [], constraints: [], optimalConditions: { temperature: { min: 24, max: 30 }, visibility: 5, currentSpeed: 0.5 } }, protocols: { preTransplant: [], transport: { container: '', temperature: { min: 24, max: 28 }, maxDuration: '', handling: [] }, attachment: [], documentation: { preTransplant: [], duringTransplant: [], postTransplant: [] } }, postCare: { monitoring: { frequency: '', duration: '', parameters: [], photography: true }, maintenance: [], interventions: [] } },
    monitoring: { objectives: [], indicators: [], methods: [], schedule: { regular: { frequency: '', duration: '' }, postEvent: [], adaptive: '' }, dataManagement: { platform: '', storage: '', sharing: '', quality: '', backup: '' }, reporting: { internal: { frequency: '', recipients: [] }, external: { frequency: '', recipients: [] }, public: { frequency: '', channels: [] } } },
    communityEngagement: { engagement: { activities: [], participationTarget: 0, stakeholders: [] }, education: { schools: { schools: 0, students: 0, curriculum: [], activities: [] }, publicAwareness: { campaigns: [], materials: [], reach: 0, channels: [] }, training: { topics: [], participants: 0, outcomes: [] } }, livelihoods: { opportunities: [], beneficiaries: 0, sustainableUse: [] }, governance: { structure: '', decisionMaking: '', conflictResolution: '', compliance: '' } },
  };
}

// ============================================================================
// Exports
// ============================================================================

export default {
  WIACoralReefClient,
  generateUUID,
  calculateSurvivalRate,
  calculateGrowthRate,
  createMinimalProject,
};
