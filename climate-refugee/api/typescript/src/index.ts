/**
 * WIA Climate Refugee Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig,
  WIAClimateRefugeeProject,
  ProjectResponse,
  ValidationResult,
  PaginatedResponse,
  AffectedPopulation,
  ClimateHazard,
  DisplacementScenario,
  AssistanceSector,
  ResettlementSite,
  Indicator,
  VulnerableGroup,
} from './types';

// ============================================================================
// WIA Climate Refugee Client
// ============================================================================

export class WIAClimateRefugeeClient {
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

  async createProject(project: WIAClimateRefugeeProject): Promise<ProjectResponse> {
    const response = await this.axios.post<ProjectResponse>('/projects', project);
    return response.data;
  }

  async getProject(id: string): Promise<WIAClimateRefugeeProject> {
    const response = await this.axios.get<WIAClimateRefugeeProject>(`/projects/${id}`);
    return response.data;
  }

  async listProjects(params?: {
    type?: string;
    scope?: string;
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

  async updateProject(id: string, updates: Partial<WIAClimateRefugeeProject>): Promise<ProjectResponse> {
    const response = await this.axios.put<ProjectResponse>(`/projects/${id}`, updates);
    return response.data;
  }

  async deleteProject(id: string): Promise<void> {
    await this.axios.delete(`/projects/${id}`);
  }

  // ========================================================================
  // Climate Impact Assessment
  // ========================================================================

  async getClimateAssessment(projectId: string): Promise<any> {
    const response = await this.axios.get(`/projects/${projectId}/assessment`);
    return response.data;
  }

  async updateClimateAssessment(projectId: string, assessment: any): Promise<any> {
    const response = await this.axios.put(`/projects/${projectId}/assessment`, assessment);
    return response.data;
  }

  async addHazard(projectId: string, hazard: ClimateHazard): Promise<ClimateHazard> {
    const response = await this.axios.post<ClimateHazard>(
      `/projects/${projectId}/hazards`,
      hazard
    );
    return response.data;
  }

  async listHazards(projectId: string): Promise<ClimateHazard[]> {
    const response = await this.axios.get<ClimateHazard[]>(`/projects/${projectId}/hazards`);
    return response.data;
  }

  async calculateRiskLevel(projectId: string): Promise<RiskCalculation> {
    const response = await this.axios.post<RiskCalculation>(
      `/projects/${projectId}/assessment/risk`
    );
    return response.data;
  }

  // ========================================================================
  // Population Management
  // ========================================================================

  async getPopulation(projectId: string): Promise<AffectedPopulation> {
    const response = await this.axios.get<AffectedPopulation>(
      `/projects/${projectId}/population`
    );
    return response.data;
  }

  async updatePopulation(
    projectId: string,
    population: Partial<AffectedPopulation>
  ): Promise<AffectedPopulation> {
    const response = await this.axios.put<AffectedPopulation>(
      `/projects/${projectId}/population`,
      population
    );
    return response.data;
  }

  async registerPerson(projectId: string, person: PersonRegistration): Promise<RegistrationResult> {
    const response = await this.axios.post<RegistrationResult>(
      `/projects/${projectId}/population/register`,
      person
    );
    return response.data;
  }

  async bulkRegister(
    projectId: string,
    persons: PersonRegistration[]
  ): Promise<BulkRegistrationResult> {
    const response = await this.axios.post<BulkRegistrationResult>(
      `/projects/${projectId}/population/bulk-register`,
      { persons }
    );
    return response.data;
  }

  async searchPopulation(
    projectId: string,
    query: PopulationSearchQuery
  ): Promise<PaginatedResponse<PersonRecord>> {
    const response = await this.axios.post<PaginatedResponse<PersonRecord>>(
      `/projects/${projectId}/population/search`,
      query
    );
    return response.data;
  }

  async getVulnerableGroups(projectId: string): Promise<VulnerableGroup[]> {
    const response = await this.axios.get<VulnerableGroup[]>(
      `/projects/${projectId}/population/vulnerable`
    );
    return response.data;
  }

  // ========================================================================
  // Displacement Management
  // ========================================================================

  async getDisplacementPlan(projectId: string): Promise<any> {
    const response = await this.axios.get(`/projects/${projectId}/displacement`);
    return response.data;
  }

  async createScenario(
    projectId: string,
    scenario: DisplacementScenario
  ): Promise<DisplacementScenario> {
    const response = await this.axios.post<DisplacementScenario>(
      `/projects/${projectId}/displacement/scenarios`,
      scenario
    );
    return response.data;
  }

  async triggerEvacuation(
    projectId: string,
    scenarioId: string,
    data: EvacuationTrigger
  ): Promise<EvacuationResponse> {
    const response = await this.axios.post<EvacuationResponse>(
      `/projects/${projectId}/displacement/evacuate`,
      { scenarioId, ...data }
    );
    return response.data;
  }

  async getShelterStatus(projectId: string): Promise<ShelterStatus[]> {
    const response = await this.axios.get<ShelterStatus[]>(
      `/projects/${projectId}/displacement/shelters`
    );
    return response.data;
  }

  async allocateShelter(
    projectId: string,
    allocation: ShelterAllocation
  ): Promise<AllocationResult> {
    const response = await this.axios.post<AllocationResult>(
      `/projects/${projectId}/displacement/allocate`,
      allocation
    );
    return response.data;
  }

  // ========================================================================
  // Assistance Programs
  // ========================================================================

  async getAssistanceProgram(projectId: string): Promise<any> {
    const response = await this.axios.get(`/projects/${projectId}/assistance`);
    return response.data;
  }

  async addSector(projectId: string, sector: AssistanceSector): Promise<AssistanceSector> {
    const response = await this.axios.post<AssistanceSector>(
      `/projects/${projectId}/assistance/sectors`,
      sector
    );
    return response.data;
  }

  async listSectors(projectId: string): Promise<AssistanceSector[]> {
    const response = await this.axios.get<AssistanceSector[]>(
      `/projects/${projectId}/assistance/sectors`
    );
    return response.data;
  }

  async distributeCash(
    projectId: string,
    distribution: CashDistribution
  ): Promise<DistributionResult> {
    const response = await this.axios.post<DistributionResult>(
      `/projects/${projectId}/assistance/cash/distribute`,
      distribution
    );
    return response.data;
  }

  async recordDelivery(
    projectId: string,
    delivery: AssistanceDelivery
  ): Promise<DeliveryResult> {
    const response = await this.axios.post<DeliveryResult>(
      `/projects/${projectId}/assistance/deliver`,
      delivery
    );
    return response.data;
  }

  async getAssistanceHistory(
    personId: string,
    params?: { start?: string; end?: string }
  ): Promise<AssistanceRecord[]> {
    const response = await this.axios.get<AssistanceRecord[]>(
      `/persons/${personId}/assistance`,
      { params }
    );
    return response.data;
  }

  // ========================================================================
  // Resettlement
  // ========================================================================

  async getResettlementPlan(projectId: string): Promise<any> {
    const response = await this.axios.get(`/projects/${projectId}/resettlement`);
    return response.data;
  }

  async addResettlementSite(
    projectId: string,
    site: ResettlementSite
  ): Promise<ResettlementSite> {
    const response = await this.axios.post<ResettlementSite>(
      `/projects/${projectId}/resettlement/sites`,
      site
    );
    return response.data;
  }

  async listResettlementSites(projectId: string): Promise<ResettlementSite[]> {
    const response = await this.axios.get<ResettlementSite[]>(
      `/projects/${projectId}/resettlement/sites`
    );
    return response.data;
  }

  async submitResettlementApplication(
    projectId: string,
    application: ResettlementApplication
  ): Promise<ApplicationResult> {
    const response = await this.axios.post<ApplicationResult>(
      `/projects/${projectId}/resettlement/apply`,
      application
    );
    return response.data;
  }

  async getResettlementStatus(applicationId: string): Promise<ResettlementStatus> {
    const response = await this.axios.get<ResettlementStatus>(
      `/resettlement/applications/${applicationId}`
    );
    return response.data;
  }

  async processResettlement(
    applicationId: string,
    decision: ResettlementDecision
  ): Promise<ProcessResult> {
    const response = await this.axios.post<ProcessResult>(
      `/resettlement/applications/${applicationId}/process`,
      decision
    );
    return response.data;
  }

  // ========================================================================
  // Monitoring & Reporting
  // ========================================================================

  async getIndicators(projectId: string): Promise<Indicator[]> {
    const response = await this.axios.get<Indicator[]>(`/projects/${projectId}/indicators`);
    return response.data;
  }

  async updateIndicator(projectId: string, indicatorId: string, value: number): Promise<Indicator> {
    const response = await this.axios.put<Indicator>(
      `/projects/${projectId}/indicators/${indicatorId}`,
      { value }
    );
    return response.data;
  }

  async submitMonitoringData(
    projectId: string,
    data: MonitoringSubmission
  ): Promise<SubmissionResult> {
    const response = await this.axios.post<SubmissionResult>(
      `/projects/${projectId}/monitoring`,
      data
    );
    return response.data;
  }

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

  async getDashboard(projectId: string): Promise<DashboardData> {
    const response = await this.axios.get<DashboardData>(`/projects/${projectId}/dashboard`);
    return response.data;
  }

  // ========================================================================
  // Validation
  // ========================================================================

  validateProject(project: WIAClimateRefugeeProject): ValidationResult {
    const errors: any[] = [];

    if (!project.standard || project.standard !== 'WIA-CLIMATE-REFUGEE') {
      errors.push({
        path: 'standard',
        message: 'Standard must be "WIA-CLIMATE-REFUGEE"',
      });
    }

    if (!project.metadata?.id) {
      errors.push({ path: 'metadata.id', message: 'Project ID is required' });
    }

    if (!project.assessment?.hazards?.length) {
      errors.push({
        path: 'assessment.hazards',
        message: 'At least one climate hazard must be identified',
      });
    }

    if (!project.population?.total || project.population.total < 0) {
      errors.push({
        path: 'population.total',
        message: 'Valid affected population count is required',
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

export interface RiskCalculation {
  level: string;
  score: number;
  factors: { factor: string; weight: number; score: number }[];
  recommendations: string[];
}

export interface PersonRegistration {
  firstName: string;
  lastName: string;
  dateOfBirth?: string;
  gender?: string;
  nationality?: string;
  householdId?: string;
  vulnerabilities?: string[];
  location?: string;
  documents?: { type: string; number: string }[];
}

export interface RegistrationResult {
  id: string;
  registrationNumber: string;
  status: 'registered' | 'pending-verification';
  registeredAt: string;
}

export interface BulkRegistrationResult {
  total: number;
  successful: number;
  failed: number;
  errors?: { index: number; error: string }[];
}

export interface PopulationSearchQuery {
  name?: string;
  householdId?: string;
  location?: string;
  vulnerability?: string;
  limit?: number;
  offset?: number;
}

export interface PersonRecord {
  id: string;
  registrationNumber: string;
  name: string;
  householdId?: string;
  location?: string;
  status: string;
  vulnerabilities: string[];
}

export interface EvacuationTrigger {
  reason: string;
  urgency: 'low' | 'medium' | 'high' | 'critical';
  affectedAreas: string[];
  estimatedPopulation: number;
}

export interface EvacuationResponse {
  id: string;
  status: 'initiated' | 'in-progress' | 'completed';
  activatedRoutes: string[];
  sheltersActivated: string[];
  evacuatedCount?: number;
}

export interface ShelterStatus {
  id: string;
  name: string;
  capacity: number;
  occupied: number;
  available: number;
  status: string;
}

export interface ShelterAllocation {
  shelterId: string;
  householdIds: string[];
  arrivalDate: string;
}

export interface AllocationResult {
  allocations: { householdId: string; shelterId: string; unit?: string }[];
  successful: number;
  failed: number;
}

export interface CashDistribution {
  beneficiaryIds: string[];
  amount: number;
  currency: string;
  purpose: string;
  deliveryMethod: string;
}

export interface DistributionResult {
  id: string;
  totalDistributed: number;
  beneficiariesReached: number;
  timestamp: string;
}

export interface AssistanceDelivery {
  type: string;
  beneficiaryIds: string[];
  items?: { name: string; quantity: number }[];
  location: string;
  date: string;
}

export interface DeliveryResult {
  id: string;
  delivered: number;
  timestamp: string;
}

export interface AssistanceRecord {
  id: string;
  type: string;
  date: string;
  value?: number;
  items?: { name: string; quantity: number }[];
}

export interface ResettlementApplication {
  householdId: string;
  preferredSites: string[];
  specialNeeds?: string[];
  urgency: 'normal' | 'priority' | 'emergency';
  documents: string[];
}

export interface ApplicationResult {
  applicationId: string;
  status: 'submitted' | 'under-review';
  submittedAt: string;
}

export interface ResettlementStatus {
  applicationId: string;
  status: 'submitted' | 'under-review' | 'approved' | 'rejected' | 'completed';
  assignedSite?: string;
  timeline?: string;
}

export interface ResettlementDecision {
  decision: 'approve' | 'reject' | 'defer';
  siteId?: string;
  reason?: string;
  conditions?: string[];
}

export interface ProcessResult {
  applicationId: string;
  status: string;
  processedAt: string;
  nextSteps?: string[];
}

export interface MonitoringSubmission {
  period: { start: string; end: string };
  indicators: { id: string; value: number }[];
  narrative?: string;
  submitter: string;
}

export interface SubmissionResult {
  id: string;
  status: 'accepted' | 'pending-validation';
  timestamp: string;
}

export interface ReportOptions {
  format?: 'pdf' | 'xlsx' | 'docx';
  language?: string;
  sections?: string[];
}

export interface ReportResult {
  id: string;
  status: 'queued' | 'processing' | 'completed' | 'failed';
  url?: string;
}

export interface DashboardData {
  population: { total: number; registered: number; vulnerable: number };
  displacement: { displaced: number; sheltered: number; returned: number };
  assistance: { reached: number; pending: number; sectors: Record<string, number> };
  indicators: { id: string; name: string; current: number; target: number }[];
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

export function createMinimalProject(name: string, country: string): WIAClimateRefugeeProject {
  return {
    standard: 'WIA-CLIMATE-REFUGEE',
    version: '1.0.0',
    metadata: {
      id: generateUUID(),
      name,
      type: 'emergency-response',
      scope: 'national',
      region: { name: country, country },
      timeframe: { start: new Date().toISOString().split('T')[0] },
      leadOrganization: { name: 'Organization', type: 'ngo', country },
      status: 'planning',
    },
    assessment: { hazards: [], vulnerabilities: [], impacts: [], projections: [], riskLevel: 'moderate' },
    population: { total: 0, categories: [], vulnerableGroups: [], demographics: { ageDistribution: [], genderRatio: { male: 50, female: 50 }, householdSize: 4, urbanRural: { urban: 50, rural: 50 } } },
    displacement: { type: 'voluntary', triggers: [], scenarios: [], routes: [], temporaryShelter: { sites: [], totalCapacity: 0, standards: 'Sphere', duration: '6 months' } },
    protection: { legalFramework: { nationalLaws: [], internationalAgreements: [], gaps: [], recommendations: [] }, rights: [], mechanisms: [], documentation: { types: [], accessPoints: 0, processingTime: 'TBD', challenges: [] }, safetyMeasures: [] },
    assistance: { sectors: [], delivery: [], targeting: { approach: 'vulnerability', criteria: [], verification: 'TBD', appeals: 'TBD' }, budget: { total: { amount: 0, currency: 'USD' }, breakdown: [], funded: 0, gap: { amount: 0, currency: 'USD' }, sources: [] }, timeline: 'TBD' },
    resettlement: { type: 'internal', sites: [], criteria: { priorityGroups: [], eligibility: [], process: 'TBD', timeline: 'TBD' }, support: { package: { housing: 'TBD', livelihood: 'TBD', integration: 'TBD' }, duration: 'TBD', phaseOut: 'TBD' }, integration: { hostCommunity: { consultations: 'TBD', benefits: [], conflictPrevention: 'TBD' }, services: [], livelihoods: { programs: [], training: [], employment: 'TBD', entrepreneurship: 'TBD' }, social: { language: 'TBD', cultural: 'TBD', community: 'TBD', timeline: 'TBD' } }, timeline: { phases: [], milestones: [] } },
    monitoring: { indicators: [], dataCollection: { methods: [], frequency: 'monthly', responsibility: 'TBD', tools: [] }, reporting: { internal: 'monthly', external: 'quarterly', formats: ['pdf'], recipients: [] }, evaluation: { type: 'both', frequency: 'annual', methodology: 'TBD', independence: true } },
  };
}

// ============================================================================
// Exports
// ============================================================================

export default {
  WIAClimateRefugeeClient,
  generateUUID,
  createMinimalProject,
};
