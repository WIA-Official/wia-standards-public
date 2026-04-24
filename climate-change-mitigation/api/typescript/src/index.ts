/**
 * WIA Climate Change Mitigation Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig,
  WIAMitigationProject,
  ProjectResponse,
  ValidationResult,
  PaginatedResponse,
  CreditIssuance,
  MitigationIntervention,
  EmissionInventory,
  MonitoringParameter,
  EmissionReduction,
  GreenhouseGas,
} from './types';

// ============================================================================
// WIA Mitigation Client
// ============================================================================

export class WIAMitigationClient {
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

  async createProject(project: WIAMitigationProject): Promise<ProjectResponse> {
    const response = await this.axios.post<ProjectResponse>('/projects', project);
    return response.data;
  }

  async getProject(id: string): Promise<WIAMitigationProject> {
    const response = await this.axios.get<WIAMitigationProject>(`/projects/${id}`);
    return response.data;
  }

  async listProjects(params?: {
    type?: string;
    sector?: string;
    status?: string;
    country?: string;
    limit?: number;
    offset?: number;
  }): Promise<PaginatedResponse<ProjectResponse>> {
    const response = await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', {
      params,
    });
    return response.data;
  }

  async updateProject(id: string, updates: Partial<WIAMitigationProject>): Promise<ProjectResponse> {
    const response = await this.axios.put<ProjectResponse>(`/projects/${id}`, updates);
    return response.data;
  }

  async deleteProject(id: string): Promise<void> {
    await this.axios.delete(`/projects/${id}`);
  }

  async submitForValidation(id: string): Promise<{ status: string; validationId: string }> {
    const response = await this.axios.post(`/projects/${id}/validate`);
    return response.data;
  }

  async registerProject(id: string): Promise<{ registrationId: string; registeredAt: string }> {
    const response = await this.axios.post(`/projects/${id}/register`);
    return response.data;
  }

  // ========================================================================
  // Baseline & Emissions
  // ========================================================================

  async calculateBaseline(
    projectId: string,
    data: BaselineCalculationInput
  ): Promise<BaselineCalculationResult> {
    const response = await this.axios.post<BaselineCalculationResult>(
      `/projects/${projectId}/baseline/calculate`,
      data
    );
    return response.data;
  }

  async getEmissionInventory(projectId: string): Promise<EmissionInventory> {
    const response = await this.axios.get<EmissionInventory>(
      `/projects/${projectId}/emissions`
    );
    return response.data;
  }

  async updateEmissionInventory(
    projectId: string,
    inventory: Partial<EmissionInventory>
  ): Promise<EmissionInventory> {
    const response = await this.axios.put<EmissionInventory>(
      `/projects/${projectId}/emissions`,
      inventory
    );
    return response.data;
  }

  async calculateEmissionReductions(
    projectId: string,
    period: { start: string; end: string }
  ): Promise<EmissionReductionResult> {
    const response = await this.axios.post<EmissionReductionResult>(
      `/projects/${projectId}/reductions/calculate`,
      period
    );
    return response.data;
  }

  // ========================================================================
  // Interventions
  // ========================================================================

  async addIntervention(
    projectId: string,
    intervention: MitigationIntervention
  ): Promise<MitigationIntervention> {
    const response = await this.axios.post<MitigationIntervention>(
      `/projects/${projectId}/interventions`,
      intervention
    );
    return response.data;
  }

  async listInterventions(projectId: string): Promise<MitigationIntervention[]> {
    const response = await this.axios.get<MitigationIntervention[]>(
      `/projects/${projectId}/interventions`
    );
    return response.data;
  }

  async updateIntervention(
    projectId: string,
    interventionId: string,
    updates: Partial<MitigationIntervention>
  ): Promise<MitigationIntervention> {
    const response = await this.axios.put<MitigationIntervention>(
      `/projects/${projectId}/interventions/${interventionId}`,
      updates
    );
    return response.data;
  }

  async deleteIntervention(projectId: string, interventionId: string): Promise<void> {
    await this.axios.delete(`/projects/${projectId}/interventions/${interventionId}`);
  }

  // ========================================================================
  // Monitoring
  // ========================================================================

  async submitMonitoringData(
    projectId: string,
    data: MonitoringDataSubmission
  ): Promise<{ submissionId: string; status: string }> {
    const response = await this.axios.post(
      `/projects/${projectId}/monitoring/data`,
      data
    );
    return response.data;
  }

  async getMonitoringData(
    projectId: string,
    params?: {
      parameter?: string;
      start?: string;
      end?: string;
      limit?: number;
    }
  ): Promise<MonitoringDataResponse> {
    const response = await this.axios.get<MonitoringDataResponse>(
      `/projects/${projectId}/monitoring/data`,
      { params }
    );
    return response.data;
  }

  async listMonitoringParameters(projectId: string): Promise<MonitoringParameter[]> {
    const response = await this.axios.get<MonitoringParameter[]>(
      `/projects/${projectId}/monitoring/parameters`
    );
    return response.data;
  }

  async generateMonitoringReport(
    projectId: string,
    period: { start: string; end: string }
  ): Promise<ReportResult> {
    const response = await this.axios.post<ReportResult>(
      `/projects/${projectId}/monitoring/report`,
      period
    );
    return response.data;
  }

  // ========================================================================
  // Verification & Credits
  // ========================================================================

  async submitForVerification(
    projectId: string,
    data: VerificationSubmission
  ): Promise<{ verificationId: string; status: string }> {
    const response = await this.axios.post(
      `/projects/${projectId}/verification/submit`,
      data
    );
    return response.data;
  }

  async getVerificationStatus(
    projectId: string,
    verificationId: string
  ): Promise<VerificationStatus> {
    const response = await this.axios.get<VerificationStatus>(
      `/projects/${projectId}/verification/${verificationId}`
    );
    return response.data;
  }

  async requestCreditIssuance(
    projectId: string,
    request: CreditIssuanceRequest
  ): Promise<CreditIssuance> {
    const response = await this.axios.post<CreditIssuance>(
      `/projects/${projectId}/credits/issue`,
      request
    );
    return response.data;
  }

  async listCredits(projectId: string): Promise<CreditIssuance[]> {
    const response = await this.axios.get<CreditIssuance[]>(
      `/projects/${projectId}/credits`
    );
    return response.data;
  }

  async retireCredits(
    creditId: string,
    quantity: number,
    beneficiary: string,
    reason: string
  ): Promise<CreditRetirement> {
    const response = await this.axios.post<CreditRetirement>(
      `/credits/${creditId}/retire`,
      { quantity, beneficiary, reason }
    );
    return response.data;
  }

  async transferCredits(
    creditId: string,
    quantity: number,
    recipient: string
  ): Promise<CreditTransfer> {
    const response = await this.axios.post<CreditTransfer>(
      `/credits/${creditId}/transfer`,
      { quantity, recipient }
    );
    return response.data;
  }

  // ========================================================================
  // Reporting
  // ========================================================================

  async generateReport(
    projectId: string,
    templateId: string,
    options?: ReportOptions
  ): Promise<ReportResult> {
    const response = await this.axios.post<ReportResult>(
      `/projects/${projectId}/reports/generate`,
      { templateId, options }
    );
    return response.data;
  }

  async listReports(projectId: string): Promise<ReportSummary[]> {
    const response = await this.axios.get<ReportSummary[]>(
      `/projects/${projectId}/reports`
    );
    return response.data;
  }

  async getReport(reportId: string): Promise<any> {
    const response = await this.axios.get(`/reports/${reportId}`);
    return response.data;
  }

  // ========================================================================
  // Validation
  // ========================================================================

  validateProject(project: WIAMitigationProject): ValidationResult {
    const errors: any[] = [];

    if (!project.standard || project.standard !== 'WIA-CLIMATE-CHANGE-MITIGATION') {
      errors.push({
        path: 'standard',
        message: 'Standard must be "WIA-CLIMATE-CHANGE-MITIGATION"',
      });
    }

    if (!project.metadata?.id) {
      errors.push({ path: 'metadata.id', message: 'Project ID is required' });
    }

    if (!project.baseline?.methodology) {
      errors.push({ path: 'baseline.methodology', message: 'Methodology is required' });
    }

    if (!project.interventions?.length) {
      errors.push({
        path: 'interventions',
        message: 'At least one intervention is required',
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

export interface BaselineCalculationInput {
  year: number;
  activityData: Record<string, number>;
  emissionFactors?: Record<string, number>;
}

export interface BaselineCalculationResult {
  total: number;
  unit: 'tCO2e';
  breakdown: Record<string, number>;
  methodology: string;
  calculatedAt: string;
}

export interface EmissionReductionResult {
  period: { start: string; end: string };
  baseline: number;
  project: number;
  leakage: number;
  netReduction: number;
  unit: 'tCO2e';
  methodology: string;
}

export interface MonitoringDataSubmission {
  period: { start: string; end: string };
  parameters: ParameterData[];
  submitter: string;
  notes?: string;
}

export interface ParameterData {
  parameterId: string;
  values: DataValue[];
}

export interface DataValue {
  timestamp: string;
  value: number;
  quality?: string;
}

export interface MonitoringDataResponse {
  data: ParameterData[];
  period: { start: string; end: string };
  coverage: number;
}

export interface VerificationSubmission {
  period: { start: string; end: string };
  monitoringReportId: string;
  claimedReductions: number;
  documents: string[];
}

export interface VerificationStatus {
  id: string;
  status: 'submitted' | 'in-review' | 'site-visit' | 'completed' | 'rejected';
  verifier: string;
  startedAt: string;
  completedAt?: string;
  findings?: string[];
  verifiedReductions?: number;
}

export interface CreditIssuanceRequest {
  verificationId: string;
  vintage: number;
  quantity: number;
  serialNumberPrefix?: string;
}

export interface CreditRetirement {
  id: string;
  creditId: string;
  quantity: number;
  beneficiary: string;
  reason: string;
  retiredAt: string;
  certificate?: string;
}

export interface CreditTransfer {
  id: string;
  creditId: string;
  quantity: number;
  from: string;
  to: string;
  transferredAt: string;
}

export interface ReportOptions {
  format?: 'pdf' | 'xlsx' | 'json';
  language?: string;
  sections?: string[];
}

export interface ReportResult {
  id: string;
  status: 'queued' | 'processing' | 'completed' | 'failed';
  url?: string;
  createdAt: string;
}

export interface ReportSummary {
  id: string;
  name: string;
  type: string;
  createdAt: string;
  status: string;
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

export function calculateGWP(gas: GreenhouseGas, quantity: number): number {
  const gwpValues: Record<GreenhouseGas, number> = {
    CO2: 1,
    CH4: 28,
    N2O: 265,
    HFCs: 1300,
    PFCs: 7000,
    SF6: 23500,
    NF3: 16100,
  };
  return quantity * (gwpValues[gas] || 1);
}

export function convertToTCO2e(gas: GreenhouseGas, tonnes: number): number {
  return calculateGWP(gas, tonnes);
}

export function createMinimalProject(
  name: string,
  type: string,
  country: string
): WIAMitigationProject {
  return {
    standard: 'WIA-CLIMATE-CHANGE-MITIGATION',
    version: '1.0.0',
    metadata: {
      id: generateUUID(),
      name,
      type: type as any,
      sector: 'energy',
      location: { country },
      timeframe: {
        startDate: new Date().toISOString().split('T')[0],
        creditingPeriod: {
          start: new Date().toISOString().split('T')[0],
          end: new Date(Date.now() + 7 * 365 * 24 * 60 * 60 * 1000).toISOString().split('T')[0],
          renewable: true,
        },
      },
      developer: { name: 'Developer', type: 'company', country },
      status: 'concept',
    },
    baseline: {
      scenario: {
        description: 'Business as usual scenario',
        assumptions: [],
        dataYear: new Date().getFullYear() - 1,
        projectionMethod: 'linear',
      },
      emissions: {
        scope1: [],
        total: { value: 0, unit: 'tCO2e', breakdown: {} },
      },
      methodology: {
        name: 'Methodology',
        version: '1.0',
        registry: 'WIA',
        applicability: [],
        baselineApproach: 'project-specific',
        quantificationMethod: 'direct-measurement',
      },
      additionality: {
        type: 'barrier-analysis',
        conclusion: 'To be determined',
      },
    },
    interventions: [],
    monitoring: {
      parameters: [],
      procedures: [],
      dataManagement: {
        storage: 'cloud',
        backup: 'daily',
        retention: 10,
        security: 'encrypted',
        access: [],
      },
      qualityControl: {
        procedures: [],
        audits: { internal: { frequency: 'annual', scope: 'full' } },
      },
    },
    verification: {
      verifier: { name: 'TBD', accreditation: 'TBD', country: 'TBD' },
      schedule: {
        frequency: 'annual',
        firstVerification: 'TBD',
        reportingDeadline: 'TBD',
      },
      scope: { boundaries: 'project', gases: ['CO2'], sources: [] },
      standards: ['WIA'],
    },
    reporting: { templates: [], schedule: { monitoringReports: 'quarterly', verificationReports: 'annual', annualReports: 'annual' }, recipients: [] },
  };
}

// ============================================================================
// Exports
// ============================================================================

export default {
  WIAMitigationClient,
  generateUUID,
  calculateGWP,
  convertToTCO2e,
  createMinimalProject,
};
