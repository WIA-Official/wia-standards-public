/**
 * WIA Cryo Asset Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig,
  WIACryoAssetProject,
  ProjectResponse,
  ValidationResult,
  PaginatedResponse,
  CryoAsset,
  AssetStatus,
  StorageFacility,
  StorageEquipment,
  ValuationRecord,
  InsurancePolicy,
  AssetEvent,
} from './types';

// ============================================================================
// WIA Cryo Asset Client
// ============================================================================

export class WIACryoAssetClient {
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

  async createProject(project: WIACryoAssetProject): Promise<ProjectResponse> {
    const response = await this.axios.post<ProjectResponse>('/projects', project);
    return response.data;
  }

  async getProject(id: string): Promise<WIACryoAssetProject> {
    const response = await this.axios.get<WIACryoAssetProject>(`/projects/${id}`);
    return response.data;
  }

  async listProjects(params?: {
    type?: string;
    status?: string;
    organization?: string;
    limit?: number;
    offset?: number;
  }): Promise<PaginatedResponse<ProjectResponse>> {
    const response = await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', {
      params,
    });
    return response.data;
  }

  async updateProject(id: string, updates: Partial<WIACryoAssetProject>): Promise<ProjectResponse> {
    const response = await this.axios.put<ProjectResponse>(`/projects/${id}`, updates);
    return response.data;
  }

  async deleteProject(id: string): Promise<void> {
    await this.axios.delete(`/projects/${id}`);
  }

  // ========================================================================
  // Asset Management
  // ========================================================================

  async registerAsset(projectId: string, asset: Partial<CryoAsset>): Promise<CryoAsset> {
    const response = await this.axios.post<CryoAsset>(
      `/projects/${projectId}/assets`,
      asset
    );
    return response.data;
  }

  async getAsset(projectId: string, assetId: string): Promise<CryoAsset> {
    const response = await this.axios.get<CryoAsset>(
      `/projects/${projectId}/assets/${assetId}`
    );
    return response.data;
  }

  async listAssets(projectId: string, params?: {
    type?: string;
    category?: string;
    status?: string;
    facilityId?: string;
    limit?: number;
    offset?: number;
  }): Promise<PaginatedResponse<CryoAsset>> {
    const response = await this.axios.get<PaginatedResponse<CryoAsset>>(
      `/projects/${projectId}/assets`,
      { params }
    );
    return response.data;
  }

  async updateAsset(
    projectId: string,
    assetId: string,
    updates: Partial<CryoAsset>
  ): Promise<CryoAsset> {
    const response = await this.axios.put<CryoAsset>(
      `/projects/${projectId}/assets/${assetId}`,
      updates
    );
    return response.data;
  }

  async updateAssetStatus(
    projectId: string,
    assetId: string,
    status: AssetStatus,
    notes?: string
  ): Promise<CryoAsset> {
    const response = await this.axios.put<CryoAsset>(
      `/projects/${projectId}/assets/${assetId}/status`,
      { status, notes }
    );
    return response.data;
  }

  async transferAsset(
    projectId: string,
    assetId: string,
    transfer: AssetTransfer
  ): Promise<TransferResult> {
    const response = await this.axios.post<TransferResult>(
      `/projects/${projectId}/assets/${assetId}/transfer`,
      transfer
    );
    return response.data;
  }

  async getAssetHistory(
    projectId: string,
    assetId: string,
    params?: { start?: string; end?: string; type?: string }
  ): Promise<AssetEvent[]> {
    const response = await this.axios.get<AssetEvent[]>(
      `/projects/${projectId}/assets/${assetId}/history`,
      { params }
    );
    return response.data;
  }

  async disposeAsset(
    projectId: string,
    assetId: string,
    disposal: DisposalRequest
  ): Promise<DisposalResult> {
    const response = await this.axios.post<DisposalResult>(
      `/projects/${projectId}/assets/${assetId}/dispose`,
      disposal
    );
    return response.data;
  }

  // ========================================================================
  // Storage Management
  // ========================================================================

  async listFacilities(projectId: string): Promise<StorageFacility[]> {
    const response = await this.axios.get<StorageFacility[]>(
      `/projects/${projectId}/facilities`
    );
    return response.data;
  }

  async getFacility(projectId: string, facilityId: string): Promise<StorageFacility> {
    const response = await this.axios.get<StorageFacility>(
      `/projects/${projectId}/facilities/${facilityId}`
    );
    return response.data;
  }

  async addFacility(
    projectId: string,
    facility: Partial<StorageFacility>
  ): Promise<StorageFacility> {
    const response = await this.axios.post<StorageFacility>(
      `/projects/${projectId}/facilities`,
      facility
    );
    return response.data;
  }

  async listEquipment(
    projectId: string,
    facilityId?: string
  ): Promise<StorageEquipment[]> {
    const response = await this.axios.get<StorageEquipment[]>(
      `/projects/${projectId}/equipment`,
      { params: { facilityId } }
    );
    return response.data;
  }

  async getEquipment(
    projectId: string,
    equipmentId: string
  ): Promise<StorageEquipment> {
    const response = await this.axios.get<StorageEquipment>(
      `/projects/${projectId}/equipment/${equipmentId}`
    );
    return response.data;
  }

  async getEquipmentReadings(
    projectId: string,
    equipmentId: string,
    params?: { start?: string; end?: string; parameter?: string }
  ): Promise<EquipmentReading[]> {
    const response = await this.axios.get<EquipmentReading[]>(
      `/projects/${projectId}/equipment/${equipmentId}/readings`,
      { params }
    );
    return response.data;
  }

  async assignStorage(
    projectId: string,
    assetId: string,
    assignment: StorageAssignmentRequest
  ): Promise<StorageAssignmentResult> {
    const response = await this.axios.post<StorageAssignmentResult>(
      `/projects/${projectId}/assets/${assetId}/assign`,
      assignment
    );
    return response.data;
  }

  async getStorageInventory(
    projectId: string,
    facilityId: string,
    equipmentId?: string
  ): Promise<InventoryReport> {
    const response = await this.axios.get<InventoryReport>(
      `/projects/${projectId}/facilities/${facilityId}/inventory`,
      { params: { equipmentId } }
    );
    return response.data;
  }

  // ========================================================================
  // Valuation
  // ========================================================================

  async requestValuation(
    projectId: string,
    request: ValuationRequest
  ): Promise<ValuationResult> {
    const response = await this.axios.post<ValuationResult>(
      `/projects/${projectId}/valuations`,
      request
    );
    return response.data;
  }

  async getValuationHistory(projectId: string): Promise<ValuationRecord[]> {
    const response = await this.axios.get<ValuationRecord[]>(
      `/projects/${projectId}/valuations`
    );
    return response.data;
  }

  async getAssetValuation(
    projectId: string,
    assetId: string
  ): Promise<AssetValuation> {
    const response = await this.axios.get<AssetValuation>(
      `/projects/${projectId}/assets/${assetId}/valuation`
    );
    return response.data;
  }

  // ========================================================================
  // Insurance
  // ========================================================================

  async addInsurancePolicy(
    projectId: string,
    policy: Partial<InsurancePolicy>
  ): Promise<InsurancePolicy> {
    const response = await this.axios.post<InsurancePolicy>(
      `/projects/${projectId}/insurance`,
      policy
    );
    return response.data;
  }

  async listInsurancePolicies(projectId: string): Promise<InsurancePolicy[]> {
    const response = await this.axios.get<InsurancePolicy[]>(
      `/projects/${projectId}/insurance`
    );
    return response.data;
  }

  async fileClaim(
    projectId: string,
    policyId: string,
    claim: ClaimRequest
  ): Promise<ClaimResult> {
    const response = await this.axios.post<ClaimResult>(
      `/projects/${projectId}/insurance/${policyId}/claims`,
      claim
    );
    return response.data;
  }

  async getCoverageAnalysis(projectId: string): Promise<CoverageAnalysis> {
    const response = await this.axios.get<CoverageAnalysis>(
      `/projects/${projectId}/insurance/analysis`
    );
    return response.data;
  }

  // ========================================================================
  // Monitoring & Alerts
  // ========================================================================

  async getCurrentReadings(projectId: string, equipmentId?: string): Promise<CurrentReading[]> {
    const response = await this.axios.get<CurrentReading[]>(
      `/projects/${projectId}/monitoring/current`,
      { params: { equipmentId } }
    );
    return response.data;
  }

  async getAlerts(
    projectId: string,
    params?: { severity?: string; acknowledged?: boolean; limit?: number }
  ): Promise<Alert[]> {
    const response = await this.axios.get<Alert[]>(
      `/projects/${projectId}/alerts`,
      { params }
    );
    return response.data;
  }

  async acknowledgeAlert(projectId: string, alertId: string): Promise<void> {
    await this.axios.post(`/projects/${projectId}/alerts/${alertId}/acknowledge`);
  }

  async getDashboard(projectId: string): Promise<DashboardData> {
    const response = await this.axios.get<DashboardData>(
      `/projects/${projectId}/dashboard`
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

  // ========================================================================
  // Validation
  // ========================================================================

  validateProject(project: WIACryoAssetProject): ValidationResult {
    const errors: any[] = [];

    if (!project.standard || project.standard !== 'WIA-CRYO-ASSET') {
      errors.push({
        path: 'standard',
        message: 'Standard must be "WIA-CRYO-ASSET"',
      });
    }

    if (!project.metadata?.id) {
      errors.push({ path: 'metadata.id', message: 'Project ID is required' });
    }

    if (!project.storage?.facilities?.length) {
      errors.push({
        path: 'storage.facilities',
        message: 'At least one storage facility is required',
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

export interface AssetTransfer {
  toFacilityId: string;
  toEquipmentId: string;
  toPosition?: string;
  scheduledDate: string;
  transportMethod: string;
  handler: string;
  notes?: string;
}

export interface TransferResult {
  id: string;
  status: 'scheduled' | 'in-transit' | 'completed' | 'failed';
  scheduledDate: string;
  completedDate?: string;
}

export interface DisposalRequest {
  method: 'destruction' | 'donation' | 'research' | 'return';
  reason: string;
  authorization: string;
  scheduledDate: string;
  handler: string;
  documentation?: string[];
}

export interface DisposalResult {
  id: string;
  status: 'scheduled' | 'completed';
  completedDate?: string;
  certificate?: string;
}

export interface EquipmentReading {
  timestamp: string;
  parameter: string;
  value: number;
  unit: string;
  status: 'normal' | 'warning' | 'critical';
}

export interface StorageAssignmentRequest {
  facilityId: string;
  equipmentId: string;
  rackId?: string;
  boxId?: string;
  position: string;
}

export interface StorageAssignmentResult {
  assetId: string;
  assignment: { facilityId: string; equipmentId: string; position: string };
  assignedAt: string;
}

export interface InventoryReport {
  facilityId: string;
  equipmentId?: string;
  totalAssets: number;
  byType: Record<string, number>;
  byStatus: Record<string, number>;
  utilization: number;
  generatedAt: string;
}

export interface ValuationRequest {
  scope: 'all' | 'facility' | 'asset';
  targetId?: string;
  method?: string;
}

export interface ValuationResult {
  id: string;
  status: 'completed' | 'pending';
  totalValue: number;
  currency: string;
  completedAt?: string;
}

export interface AssetValuation {
  assetId: string;
  currentValue: number;
  currency: string;
  method: string;
  valuedAt: string;
  history: { date: string; value: number }[];
}

export interface ClaimRequest {
  assetIds: string[];
  incidentDate: string;
  description: string;
  estimatedLoss: number;
  documentation: string[];
}

export interface ClaimResult {
  claimId: string;
  status: 'filed' | 'processing';
  filedAt: string;
  reference: string;
}

export interface CoverageAnalysis {
  totalAssetValue: number;
  totalCoverage: number;
  coverageRatio: number;
  gaps: { type: string; exposure: number }[];
  recommendations: string[];
}

export interface CurrentReading {
  equipmentId: string;
  name: string;
  temperature: number;
  level?: number;
  status: string;
  lastUpdated: string;
}

export interface Alert {
  id: string;
  equipmentId: string;
  type: string;
  severity: 'warning' | 'critical';
  message: string;
  timestamp: string;
  acknowledged: boolean;
}

export interface DashboardData {
  summary: { totalAssets: number; totalValue: number; facilities: number };
  storage: { utilization: number; critical: number };
  monitoring: { activeAlerts: number; temperature: { min: number; max: number; avg: number } };
  recent: { type: string; description: string; timestamp: string }[];
}

export interface ReportOptions {
  format?: 'pdf' | 'excel' | 'json';
  period?: { start: string; end: string };
  includeValuation?: boolean;
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

export function generateBarcode(): string {
  return 'CRY' + Date.now().toString(36).toUpperCase() + Math.random().toString(36).substring(2, 6).toUpperCase();
}

export function createMinimalProject(name: string, organization: string): WIACryoAssetProject {
  return {
    standard: 'WIA-CRYO-ASSET',
    version: '1.0.0',
    metadata: {
      id: generateUUID(),
      name,
      type: 'biological',
      organization: { name: organization, type: 'facility', country: 'US' },
      jurisdiction: 'US',
      createdAt: new Date().toISOString(),
      status: 'active',
    },
    assets: [],
    storage: { facilities: [], equipment: [], protocols: [], capacity: { total: 0, used: 0, available: 0, reserved: 0 } },
    valuation: { methodology: { primary: 'cost-based', factors: [], documentation: '' }, schedule: { frequency: 'annual', lastValuation: '', nextValuation: '' }, history: [] },
    custody: { custodian: { id: '', name: organization, type: 'facility', licenses: [], contact: { name: '', email: '' }, insurance: '' }, terms: { startDate: new Date().toISOString().split('T')[0], terminationConditions: [], fees: [] }, responsibilities: [], reporting: { inventory: { frequency: 'monthly', format: 'pdf' }, monitoring: { frequency: 'daily', format: 'json' }, financial: { frequency: 'quarterly', format: 'pdf' } } },
    insurance: { policies: [], totalCoverage: 0, currency: 'USD' },
    monitoring: { realTime: { enabled: true, parameters: [], dashboard: '', retention: '1 year' }, alerts: { channels: [], escalation: { levels: [], timeout: 300 }, acknowledgement: 'required' }, reporting: { automated: [], onDemand: [] }, audit: { frequency: 'annual', scope: [], auditor: 'external' } },
  };
}

// ============================================================================
// Exports
// ============================================================================

export default {
  WIACryoAssetClient,
  generateUUID,
  generateBarcode,
  createMinimalProject,
};
