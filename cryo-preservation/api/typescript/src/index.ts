/**
 * WIA Cryo Preservation Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type { APIConfig, WIACryoPreservationProject, ProjectResponse, ValidationResult, PaginatedResponse, Specimen, PreservationProtocol, StorageEquipment, QualityAssessment, SpecimenEvent } from './types';

// ============================================================================
// WIA Cryo Preservation Client
// ============================================================================

export class WIACryoPreservationClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: { 'Content-Type': 'application/json', ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }) },
    });
  }

  // Project Management
  async createProject(project: WIACryoPreservationProject): Promise<ProjectResponse> { return (await this.axios.post<ProjectResponse>('/projects', project)).data; }
  async getProject(id: string): Promise<WIACryoPreservationProject> { return (await this.axios.get<WIACryoPreservationProject>(`/projects/${id}`)).data; }
  async listProjects(params?: { status?: string; type?: string; limit?: number; offset?: number }): Promise<PaginatedResponse<ProjectResponse>> { return (await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', { params })).data; }
  async updateProject(id: string, updates: Partial<WIACryoPreservationProject>): Promise<ProjectResponse> { return (await this.axios.put<ProjectResponse>(`/projects/${id}`, updates)).data; }

  // Specimen Management
  async preserveSpecimen(projectId: string, specimen: Partial<Specimen>): Promise<Specimen> { return (await this.axios.post<Specimen>(`/projects/${projectId}/specimens`, specimen)).data; }
  async getSpecimen(projectId: string, specimenId: string): Promise<Specimen> { return (await this.axios.get<Specimen>(`/projects/${projectId}/specimens/${specimenId}`)).data; }
  async listSpecimens(projectId: string, params?: { type?: string; status?: string; limit?: number }): Promise<PaginatedResponse<Specimen>> { return (await this.axios.get<PaginatedResponse<Specimen>>(`/projects/${projectId}/specimens`, { params })).data; }
  async updateSpecimen(projectId: string, specimenId: string, updates: Partial<Specimen>): Promise<Specimen> { return (await this.axios.put<Specimen>(`/projects/${projectId}/specimens/${specimenId}`, updates)).data; }
  async updateSpecimenLocation(projectId: string, specimenId: string, location: LocationUpdate): Promise<Specimen> { return (await this.axios.put<Specimen>(`/projects/${projectId}/specimens/${specimenId}/location`, location)).data; }
  async recordQuality(projectId: string, specimenId: string, assessment: QualityAssessment): Promise<Specimen> { return (await this.axios.post<Specimen>(`/projects/${projectId}/specimens/${specimenId}/quality`, assessment)).data; }
  async getSpecimenHistory(projectId: string, specimenId: string): Promise<SpecimenEvent[]> { return (await this.axios.get<SpecimenEvent[]>(`/projects/${projectId}/specimens/${specimenId}/history`)).data; }

  // Thawing & Distribution
  async initiateThaw(projectId: string, specimenId: string, thaw: ThawRequest): Promise<ThawResult> { return (await this.axios.post<ThawResult>(`/projects/${projectId}/specimens/${specimenId}/thaw`, thaw)).data; }
  async recordThawResult(projectId: string, specimenId: string, result: ThawResultRecord): Promise<Specimen> { return (await this.axios.post<Specimen>(`/projects/${projectId}/specimens/${specimenId}/thaw/result`, result)).data; }
  async distributeSpecimen(projectId: string, specimenId: string, distribution: DistributionRequest): Promise<DistributionResult> { return (await this.axios.post<DistributionResult>(`/projects/${projectId}/specimens/${specimenId}/distribute`, distribution)).data; }

  // Protocol Management
  async listProtocols(projectId: string, params?: { type?: string; status?: string }): Promise<PreservationProtocol[]> { return (await this.axios.get<PreservationProtocol[]>(`/projects/${projectId}/protocols`, { params })).data; }
  async getProtocol(projectId: string, protocolId: string): Promise<PreservationProtocol> { return (await this.axios.get<PreservationProtocol>(`/projects/${projectId}/protocols/${protocolId}`)).data; }
  async createProtocol(projectId: string, protocol: Partial<PreservationProtocol>): Promise<PreservationProtocol> { return (await this.axios.post<PreservationProtocol>(`/projects/${projectId}/protocols`, protocol)).data; }

  // Storage Management
  async listEquipment(projectId: string): Promise<StorageEquipment[]> { return (await this.axios.get<StorageEquipment[]>(`/projects/${projectId}/equipment`)).data; }
  async getEquipmentReadings(projectId: string, equipmentId: string, params?: { start?: string; end?: string }): Promise<EquipmentReading[]> { return (await this.axios.get<EquipmentReading[]>(`/projects/${projectId}/equipment/${equipmentId}/readings`, { params })).data; }
  async getInventory(projectId: string, equipmentId?: string): Promise<InventoryReport> { return (await this.axios.get<InventoryReport>(`/projects/${projectId}/inventory`, { params: { equipmentId } })).data; }

  // Monitoring & Alerts
  async getCurrentReadings(projectId: string): Promise<CurrentReadings> { return (await this.axios.get<CurrentReadings>(`/projects/${projectId}/monitoring/current`)).data; }
  async getAlerts(projectId: string, params?: { severity?: string; acknowledged?: boolean }): Promise<Alert[]> { return (await this.axios.get<Alert[]>(`/projects/${projectId}/alerts`, { params })).data; }
  async acknowledgeAlert(projectId: string, alertId: string): Promise<void> { await this.axios.post(`/projects/${projectId}/alerts/${alertId}/acknowledge`); }
  async getDashboard(projectId: string): Promise<DashboardData> { return (await this.axios.get<DashboardData>(`/projects/${projectId}/dashboard`)).data; }

  // Validation
  validateProject(project: WIACryoPreservationProject): ValidationResult {
    const errors: any[] = [];
    if (!project.standard || project.standard !== 'WIA-CRYO-PRESERVATION') errors.push({ path: 'standard', message: 'Standard must be "WIA-CRYO-PRESERVATION"' });
    if (!project.metadata?.id) errors.push({ path: 'metadata.id', message: 'Project ID is required' });
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

// ============================================================================
// Supporting Types
// ============================================================================

export interface LocationUpdate { facilityId: string; tankId: string; position: string; reason: string; }
export interface ThawRequest { protocol: string; purpose: string; requestor: string; scheduledTime?: string; }
export interface ThawResult { id: string; status: 'scheduled' | 'in-progress' | 'completed'; scheduledTime: string; }
export interface ThawResultRecord { startTime: string; endTime: string; operator: string; viability?: number; notes?: string; }
export interface DistributionRequest { recipient: string; purpose: string; quantity?: number; shipping: { method: string; container: string }; }
export interface DistributionResult { id: string; trackingNumber: string; shippedAt: string; estimatedArrival: string; }
export interface EquipmentReading { timestamp: string; temperature: number; level?: number; status: string; }
export interface InventoryReport { total: number; byType: Record<string, number>; byStatus: Record<string, number>; byFacility: Record<string, number>; generatedAt: string; }
export interface CurrentReadings { equipment: { id: string; name: string; temperature: number; level?: number; status: string }[]; lastUpdated: string; }
export interface Alert { id: string; equipmentId: string; type: string; severity: string; message: string; timestamp: string; acknowledged: boolean; }
export interface DashboardData { specimens: { total: number; byType: Record<string, number>; byStatus: Record<string, number> }; storage: { utilization: number; critical: number }; quality: { pendingQA: number; recentViability: number }; alerts: { active: number; critical: number }; }

// ============================================================================
// Utility Functions
// ============================================================================

export function generateUUID(): string { return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => { const r = (Math.random() * 16) | 0; return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16); }); }

export function createMinimalProject(name: string, organization: string): WIACryoPreservationProject {
  return {
    standard: 'WIA-CRYO-PRESERVATION', version: '1.0.0',
    metadata: { id: generateUUID(), name, type: 'cell', organization: { name: organization, type: 'facility', country: 'US', contact: { name: '', email: '' } }, facility: { id: '', name: '', location: '', certifications: [] }, createdAt: new Date().toISOString(), status: 'active' },
    specimens: [],
    protocols: [],
    storage: { facilities: [], equipment: [], logistics: { shipping: { containers: [], carriers: [], monitoring: true, documentation: [] }, receiving: { verification: [], quarantine: true, documentation: [] }, chain: { required: true, tracking: 'barcode', documentation: '' } }, inventory: { tracking: 'real-time', audits: 'annual', reconciliation: 'monthly' } },
    quality: { standards: [], procedures: [], testing: { viability: [], sterility: [], identity: [] }, deviations: { categories: [], investigation: '', timeline: '' }, capa: { enabled: true, workflow: '', tracking: true } },
    monitoring: { parameters: [], alerts: { channels: [], escalation: { levels: 3, timeout: 300 } }, logging: { retention: '10 years', backup: 'daily', integrity: true }, reporting: { automated: [], onDemand: [] } },
    recovery: { thawing: [], validation: { required: true, tests: [], acceptance: '', timeline: '' }, distribution: { packaging: '', documentation: [], tracking: true } },
  };
}

export default { WIACryoPreservationClient, generateUUID, createMinimalProject };
