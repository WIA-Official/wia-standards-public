/**
 * WIA Cryo Facility Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig,
  WIACryoFacilityProject,
  ProjectResponse,
  ValidationResult,
  PaginatedResponse,
  CryoStorageEquipment,
  EquipmentStatus,
  FacilityZone,
  SafetyEquipment,
  StandardProcedure,
} from './types';

// ============================================================================
// WIA Cryo Facility Client
// ============================================================================

export class WIACryoFacilityClient {
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

  async createProject(project: WIACryoFacilityProject): Promise<ProjectResponse> {
    const response = await this.axios.post<ProjectResponse>('/projects', project);
    return response.data;
  }

  async getProject(id: string): Promise<WIACryoFacilityProject> {
    const response = await this.axios.get<WIACryoFacilityProject>(`/projects/${id}`);
    return response.data;
  }

  async listProjects(params?: {
    type?: string;
    status?: string;
    limit?: number;
    offset?: number;
  }): Promise<PaginatedResponse<ProjectResponse>> {
    const response = await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', { params });
    return response.data;
  }

  async updateProject(id: string, updates: Partial<WIACryoFacilityProject>): Promise<ProjectResponse> {
    const response = await this.axios.put<ProjectResponse>(`/projects/${id}`, updates);
    return response.data;
  }

  // ========================================================================
  // Equipment Management
  // ========================================================================

  async listEquipment(projectId: string, params?: { type?: string; status?: string; zone?: string }): Promise<CryoStorageEquipment[]> {
    const response = await this.axios.get<CryoStorageEquipment[]>(`/projects/${projectId}/equipment`, { params });
    return response.data;
  }

  async getEquipment(projectId: string, equipmentId: string): Promise<CryoStorageEquipment> {
    const response = await this.axios.get<CryoStorageEquipment>(`/projects/${projectId}/equipment/${equipmentId}`);
    return response.data;
  }

  async addEquipment(projectId: string, equipment: Partial<CryoStorageEquipment>): Promise<CryoStorageEquipment> {
    const response = await this.axios.post<CryoStorageEquipment>(`/projects/${projectId}/equipment`, equipment);
    return response.data;
  }

  async updateEquipmentStatus(projectId: string, equipmentId: string, status: EquipmentStatus, notes?: string): Promise<CryoStorageEquipment> {
    const response = await this.axios.put<CryoStorageEquipment>(`/projects/${projectId}/equipment/${equipmentId}/status`, { status, notes });
    return response.data;
  }

  async getEquipmentReadings(projectId: string, equipmentId: string, params?: { start?: string; end?: string }): Promise<EquipmentReading[]> {
    const response = await this.axios.get<EquipmentReading[]>(`/projects/${projectId}/equipment/${equipmentId}/readings`, { params });
    return response.data;
  }

  async recordMaintenance(projectId: string, equipmentId: string, maintenance: MaintenanceEntry): Promise<MaintenanceResult> {
    const response = await this.axios.post<MaintenanceResult>(`/projects/${projectId}/equipment/${equipmentId}/maintenance`, maintenance);
    return response.data;
  }

  // ========================================================================
  // Zone Management
  // ========================================================================

  async listZones(projectId: string): Promise<FacilityZone[]> {
    const response = await this.axios.get<FacilityZone[]>(`/projects/${projectId}/zones`);
    return response.data;
  }

  async getZone(projectId: string, zoneId: string): Promise<FacilityZone> {
    const response = await this.axios.get<FacilityZone>(`/projects/${projectId}/zones/${zoneId}`);
    return response.data;
  }

  async getZoneEnvironmental(projectId: string, zoneId: string): Promise<EnvironmentalData> {
    const response = await this.axios.get<EnvironmentalData>(`/projects/${projectId}/zones/${zoneId}/environmental`);
    return response.data;
  }

  // ========================================================================
  // Safety Management
  // ========================================================================

  async listSafetyEquipment(projectId: string): Promise<SafetyEquipment[]> {
    const response = await this.axios.get<SafetyEquipment[]>(`/projects/${projectId}/safety/equipment`);
    return response.data;
  }

  async recordSafetyInspection(projectId: string, inspection: SafetyInspection): Promise<InspectionResult> {
    const response = await this.axios.post<InspectionResult>(`/projects/${projectId}/safety/inspections`, inspection);
    return response.data;
  }

  async reportIncident(projectId: string, incident: IncidentReport): Promise<IncidentResult> {
    const response = await this.axios.post<IncidentResult>(`/projects/${projectId}/safety/incidents`, incident);
    return response.data;
  }

  async getIncidents(projectId: string, params?: { severity?: string; start?: string; end?: string }): Promise<Incident[]> {
    const response = await this.axios.get<Incident[]>(`/projects/${projectId}/safety/incidents`, { params });
    return response.data;
  }

  // ========================================================================
  // Procedures & Quality
  // ========================================================================

  async listProcedures(projectId: string, params?: { category?: string }): Promise<StandardProcedure[]> {
    const response = await this.axios.get<StandardProcedure[]>(`/projects/${projectId}/procedures`, { params });
    return response.data;
  }

  async getProcedure(projectId: string, procedureId: string): Promise<StandardProcedure> {
    const response = await this.axios.get<StandardProcedure>(`/projects/${projectId}/procedures/${procedureId}`);
    return response.data;
  }

  async recordDeviation(projectId: string, deviation: DeviationReport): Promise<DeviationResult> {
    const response = await this.axios.post<DeviationResult>(`/projects/${projectId}/quality/deviations`, deviation);
    return response.data;
  }

  async getDeviations(projectId: string, params?: { status?: string }): Promise<Deviation[]> {
    const response = await this.axios.get<Deviation[]>(`/projects/${projectId}/quality/deviations`, { params });
    return response.data;
  }

  // ========================================================================
  // Monitoring & Alerts
  // ========================================================================

  async getCurrentReadings(projectId: string): Promise<CurrentReadings> {
    const response = await this.axios.get<CurrentReadings>(`/projects/${projectId}/monitoring/current`);
    return response.data;
  }

  async getAlerts(projectId: string, params?: { severity?: string; acknowledged?: boolean }): Promise<Alert[]> {
    const response = await this.axios.get<Alert[]>(`/projects/${projectId}/alerts`, { params });
    return response.data;
  }

  async acknowledgeAlert(projectId: string, alertId: string): Promise<void> {
    await this.axios.post(`/projects/${projectId}/alerts/${alertId}/acknowledge`);
  }

  async getDashboard(projectId: string): Promise<DashboardData> {
    const response = await this.axios.get<DashboardData>(`/projects/${projectId}/dashboard`);
    return response.data;
  }

  // ========================================================================
  // Validation
  // ========================================================================

  validateProject(project: WIACryoFacilityProject): ValidationResult {
    const errors: any[] = [];
    if (!project.standard || project.standard !== 'WIA-CRYO-FACILITY') {
      errors.push({ path: 'standard', message: 'Standard must be "WIA-CRYO-FACILITY"' });
    }
    if (!project.metadata?.id) {
      errors.push({ path: 'metadata.id', message: 'Project ID is required' });
    }
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

// ============================================================================
// Supporting Types
// ============================================================================

export interface EquipmentReading { timestamp: string; parameter: string; value: number; unit: string; status: string; }
export interface MaintenanceEntry { type: string; technician: string; description: string; parts?: string[]; date: string; }
export interface MaintenanceResult { id: string; recorded: boolean; nextDue: string; }
export interface EnvironmentalData { temperature: number; humidity: number; pressure?: number; lastUpdated: string; }
export interface SafetyInspection { equipmentIds: string[]; inspector: string; findings: { id: string; status: string; notes?: string }[]; date: string; }
export interface InspectionResult { id: string; recorded: boolean; followUpRequired: boolean; }
export interface IncidentReport { type: string; severity: string; location: string; description: string; injuries?: number; reporter: string; }
export interface IncidentResult { id: string; status: string; reportedAt: string; }
export interface Incident extends IncidentReport { id: string; status: string; reportedAt: string; resolvedAt?: string; }
export interface DeviationReport { type: string; description: string; impact: string; reporter: string; }
export interface DeviationResult { id: string; status: string; createdAt: string; }
export interface Deviation extends DeviationReport { id: string; status: string; createdAt: string; resolvedAt?: string; }
export interface CurrentReadings { equipment: { id: string; name: string; temperature: number; level?: number; status: string }[]; zones: { id: string; name: string; temperature: number; humidity: number }[]; lastUpdated: string; }
export interface Alert { id: string; equipmentId?: string; zoneId?: string; type: string; severity: string; message: string; timestamp: string; acknowledged: boolean; }
export interface DashboardData { status: string; equipment: { total: number; operational: number; warning: number; critical: number }; capacity: { total: number; used: number; available: number }; alerts: { active: number; critical: number }; environmental: { inSpec: number; outOfSpec: number }; }

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

export function createMinimalProject(name: string, organization: string): WIACryoFacilityProject {
  return {
    standard: 'WIA-CRYO-FACILITY',
    version: '1.0.0',
    metadata: { id: generateUUID(), name, type: 'biobank', location: { address: '', city: '', country: '', postalCode: '', coordinates: { latitude: 0, longitude: 0 }, timezone: 'UTC' }, organization: { name: organization, type: 'facility', contact: { name: '', email: '', phone: '' } }, licenses: [], certifications: [], createdAt: new Date().toISOString(), status: 'operational' },
    facility: { layout: { totalArea: 0, storageArea: 0, processingArea: 0, officeArea: 0, unit: 'sqm', floors: [] }, zones: [], capacity: { storage: { tanks: 0, totalSpecimens: 0, currentSpecimens: 0, utilizationPercent: 0 }, processing: { daily: 0, weekly: 0, unit: 'specimens' }, personnel: { maximum: 0, current: 0, shifts: [] } }, infrastructure: { power: { mainSupply: '', capacity: 0, unit: 'kW', redundancy: true, ups: { capacity: 0, runtime: 0, units: 0 } }, hvac: { type: '', zones: 0, redundancy: true, filtration: '' }, gasSupply: { liquidNitrogen: { bulkTank: { capacity: 0, unit: 'liters' }, deliverySchedule: '', backupSupply: true, monitoring: true } }, backup: { generator: { type: '', capacity: 0, fuelType: '', fuelCapacity: 0, autoStart: true, testingSchedule: '' }, alternateStorage: true, disasterRecovery: '' }, networking: { type: '', redundancy: true, bandwidth: 0, security: [] } }, access: { system: '', methods: ['keycard', 'biometric'], logging: true, retention: '5 years' } },
    equipment: { cryoStorage: [], processing: [], monitoring: [], safety: [], maintenance: { preventive: [], predictive: true, contracts: [] } },
    operations: { sops: [], workflows: [], training: { requirements: [], records: true, refreshInterval: 'annual', competencyAssessment: true }, documentation: { format: 'electronic', storage: 'cloud', retention: '10 years', version: true, audit: true } },
    safety: { policies: [], hazards: [], emergency: { contacts: [], procedures: [], drills: { type: '', frequency: '', lastDrill: '', nextDrill: '' }, equipment: [] }, incidents: { reporting: '', investigation: '', corrective: '', tracking: true }, ppe: { zones: [], training: true, inspection: 'monthly' } },
    quality: { system: 'ISO 9001', audits: { internal: { frequency: 'annual', scope: [] }, external: { frequency: 'annual', bodies: [] }, tracking: true }, deviations: { categories: [], investigation: '', timeline: '' }, capa: { enabled: true, workflow: '', tracking: true, effectiveness: '' } },
    environmental: { monitoring: { parameters: [], frequency: 'continuous', logging: true }, controls: { hvac: true, humidification: true, filtration: '', pressurization: true }, alerts: { enabled: true, thresholds: [], notifications: [] } },
    monitoring: { realtime: { enabled: true, dashboard: '', refresh: 30 }, alerts: { channels: [], escalation: { levels: 3, timeout: 300 }, acknowledgement: 'required' }, reporting: { automated: [], onDemand: [] }, integration: { lims: true, buildingManagement: true, external: [] } },
  };
}

export default { WIACryoFacilityClient, generateUUID, createMinimalProject };
