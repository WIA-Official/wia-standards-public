/**
 * WIA Cyber Weapon Defense Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type { APIConfig, WIACyberWeaponDefenseProject, ProjectResponse, ValidationResult, PaginatedResponse, ThreatActor, CyberWeapon, ThreatIndicator, ThreatCampaign, ResponsePlaybook, SecurityControl } from './types';

// ============================================================================
// WIA Cyber Weapon Defense Client
// ============================================================================

export class WIACyberWeaponDefenseClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: { 'Content-Type': 'application/json', ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }) },
    });
  }

  // Project Management
  async createProject(project: WIACyberWeaponDefenseProject): Promise<ProjectResponse> { return (await this.axios.post<ProjectResponse>('/projects', project)).data; }
  async getProject(id: string): Promise<WIACyberWeaponDefenseProject> { return (await this.axios.get<WIACyberWeaponDefenseProject>(`/projects/${id}`)).data; }
  async listProjects(params?: { status?: string; classification?: string; limit?: number }): Promise<PaginatedResponse<ProjectResponse>> { return (await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', { params })).data; }
  async updateProject(id: string, updates: Partial<WIACyberWeaponDefenseProject>): Promise<ProjectResponse> { return (await this.axios.put<ProjectResponse>(`/projects/${id}`, updates)).data; }

  // Threat Intelligence
  async addThreatActor(projectId: string, actor: Partial<ThreatActor>): Promise<ThreatActor> { return (await this.axios.post<ThreatActor>(`/projects/${projectId}/actors`, actor)).data; }
  async listThreatActors(projectId: string, params?: { type?: string }): Promise<ThreatActor[]> { return (await this.axios.get<ThreatActor[]>(`/projects/${projectId}/actors`, { params })).data; }
  async getThreatActor(projectId: string, actorId: string): Promise<ThreatActor> { return (await this.axios.get<ThreatActor>(`/projects/${projectId}/actors/${actorId}`)).data; }

  async addCyberWeapon(projectId: string, weapon: Partial<CyberWeapon>): Promise<CyberWeapon> { return (await this.axios.post<CyberWeapon>(`/projects/${projectId}/weapons`, weapon)).data; }
  async listCyberWeapons(projectId: string, params?: { type?: string; category?: string }): Promise<CyberWeapon[]> { return (await this.axios.get<CyberWeapon[]>(`/projects/${projectId}/weapons`, { params })).data; }
  async getCyberWeapon(projectId: string, weaponId: string): Promise<CyberWeapon> { return (await this.axios.get<CyberWeapon>(`/projects/${projectId}/weapons/${weaponId}`)).data; }

  async addIndicator(projectId: string, indicator: Partial<ThreatIndicator>): Promise<ThreatIndicator> { return (await this.axios.post<ThreatIndicator>(`/projects/${projectId}/indicators`, indicator)).data; }
  async listIndicators(projectId: string, params?: { type?: string; limit?: number }): Promise<PaginatedResponse<ThreatIndicator>> { return (await this.axios.get<PaginatedResponse<ThreatIndicator>>(`/projects/${projectId}/indicators`, { params })).data; }
  async searchIndicators(projectId: string, query: IndicatorSearch): Promise<ThreatIndicator[]> { return (await this.axios.post<ThreatIndicator[]>(`/projects/${projectId}/indicators/search`, query)).data; }

  async listCampaigns(projectId: string, params?: { status?: string }): Promise<ThreatCampaign[]> { return (await this.axios.get<ThreatCampaign[]>(`/projects/${projectId}/campaigns`, { params })).data; }
  async getCampaign(projectId: string, campaignId: string): Promise<ThreatCampaign> { return (await this.axios.get<ThreatCampaign>(`/projects/${projectId}/campaigns/${campaignId}`)).data; }

  // Defense Controls
  async listControls(projectId: string, params?: { type?: string; status?: string }): Promise<SecurityControl[]> { return (await this.axios.get<SecurityControl[]>(`/projects/${projectId}/controls`, { params })).data; }
  async addControl(projectId: string, control: Partial<SecurityControl>): Promise<SecurityControl> { return (await this.axios.post<SecurityControl>(`/projects/${projectId}/controls`, control)).data; }
  async assessControl(projectId: string, controlId: string, assessment: ControlAssessment): Promise<SecurityControl> { return (await this.axios.post<SecurityControl>(`/projects/${projectId}/controls/${controlId}/assess`, assessment)).data; }

  // Detection & Response
  async reportDetection(projectId: string, detection: DetectionReport): Promise<DetectionResult> { return (await this.axios.post<DetectionResult>(`/projects/${projectId}/detections`, detection)).data; }
  async listDetections(projectId: string, params?: { severity?: string; start?: string; end?: string }): Promise<Detection[]> { return (await this.axios.get<Detection[]>(`/projects/${projectId}/detections`, { params })).data; }
  async correlateEvents(projectId: string, events: string[]): Promise<CorrelationResult> { return (await this.axios.post<CorrelationResult>(`/projects/${projectId}/correlate`, { events })).data; }

  async listPlaybooks(projectId: string): Promise<ResponsePlaybook[]> { return (await this.axios.get<ResponsePlaybook[]>(`/projects/${projectId}/playbooks`)).data; }
  async executePlaybook(projectId: string, playbookId: string, context: PlaybookContext): Promise<ExecutionResult> { return (await this.axios.post<ExecutionResult>(`/projects/${projectId}/playbooks/${playbookId}/execute`, context)).data; }

  async createIncident(projectId: string, incident: IncidentReport): Promise<Incident> { return (await this.axios.post<Incident>(`/projects/${projectId}/incidents`, incident)).data; }
  async listIncidents(projectId: string, params?: { status?: string; severity?: string }): Promise<Incident[]> { return (await this.axios.get<Incident[]>(`/projects/${projectId}/incidents`, { params })).data; }
  async updateIncident(projectId: string, incidentId: string, update: IncidentUpdate): Promise<Incident> { return (await this.axios.put<Incident>(`/projects/${projectId}/incidents/${incidentId}`, update)).data; }

  // Dashboard & Reporting
  async getDashboard(projectId: string): Promise<DashboardData> { return (await this.axios.get<DashboardData>(`/projects/${projectId}/dashboard`)).data; }
  async generateReport(projectId: string, type: string, options?: ReportOptions): Promise<ReportResult> { return (await this.axios.post<ReportResult>(`/projects/${projectId}/reports`, { type, options })).data; }

  // Validation
  validateProject(project: WIACyberWeaponDefenseProject): ValidationResult {
    const errors: any[] = [];
    if (!project.standard || project.standard !== 'WIA-CYBER-WEAPON-DEFENSE') errors.push({ path: 'standard', message: 'Standard must be "WIA-CYBER-WEAPON-DEFENSE"' });
    if (!project.metadata?.id) errors.push({ path: 'metadata.id', message: 'Project ID is required' });
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

// ============================================================================
// Supporting Types
// ============================================================================

export interface IndicatorSearch { type?: string; value?: string; dateRange?: { start: string; end: string }; campaigns?: string[]; }
export interface ControlAssessment { effectiveness: number; findings: string[]; recommendations: string[]; assessor: string; }
export interface DetectionReport { source: string; type: string; severity: string; indicators: string[]; description: string; rawData?: unknown; }
export interface DetectionResult { id: string; status: 'new' | 'correlated' | 'escalated'; timestamp: string; }
export interface Detection { id: string; source: string; type: string; severity: string; timestamp: string; status: string; indicators: string[]; }
export interface CorrelationResult { correlationId: string; events: string[]; confidence: number; relatedCampaigns?: string[]; relatedActors?: string[]; }
export interface PlaybookContext { detectionId?: string; incidentId?: string; indicators?: string[]; notes?: string; }
export interface ExecutionResult { executionId: string; status: 'started' | 'completed' | 'failed'; steps: { step: number; status: string; output?: string }[]; }
export interface IncidentReport { title: string; severity: string; type: string; detections: string[]; description: string; impact?: string; }
export interface Incident extends IncidentReport { id: string; status: 'open' | 'investigating' | 'contained' | 'eradicated' | 'recovered' | 'closed'; createdAt: string; timeline: { timestamp: string; action: string; actor: string }[]; }
export interface IncidentUpdate { status?: string; actions?: string; notes?: string; }
export interface DashboardData { threats: { actors: number; weapons: number; campaigns: number; indicators: number }; detections: { last24h: number; byType: Record<string, number>; bySeverity: Record<string, number> }; incidents: { open: number; bySeverity: Record<string, number> }; controls: { implemented: number; effectiveness: number }; }
export interface ReportOptions { format?: 'pdf' | 'json'; period?: { start: string; end: string }; classification?: string; }
export interface ReportResult { id: string; status: 'queued' | 'processing' | 'completed'; url?: string; }

// ============================================================================
// Utility Functions
// ============================================================================

export function generateUUID(): string { return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => { const r = (Math.random() * 16) | 0; return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16); }); }

export function createMinimalProject(name: string, organization: string): WIACyberWeaponDefenseProject {
  return {
    standard: 'WIA-CYBER-WEAPON-DEFENSE', version: '1.0.0',
    metadata: { id: generateUUID(), name, classification: 'confidential', organization: { name: organization, type: 'enterprise', country: 'US', contact: { name: '', email: '' } }, scope: { assets: [], networks: [], criticality: 'high' }, createdAt: new Date().toISOString(), status: 'active' },
    threats: { sources: [], actors: [], weapons: [], campaigns: [], indicators: [] },
    defenses: { layers: [], controls: [], technologies: [], resilience: { redundancy: [], recovery: { rto: 4, rpo: 1, procedures: [], testing: '' }, continuity: { scenarios: [], procedures: [], resources: [], testing: '' }, isolation: { segmentation: [], airGap: false, killSwitch: true, quarantine: '' } } },
    detection: { capabilities: [], sensors: [], analytics: [], correlation: { rules: 0, sources: [], latency: '', automation: 0 }, hunting: { frequency: '', methodologies: [], tools: [], team: '' } },
    response: { playbooks: [], team: { structure: '', roles: [], onCall: '', training: '' }, escalation: { levels: [] }, communication: { internal: { channels: [], templates: [] }, external: { stakeholders: [], protocols: [] }, authorities: { agencies: [], procedures: [] } }, recovery: { containment: [], eradication: [], recovery: [], validation: [] } },
    attribution: { methodology: { approaches: [], standards: [], legal: '' }, evidence: { forensics: [], chain: '', preservation: '' }, analysis: { techniques: [], correlation: [], validation: '' }, confidence: { levels: [], reporting: '' } },
    compliance: { regulations: [], standards: [], reporting: { mandatory: [], voluntary: [] }, audits: { internal: { frequency: '', scope: [] }, external: { frequency: '', auditor: '' } } },
  };
}

export default { WIACyberWeaponDefenseClient, generateUUID, createMinimalProject };
