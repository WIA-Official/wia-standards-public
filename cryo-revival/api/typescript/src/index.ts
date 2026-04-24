/**
 * WIA Cryo Revival Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type { APIConfig, WIACryoRevivalProject, ProjectResponse, ValidationResult, PaginatedResponse, RevivalSubject, RevivalProtocol, SubjectEvent, HealthAssessment, RevivalOutcome } from './types';

// ============================================================================
// WIA Cryo Revival Client
// ============================================================================

export class WIACryoRevivalClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: { 'Content-Type': 'application/json', ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }) },
    });
  }

  // Project Management
  async createProject(project: WIACryoRevivalProject): Promise<ProjectResponse> { return (await this.axios.post<ProjectResponse>('/projects', project)).data; }
  async getProject(id: string): Promise<WIACryoRevivalProject> { return (await this.axios.get<WIACryoRevivalProject>(`/projects/${id}`)).data; }
  async listProjects(params?: { status?: string; type?: string; limit?: number }): Promise<PaginatedResponse<ProjectResponse>> { return (await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', { params })).data; }
  async updateProject(id: string, updates: Partial<WIACryoRevivalProject>): Promise<ProjectResponse> { return (await this.axios.put<ProjectResponse>(`/projects/${id}`, updates)).data; }

  // Subject Management
  async registerSubject(projectId: string, subject: Partial<RevivalSubject>): Promise<RevivalSubject> { return (await this.axios.post<RevivalSubject>(`/projects/${projectId}/subjects`, subject)).data; }
  async getSubject(projectId: string, subjectId: string): Promise<RevivalSubject> { return (await this.axios.get<RevivalSubject>(`/projects/${projectId}/subjects/${subjectId}`)).data; }
  async listSubjects(projectId: string, params?: { status?: string; limit?: number }): Promise<PaginatedResponse<RevivalSubject>> { return (await this.axios.get<PaginatedResponse<RevivalSubject>>(`/projects/${projectId}/subjects`, { params })).data; }
  async updateSubject(projectId: string, subjectId: string, updates: Partial<RevivalSubject>): Promise<RevivalSubject> { return (await this.axios.put<RevivalSubject>(`/projects/${projectId}/subjects/${subjectId}`, updates)).data; }
  async getSubjectHistory(projectId: string, subjectId: string): Promise<SubjectEvent[]> { return (await this.axios.get<SubjectEvent[]>(`/projects/${projectId}/subjects/${subjectId}/history`)).data; }

  // Revival Process
  async scheduleRevival(projectId: string, subjectId: string, schedule: RevivalSchedule): Promise<ScheduleResult> { return (await this.axios.post<ScheduleResult>(`/projects/${projectId}/subjects/${subjectId}/schedule`, schedule)).data; }
  async initiateRevival(projectId: string, subjectId: string, initiation: RevivalInitiation): Promise<InitiationResult> { return (await this.axios.post<InitiationResult>(`/projects/${projectId}/subjects/${subjectId}/initiate`, initiation)).data; }
  async recordProgress(projectId: string, subjectId: string, progress: RevivalProgress): Promise<RevivalSubject> { return (await this.axios.post<RevivalSubject>(`/projects/${projectId}/subjects/${subjectId}/progress`, progress)).data; }
  async recordOutcome(projectId: string, subjectId: string, outcome: RevivalOutcome): Promise<RevivalSubject> { return (await this.axios.post<RevivalSubject>(`/projects/${projectId}/subjects/${subjectId}/outcome`, outcome)).data; }
  async abortRevival(projectId: string, subjectId: string, abort: AbortRequest): Promise<AbortResult> { return (await this.axios.post<AbortResult>(`/projects/${projectId}/subjects/${subjectId}/abort`, abort)).data; }

  // Health Monitoring
  async recordHealthAssessment(projectId: string, subjectId: string, assessment: HealthAssessment): Promise<RevivalSubject> { return (await this.axios.post<RevivalSubject>(`/projects/${projectId}/subjects/${subjectId}/health`, assessment)).data; }
  async getCurrentVitals(projectId: string, subjectId: string): Promise<VitalsData> { return (await this.axios.get<VitalsData>(`/projects/${projectId}/subjects/${subjectId}/vitals`)).data; }
  async getVitalsHistory(projectId: string, subjectId: string, params?: { start?: string; end?: string }): Promise<VitalsRecord[]> { return (await this.axios.get<VitalsRecord[]>(`/projects/${projectId}/subjects/${subjectId}/vitals/history`, { params })).data; }

  // Protocols
  async listProtocols(projectId: string, params?: { status?: string }): Promise<RevivalProtocol[]> { return (await this.axios.get<RevivalProtocol[]>(`/projects/${projectId}/protocols`, { params })).data; }
  async getProtocol(projectId: string, protocolId: string): Promise<RevivalProtocol> { return (await this.axios.get<RevivalProtocol>(`/projects/${projectId}/protocols/${protocolId}`)).data; }
  async createProtocol(projectId: string, protocol: Partial<RevivalProtocol>): Promise<RevivalProtocol> { return (await this.axios.post<RevivalProtocol>(`/projects/${projectId}/protocols`, protocol)).data; }

  // Alerts & Dashboard
  async getAlerts(projectId: string, params?: { severity?: string; subjectId?: string }): Promise<Alert[]> { return (await this.axios.get<Alert[]>(`/projects/${projectId}/alerts`, { params })).data; }
  async acknowledgeAlert(projectId: string, alertId: string): Promise<void> { await this.axios.post(`/projects/${projectId}/alerts/${alertId}/acknowledge`); }
  async getDashboard(projectId: string): Promise<DashboardData> { return (await this.axios.get<DashboardData>(`/projects/${projectId}/dashboard`)).data; }

  // Validation
  validateProject(project: WIACryoRevivalProject): ValidationResult {
    const errors: any[] = [];
    if (!project.standard || project.standard !== 'WIA-CRYO-REVIVAL') errors.push({ path: 'standard', message: 'Standard must be "WIA-CRYO-REVIVAL"' });
    if (!project.metadata?.id) errors.push({ path: 'metadata.id', message: 'Project ID is required' });
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

// ============================================================================
// Supporting Types
// ============================================================================

export interface RevivalSchedule { protocol: string; scheduledDate: string; team: string[]; facility: string; }
export interface ScheduleResult { id: string; scheduled: boolean; scheduledDate: string; }
export interface RevivalInitiation { protocol: string; team: string[]; authorizedBy: string; }
export interface InitiationResult { id: string; status: 'initiated' | 'failed'; startedAt: string; }
export interface RevivalProgress { phase: string; step: number; vitals: Record<string, number>; observations: string; timestamp: string; }
export interface AbortRequest { reason: string; authorizedBy: string; stabilization: string; }
export interface AbortResult { id: string; status: 'aborted'; abortedAt: string; stabilized: boolean; }
export interface VitalsData { subjectId: string; vitals: Record<string, number>; consciousness?: string; timestamp: string; status: string; }
export interface VitalsRecord { timestamp: string; vitals: Record<string, number>; notes?: string; }
export interface Alert { id: string; subjectId?: string; type: string; severity: string; message: string; timestamp: string; acknowledged: boolean; }
export interface DashboardData { subjects: { total: number; byStatus: Record<string, number> }; active: { inRevival: number; recovery: number; rehabilitation: number }; outcomes: { success: number; complications: number }; alerts: { active: number; critical: number }; }

// ============================================================================
// Utility Functions
// ============================================================================

export function generateUUID(): string { return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => { const r = (Math.random() * 16) | 0; return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16); }); }

export function createMinimalProject(name: string, organization: string): WIACryoRevivalProject {
  return {
    standard: 'WIA-CRYO-REVIVAL', version: '1.0.0',
    metadata: { id: generateUUID(), name, type: 'research', organization: { name: organization, type: 'research', country: 'US', contact: { name: '', email: '' } }, facility: { id: '', name: '', location: '', capabilities: [] }, createdAt: new Date().toISOString(), status: 'active' },
    subjects: [],
    protocols: [],
    technology: { warming: { method: '', rate: '', uniformity: '', monitoring: [] }, perfusion: { type: '', solutions: [], pressure: '', temperature: '', oxygenation: '' }, monitoring: { vitals: [], imaging: [], biochemical: [], neural: [] }, support: { respiratory: '', cardiovascular: '', temperature: '', nutrition: '' }, imaging: { modalities: [], realTime: true, ai: true } },
    ethics: { principles: [], consent: { prePreservation: [], revival: [], ongoing: [], withdrawal: '' }, review: { board: '', approval: '', monitoring: '', reporting: '' }, rights: { information: [], autonomy: [], privacy: [], welfare: [] } },
    medical: { team: { lead: '', specialists: [], support: [] }, facilities: [], emergency: { procedures: [], resources: [], contacts: [] }, followUp: { schedule: '', assessments: [], duration: '' } },
    monitoring: { realTime: { parameters: [], frequency: 1, dashboard: '' }, alerts: { thresholds: [], channels: [], escalation: '' }, documentation: { automated: true, retention: '', format: '' }, reporting: { internal: '', regulatory: '', research: '' } },
  };
}

export default { WIACryoRevivalClient, generateUUID, createMinimalProject };
