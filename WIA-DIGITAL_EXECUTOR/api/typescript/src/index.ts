/**
 * WIA Digital Executor Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIADigitalExecutor, ExecutorResponse, DigitalAsset, ExecutionInstruction,
  ExecutionTrigger, AuditEvent, ValidationResult, PaginatedResponse,
} from './types';

export class WIADigitalExecutorClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: { 'Content-Type': 'application/json', ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }) },
    });
  }

  async createExecutorProfile(profile: WIADigitalExecutor): Promise<ExecutorResponse> {
    const response = await this.axios.post<ExecutorResponse>('/executors', profile);
    return response.data;
  }

  async getExecutorProfile(id: string): Promise<WIADigitalExecutor> {
    const response = await this.axios.get<WIADigitalExecutor>(`/executors/${id}`);
    return response.data;
  }

  async listExecutorProfiles(params?: { status?: string; limit?: number }): Promise<PaginatedResponse<ExecutorResponse>> {
    const response = await this.axios.get<PaginatedResponse<ExecutorResponse>>('/executors', { params });
    return response.data;
  }

  async updateExecutorProfile(id: string, updates: Partial<WIADigitalExecutor>): Promise<ExecutorResponse> {
    const response = await this.axios.put<ExecutorResponse>(`/executors/${id}`, updates);
    return response.data;
  }

  async addDigitalAsset(executorId: string, asset: DigitalAsset): Promise<DigitalAsset> {
    const response = await this.axios.post<DigitalAsset>(`/executors/${executorId}/assets`, asset);
    return response.data;
  }

  async listDigitalAssets(executorId: string): Promise<DigitalAsset[]> {
    const response = await this.axios.get<DigitalAsset[]>(`/executors/${executorId}/assets`);
    return response.data;
  }

  async updateAsset(executorId: string, assetId: string, updates: Partial<DigitalAsset>): Promise<DigitalAsset> {
    const response = await this.axios.put<DigitalAsset>(`/executors/${executorId}/assets/${assetId}`, updates);
    return response.data;
  }

  async addInstruction(executorId: string, instruction: ExecutionInstruction): Promise<ExecutionInstruction> {
    const response = await this.axios.post<ExecutionInstruction>(`/executors/${executorId}/instructions`, instruction);
    return response.data;
  }

  async executeInstruction(executorId: string, instructionId: string): Promise<{ success: boolean; message: string }> {
    const response = await this.axios.post(`/executors/${executorId}/instructions/${instructionId}/execute`);
    return response.data;
  }

  async armTrigger(executorId: string, trigger: ExecutionTrigger): Promise<ExecutionTrigger> {
    const response = await this.axios.post<ExecutionTrigger>(`/executors/${executorId}/triggers`, trigger);
    return response.data;
  }

  async activateTrigger(executorId: string, triggerId: string, proof: Record<string, unknown>): Promise<void> {
    await this.axios.post(`/executors/${executorId}/triggers/${triggerId}/activate`, proof);
  }

  async reportDeath(executorId: string, deathInfo: { certificateNumber: string; dateOfDeath: string; issuedBy: string }): Promise<void> {
    await this.axios.post(`/executors/${executorId}/report-death`, deathInfo);
  }

  async verifyBeneficiary(executorId: string, beneficiaryId: string, proof: Record<string, unknown>): Promise<{ verified: boolean }> {
    const response = await this.axios.post(`/executors/${executorId}/beneficiaries/${beneficiaryId}/verify`, proof);
    return response.data;
  }

  async getAuditLog(executorId: string): Promise<AuditEvent[]> {
    const response = await this.axios.get<AuditEvent[]>(`/executors/${executorId}/audit`);
    return response.data;
  }

  async getExecutionReport(executorId: string): Promise<{ completed: number; pending: number; failed: number; assets: { transferred: number; deleted: number } }> {
    const response = await this.axios.get(`/executors/${executorId}/report`);
    return response.data;
  }

  validateProfile(profile: WIADigitalExecutor): ValidationResult {
    const errors: any[] = [];
    if (profile.standard !== 'WIA-DIGITAL-EXECUTOR') errors.push({ path: 'standard', message: 'Invalid standard' });
    if (!profile.executor?.id) errors.push({ path: 'executor.id', message: 'Executor ID required' });
    if (!profile.principal?.id) errors.push({ path: 'principal.id', message: 'Principal ID required' });
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16);
  });
}

export function createMinimalProfile(executorName: string, principalName: string, email: string): WIADigitalExecutor {
  return {
    standard: 'WIA-DIGITAL-EXECUTOR',
    version: '1.0.0',
    executor: {
      id: generateUUID(), name: executorName, type: 'individual', status: 'active', jurisdiction: 'US',
      createdAt: new Date().toISOString(), contact: { email, notificationPreferences: ['email'] },
    },
    principal: {
      id: generateUUID(), name: principalName, identifiers: [], lastKnownStatus: 'alive', statusUpdatedAt: new Date().toISOString(),
    },
    digitalAssets: [],
    instructions: [],
    triggers: [{ id: generateUUID(), type: 'inactivity', configuration: { inactivityDays: 180, verificationRequired: true }, status: 'armed' }],
    verification: {
      deathVerification: { methods: ['death-certificate'], requiredConfirmations: 1 },
      executorVerification: { identityVerification: true, periodicReVerification: true, reVerificationInterval: 365 },
      beneficiaryVerification: { identityVerification: true, relationshipVerification: false },
    },
    audit: { enabled: true, retentionYears: 7, immutable: true, events: [] },
  };
}

export default { WIADigitalExecutorClient, generateUUID, createMinimalProfile };
