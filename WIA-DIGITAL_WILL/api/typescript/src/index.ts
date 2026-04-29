/**
 * WIA Digital Will Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIADigitalWill, WillResponse, DigitalAsset, Beneficiary,
  WillInstruction, SignatureInfo, ValidationResult, PaginatedResponse,
} from './types';

export class WIADigitalWillClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: { 'Content-Type': 'application/json', ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }) },
    });
  }

  async createWill(will: WIADigitalWill): Promise<WillResponse> {
    const response = await this.axios.post<WillResponse>('/wills', will);
    return response.data;
  }

  async getWill(id: string): Promise<WIADigitalWill> {
    const response = await this.axios.get<WIADigitalWill>(`/wills/${id}`);
    return response.data;
  }

  async listWills(params?: { status?: string; limit?: number }): Promise<PaginatedResponse<WillResponse>> {
    const response = await this.axios.get<PaginatedResponse<WillResponse>>('/wills', { params });
    return response.data;
  }

  async updateWill(id: string, updates: Partial<WIADigitalWill>): Promise<WillResponse> {
    const response = await this.axios.put<WillResponse>(`/wills/${id}`, updates);
    return response.data;
  }

  async addAsset(willId: string, asset: DigitalAsset): Promise<DigitalAsset> {
    const response = await this.axios.post<DigitalAsset>(`/wills/${willId}/assets`, asset);
    return response.data;
  }

  async listAssets(willId: string): Promise<DigitalAsset[]> {
    const response = await this.axios.get<DigitalAsset[]>(`/wills/${willId}/assets`);
    return response.data;
  }

  async addBeneficiary(willId: string, beneficiary: Beneficiary): Promise<Beneficiary> {
    const response = await this.axios.post<Beneficiary>(`/wills/${willId}/beneficiaries`, beneficiary);
    return response.data;
  }

  async addInstruction(willId: string, instruction: WillInstruction): Promise<WillInstruction> {
    const response = await this.axios.post<WillInstruction>(`/wills/${willId}/instructions`, instruction);
    return response.data;
  }

  async signWill(willId: string, signerType: string, signature: { method: string; certificate?: string }): Promise<SignatureInfo> {
    const response = await this.axios.post<SignatureInfo>(`/wills/${willId}/sign`, { signerType, ...signature });
    return response.data;
  }

  async requestNotarization(willId: string, notaryInfo: { name: string; jurisdiction: string }): Promise<{ requestId: string }> {
    const response = await this.axios.post(`/wills/${willId}/notarize`, notaryInfo);
    return response.data;
  }

  async revokeWill(willId: string, reason: string): Promise<void> {
    await this.axios.post(`/wills/${willId}/revoke`, { reason });
  }

  async verifyWill(willId: string): Promise<{ valid: boolean; issues: string[] }> {
    const response = await this.axios.get(`/wills/${willId}/verify`);
    return response.data;
  }

  async getExecutionStatus(willId: string): Promise<{ status: string; completedInstructions: number; pendingInstructions: number }> {
    const response = await this.axios.get(`/wills/${willId}/execution-status`);
    return response.data;
  }

  async exportWill(willId: string, format: 'pdf' | 'json'): Promise<{ url: string; expiresAt: string }> {
    const response = await this.axios.get(`/wills/${willId}/export`, { params: { format } });
    return response.data;
  }

  validateWill(will: WIADigitalWill): ValidationResult {
    const errors: any[] = [];
    if (will.standard !== 'WIA-DIGITAL-WILL') errors.push({ path: 'standard', message: 'Invalid standard' });
    if (!will.will?.id) errors.push({ path: 'will.id', message: 'Will ID required' });
    if (!will.testator?.fullName) errors.push({ path: 'testator.fullName', message: 'Testator name required' });
    if (!will.witnesses?.length || will.witnesses.length < 2) errors.push({ path: 'witnesses', message: 'At least 2 witnesses required' });
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16);
  });
}

export function createMinimalWill(testatorName: string, jurisdiction: string): WIADigitalWill {
  return {
    standard: 'WIA-DIGITAL-WILL',
    version: '1.0.0',
    will: { id: generateUUID(), status: 'draft', createdAt: new Date().toISOString(), revisionNumber: 1, jurisdiction, language: 'en' },
    testator: {
      id: generateUUID(), fullName: testatorName, dateOfBirth: '', nationality: '', status: 'alive',
      residenceAddress: { street: '', city: '', country: '', postalCode: '' },
      identificationDocuments: [], mentalCapacity: { declared: true, date: new Date().toISOString() },
    },
    assets: [],
    beneficiaries: [],
    instructions: [],
    witnesses: [],
    executor: { primary: { id: generateUUID(), type: 'individual', name: '', contactInfo: { email: '' }, acceptanceStatus: 'pending' }, alternates: [], powers: ['access-accounts', 'transfer-assets'] },
    legal: {
      governingLaw: jurisdiction, notarization: { required: true }, signatures: [],
      revocationClause: 'This will may be revoked at any time by the testator.',
      disputeResolution: { method: 'arbitration' }, privacySettings: { publicAfterExecution: false, redactedFields: [], accessRestrictions: [] },
    },
  };
}

export default { WIADigitalWillClient, generateUUID, createMinimalWill };
