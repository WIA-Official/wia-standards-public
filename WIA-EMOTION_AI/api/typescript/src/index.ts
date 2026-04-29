/**
 * WIA Emotion AI Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIAEmotionAI, SystemResponse, EmotionModel, EmotionOutput,
  EmotionPrediction, BiasAssessment, ValidationResult, PaginatedResponse,
} from './types';

export class WIAEmotionAIClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: { 'Content-Type': 'application/json', ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }) },
    });
  }

  async createSystem(system: WIAEmotionAI): Promise<SystemResponse> {
    const response = await this.axios.post<SystemResponse>('/systems', system);
    return response.data;
  }

  async getSystem(id: string): Promise<WIAEmotionAI> {
    const response = await this.axios.get<WIAEmotionAI>(`/systems/${id}`);
    return response.data;
  }

  async listSystems(params?: { type?: string; status?: string; limit?: number }): Promise<PaginatedResponse<SystemResponse>> {
    const response = await this.axios.get<PaginatedResponse<SystemResponse>>('/systems', { params });
    return response.data;
  }

  async updateSystem(id: string, updates: Partial<WIAEmotionAI>): Promise<SystemResponse> {
    const response = await this.axios.put<SystemResponse>(`/systems/${id}`, updates);
    return response.data;
  }

  async registerModel(systemId: string, model: Omit<EmotionModel, 'id' | 'status'>): Promise<EmotionModel> {
    const response = await this.axios.post<EmotionModel>(`/systems/${systemId}/models`, model);
    return response.data;
  }

  async listModels(systemId: string): Promise<EmotionModel[]> {
    const response = await this.axios.get<EmotionModel[]>(`/systems/${systemId}/models`);
    return response.data;
  }

  async getModelPerformance(systemId: string, modelId: string): Promise<{ metrics: any; benchmarks: any }> {
    const response = await this.axios.get(`/systems/${systemId}/models/${modelId}/performance`);
    return response.data;
  }

  async deployModel(systemId: string, modelId: string): Promise<{ status: string; endpoint: string }> {
    const response = await this.axios.post(`/systems/${systemId}/models/${modelId}/deploy`);
    return response.data;
  }

  async analyzeEmotion(systemId: string, input: { type: string; data: string | Buffer; options?: any }): Promise<EmotionOutput> {
    const response = await this.axios.post<EmotionOutput>(`/systems/${systemId}/analyze`, input);
    return response.data;
  }

  async analyzeEmotionBatch(systemId: string, inputs: { type: string; data: string }[]): Promise<EmotionOutput[]> {
    const response = await this.axios.post<EmotionOutput[]>(`/systems/${systemId}/analyze/batch`, { inputs });
    return response.data;
  }

  async streamAnalysis(systemId: string, sessionId: string): Promise<{ streamUrl: string; protocol: string }> {
    const response = await this.axios.post(`/systems/${systemId}/stream`, { sessionId });
    return response.data;
  }

  async getEmotionHistory(systemId: string, params?: { from?: string; to?: string; sessionId?: string }): Promise<EmotionOutput[]> {
    const response = await this.axios.get<EmotionOutput[]>(`/systems/${systemId}/history`, { params });
    return response.data;
  }

  async aggregateEmotions(systemId: string, sessionId: string): Promise<{ dominant: EmotionPrediction; timeline: any[]; summary: any }> {
    const response = await this.axios.get(`/systems/${systemId}/sessions/${sessionId}/aggregate`);
    return response.data;
  }

  async getBiasReport(systemId: string, modelId: string): Promise<BiasAssessment> {
    const response = await this.axios.get<BiasAssessment>(`/systems/${systemId}/models/${modelId}/bias`);
    return response.data;
  }

  async runBiasAudit(systemId: string, modelId: string, testDataset: string): Promise<{ auditId: string; status: string }> {
    const response = await this.axios.post(`/systems/${systemId}/models/${modelId}/bias/audit`, { testDataset });
    return response.data;
  }

  async getExplainability(systemId: string, outputId: string): Promise<{ method: string; explanation: any }> {
    const response = await this.axios.get(`/systems/${systemId}/outputs/${outputId}/explain`);
    return response.data;
  }

  async recordConsent(systemId: string, subjectId: string, consent: { type: string; purposes: string[]; expires?: string }): Promise<{ consentId: string }> {
    const response = await this.axios.post(`/systems/${systemId}/consent`, { subjectId, ...consent });
    return response.data;
  }

  async revokeConsent(systemId: string, consentId: string): Promise<void> {
    await this.axios.delete(`/systems/${systemId}/consent/${consentId}`);
  }

  async requestDataDeletion(systemId: string, subjectId: string): Promise<{ requestId: string; status: string }> {
    const response = await this.axios.post(`/systems/${systemId}/privacy/deletion`, { subjectId });
    return response.data;
  }

  async getComplianceStatus(systemId: string): Promise<{ regulations: any[]; overall: string }> {
    const response = await this.axios.get(`/systems/${systemId}/compliance`);
    return response.data;
  }

  async getEthicsAudit(systemId: string): Promise<{ audits: any[]; recommendations: string[] }> {
    const response = await this.axios.get(`/systems/${systemId}/ethics/audit`);
    return response.data;
  }

  validateEmotionAI(emotionAI: WIAEmotionAI): ValidationResult {
    const errors: { path: string; message: string }[] = [];
    if (emotionAI.standard !== 'WIA-EMOTION-AI') errors.push({ path: 'standard', message: 'Invalid standard' });
    if (!emotionAI.system?.id) errors.push({ path: 'system.id', message: 'System ID required' });
    if (!emotionAI.ethics?.principles?.length) errors.push({ path: 'ethics.principles', message: 'Ethics principles required' });
    if (!emotionAI.privacy?.consent) errors.push({ path: 'privacy.consent', message: 'Privacy consent config required' });
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16);
  });
}

export function createMinimalEmotionAI(name: string, type: string = 'recognition'): WIAEmotionAI {
  return {
    standard: 'WIA-EMOTION-AI',
    version: '1.0.0',
    system: {
      id: generateUUID(), name, type: type as any, status: 'development', createdAt: new Date().toISOString(),
      capabilities: [{ type: 'basic-emotions', supported: true }],
      modalities: ['facial'],
      deployment: { environment: 'cloud', latency: 100, throughput: 1000, availability: 99.9 },
    },
    models: [],
    inputs: [],
    outputs: [],
    ethics: {
      principles: [{ name: 'Transparency', description: 'Be transparent about AI use', implementation: [], monitoring: [] }],
      governance: { board: { members: [], meetingFrequency: 'quarterly', decisionProcess: 'consensus' }, policies: [], escalation: { levels: [], sla: 24 } },
      audits: [],
      compliance: [],
    },
    privacy: {
      dataMinimization: true,
      retention: { rawData: 24, processedData: 168, aggregated: 8760, unit: 'hours' },
      anonymization: { techniques: ['face-blur'] },
      rights: { access: true, rectification: true, erasure: true, portability: true, objection: true, restriction: true },
      security: { encryption: { atRest: true, inTransit: true, algorithm: 'AES-256' }, accessControl: { type: 'RBAC', mfa: true }, audit: true, incidentResponse: true },
    },
    validation: {
      crossCultural: { cultures: [], adaptations: [], universality: 0 },
      clinical: { populations: [], protocols: [], approvals: [], contraindications: [] },
      realWorld: { environments: [], conditions: [], performance: [] },
    },
  };
}

export default { WIAEmotionAIClient, generateUUID, createMinimalEmotionAI };
