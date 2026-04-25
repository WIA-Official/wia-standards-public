/**
 * WIA DDoS Protection Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig,
  WIADDoSProtection,
  ServiceResponse,
  ProtectionTarget,
  ProtectionPolicy,
  RateLimitConfig,
  AttackEvent,
  IPEntry,
  ValidationResult,
  PaginatedResponse,
} from './types';

// ============================================================================
// WIA DDoS Protection Client
// ============================================================================

export class WIADDoSProtectionClient {
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
  // Service Management
  // ========================================================================

  async createService(service: WIADDoSProtection): Promise<ServiceResponse> {
    const response = await this.axios.post<ServiceResponse>('/services', service);
    return response.data;
  }

  async getService(id: string): Promise<WIADDoSProtection> {
    const response = await this.axios.get<WIADDoSProtection>(`/services/${id}`);
    return response.data;
  }

  async listServices(params?: {
    status?: string;
    tier?: string;
    limit?: number;
    offset?: number;
  }): Promise<PaginatedResponse<ServiceResponse>> {
    const response = await this.axios.get<PaginatedResponse<ServiceResponse>>('/services', { params });
    return response.data;
  }

  async updateService(id: string, updates: Partial<WIADDoSProtection>): Promise<ServiceResponse> {
    const response = await this.axios.put<ServiceResponse>(`/services/${id}`, updates);
    return response.data;
  }

  async deleteService(id: string): Promise<void> {
    await this.axios.delete(`/services/${id}`);
  }

  // ========================================================================
  // Protection Targets
  // ========================================================================

  async addTarget(serviceId: string, target: ProtectionTarget): Promise<ProtectionTarget> {
    const response = await this.axios.post<ProtectionTarget>(`/services/${serviceId}/targets`, target);
    return response.data;
  }

  async listTargets(serviceId: string): Promise<ProtectionTarget[]> {
    const response = await this.axios.get<ProtectionTarget[]>(`/services/${serviceId}/targets`);
    return response.data;
  }

  async removeTarget(serviceId: string, targetId: string): Promise<void> {
    await this.axios.delete(`/services/${serviceId}/targets/${targetId}`);
  }

  // ========================================================================
  // Policies
  // ========================================================================

  async createPolicy(serviceId: string, policy: ProtectionPolicy): Promise<ProtectionPolicy> {
    const response = await this.axios.post<ProtectionPolicy>(`/services/${serviceId}/policies`, policy);
    return response.data;
  }

  async getPolicy(serviceId: string, policyId: string): Promise<ProtectionPolicy> {
    const response = await this.axios.get<ProtectionPolicy>(`/services/${serviceId}/policies/${policyId}`);
    return response.data;
  }

  async updatePolicy(serviceId: string, policyId: string, updates: Partial<ProtectionPolicy>): Promise<ProtectionPolicy> {
    const response = await this.axios.put<ProtectionPolicy>(`/services/${serviceId}/policies/${policyId}`, updates);
    return response.data;
  }

  async deletePolicy(serviceId: string, policyId: string): Promise<void> {
    await this.axios.delete(`/services/${serviceId}/policies/${policyId}`);
  }

  // ========================================================================
  // Rate Limiting
  // ========================================================================

  async createRateLimit(serviceId: string, rateLimit: RateLimitConfig): Promise<RateLimitConfig> {
    const response = await this.axios.post<RateLimitConfig>(`/services/${serviceId}/rate-limits`, rateLimit);
    return response.data;
  }

  async listRateLimits(serviceId: string): Promise<RateLimitConfig[]> {
    const response = await this.axios.get<RateLimitConfig[]>(`/services/${serviceId}/rate-limits`);
    return response.data;
  }

  async updateRateLimit(serviceId: string, rateLimitId: string, updates: Partial<RateLimitConfig>): Promise<RateLimitConfig> {
    const response = await this.axios.put<RateLimitConfig>(`/services/${serviceId}/rate-limits/${rateLimitId}`, updates);
    return response.data;
  }

  // ========================================================================
  // IP Lists
  // ========================================================================

  async addToAllowList(serviceId: string, entry: IPEntry): Promise<IPEntry> {
    const response = await this.axios.post<IPEntry>(`/services/${serviceId}/ip-lists/allow`, entry);
    return response.data;
  }

  async addToBlockList(serviceId: string, entry: IPEntry): Promise<IPEntry> {
    const response = await this.axios.post<IPEntry>(`/services/${serviceId}/ip-lists/block`, entry);
    return response.data;
  }

  async getAllowList(serviceId: string): Promise<IPEntry[]> {
    const response = await this.axios.get<IPEntry[]>(`/services/${serviceId}/ip-lists/allow`);
    return response.data;
  }

  async getBlockList(serviceId: string): Promise<IPEntry[]> {
    const response = await this.axios.get<IPEntry[]>(`/services/${serviceId}/ip-lists/block`);
    return response.data;
  }

  async removeFromList(serviceId: string, listType: 'allow' | 'block', ip: string): Promise<void> {
    await this.axios.delete(`/services/${serviceId}/ip-lists/${listType}/${encodeURIComponent(ip)}`);
  }

  // ========================================================================
  // Attack Events
  // ========================================================================

  async getAttacks(serviceId: string, params?: {
    status?: string;
    type?: string;
    startTime?: string;
    endTime?: string;
    limit?: number;
    offset?: number;
  }): Promise<PaginatedResponse<AttackEvent>> {
    const response = await this.axios.get<PaginatedResponse<AttackEvent>>(`/services/${serviceId}/attacks`, { params });
    return response.data;
  }

  async getAttack(serviceId: string, attackId: string): Promise<AttackEvent> {
    const response = await this.axios.get<AttackEvent>(`/services/${serviceId}/attacks/${attackId}`);
    return response.data;
  }

  async getCurrentAttacks(serviceId: string): Promise<AttackEvent[]> {
    const response = await this.axios.get<AttackEvent[]>(`/services/${serviceId}/attacks/current`);
    return response.data;
  }

  // ========================================================================
  // Metrics
  // ========================================================================

  async getMetrics(serviceId: string, params?: {
    metrics?: string[];
    startTime?: string;
    endTime?: string;
    granularity?: 'minute' | 'hour' | 'day';
  }): Promise<{
    timestamps: string[];
    metrics: Record<string, number[]>;
  }> {
    const response = await this.axios.get(`/services/${serviceId}/metrics`, { params });
    return response.data;
  }

  async getRealTimeStats(serviceId: string): Promise<{
    requestsPerSecond: number;
    bytesPerSecond: number;
    blockedRequests: number;
    activeConnections: number;
    attackStatus: string;
  }> {
    const response = await this.axios.get(`/services/${serviceId}/stats/realtime`);
    return response.data;
  }

  // ========================================================================
  // Emergency Actions
  // ========================================================================

  async enableEmergencyMode(serviceId: string): Promise<void> {
    await this.axios.post(`/services/${serviceId}/emergency/enable`);
  }

  async disableEmergencyMode(serviceId: string): Promise<void> {
    await this.axios.post(`/services/${serviceId}/emergency/disable`);
  }

  async triggerManualMitigation(serviceId: string, level: number): Promise<void> {
    await this.axios.post(`/services/${serviceId}/mitigation/trigger`, { level });
  }

  // ========================================================================
  // Validation
  // ========================================================================

  validateService(service: WIADDoSProtection): ValidationResult {
    const errors: any[] = [];

    if (!service.standard || service.standard !== 'WIA-DDOS-PROTECTION') {
      errors.push({ path: 'standard', message: 'Standard must be "WIA-DDOS-PROTECTION"' });
    }

    if (!service.version || !/^\d+\.\d+\.\d+$/.test(service.version)) {
      errors.push({ path: 'version', message: 'Version must follow semantic versioning' });
    }

    if (!service.service?.id) {
      errors.push({ path: 'service.id', message: 'Service ID is required' });
    }

    if (!service.protection?.targets?.length) {
      errors.push({ path: 'protection.targets', message: 'At least one protection target is required' });
    }

    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
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

export function createMinimalService(name: string, targetDomain: string): WIADDoSProtection {
  return {
    standard: 'WIA-DDOS-PROTECTION',
    version: '1.0.0',
    service: {
      id: generateUUID(),
      name,
      owner: 'default',
      createdAt: new Date().toISOString(),
      status: 'active',
      tier: 'standard',
    },
    protection: {
      enabled: true,
      mode: 'always-on',
      targets: [{ id: generateUUID(), type: 'domain', identifier: targetDomain, priority: 1 }],
      policies: [],
      rateLimits: [],
    },
    detection: {
      enabled: true,
      algorithms: [{ id: generateUUID(), type: 'rate-based', enabled: true, sensitivity: 'medium' }],
      baselines: { learningPeriod: 7, updateInterval: 24, metrics: [] },
      anomalyDetection: { enabled: true, methods: ['statistical'], alertThreshold: 0.8, mitigationThreshold: 0.95 },
      signatureDetection: { enabled: true, signatureSets: ['default'], updateFrequency: 24 },
    },
    mitigation: {
      automaticResponse: true,
      escalationLevels: [
        { level: 1, name: 'Low', triggers: [{ metric: 'rps', threshold: 10000 }], actions: [{ type: 'rate-limit', parameters: {}, priority: 1 }] },
        { level: 2, name: 'Medium', triggers: [{ metric: 'rps', threshold: 50000 }], actions: [{ type: 'challenge', parameters: {}, priority: 1 }] },
        { level: 3, name: 'High', triggers: [{ metric: 'rps', threshold: 100000 }], actions: [{ type: 'scrubbing-center', parameters: {}, priority: 1 }] },
      ],
      techniques: [],
    },
  };
}

export default { WIADDoSProtectionClient, generateUUID, createMinimalService };
