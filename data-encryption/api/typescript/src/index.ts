/**
 * WIA Data Encryption Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIADataEncryptionProject, ProjectResponse, ValidationResult, PaginatedResponse,
  Key, KeyVersion, EncryptionRequest, EncryptionResult, DecryptionRequest, DecryptionResult,
  SigningRequest, SignatureResult, VerificationRequest, VerificationResult,
  AccessPolicy, UsagePolicy, RotationPolicyDef, SymmetricAlgorithm, AsymmetricAlgorithm,
  FieldEncryptionRule, AlertRule, ComplianceStandard
} from './types';

// ============================================================================
// WIA Data Encryption Client
// ============================================================================

export class WIADataEncryptionClient {
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

  // Project Management
  async createProject(project: WIADataEncryptionProject): Promise<ProjectResponse> {
    return (await this.axios.post<ProjectResponse>('/projects', project)).data;
  }

  async getProject(id: string): Promise<WIADataEncryptionProject> {
    return (await this.axios.get<WIADataEncryptionProject>(`/projects/${id}`)).data;
  }

  async listProjects(params?: { status?: string; classification?: string; limit?: number }): Promise<PaginatedResponse<ProjectResponse>> {
    return (await this.axios.get<PaginatedResponse<ProjectResponse>>('/projects', { params })).data;
  }

  async updateProject(id: string, updates: Partial<WIADataEncryptionProject>): Promise<ProjectResponse> {
    return (await this.axios.put<ProjectResponse>(`/projects/${id}`, updates)).data;
  }

  async deleteProject(id: string): Promise<void> {
    await this.axios.delete(`/projects/${id}`);
  }

  // Key Management
  async generateKey(projectId: string, config: KeyGenerationConfig): Promise<Key> {
    return (await this.axios.post<Key>(`/projects/${projectId}/keys`, config)).data;
  }

  async listKeys(projectId: string, params?: { type?: string; status?: string; purpose?: string }): Promise<Key[]> {
    return (await this.axios.get<Key[]>(`/projects/${projectId}/keys`, { params })).data;
  }

  async getKey(projectId: string, keyId: string): Promise<Key> {
    return (await this.axios.get<Key>(`/projects/${projectId}/keys/${keyId}`)).data;
  }

  async getKeyVersions(projectId: string, keyId: string): Promise<KeyVersion[]> {
    return (await this.axios.get<KeyVersion[]>(`/projects/${projectId}/keys/${keyId}/versions`)).data;
  }

  async rotateKey(projectId: string, keyId: string): Promise<Key> {
    return (await this.axios.post<Key>(`/projects/${projectId}/keys/${keyId}/rotate`)).data;
  }

  async disableKey(projectId: string, keyId: string): Promise<Key> {
    return (await this.axios.post<Key>(`/projects/${projectId}/keys/${keyId}/disable`)).data;
  }

  async enableKey(projectId: string, keyId: string): Promise<Key> {
    return (await this.axios.post<Key>(`/projects/${projectId}/keys/${keyId}/enable`)).data;
  }

  async scheduleKeyDeletion(projectId: string, keyId: string, pendingWindowDays: number): Promise<Key> {
    return (await this.axios.post<Key>(`/projects/${projectId}/keys/${keyId}/schedule-deletion`, { pendingWindowDays })).data;
  }

  async cancelKeyDeletion(projectId: string, keyId: string): Promise<Key> {
    return (await this.axios.post<Key>(`/projects/${projectId}/keys/${keyId}/cancel-deletion`)).data;
  }

  async importKey(projectId: string, importConfig: KeyImportConfig): Promise<Key> {
    return (await this.axios.post<Key>(`/projects/${projectId}/keys/import`, importConfig)).data;
  }

  async exportPublicKey(projectId: string, keyId: string): Promise<{ publicKey: string; format: string }> {
    return (await this.axios.get<{ publicKey: string; format: string }>(`/projects/${projectId}/keys/${keyId}/public`)).data;
  }

  // Encryption Operations
  async encrypt(projectId: string, request: EncryptionRequest): Promise<EncryptionResult> {
    return (await this.axios.post<EncryptionResult>(`/projects/${projectId}/encrypt`, request)).data;
  }

  async decrypt(projectId: string, request: DecryptionRequest): Promise<DecryptionResult> {
    return (await this.axios.post<DecryptionResult>(`/projects/${projectId}/decrypt`, request)).data;
  }

  async reEncrypt(projectId: string, ciphertext: string, sourceKeyId: string, targetKeyId: string): Promise<EncryptionResult> {
    return (await this.axios.post<EncryptionResult>(`/projects/${projectId}/re-encrypt`, { ciphertext, sourceKeyId, targetKeyId })).data;
  }

  async encryptBatch(projectId: string, requests: EncryptionRequest[]): Promise<EncryptionResult[]> {
    return (await this.axios.post<EncryptionResult[]>(`/projects/${projectId}/encrypt/batch`, { requests })).data;
  }

  async decryptBatch(projectId: string, requests: DecryptionRequest[]): Promise<DecryptionResult[]> {
    return (await this.axios.post<DecryptionResult[]>(`/projects/${projectId}/decrypt/batch`, { requests })).data;
  }

  // Signing Operations
  async sign(projectId: string, request: SigningRequest): Promise<SignatureResult> {
    return (await this.axios.post<SignatureResult>(`/projects/${projectId}/sign`, request)).data;
  }

  async verify(projectId: string, request: VerificationRequest): Promise<VerificationResult> {
    return (await this.axios.post<VerificationResult>(`/projects/${projectId}/verify`, request)).data;
  }

  async signBatch(projectId: string, requests: SigningRequest[]): Promise<SignatureResult[]> {
    return (await this.axios.post<SignatureResult[]>(`/projects/${projectId}/sign/batch`, { requests })).data;
  }

  // Hash Operations
  async hash(projectId: string, data: string, algorithm?: string): Promise<{ hash: string; algorithm: string }> {
    return (await this.axios.post<{ hash: string; algorithm: string }>(`/projects/${projectId}/hash`, { data, algorithm })).data;
  }

  async hmac(projectId: string, data: string, keyId: string): Promise<{ mac: string; keyId: string }> {
    return (await this.axios.post<{ mac: string; keyId: string }>(`/projects/${projectId}/hmac`, { data, keyId })).data;
  }

  // Data Key Operations (for envelope encryption)
  async generateDataKey(projectId: string, keyId: string, keySpec?: string): Promise<DataKeyResult> {
    return (await this.axios.post<DataKeyResult>(`/projects/${projectId}/keys/${keyId}/data-key`, { keySpec })).data;
  }

  async generateDataKeyWithoutPlaintext(projectId: string, keyId: string, keySpec?: string): Promise<{ ciphertextBlob: string; keyId: string }> {
    return (await this.axios.post<{ ciphertextBlob: string; keyId: string }>(`/projects/${projectId}/keys/${keyId}/data-key-without-plaintext`, { keySpec })).data;
  }

  // Algorithm Management
  async listSymmetricAlgorithms(projectId: string): Promise<SymmetricAlgorithm[]> {
    return (await this.axios.get<SymmetricAlgorithm[]>(`/projects/${projectId}/algorithms/symmetric`)).data;
  }

  async listAsymmetricAlgorithms(projectId: string): Promise<AsymmetricAlgorithm[]> {
    return (await this.axios.get<AsymmetricAlgorithm[]>(`/projects/${projectId}/algorithms/asymmetric`)).data;
  }

  // Field-Level Encryption
  async listFieldEncryptionRules(projectId: string): Promise<FieldEncryptionRule[]> {
    return (await this.axios.get<FieldEncryptionRule[]>(`/projects/${projectId}/field-encryption/rules`)).data;
  }

  async createFieldEncryptionRule(projectId: string, rule: Partial<FieldEncryptionRule>): Promise<FieldEncryptionRule> {
    return (await this.axios.post<FieldEncryptionRule>(`/projects/${projectId}/field-encryption/rules`, rule)).data;
  }

  async encryptField(projectId: string, value: string, classification: string): Promise<{ encrypted: string; metadata: Record<string, unknown> }> {
    return (await this.axios.post<{ encrypted: string; metadata: Record<string, unknown> }>(`/projects/${projectId}/field-encryption/encrypt`, { value, classification })).data;
  }

  async decryptField(projectId: string, encrypted: string): Promise<{ value: string }> {
    return (await this.axios.post<{ value: string }>(`/projects/${projectId}/field-encryption/decrypt`, { encrypted })).data;
  }

  // Policy Management
  async listAccessPolicies(projectId: string): Promise<AccessPolicy[]> {
    return (await this.axios.get<AccessPolicy[]>(`/projects/${projectId}/policies/access`)).data;
  }

  async createAccessPolicy(projectId: string, policy: Partial<AccessPolicy>): Promise<AccessPolicy> {
    return (await this.axios.post<AccessPolicy>(`/projects/${projectId}/policies/access`, policy)).data;
  }

  async updateAccessPolicy(projectId: string, policyId: string, updates: Partial<AccessPolicy>): Promise<AccessPolicy> {
    return (await this.axios.put<AccessPolicy>(`/projects/${projectId}/policies/access/${policyId}`, updates)).data;
  }

  async listUsagePolicies(projectId: string): Promise<UsagePolicy[]> {
    return (await this.axios.get<UsagePolicy[]>(`/projects/${projectId}/policies/usage`)).data;
  }

  async createUsagePolicy(projectId: string, policy: Partial<UsagePolicy>): Promise<UsagePolicy> {
    return (await this.axios.post<UsagePolicy>(`/projects/${projectId}/policies/usage`, policy)).data;
  }

  async listRotationPolicies(projectId: string): Promise<RotationPolicyDef[]> {
    return (await this.axios.get<RotationPolicyDef[]>(`/projects/${projectId}/policies/rotation`)).data;
  }

  async createRotationPolicy(projectId: string, policy: Partial<RotationPolicyDef>): Promise<RotationPolicyDef> {
    return (await this.axios.post<RotationPolicyDef>(`/projects/${projectId}/policies/rotation`, policy)).data;
  }

  // Compliance
  async getComplianceStatus(projectId: string): Promise<ComplianceStatus> {
    return (await this.axios.get<ComplianceStatus>(`/projects/${projectId}/compliance/status`)).data;
  }

  async listComplianceStandards(projectId: string): Promise<ComplianceStandard[]> {
    return (await this.axios.get<ComplianceStandard[]>(`/projects/${projectId}/compliance/standards`)).data;
  }

  async generateComplianceReport(projectId: string, standards: string[]): Promise<{ reportId: string; status: string }> {
    return (await this.axios.post<{ reportId: string; status: string }>(`/projects/${projectId}/compliance/report`, { standards })).data;
  }

  // Audit
  async getAuditLog(projectId: string, params?: { keyId?: string; operation?: string; start?: string; end?: string; limit?: number }): Promise<PaginatedResponse<AuditLogEntry>> {
    return (await this.axios.get<PaginatedResponse<AuditLogEntry>>(`/projects/${projectId}/audit`, { params })).data;
  }

  async getKeyUsageStats(projectId: string, keyId: string, period: string): Promise<KeyUsageStats> {
    return (await this.axios.get<KeyUsageStats>(`/projects/${projectId}/keys/${keyId}/usage`, { params: { period } })).data;
  }

  // Alerts
  async listAlertRules(projectId: string): Promise<AlertRule[]> {
    return (await this.axios.get<AlertRule[]>(`/projects/${projectId}/alerts/rules`)).data;
  }

  async createAlertRule(projectId: string, rule: Partial<AlertRule>): Promise<AlertRule> {
    return (await this.axios.post<AlertRule>(`/projects/${projectId}/alerts/rules`, rule)).data;
  }

  async getActiveAlerts(projectId: string): Promise<Alert[]> {
    return (await this.axios.get<Alert[]>(`/projects/${projectId}/alerts/active`)).data;
  }

  // Validation
  validateProject(project: WIADataEncryptionProject): ValidationResult {
    const errors: { path: string; message: string }[] = [];

    if (!project.standard || project.standard !== 'WIA-DATA-ENCRYPTION') {
      errors.push({ path: 'standard', message: 'Standard must be "WIA-DATA-ENCRYPTION"' });
    }
    if (!project.version) {
      errors.push({ path: 'version', message: 'Version is required' });
    }
    if (!project.metadata?.id) {
      errors.push({ path: 'metadata.id', message: 'Project ID is required' });
    }
    if (!project.metadata?.name) {
      errors.push({ path: 'metadata.name', message: 'Project name is required' });
    }
    if (!project.keyManagement) {
      errors.push({ path: 'keyManagement', message: 'Key management configuration is required' });
    }
    if (!project.encryption) {
      errors.push({ path: 'encryption', message: 'Encryption configuration is required' });
    }

    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

// ============================================================================
// Supporting Types
// ============================================================================

export interface KeyGenerationConfig {
  name: string;
  type: 'symmetric' | 'asymmetric';
  algorithm: string;
  keyLength?: number;
  purpose: string;
  tags?: Record<string, string>;
  rotationEnabled?: boolean;
}

export interface KeyImportConfig {
  name: string;
  type: 'symmetric' | 'asymmetric';
  algorithm: string;
  keyMaterial: string;
  wrappingKeyId?: string;
  purpose: string;
}

export interface DataKeyResult {
  plaintext: string;
  ciphertextBlob: string;
  keyId: string;
}

export interface ComplianceStatus {
  overall: 'compliant' | 'partial' | 'non-compliant';
  standards: { standard: string; status: string; coverage: number }[];
  lastAssessed: string;
  gaps: number;
}

export interface AuditLogEntry {
  id: string;
  timestamp: string;
  operation: string;
  keyId?: string;
  principal: string;
  sourceIp?: string;
  success: boolean;
  errorCode?: string;
  requestId: string;
}

export interface KeyUsageStats {
  keyId: string;
  period: string;
  operations: { operation: string; count: number }[];
  peakUsage: { timestamp: string; count: number };
  errors: number;
}

export interface Alert {
  id: string;
  ruleId: string;
  severity: string;
  message: string;
  triggeredAt: string;
  status: 'active' | 'acknowledged' | 'resolved';
}

// ============================================================================
// Utility Functions
// ============================================================================

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16);
  });
}

export function createMinimalProject(name: string, organization: string): WIADataEncryptionProject {
  return {
    standard: 'WIA-DATA-ENCRYPTION',
    version: '1.0.0',
    metadata: {
      id: generateUUID(),
      name,
      organization: { name: organization, securityContact: { name: '', email: '', role: 'Security Officer' } },
      classification: 'confidential',
      createdAt: new Date().toISOString(),
      status: 'active',
    },
    keyManagement: {
      provider: { type: 'aws-kms', authentication: { method: 'iam', credentials: { type: 'env', reference: 'AWS_CREDENTIALS' } } },
      hierarchy: { levels: [{ level: 1, name: 'Master Key', purpose: 'master', algorithm: 'AES', keyLength: 256, protection: 'hsm' }], wrapping: { algorithm: 'aes-wrap' }, derivation: { function: 'hkdf', saltLength: 32 } },
      lifecycle: { generation: { source: 'hardware', entropy: 'hardware', validation: true }, rotation: { automatic: true, interval: '365d', triggerConditions: [], gracePeriod: '30d', notification: [] }, archival: { enabled: true, retentionPeriod: '7y', format: 'encrypted', location: '' }, destruction: { method: 'crypto-erase', verification: true, audit: true } },
      storage: { primary: { type: 'kms', location: '', redundancy: 'geo', encryption: true }, caching: { enabled: true, ttl: '1h', maxEntries: 1000, encryption: true } },
      backup: { enabled: true, frequency: 'daily', destinations: [], encryption: { algorithm: 'AES-256', keySource: 'separate' }, testing: 'quarterly' },
    },
    encryption: {
      algorithms: { symmetric: [{ id: 'aes-256-gcm', name: 'AES-256-GCM', algorithm: 'aes', keyLength: 256, mode: 'gcm', recommended: true }], asymmetric: [{ id: 'rsa-4096', name: 'RSA-4096', algorithm: 'rsa', keyLength: 4096, purpose: 'encryption' }], hashing: [{ id: 'sha-256', name: 'SHA-256', algorithm: 'sha256', outputLength: 256, recommended: true }], signing: [{ id: 'rsa-pss', name: 'RSA-PSS', algorithm: 'rsa-pss', hashAlgorithm: 'sha256' }] },
      modes: { atRest: { enabled: true, defaultAlgorithm: 'aes-256-gcm', keyScope: 'table', transparentEncryption: true }, inTransit: { required: true, protocols: { minimumVersion: '1.2', cipherSuites: [], certificateValidation: true }, mtls: false, certificateManagement: '' }, inUse: { enabled: false, technology: 'none', scope: [] } },
      dataTypes: [],
      fieldLevel: { enabled: true, defaultAction: 'encrypt', rules: [] },
    },
    policies: {
      accessPolicies: [],
      usagePolicies: [],
      rotationPolicies: [],
      exceptionHandling: { emergencyAccess: { enabled: true, approvers: [], maxDuration: '4h', auditRequired: true }, breakGlass: { enabled: true, procedure: '', notification: [], postIncidentReview: true } },
    },
    operations: {
      automation: { keyRotation: true, certificateRenewal: true, policyEnforcement: true, incidentResponse: false },
      integration: { apis: [], sdks: ['typescript', 'python', 'java'], plugins: [] },
      performance: { caching: true, batching: { enabled: true, maxSize: 100, maxLatency: '100ms' }, threading: { poolSize: 10, async: true } },
    },
    compliance: {
      standards: [],
      certifications: [],
      controls: [],
      auditing: { enabled: true, events: [], retention: '7y', integrity: 'signature', destination: '' },
    },
    monitoring: {
      metrics: { enabled: true, provider: '', metrics: [], interval: '1m' },
      alerting: { enabled: true, rules: [], channels: [] },
      logging: { level: 'info', format: 'json', destination: '', sensitiveDataMasking: true },
      reporting: { scheduled: [], onDemand: true, formats: ['pdf', 'json'] },
    },
  };
}

export default { WIADataEncryptionClient, generateUUID, createMinimalProject };
