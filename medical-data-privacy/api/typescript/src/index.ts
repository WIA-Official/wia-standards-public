/**
 * WIA-MED-017 Medical Data Privacy Standard - TypeScript SDK
 * @version 1.0.0
 */

import axios, { AxiosInstance } from 'axios';
import EventEmitter from 'eventemitter3';
import type * as Types from './types';

export * from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

export class MedicalDataPrivacySDK extends EventEmitter {
  private api: AxiosInstance;
  private config: Types.SDKConfig;

  // Sub-modules
  public encryption: EncryptionService;
  public access: AccessControlService;
  public consent: ConsentService;
  public audit: AuditService;
  public breach: BreachService;
  public compliance: ComplianceService;
  public deidentification: DeIdentificationService;

  constructor(config: Types.SDKConfig) {
    super();
    this.config = config;

    // Determine base URL
    const baseUrl = config.baseUrl || this.getDefaultBaseUrl(config.environment);

    // Initialize HTTP client
    this.api = axios.create({
      baseURL: baseUrl,
      timeout: config.timeout || 30000,
      headers: {
        'Authorization': `Bearer ${config.apiKey}`,
        'Content-Type': 'application/json',
        'User-Agent': 'WIA-Medical-Data-Privacy-SDK/1.0.0'
      }
    });

    // Add response interceptor for error handling
    this.api.interceptors.response.use(
      response => response,
      error => {
        const apiError = this.handleAPIError(error);
        throw apiError;
      }
    );

    // Initialize sub-modules
    this.encryption = new EncryptionService(this.api);
    this.access = new AccessControlService(this.api);
    this.consent = new ConsentService(this.api);
    this.audit = new AuditService(this.api);
    this.breach = new BreachService(this.api);
    this.compliance = new ComplianceService(this.api);
    this.deidentification = new DeIdentificationService(this.api);
  }

  private getDefaultBaseUrl(environment: string): string {
    const urls = {
      production: 'https://api.wia.org/medical-privacy/v1',
      staging: 'https://staging-api.wia.org/medical-privacy/v1',
      development: 'http://localhost:3000/api/v1'
    };
    return urls[environment as keyof typeof urls] || urls.production;
  }

  private handleAPIError(error: any): Types.MedicalDataPrivacyError {
    if (error.response) {
      const apiError: Types.APIErrorResponse = error.response.data;
      return new Types.MedicalDataPrivacyError(
        apiError.error.message,
        apiError.error.code,
        error.response.status,
        apiError.error.details
      );
    } else if (error.request) {
      return new Types.MedicalDataPrivacyError(
        'No response from server',
        'NETWORK_ERROR'
      );
    } else {
      return new Types.MedicalDataPrivacyError(
        error.message,
        'UNKNOWN_ERROR'
      );
    }
  }
}

// ============================================================================
// Encryption Service
// ============================================================================

class EncryptionService {
  constructor(private api: AxiosInstance) {}

  /**
   * Encrypt PHI data
   */
  async encryptData(request: Types.EncryptDataRequest): Promise<Types.EncryptDataResponse> {
    const response = await this.api.post('/encryption/encrypt', request);
    return response.data;
  }

  /**
   * Decrypt PHI data
   */
  async decryptData(encryptedData: string, keyId: string): Promise<any> {
    const response = await this.api.post('/encryption/decrypt', {
      encryptedData,
      keyId
    });
    return response.data;
  }

  /**
   * Rotate encryption keys
   */
  async rotateKey(keyId: string): Promise<{ newKeyId: string }> {
    const response = await this.api.post('/encryption/rotate-key', { keyId });
    return response.data;
  }

  /**
   * Get key metadata
   */
  async getKeyInfo(keyId: string): Promise<Types.EncryptionConfig> {
    const response = await this.api.get(`/encryption/keys/${keyId}`);
    return response.data;
  }
}

// ============================================================================
// Access Control Service
// ============================================================================

class AccessControlService {
  constructor(private api: AxiosInstance) {}

  /**
   * Check access permission
   */
  async checkAccess(request: Types.AccessRequestParams): Promise<Types.AccessRequestResponse> {
    const response = await this.api.post('/access/check', request);
    return response.data;
  }

  /**
   * Create or update access control policy
   */
  async setPolicy(policy: Types.AccessControlPolicy): Promise<Types.AccessControlPolicy> {
    const response = await this.api.post('/access/policies', policy);
    return response.data;
  }

  /**
   * Get policy by resource ID
   */
  async getPolicy(resourceId: string): Promise<Types.AccessControlPolicy> {
    const response = await this.api.get(`/access/policies/${resourceId}`);
    return response.data;
  }

  /**
   * List all policies
   */
  async listPolicies(params?: {
    userId?: string;
    roleId?: string;
    limit?: number;
    offset?: number;
  }): Promise<{ total: number; policies: Types.AccessControlPolicy[] }> {
    const response = await this.api.get('/access/policies', { params });
    return response.data;
  }

  /**
   * Revoke access
   */
  async revokeAccess(resourceId: string, userId: string): Promise<void> {
    await this.api.delete(`/access/policies/${resourceId}/users/${userId}`);
  }
}

// ============================================================================
// Consent Service
// ============================================================================

class ConsentService {
  constructor(private api: AxiosInstance) {}

  /**
   * Create consent record
   */
  async create(request: Types.CreateConsentRequest): Promise<Types.CreateConsentResponse> {
    const response = await this.api.post('/consent', request);
    return response.data;
  }

  /**
   * Get consent record
   */
  async get(consentId: string): Promise<Types.ConsentRecord> {
    const response = await this.api.get(`/consent/${consentId}`);
    return response.data;
  }

  /**
   * Withdraw consent
   */
  async withdraw(consentId: string, reason?: string): Promise<Types.ConsentRecord> {
    const response = await this.api.post(`/consent/${consentId}/withdraw`, { reason });
    return response.data;
  }

  /**
   * Renew consent
   */
  async renew(consentId: string, expiresAt?: string): Promise<Types.ConsentRecord> {
    const response = await this.api.post(`/consent/${consentId}/renew`, { expiresAt });
    return response.data;
  }

  /**
   * Get consent history for patient
   */
  async getPatientConsents(patientId: string): Promise<Types.ConsentRecord[]> {
    const response = await this.api.get(`/consent/patient/${patientId}`);
    return response.data;
  }

  /**
   * Verify consent for specific purpose
   */
  async verify(patientId: string, purpose: string): Promise<{ granted: boolean; consentId?: string }> {
    const response = await this.api.post('/consent/verify', { patientId, purpose });
    return response.data;
  }
}

// ============================================================================
// Audit Service
// ============================================================================

class AuditService {
  constructor(private api: AxiosInstance) {}

  /**
   * Log audit event
   */
  async log(event: Omit<Types.AuditLog, 'id' | 'hash' | 'previousHash'>): Promise<Types.AuditLog> {
    const response = await this.api.post('/audit/log', event);
    return response.data;
  }

  /**
   * Get audit logs
   */
  async query(params: {
    resourceId?: string;
    userId?: string;
    eventType?: Types.AuditEventType;
    startDate?: string;
    endDate?: string;
    limit?: number;
    offset?: number;
  }): Promise<{ total: number; logs: Types.AuditLog[] }> {
    const response = await this.api.get('/audit/logs', { params });
    return response.data;
  }

  /**
   * Verify audit log integrity
   */
  async verifyIntegrity(logId: string): Promise<{ valid: boolean; message?: string }> {
    const response = await this.api.get(`/audit/logs/${logId}/verify`);
    return response.data;
  }

  /**
   * Detect anomalies in access patterns
   */
  async detectAnomalies(userId: string, days: number = 7): Promise<{
    anomalies: Array<{ timestamp: string; description: string; severity: string }>;
  }> {
    const response = await this.api.get(`/audit/anomalies/${userId}`, {
      params: { days }
    });
    return response.data;
  }
}

// ============================================================================
// Breach Service
// ============================================================================

class BreachService {
  constructor(private api: AxiosInstance) {}

  /**
   * Report a data breach
   */
  async report(request: Types.ReportBreachRequest): Promise<Types.ReportBreachResponse> {
    const response = await this.api.post('/breach/report', request);
    return response.data;
  }

  /**
   * Get breach incident
   */
  async get(incidentId: string): Promise<Types.BreachIncident> {
    const response = await this.api.get(`/breach/incidents/${incidentId}`);
    return response.data;
  }

  /**
   * Update breach incident
   */
  async update(incidentId: string, updates: Partial<Types.BreachIncident>): Promise<Types.BreachIncident> {
    const response = await this.api.patch(`/breach/incidents/${incidentId}`, updates);
    return response.data;
  }

  /**
   * Send breach notification
   */
  async notify(incidentId: string, notification: Types.BreachNotification): Promise<void> {
    await this.api.post(`/breach/incidents/${incidentId}/notify`, notification);
  }

  /**
   * List all breach incidents
   */
  async list(params?: {
    severity?: Types.BreachSeverity;
    status?: string;
    limit?: number;
    offset?: number;
  }): Promise<{ total: number; incidents: Types.BreachIncident[] }> {
    const response = await this.api.get('/breach/incidents', { params });
    return response.data;
  }
}

// ============================================================================
// Compliance Service
// ============================================================================

class ComplianceService {
  constructor(private api: AxiosInstance) {}

  /**
   * Run compliance check
   */
  async check(request: Types.ComplianceCheckRequest): Promise<Types.ComplianceCheckResponse> {
    const response = await this.api.post('/compliance/check', request);
    return response.data;
  }

  /**
   * Get compliance status
   */
  async getStatus(standard: Types.ComplianceStandard): Promise<{
    compliant: boolean;
    score: number;
    lastChecked: string;
  }> {
    const response = await this.api.get(`/compliance/status/${standard}`);
    return response.data;
  }

  /**
   * Generate compliance report
   */
  async generateReport(standard: Types.ComplianceStandard, format: 'pdf' | 'json' = 'json'): Promise<any> {
    const response = await this.api.get(`/compliance/report/${standard}`, {
      params: { format }
    });
    return response.data;
  }
}

// ============================================================================
// De-Identification Service
// ============================================================================

class DeIdentificationService {
  constructor(private api: AxiosInstance) {}

  /**
   * De-identify PHI data
   */
  async deidentify(request: Types.DeIdentifyRequest): Promise<Types.DeIdentifyResponse> {
    const response = await this.api.post('/deidentify', request);
    return response.data;
  }

  /**
   * Calculate k-anonymity
   */
  async calculateKAnonymity(data: any[], quasiIdentifiers: string[]): Promise<{ k: number }> {
    const response = await this.api.post('/deidentify/k-anonymity', {
      data,
      quasiIdentifiers
    });
    return response.data;
  }

  /**
   * Apply differential privacy
   */
  async applyDifferentialPrivacy(
    query: string,
    epsilon: number,
    mechanism: 'laplace' | 'gaussian'
  ): Promise<{ result: any; epsilonUsed: number }> {
    const response = await this.api.post('/deidentify/differential-privacy', {
      query,
      epsilon,
      mechanism
    });
    return response.data;
  }
}

// ============================================================================
// Exports
// ============================================================================

export default MedicalDataPrivacySDK;

// Example usage:
/*
import MedicalDataPrivacySDK from '@wia/medical-data-privacy';

const sdk = new MedicalDataPrivacySDK({
  apiKey: 'your-api-key',
  environment: 'production',
  complianceMode: ['HIPAA', 'GDPR']
});

// Encrypt PHI data
const encrypted = await sdk.encryption.encryptData({
  data: { patientName: 'John Doe', ssn: '123-45-6789' },
  algorithm: 'AES-256-GCM'
});

// Create consent
const consent = await sdk.consent.create({
  patientId: 'patient-123',
  purposes: [
    { id: 'treatment', name: 'Treatment', description: 'Medical treatment', required: true, granted: true },
    { id: 'research', name: 'Research', description: 'Medical research', required: false, granted: true }
  ],
  dataTypes: ['medical_records', 'lab_results'],
  processingActivities: ['diagnosis', 'treatment_planning']
});

// Check access
const access = await sdk.access.checkAccess({
  userId: 'doctor-456',
  resourceId: 'patient-123',
  action: 'read',
  context: {
    ipAddress: '192.168.1.1',
    mfaVerified: true
  }
});

// De-identify data
const deidentified = await sdk.deidentification.deidentify({
  data: patientRecords,
  config: {
    method: 'safe_harbor',
    kAnonymity: { k: 5, quasiIdentifiers: ['age', 'zipcode', 'gender'] }
  }
});

// Report breach
const breach = await sdk.breach.report({
  type: 'unauthorized_access',
  severity: 'high',
  description: 'Unauthorized access to patient records',
  affectedRecords: 150,
  affectedIndividuals: 150,
  dataTypes: ['medical_records', 'billing_info'],
  detectedAt: new Date().toISOString()
});

// Run compliance check
const complianceResult = await sdk.compliance.check({
  standard: 'HIPAA',
  scope: ['encryption', 'access_control', 'audit_logging']
});
*/
