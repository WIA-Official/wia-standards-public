/**
 * WIA-IOT_SECURITY: TypeScript SDK
 * Version: 1.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 *
 * Comprehensive SDK for IoT security management, device authentication,
 * firmware updates, security monitoring, and incident response.
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import * as jwt from 'jsonwebtoken';
import * as forge from 'node-forge';

import {
  WIAIoTSecurityConfig,
  DeviceRegistration,
  DeviceRegistrationResponse,
  AuthenticationRequest,
  AuthenticationResponse,
  TokenRefreshRequest,
  TokenRefreshResponse,
  Device,
  DeviceIdentity,
  SecurityCredential,
  FirmwareUpdateCheck,
  FirmwareUpdateResponse,
  FirmwareInstallation,
  FirmwareValidation,
  SecurityEvent,
  SecurityAlert,
  SecurityPolicy,
  PolicyApplication,
  PolicyApplicationResult,
  SecurityAudit,
  AuditResults,
  SecurityIncident,
  MonitoringMetrics,
  APIResponse,
  APIError,
  PaginatedResponse,
  DeviceStatus,
} from './types';

// Export all types
export * from './types';

/**
 * Main SDK class for WIA IoT Security Standard
 */
export class WIAIoTSecurity {
  private client: AxiosInstance;
  private config: WIAIoTSecurityConfig;
  private accessToken?: string;

  /**
   * Initialize the SDK
   * @param config SDK configuration
   */
  constructor(config: WIAIoTSecurityConfig) {
    this.config = {
      timeout: 30000,
      retry: {
        maxRetries: 3,
        retryDelay: 1000,
      },
      ...config,
    };

    this.client = axios.create({
      baseURL: this.config.apiBaseUrl,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'User-Agent': 'WIA-IoT-Security-SDK/1.0',
      },
    });

    // Add request interceptor for authentication
    this.client.interceptors.request.use((config) => {
      if (this.accessToken) {
        config.headers.Authorization = `Bearer ${this.accessToken}`;
      } else if (this.config.apiKey) {
        config.headers['X-API-Key'] = this.config.apiKey;
      }
      return config;
    });

    // Add response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      (error: AxiosError) => {
        return Promise.reject(this.handleError(error));
      }
    );
  }

  /**
   * Set access token for authenticated requests
   * @param token Access token
   */
  setAccessToken(token: string): void {
    this.accessToken = token;
  }

  /**
   * Handle API errors
   * @param error Axios error
   * @returns Formatted API error
   */
  private handleError(error: AxiosError): APIError {
    const apiError: APIError = {
      code: error.response?.data?.code || 'UNKNOWN_ERROR',
      message: error.response?.data?.message || error.message,
      timestamp: new Date().toISOString(),
      requestId: error.response?.headers['x-request-id'] || 'unknown',
      details: error.response?.data?.details,
    };
    return apiError;
  }

  // ============================================================================
  // Device Registration and Management
  // ============================================================================

  /**
   * Register a new IoT device
   * @param registration Device registration data
   * @returns Registration response with device ID and certificate
   */
  async registerDevice(
    registration: DeviceRegistration
  ): Promise<APIResponse<DeviceRegistrationResponse>> {
    const response = await this.client.post<APIResponse<DeviceRegistrationResponse>>(
      '/devices/register',
      registration
    );
    return response.data;
  }

  /**
   * Get device details by ID
   * @param deviceId Device identifier
   * @returns Device information
   */
  async getDevice(deviceId: string): Promise<APIResponse<Device>> {
    const response = await this.client.get<APIResponse<Device>>(`/devices/${deviceId}`);
    return response.data;
  }

  /**
   * Update device configuration
   * @param deviceId Device identifier
   * @param updates Device updates
   * @returns Updated device information
   */
  async updateDevice(
    deviceId: string,
    updates: Partial<DeviceIdentity>
  ): Promise<APIResponse<Device>> {
    const response = await this.client.patch<APIResponse<Device>>(
      `/devices/${deviceId}`,
      updates
    );
    return response.data;
  }

  /**
   * List all devices with optional filtering
   * @param filters Filter parameters
   * @returns Paginated list of devices
   */
  async listDevices(filters?: {
    status?: DeviceStatus;
    manufacturer?: string;
    model?: string;
    page?: number;
    pageSize?: number;
  }): Promise<APIResponse<PaginatedResponse<Device>>> {
    const response = await this.client.get<APIResponse<PaginatedResponse<Device>>>(
      '/devices',
      { params: filters }
    );
    return response.data;
  }

  /**
   * Revoke device credentials and block access
   * @param deviceId Device identifier
   * @param reason Revocation reason
   * @param immediate Immediate revocation flag
   * @returns Revocation result
   */
  async revokeDevice(
    deviceId: string,
    reason: string,
    immediate: boolean = true
  ): Promise<APIResponse<{ status: string; revokedAt: string }>> {
    const response = await this.client.post<APIResponse<{ status: string; revokedAt: string }>>(
      `/devices/${deviceId}/revoke`,
      { reason, immediate }
    );
    return response.data;
  }

  /**
   * Delete a device permanently
   * @param deviceId Device identifier
   * @returns Deletion confirmation
   */
  async deleteDevice(deviceId: string): Promise<APIResponse<{ deleted: boolean }>> {
    const response = await this.client.delete<APIResponse<{ deleted: boolean }>>(
      `/devices/${deviceId}`
    );
    return response.data;
  }

  // ============================================================================
  // Authentication and Authorization
  // ============================================================================

  /**
   * Authenticate a device and obtain access tokens
   * @param request Authentication request
   * @returns Authentication tokens
   */
  async authenticateDevice(
    request: AuthenticationRequest
  ): Promise<APIResponse<AuthenticationResponse>> {
    const response = await this.client.post<APIResponse<AuthenticationResponse>>(
      '/devices/authenticate',
      request
    );

    // Store access token
    if (response.data.data.accessToken) {
      this.setAccessToken(response.data.data.accessToken);
    }

    return response.data;
  }

  /**
   * Refresh an expired access token
   * @param refreshToken Refresh token
   * @returns New access token
   */
  async refreshToken(refreshToken: string): Promise<APIResponse<TokenRefreshResponse>> {
    const response = await this.client.post<APIResponse<TokenRefreshResponse>>(
      '/auth/refresh',
      { refreshToken }
    );

    // Update access token
    if (response.data.data.accessToken) {
      this.setAccessToken(response.data.data.accessToken);
    }

    return response.data;
  }

  /**
   * Generate authentication challenge for device
   * @param deviceId Device identifier
   * @returns Challenge data
   */
  async generateChallenge(
    deviceId: string
  ): Promise<APIResponse<{ challenge: string; expiresIn: number }>> {
    const response = await this.client.post<
      APIResponse<{ challenge: string; expiresIn: number }>
    >(`/devices/${deviceId}/challenge`);
    return response.data;
  }

  /**
   * Verify JWT token
   * @param token JWT token
   * @param publicKey Public key for verification
   * @returns Decoded token payload
   */
  verifyToken(token: string, publicKey: string): any {
    try {
      return jwt.verify(token, publicKey, { algorithms: ['RS256', 'ES256'] });
    } catch (error) {
      throw new Error(`Token verification failed: ${error.message}`);
    }
  }

  // ============================================================================
  // Credential Management
  // ============================================================================

  /**
   * Get device credentials
   * @param deviceId Device identifier
   * @returns Security credentials
   */
  async getCredentials(deviceId: string): Promise<APIResponse<SecurityCredential[]>> {
    const response = await this.client.get<APIResponse<SecurityCredential[]>>(
      `/devices/${deviceId}/credentials`
    );
    return response.data;
  }

  /**
   * Rotate device credentials
   * @param deviceId Device identifier
   * @returns New credentials
   */
  async rotateCredentials(
    deviceId: string
  ): Promise<APIResponse<SecurityCredential>> {
    const response = await this.client.post<APIResponse<SecurityCredential>>(
      `/devices/${deviceId}/credentials/rotate`
    );
    return response.data;
  }

  /**
   * Revoke specific credential
   * @param credentialId Credential identifier
   * @returns Revocation result
   */
  async revokeCredential(
    credentialId: string
  ): Promise<APIResponse<{ revoked: boolean }>> {
    const response = await this.client.post<APIResponse<{ revoked: boolean }>>(
      `/credentials/${credentialId}/revoke`
    );
    return response.data;
  }

  // ============================================================================
  // Firmware Management
  // ============================================================================

  /**
   * Check for firmware updates
   * @param check Update check parameters
   * @returns Available firmware updates
   */
  async checkFirmwareUpdate(
    check: FirmwareUpdateCheck
  ): Promise<APIResponse<FirmwareUpdateResponse>> {
    const response = await this.client.get<APIResponse<FirmwareUpdateResponse>>(
      '/firmware/updates',
      { params: check }
    );
    return response.data;
  }

  /**
   * Download firmware image
   * @param version Firmware version
   * @param deviceId Device identifier
   * @returns Firmware binary data
   */
  async downloadFirmware(version: string, deviceId: string): Promise<ArrayBuffer> {
    const response = await this.client.get<ArrayBuffer>(
      `/firmware/download/${version}`,
      {
        params: { deviceId },
        responseType: 'arraybuffer',
      }
    );
    return response.data;
  }

  /**
   * Report firmware installation
   * @param installation Installation report
   * @returns Installation confirmation
   */
  async reportFirmwareInstallation(
    installation: FirmwareInstallation
  ): Promise<APIResponse<{ installationId: string; verified: boolean }>> {
    const response = await this.client.post<
      APIResponse<{ installationId: string; verified: boolean }>
    >('/firmware/installations', installation);
    return response.data;
  }

  /**
   * Validate firmware image
   * @param firmwareData Firmware binary data
   * @param expectedChecksum Expected SHA-256 checksum
   * @param signature Digital signature
   * @param publicKey Public key for signature verification
   * @returns Validation result
   */
  validateFirmware(
    firmwareData: ArrayBuffer,
    expectedChecksum: string,
    signature: string,
    publicKey: string
  ): FirmwareValidation {
    try {
      // Calculate checksum
      const md = forge.md.sha256.create();
      md.update(forge.util.createBuffer(firmwareData));
      const actualChecksum = md.digest().toHex();

      const checksumVerified = actualChecksum === expectedChecksum;

      // Verify signature
      const publicKeyObj = forge.pki.publicKeyFromPem(publicKey);
      const signatureBytes = forge.util.decode64(signature);
      const signatureVerified = publicKeyObj.verify(
        md.digest().bytes(),
        signatureBytes
      );

      return {
        valid: checksumVerified && signatureVerified,
        signatureVerified,
        checksumVerified,
        compatible: true, // Additional compatibility checks can be added
        errors: [],
      };
    } catch (error) {
      return {
        valid: false,
        signatureVerified: false,
        checksumVerified: false,
        compatible: false,
        errors: [error.message],
      };
    }
  }

  // ============================================================================
  // Security Policies
  // ============================================================================

  /**
   * Get security policies for a device
   * @param deviceId Device identifier (optional)
   * @returns Security policies
   */
  async getSecurityPolicies(deviceId?: string): Promise<APIResponse<SecurityPolicy[]>> {
    const response = await this.client.get<APIResponse<SecurityPolicy[]>>('/policies', {
      params: deviceId ? { deviceId } : undefined,
    });
    return response.data;
  }

  /**
   * Create a new security policy
   * @param policy Policy definition
   * @returns Created policy
   */
  async createSecurityPolicy(
    policy: Omit<SecurityPolicy, 'policyId' | 'appliedAt'>
  ): Promise<APIResponse<SecurityPolicy>> {
    const response = await this.client.post<APIResponse<SecurityPolicy>>(
      '/policies',
      policy
    );
    return response.data;
  }

  /**
   * Apply security policy to devices
   * @param application Policy application request
   * @returns Application result
   */
  async applySecurityPolicy(
    application: PolicyApplication
  ): Promise<APIResponse<PolicyApplicationResult>> {
    const response = await this.client.post<APIResponse<PolicyApplicationResult>>(
      '/policies/apply',
      application
    );
    return response.data;
  }

  /**
   * Delete a security policy
   * @param policyId Policy identifier
   * @returns Deletion confirmation
   */
  async deleteSecurityPolicy(
    policyId: string
  ): Promise<APIResponse<{ deleted: boolean }>> {
    const response = await this.client.delete<APIResponse<{ deleted: boolean }>>(
      `/policies/${policyId}`
    );
    return response.data;
  }

  // ============================================================================
  // Security Monitoring
  // ============================================================================

  /**
   * Report security event
   * @param event Security event data
   * @returns Event confirmation
   */
  async reportSecurityEvent(
    event: Omit<SecurityEvent, 'eventId'>
  ): Promise<APIResponse<SecurityEvent>> {
    const response = await this.client.post<APIResponse<SecurityEvent>>(
      '/events',
      event
    );
    return response.data;
  }

  /**
   * Get security events for a device
   * @param deviceId Device identifier
   * @param filters Event filters
   * @returns Security events
   */
  async getSecurityEvents(
    deviceId: string,
    filters?: {
      startDate?: string;
      endDate?: string;
      severity?: string;
      category?: string;
    }
  ): Promise<APIResponse<PaginatedResponse<SecurityEvent>>> {
    const response = await this.client.get<APIResponse<PaginatedResponse<SecurityEvent>>>(
      `/devices/${deviceId}/events`,
      { params: filters }
    );
    return response.data;
  }

  /**
   * Create security alert
   * @param alert Alert configuration
   * @returns Created alert
   */
  async createSecurityAlert(
    alert: Omit<SecurityAlert, 'alertId'>
  ): Promise<APIResponse<SecurityAlert>> {
    const response = await this.client.post<APIResponse<SecurityAlert>>(
      '/alerts',
      alert
    );
    return response.data;
  }

  /**
   * Get monitoring metrics for a device
   * @param deviceId Device identifier
   * @param timeRange Time range for metrics
   * @returns Monitoring metrics
   */
  async getMonitoringMetrics(
    deviceId: string,
    timeRange?: { start: string; end: string }
  ): Promise<APIResponse<MonitoringMetrics[]>> {
    const response = await this.client.get<APIResponse<MonitoringMetrics[]>>(
      `/devices/${deviceId}/metrics`,
      { params: timeRange }
    );
    return response.data;
  }

  /**
   * Send monitoring metrics
   * @param metrics Monitoring metrics data
   * @returns Confirmation
   */
  async sendMonitoringMetrics(
    metrics: MonitoringMetrics
  ): Promise<APIResponse<{ received: boolean }>> {
    const response = await this.client.post<APIResponse<{ received: boolean }>>(
      '/metrics',
      metrics
    );
    return response.data;
  }

  // ============================================================================
  // Security Audits
  // ============================================================================

  /**
   * Run security audit on devices
   * @param audit Audit configuration
   * @returns Audit initiation response
   */
  async runSecurityAudit(
    audit: Omit<SecurityAudit, 'auditId' | 'status' | 'startedAt'>
  ): Promise<APIResponse<SecurityAudit>> {
    const response = await this.client.post<APIResponse<SecurityAudit>>(
      '/audits/run',
      audit
    );
    return response.data;
  }

  /**
   * Get audit results
   * @param auditId Audit identifier
   * @returns Audit results
   */
  async getAuditResults(auditId: string): Promise<APIResponse<AuditResults>> {
    const response = await this.client.get<APIResponse<AuditResults>>(
      `/audits/${auditId}`
    );
    return response.data;
  }

  /**
   * List all audits
   * @param filters Audit filters
   * @returns Paginated audit list
   */
  async listAudits(filters?: {
    status?: string;
    deviceId?: string;
    startDate?: string;
    endDate?: string;
  }): Promise<APIResponse<PaginatedResponse<SecurityAudit>>> {
    const response = await this.client.get<APIResponse<PaginatedResponse<SecurityAudit>>>(
      '/audits',
      { params: filters }
    );
    return response.data;
  }

  // ============================================================================
  // Incident Response
  // ============================================================================

  /**
   * Report security incident
   * @param incident Incident report
   * @returns Incident details
   */
  async reportIncident(
    incident: Omit<SecurityIncident, 'incidentId' | 'status' | 'timeline'>
  ): Promise<APIResponse<SecurityIncident>> {
    const response = await this.client.post<APIResponse<SecurityIncident>>(
      '/incidents',
      incident
    );
    return response.data;
  }

  /**
   * Get incident details
   * @param incidentId Incident identifier
   * @returns Incident information
   */
  async getIncident(incidentId: string): Promise<APIResponse<SecurityIncident>> {
    const response = await this.client.get<APIResponse<SecurityIncident>>(
      `/incidents/${incidentId}`
    );
    return response.data;
  }

  /**
   * Update incident status
   * @param incidentId Incident identifier
   * @param updates Incident updates
   * @returns Updated incident
   */
  async updateIncident(
    incidentId: string,
    updates: Partial<SecurityIncident>
  ): Promise<APIResponse<SecurityIncident>> {
    const response = await this.client.patch<APIResponse<SecurityIncident>>(
      `/incidents/${incidentId}`,
      updates
    );
    return response.data;
  }

  /**
   * List all incidents
   * @param filters Incident filters
   * @returns Paginated incident list
   */
  async listIncidents(filters?: {
    status?: string;
    severity?: string;
    deviceId?: string;
    startDate?: string;
    endDate?: string;
  }): Promise<APIResponse<PaginatedResponse<SecurityIncident>>> {
    const response = await this.client.get<
      APIResponse<PaginatedResponse<SecurityIncident>>
    >('/incidents', { params: filters });
    return response.data;
  }

  // ============================================================================
  // Utility Methods
  // ============================================================================

  /**
   * Generate device certificate signing request (CSR)
   * @param deviceInfo Device information
   * @param keyPair Key pair (public and private keys)
   * @returns CSR in PEM format
   */
  generateCSR(
    deviceInfo: DeviceIdentity,
    keyPair: { privateKey: string; publicKey: string }
  ): string {
    const privateKey = forge.pki.privateKeyFromPem(keyPair.privateKey);

    const csr = forge.pki.createCertificationRequest();
    csr.publicKey = forge.pki.publicKeyFromPem(keyPair.publicKey);

    csr.setSubject([
      { name: 'commonName', value: deviceInfo.deviceId },
      { name: 'serialNumber', value: deviceInfo.serialNumber },
      { name: 'organizationName', value: deviceInfo.manufacturer },
    ]);

    csr.sign(privateKey, forge.md.sha256.create());

    return forge.pki.certificationRequestToPem(csr);
  }

  /**
   * Generate key pair for device
   * @param algorithm Key algorithm ('RSA' or 'ECDSA')
   * @param keySize Key size (2048 or 4096 for RSA)
   * @returns Key pair
   */
  generateKeyPair(
    algorithm: 'RSA' | 'ECDSA' = 'RSA',
    keySize: number = 2048
  ): { privateKey: string; publicKey: string } {
    if (algorithm === 'RSA') {
      const keypair = forge.pki.rsa.generateKeyPair({ bits: keySize });
      return {
        privateKey: forge.pki.privateKeyToPem(keypair.privateKey),
        publicKey: forge.pki.publicKeyToPem(keypair.publicKey),
      };
    } else {
      // ECDSA key generation would require additional libraries
      throw new Error('ECDSA key generation not implemented in this version');
    }
  }

  /**
   * Calculate SHA-256 hash
   * @param data Data to hash
   * @returns Hex-encoded hash
   */
  calculateHash(data: string | ArrayBuffer): string {
    const md = forge.md.sha256.create();
    if (typeof data === 'string') {
      md.update(data);
    } else {
      md.update(forge.util.createBuffer(data));
    }
    return md.digest().toHex();
  }

  /**
   * Sign data with private key
   * @param data Data to sign
   * @param privateKey Private key in PEM format
   * @returns Base64-encoded signature
   */
  signData(data: string, privateKey: string): string {
    const key = forge.pki.privateKeyFromPem(privateKey);
    const md = forge.md.sha256.create();
    md.update(data, 'utf8');
    const signature = key.sign(md);
    return forge.util.encode64(signature);
  }

  /**
   * Verify signature
   * @param data Original data
   * @param signature Base64-encoded signature
   * @param publicKey Public key in PEM format
   * @returns Verification result
   */
  verifySignature(data: string, signature: string, publicKey: string): boolean {
    try {
      const key = forge.pki.publicKeyFromPem(publicKey);
      const md = forge.md.sha256.create();
      md.update(data, 'utf8');
      const signatureBytes = forge.util.decode64(signature);
      return key.verify(md.digest().bytes(), signatureBytes);
    } catch (error) {
      return false;
    }
  }
}

/**
 * Create a new WIA IoT Security SDK instance
 * @param config SDK configuration
 * @returns SDK instance
 */
export function createClient(config: WIAIoTSecurityConfig): WIAIoTSecurity {
  return new WIAIoTSecurity(config);
}

// Default export
export default WIAIoTSecurity;
