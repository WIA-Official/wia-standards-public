/**
 * WIA-MED-016: Medical Device Security Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import EventEmitter from 'eventemitter3';
import {
  WIAConfig,
  APIResponse,
  PaginatedResponse,
  MedicalDevice,
  Vulnerability,
  SecurityIncident,
  SecurityPatch,
  ComplianceCheck,
  ThreatLevel,
  DeviceStatus,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIADeviceSecurityConfig extends WIAConfig {
  facilityId?: string;
}

// ============================================================================
// Main SDK Client
// ============================================================================

export class WIADeviceSecurityClient {
  private config: Required<WIADeviceSecurityConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIADeviceSecurityConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/device-security',
      timeout: 60000,
      debug: false,
      facilityId: '',
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Device Operations
  // ==========================================================================

  async getDevice(deviceId: string): Promise<APIResponse<MedicalDevice>> {
    return this.makeRequest('GET', `/devices/${deviceId}`);
  }

  async listDevices(filters?: {
    status?: DeviceStatus;
    manufacturer?: string;
    deviceClass?: string;
  }): Promise<PaginatedResponse<MedicalDevice>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.makeRequest('GET', `/devices?${params}`);
  }

  async registerDevice(device: Omit<MedicalDevice, 'deviceId' | 'lastScan'>): Promise<APIResponse<MedicalDevice>> {
    return this.makeRequest('POST', '/devices', device);
  }

  async updateDeviceStatus(deviceId: string, status: DeviceStatus): Promise<APIResponse<MedicalDevice>> {
    return this.makeRequest('PATCH', `/devices/${deviceId}/status`, { status });
  }

  async quarantineDevice(deviceId: string, reason: string): Promise<APIResponse<MedicalDevice>> {
    return this.makeRequest('POST', `/devices/${deviceId}/quarantine`, { reason });
  }

  // ==========================================================================
  // Vulnerability Operations
  // ==========================================================================

  async getVulnerabilities(deviceId: string): Promise<PaginatedResponse<Vulnerability>> {
    return this.makeRequest('GET', `/devices/${deviceId}/vulnerabilities`);
  }

  async reportVulnerability(vulnerability: Omit<Vulnerability, 'vulnerabilityId' | 'discoveredAt'>): Promise<APIResponse<Vulnerability>> {
    return this.makeRequest('POST', '/vulnerabilities', vulnerability);
  }

  async updateVulnerabilityStatus(vulnerabilityId: string, status: string, notes?: string): Promise<APIResponse<Vulnerability>> {
    return this.makeRequest('PATCH', `/vulnerabilities/${vulnerabilityId}`, { status, notes });
  }

  async getVulnerabilityByCSE(cveId: string): Promise<APIResponse<Vulnerability[]>> {
    return this.makeRequest('GET', `/vulnerabilities/cve/${cveId}`);
  }

  // ==========================================================================
  // Incident Operations
  // ==========================================================================

  async reportIncident(incident: Omit<SecurityIncident, 'incidentId' | 'detectedAt'>): Promise<APIResponse<SecurityIncident>> {
    return this.makeRequest('POST', '/incidents', incident);
  }

  async getIncident(incidentId: string): Promise<APIResponse<SecurityIncident>> {
    return this.makeRequest('GET', `/incidents/${incidentId}`);
  }

  async listIncidents(filters?: {
    deviceId?: string;
    threatLevel?: ThreatLevel;
    resolved?: boolean;
  }): Promise<PaginatedResponse<SecurityIncident>> {
    const params = new URLSearchParams(filters as Record<string, string>);
    return this.makeRequest('GET', `/incidents?${params}`);
  }

  async resolveIncident(incidentId: string, resolution: {
    resolvedBy: string;
    actions: string[];
    notes: string;
  }): Promise<APIResponse<SecurityIncident>> {
    return this.makeRequest('POST', `/incidents/${incidentId}/resolve`, resolution);
  }

  // ==========================================================================
  // Patch Operations
  // ==========================================================================

  async getAvailablePatches(deviceId: string): Promise<PaginatedResponse<SecurityPatch>> {
    return this.makeRequest('GET', `/devices/${deviceId}/patches`);
  }

  async schedulePatch(deviceId: string, patchId: string, scheduledTime: string): Promise<APIResponse<SecurityPatch>> {
    return this.makeRequest('POST', `/devices/${deviceId}/patches/${patchId}/schedule`, { scheduledTime });
  }

  async applyPatch(deviceId: string, patchId: string): Promise<APIResponse<SecurityPatch>> {
    return this.makeRequest('POST', `/devices/${deviceId}/patches/${patchId}/apply`);
  }

  // ==========================================================================
  // Compliance Operations
  // ==========================================================================

  async runComplianceCheck(deviceId: string, framework: string): Promise<APIResponse<ComplianceCheck>> {
    return this.makeRequest('POST', `/devices/${deviceId}/compliance/check`, { framework });
  }

  async getComplianceHistory(deviceId: string): Promise<PaginatedResponse<ComplianceCheck>> {
    return this.makeRequest('GET', `/devices/${deviceId}/compliance/history`);
  }

  // ==========================================================================
  // Scanning Operations
  // ==========================================================================

  async scanDevice(deviceId: string): Promise<APIResponse<{
    scanId: string;
    status: string;
    startedAt: string;
  }>> {
    return this.makeRequest('POST', `/devices/${deviceId}/scan`);
  }

  async getScanResults(deviceId: string, scanId: string): Promise<APIResponse<{
    vulnerabilities: Vulnerability[];
    complianceIssues: string[];
    riskScore: number;
  }>> {
    return this.makeRequest('GET', `/devices/${deviceId}/scans/${scanId}`);
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'vulnerabilityDetected' | 'incidentReported' | 'patchAvailable' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async makeRequest<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Device Security] ${method} ${url}`, body);
    }

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'MED-016',
          'X-WIA-Version': '1.0.0',
          ...(this.config.facilityId && { 'X-Facility-ID': this.config.facilityId }),
        },
        body: body ? JSON.stringify(body) : undefined,
      });

      return await response.json();
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'Unknown error';
      this.eventEmitter.emit('error', { code: 'REQUEST_FAILED', message });
      throw error;
    }
  }
}

export function createClient(config: WIADeviceSecurityConfig): WIADeviceSecurityClient {
  return new WIADeviceSecurityClient(config);
}

export default WIADeviceSecurityClient;
