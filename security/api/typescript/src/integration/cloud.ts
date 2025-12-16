/**
 * WIA Security Cloud Integration
 * AWS Security Hub, Azure Sentinel, GCP Security Command Center
 */

import { WiaSecurityEvent, AlertEvent, VulnerabilityEvent } from '../types';

// ============================================================================
// Types
// ============================================================================

export interface CloudSecurityConfig {
  region?: string;
  credentials?: {
    accessKeyId?: string;
    secretAccessKey?: string;
    sessionToken?: string;
  };
  projectId?: string;
  subscriptionId?: string;
  tenantId?: string;
  clientId?: string;
  clientSecret?: string;
}

export interface CloudFinding {
  id: string;
  title: string;
  description: string;
  severity: 'INFORMATIONAL' | 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  type: string;
  resourceType?: string;
  resourceId?: string;
  createdAt: string;
  updatedAt: string;
  status: 'NEW' | 'ACTIVE' | 'RESOLVED' | 'SUPPRESSED';
}

// ============================================================================
// AWS Security Hub Integration
// ============================================================================

export interface AwsSecurityHubConfig extends CloudSecurityConfig {
  accountId: string;
  region: string;
}

export class AwsSecurityHubClient {
  private config: AwsSecurityHubConfig;
  private endpoint: string;

  constructor(config: AwsSecurityHubConfig) {
    this.config = config;
    this.endpoint = `https://securityhub.${config.region}.amazonaws.com`;
  }

  /**
   * Import finding from WIA Security event
   */
  async importFinding(event: WiaSecurityEvent): Promise<string> {
    const finding = this.toAwsFinding(event);

    const body = JSON.stringify({
      Findings: [finding]
    });

    const response = await this.signedRequest('POST', '/findings/import', body);

    if (!response.ok) {
      throw new Error(`AWS Security Hub import error: ${response.status}`);
    }

    return finding.Id;
  }

  /**
   * Batch import findings
   */
  async batchImportFindings(events: WiaSecurityEvent[]): Promise<{
    successCount: number;
    failedCount: number;
  }> {
    const findings = events.map(e => this.toAwsFinding(e));

    const body = JSON.stringify({
      Findings: findings
    });

    const response = await this.signedRequest('POST', '/findings/import', body);

    if (!response.ok) {
      throw new Error(`AWS Security Hub batch import error: ${response.status}`);
    }

    const result = await response.json();
    return {
      successCount: result.SuccessCount || 0,
      failedCount: result.FailedCount || 0
    };
  }

  /**
   * Get findings
   */
  async getFindings(filters?: {
    severityLabel?: string[];
    type?: string[];
    resourceType?: string[];
    maxResults?: number;
  }): Promise<CloudFinding[]> {
    const awsFilters: Record<string, unknown> = {};

    if (filters?.severityLabel) {
      awsFilters.SeverityLabel = filters.severityLabel.map(s => ({
        Value: s,
        Comparison: 'EQUALS'
      }));
    }

    if (filters?.type) {
      awsFilters.Type = filters.type.map(t => ({
        Value: t,
        Comparison: 'PREFIX'
      }));
    }

    const body = JSON.stringify({
      Filters: awsFilters,
      MaxResults: filters?.maxResults || 100
    });

    const response = await this.signedRequest('POST', '/findings', body);

    if (!response.ok) {
      throw new Error(`AWS Security Hub get findings error: ${response.status}`);
    }

    const data = await response.json();
    return (data.Findings || []).map((f: any) => this.fromAwsFinding(f));
  }

  /**
   * Update finding
   */
  async updateFinding(
    findingId: string,
    productArn: string,
    updates: {
      note?: string;
      severity?: string;
      status?: 'NEW' | 'NOTIFIED' | 'RESOLVED' | 'SUPPRESSED';
    }
  ): Promise<void> {
    const findingIdentifier = {
      Id: findingId,
      ProductArn: productArn
    };

    const update: Record<string, unknown> = {};

    if (updates.note) {
      update.Note = {
        Text: updates.note,
        UpdatedBy: 'WIA Security'
      };
    }

    if (updates.severity) {
      update.Severity = {
        Label: updates.severity
      };
    }

    if (updates.status) {
      update.Workflow = {
        Status: updates.status
      };
    }

    const body = JSON.stringify({
      FindingIdentifiers: [findingIdentifier],
      ...update
    });

    const response = await this.signedRequest('PATCH', '/findings', body);

    if (!response.ok) {
      throw new Error(`AWS Security Hub update error: ${response.status}`);
    }
  }

  private toAwsFinding(event: WiaSecurityEvent): Record<string, unknown> {
    const now = new Date().toISOString();
    const productArn = `arn:aws:securityhub:${this.config.region}:${this.config.accountId}:product/${this.config.accountId}/wia-security`;

    return {
      SchemaVersion: '2018-10-08',
      Id: event.id,
      ProductArn: productArn,
      GeneratorId: 'wia-security',
      AwsAccountId: this.config.accountId,
      Types: [this.mapEventType(event.type)],
      CreatedAt: event.timestamp,
      UpdatedAt: now,
      Severity: {
        Label: this.mapSeverity(event.severity),
        Original: event.severity.toString()
      },
      Title: event.description.substring(0, 256),
      Description: event.description,
      Resources: this.extractResources(event),
      Workflow: {
        Status: 'NEW'
      },
      RecordState: 'ACTIVE'
    };
  }

  private fromAwsFinding(finding: any): CloudFinding {
    return {
      id: finding.Id,
      title: finding.Title,
      description: finding.Description,
      severity: finding.Severity?.Label || 'MEDIUM',
      type: finding.Types?.[0] || 'Unknown',
      resourceType: finding.Resources?.[0]?.Type,
      resourceId: finding.Resources?.[0]?.Id,
      createdAt: finding.CreatedAt,
      updatedAt: finding.UpdatedAt,
      status: finding.Workflow?.Status || 'NEW'
    };
  }

  private mapEventType(type: string): string {
    const mapping: Record<string, string> = {
      'alert': 'Software and Configuration Checks/Vulnerabilities/CVE',
      'threat_intel': 'TTPs/Initial Access',
      'vulnerability': 'Software and Configuration Checks/Vulnerabilities/CVE',
      'incident': 'Unusual Behaviors/VM/Intrusion',
      'network_event': 'Effects/Network Effects',
      'endpoint_event': 'Unusual Behaviors/Process',
      'auth_event': 'Sensitive Data Identifications/Authentication'
    };
    return mapping[type] || 'Other';
  }

  private mapSeverity(severity: number): string {
    if (severity >= 9) return 'CRITICAL';
    if (severity >= 7) return 'HIGH';
    if (severity >= 4) return 'MEDIUM';
    if (severity >= 1) return 'LOW';
    return 'INFORMATIONAL';
  }

  private extractResources(event: WiaSecurityEvent): unknown[] {
    const resources: unknown[] = [];

    if (event.context?.host) {
      resources.push({
        Type: 'AwsEc2Instance',
        Id: event.context.host.hostname || event.context.host.ip?.[0] || 'unknown',
        Details: {
          Other: {
            hostname: event.context.host.hostname,
            ip: event.context.host.ip?.join(',')
          }
        }
      });
    }

    if (resources.length === 0) {
      resources.push({
        Type: 'Other',
        Id: event.id
      });
    }

    return resources;
  }

  private async signedRequest(
    method: string,
    path: string,
    body?: string
  ): Promise<Response> {
    // In production, use AWS SDK or proper Signature V4 signing
    // This is a simplified placeholder
    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
      'Host': new URL(this.endpoint).host,
      'X-Amz-Date': new Date().toISOString().replace(/[:-]|\.\d{3}/g, '')
    };

    if (this.config.credentials?.sessionToken) {
      headers['X-Amz-Security-Token'] = this.config.credentials.sessionToken;
    }

    return fetch(`${this.endpoint}${path}`, {
      method,
      headers,
      body
    });
  }
}

// ============================================================================
// Azure Sentinel Integration
// ============================================================================

export interface AzureSentinelConfig extends CloudSecurityConfig {
  workspaceId: string;
  subscriptionId: string;
  resourceGroup: string;
}

export class AzureSentinelClient {
  private config: AzureSentinelConfig;
  private accessToken: string | null = null;
  private tokenExpiry: Date | null = null;

  constructor(config: AzureSentinelConfig) {
    this.config = config;
  }

  /**
   * Create incident from WIA Security event
   */
  async createIncident(event: WiaSecurityEvent): Promise<string> {
    await this.ensureToken();

    const incidentId = `wia-${event.id}`;
    const url = this.buildUrl(`/incidents/${incidentId}`);

    const incident = {
      properties: {
        title: event.description.substring(0, 256),
        description: event.description,
        severity: this.mapSeverity(event.severity),
        status: 'New',
        labels: [
          { labelName: 'wia-security' },
          { labelName: event.type }
        ]
      }
    };

    const response = await fetch(url, {
      method: 'PUT',
      headers: {
        'Authorization': `Bearer ${this.accessToken}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(incident)
    });

    if (!response.ok) {
      throw new Error(`Azure Sentinel create incident error: ${response.status}`);
    }

    const result = await response.json();
    return result.name;
  }

  /**
   * List incidents
   */
  async listIncidents(filters?: {
    severity?: string[];
    status?: string[];
    top?: number;
  }): Promise<CloudFinding[]> {
    await this.ensureToken();

    let url = this.buildUrl('/incidents');
    const params = new URLSearchParams();

    if (filters?.top) {
      params.set('$top', filters.top.toString());
    }

    if (filters?.severity) {
      const filter = filters.severity.map(s => `properties/severity eq '${s}'`).join(' or ');
      params.set('$filter', filter);
    }

    if (params.toString()) {
      url += `?${params.toString()}`;
    }

    const response = await fetch(url, {
      headers: {
        'Authorization': `Bearer ${this.accessToken}`
      }
    });

    if (!response.ok) {
      throw new Error(`Azure Sentinel list incidents error: ${response.status}`);
    }

    const data = await response.json();
    return (data.value || []).map((i: any) => this.fromAzureIncident(i));
  }

  /**
   * Update incident
   */
  async updateIncident(
    incidentId: string,
    updates: {
      status?: 'New' | 'Active' | 'Closed';
      severity?: 'Informational' | 'Low' | 'Medium' | 'High';
      owner?: string;
      classification?: string;
    }
  ): Promise<void> {
    await this.ensureToken();

    const url = this.buildUrl(`/incidents/${incidentId}`);

    // Get current incident
    const getResponse = await fetch(url, {
      headers: {
        'Authorization': `Bearer ${this.accessToken}`
      }
    });

    if (!getResponse.ok) {
      throw new Error(`Azure Sentinel get incident error: ${getResponse.status}`);
    }

    const current = await getResponse.json();

    // Update properties
    if (updates.status) current.properties.status = updates.status;
    if (updates.severity) current.properties.severity = updates.severity;
    if (updates.classification) current.properties.classification = updates.classification;
    if (updates.owner) {
      current.properties.owner = {
        assignedTo: updates.owner
      };
    }

    const response = await fetch(url, {
      method: 'PUT',
      headers: {
        'Authorization': `Bearer ${this.accessToken}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(current)
    });

    if (!response.ok) {
      throw new Error(`Azure Sentinel update incident error: ${response.status}`);
    }
  }

  /**
   * Add comment to incident
   */
  async addIncidentComment(incidentId: string, message: string): Promise<void> {
    await this.ensureToken();

    const commentId = `wia-comment-${Date.now()}`;
    const url = this.buildUrl(`/incidents/${incidentId}/comments/${commentId}`);

    const response = await fetch(url, {
      method: 'PUT',
      headers: {
        'Authorization': `Bearer ${this.accessToken}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        properties: {
          message
        }
      })
    });

    if (!response.ok) {
      throw new Error(`Azure Sentinel add comment error: ${response.status}`);
    }
  }

  private fromAzureIncident(incident: any): CloudFinding {
    const props = incident.properties || {};
    return {
      id: incident.name,
      title: props.title,
      description: props.description,
      severity: props.severity?.toUpperCase() || 'MEDIUM',
      type: 'AzureSentinelIncident',
      createdAt: props.createdTimeUtc,
      updatedAt: props.lastModifiedTimeUtc,
      status: this.mapAzureStatus(props.status)
    };
  }

  private mapSeverity(severity: number): string {
    if (severity >= 9) return 'High';
    if (severity >= 6) return 'Medium';
    if (severity >= 3) return 'Low';
    return 'Informational';
  }

  private mapAzureStatus(status: string): 'NEW' | 'ACTIVE' | 'RESOLVED' | 'SUPPRESSED' {
    switch (status?.toLowerCase()) {
      case 'new': return 'NEW';
      case 'active': return 'ACTIVE';
      case 'closed': return 'RESOLVED';
      default: return 'NEW';
    }
  }

  private buildUrl(path: string): string {
    return `https://management.azure.com/subscriptions/${this.config.subscriptionId}/resourceGroups/${this.config.resourceGroup}/providers/Microsoft.OperationalInsights/workspaces/${this.config.workspaceId}/providers/Microsoft.SecurityInsights${path}?api-version=2023-02-01`;
  }

  private async ensureToken(): Promise<void> {
    if (this.accessToken && this.tokenExpiry && this.tokenExpiry > new Date()) {
      return;
    }

    const tokenUrl = `https://login.microsoftonline.com/${this.config.tenantId}/oauth2/v2.0/token`;

    const params = new URLSearchParams({
      client_id: this.config.clientId || '',
      client_secret: this.config.clientSecret || '',
      scope: 'https://management.azure.com/.default',
      grant_type: 'client_credentials'
    });

    const response = await fetch(tokenUrl, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/x-www-form-urlencoded'
      },
      body: params.toString()
    });

    if (!response.ok) {
      throw new Error(`Azure auth error: ${response.status}`);
    }

    const data = await response.json();
    this.accessToken = data.access_token;
    this.tokenExpiry = new Date(Date.now() + (data.expires_in - 60) * 1000);
  }
}

// ============================================================================
// GCP Security Command Center Integration
// ============================================================================

export interface GcpSccConfig extends CloudSecurityConfig {
  projectId: string;
  organizationId: string;
  sourceId?: string;
}

export class GcpSccClient {
  private config: GcpSccConfig;
  private accessToken: string | null = null;
  private tokenExpiry: Date | null = null;

  constructor(config: GcpSccConfig) {
    this.config = config;
  }

  /**
   * Create finding from WIA Security event
   */
  async createFinding(event: WiaSecurityEvent): Promise<string> {
    await this.ensureToken();

    const sourceId = this.config.sourceId || 'wia-security';
    const findingId = event.id.replace(/[^a-zA-Z0-9_-]/g, '_');
    const parent = `organizations/${this.config.organizationId}/sources/${sourceId}`;
    const url = `https://securitycenter.googleapis.com/v1/${parent}/findings?findingId=${findingId}`;

    const finding = {
      state: 'ACTIVE',
      category: event.type,
      severity: this.mapSeverity(event.severity),
      eventTime: event.timestamp,
      sourceProperties: {
        wia_event_id: { stringValue: event.id },
        wia_event_type: { stringValue: event.type },
        description: { stringValue: event.description }
      }
    };

    const response = await fetch(url, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${this.accessToken}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(finding)
    });

    if (!response.ok) {
      throw new Error(`GCP SCC create finding error: ${response.status}`);
    }

    const result = await response.json();
    return result.name;
  }

  /**
   * List findings
   */
  async listFindings(filters?: {
    category?: string;
    severity?: string[];
    state?: 'ACTIVE' | 'INACTIVE';
    pageSize?: number;
  }): Promise<CloudFinding[]> {
    await this.ensureToken();

    const parent = `organizations/${this.config.organizationId}/sources/-`;
    let url = `https://securitycenter.googleapis.com/v1/${parent}/findings`;

    const params = new URLSearchParams();

    if (filters?.pageSize) {
      params.set('pageSize', filters.pageSize.toString());
    }

    const filterParts: string[] = [];
    if (filters?.category) {
      filterParts.push(`category="${filters.category}"`);
    }
    if (filters?.state) {
      filterParts.push(`state="${filters.state}"`);
    }
    if (filters?.severity) {
      const sevFilter = filters.severity.map(s => `severity="${s}"`).join(' OR ');
      filterParts.push(`(${sevFilter})`);
    }

    if (filterParts.length > 0) {
      params.set('filter', filterParts.join(' AND '));
    }

    if (params.toString()) {
      url += `?${params.toString()}`;
    }

    const response = await fetch(url, {
      headers: {
        'Authorization': `Bearer ${this.accessToken}`
      }
    });

    if (!response.ok) {
      throw new Error(`GCP SCC list findings error: ${response.status}`);
    }

    const data = await response.json();
    return (data.listFindingsResults || []).map((r: any) => this.fromGcpFinding(r.finding));
  }

  /**
   * Update finding state
   */
  async updateFindingState(
    findingName: string,
    state: 'ACTIVE' | 'INACTIVE'
  ): Promise<void> {
    await this.ensureToken();

    const url = `https://securitycenter.googleapis.com/v1/${findingName}:setState`;

    const response = await fetch(url, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${this.accessToken}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        state,
        startTime: new Date().toISOString()
      })
    });

    if (!response.ok) {
      throw new Error(`GCP SCC update state error: ${response.status}`);
    }
  }

  /**
   * Add security marks
   */
  async addSecurityMarks(
    findingName: string,
    marks: Record<string, string>
  ): Promise<void> {
    await this.ensureToken();

    const url = `https://securitycenter.googleapis.com/v1/${findingName}/securityMarks`;

    const response = await fetch(url, {
      method: 'PATCH',
      headers: {
        'Authorization': `Bearer ${this.accessToken}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        marks
      })
    });

    if (!response.ok) {
      throw new Error(`GCP SCC add marks error: ${response.status}`);
    }
  }

  private fromGcpFinding(finding: any): CloudFinding {
    return {
      id: finding.name,
      title: finding.category,
      description: finding.sourceProperties?.description?.stringValue || '',
      severity: finding.severity || 'MEDIUM',
      type: finding.category,
      resourceType: finding.resourceName?.split('/')[3],
      resourceId: finding.resourceName,
      createdAt: finding.createTime,
      updatedAt: finding.eventTime,
      status: finding.state === 'ACTIVE' ? 'ACTIVE' : 'RESOLVED'
    };
  }

  private mapSeverity(severity: number): string {
    if (severity >= 9) return 'CRITICAL';
    if (severity >= 7) return 'HIGH';
    if (severity >= 4) return 'MEDIUM';
    return 'LOW';
  }

  private async ensureToken(): Promise<void> {
    if (this.accessToken && this.tokenExpiry && this.tokenExpiry > new Date()) {
      return;
    }

    // In production, use Google Cloud SDK or service account credentials
    // This is a placeholder for the token refresh logic
    const metadataUrl = 'http://metadata.google.internal/computeMetadata/v1/instance/service-accounts/default/token';

    try {
      const response = await fetch(metadataUrl, {
        headers: {
          'Metadata-Flavor': 'Google'
        }
      });

      if (response.ok) {
        const data = await response.json();
        this.accessToken = data.access_token;
        this.tokenExpiry = new Date(Date.now() + (data.expires_in - 60) * 1000);
      }
    } catch {
      // Not running on GCP, need external auth
      throw new Error('GCP authentication required. Please provide credentials.');
    }
  }
}

// ============================================================================
// Factory Functions
// ============================================================================

export function createAwsSecurityHubClient(config: AwsSecurityHubConfig): AwsSecurityHubClient {
  return new AwsSecurityHubClient(config);
}

export function createAzureSentinelClient(config: AzureSentinelConfig): AzureSentinelClient {
  return new AzureSentinelClient(config);
}

export function createGcpSccClient(config: GcpSccConfig): GcpSccClient {
  return new GcpSccClient(config);
}
