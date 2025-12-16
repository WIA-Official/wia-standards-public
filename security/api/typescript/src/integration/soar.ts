/**
 * WIA Security SOAR Integration
 * Security Orchestration, Automation, and Response
 * Phantom, XSOAR, Shuffle
 */

import { WiaSecurityEvent, IncidentEvent, AlertEvent } from '../types';

// ============================================================================
// Types
// ============================================================================

export interface SoarConfig {
  url: string;
  apiKey?: string;
  username?: string;
  password?: string;
  timeout?: number;
}

export interface PlaybookAction {
  id: string;
  name: string;
  app: string;
  action: string;
  parameters?: Record<string, unknown>;
  status?: 'pending' | 'running' | 'success' | 'failed';
  result?: unknown;
}

export interface Playbook {
  id: string;
  name: string;
  description?: string;
  actions: PlaybookAction[];
  triggers?: string[];
  enabled?: boolean;
}

export interface SoarCase {
  id: string;
  name: string;
  status: 'new' | 'open' | 'in_progress' | 'resolved' | 'closed';
  severity: 'low' | 'medium' | 'high' | 'critical';
  owner?: string;
  artifacts?: Artifact[];
  events?: string[];
  createdAt: string;
  updatedAt: string;
}

export interface Artifact {
  id: string;
  type: string;
  value: string;
  tags?: string[];
  source?: string;
}

// ============================================================================
// Splunk Phantom (SOAR) Integration
// ============================================================================

export interface PhantomConfig extends SoarConfig {
  verifyTls?: boolean;
}

export class PhantomClient {
  private config: PhantomConfig;
  private headers: Record<string, string>;

  constructor(config: PhantomConfig) {
    this.config = {
      timeout: 30000,
      verifyTls: true,
      ...config
    };

    this.headers = {
      'ph-auth-token': config.apiKey || '',
      'Content-Type': 'application/json'
    };
  }

  /**
   * Create container (case)
   */
  async createContainer(event: WiaSecurityEvent): Promise<string> {
    const container = {
      name: event.description,
      label: 'wia_security',
      severity: this.mapSeverity(event.severity),
      status: 'new',
      source_data_identifier: event.id,
      custom_fields: {
        wia_event_type: event.type,
        wia_event_id: event.id
      }
    };

    const response = await fetch(`${this.config.url}/rest/container`, {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify(container),
      signal: AbortSignal.timeout(this.config.timeout || 30000)
    });

    if (!response.ok) {
      throw new Error(`Phantom container error: ${response.status}`);
    }

    const result = await response.json();
    return result.id.toString();
  }

  /**
   * Add artifact to container
   */
  async addArtifact(containerId: string, artifact: Artifact): Promise<string> {
    const phantomArtifact = {
      container_id: parseInt(containerId),
      name: artifact.type,
      source_data_identifier: artifact.id,
      cef: {
        [artifact.type]: artifact.value
      },
      tags: artifact.tags
    };

    const response = await fetch(`${this.config.url}/rest/artifact`, {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify(phantomArtifact),
      signal: AbortSignal.timeout(this.config.timeout || 30000)
    });

    if (!response.ok) {
      throw new Error(`Phantom artifact error: ${response.status}`);
    }

    const result = await response.json();
    return result.id.toString();
  }

  /**
   * Run playbook
   */
  async runPlaybook(
    playbookName: string,
    containerId: string,
    scope: 'all' | 'new' = 'all'
  ): Promise<string> {
    const response = await fetch(`${this.config.url}/rest/playbook_run`, {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify({
        container_id: parseInt(containerId),
        playbook_id: playbookName,
        scope
      }),
      signal: AbortSignal.timeout(this.config.timeout || 30000)
    });

    if (!response.ok) {
      throw new Error(`Phantom playbook error: ${response.status}`);
    }

    const result = await response.json();
    return result.playbook_run_id.toString();
  }

  /**
   * Get playbook run status
   */
  async getPlaybookStatus(runId: string): Promise<{ status: string; message?: string }> {
    const response = await fetch(
      `${this.config.url}/rest/playbook_run/${runId}`,
      {
        headers: this.headers,
        signal: AbortSignal.timeout(this.config.timeout || 30000)
      }
    );

    if (!response.ok) {
      throw new Error(`Phantom status error: ${response.status}`);
    }

    const result = await response.json();
    return {
      status: result.status,
      message: result.message
    };
  }

  /**
   * Get container
   */
  async getContainer(containerId: string): Promise<SoarCase> {
    const response = await fetch(
      `${this.config.url}/rest/container/${containerId}`,
      {
        headers: this.headers,
        signal: AbortSignal.timeout(this.config.timeout || 30000)
      }
    );

    if (!response.ok) {
      throw new Error(`Phantom container error: ${response.status}`);
    }

    const data = await response.json();
    return {
      id: data.id.toString(),
      name: data.name,
      status: this.mapPhantomStatus(data.status),
      severity: this.reverseMapSeverity(data.severity),
      owner: data.owner_name,
      createdAt: data.create_time,
      updatedAt: data.update_time
    };
  }

  /**
   * Update container status
   */
  async updateContainerStatus(
    containerId: string,
    status: 'new' | 'open' | 'in_progress' | 'resolved' | 'closed'
  ): Promise<void> {
    const response = await fetch(
      `${this.config.url}/rest/container/${containerId}`,
      {
        method: 'POST',
        headers: this.headers,
        body: JSON.stringify({ status: this.mapStatusToPhantom(status) }),
        signal: AbortSignal.timeout(this.config.timeout || 30000)
      }
    );

    if (!response.ok) {
      throw new Error(`Phantom update error: ${response.status}`);
    }
  }

  private mapSeverity(severity: number): string {
    if (severity >= 9) return 'critical';
    if (severity >= 7) return 'high';
    if (severity >= 4) return 'medium';
    return 'low';
  }

  private reverseMapSeverity(severity: string): 'low' | 'medium' | 'high' | 'critical' {
    const map: Record<string, 'low' | 'medium' | 'high' | 'critical'> = {
      'low': 'low',
      'medium': 'medium',
      'high': 'high',
      'critical': 'critical'
    };
    return map[severity] || 'medium';
  }

  private mapPhantomStatus(status: string): 'new' | 'open' | 'in_progress' | 'resolved' | 'closed' {
    const map: Record<string, 'new' | 'open' | 'in_progress' | 'resolved' | 'closed'> = {
      'new': 'new',
      'open': 'open',
      'in progress': 'in_progress',
      'resolved': 'resolved',
      'closed': 'closed'
    };
    return map[status.toLowerCase()] || 'open';
  }

  private mapStatusToPhantom(status: string): string {
    const map: Record<string, string> = {
      'new': 'new',
      'open': 'open',
      'in_progress': 'in progress',
      'resolved': 'resolved',
      'closed': 'closed'
    };
    return map[status] || 'open';
  }
}

// ============================================================================
// Palo Alto XSOAR Integration
// ============================================================================

export interface XsoarConfig extends SoarConfig {}

export class XsoarClient {
  private config: XsoarConfig;
  private headers: Record<string, string>;

  constructor(config: XsoarConfig) {
    this.config = {
      timeout: 30000,
      ...config
    };

    this.headers = {
      'Authorization': config.apiKey || '',
      'Content-Type': 'application/json',
      'Accept': 'application/json'
    };
  }

  /**
   * Create incident
   */
  async createIncident(event: WiaSecurityEvent): Promise<string> {
    const incident = {
      name: event.description,
      type: 'WIA Security Event',
      severity: this.mapSeverity(event.severity),
      labels: [
        { type: 'wia_event_type', value: event.type },
        { type: 'wia_event_id', value: event.id }
      ],
      rawJSON: JSON.stringify(event)
    };

    const response = await fetch(`${this.config.url}/incident`, {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify(incident),
      signal: AbortSignal.timeout(this.config.timeout || 30000)
    });

    if (!response.ok) {
      throw new Error(`XSOAR incident error: ${response.status}`);
    }

    const result = await response.json();
    return result.id;
  }

  /**
   * Add indicator
   */
  async addIndicator(indicator: {
    value: string;
    type: string;
    score: number;
    source: string;
  }): Promise<string> {
    const response = await fetch(`${this.config.url}/indicator/create`, {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify({
        indicator: {
          value: indicator.value,
          indicator_type: this.mapIndicatorType(indicator.type),
          score: indicator.score,
          source: indicator.source
        }
      }),
      signal: AbortSignal.timeout(this.config.timeout || 30000)
    });

    if (!response.ok) {
      throw new Error(`XSOAR indicator error: ${response.status}`);
    }

    const result = await response.json();
    return result.id;
  }

  /**
   * Run playbook
   */
  async runPlaybook(playbookId: string, incidentId: string): Promise<string> {
    const response = await fetch(`${this.config.url}/incident/investigate`, {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify({
        id: incidentId,
        playbookId
      }),
      signal: AbortSignal.timeout(this.config.timeout || 30000)
    });

    if (!response.ok) {
      throw new Error(`XSOAR playbook error: ${response.status}`);
    }

    const result = await response.json();
    return result.investigationId;
  }

  /**
   * Get incident
   */
  async getIncident(incidentId: string): Promise<SoarCase> {
    const response = await fetch(
      `${this.config.url}/incident/load/${incidentId}`,
      {
        headers: this.headers,
        signal: AbortSignal.timeout(this.config.timeout || 30000)
      }
    );

    if (!response.ok) {
      throw new Error(`XSOAR incident error: ${response.status}`);
    }

    const data = await response.json();
    return {
      id: data.id,
      name: data.name,
      status: this.mapXsoarStatus(data.status),
      severity: this.reverseMapSeverity(data.severity),
      owner: data.owner,
      createdAt: data.created,
      updatedAt: data.modified
    };
  }

  /**
   * Close incident
   */
  async closeIncident(
    incidentId: string,
    closeReason: string,
    closeNotes?: string
  ): Promise<void> {
    const response = await fetch(`${this.config.url}/incident/close`, {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify({
        id: incidentId,
        closeReason,
        closeNotes
      }),
      signal: AbortSignal.timeout(this.config.timeout || 30000)
    });

    if (!response.ok) {
      throw new Error(`XSOAR close error: ${response.status}`);
    }
  }

  private mapSeverity(severity: number): number {
    if (severity >= 9) return 4; // Critical
    if (severity >= 7) return 3; // High
    if (severity >= 4) return 2; // Medium
    return 1; // Low
  }

  private reverseMapSeverity(severity: number): 'low' | 'medium' | 'high' | 'critical' {
    switch (severity) {
      case 4: return 'critical';
      case 3: return 'high';
      case 2: return 'medium';
      default: return 'low';
    }
  }

  private mapXsoarStatus(status: number): 'new' | 'open' | 'in_progress' | 'resolved' | 'closed' {
    switch (status) {
      case 0: return 'new';
      case 1: return 'open';
      case 2: return 'in_progress';
      case 3: return 'closed';
      default: return 'open';
    }
  }

  private mapIndicatorType(type: string): string {
    const map: Record<string, string> = {
      'ipv4': 'IP',
      'ipv6': 'IPv6',
      'domain': 'Domain',
      'url': 'URL',
      'email': 'Email',
      'md5': 'File MD5',
      'sha1': 'File SHA-1',
      'sha256': 'File SHA-256'
    };
    return map[type] || type;
  }
}

// ============================================================================
// Shuffle SOAR Integration
// ============================================================================

export interface ShuffleConfig extends SoarConfig {}

export class ShuffleClient {
  private config: ShuffleConfig;
  private headers: Record<string, string>;

  constructor(config: ShuffleConfig) {
    this.config = {
      timeout: 30000,
      ...config
    };

    this.headers = {
      'Authorization': `Bearer ${config.apiKey}`,
      'Content-Type': 'application/json'
    };
  }

  /**
   * Execute workflow
   */
  async executeWorkflow(
    workflowId: string,
    event: WiaSecurityEvent
  ): Promise<string> {
    const response = await fetch(
      `${this.config.url}/api/v1/workflows/${workflowId}/execute`,
      {
        method: 'POST',
        headers: this.headers,
        body: JSON.stringify({
          execution_argument: JSON.stringify(event),
          start: ''
        }),
        signal: AbortSignal.timeout(this.config.timeout || 30000)
      }
    );

    if (!response.ok) {
      throw new Error(`Shuffle workflow error: ${response.status}`);
    }

    const result = await response.json();
    return result.execution_id;
  }

  /**
   * Get execution status
   */
  async getExecutionStatus(executionId: string): Promise<{
    status: string;
    results?: unknown[];
  }> {
    const response = await fetch(
      `${this.config.url}/api/v1/streams/results/${executionId}`,
      {
        headers: this.headers,
        signal: AbortSignal.timeout(this.config.timeout || 30000)
      }
    );

    if (!response.ok) {
      throw new Error(`Shuffle execution error: ${response.status}`);
    }

    const data = await response.json();
    return {
      status: data.status,
      results: data.results
    };
  }

  /**
   * List workflows
   */
  async listWorkflows(): Promise<Playbook[]> {
    const response = await fetch(
      `${this.config.url}/api/v1/workflows`,
      {
        headers: this.headers,
        signal: AbortSignal.timeout(this.config.timeout || 30000)
      }
    );

    if (!response.ok) {
      throw new Error(`Shuffle workflows error: ${response.status}`);
    }

    const data = await response.json();
    return data.map((wf: any) => ({
      id: wf.id,
      name: wf.name,
      description: wf.description,
      actions: (wf.actions || []).map((a: any) => ({
        id: a.id,
        name: a.name,
        app: a.app_name,
        action: a.name,
        parameters: a.parameters
      })),
      triggers: (wf.triggers || []).map((t: any) => t.name),
      enabled: wf.is_valid
    }));
  }

  /**
   * Create webhook trigger
   */
  async createWebhook(
    workflowId: string,
    name: string
  ): Promise<{ id: string; url: string }> {
    const response = await fetch(
      `${this.config.url}/api/v1/hooks/new`,
      {
        method: 'POST',
        headers: this.headers,
        body: JSON.stringify({
          name,
          type: 'webhook',
          workflow: workflowId
        }),
        signal: AbortSignal.timeout(this.config.timeout || 30000)
      }
    );

    if (!response.ok) {
      throw new Error(`Shuffle webhook error: ${response.status}`);
    }

    const data = await response.json();
    return {
      id: data.id,
      url: `${this.config.url}/api/v1/hooks/webhook_${data.id}`
    };
  }
}

// ============================================================================
// Factory Functions
// ============================================================================

export function createPhantomClient(config: PhantomConfig): PhantomClient {
  return new PhantomClient(config);
}

export function createXsoarClient(config: XsoarConfig): XsoarClient {
  return new XsoarClient(config);
}

export function createShuffleClient(config: ShuffleConfig): ShuffleClient {
  return new ShuffleClient(config);
}
