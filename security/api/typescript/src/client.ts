/**
 * WIA Security Client
 * Main client for interacting with WIA Security events
 */

import { v4 as uuidv4 } from 'uuid';
import {
  WiaSecurityEvent,
  EventType,
  Source,
  AlertEvent,
  ThreatIntelEvent,
  VulnerabilityEvent,
  IncidentEvent,
  NetworkEvent,
  EndpointEvent,
  AuthEvent
} from './types';
import { validateEvent, ValidationResult } from './validator';
import {
  createAlert,
  createThreatIntel,
  createVulnerability,
  createIncident,
  createNetworkEvent,
  createEndpointEvent,
  createAuthEvent
} from './builder';
import { toStixBundle, toEcsEvent, toOcsfEvent, toSplunkEvent, toElasticEvent } from './converter';

// ============================================================================
// Client Configuration
// ============================================================================

export interface SecurityClientConfig {
  /** Default source for events */
  defaultSource?: Source;
  /** Auto-generate IDs if not provided */
  autoGenerateIds?: boolean;
  /** Auto-set timestamps if not provided */
  autoTimestamp?: boolean;
  /** Validate events before operations */
  validateOnCreate?: boolean;
  /** Event handlers */
  handlers?: {
    onEventCreated?: (event: WiaSecurityEvent) => void;
    onValidationError?: (errors: string[]) => void;
  };
}

// ============================================================================
// Event Storage Interface
// ============================================================================

export interface EventStore {
  save(event: WiaSecurityEvent): Promise<void>;
  get(id: string): Promise<WiaSecurityEvent | null>;
  query(filter: EventFilter): Promise<WiaSecurityEvent[]>;
  delete(id: string): Promise<boolean>;
}

export interface EventFilter {
  type?: EventType | EventType[];
  severityMin?: number;
  severityMax?: number;
  startTime?: string;
  endTime?: string;
  tags?: string[];
  limit?: number;
  offset?: number;
}

// ============================================================================
// In-Memory Store
// ============================================================================

class InMemoryStore implements EventStore {
  private events: Map<string, WiaSecurityEvent> = new Map();

  async save(event: WiaSecurityEvent): Promise<void> {
    this.events.set(event.id, event);
  }

  async get(id: string): Promise<WiaSecurityEvent | null> {
    return this.events.get(id) || null;
  }

  async query(filter: EventFilter): Promise<WiaSecurityEvent[]> {
    let results = Array.from(this.events.values());

    if (filter.type) {
      const types = Array.isArray(filter.type) ? filter.type : [filter.type];
      results = results.filter(e => types.includes(e.type));
    }

    if (filter.severityMin !== undefined) {
      results = results.filter(e => e.severity >= filter.severityMin!);
    }

    if (filter.severityMax !== undefined) {
      results = results.filter(e => e.severity <= filter.severityMax!);
    }

    if (filter.startTime) {
      results = results.filter(e => e.timestamp >= filter.startTime!);
    }

    if (filter.endTime) {
      results = results.filter(e => e.timestamp <= filter.endTime!);
    }

    if (filter.tags && filter.tags.length > 0) {
      results = results.filter(e =>
        e.meta?.tags?.some(tag => filter.tags!.includes(tag))
      );
    }

    // Sort by timestamp descending
    results.sort((a, b) => new Date(b.timestamp).getTime() - new Date(a.timestamp).getTime());

    // Apply pagination
    if (filter.offset) {
      results = results.slice(filter.offset);
    }

    if (filter.limit) {
      results = results.slice(0, filter.limit);
    }

    return results;
  }

  async delete(id: string): Promise<boolean> {
    return this.events.delete(id);
  }
}

// ============================================================================
// Security Client
// ============================================================================

export class SecurityClient {
  private config: SecurityClientConfig;
  private store: EventStore;

  constructor(config: SecurityClientConfig = {}, store?: EventStore) {
    this.config = {
      autoGenerateIds: true,
      autoTimestamp: true,
      validateOnCreate: true,
      ...config
    };
    this.store = store || new InMemoryStore();
  }

  // -------------------------------------------------------------------------
  // Event Creation
  // -------------------------------------------------------------------------

  /**
   * Create an alert event
   */
  alert() {
    const builder = createAlert();
    if (this.config.defaultSource) {
      builder.source(this.config.defaultSource);
    }
    return builder;
  }

  /**
   * Create a threat intelligence event
   */
  threatIntel() {
    const builder = createThreatIntel();
    if (this.config.defaultSource) {
      builder.source(this.config.defaultSource);
    }
    return builder;
  }

  /**
   * Create a vulnerability event
   */
  vulnerability() {
    const builder = createVulnerability();
    if (this.config.defaultSource) {
      builder.source(this.config.defaultSource);
    }
    return builder;
  }

  /**
   * Create an incident event
   */
  incident() {
    const builder = createIncident();
    if (this.config.defaultSource) {
      builder.source(this.config.defaultSource);
    }
    return builder;
  }

  /**
   * Create a network event
   */
  networkEvent() {
    const builder = createNetworkEvent();
    if (this.config.defaultSource) {
      builder.source(this.config.defaultSource);
    }
    return builder;
  }

  /**
   * Create an endpoint event
   */
  endpointEvent() {
    const builder = createEndpointEvent();
    if (this.config.defaultSource) {
      builder.source(this.config.defaultSource);
    }
    return builder;
  }

  /**
   * Create an authentication event
   */
  authEvent() {
    const builder = createAuthEvent();
    if (this.config.defaultSource) {
      builder.source(this.config.defaultSource);
    }
    return builder;
  }

  // -------------------------------------------------------------------------
  // Event Operations
  // -------------------------------------------------------------------------

  /**
   * Validate an event
   */
  validate(event: WiaSecurityEvent): ValidationResult {
    return validateEvent(event);
  }

  /**
   * Save an event to the store
   */
  async save(event: WiaSecurityEvent): Promise<WiaSecurityEvent> {
    // Auto-generate ID if needed
    if (this.config.autoGenerateIds && !event.id) {
      (event as any).id = uuidv4();
    }

    // Auto-set timestamp if needed
    if (this.config.autoTimestamp && !event.timestamp) {
      (event as any).timestamp = new Date().toISOString();
    }

    // Validate if configured
    if (this.config.validateOnCreate) {
      const result = this.validate(event);
      if (!result.valid) {
        this.config.handlers?.onValidationError?.(result.errors);
        throw new Error(`Validation failed: ${result.errors.join(', ')}`);
      }
    }

    await this.store.save(event);
    this.config.handlers?.onEventCreated?.(event);

    return event;
  }

  /**
   * Get an event by ID
   */
  async get(id: string): Promise<WiaSecurityEvent | null> {
    return this.store.get(id);
  }

  /**
   * Query events
   */
  async query(filter: EventFilter = {}): Promise<WiaSecurityEvent[]> {
    return this.store.query(filter);
  }

  /**
   * Delete an event
   */
  async delete(id: string): Promise<boolean> {
    return this.store.delete(id);
  }

  /**
   * Get all alerts
   */
  async getAlerts(filter: Omit<EventFilter, 'type'> = {}): Promise<AlertEvent[]> {
    return this.query({ ...filter, type: 'alert' }) as Promise<AlertEvent[]>;
  }

  /**
   * Get all threat intelligence
   */
  async getThreatIntel(filter: Omit<EventFilter, 'type'> = {}): Promise<ThreatIntelEvent[]> {
    return this.query({ ...filter, type: 'threat_intel' }) as Promise<ThreatIntelEvent[]>;
  }

  /**
   * Get all vulnerabilities
   */
  async getVulnerabilities(filter: Omit<EventFilter, 'type'> = {}): Promise<VulnerabilityEvent[]> {
    return this.query({ ...filter, type: 'vulnerability' }) as Promise<VulnerabilityEvent[]>;
  }

  /**
   * Get all incidents
   */
  async getIncidents(filter: Omit<EventFilter, 'type'> = {}): Promise<IncidentEvent[]> {
    return this.query({ ...filter, type: 'incident' }) as Promise<IncidentEvent[]>;
  }

  /**
   * Get high severity events
   */
  async getHighSeverity(minSeverity = 7): Promise<WiaSecurityEvent[]> {
    return this.query({ severityMin: minSeverity });
  }

  // -------------------------------------------------------------------------
  // Export/Convert
  // -------------------------------------------------------------------------

  /**
   * Export event to STIX 2.1 bundle
   */
  toStix(event: WiaSecurityEvent) {
    return toStixBundle(event);
  }

  /**
   * Export event to ECS format
   */
  toEcs(event: WiaSecurityEvent) {
    return toEcsEvent(event);
  }

  /**
   * Export event to OCSF format
   */
  toOcsf(event: WiaSecurityEvent) {
    return toOcsfEvent(event);
  }

  /**
   * Export event to Splunk HEC format
   */
  toSplunk(event: WiaSecurityEvent, index?: string) {
    return toSplunkEvent(event, index);
  }

  /**
   * Export event to Elasticsearch document
   */
  toElastic(event: WiaSecurityEvent, indexPrefix?: string) {
    return toElasticEvent(event, indexPrefix);
  }

  /**
   * Export multiple events to JSON
   */
  exportJson(events: WiaSecurityEvent[]): string {
    return JSON.stringify(events, null, 2);
  }

  /**
   * Import events from JSON
   */
  async importJson(json: string): Promise<WiaSecurityEvent[]> {
    const events = JSON.parse(json) as WiaSecurityEvent[];
    const saved: WiaSecurityEvent[] = [];

    for (const event of events) {
      saved.push(await this.save(event));
    }

    return saved;
  }

  // -------------------------------------------------------------------------
  // Statistics
  // -------------------------------------------------------------------------

  /**
   * Get event statistics
   */
  async getStats(): Promise<{
    total: number;
    byType: Record<EventType, number>;
    bySeverity: Record<string, number>;
    avgSeverity: number;
  }> {
    const events = await this.query({});

    const byType: Record<string, number> = {};
    const bySeverity: Record<string, number> = {
      info: 0,
      low: 0,
      medium: 0,
      high: 0,
      critical: 0
    };
    let totalSeverity = 0;

    for (const event of events) {
      // Count by type
      byType[event.type] = (byType[event.type] || 0) + 1;

      // Count by severity
      totalSeverity += event.severity;
      if (event.severity <= 2) bySeverity.info++;
      else if (event.severity <= 4) bySeverity.low++;
      else if (event.severity <= 6) bySeverity.medium++;
      else if (event.severity <= 8) bySeverity.high++;
      else bySeverity.critical++;
    }

    return {
      total: events.length,
      byType: byType as Record<EventType, number>,
      bySeverity,
      avgSeverity: events.length > 0 ? totalSeverity / events.length : 0
    };
  }
}

// ============================================================================
// Factory Function
// ============================================================================

export function createClient(config?: SecurityClientConfig, store?: EventStore): SecurityClient {
  return new SecurityClient(config, store);
}
