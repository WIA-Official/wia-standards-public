/**
 * WIA Security Event Builder
 * Fluent API for creating WIA Security events
 */

import { v4 as uuidv4 } from 'uuid';
import {
  WiaSecurityEvent,
  EventType,
  Source,
  Context,
  Mitre,
  Meta,
  AlertData,
  ThreatIntelData,
  VulnerabilityData,
  IncidentData,
  NetworkEventData,
  EndpointEventData,
  AuthEventData,
  Host,
  User,
  Cloud,
  NetworkContext,
  AlertEvent,
  ThreatIntelEvent,
  VulnerabilityEvent,
  IncidentEvent,
  NetworkEvent,
  EndpointEvent,
  AuthEvent
} from './types';

// ============================================================================
// Base Event Builder
// ============================================================================

export class EventBuilder<T extends WiaSecurityEvent = WiaSecurityEvent> {
  protected event: Partial<T>;

  constructor() {
    this.event = {
      $schema: 'https://wia.live/security/v1/schema.json',
      version: '1.0.0',
      id: uuidv4(),
      timestamp: new Date().toISOString(),
      severity: 5
    } as Partial<T>;
  }

  /**
   * Set the event ID
   */
  id(id: string): this {
    this.event.id = id;
    return this;
  }

  /**
   * Set the timestamp
   */
  timestamp(timestamp: string | Date): this {
    this.event.timestamp = timestamp instanceof Date ? timestamp.toISOString() : timestamp;
    return this;
  }

  /**
   * Set the severity (0-10)
   */
  severity(severity: number): this {
    this.event.severity = Math.max(0, Math.min(10, severity));
    return this;
  }

  /**
   * Set the source
   */
  source(source: Source): this {
    this.event.source = source;
    return this;
  }

  /**
   * Set the context
   */
  context(context: Context): this {
    this.event.context = context;
    return this;
  }

  /**
   * Set the host context
   */
  host(host: Host): this {
    if (!this.event.context) {
      this.event.context = {};
    }
    this.event.context.host = host;
    return this;
  }

  /**
   * Set the user context
   */
  user(user: User): this {
    if (!this.event.context) {
      this.event.context = {};
    }
    this.event.context.user = user;
    return this;
  }

  /**
   * Set the cloud context
   */
  cloud(cloud: Cloud): this {
    if (!this.event.context) {
      this.event.context = {};
    }
    this.event.context.cloud = cloud;
    return this;
  }

  /**
   * Set the network context
   */
  network(network: NetworkContext): this {
    if (!this.event.context) {
      this.event.context = {};
    }
    this.event.context.network = network;
    return this;
  }

  /**
   * Set MITRE ATT&CK mapping
   */
  mitre(mitre: Mitre): this {
    this.event.mitre = mitre;
    return this;
  }

  /**
   * Set MITRE tactic
   */
  tactic(id: string, name?: string): this {
    if (!this.event.mitre) {
      this.event.mitre = {};
    }
    this.event.mitre.tactic = id;
    if (name) this.event.mitre.tactic_name = name;
    return this;
  }

  /**
   * Set MITRE technique
   */
  technique(id: string, name?: string): this {
    if (!this.event.mitre) {
      this.event.mitre = {};
    }
    this.event.mitre.technique = id;
    if (name) this.event.mitre.technique_name = name;
    return this;
  }

  /**
   * Set MITRE sub-technique
   */
  subTechnique(id: string, name?: string): this {
    if (!this.event.mitre) {
      this.event.mitre = {};
    }
    this.event.mitre.sub_technique = id;
    if (name) this.event.mitre.sub_technique_name = name;
    return this;
  }

  /**
   * Set metadata
   */
  meta(meta: Meta): this {
    this.event.meta = meta;
    return this;
  }

  /**
   * Set confidence score
   */
  confidence(confidence: number): this {
    if (!this.event.meta) {
      this.event.meta = {};
    }
    this.event.meta.confidence = Math.max(0, Math.min(1, confidence));
    return this;
  }

  /**
   * Add tags
   */
  tags(...tags: string[]): this {
    if (!this.event.meta) {
      this.event.meta = {};
    }
    if (!this.event.meta.tags) {
      this.event.meta.tags = [];
    }
    this.event.meta.tags.push(...tags);
    return this;
  }

  /**
   * Add labels
   */
  labels(labels: Record<string, string>): this {
    if (!this.event.meta) {
      this.event.meta = {};
    }
    this.event.meta.labels = { ...this.event.meta.labels, ...labels };
    return this;
  }

  /**
   * Set correlation ID
   */
  correlationId(id: string): this {
    if (!this.event.meta) {
      this.event.meta = {};
    }
    this.event.meta.correlation_id = id;
    return this;
  }

  /**
   * Set raw log
   */
  raw(raw: string): this {
    if (!this.event.meta) {
      this.event.meta = {};
    }
    this.event.meta.raw = raw;
    return this;
  }

  /**
   * Build the event
   */
  build(): T {
    if (!this.event.type) {
      throw new Error('Event type is required');
    }
    if (!this.event.source) {
      throw new Error('Event source is required');
    }
    if (!this.event.data) {
      throw new Error('Event data is required');
    }
    return this.event as T;
  }
}

// ============================================================================
// Alert Builder
// ============================================================================

export class AlertBuilder extends EventBuilder<AlertEvent> {
  constructor() {
    super();
    (this.event as Partial<AlertEvent>).type = 'alert';
  }

  /**
   * Set alert data
   */
  data(data: AlertData): this {
    this.event.data = data;
    return this;
  }

  /**
   * Set alert ID
   */
  alertId(alertId: string): this {
    if (!this.event.data) {
      this.event.data = {} as AlertData;
    }
    this.event.data.alert_id = alertId;
    return this;
  }

  /**
   * Set alert title
   */
  title(title: string): this {
    if (!this.event.data) {
      this.event.data = {} as AlertData;
    }
    this.event.data.title = title;
    return this;
  }

  /**
   * Set alert description
   */
  description(description: string): this {
    if (!this.event.data) {
      this.event.data = {} as AlertData;
    }
    this.event.data.description = description;
    return this;
  }

  /**
   * Set alert category
   */
  category(category: AlertData['category']): this {
    if (!this.event.data) {
      this.event.data = {} as AlertData;
    }
    this.event.data.category = category;
    return this;
  }

  /**
   * Set alert status
   */
  status(status: AlertData['status']): this {
    if (!this.event.data) {
      this.event.data = {} as AlertData;
    }
    this.event.data.status = status;
    return this;
  }

  /**
   * Set alert priority
   */
  priority(priority: AlertData['priority']): this {
    if (!this.event.data) {
      this.event.data = {} as AlertData;
    }
    this.event.data.priority = priority;
    return this;
  }

  /**
   * Set detection rule
   */
  detectionRule(id: string, name: string, version?: string): this {
    if (!this.event.data) {
      this.event.data = {} as AlertData;
    }
    this.event.data.detection_rule = { id, name, version };
    return this;
  }

  /**
   * Add indicator
   */
  addIndicator(indicator: AlertData['indicators'][0]): this {
    if (!this.event.data) {
      this.event.data = {} as AlertData;
    }
    if (!this.event.data.indicators) {
      this.event.data.indicators = [];
    }
    this.event.data.indicators.push(indicator);
    return this;
  }
}

// ============================================================================
// Threat Intel Builder
// ============================================================================

export class ThreatIntelBuilder extends EventBuilder<ThreatIntelEvent> {
  constructor() {
    super();
    (this.event as Partial<ThreatIntelEvent>).type = 'threat_intel';
  }

  data(data: ThreatIntelData): this {
    this.event.data = data;
    return this;
  }

  threatType(type: ThreatIntelData['threat_type']): this {
    if (!this.event.data) {
      this.event.data = {} as ThreatIntelData;
    }
    this.event.data.threat_type = type;
    return this;
  }

  threatName(name: string): this {
    if (!this.event.data) {
      this.event.data = {} as ThreatIntelData;
    }
    this.event.data.threat_name = name;
    return this;
  }

  threatFamily(family: string): this {
    if (!this.event.data) {
      this.event.data = {} as ThreatIntelData;
    }
    this.event.data.threat_family = family;
    return this;
  }

  status(status: ThreatIntelData['status']): this {
    if (!this.event.data) {
      this.event.data = {} as ThreatIntelData;
    }
    this.event.data.status = status;
    return this;
  }

  targetSectors(...sectors: string[]): this {
    if (!this.event.data) {
      this.event.data = {} as ThreatIntelData;
    }
    this.event.data.target_sectors = sectors;
    return this;
  }

  targetCountries(...countries: string[]): this {
    if (!this.event.data) {
      this.event.data = {} as ThreatIntelData;
    }
    this.event.data.target_countries = countries;
    return this;
  }
}

// ============================================================================
// Vulnerability Builder
// ============================================================================

export class VulnerabilityBuilder extends EventBuilder<VulnerabilityEvent> {
  constructor() {
    super();
    (this.event as Partial<VulnerabilityEvent>).type = 'vulnerability';
  }

  data(data: VulnerabilityData): this {
    this.event.data = data;
    return this;
  }

  vulnId(id: string): this {
    if (!this.event.data) {
      this.event.data = {} as VulnerabilityData;
    }
    this.event.data.vuln_id = id;
    return this;
  }

  title(title: string): this {
    if (!this.event.data) {
      this.event.data = {} as VulnerabilityData;
    }
    this.event.data.title = title;
    return this;
  }

  cvss(cvss: VulnerabilityData['cvss']): this {
    if (!this.event.data) {
      this.event.data = {} as VulnerabilityData;
    }
    this.event.data.cvss = cvss;
    return this;
  }

  cwe(...cweIds: string[]): this {
    if (!this.event.data) {
      this.event.data = {} as VulnerabilityData;
    }
    this.event.data.cwe = cweIds;
    return this;
  }

  exploitAvailable(available: boolean): this {
    if (!this.event.data) {
      this.event.data = {} as VulnerabilityData;
    }
    this.event.data.exploit_available = available;
    return this;
  }

  patchAvailable(available: boolean): this {
    if (!this.event.data) {
      this.event.data = {} as VulnerabilityData;
    }
    this.event.data.patch_available = available;
    return this;
  }
}

// ============================================================================
// Incident Builder
// ============================================================================

export class IncidentBuilder extends EventBuilder<IncidentEvent> {
  constructor() {
    super();
    (this.event as Partial<IncidentEvent>).type = 'incident';
  }

  data(data: IncidentData): this {
    this.event.data = data;
    return this;
  }

  incidentId(id: string): this {
    if (!this.event.data) {
      this.event.data = {} as IncidentData;
    }
    this.event.data.incident_id = id;
    return this;
  }

  title(title: string): this {
    if (!this.event.data) {
      this.event.data = {} as IncidentData;
    }
    this.event.data.title = title;
    return this;
  }

  category(category: IncidentData['category']): this {
    if (!this.event.data) {
      this.event.data = {} as IncidentData;
    }
    this.event.data.category = category;
    return this;
  }

  status(status: IncidentData['status']): this {
    if (!this.event.data) {
      this.event.data = {} as IncidentData;
    }
    this.event.data.status = status;
    return this;
  }

  priority(priority: IncidentData['priority']): this {
    if (!this.event.data) {
      this.event.data = {} as IncidentData;
    }
    this.event.data.priority = priority;
    return this;
  }

  impact(impact: IncidentData['impact']): this {
    if (!this.event.data) {
      this.event.data = {} as IncidentData;
    }
    this.event.data.impact = impact;
    return this;
  }

  leadAnalyst(analyst: string): this {
    if (!this.event.data) {
      this.event.data = {} as IncidentData;
    }
    this.event.data.lead_analyst = analyst;
    return this;
  }

  team(...members: string[]): this {
    if (!this.event.data) {
      this.event.data = {} as IncidentData;
    }
    this.event.data.team = members;
    return this;
  }
}

// ============================================================================
// Network Event Builder
// ============================================================================

export class NetworkEventBuilder extends EventBuilder<NetworkEvent> {
  constructor() {
    super();
    (this.event as Partial<NetworkEvent>).type = 'network_event';
  }

  data(data: NetworkEventData): this {
    this.event.data = data;
    return this;
  }

  eventType(type: NetworkEventData['event_type']): this {
    if (!this.event.data) {
      this.event.data = {} as NetworkEventData;
    }
    this.event.data.event_type = type;
    return this;
  }

  protocol(protocol: NetworkEventData['protocol']): this {
    if (!this.event.data) {
      this.event.data = {} as NetworkEventData;
    }
    this.event.data.protocol = protocol;
    return this;
  }

  direction(direction: NetworkEventData['direction']): this {
    if (!this.event.data) {
      this.event.data = {} as NetworkEventData;
    }
    this.event.data.direction = direction;
    return this;
  }

  action(action: NetworkEventData['action']): this {
    if (!this.event.data) {
      this.event.data = {} as NetworkEventData;
    }
    this.event.data.action = action;
    return this;
  }

  sourceHost(host: NetworkEventData['source']): this {
    if (!this.event.data) {
      this.event.data = {} as NetworkEventData;
    }
    this.event.data.source = host;
    return this;
  }

  destHost(host: NetworkEventData['destination']): this {
    if (!this.event.data) {
      this.event.data = {} as NetworkEventData;
    }
    this.event.data.destination = host;
    return this;
  }
}

// ============================================================================
// Endpoint Event Builder
// ============================================================================

export class EndpointEventBuilder extends EventBuilder<EndpointEvent> {
  constructor() {
    super();
    (this.event as Partial<EndpointEvent>).type = 'endpoint_event';
  }

  data(data: EndpointEventData): this {
    this.event.data = data;
    return this;
  }

  eventType(type: EndpointEventData['event_type']): this {
    if (!this.event.data) {
      this.event.data = {} as EndpointEventData;
    }
    this.event.data.event_type = type;
    return this;
  }

  host(host: EndpointEventData['host']): this {
    if (!this.event.data) {
      this.event.data = {} as EndpointEventData;
    }
    this.event.data.host = host;
    return this;
  }

  process(process: EndpointEventData['process']): this {
    if (!this.event.data) {
      this.event.data = {} as EndpointEventData;
    }
    this.event.data.process = process;
    return this;
  }

  file(file: EndpointEventData['file']): this {
    if (!this.event.data) {
      this.event.data = {} as EndpointEventData;
    }
    this.event.data.file = file;
    return this;
  }

  registry(registry: EndpointEventData['registry']): this {
    if (!this.event.data) {
      this.event.data = {} as EndpointEventData;
    }
    this.event.data.registry = registry;
    return this;
  }
}

// ============================================================================
// Auth Event Builder
// ============================================================================

export class AuthEventBuilder extends EventBuilder<AuthEvent> {
  constructor() {
    super();
    (this.event as Partial<AuthEvent>).type = 'auth_event';
  }

  data(data: AuthEventData): this {
    this.event.data = data;
    return this;
  }

  eventType(type: AuthEventData['event_type']): this {
    if (!this.event.data) {
      this.event.data = {} as AuthEventData;
    }
    this.event.data.event_type = type;
    return this;
  }

  result(result: AuthEventData['result']): this {
    if (!this.event.data) {
      this.event.data = {} as AuthEventData;
    }
    this.event.data.result = result;
    return this;
  }

  authUser(user: AuthEventData['user']): this {
    if (!this.event.data) {
      this.event.data = {} as AuthEventData;
    }
    this.event.data.user = user;
    return this;
  }

  target(target: AuthEventData['target']): this {
    if (!this.event.data) {
      this.event.data = {} as AuthEventData;
    }
    this.event.data.target = target;
    return this;
  }

  authMethod(method: AuthEventData['auth_method']): this {
    if (!this.event.data) {
      this.event.data = {} as AuthEventData;
    }
    this.event.data.auth_method = method;
    return this;
  }

  riskScore(score: number): this {
    if (!this.event.data) {
      this.event.data = {} as AuthEventData;
    }
    this.event.data.risk_score = Math.max(0, Math.min(1, score));
    return this;
  }

  riskFactors(...factors: string[]): this {
    if (!this.event.data) {
      this.event.data = {} as AuthEventData;
    }
    this.event.data.risk_factors = factors;
    return this;
  }
}

// ============================================================================
// Factory Functions
// ============================================================================

export function createAlert(): AlertBuilder {
  return new AlertBuilder();
}

export function createThreatIntel(): ThreatIntelBuilder {
  return new ThreatIntelBuilder();
}

export function createVulnerability(): VulnerabilityBuilder {
  return new VulnerabilityBuilder();
}

export function createIncident(): IncidentBuilder {
  return new IncidentBuilder();
}

export function createNetworkEvent(): NetworkEventBuilder {
  return new NetworkEventBuilder();
}

export function createEndpointEvent(): EndpointEventBuilder {
  return new EndpointEventBuilder();
}

export function createAuthEvent(): AuthEventBuilder {
  return new AuthEventBuilder();
}
