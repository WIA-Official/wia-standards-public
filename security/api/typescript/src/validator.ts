/**
 * WIA Security Event Validator
 * Validation utilities for WIA Security events
 */

import {
  WiaSecurityEvent,
  EventType,
  AlertData,
  ThreatIntelData,
  VulnerabilityData,
  IncidentData,
  NetworkEventData,
  EndpointEventData,
  AuthEventData
} from './types';

// ============================================================================
// Validation Result
// ============================================================================

export interface ValidationResult {
  valid: boolean;
  errors: string[];
  warnings: string[];
}

// ============================================================================
// Validation Patterns
// ============================================================================

const UUID_PATTERN = /^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/i;
const ISO8601_PATTERN = /^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d{3})?Z$/;
const SEMVER_PATTERN = /^\d+\.\d+\.\d+$/;
const TACTIC_PATTERN = /^TA\d{4}$/;
const TECHNIQUE_PATTERN = /^T\d{4}(\.\d{3})?$/;
const CVE_PATTERN = /^CVE-\d{4}-\d{4,}$/;
const CWE_PATTERN = /^CWE-\d+$/;

// ============================================================================
// Valid Values
// ============================================================================

const VALID_EVENT_TYPES: EventType[] = [
  'alert', 'threat_intel', 'vulnerability', 'incident',
  'network_event', 'endpoint_event', 'auth_event'
];

const VALID_SOURCE_TYPES = ['siem', 'edr', 'ids', 'firewall', 'scanner', 'custom'];
const VALID_PRIORITIES = ['critical', 'high', 'medium', 'low', 'info'];
const VALID_DIRECTIONS = ['inbound', 'outbound', 'internal'];
const VALID_CLOUD_PROVIDERS = ['aws', 'azure', 'gcp'];

// ============================================================================
// Main Validator
// ============================================================================

export function validateEvent(event: unknown): ValidationResult {
  const errors: string[] = [];
  const warnings: string[] = [];

  if (!event || typeof event !== 'object') {
    return { valid: false, errors: ['Event must be an object'], warnings: [] };
  }

  const e = event as Record<string, unknown>;

  // Required fields
  validateRequiredFields(e, errors);

  // Source validation
  if (e.source) {
    validateSource(e.source, errors);
  }

  // Data validation
  if (e.data && e.type) {
    validateEventData(e.type as EventType, e.data as Record<string, unknown>, errors);
  }

  // Optional fields
  if (e.context) {
    validateContext(e.context, errors);
  }

  if (e.mitre) {
    validateMitre(e.mitre, errors);
  }

  if (e.meta) {
    validateMeta(e.meta, errors, warnings);
  }

  // Warnings
  if (!e.$schema) {
    warnings.push('Recommended field $schema is missing');
  }

  return {
    valid: errors.length === 0,
    errors,
    warnings
  };
}

// ============================================================================
// Field Validators
// ============================================================================

function validateRequiredFields(event: Record<string, unknown>, errors: string[]): void {
  if (!event.version) {
    errors.push('Missing required field: version');
  } else if (typeof event.version !== 'string' || !SEMVER_PATTERN.test(event.version)) {
    errors.push('Invalid version format. Expected SemVer (e.g., 1.0.0)');
  }

  if (!event.id) {
    errors.push('Missing required field: id');
  } else if (typeof event.id !== 'string' || !UUID_PATTERN.test(event.id)) {
    errors.push('Invalid id format. Expected UUID v4');
  }

  if (!event.type) {
    errors.push('Missing required field: type');
  } else if (!VALID_EVENT_TYPES.includes(event.type as EventType)) {
    errors.push(`Invalid event type: ${event.type}`);
  }

  if (!event.timestamp) {
    errors.push('Missing required field: timestamp');
  } else if (typeof event.timestamp !== 'string' || !ISO8601_PATTERN.test(event.timestamp)) {
    errors.push('Invalid timestamp format. Expected ISO 8601');
  }

  if (event.severity === undefined || event.severity === null) {
    errors.push('Missing required field: severity');
  } else if (typeof event.severity !== 'number' || event.severity < 0 || event.severity > 10) {
    errors.push('Invalid severity. Expected number between 0 and 10');
  }

  if (!event.source) {
    errors.push('Missing required field: source');
  }

  if (!event.data) {
    errors.push('Missing required field: data');
  } else if (typeof event.data !== 'object') {
    errors.push('Invalid data field. Expected object');
  }
}

function validateSource(source: unknown, errors: string[]): void {
  if (typeof source !== 'object' || source === null) {
    errors.push('Source must be an object');
    return;
  }

  const s = source as Record<string, unknown>;

  if (!s.type) {
    errors.push('Missing required field in source: type');
  } else if (!VALID_SOURCE_TYPES.includes(s.type as string)) {
    errors.push(`Invalid source type: ${s.type}`);
  }

  if (!s.name) {
    errors.push('Missing required field in source: name');
  }
}

function validateContext(context: unknown, errors: string[]): void {
  if (typeof context !== 'object' || context === null) {
    errors.push('Context must be an object');
    return;
  }

  const c = context as Record<string, unknown>;

  if (c.host && typeof c.host === 'object') {
    const host = c.host as Record<string, unknown>;
    if (host.ip && !Array.isArray(host.ip)) {
      errors.push('context.host.ip must be an array');
    }
    if (host.mac && !Array.isArray(host.mac)) {
      errors.push('context.host.mac must be an array');
    }
  }

  if (c.network && typeof c.network === 'object') {
    const network = c.network as Record<string, unknown>;
    if (network.direction && !VALID_DIRECTIONS.includes(network.direction as string)) {
      errors.push(`Invalid network direction: ${network.direction}`);
    }
  }

  if (c.cloud && typeof c.cloud === 'object') {
    const cloud = c.cloud as Record<string, unknown>;
    if (cloud.provider && !VALID_CLOUD_PROVIDERS.includes(cloud.provider as string)) {
      errors.push(`Invalid cloud provider: ${cloud.provider}`);
    }
  }
}

function validateMitre(mitre: unknown, errors: string[]): void {
  if (typeof mitre !== 'object' || mitre === null) {
    errors.push('Mitre must be an object');
    return;
  }

  const m = mitre as Record<string, unknown>;

  if (m.tactic && typeof m.tactic === 'string' && !TACTIC_PATTERN.test(m.tactic)) {
    errors.push(`Invalid MITRE tactic ID format: ${m.tactic}`);
  }

  if (m.technique && typeof m.technique === 'string' && !TECHNIQUE_PATTERN.test(m.technique)) {
    errors.push(`Invalid MITRE technique ID format: ${m.technique}`);
  }
}

function validateMeta(meta: unknown, errors: string[], warnings: string[]): void {
  if (typeof meta !== 'object' || meta === null) {
    errors.push('Meta must be an object');
    return;
  }

  const m = meta as Record<string, unknown>;

  if (m.confidence !== undefined) {
    if (typeof m.confidence !== 'number' || m.confidence < 0 || m.confidence > 1) {
      errors.push('meta.confidence must be a number between 0 and 1');
    }
  }

  if (m.tags && !Array.isArray(m.tags)) {
    errors.push('meta.tags must be an array');
  }

  if (m.labels && typeof m.labels !== 'object') {
    errors.push('meta.labels must be an object');
  }
}

function validateEventData(type: EventType, data: Record<string, unknown>, errors: string[]): void {
  const validators: Record<EventType, (d: Record<string, unknown>, e: string[]) => void> = {
    alert: validateAlertData,
    threat_intel: validateThreatIntelData,
    vulnerability: validateVulnerabilityData,
    incident: validateIncidentData,
    network_event: validateNetworkEventData,
    endpoint_event: validateEndpointEventData,
    auth_event: validateAuthEventData
  };

  if (validators[type]) {
    validators[type](data, errors);
  }
}

function validateAlertData(data: Record<string, unknown>, errors: string[]): void {
  const required = ['alert_id', 'title', 'category', 'status', 'priority'];
  required.forEach(field => {
    if (!data[field]) errors.push(`Missing required field in alert data: ${field}`);
  });

  const validCategories = ['malware', 'intrusion', 'policy', 'reconnaissance', 'other'];
  if (data.category && !validCategories.includes(data.category as string)) {
    errors.push(`Invalid alert category: ${data.category}`);
  }

  const validStatuses = ['new', 'investigating', 'resolved', 'false_positive', 'closed'];
  if (data.status && !validStatuses.includes(data.status as string)) {
    errors.push(`Invalid alert status: ${data.status}`);
  }

  if (data.priority && !VALID_PRIORITIES.includes(data.priority as string)) {
    errors.push(`Invalid alert priority: ${data.priority}`);
  }
}

function validateThreatIntelData(data: Record<string, unknown>, errors: string[]): void {
  const required = ['threat_type', 'threat_name', 'status'];
  required.forEach(field => {
    if (!data[field]) errors.push(`Missing required field in threat_intel data: ${field}`);
  });

  const validTypes = ['malware', 'apt', 'campaign', 'botnet', 'ransomware', 'phishing'];
  if (data.threat_type && !validTypes.includes(data.threat_type as string)) {
    errors.push(`Invalid threat_type: ${data.threat_type}`);
  }

  const validStatuses = ['active', 'inactive', 'unknown'];
  if (data.status && !validStatuses.includes(data.status as string)) {
    errors.push(`Invalid threat_intel status: ${data.status}`);
  }
}

function validateVulnerabilityData(data: Record<string, unknown>, errors: string[]): void {
  const required = ['vuln_id', 'title', 'cvss'];
  required.forEach(field => {
    if (!data[field]) errors.push(`Missing required field in vulnerability data: ${field}`);
  });

  if (data.vuln_id && typeof data.vuln_id === 'string' && !CVE_PATTERN.test(data.vuln_id)) {
    errors.push(`Invalid CVE format: ${data.vuln_id}`);
  }

  if (data.cvss && typeof data.cvss === 'object') {
    const cvss = data.cvss as Record<string, unknown>;
    if (cvss.score !== undefined && (typeof cvss.score !== 'number' || cvss.score < 0 || cvss.score > 10)) {
      errors.push('cvss.score must be a number between 0 and 10');
    }
  }

  if (data.cwe && Array.isArray(data.cwe)) {
    (data.cwe as string[]).forEach(cwe => {
      if (!CWE_PATTERN.test(cwe)) {
        errors.push(`Invalid CWE format: ${cwe}`);
      }
    });
  }
}

function validateIncidentData(data: Record<string, unknown>, errors: string[]): void {
  const required = ['incident_id', 'title', 'category', 'status', 'priority'];
  required.forEach(field => {
    if (!data[field]) errors.push(`Missing required field in incident data: ${field}`);
  });

  const validCategories = ['malware', 'phishing', 'ransomware', 'data_breach', 'ddos',
    'unauthorized_access', 'insider_threat', 'apt', 'other'];
  if (data.category && !validCategories.includes(data.category as string)) {
    errors.push(`Invalid incident category: ${data.category}`);
  }

  const validStatuses = ['new', 'triaging', 'investigating', 'containing',
    'eradicating', 'recovering', 'closed'];
  if (data.status && !validStatuses.includes(data.status as string)) {
    errors.push(`Invalid incident status: ${data.status}`);
  }
}

function validateNetworkEventData(data: Record<string, unknown>, errors: string[]): void {
  const required = ['event_type', 'protocol', 'source', 'destination'];
  required.forEach(field => {
    if (!data[field]) errors.push(`Missing required field in network_event data: ${field}`);
  });

  const validEventTypes = ['connection', 'dns', 'http', 'tls', 'smtp', 'ftp', 'ssh', 'rdp', 'custom'];
  if (data.event_type && !validEventTypes.includes(data.event_type as string)) {
    errors.push(`Invalid network event_type: ${data.event_type}`);
  }

  const validProtocols = ['TCP', 'UDP', 'ICMP', 'GRE', 'ESP'];
  if (data.protocol && !validProtocols.includes(data.protocol as string)) {
    errors.push(`Invalid network protocol: ${data.protocol}`);
  }
}

function validateEndpointEventData(data: Record<string, unknown>, errors: string[]): void {
  const required = ['event_type', 'host'];
  required.forEach(field => {
    if (!data[field]) errors.push(`Missing required field in endpoint_event data: ${field}`);
  });

  const validEventTypes = [
    'process_creation', 'process_termination', 'file_create', 'file_modify', 'file_delete',
    'registry_create', 'registry_modify', 'registry_delete', 'network_connection',
    'dll_load', 'driver_load', 'service_install'
  ];
  if (data.event_type && !validEventTypes.includes(data.event_type as string)) {
    errors.push(`Invalid endpoint event_type: ${data.event_type}`);
  }
}

function validateAuthEventData(data: Record<string, unknown>, errors: string[]): void {
  const required = ['event_type', 'result', 'user', 'target'];
  required.forEach(field => {
    if (!data[field]) errors.push(`Missing required field in auth_event data: ${field}`);
  });

  const validEventTypes = [
    'login_success', 'login_failure', 'logout', 'password_change',
    'account_locked', 'account_unlocked', 'mfa_success', 'mfa_failure', 'privilege_escalation'
  ];
  if (data.event_type && !validEventTypes.includes(data.event_type as string)) {
    errors.push(`Invalid auth event_type: ${data.event_type}`);
  }

  if (data.result && !['success', 'failure'].includes(data.result as string)) {
    errors.push(`Invalid auth result: ${data.result}`);
  }

  if (data.risk_score !== undefined) {
    const score = data.risk_score as number;
    if (typeof score !== 'number' || score < 0 || score > 1) {
      errors.push('risk_score must be a number between 0 and 1');
    }
  }
}

// ============================================================================
// Type Guards
// ============================================================================

export function isAlertEvent(event: WiaSecurityEvent): event is WiaSecurityEvent & { type: 'alert'; data: AlertData } {
  return event.type === 'alert';
}

export function isThreatIntelEvent(event: WiaSecurityEvent): event is WiaSecurityEvent & { type: 'threat_intel'; data: ThreatIntelData } {
  return event.type === 'threat_intel';
}

export function isVulnerabilityEvent(event: WiaSecurityEvent): event is WiaSecurityEvent & { type: 'vulnerability'; data: VulnerabilityData } {
  return event.type === 'vulnerability';
}

export function isIncidentEvent(event: WiaSecurityEvent): event is WiaSecurityEvent & { type: 'incident'; data: IncidentData } {
  return event.type === 'incident';
}

export function isNetworkEvent(event: WiaSecurityEvent): event is WiaSecurityEvent & { type: 'network_event'; data: NetworkEventData } {
  return event.type === 'network_event';
}

export function isEndpointEvent(event: WiaSecurityEvent): event is WiaSecurityEvent & { type: 'endpoint_event'; data: EndpointEventData } {
  return event.type === 'endpoint_event';
}

export function isAuthEvent(event: WiaSecurityEvent): event is WiaSecurityEvent & { type: 'auth_event'; data: AuthEventData } {
  return event.type === 'auth_event';
}
