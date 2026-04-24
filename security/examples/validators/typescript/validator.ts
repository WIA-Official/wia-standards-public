/**
 * WIA Security Event Validator
 * TypeScript implementation for validating WIA Security events
 */

// Event Types
type EventType = 'alert' | 'threat_intel' | 'vulnerability' | 'incident' | 'network_event' | 'endpoint_event' | 'auth_event';
type SourceType = 'siem' | 'edr' | 'ids' | 'firewall' | 'scanner' | 'custom';
type Priority = 'critical' | 'high' | 'medium' | 'low' | 'info';
type Direction = 'inbound' | 'outbound' | 'internal';

// Base Interfaces
interface Source {
  type: SourceType;
  name: string;
  vendor?: string;
  version?: string;
}

interface Host {
  hostname?: string;
  ip?: string[];
  mac?: string[];
  os?: {
    name?: string;
    version?: string;
    build?: string;
  };
}

interface NetworkEndpoint {
  ip?: string;
  port?: number;
  hostname?: string;
}

interface User {
  name?: string;
  domain?: string;
  email?: string;
  employee_id?: string;
}

interface Cloud {
  provider?: 'aws' | 'azure' | 'gcp';
  account_id?: string;
  region?: string;
}

interface Context {
  host?: Host;
  network?: {
    source?: NetworkEndpoint;
    destination?: NetworkEndpoint;
    protocol?: string;
    direction?: Direction;
  };
  user?: User;
  cloud?: Cloud;
}

interface Mitre {
  tactic?: string;
  tactic_name?: string;
  technique?: string;
  technique_name?: string;
  sub_technique?: string;
  sub_technique_name?: string;
}

interface Meta {
  confidence?: number;
  tags?: string[];
  labels?: Record<string, string>;
  raw?: string;
  correlation_id?: string;
  custom?: Record<string, unknown>;
}

// Base Event Interface
interface WiaSecurityEvent {
  $schema?: string;
  version: string;
  id: string;
  type: EventType;
  timestamp: string;
  severity: number;
  source: Source;
  data: Record<string, unknown>;
  context?: Context;
  mitre?: Mitre;
  meta?: Meta;
}

// Validation Result
interface ValidationResult {
  valid: boolean;
  errors: string[];
  warnings: string[];
}

// UUID v4 regex pattern
const UUID_PATTERN = /^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/i;
const ISO8601_PATTERN = /^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d{3})?Z$/;
const SEMVER_PATTERN = /^\d+\.\d+\.\d+$/;
const TACTIC_PATTERN = /^TA\d{4}$/;
const TECHNIQUE_PATTERN = /^T\d{4}(\.\d{3})?$/;
const CVE_PATTERN = /^CVE-\d{4}-\d{4,}$/;
const CWE_PATTERN = /^CWE-\d+$/;

/**
 * Validates a WIA Security Event
 */
export function validateEvent(event: unknown): ValidationResult {
  const errors: string[] = [];
  const warnings: string[] = [];

  if (!event || typeof event !== 'object') {
    return { valid: false, errors: ['Event must be an object'], warnings: [] };
  }

  const e = event as Record<string, unknown>;

  // Required fields validation
  if (!e.version) {
    errors.push('Missing required field: version');
  } else if (typeof e.version !== 'string' || !SEMVER_PATTERN.test(e.version)) {
    errors.push('Invalid version format. Expected SemVer (e.g., 1.0.0)');
  }

  if (!e.id) {
    errors.push('Missing required field: id');
  } else if (typeof e.id !== 'string' || !UUID_PATTERN.test(e.id)) {
    errors.push('Invalid id format. Expected UUID v4');
  }

  if (!e.type) {
    errors.push('Missing required field: type');
  } else if (!isValidEventType(e.type as string)) {
    errors.push(`Invalid event type: ${e.type}. Expected one of: alert, threat_intel, vulnerability, incident, network_event, endpoint_event, auth_event`);
  }

  if (!e.timestamp) {
    errors.push('Missing required field: timestamp');
  } else if (typeof e.timestamp !== 'string' || !ISO8601_PATTERN.test(e.timestamp)) {
    errors.push('Invalid timestamp format. Expected ISO 8601 (e.g., 2025-12-14T00:00:00.000Z)');
  }

  if (e.severity === undefined || e.severity === null) {
    errors.push('Missing required field: severity');
  } else if (typeof e.severity !== 'number' || e.severity < 0 || e.severity > 10) {
    errors.push('Invalid severity. Expected number between 0 and 10');
  }

  if (!e.source) {
    errors.push('Missing required field: source');
  } else {
    validateSource(e.source, errors, warnings);
  }

  if (!e.data) {
    errors.push('Missing required field: data');
  } else if (typeof e.data !== 'object') {
    errors.push('Invalid data field. Expected object');
  } else {
    // Validate event-specific data
    validateEventData(e.type as EventType, e.data as Record<string, unknown>, errors, warnings);
  }

  // Optional fields validation
  if (e.context) {
    validateContext(e.context, errors, warnings);
  }

  if (e.mitre) {
    validateMitre(e.mitre, errors, warnings);
  }

  if (e.meta) {
    validateMeta(e.meta, errors, warnings);
  }

  // Warnings for recommended fields
  if (!e.$schema) {
    warnings.push('Recommended field $schema is missing');
  }

  return {
    valid: errors.length === 0,
    errors,
    warnings
  };
}

function isValidEventType(type: string): boolean {
  return ['alert', 'threat_intel', 'vulnerability', 'incident', 'network_event', 'endpoint_event', 'auth_event'].includes(type);
}

function validateSource(source: unknown, errors: string[], warnings: string[]): void {
  if (typeof source !== 'object' || source === null) {
    errors.push('Source must be an object');
    return;
  }

  const s = source as Record<string, unknown>;

  if (!s.type) {
    errors.push('Missing required field in source: type');
  } else if (!['siem', 'edr', 'ids', 'firewall', 'scanner', 'custom'].includes(s.type as string)) {
    errors.push(`Invalid source type: ${s.type}`);
  }

  if (!s.name) {
    errors.push('Missing required field in source: name');
  }

  if (s.vendor && typeof s.vendor !== 'string') {
    errors.push('source.vendor must be a string');
  }

  if (s.version && typeof s.version !== 'string') {
    errors.push('source.version must be a string');
  }
}

function validateContext(context: unknown, errors: string[], warnings: string[]): void {
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
    if (network.direction && !['inbound', 'outbound', 'internal'].includes(network.direction as string)) {
      errors.push(`Invalid network direction: ${network.direction}`);
    }
  }

  if (c.cloud && typeof c.cloud === 'object') {
    const cloud = c.cloud as Record<string, unknown>;
    if (cloud.provider && !['aws', 'azure', 'gcp'].includes(cloud.provider as string)) {
      errors.push(`Invalid cloud provider: ${cloud.provider}`);
    }
  }
}

function validateMitre(mitre: unknown, errors: string[], warnings: string[]): void {
  if (typeof mitre !== 'object' || mitre === null) {
    errors.push('Mitre must be an object');
    return;
  }

  const m = mitre as Record<string, unknown>;

  if (m.tactic && typeof m.tactic === 'string' && !TACTIC_PATTERN.test(m.tactic)) {
    errors.push(`Invalid MITRE tactic ID format: ${m.tactic}. Expected TAxxxx`);
  }

  if (m.technique && typeof m.technique === 'string' && !TECHNIQUE_PATTERN.test(m.technique)) {
    errors.push(`Invalid MITRE technique ID format: ${m.technique}. Expected Txxxx or Txxxx.xxx`);
  }

  if (m.sub_technique && typeof m.sub_technique === 'string' && !/^T\d{4}\.\d{3}$/.test(m.sub_technique)) {
    errors.push(`Invalid MITRE sub-technique ID format: ${m.sub_technique}. Expected Txxxx.xxx`);
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

function validateEventData(type: EventType, data: Record<string, unknown>, errors: string[], warnings: string[]): void {
  switch (type) {
    case 'alert':
      validateAlertData(data, errors, warnings);
      break;
    case 'threat_intel':
      validateThreatIntelData(data, errors, warnings);
      break;
    case 'vulnerability':
      validateVulnerabilityData(data, errors, warnings);
      break;
    case 'incident':
      validateIncidentData(data, errors, warnings);
      break;
    case 'network_event':
      validateNetworkEventData(data, errors, warnings);
      break;
    case 'endpoint_event':
      validateEndpointEventData(data, errors, warnings);
      break;
    case 'auth_event':
      validateAuthEventData(data, errors, warnings);
      break;
  }
}

function validateAlertData(data: Record<string, unknown>, errors: string[], warnings: string[]): void {
  const requiredFields = ['alert_id', 'title', 'category', 'status', 'priority'];
  requiredFields.forEach(field => {
    if (!data[field]) {
      errors.push(`Missing required field in alert data: ${field}`);
    }
  });

  if (data.category && !['malware', 'intrusion', 'policy', 'reconnaissance', 'other'].includes(data.category as string)) {
    errors.push(`Invalid alert category: ${data.category}`);
  }

  if (data.status && !['new', 'investigating', 'resolved', 'false_positive', 'closed'].includes(data.status as string)) {
    errors.push(`Invalid alert status: ${data.status}`);
  }

  if (data.priority && !['critical', 'high', 'medium', 'low', 'info'].includes(data.priority as string)) {
    errors.push(`Invalid alert priority: ${data.priority}`);
  }
}

function validateThreatIntelData(data: Record<string, unknown>, errors: string[], warnings: string[]): void {
  const requiredFields = ['threat_type', 'threat_name', 'status'];
  requiredFields.forEach(field => {
    if (!data[field]) {
      errors.push(`Missing required field in threat_intel data: ${field}`);
    }
  });

  if (data.threat_type && !['malware', 'apt', 'campaign', 'botnet', 'ransomware', 'phishing'].includes(data.threat_type as string)) {
    errors.push(`Invalid threat_type: ${data.threat_type}`);
  }

  if (data.status && !['active', 'inactive', 'unknown'].includes(data.status as string)) {
    errors.push(`Invalid threat_intel status: ${data.status}`);
  }
}

function validateVulnerabilityData(data: Record<string, unknown>, errors: string[], warnings: string[]): void {
  const requiredFields = ['vuln_id', 'title', 'cvss'];
  requiredFields.forEach(field => {
    if (!data[field]) {
      errors.push(`Missing required field in vulnerability data: ${field}`);
    }
  });

  if (data.vuln_id && typeof data.vuln_id === 'string' && !CVE_PATTERN.test(data.vuln_id)) {
    errors.push(`Invalid CVE format: ${data.vuln_id}. Expected CVE-YYYY-NNNNN`);
  }

  if (data.cvss && typeof data.cvss === 'object') {
    const cvss = data.cvss as Record<string, unknown>;
    if (cvss.score !== undefined && (typeof cvss.score !== 'number' || cvss.score < 0 || cvss.score > 10)) {
      errors.push('cvss.score must be a number between 0 and 10');
    }
    if (cvss.severity && !['critical', 'high', 'medium', 'low', 'none'].includes(cvss.severity as string)) {
      errors.push(`Invalid CVSS severity: ${cvss.severity}`);
    }
  }

  if (data.cwe && Array.isArray(data.cwe)) {
    (data.cwe as string[]).forEach(cwe => {
      if (!CWE_PATTERN.test(cwe)) {
        errors.push(`Invalid CWE format: ${cwe}. Expected CWE-NNN`);
      }
    });
  }
}

function validateIncidentData(data: Record<string, unknown>, errors: string[], warnings: string[]): void {
  const requiredFields = ['incident_id', 'title', 'category', 'status', 'priority'];
  requiredFields.forEach(field => {
    if (!data[field]) {
      errors.push(`Missing required field in incident data: ${field}`);
    }
  });

  const validCategories = ['malware', 'phishing', 'ransomware', 'data_breach', 'ddos', 'unauthorized_access', 'insider_threat', 'apt', 'other'];
  if (data.category && !validCategories.includes(data.category as string)) {
    errors.push(`Invalid incident category: ${data.category}`);
  }

  const validStatuses = ['new', 'triaging', 'investigating', 'containing', 'eradicating', 'recovering', 'closed'];
  if (data.status && !validStatuses.includes(data.status as string)) {
    errors.push(`Invalid incident status: ${data.status}`);
  }
}

function validateNetworkEventData(data: Record<string, unknown>, errors: string[], warnings: string[]): void {
  const requiredFields = ['event_type', 'protocol', 'source', 'destination'];
  requiredFields.forEach(field => {
    if (!data[field]) {
      errors.push(`Missing required field in network_event data: ${field}`);
    }
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

function validateEndpointEventData(data: Record<string, unknown>, errors: string[], warnings: string[]): void {
  const requiredFields = ['event_type', 'host'];
  requiredFields.forEach(field => {
    if (!data[field]) {
      errors.push(`Missing required field in endpoint_event data: ${field}`);
    }
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

function validateAuthEventData(data: Record<string, unknown>, errors: string[], warnings: string[]): void {
  const requiredFields = ['event_type', 'result', 'user', 'target'];
  requiredFields.forEach(field => {
    if (!data[field]) {
      errors.push(`Missing required field in auth_event data: ${field}`);
    }
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

// CLI usage
if (typeof process !== 'undefined' && process.argv && process.argv[1]?.includes('validator')) {
  const fs = require('fs');
  const filePath = process.argv[2];

  if (!filePath) {
    console.log('Usage: npx ts-node validator.ts <event.json>');
    process.exit(1);
  }

  try {
    const content = fs.readFileSync(filePath, 'utf-8');
    const event = JSON.parse(content);
    const result = validateEvent(event);

    console.log('\n=== WIA Security Event Validation ===\n');
    console.log(`File: ${filePath}`);
    console.log(`Valid: ${result.valid ? 'YES ✓' : 'NO ✗'}\n`);

    if (result.errors.length > 0) {
      console.log('Errors:');
      result.errors.forEach(e => console.log(`  ✗ ${e}`));
    }

    if (result.warnings.length > 0) {
      console.log('\nWarnings:');
      result.warnings.forEach(w => console.log(`  ⚠ ${w}`));
    }

    if (result.valid && result.warnings.length === 0) {
      console.log('No issues found.');
    }

    process.exit(result.valid ? 0 : 1);
  } catch (error) {
    console.error('Error:', (error as Error).message);
    process.exit(1);
  }
}

export type { WiaSecurityEvent, ValidationResult, EventType, Source, Context, Mitre, Meta };
