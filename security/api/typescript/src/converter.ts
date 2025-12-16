/**
 * WIA Security Event Converter
 * Utilities for converting between WIA Security and other formats
 */

import { v4 as uuidv4 } from 'uuid';
import {
  WiaSecurityEvent,
  ThreatIntelEvent,
  AlertEvent,
  VulnerabilityEvent,
  Indicator
} from './types';

// ============================================================================
// STIX 2.1 Conversion
// ============================================================================

export interface StixBundle {
  type: 'bundle';
  id: string;
  objects: StixObject[];
}

export interface StixObject {
  type: string;
  spec_version: '2.1';
  id: string;
  created: string;
  modified: string;
  [key: string]: unknown;
}

export interface StixIndicator extends StixObject {
  type: 'indicator';
  name: string;
  pattern: string;
  pattern_type: 'stix';
  valid_from: string;
  labels?: string[];
  description?: string;
  kill_chain_phases?: Array<{
    kill_chain_name: string;
    phase_name: string;
  }>;
}

/**
 * Convert WIA Security event to STIX 2.1 Bundle
 */
export function toStixBundle(event: WiaSecurityEvent): StixBundle {
  const bundle: StixBundle = {
    type: 'bundle',
    id: `bundle--${uuidv4()}`,
    objects: []
  };

  if (event.type === 'threat_intel') {
    const threatEvent = event as ThreatIntelEvent;

    // Convert indicators to STIX indicators
    if (threatEvent.data.indicators) {
      for (const indicator of threatEvent.data.indicators) {
        bundle.objects.push(toStixIndicator(indicator, event.timestamp));
      }
    }

    // Add malware object if applicable
    if (threatEvent.data.threat_type === 'malware' || threatEvent.data.threat_type === 'ransomware') {
      bundle.objects.push({
        type: 'malware',
        spec_version: '2.1',
        id: `malware--${uuidv4()}`,
        created: event.timestamp,
        modified: event.timestamp,
        name: threatEvent.data.threat_name,
        malware_types: [threatEvent.data.threat_type],
        is_family: !!threatEvent.data.threat_family,
        aliases: threatEvent.data.aliases || []
      });
    }

    // Add threat actor if APT
    if (threatEvent.data.threat_type === 'apt') {
      bundle.objects.push({
        type: 'threat-actor',
        spec_version: '2.1',
        id: `threat-actor--${uuidv4()}`,
        created: event.timestamp,
        modified: event.timestamp,
        name: threatEvent.data.threat_name,
        aliases: threatEvent.data.aliases || [],
        threat_actor_types: ['nation-state'],
        primary_motivation: 'organizational-gain'
      });
    }
  }

  if (event.type === 'alert') {
    const alertEvent = event as AlertEvent;

    // Convert to sighting
    bundle.objects.push({
      type: 'sighting',
      spec_version: '2.1',
      id: `sighting--${uuidv4()}`,
      created: event.timestamp,
      modified: event.timestamp,
      first_seen: alertEvent.data.first_seen || event.timestamp,
      last_seen: alertEvent.data.last_seen || event.timestamp,
      count: alertEvent.data.count || 1,
      description: alertEvent.data.description
    });

    // Convert indicators
    if (alertEvent.data.indicators) {
      for (const indicator of alertEvent.data.indicators) {
        bundle.objects.push(toStixIndicator(indicator, event.timestamp));
      }
    }
  }

  if (event.type === 'vulnerability') {
    const vulnEvent = event as VulnerabilityEvent;

    bundle.objects.push({
      type: 'vulnerability',
      spec_version: '2.1',
      id: `vulnerability--${uuidv4()}`,
      created: event.timestamp,
      modified: event.timestamp,
      name: vulnEvent.data.vuln_id,
      description: vulnEvent.data.description || vulnEvent.data.title,
      external_references: [
        {
          source_name: 'cve',
          external_id: vulnEvent.data.vuln_id
        }
      ]
    });
  }

  return bundle;
}

/**
 * Convert WIA indicator to STIX indicator
 */
export function toStixIndicator(indicator: Indicator, timestamp: string): StixIndicator {
  let pattern: string;

  switch (indicator.type) {
    case 'ip':
      pattern = `[ipv4-addr:value = '${indicator.value}']`;
      break;
    case 'domain':
      pattern = `[domain-name:value = '${indicator.value}']`;
      break;
    case 'url':
      pattern = `[url:value = '${indicator.value}']`;
      break;
    case 'hash':
    case 'file_hash_md5':
      pattern = `[file:hashes.MD5 = '${indicator.value}']`;
      break;
    case 'file_hash_sha1':
      pattern = `[file:hashes.'SHA-1' = '${indicator.value}']`;
      break;
    case 'file_hash_sha256':
      pattern = `[file:hashes.'SHA-256' = '${indicator.value}']`;
      break;
    case 'email':
      pattern = `[email-addr:value = '${indicator.value}']`;
      break;
    default:
      pattern = `[x-custom:value = '${indicator.value}']`;
  }

  return {
    type: 'indicator',
    spec_version: '2.1',
    id: `indicator--${uuidv4()}`,
    created: timestamp,
    modified: timestamp,
    name: indicator.context || `${indicator.type}: ${indicator.value}`,
    pattern,
    pattern_type: 'stix',
    valid_from: indicator.first_seen || timestamp
  };
}

/**
 * Convert STIX indicator to WIA indicator
 */
export function fromStixIndicator(stix: StixIndicator): Indicator {
  const patternMatch = stix.pattern.match(/\[(\w+(?:-\w+)?):(\w+(?:\.\w+)*)\s*=\s*'([^']+)'\]/);

  if (!patternMatch) {
    return {
      type: 'hash',
      value: stix.pattern,
      context: stix.description
    };
  }

  const [, objType, , value] = patternMatch;
  let type: Indicator['type'] = 'hash';

  switch (objType) {
    case 'ipv4-addr':
    case 'ipv6-addr':
      type = 'ip';
      break;
    case 'domain-name':
      type = 'domain';
      break;
    case 'url':
      type = 'url';
      break;
    case 'file':
      type = 'file_hash_sha256';
      break;
    case 'email-addr':
      type = 'email';
      break;
  }

  return {
    type,
    value,
    context: stix.description,
    first_seen: stix.valid_from
  };
}

// ============================================================================
// ECS (Elastic Common Schema) Conversion
// ============================================================================

export interface EcsEvent {
  '@timestamp': string;
  event: {
    id: string;
    kind: string;
    category: string[];
    type: string[];
    severity: number;
    created: string;
    original?: string;
  };
  source?: {
    ip?: string;
    port?: number;
    domain?: string;
    geo?: {
      country_iso_code?: string;
      city_name?: string;
    };
  };
  destination?: {
    ip?: string;
    port?: number;
    domain?: string;
  };
  host?: {
    hostname?: string;
    ip?: string[];
    mac?: string[];
    os?: {
      name?: string;
      version?: string;
    };
  };
  user?: {
    name?: string;
    domain?: string;
    email?: string;
  };
  threat?: {
    framework?: string;
    tactic?: {
      id?: string[];
      name?: string[];
    };
    technique?: {
      id?: string[];
      name?: string[];
    };
  };
  tags?: string[];
  labels?: Record<string, string>;
  wia?: {
    version: string;
    type: string;
    data: unknown;
  };
}

/**
 * Convert WIA Security event to ECS format
 */
export function toEcsEvent(event: WiaSecurityEvent): EcsEvent {
  const ecs: EcsEvent = {
    '@timestamp': event.timestamp,
    event: {
      id: event.id,
      kind: mapEventKind(event.type),
      category: mapEventCategory(event.type),
      type: mapEventType(event.type),
      severity: event.severity,
      created: event.timestamp,
      original: event.meta?.raw
    },
    wia: {
      version: event.version,
      type: event.type,
      data: event.data
    }
  };

  // Map context
  if (event.context?.host) {
    ecs.host = {
      hostname: event.context.host.hostname,
      ip: event.context.host.ip,
      mac: event.context.host.mac,
      os: event.context.host.os ? {
        name: event.context.host.os.name,
        version: event.context.host.os.version
      } : undefined
    };
  }

  if (event.context?.user) {
    ecs.user = {
      name: event.context.user.name,
      domain: event.context.user.domain,
      email: event.context.user.email
    };
  }

  if (event.context?.network) {
    if (event.context.network.source) {
      ecs.source = {
        ip: event.context.network.source.ip,
        port: event.context.network.source.port,
        domain: event.context.network.source.hostname
      };
    }
    if (event.context.network.destination) {
      ecs.destination = {
        ip: event.context.network.destination.ip,
        port: event.context.network.destination.port,
        domain: event.context.network.destination.hostname
      };
    }
  }

  // Map MITRE ATT&CK
  if (event.mitre) {
    ecs.threat = {
      framework: 'MITRE ATT&CK',
      tactic: event.mitre.tactic ? {
        id: [event.mitre.tactic],
        name: event.mitre.tactic_name ? [event.mitre.tactic_name] : undefined
      } : undefined,
      technique: event.mitre.technique ? {
        id: [event.mitre.technique],
        name: event.mitre.technique_name ? [event.mitre.technique_name] : undefined
      } : undefined
    };
  }

  // Map meta
  if (event.meta?.tags) {
    ecs.tags = event.meta.tags;
  }
  if (event.meta?.labels) {
    ecs.labels = event.meta.labels;
  }

  return ecs;
}

function mapEventKind(type: string): string {
  const kindMap: Record<string, string> = {
    alert: 'alert',
    threat_intel: 'enrichment',
    vulnerability: 'state',
    incident: 'alert',
    network_event: 'event',
    endpoint_event: 'event',
    auth_event: 'event'
  };
  return kindMap[type] || 'event';
}

function mapEventCategory(type: string): string[] {
  const categoryMap: Record<string, string[]> = {
    alert: ['intrusion_detection'],
    threat_intel: ['threat'],
    vulnerability: ['vulnerability'],
    incident: ['intrusion_detection'],
    network_event: ['network'],
    endpoint_event: ['host'],
    auth_event: ['authentication']
  };
  return categoryMap[type] || ['configuration'];
}

function mapEventType(type: string): string[] {
  const typeMap: Record<string, string[]> = {
    alert: ['info'],
    threat_intel: ['indicator'],
    vulnerability: ['info'],
    incident: ['info'],
    network_event: ['connection'],
    endpoint_event: ['info'],
    auth_event: ['start']
  };
  return typeMap[type] || ['info'];
}

// ============================================================================
// OCSF Conversion
// ============================================================================

export interface OcsfEvent {
  class_uid: number;
  class_name: string;
  category_uid: number;
  category_name: string;
  severity_id: number;
  severity: string;
  activity_id: number;
  activity_name: string;
  time: number;
  metadata: {
    version: string;
    product: {
      name: string;
      vendor_name: string;
    };
    uid: string;
    original_time: string;
  };
  observables?: Array<{
    type_id: number;
    type: string;
    value: string;
  }>;
  attacks?: Array<{
    tactic: {
      uid: string;
      name: string;
    };
    technique: {
      uid: string;
      name: string;
    };
  }>;
  unmapped?: Record<string, unknown>;
}

/**
 * Convert WIA Security event to OCSF format
 */
export function toOcsfEvent(event: WiaSecurityEvent): OcsfEvent {
  const { classUid, className, categoryUid, categoryName } = mapOcsfClass(event.type);

  const ocsf: OcsfEvent = {
    class_uid: classUid,
    class_name: className,
    category_uid: categoryUid,
    category_name: categoryName,
    severity_id: mapOcsfSeverity(event.severity),
    severity: mapOcsfSeverityName(event.severity),
    activity_id: 1,
    activity_name: 'Create',
    time: new Date(event.timestamp).getTime(),
    metadata: {
      version: '1.1.0',
      product: {
        name: event.source.name,
        vendor_name: event.source.vendor || 'WIA'
      },
      uid: event.id,
      original_time: event.timestamp
    },
    unmapped: {
      wia_version: event.version,
      wia_type: event.type,
      wia_data: event.data
    }
  };

  // Map MITRE ATT&CK
  if (event.mitre) {
    ocsf.attacks = [{
      tactic: {
        uid: event.mitre.tactic || '',
        name: event.mitre.tactic_name || ''
      },
      technique: {
        uid: event.mitre.technique || '',
        name: event.mitre.technique_name || ''
      }
    }];
  }

  return ocsf;
}

function mapOcsfClass(type: string): { classUid: number; className: string; categoryUid: number; categoryName: string } {
  const classMap: Record<string, { classUid: number; className: string; categoryUid: number; categoryName: string }> = {
    alert: { classUid: 2001, className: 'Security Finding', categoryUid: 2, categoryName: 'Findings' },
    threat_intel: { classUid: 2001, className: 'Security Finding', categoryUid: 2, categoryName: 'Findings' },
    vulnerability: { classUid: 2002, className: 'Vulnerability Finding', categoryUid: 2, categoryName: 'Findings' },
    incident: { classUid: 2003, className: 'Incident Finding', categoryUid: 2, categoryName: 'Findings' },
    network_event: { classUid: 4001, className: 'Network Activity', categoryUid: 4, categoryName: 'Network Activity' },
    endpoint_event: { classUid: 1001, className: 'Process Activity', categoryUid: 1, categoryName: 'System Activity' },
    auth_event: { classUid: 3001, className: 'Authentication', categoryUid: 3, categoryName: 'Identity & Access' }
  };
  return classMap[type] || { classUid: 0, className: 'Unknown', categoryUid: 0, categoryName: 'Unknown' };
}

function mapOcsfSeverity(severity: number): number {
  if (severity === 0) return 0; // Unknown
  if (severity <= 2) return 1; // Informational
  if (severity <= 4) return 2; // Low
  if (severity <= 6) return 3; // Medium
  if (severity <= 8) return 4; // High
  return 5; // Critical
}

function mapOcsfSeverityName(severity: number): string {
  const names = ['Unknown', 'Informational', 'Low', 'Medium', 'High', 'Critical'];
  return names[mapOcsfSeverity(severity)];
}

// ============================================================================
// SIEM Format Conversion
// ============================================================================

export interface SplunkEvent {
  time: number;
  host: string;
  source: string;
  sourcetype: string;
  index?: string;
  event: Record<string, unknown>;
}

/**
 * Convert WIA Security event to Splunk HEC format
 */
export function toSplunkEvent(event: WiaSecurityEvent, index?: string): SplunkEvent {
  return {
    time: new Date(event.timestamp).getTime() / 1000,
    host: event.context?.host?.hostname || 'unknown',
    source: event.source.name,
    sourcetype: `wia:security:${event.type}`,
    index,
    event: {
      ...event,
      _wia_version: event.version,
      _wia_type: event.type
    }
  };
}

export interface ElasticDocument {
  _index: string;
  _id: string;
  _source: Record<string, unknown>;
}

/**
 * Convert WIA Security event to Elasticsearch document format
 */
export function toElasticEvent(event: WiaSecurityEvent, indexPrefix = 'wia-security'): ElasticDocument {
  const date = new Date(event.timestamp);
  const dateStr = date.toISOString().split('T')[0].replace(/-/g, '.');

  return {
    _index: `${indexPrefix}-${event.type}-${dateStr}`,
    _id: event.id,
    _source: {
      '@timestamp': event.timestamp,
      ...toEcsEvent(event)
    }
  };
}
