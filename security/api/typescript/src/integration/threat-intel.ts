/**
 * WIA Security Threat Intelligence Feed Integration
 * MISP, OTX, VirusTotal
 */

import { WiaSecurityEvent, ThreatIntelEvent, Indicator, IndicatorType } from '../types';

// ============================================================================
// Types
// ============================================================================

export interface ThreatIntelFeedConfig {
  url: string;
  apiKey: string;
  timeout?: number;
  pollInterval?: number;
}

export interface IoC {
  type: IndicatorType;
  value: string;
  confidence?: number;
  firstSeen?: string;
  lastSeen?: string;
  tags?: string[];
  source?: string;
}

export interface ThreatFeed {
  id: string;
  name: string;
  provider: string;
  lastUpdated: string;
  indicators: IoC[];
}

// ============================================================================
// MISP Integration
// ============================================================================

export interface MispConfig extends ThreatIntelFeedConfig {
  verifyTls?: boolean;
}

export class MispClient {
  private config: MispConfig;
  private headers: Record<string, string>;

  constructor(config: MispConfig) {
    this.config = {
      timeout: 30000,
      pollInterval: 300000, // 5 minutes
      verifyTls: true,
      ...config
    };

    this.headers = {
      'Authorization': config.apiKey,
      'Accept': 'application/json',
      'Content-Type': 'application/json'
    };
  }

  /**
   * Search for events
   */
  async searchEvents(query: {
    type?: string;
    value?: string;
    category?: string;
    org?: string;
    tags?: string[];
    from?: string;
    to?: string;
    limit?: number;
  }): Promise<ThreatFeed> {
    const body: Record<string, unknown> = {
      returnFormat: 'json',
      limit: query.limit || 100
    };

    if (query.type) body.type = query.type;
    if (query.value) body.value = query.value;
    if (query.category) body.category = query.category;
    if (query.org) body.org = query.org;
    if (query.tags) body.tags = query.tags;
    if (query.from) body.from = query.from;
    if (query.to) body.to = query.to;

    const response = await fetch(`${this.config.url}/events/restSearch`, {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify(body),
      signal: AbortSignal.timeout(this.config.timeout || 30000)
    });

    if (!response.ok) {
      throw new Error(`MISP search error: ${response.status}`);
    }

    const data = await response.json();
    return this.parseMispResponse(data);
  }

  /**
   * Get attributes by type
   */
  async getAttributes(type: string, limit: number = 100): Promise<IoC[]> {
    const response = await fetch(
      `${this.config.url}/attributes/restSearch`,
      {
        method: 'POST',
        headers: this.headers,
        body: JSON.stringify({
          returnFormat: 'json',
          type,
          limit
        }),
        signal: AbortSignal.timeout(this.config.timeout || 30000)
      }
    );

    if (!response.ok) {
      throw new Error(`MISP attributes error: ${response.status}`);
    }

    const data = await response.json();
    return this.parseMispAttributes(data);
  }

  /**
   * Add event to MISP
   */
  async addEvent(event: WiaSecurityEvent): Promise<string> {
    const mispEvent = this.toMispEvent(event);

    const response = await fetch(`${this.config.url}/events/add`, {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify(mispEvent),
      signal: AbortSignal.timeout(this.config.timeout || 30000)
    });

    if (!response.ok) {
      throw new Error(`MISP add event error: ${response.status}`);
    }

    const result = await response.json();
    return result.Event.uuid;
  }

  private parseMispResponse(data: any): ThreatFeed {
    const indicators: IoC[] = [];

    for (const event of data.response || []) {
      const e = event.Event;
      for (const attr of e.Attribute || []) {
        indicators.push({
          type: this.mapMispType(attr.type),
          value: attr.value,
          confidence: attr.to_ids ? 80 : 50,
          firstSeen: attr.first_seen,
          lastSeen: attr.last_seen,
          tags: (e.Tag || []).map((t: any) => t.name),
          source: 'MISP'
        });
      }
    }

    return {
      id: `misp-${Date.now()}`,
      name: 'MISP Feed',
      provider: 'MISP',
      lastUpdated: new Date().toISOString(),
      indicators
    };
  }

  private parseMispAttributes(data: any): IoC[] {
    return (data.response?.Attribute || []).map((attr: any) => ({
      type: this.mapMispType(attr.type),
      value: attr.value,
      confidence: attr.to_ids ? 80 : 50,
      firstSeen: attr.first_seen,
      lastSeen: attr.last_seen,
      source: 'MISP'
    }));
  }

  private mapMispType(mispType: string): IndicatorType {
    const mapping: Record<string, IndicatorType> = {
      'ip-src': 'ipv4',
      'ip-dst': 'ipv4',
      'domain': 'domain',
      'hostname': 'domain',
      'url': 'url',
      'md5': 'md5',
      'sha1': 'sha1',
      'sha256': 'sha256',
      'email-src': 'email',
      'email-dst': 'email'
    };
    return mapping[mispType] || 'other';
  }

  private toMispEvent(event: WiaSecurityEvent): Record<string, unknown> {
    const threatIntel = event as ThreatIntelEvent;

    return {
      Event: {
        info: event.description,
        threat_level_id: this.severityToThreatLevel(event.severity),
        analysis: 2,
        distribution: 0,
        Attribute: (threatIntel.indicators || []).map(ind => ({
          type: this.indicatorToMispType(ind.type),
          value: ind.value,
          category: 'Network activity',
          to_ids: true
        }))
      }
    };
  }

  private severityToThreatLevel(severity: number): number {
    if (severity >= 9) return 1; // High
    if (severity >= 6) return 2; // Medium
    if (severity >= 3) return 3; // Low
    return 4; // Undefined
  }

  private indicatorToMispType(type: IndicatorType): string {
    const mapping: Record<IndicatorType, string> = {
      'ipv4': 'ip-dst',
      'ipv6': 'ip-dst',
      'domain': 'domain',
      'url': 'url',
      'email': 'email-dst',
      'md5': 'md5',
      'sha1': 'sha1',
      'sha256': 'sha256',
      'ssdeep': 'ssdeep',
      'imphash': 'imphash',
      'other': 'text'
    };
    return mapping[type] || 'text';
  }
}

// ============================================================================
// AlienVault OTX Integration
// ============================================================================

export interface OtxConfig extends ThreatIntelFeedConfig {}

export class OtxClient {
  private config: OtxConfig;
  private headers: Record<string, string>;

  constructor(config: OtxConfig) {
    this.config = {
      url: 'https://otx.alienvault.com',
      timeout: 30000,
      ...config
    };

    this.headers = {
      'X-OTX-API-KEY': config.apiKey,
      'Accept': 'application/json'
    };
  }

  /**
   * Get subscribed pulses
   */
  async getSubscribedPulses(limit: number = 10): Promise<ThreatFeed[]> {
    const response = await fetch(
      `${this.config.url}/api/v1/pulses/subscribed?limit=${limit}`,
      {
        headers: this.headers,
        signal: AbortSignal.timeout(this.config.timeout || 30000)
      }
    );

    if (!response.ok) {
      throw new Error(`OTX pulses error: ${response.status}`);
    }

    const data = await response.json();
    return data.results.map((pulse: any) => this.parsePulse(pulse));
  }

  /**
   * Get pulse by ID
   */
  async getPulse(pulseId: string): Promise<ThreatFeed> {
    const response = await fetch(
      `${this.config.url}/api/v1/pulses/${pulseId}`,
      {
        headers: this.headers,
        signal: AbortSignal.timeout(this.config.timeout || 30000)
      }
    );

    if (!response.ok) {
      throw new Error(`OTX pulse error: ${response.status}`);
    }

    const data = await response.json();
    return this.parsePulse(data);
  }

  /**
   * Search indicators
   */
  async searchIndicators(
    type: 'IPv4' | 'domain' | 'hostname' | 'url' | 'FileHash-MD5' | 'FileHash-SHA1' | 'FileHash-SHA256',
    value: string
  ): Promise<IoC[]> {
    const response = await fetch(
      `${this.config.url}/api/v1/indicators/${type}/${value}/general`,
      {
        headers: this.headers,
        signal: AbortSignal.timeout(this.config.timeout || 30000)
      }
    );

    if (!response.ok) {
      throw new Error(`OTX indicator search error: ${response.status}`);
    }

    const data = await response.json();
    return this.parseIndicatorResponse(data, type, value);
  }

  private parsePulse(pulse: any): ThreatFeed {
    const indicators: IoC[] = (pulse.indicators || []).map((ind: any) => ({
      type: this.mapOtxType(ind.type),
      value: ind.indicator,
      confidence: 70,
      firstSeen: ind.created,
      tags: pulse.tags || [],
      source: 'OTX'
    }));

    return {
      id: pulse.id,
      name: pulse.name,
      provider: 'AlienVault OTX',
      lastUpdated: pulse.modified,
      indicators
    };
  }

  private parseIndicatorResponse(data: any, type: string, value: string): IoC[] {
    if (data.pulse_info?.count > 0) {
      return [{
        type: this.mapOtxType(type),
        value,
        confidence: Math.min(data.pulse_info.count * 10, 100),
        tags: data.pulse_info.pulses?.flatMap((p: any) => p.tags || []) || [],
        source: 'OTX'
      }];
    }
    return [];
  }

  private mapOtxType(otxType: string): IndicatorType {
    const mapping: Record<string, IndicatorType> = {
      'IPv4': 'ipv4',
      'IPv6': 'ipv6',
      'domain': 'domain',
      'hostname': 'domain',
      'url': 'url',
      'URL': 'url',
      'FileHash-MD5': 'md5',
      'FileHash-SHA1': 'sha1',
      'FileHash-SHA256': 'sha256',
      'email': 'email'
    };
    return mapping[otxType] || 'other';
  }
}

// ============================================================================
// VirusTotal Integration
// ============================================================================

export interface VirusTotalConfig extends ThreatIntelFeedConfig {}

export class VirusTotalClient {
  private config: VirusTotalConfig;
  private headers: Record<string, string>;

  constructor(config: VirusTotalConfig) {
    this.config = {
      url: 'https://www.virustotal.com/api/v3',
      timeout: 30000,
      ...config
    };

    this.headers = {
      'x-apikey': config.apiKey,
      'Accept': 'application/json'
    };
  }

  /**
   * Get file report by hash
   */
  async getFileReport(hash: string): Promise<IoC | null> {
    const response = await fetch(
      `${this.config.url}/files/${hash}`,
      {
        headers: this.headers,
        signal: AbortSignal.timeout(this.config.timeout || 30000)
      }
    );

    if (response.status === 404) {
      return null;
    }

    if (!response.ok) {
      throw new Error(`VirusTotal file error: ${response.status}`);
    }

    const data = await response.json();
    return this.parseFileResponse(data);
  }

  /**
   * Get URL report
   */
  async getUrlReport(url: string): Promise<IoC | null> {
    const urlId = Buffer.from(url).toString('base64').replace(/=/g, '');

    const response = await fetch(
      `${this.config.url}/urls/${urlId}`,
      {
        headers: this.headers,
        signal: AbortSignal.timeout(this.config.timeout || 30000)
      }
    );

    if (response.status === 404) {
      return null;
    }

    if (!response.ok) {
      throw new Error(`VirusTotal URL error: ${response.status}`);
    }

    const data = await response.json();
    return this.parseUrlResponse(data, url);
  }

  /**
   * Get domain report
   */
  async getDomainReport(domain: string): Promise<IoC | null> {
    const response = await fetch(
      `${this.config.url}/domains/${domain}`,
      {
        headers: this.headers,
        signal: AbortSignal.timeout(this.config.timeout || 30000)
      }
    );

    if (response.status === 404) {
      return null;
    }

    if (!response.ok) {
      throw new Error(`VirusTotal domain error: ${response.status}`);
    }

    const data = await response.json();
    return this.parseDomainResponse(data, domain);
  }

  /**
   * Get IP report
   */
  async getIpReport(ip: string): Promise<IoC | null> {
    const response = await fetch(
      `${this.config.url}/ip_addresses/${ip}`,
      {
        headers: this.headers,
        signal: AbortSignal.timeout(this.config.timeout || 30000)
      }
    );

    if (response.status === 404) {
      return null;
    }

    if (!response.ok) {
      throw new Error(`VirusTotal IP error: ${response.status}`);
    }

    const data = await response.json();
    return this.parseIpResponse(data, ip);
  }

  private parseFileResponse(data: any): IoC {
    const attrs = data.data.attributes;
    const stats = attrs.last_analysis_stats || {};
    const malicious = stats.malicious || 0;
    const total = Object.values(stats).reduce((a: number, b: any) => a + b, 0);

    return {
      type: this.detectHashType(data.data.id),
      value: data.data.id,
      confidence: total > 0 ? (malicious / total) * 100 : 0,
      firstSeen: attrs.first_submission_date
        ? new Date(attrs.first_submission_date * 1000).toISOString()
        : undefined,
      lastSeen: attrs.last_analysis_date
        ? new Date(attrs.last_analysis_date * 1000).toISOString()
        : undefined,
      tags: attrs.tags || [],
      source: 'VirusTotal'
    };
  }

  private parseUrlResponse(data: any, url: string): IoC {
    const attrs = data.data.attributes;
    const stats = attrs.last_analysis_stats || {};
    const malicious = stats.malicious || 0;
    const total = Object.values(stats).reduce((a: number, b: any) => a + b, 0);

    return {
      type: 'url',
      value: url,
      confidence: total > 0 ? (malicious / total) * 100 : 0,
      lastSeen: attrs.last_analysis_date
        ? new Date(attrs.last_analysis_date * 1000).toISOString()
        : undefined,
      tags: attrs.tags || [],
      source: 'VirusTotal'
    };
  }

  private parseDomainResponse(data: any, domain: string): IoC {
    const attrs = data.data.attributes;
    const stats = attrs.last_analysis_stats || {};
    const malicious = stats.malicious || 0;
    const total = Object.values(stats).reduce((a: number, b: any) => a + b, 0);

    return {
      type: 'domain',
      value: domain,
      confidence: total > 0 ? (malicious / total) * 100 : 0,
      lastSeen: attrs.last_analysis_date
        ? new Date(attrs.last_analysis_date * 1000).toISOString()
        : undefined,
      tags: attrs.tags || [],
      source: 'VirusTotal'
    };
  }

  private parseIpResponse(data: any, ip: string): IoC {
    const attrs = data.data.attributes;
    const stats = attrs.last_analysis_stats || {};
    const malicious = stats.malicious || 0;
    const total = Object.values(stats).reduce((a: number, b: any) => a + b, 0);

    return {
      type: ip.includes(':') ? 'ipv6' : 'ipv4',
      value: ip,
      confidence: total > 0 ? (malicious / total) * 100 : 0,
      lastSeen: attrs.last_analysis_date
        ? new Date(attrs.last_analysis_date * 1000).toISOString()
        : undefined,
      source: 'VirusTotal'
    };
  }

  private detectHashType(hash: string): IndicatorType {
    switch (hash.length) {
      case 32: return 'md5';
      case 40: return 'sha1';
      case 64: return 'sha256';
      default: return 'other';
    }
  }
}

// ============================================================================
// Factory Functions
// ============================================================================

export function createMispClient(config: MispConfig): MispClient {
  return new MispClient(config);
}

export function createOtxClient(config: OtxConfig): OtxClient {
  return new OtxClient(config);
}

export function createVirusTotalClient(config: VirusTotalConfig): VirusTotalClient {
  return new VirusTotalClient(config);
}
