/**
 * WIA Security SIEM Adapters
 * Splunk, Elastic, QRadar integration
 */

import { WiaSecurityEvent } from '../types';
import { toSplunkEvent, toElasticEvent, toEcsEvent } from '../converter';

// ============================================================================
// Splunk HEC Adapter
// ============================================================================

export interface SplunkConfig {
  url: string;
  token: string;
  index?: string;
  source?: string;
  sourcetype?: string;
  timeout?: number;
  batchSize?: number;
  flushInterval?: number;
}

export class SplunkAdapter {
  private config: SplunkConfig;
  private buffer: unknown[] = [];
  private flushTimer: NodeJS.Timeout | null = null;

  constructor(config: SplunkConfig) {
    this.config = {
      timeout: 30000,
      batchSize: 100,
      flushInterval: 5000,
      ...config
    };

    if (this.config.flushInterval) {
      this.flushTimer = setInterval(() => this.flush(), this.config.flushInterval);
    }
  }

  /**
   * Send event to Splunk
   */
  async send(event: WiaSecurityEvent): Promise<void> {
    const splunkEvent = toSplunkEvent(event, this.config.index);

    if (this.config.source) {
      splunkEvent.source = this.config.source;
    }
    if (this.config.sourcetype) {
      splunkEvent.sourcetype = this.config.sourcetype;
    }

    this.buffer.push(splunkEvent);

    if (this.buffer.length >= (this.config.batchSize || 100)) {
      await this.flush();
    }
  }

  /**
   * Send multiple events
   */
  async sendBatch(events: WiaSecurityEvent[]): Promise<void> {
    for (const event of events) {
      await this.send(event);
    }
  }

  /**
   * Flush buffer to Splunk
   */
  async flush(): Promise<void> {
    if (this.buffer.length === 0) return;

    const events = [...this.buffer];
    this.buffer = [];

    const body = events.map(e => JSON.stringify(e)).join('\n');

    const response = await fetch(`${this.config.url}/services/collector/event`, {
      method: 'POST',
      headers: {
        'Authorization': `Splunk ${this.config.token}`,
        'Content-Type': 'application/json'
      },
      body,
      signal: AbortSignal.timeout(this.config.timeout || 30000)
    });

    if (!response.ok) {
      throw new Error(`Splunk HEC error: ${response.status} ${response.statusText}`);
    }
  }

  /**
   * Close adapter
   */
  close(): void {
    if (this.flushTimer) {
      clearInterval(this.flushTimer);
      this.flushTimer = null;
    }
    this.flush().catch(console.error);
  }

  /**
   * Search Splunk
   */
  async search(query: string, options?: {
    earliest?: string;
    latest?: string;
    maxResults?: number;
  }): Promise<unknown[]> {
    const searchQuery = `search ${query}`;
    const params = new URLSearchParams({
      search: searchQuery,
      output_mode: 'json',
      earliest_time: options?.earliest || '-24h',
      latest_time: options?.latest || 'now'
    });

    if (options?.maxResults) {
      params.set('max_count', options.maxResults.toString());
    }

    const response = await fetch(
      `${this.config.url}/services/search/jobs/export?${params.toString()}`,
      {
        method: 'GET',
        headers: {
          'Authorization': `Splunk ${this.config.token}`
        },
        signal: AbortSignal.timeout(this.config.timeout || 30000)
      }
    );

    if (!response.ok) {
      throw new Error(`Splunk search error: ${response.status}`);
    }

    const text = await response.text();
    const lines = text.trim().split('\n').filter(l => l);
    return lines.map(l => JSON.parse(l));
  }
}

// ============================================================================
// Elasticsearch Adapter
// ============================================================================

export interface ElasticConfig {
  url: string;
  username?: string;
  password?: string;
  apiKey?: string;
  indexPrefix?: string;
  timeout?: number;
  batchSize?: number;
  flushInterval?: number;
}

export class ElasticAdapter {
  private config: ElasticConfig;
  private buffer: unknown[] = [];
  private flushTimer: NodeJS.Timeout | null = null;
  private headers: Record<string, string>;

  constructor(config: ElasticConfig) {
    this.config = {
      indexPrefix: 'wia-security',
      timeout: 30000,
      batchSize: 100,
      flushInterval: 5000,
      ...config
    };

    this.headers = {
      'Content-Type': 'application/json'
    };

    if (config.apiKey) {
      this.headers['Authorization'] = `ApiKey ${config.apiKey}`;
    } else if (config.username && config.password) {
      const auth = Buffer.from(`${config.username}:${config.password}`).toString('base64');
      this.headers['Authorization'] = `Basic ${auth}`;
    }

    if (this.config.flushInterval) {
      this.flushTimer = setInterval(() => this.flush(), this.config.flushInterval);
    }
  }

  /**
   * Index event
   */
  async index(event: WiaSecurityEvent): Promise<void> {
    const doc = toElasticEvent(event, this.config.indexPrefix);
    this.buffer.push(doc);

    if (this.buffer.length >= (this.config.batchSize || 100)) {
      await this.flush();
    }
  }

  /**
   * Index multiple events
   */
  async indexBatch(events: WiaSecurityEvent[]): Promise<void> {
    for (const event of events) {
      await this.index(event);
    }
  }

  /**
   * Flush buffer to Elasticsearch
   */
  async flush(): Promise<void> {
    if (this.buffer.length === 0) return;

    const docs = [...this.buffer];
    this.buffer = [];

    // Build bulk request body
    const body = docs.map(doc => {
      const d = doc as { _index: string; _id: string; _source: unknown };
      return [
        JSON.stringify({ index: { _index: d._index, _id: d._id } }),
        JSON.stringify(d._source)
      ].join('\n');
    }).join('\n') + '\n';

    const response = await fetch(`${this.config.url}/_bulk`, {
      method: 'POST',
      headers: this.headers,
      body,
      signal: AbortSignal.timeout(this.config.timeout || 30000)
    });

    if (!response.ok) {
      throw new Error(`Elasticsearch bulk error: ${response.status}`);
    }

    const result = await response.json();
    if (result.errors) {
      console.error('Elasticsearch bulk errors:', result.items.filter((i: any) => i.index?.error));
    }
  }

  /**
   * Close adapter
   */
  close(): void {
    if (this.flushTimer) {
      clearInterval(this.flushTimer);
      this.flushTimer = null;
    }
    this.flush().catch(console.error);
  }

  /**
   * Search events
   */
  async search(query: unknown, options?: {
    index?: string;
    size?: number;
    from?: number;
  }): Promise<{ hits: unknown[]; total: number }> {
    const index = options?.index || `${this.config.indexPrefix}-*`;
    const body = {
      query,
      size: options?.size || 100,
      from: options?.from || 0,
      sort: [{ '@timestamp': 'desc' }]
    };

    const response = await fetch(`${this.config.url}/${index}/_search`, {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify(body),
      signal: AbortSignal.timeout(this.config.timeout || 30000)
    });

    if (!response.ok) {
      throw new Error(`Elasticsearch search error: ${response.status}`);
    }

    const result = await response.json();
    return {
      hits: result.hits.hits.map((h: any) => h._source),
      total: typeof result.hits.total === 'number'
        ? result.hits.total
        : result.hits.total.value
    };
  }

  /**
   * Create index template
   */
  async createTemplate(name: string = 'wia-security'): Promise<void> {
    const template = {
      index_patterns: [`${this.config.indexPrefix}-*`],
      template: {
        settings: {
          number_of_shards: 1,
          number_of_replicas: 0
        },
        mappings: {
          properties: {
            '@timestamp': { type: 'date' },
            'event.id': { type: 'keyword' },
            'event.kind': { type: 'keyword' },
            'event.category': { type: 'keyword' },
            'event.severity': { type: 'integer' },
            'wia.version': { type: 'keyword' },
            'wia.type': { type: 'keyword' },
            'host.hostname': { type: 'keyword' },
            'host.ip': { type: 'ip' },
            'user.name': { type: 'keyword' },
            'source.ip': { type: 'ip' },
            'destination.ip': { type: 'ip' },
            'threat.framework': { type: 'keyword' },
            'threat.tactic.id': { type: 'keyword' },
            'threat.technique.id': { type: 'keyword' }
          }
        }
      }
    };

    const response = await fetch(`${this.config.url}/_index_template/${name}`, {
      method: 'PUT',
      headers: this.headers,
      body: JSON.stringify(template),
      signal: AbortSignal.timeout(this.config.timeout || 30000)
    });

    if (!response.ok) {
      throw new Error(`Failed to create template: ${response.status}`);
    }
  }
}

// ============================================================================
// QRadar Adapter
// ============================================================================

export interface QRadarConfig {
  url: string;
  token: string;
  timeout?: number;
}

export class QRadarAdapter {
  private config: QRadarConfig;
  private headers: Record<string, string>;

  constructor(config: QRadarConfig) {
    this.config = {
      timeout: 30000,
      ...config
    };

    this.headers = {
      'SEC': config.token,
      'Content-Type': 'application/json',
      'Accept': 'application/json'
    };
  }

  /**
   * Send event as syslog (LEEF format)
   */
  async sendSyslog(event: WiaSecurityEvent): Promise<void> {
    const leef = this.toLeef(event);

    const response = await fetch(`${this.config.url}/api/siem/log_sources/events`, {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify({ events: [leef] }),
      signal: AbortSignal.timeout(this.config.timeout || 30000)
    });

    if (!response.ok) {
      throw new Error(`QRadar syslog error: ${response.status}`);
    }
  }

  /**
   * Search using AQL
   */
  async search(aql: string): Promise<unknown[]> {
    // Create search
    const createResponse = await fetch(`${this.config.url}/api/ariel/searches`, {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify({ query_expression: aql }),
      signal: AbortSignal.timeout(this.config.timeout || 30000)
    });

    if (!createResponse.ok) {
      throw new Error(`QRadar search creation error: ${createResponse.status}`);
    }

    const { search_id } = await createResponse.json();

    // Poll for results
    let status = 'EXECUTE';
    while (status === 'EXECUTE' || status === 'SORTING') {
      await new Promise(resolve => setTimeout(resolve, 1000));

      const statusResponse = await fetch(
        `${this.config.url}/api/ariel/searches/${search_id}`,
        { headers: this.headers }
      );

      const statusResult = await statusResponse.json();
      status = statusResult.status;
    }

    // Get results
    const resultsResponse = await fetch(
      `${this.config.url}/api/ariel/searches/${search_id}/results`,
      { headers: this.headers }
    );

    const results = await resultsResponse.json();
    return results.events || [];
  }

  /**
   * Convert WIA event to LEEF format
   */
  private toLeef(event: WiaSecurityEvent): string {
    const parts = [
      'LEEF:2.0',
      'WIA',
      'Security',
      '1.0.0',
      event.type
    ];

    const attrs: string[] = [];

    // Standard LEEF attributes
    attrs.push(`devTime=${event.timestamp}`);
    attrs.push(`sev=${Math.round(event.severity)}`);
    attrs.push(`cat=${event.type}`);

    if (event.context?.host?.ip?.[0]) {
      attrs.push(`src=${event.context.host.ip[0]}`);
    }

    if (event.context?.user?.name) {
      attrs.push(`usrName=${event.context.user.name}`);
    }

    // MITRE ATT&CK
    if (event.mitre?.tactic) {
      attrs.push(`mitreTactic=${event.mitre.tactic}`);
    }
    if (event.mitre?.technique) {
      attrs.push(`mitreTechnique=${event.mitre.technique}`);
    }

    return `${parts.join('|')}\t${attrs.join('\t')}`;
  }
}

// ============================================================================
// Factory Functions
// ============================================================================

export function createSplunkAdapter(config: SplunkConfig): SplunkAdapter {
  return new SplunkAdapter(config);
}

export function createElasticAdapter(config: ElasticConfig): ElasticAdapter {
  return new ElasticAdapter(config);
}

export function createQRadarAdapter(config: QRadarConfig): QRadarAdapter {
  return new QRadarAdapter(config);
}
