/**
 * WIA Security TAXII 2.1 Client
 * Threat Intelligence Sharing Protocol
 */

import { WiaSecurityEvent, ThreatIntelEvent } from '../types';
import { toStixBundle, StixBundle } from '../converter';

// ============================================================================
// Types
// ============================================================================

export interface TaxiiConfig {
  serverUrl: string;
  apiRoot?: string;
  username?: string;
  password?: string;
  apiKey?: string;
  timeout?: number;
}

export interface TaxiiDiscovery {
  title: string;
  description?: string;
  contact?: string;
  default?: string;
  api_roots?: string[];
}

export interface TaxiiApiRoot {
  title: string;
  description?: string;
  versions: string[];
  max_content_length: number;
}

export interface TaxiiCollection {
  id: string;
  title: string;
  description?: string;
  can_read: boolean;
  can_write: boolean;
  media_types?: string[];
}

export interface TaxiiStatus {
  id: string;
  status: 'pending' | 'complete' | 'failed';
  request_timestamp?: string;
  total_count?: number;
  success_count?: number;
  failure_count?: number;
  pending_count?: number;
  failures?: Array<{
    id: string;
    message: string;
  }>;
}

export interface TaxiiManifestEntry {
  id: string;
  date_added: string;
  version?: string;
  media_type?: string;
}

// ============================================================================
// TAXII Client
// ============================================================================

export class TaxiiClient {
  private config: TaxiiConfig;
  private headers: Record<string, string>;

  constructor(config: TaxiiConfig) {
    this.config = {
      timeout: 30000,
      ...config
    };

    this.headers = {
      'Accept': 'application/taxii+json;version=2.1',
      'Content-Type': 'application/taxii+json;version=2.1'
    };

    if (config.apiKey) {
      this.headers['Authorization'] = `Bearer ${config.apiKey}`;
    } else if (config.username && config.password) {
      const auth = Buffer.from(`${config.username}:${config.password}`).toString('base64');
      this.headers['Authorization'] = `Basic ${auth}`;
    }
  }

  /**
   * Discover TAXII server
   */
  async discover(): Promise<TaxiiDiscovery> {
    const response = await this.request('GET', '/taxii2/');
    return response as TaxiiDiscovery;
  }

  /**
   * Get API root information
   */
  async getApiRoot(apiRoot?: string): Promise<TaxiiApiRoot> {
    const root = apiRoot || this.config.apiRoot || '';
    const response = await this.request('GET', `/${root}/`);
    return response as TaxiiApiRoot;
  }

  /**
   * List collections
   */
  async getCollections(apiRoot?: string): Promise<TaxiiCollection[]> {
    const root = apiRoot || this.config.apiRoot || '';
    const response = await this.request('GET', `/${root}/collections/`);
    return (response as { collections: TaxiiCollection[] }).collections || [];
  }

  /**
   * Get collection by ID
   */
  async getCollection(collectionId: string, apiRoot?: string): Promise<TaxiiCollection> {
    const root = apiRoot || this.config.apiRoot || '';
    const response = await this.request('GET', `/${root}/collections/${collectionId}/`);
    return response as TaxiiCollection;
  }

  /**
   * Get collection manifest
   */
  async getManifest(
    collectionId: string,
    options?: {
      addedAfter?: string;
      limit?: number;
      next?: string;
    },
    apiRoot?: string
  ): Promise<{ objects: TaxiiManifestEntry[]; more?: boolean; next?: string }> {
    const root = apiRoot || this.config.apiRoot || '';
    const params = new URLSearchParams();

    if (options?.addedAfter) params.set('added_after', options.addedAfter);
    if (options?.limit) params.set('limit', options.limit.toString());
    if (options?.next) params.set('next', options.next);

    const query = params.toString() ? `?${params.toString()}` : '';
    const response = await this.request(
      'GET',
      `/${root}/collections/${collectionId}/manifest/${query}`
    );

    return response as { objects: TaxiiManifestEntry[]; more?: boolean; next?: string };
  }

  /**
   * Get objects from collection
   */
  async getObjects(
    collectionId: string,
    options?: {
      addedAfter?: string;
      limit?: number;
      next?: string;
      type?: string[];
      id?: string[];
    },
    apiRoot?: string
  ): Promise<StixBundle> {
    const root = apiRoot || this.config.apiRoot || '';
    const params = new URLSearchParams();

    if (options?.addedAfter) params.set('added_after', options.addedAfter);
    if (options?.limit) params.set('limit', options.limit.toString());
    if (options?.next) params.set('next', options.next);
    if (options?.type) options.type.forEach(t => params.append('type', t));
    if (options?.id) options.id.forEach(i => params.append('match[id]', i));

    const query = params.toString() ? `?${params.toString()}` : '';
    const response = await this.request(
      'GET',
      `/${root}/collections/${collectionId}/objects/${query}`
    );

    return response as StixBundle;
  }

  /**
   * Get object by ID
   */
  async getObject(
    collectionId: string,
    objectId: string,
    apiRoot?: string
  ): Promise<StixBundle> {
    const root = apiRoot || this.config.apiRoot || '';
    const response = await this.request(
      'GET',
      `/${root}/collections/${collectionId}/objects/${objectId}/`
    );
    return response as StixBundle;
  }

  /**
   * Add objects to collection
   */
  async addObjects(
    collectionId: string,
    bundle: StixBundle,
    apiRoot?: string
  ): Promise<TaxiiStatus> {
    const root = apiRoot || this.config.apiRoot || '';
    const response = await this.request(
      'POST',
      `/${root}/collections/${collectionId}/objects/`,
      bundle
    );
    return response as TaxiiStatus;
  }

  /**
   * Add WIA Security event to collection (converts to STIX)
   */
  async addWiaEvent(
    collectionId: string,
    event: WiaSecurityEvent,
    apiRoot?: string
  ): Promise<TaxiiStatus> {
    const bundle = toStixBundle(event);
    return this.addObjects(collectionId, bundle, apiRoot);
  }

  /**
   * Add multiple WIA Security events to collection
   */
  async addWiaEvents(
    collectionId: string,
    events: WiaSecurityEvent[],
    apiRoot?: string
  ): Promise<TaxiiStatus[]> {
    const results: TaxiiStatus[] = [];
    for (const event of events) {
      const status = await this.addWiaEvent(collectionId, event, apiRoot);
      results.push(status);
    }
    return results;
  }

  /**
   * Get status of add request
   */
  async getStatus(statusId: string, apiRoot?: string): Promise<TaxiiStatus> {
    const root = apiRoot || this.config.apiRoot || '';
    const response = await this.request('GET', `/${root}/status/${statusId}/`);
    return response as TaxiiStatus;
  }

  /**
   * Delete object from collection
   */
  async deleteObject(
    collectionId: string,
    objectId: string,
    apiRoot?: string
  ): Promise<void> {
    const root = apiRoot || this.config.apiRoot || '';
    await this.request(
      'DELETE',
      `/${root}/collections/${collectionId}/objects/${objectId}/`
    );
  }

  // -------------------------------------------------------------------------
  // Private Methods
  // -------------------------------------------------------------------------

  private async request(
    method: string,
    path: string,
    body?: unknown
  ): Promise<unknown> {
    const url = `${this.config.serverUrl}${path}`;

    const options: RequestInit = {
      method,
      headers: this.headers,
      signal: AbortSignal.timeout(this.config.timeout || 30000)
    };

    if (body) {
      options.body = JSON.stringify(body);
    }

    const response = await fetch(url, options);

    if (!response.ok) {
      throw new Error(`TAXII request failed: ${response.status} ${response.statusText}`);
    }

    const contentType = response.headers.get('content-type');
    if (contentType?.includes('application/taxii+json') || contentType?.includes('application/json')) {
      return response.json();
    }

    return response.text();
  }
}

// ============================================================================
// Factory
// ============================================================================

export function createTaxiiClient(config: TaxiiConfig): TaxiiClient {
  return new TaxiiClient(config);
}
