/**
 * WIA-EDU-024: Museum Digital Archive Standard - TypeScript SDK
 *
 * @description Official SDK for the WIA Museum Digital Archive Standard
 * @standard WIA-EDU-024
 * @version 1.0.0
 * @philosophy 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import {
  ClientConfig,
  MuseumObject,
  Exhibition,
  Department,
  SearchQuery,
  SearchResults,
  APIResponse,
  APIError,
  CollectionStats,
  IIIFManifest,
  WebhookRegistration,
  ExportFormat,
  BatchResult,
} from './types';

/**
 * Main client for Museum Digital Archive API
 */
export class MuseumArchiveClient {
  private client: AxiosInstance;
  private config: ClientConfig;

  public objects: ObjectsAPI;
  public exhibitions: ExhibitionsAPI;
  public departments: DepartmentsAPI;
  public creators: CreatorsAPI;
  public iiif: IIIFAPI;
  public education: EducationAPI;
  public webhooks: WebhooksAPI;

  constructor(config: ClientConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      ...config,
    };

    this.client = axios.create({
      baseURL: this.config.baseURL,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'User-Agent': 'WIA-Museum-Archive-SDK/1.0.0',
        ...(this.config.apiKey && { 'X-API-Key': this.config.apiKey }),
        ...(this.config.accessToken && {
          Authorization: `Bearer ${this.config.accessToken}`,
        }),
      },
    });

    // Initialize API modules
    this.objects = new ObjectsAPI(this.client);
    this.exhibitions = new ExhibitionsAPI(this.client);
    this.departments = new DepartmentsAPI(this.client);
    this.creators = new CreatorsAPI(this.client);
    this.iiif = new IIIFAPI(this.client);
    this.education = new EducationAPI(this.client);
    this.webhooks = new WebhooksAPI(this.client);

    // Setup error handling
    this.setupErrorHandling();
  }

  private setupErrorHandling(): void {
    this.client.interceptors.response.use(
      (response) => response,
      (error: AxiosError<APIError>) => {
        if (error.response?.data?.error) {
          throw new MuseumArchiveError(
            error.response.data.error.message,
            error.response.data.error.code,
            error.response.status,
            error.response.data.error.details
          );
        }
        throw error;
      }
    );
  }

  /**
   * Get collection statistics
   */
  async getStats(): Promise<CollectionStats> {
    const response = await this.client.get<CollectionStats>('/stats');
    return response.data;
  }
}

/**
 * Objects API module
 */
class ObjectsAPI {
  constructor(private client: AxiosInstance) {}

  /**
   * List museum objects
   */
  async list(params?: {
    page?: number;
    pageSize?: number;
    department?: string;
    medium?: string;
    hasImage?: boolean;
    onView?: boolean;
  }): Promise<APIResponse<MuseumObject[]>> {
    const response = await this.client.get<APIResponse<MuseumObject[]>>(
      '/objects',
      { params }
    );
    return response.data;
  }

  /**
   * Get a single object by ID
   */
  async get(
    id: string,
    options?: {
      include?: string[];
    }
  ): Promise<MuseumObject> {
    const response = await this.client.get<MuseumObject>(`/objects/${id}`, {
      params: options?.include
        ? { include: options.include.join(',') }
        : undefined,
    });
    return response.data;
  }

  /**
   * Search objects
   */
  async search(query: SearchQuery): Promise<SearchResults> {
    const response = await this.client.post<SearchResults>(
      '/objects/search',
      query
    );
    return response.data;
  }

  /**
   * Get random objects
   */
  async random(params?: {
    count?: number;
    department?: string;
    hasImage?: boolean;
  }): Promise<APIResponse<MuseumObject[]>> {
    const response = await this.client.get<APIResponse<MuseumObject[]>>(
      '/objects/random',
      { params }
    );
    return response.data;
  }

  /**
   * Get related objects
   */
  async getRelated(id: string): Promise<APIResponse<MuseumObject[]>> {
    const response = await this.client.get<APIResponse<MuseumObject[]>>(
      `/objects/${id}/related`
    );
    return response.data;
  }

  /**
   * Export objects
   */
  async export(
    format: ExportFormat,
    filters?: SearchQuery
  ): Promise<Blob | string> {
    const response = await this.client.post(
      '/objects/export',
      { format, ...filters },
      { responseType: format === 'csv' ? 'blob' : 'text' }
    );
    return response.data;
  }
}

/**
 * Exhibitions API module
 */
class ExhibitionsAPI {
  constructor(private client: AxiosInstance) {}

  /**
   * List exhibitions
   */
  async list(params?: {
    status?: 'current' | 'upcoming' | 'past';
    type?: 'permanent' | 'temporary' | 'virtual' | 'traveling';
    page?: number;
    pageSize?: number;
  }): Promise<APIResponse<Exhibition[]>> {
    const response = await this.client.get<APIResponse<Exhibition[]>>(
      '/exhibitions',
      { params }
    );
    return response.data;
  }

  /**
   * Get exhibition details
   */
  async get(id: string): Promise<Exhibition> {
    const response = await this.client.get<Exhibition>(`/exhibitions/${id}`);
    return response.data;
  }

  /**
   * Get objects in an exhibition
   */
  async getObjects(
    id: string,
    params?: { page?: number; pageSize?: number }
  ): Promise<APIResponse<MuseumObject[]>> {
    const response = await this.client.get<APIResponse<MuseumObject[]>>(
      `/exhibitions/${id}/objects`,
      { params }
    );
    return response.data;
  }
}

/**
 * Departments API module
 */
class DepartmentsAPI {
  constructor(private client: AxiosInstance) {}

  /**
   * List departments
   */
  async list(): Promise<APIResponse<Department[]>> {
    const response = await this.client.get<APIResponse<Department[]>>(
      '/departments'
    );
    return response.data;
  }

  /**
   * Get department details
   */
  async get(id: string): Promise<Department> {
    const response = await this.client.get<Department>(`/departments/${id}`);
    return response.data;
  }

  /**
   * Get objects in a department
   */
  async getObjects(
    id: string,
    params?: { page?: number; pageSize?: number }
  ): Promise<APIResponse<MuseumObject[]>> {
    const response = await this.client.get<APIResponse<MuseumObject[]>>(
      `/departments/${id}/objects`,
      { params }
    );
    return response.data;
  }
}

/**
 * Creators API module
 */
class CreatorsAPI {
  constructor(private client: AxiosInstance) {}

  /**
   * List creators/artists
   */
  async list(params?: {
    q?: string;
    nationality?: string;
    birthYear?: number;
    page?: number;
    pageSize?: number;
  }): Promise<APIResponse<any[]>> {
    const response = await this.client.get<APIResponse<any[]>>('/creators', {
      params,
    });
    return response.data;
  }

  /**
   * Get creator details
   */
  async get(id: string): Promise<any> {
    const response = await this.client.get<any>(`/creators/${id}`);
    return response.data;
  }

  /**
   * Get creator's works
   */
  async getWorks(
    id: string,
    params?: { page?: number; pageSize?: number }
  ): Promise<APIResponse<MuseumObject[]>> {
    const response = await this.client.get<APIResponse<MuseumObject[]>>(
      `/creators/${id}/objects`,
      { params }
    );
    return response.data;
  }
}

/**
 * IIIF API module
 */
class IIIFAPI {
  constructor(private client: AxiosInstance) {}

  /**
   * Get IIIF manifest for an object
   */
  async getManifest(id: string): Promise<IIIFManifest> {
    const response = await this.client.get<IIIFManifest>(
      `/iiif/${id}/manifest.json`
    );
    return response.data;
  }

  /**
   * Get IIIF image URL
   */
  getImageUrl(
    id: string,
    options?: {
      region?: string;
      size?: string;
      rotation?: number;
      quality?: 'default' | 'color' | 'gray' | 'bitonal';
      format?: 'jpg' | 'png' | 'webp';
    }
  ): string {
    const {
      region = 'full',
      size = 'max',
      rotation = 0,
      quality = 'default',
      format = 'jpg',
    } = options || {};

    const baseUrl = this.client.defaults.baseURL?.replace('/v1', '');
    return `${baseUrl}/iiif/${id}/${region}/${size}/${rotation}/${quality}.${format}`;
  }
}

/**
 * Education API module
 */
class EducationAPI {
  constructor(private client: AxiosInstance) {}

  /**
   * List educational resources
   */
  async list(params?: {
    type?: 'lesson-plan' | 'activity' | 'video' | 'quiz';
    gradeLevel?: string;
    subject?: string;
    page?: number;
    pageSize?: number;
  }): Promise<APIResponse<any[]>> {
    const response = await this.client.get<APIResponse<any[]>>('/education', {
      params,
    });
    return response.data;
  }

  /**
   * Get educational resource details
   */
  async get(id: string): Promise<any> {
    const response = await this.client.get<any>(`/education/${id}`);
    return response.data;
  }
}

/**
 * Webhooks API module
 */
class WebhooksAPI {
  constructor(private client: AxiosInstance) {}

  /**
   * Register a webhook
   */
  async create(webhook: WebhookRegistration): Promise<{ id: string }> {
    const response = await this.client.post<{ id: string }>(
      '/webhooks',
      webhook
    );
    return response.data;
  }

  /**
   * List webhooks
   */
  async list(): Promise<APIResponse<any[]>> {
    const response = await this.client.get<APIResponse<any[]>>('/webhooks');
    return response.data;
  }

  /**
   * Delete a webhook
   */
  async delete(id: string): Promise<void> {
    await this.client.delete(`/webhooks/${id}`);
  }
}

/**
 * Custom error class for Museum Archive API
 */
export class MuseumArchiveError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number,
    public details?: any
  ) {
    super(message);
    this.name = 'MuseumArchiveError';
  }
}

/**
 * Utility functions
 */
export const utils = {
  /**
   * Format date for API
   */
  formatDate(date: Date): string {
    return date.toISOString().split('T')[0];
  },

  /**
   * Parse date range string
   */
  parseDateRange(range: string): { start?: string; end?: string } {
    const parts = range.split('-');
    return {
      start: parts[0]?.trim(),
      end: parts[1]?.trim(),
    };
  },

  /**
   * Build IIIF image URL manually
   */
  buildIIIFUrl(
    baseUrl: string,
    id: string,
    region = 'full',
    size = 'max',
    rotation = 0,
    quality = 'default',
    format = 'jpg'
  ): string {
    return `${baseUrl}/iiif/${id}/${region}/${size}/${rotation}/${quality}.${format}`;
  },

  /**
   * Validate accession number format
   */
  validateAccessionNumber(accession: string): boolean {
    // Common format: YYYY.NNN.NN
    const pattern = /^\d{4}\.\d+\.\d+$/;
    return pattern.test(accession);
  },
};

// Export all types
export * from './types';

// Default export
export default MuseumArchiveClient;
