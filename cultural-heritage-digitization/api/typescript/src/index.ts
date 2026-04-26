/**
 * WIA-EDU-023 Cultural Heritage Digitization Standard - TypeScript SDK
 * Version: 1.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

import type {
  ArtifactMetadata,
  Model3D,
  VirtualTour,
  HistoricalReconstruction,
  ClientConfig,
  APIResponse,
  SearchParams,
  DublinCoreMetadata,
} from './types';

export * from './types';

/**
 * WIA Cultural Heritage Digitization Client
 */
export class CulturalHeritageClient {
  private config: ClientConfig;

  constructor(config: ClientConfig) {
    this.config = {
      timeout: 30000,
      enableCaching: true,
      ...config,
    };
  }

  /**
   * Get artifact by ID
   */
  async getArtifact(id: string): Promise<APIResponse<ArtifactMetadata>> {
    return this.request<ArtifactMetadata>(`/artifacts/${id}`);
  }

  /**
   * List artifacts with optional filters
   */
  async listArtifacts(params?: SearchParams): Promise<APIResponse<ArtifactMetadata[]>> {
    const queryString = params ? this.buildQueryString(params) : '';
    return this.request<ArtifactMetadata[]>(`/artifacts${queryString}`);
  }

  /**
   * Create new artifact
   */
  async createArtifact(artifact: Partial<ArtifactMetadata>): Promise<APIResponse<ArtifactMetadata>> {
    return this.request<ArtifactMetadata>('/artifacts', {
      method: 'POST',
      body: JSON.stringify(artifact),
    });
  }

  /**
   * Update existing artifact
   */
  async updateArtifact(id: string, updates: Partial<ArtifactMetadata>): Promise<APIResponse<ArtifactMetadata>> {
    return this.request<ArtifactMetadata>(`/artifacts/${id}`, {
      method: 'PUT',
      body: JSON.stringify(updates),
    });
  }

  /**
   * Delete artifact
   */
  async deleteArtifact(id: string): Promise<APIResponse<void>> {
    return this.request<void>(`/artifacts/${id}`, {
      method: 'DELETE',
    });
  }

  /**
   * Get 3D models for artifact
   */
  async getModels(artifactId: string): Promise<APIResponse<Model3D[]>> {
    return this.request<Model3D[]>(`/artifacts/${artifactId}/models`);
  }

  /**
   * Upload 3D model
   */
  async uploadModel(artifactId: string, file: File, metadata: Partial<Model3D>): Promise<APIResponse<Model3D>> {
    const formData = new FormData();
    formData.append('file', file);
    formData.append('metadata', JSON.stringify(metadata));

    return this.request<Model3D>(`/artifacts/${artifactId}/models`, {
      method: 'POST',
      body: formData,
      headers: {}, // Let browser set Content-Type for FormData
    });
  }

  /**
   * Get Dublin Core metadata
   */
  async getDublinCore(artifactId: string): Promise<APIResponse<DublinCoreMetadata>> {
    return this.request<DublinCoreMetadata>(`/artifacts/${artifactId}/metadata/dublincore`);
  }

  /**
   * Update Dublin Core metadata
   */
  async updateDublinCore(
    artifactId: string,
    metadata: DublinCoreMetadata
  ): Promise<APIResponse<DublinCoreMetadata>> {
    return this.request<DublinCoreMetadata>(`/artifacts/${artifactId}/metadata/dublincore`, {
      method: 'PUT',
      body: JSON.stringify(metadata),
    });
  }

  /**
   * Search artifacts
   */
  async searchArtifacts(params: SearchParams): Promise<APIResponse<ArtifactMetadata[]>> {
    const queryString = this.buildQueryString(params);
    return this.request<ArtifactMetadata[]>(`/search${queryString}`);
  }

  /**
   * Get virtual tour by ID
   */
  async getVirtualTour(id: string): Promise<APIResponse<VirtualTour>> {
    return this.request<VirtualTour>(`/tours/${id}`);
  }

  /**
   * List virtual tours
   */
  async listVirtualTours(): Promise<APIResponse<VirtualTour[]>> {
    return this.request<VirtualTour[]>('/tours');
  }

  /**
   * Create virtual tour
   */
  async createVirtualTour(tour: Partial<VirtualTour>): Promise<APIResponse<VirtualTour>> {
    return this.request<VirtualTour>('/tours', {
      method: 'POST',
      body: JSON.stringify(tour),
    });
  }

  /**
   * Get historical reconstruction
   */
  async getReconstruction(id: string): Promise<APIResponse<HistoricalReconstruction>> {
    return this.request<HistoricalReconstruction>(`/reconstructions/${id}`);
  }

  /**
   * Create historical reconstruction
   */
  async createReconstruction(
    reconstruction: Partial<HistoricalReconstruction>
  ): Promise<APIResponse<HistoricalReconstruction>> {
    return this.request<HistoricalReconstruction>('/reconstructions', {
      method: 'POST',
      body: JSON.stringify(reconstruction),
    });
  }

  /**
   * Get analytics for artifact
   */
  async getAnalytics(artifactId: string): Promise<APIResponse<any>> {
    return this.request<any>(`/artifacts/${artifactId}/analytics`);
  }

  /**
   * Generic request method
   */
  private async request<T>(endpoint: string, options: RequestInit = {}): Promise<APIResponse<T>> {
    const url = `${this.config.baseURL}${endpoint}`;
    const headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.accessToken}`,
      ...options.headers,
    };

    try {
      const response = await fetch(url, {
        ...options,
        headers,
      });

      const data = await response.json();

      if (!response.ok) {
        return {
          data: data as T,
          status: response.status,
          message: data.message || 'Request failed',
          errors: data.errors,
        };
      }

      return {
        data: data as T,
        status: response.status,
        message: 'Success',
      };
    } catch (error) {
      throw new Error(`API request failed: ${error}`);
    }
  }

  /**
   * Build query string from parameters
   */
  private buildQueryString(params: Record<string, any>): string {
    const entries = Object.entries(params).filter(([_, value]) => value !== undefined && value !== null);
    if (entries.length === 0) return '';

    const queryParams = entries.map(([key, value]) => `${encodeURIComponent(key)}=${encodeURIComponent(String(value))}`);
    return `?${queryParams.join('&')}`;
  }
}

/**
 * Helper function to create a client instance
 */
export function createClient(config: ClientConfig): CulturalHeritageClient {
  return new CulturalHeritageClient(config);
}

/**
 * Default export
 */
export default CulturalHeritageClient;
