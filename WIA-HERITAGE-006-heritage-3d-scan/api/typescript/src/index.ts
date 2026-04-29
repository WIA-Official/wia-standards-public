/**
 * WIA-HERITAGE-006: Heritage Site 3D Scanning Standard
 * TypeScript SDK
 *
 * 弘益人間 (Benefit All Humanity)
 * © 2025 SmileStory Inc. / WIA
 */

import {
  WIAHeritage006Config,
  ArtifactMetadata,
  Model3D,
  ProvenanceRecord,
  VirtualExhibition,
  ScanRequest,
  ScanResult,
  SearchQuery,
  SearchResults,
  APIResponse
} from './types';

/**
 * WIA-HERITAGE-006 SDK Client
 */
export class WIAHeritage006 {
  private config: WIAHeritage006Config;
  private endpoint: string;

  constructor(config: WIAHeritage006Config) {
    this.config = {
      endpoint: 'https://api.wia.org/heritage/v1',
      blockchain: 'ethereum',
      language: 'en',
      timeout: 30000,
      ...config
    };
    this.endpoint = this.config.endpoint!;
  }

  /**
   * Create a new artifact record
   */
  async createArtifact(metadata: ArtifactMetadata): Promise<APIResponse<ArtifactMetadata>> {
    return this.request('POST', '/artifacts', metadata);
  }

  /**
   * Get artifact by ID
   */
  async getArtifact(id: string): Promise<APIResponse<ArtifactMetadata>> {
    return this.request('GET', `/artifacts/${id}`);
  }

  /**
   * Update artifact metadata
   */
  async updateArtifact(id: string, metadata: Partial<ArtifactMetadata>): Promise<APIResponse<ArtifactMetadata>> {
    return this.request('PATCH', `/artifacts/${id}`, metadata);
  }

  /**
   * Delete artifact
   */
  async deleteArtifact(id: string): Promise<APIResponse<void>> {
    return this.request('DELETE', `/artifacts/${id}`);
  }

  /**
   * Search artifacts
   */
  async searchArtifacts(query: SearchQuery): Promise<APIResponse<SearchResults>> {
    return this.request('POST', '/artifacts/search', query);
  }

  /**
   * Scan and create 3D model from images
   */
  async scan(request: ScanRequest): Promise<APIResponse<ScanResult>> {
    const formData = new FormData();

    // Add images
    for (let i = 0; i < request.images.length; i++) {
      const response = await fetch(request.images[i]);
      const blob = await response.blob();
      formData.append('images', blob, `image-${i}.jpg`);
    }

    // Add metadata
    formData.append('metadata', JSON.stringify(request.metadata));

    // Add options
    if (request.options) {
      formData.append('options', JSON.stringify(request.options));
    }

    return this.request('POST', '/scan', formData);
  }

  /**
   * Get scan status
   */
  async getScanStatus(scanId: string): Promise<APIResponse<ScanResult>> {
    return this.request('GET', `/scan/${scanId}`);
  }

  /**
   * Generate 3D model from artifact
   */
  async generate3D(artifact: ArtifactMetadata): Promise<APIResponse<Model3D>> {
    return this.request('POST', '/models/generate', { artifactId: artifact.id });
  }

  /**
   * Get 3D model
   */
  async getModel(modelId: string): Promise<APIResponse<Model3D>> {
    return this.request('GET', `/models/${modelId}`);
  }

  /**
   * Record provenance on blockchain
   */
  async recordProvenance(record: Omit<ProvenanceRecord, 'id' | 'txHash' | 'timestamp'>): Promise<APIResponse<ProvenanceRecord>> {
    return this.request('POST', '/provenance', record);
  }

  /**
   * Get provenance history for artifact
   */
  async getProvenance(artifactId: string): Promise<APIResponse<ProvenanceRecord[]>> {
    return this.request('GET', `/provenance/${artifactId}`);
  }

  /**
   * Verify provenance authenticity
   */
  async verifyProvenance(recordId: string): Promise<APIResponse<{ valid: boolean; details: any }>> {
    return this.request('POST', `/provenance/${recordId}/verify`);
  }

  /**
   * Create virtual exhibition
   */
  async createExhibition(exhibition: Omit<VirtualExhibition, 'id' | 'created' | 'url'>): Promise<APIResponse<VirtualExhibition>> {
    return this.request('POST', '/exhibitions', exhibition);
  }

  /**
   * Get exhibition
   */
  async getExhibition(id: string): Promise<APIResponse<VirtualExhibition>> {
    return this.request('GET', `/exhibitions/${id}`);
  }

  /**
   * List all exhibitions
   */
  async listExhibitions(filters?: any): Promise<APIResponse<VirtualExhibition[]>> {
    return this.request('GET', '/exhibitions', filters);
  }

  /**
   * Update exhibition
   */
  async updateExhibition(id: string, updates: Partial<VirtualExhibition>): Promise<APIResponse<VirtualExhibition>> {
    return this.request('PATCH', `/exhibitions/${id}`, updates);
  }

  /**
   * Delete exhibition
   */
  async deleteExhibition(id: string): Promise<APIResponse<void>> {
    return this.request('DELETE', `/exhibitions/${id}`);
  }

  /**
   * Perform AI restoration on artifact images
   */
  async restoreArtifact(artifactId: string, options?: {
    reconstructMissing?: boolean;
    restoreColors?: boolean;
    enhanceTexture?: boolean;
  }): Promise<APIResponse<{ restoredImages: string[]; model?: Model3D }>> {
    return this.request('POST', `/artifacts/${artifactId}/restore`, options);
  }

  /**
   * Analyze artifact materials
   */
  async analyzeMaterials(artifactId: string, images: string[]): Promise<APIResponse<{
    materials: Array<{
      type: string;
      composition: string;
      percentage: number;
      confidence: number;
    }>;
  }>> {
    return this.request('POST', `/artifacts/${artifactId}/analyze-materials`, { images });
  }

  /**
   * Make HTTP request
   */
  private async request<T>(
    method: string,
    path: string,
    data?: any
  ): Promise<APIResponse<T>> {
    const url = `${this.endpoint}${path}`;

    const options: RequestInit = {
      method,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/json',
        'Accept-Language': this.config.language || 'en'
      }
    };

    if (data && !(data instanceof FormData)) {
      options.body = JSON.stringify(data);
    } else if (data instanceof FormData) {
      delete (options.headers as any)['Content-Type'];
      options.body = data;
    }

    try {
      const response = await fetch(url, options);
      const result = await response.json();

      return {
        success: response.ok,
        data: result.data,
        error: result.error,
        timestamp: new Date()
      };
    } catch (error: any) {
      return {
        success: false,
        error: error.message,
        timestamp: new Date()
      };
    }
  }
}

// Export types
export * from './types';

// Export default
export default WIAHeritage006;
