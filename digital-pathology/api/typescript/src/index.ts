/**
 * WIA-MED-008: Digital Pathology Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

export * from './types';

import {
  WholeSlideImage,
  WSIMetadata,
  SlideID,
  TileRequest,
  AnalysisRequest,
  AnalysisResult,
  UploadRequest,
  UploadResponse,
  APIResponse,
  PaginatedResponse,
  ValidationResult,
  SlideStatus,
  AnalysisType,
  StainType,
  Magnification
} from './types';

// ============================================================================
// Configuration
// ============================================================================

export interface WIADigitalPathologyConfig {
  /** API base URL */
  baseUrl: string;

  /** API key for authentication */
  apiKey: string;

  /** Request timeout in milliseconds */
  timeout?: number;

  /** Enable debug logging */
  debug?: boolean;
}

// ============================================================================
// WIA Digital Pathology SDK
// ============================================================================

export class WIADigitalPathology {
  private config: WIADigitalPathologyConfig;

  constructor(config: WIADigitalPathologyConfig) {
    this.config = {
      timeout: 30000,
      debug: false,
      ...config
    };
  }

  // ==========================================================================
  // Slide Management
  // ==========================================================================

  /**
   * Get all slides with optional filters
   */
  async getSlides(params?: {
    page?: number;
    page_size?: number;
    status?: SlideStatus;
    organ?: string;
    stain_type?: StainType;
  }): Promise<APIResponse<PaginatedResponse<WholeSlideImage>>> {
    const queryParams = new URLSearchParams();
    if (params?.page) queryParams.set('page', params.page.toString());
    if (params?.page_size) queryParams.set('page_size', params.page_size.toString());
    if (params?.status) queryParams.set('status', params.status);
    if (params?.organ) queryParams.set('organ', params.organ);
    if (params?.stain_type) queryParams.set('stain_type', params.stain_type);

    return this.request(`/api/v1/slides?${queryParams.toString()}`, {
      method: 'GET'
    });
  }

  /**
   * Get a specific slide by ID
   */
  async getSlide(slideId: SlideID): Promise<APIResponse<WholeSlideImage>> {
    return this.request(`/api/v1/slides/${slideId}`, {
      method: 'GET'
    });
  }

  /**
   * Upload a new slide
   */
  async uploadSlide(request: UploadRequest): Promise<APIResponse<UploadResponse>> {
    return this.request('/api/v1/slides/upload', {
      method: 'POST',
      body: JSON.stringify(request)
    });
  }

  /**
   * Update slide metadata
   */
  async updateSlideMetadata(
    slideId: SlideID,
    metadata: Partial<WSIMetadata>
  ): Promise<APIResponse<WholeSlideImage>> {
    return this.request(`/api/v1/slides/${slideId}/metadata`, {
      method: 'PATCH',
      body: JSON.stringify(metadata)
    });
  }

  /**
   * Delete a slide
   */
  async deleteSlide(slideId: SlideID): Promise<APIResponse<void>> {
    return this.request(`/api/v1/slides/${slideId}`, {
      method: 'DELETE'
    });
  }

  // ==========================================================================
  // Tile Access
  // ==========================================================================

  /**
   * Get a specific tile from a slide
   */
  async getTile(request: TileRequest): Promise<Blob> {
    const { slide_id, level, x, y, width = 256, height = 256 } = request;
    const url = `/api/v1/tiles/${slide_id}/${level}/${x}/${y}?width=${width}&height=${height}`;

    const response = await this.rawRequest(url, { method: 'GET' });
    return response.blob();
  }

  /**
   * Get slide thumbnail
   */
  async getThumbnail(
    slideId: SlideID,
    maxWidth: number = 1024,
    maxHeight: number = 1024
  ): Promise<Blob> {
    const url = `/api/v1/slides/${slideId}/thumbnail?max_width=${maxWidth}&max_height=${maxHeight}`;
    const response = await this.rawRequest(url, { method: 'GET' });
    return response.blob();
  }

  // ==========================================================================
  // AI Analysis
  // ==========================================================================

  /**
   * Request AI analysis on a slide
   */
  async requestAnalysis(request: AnalysisRequest): Promise<APIResponse<AnalysisResult>> {
    return this.request('/api/v1/analysis', {
      method: 'POST',
      body: JSON.stringify(request)
    });
  }

  /**
   * Get analysis result by ID
   */
  async getAnalysis(analysisId: string): Promise<APIResponse<AnalysisResult>> {
    return this.request(`/api/v1/analysis/${analysisId}`, {
      method: 'GET'
    });
  }

  /**
   * Get all analyses for a slide
   */
  async getSlideAnalyses(slideId: SlideID): Promise<APIResponse<AnalysisResult[]>> {
    return this.request(`/api/v1/slides/${slideId}/analyses`, {
      method: 'GET'
    });
  }

  // ==========================================================================
  // Validation
  // ==========================================================================

  /**
   * Validate a slide for WIA-MED-008 compliance
   */
  async validateSlide(slideId: SlideID): Promise<APIResponse<ValidationResult>> {
    return this.request(`/api/v1/slides/${slideId}/validate`, {
      method: 'POST'
    });
  }

  /**
   * Validate slide metadata
   */
  validateMetadata(metadata: WSIMetadata): ValidationResult {
    const tests: ValidationResult['tests'] = [];

    // Check required fields
    if (!metadata.wia_version) {
      tests.push({
        name: 'WIA Version',
        result: 'FAIL',
        message: 'Missing wia_version field'
      });
    } else {
      tests.push({
        name: 'WIA Version',
        result: 'PASS',
        score: 100
      });
    }

    // Check specimen info
    if (!metadata.specimen?.accession_number) {
      tests.push({
        name: 'Accession Number',
        result: 'FAIL',
        message: 'Missing accession_number'
      });
    } else {
      tests.push({
        name: 'Accession Number',
        result: 'PASS',
        score: 100
      });
    }

    // Check scan info
    if (!metadata.scan?.magnification) {
      tests.push({
        name: 'Magnification',
        result: 'FAIL',
        message: 'Missing magnification'
      });
    } else if (metadata.scan.magnification < 20) {
      tests.push({
        name: 'Magnification',
        result: 'WARNING',
        score: 75,
        message: 'Magnification below recommended 20x'
      });
    } else {
      tests.push({
        name: 'Magnification',
        result: 'PASS',
        score: 100
      });
    }

    // Check quality metrics
    if (metadata.quality?.focus_quality_score !== undefined) {
      if (metadata.quality.focus_quality_score >= 0.9) {
        tests.push({
          name: 'Focus Quality',
          result: 'PASS',
          score: metadata.quality.focus_quality_score * 100
        });
      } else {
        tests.push({
          name: 'Focus Quality',
          result: 'FAIL',
          score: metadata.quality.focus_quality_score * 100,
          message: 'Focus quality below 90%'
        });
      }
    }

    const failCount = tests.filter(t => t.result === 'FAIL').length;
    const warningCount = tests.filter(t => t.result === 'WARNING').length;

    return {
      is_valid: failCount === 0,
      tests,
      compliance: failCount === 0 ? (warningCount === 0 ? 'COMPLIANT' : 'PARTIAL') : 'NON_COMPLIANT'
    };
  }

  // ==========================================================================
  // Utilities
  // ==========================================================================

  /**
   * Calculate recommended pyramid levels
   */
  calculatePyramidLevels(width: number, height: number, tileSize: number = 256): number {
    const maxDimension = Math.max(width, height);
    return Math.ceil(Math.log2(maxDimension / tileSize)) + 1;
  }

  /**
   * Calculate file size estimate
   */
  estimateFileSize(
    width: number,
    height: number,
    magnification: Magnification,
    compressionRatio: number = 15
  ): number {
    const bpp = 24; // 24-bit RGB
    const uncompressedBytes = (width * height * bpp) / 8;
    return Math.ceil(uncompressedBytes / compressionRatio);
  }

  /**
   * Get resolution in micrometers per pixel for magnification
   */
  getResolution(magnification: Magnification): number {
    const resolutionMap: Record<number, number> = {
      [Magnification.MAG_40X]: 0.25,
      [Magnification.MAG_20X]: 0.5,
      [Magnification.MAG_10X]: 1.0,
      [Magnification.MAG_5X]: 2.0,
      [Magnification.MAG_2_5X]: 4.0,
      [Magnification.MAG_1_25X]: 8.0
    };
    return resolutionMap[magnification] || 0.25;
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async request(url: string, options: RequestInit = {}): Promise<APIResponse> {
    const response = await this.rawRequest(url, options);
    return response.json();
  }

  private async rawRequest(url: string, options: RequestInit = {}): Promise<Response> {
    const fullUrl = `${this.config.baseUrl}${url}`;

    const headers: HeadersInit = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'MED-008',
      'X-WIA-Version': '1.0.0',
      ...options.headers
    };

    if (this.config.debug) {
      console.log(`[WIA-MED-008] ${options.method || 'GET'} ${fullUrl}`);
    }

    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

    try {
      const response = await fetch(fullUrl, {
        ...options,
        headers,
        signal: controller.signal
      });

      clearTimeout(timeoutId);

      if (!response.ok && this.config.debug) {
        console.error(`[WIA-MED-008] Error: ${response.status} ${response.statusText}`);
      }

      return response;
    } catch (error) {
      clearTimeout(timeoutId);
      if (this.config.debug) {
        console.error('[WIA-MED-008] Request failed:', error);
      }
      throw error;
    }
  }
}

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Create a new WIA Digital Pathology SDK instance
 */
export function createClient(config: WIADigitalPathologyConfig): WIADigitalPathology {
  return new WIADigitalPathology(config);
}

/**
 * Generate a slide ID
 */
export function generateSlideId(prefix: string = 'WSI'): SlideID {
  const year = new Date().getFullYear();
  const random = Math.floor(Math.random() * 1000000).toString().padStart(6, '0');
  return `${prefix}-${year}-${random}`;
}

/**
 * Parse slide ID
 */
export function parseSlideId(slideId: SlideID): { prefix: string; year: number; sequence: string } | null {
  const match = slideId.match(/^([A-Z]+)-(\d{4})-(\d{6})$/);
  if (!match) return null;

  return {
    prefix: match[1],
    year: parseInt(match[2]),
    sequence: match[3]
  };
}

/**
 * Format file size
 */
export function formatFileSize(bytes: number): string {
  const units = ['B', 'KB', 'MB', 'GB', 'TB'];
  let size = bytes;
  let unitIndex = 0;

  while (size >= 1024 && unitIndex < units.length - 1) {
    size /= 1024;
    unitIndex++;
  }

  return `${size.toFixed(2)} ${units[unitIndex]}`;
}

/**
 * Format resolution
 */
export function formatResolution(umPerPixel: number): string {
  return `${umPerPixel} µm/pixel`;
}

// ============================================================================
// Default Export
// ============================================================================

export default WIADigitalPathology;
