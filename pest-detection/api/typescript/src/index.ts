/**
 * WIA Pest Detection Standard - TypeScript SDK
 * @version 1.0.0
 * @license MIT
 */

import {
  PestDetection,
  DiseaseSymptomCatalog,
  PestPopulation,
  ImageRecognition,
  TreatmentRecommendation,
  ApiResponse,
  PestDetectionQuery,
} from './types';

/**
 * Configuration options for the Pest Detection API client
 */
export interface PestDetectionConfig {
  baseUrl: string;
  apiKey?: string;
  timeout?: number;
}

/**
 * Pest Detection API Client
 *
 * @example
 * ```typescript
 * const client = new PestDetectionClient({
 *   baseUrl: 'https://api.example.com/pest-detection',
 *   apiKey: 'your-api-key'
 * });
 *
 * const detections = await client.listDetections({ severity: 'high' });
 * ```
 */
export class PestDetectionClient {
  private config: PestDetectionConfig;
  private headers: Record<string, string>;

  constructor(config: PestDetectionConfig) {
    this.config = { timeout: 30000, ...config };
    this.headers = {
      'Content-Type': 'application/json',
      'Accept': 'application/json',
    };
    if (config.apiKey) {
      this.headers['Authorization'] = `Bearer ${config.apiKey}`;
    }
  }

  private async request<T>(endpoint: string, options: RequestInit = {}): Promise<ApiResponse<T>> {
    const url = `${this.config.baseUrl}${endpoint}`;
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

    try {
      const response = await fetch(url, {
        ...options,
        headers: { ...this.headers, ...options.headers },
        signal: controller.signal,
      });
      clearTimeout(timeoutId);

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();
      return { success: true, data, timestamp: new Date().toISOString() };
    } catch (error) {
      clearTimeout(timeoutId);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error',
        timestamp: new Date().toISOString(),
      };
    }
  }

  // ==================== Pest Detections ====================

  async getDetection(detectionId: string): Promise<ApiResponse<PestDetection>> {
    return this.request<PestDetection>(`/detections/${detectionId}`);
  }

  async createDetection(detection: PestDetection): Promise<ApiResponse<PestDetection>> {
    return this.request<PestDetection>('/detections', {
      method: 'POST',
      body: JSON.stringify(detection),
    });
  }

  async updateDetection(
    detectionId: string,
    detection: Partial<PestDetection>
  ): Promise<ApiResponse<PestDetection>> {
    return this.request<PestDetection>(`/detections/${detectionId}`, {
      method: 'PUT',
      body: JSON.stringify(detection),
    });
  }

  async deleteDetection(detectionId: string): Promise<ApiResponse<void>> {
    return this.request<void>(`/detections/${detectionId}`, { method: 'DELETE' });
  }

  async listDetections(query?: PestDetectionQuery): Promise<ApiResponse<PestDetection[]>> {
    const params = new URLSearchParams();
    if (query) {
      Object.entries(query).forEach(([key, value]) => {
        if (value !== undefined) params.append(key, String(value));
      });
    }
    return this.request<PestDetection[]>(`/detections${params.toString() ? `?${params}` : ''}`);
  }

  // ==================== Disease Catalog ====================

  async getDisease(diseaseId: string): Promise<ApiResponse<DiseaseSymptomCatalog>> {
    return this.request<DiseaseSymptomCatalog>(`/diseases/${diseaseId}`);
  }

  async listDiseases(): Promise<ApiResponse<DiseaseSymptomCatalog[]>> {
    return this.request<DiseaseSymptomCatalog[]>('/diseases');
  }

  // ==================== Population Monitoring ====================

  async getPopulation(populationId: string): Promise<ApiResponse<PestPopulation>> {
    return this.request<PestPopulation>(`/populations/${populationId}`);
  }

  async recordPopulation(population: PestPopulation): Promise<ApiResponse<PestPopulation>> {
    return this.request<PestPopulation>('/populations', {
      method: 'POST',
      body: JSON.stringify(population),
    });
  }

  // ==================== Image Recognition ====================

  async analyzeImage(image: ImageRecognition): Promise<ApiResponse<ImageRecognition>> {
    return this.request<ImageRecognition>('/images/analyze', {
      method: 'POST',
      body: JSON.stringify(image),
    });
  }

  async getImageAnalysis(imageId: string): Promise<ApiResponse<ImageRecognition>> {
    return this.request<ImageRecognition>(`/images/${imageId}`);
  }

  // ==================== Treatment Recommendations ====================

  async getTreatmentRecommendations(
    pestId: string
  ): Promise<ApiResponse<TreatmentRecommendation[]>> {
    return this.request<TreatmentRecommendation[]>(`/treatments/recommend/${pestId}`);
  }
}

export * from './types';
export default PestDetectionClient;
