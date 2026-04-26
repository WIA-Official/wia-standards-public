/**
 * WIA Food Safety Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-food-safety
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 */

import {
  FoodProduct,
  Hazard,
  TestResult,
  Inspection,
  Certification,
  IncidentReport,
  ApiResponse,
  FoodSafetyQuery,
  Allergen,
  RecallNotice,
  PaginatedResponse,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface FoodSafetyConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class FoodSafetyClient {
  private config: Required<FoodSafetyConfig>;
  private headers: Record<string, string>;

  constructor(config: FoodSafetyConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'FOOD-SAFETY',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Product APIs
  // ==========================================================================

  async getProduct(productId: string): Promise<ApiResponse<FoodProduct>> {
    return this.get<FoodProduct>(`/api/v1/products/${productId}`);
  }

  async createProduct(product: Omit<FoodProduct, 'productId'>): Promise<ApiResponse<FoodProduct>> {
    return this.post<FoodProduct>('/api/v1/products', product);
  }

  async updateProduct(productId: string, updates: Partial<FoodProduct>): Promise<ApiResponse<FoodProduct>> {
    return this.put<FoodProduct>(`/api/v1/products/${productId}`, updates);
  }

  async listProducts(query?: FoodSafetyQuery): Promise<ApiResponse<FoodProduct[]>> {
    const queryString = this.buildQueryParams(query);
    return this.get<FoodProduct[]>(`/api/v1/products${queryString}`);
  }

  // ==========================================================================
  // Hazard APIs
  // ==========================================================================

  async getHazard(hazardId: string): Promise<ApiResponse<Hazard>> {
    return this.get<Hazard>(`/api/v1/hazards/${hazardId}`);
  }

  async reportHazard(hazard: Omit<Hazard, 'hazardId'>): Promise<ApiResponse<Hazard>> {
    return this.post<Hazard>('/api/v1/hazards', hazard);
  }

  // ==========================================================================
  // Test Result APIs
  // ==========================================================================

  async submitTestResult(result: Omit<TestResult, 'resultId'>): Promise<ApiResponse<TestResult>> {
    return this.post<TestResult>('/api/v1/test-results', result);
  }

  async getTestResults(productId: string): Promise<ApiResponse<TestResult[]>> {
    return this.get<TestResult[]>(`/api/v1/products/${productId}/test-results`);
  }

  // ==========================================================================
  // Inspection APIs
  // ==========================================================================

  async submitInspection(inspection: Omit<Inspection, 'inspectionId'>): Promise<ApiResponse<Inspection>> {
    return this.post<Inspection>('/api/v1/inspections', inspection);
  }

  async getInspection(inspectionId: string): Promise<ApiResponse<Inspection>> {
    return this.get<Inspection>(`/api/v1/inspections/${inspectionId}`);
  }

  // ==========================================================================
  // Certification APIs
  // ==========================================================================

  async getCertification(certificateId: string): Promise<ApiResponse<Certification>> {
    return this.get<Certification>(`/api/v1/certifications/${certificateId}`);
  }

  async verifyCertification(certificateId: string): Promise<ApiResponse<{ valid: boolean }>> {
    return this.post<{ valid: boolean }>(`/api/v1/certifications/${certificateId}/verify`, {});
  }

  // ==========================================================================
  // Incident & Recall APIs
  // ==========================================================================

  async reportIncident(incident: Omit<IncidentReport, 'incidentId'>): Promise<ApiResponse<IncidentReport>> {
    return this.post<IncidentReport>('/api/v1/incidents', incident);
  }

  async getIncidents(query?: FoodSafetyQuery): Promise<ApiResponse<IncidentReport[]>> {
    const queryString = this.buildQueryParams(query);
    return this.get<IncidentReport[]>(`/api/v1/incidents${queryString}`);
  }

  async initiateRecall(recall: Omit<RecallNotice, 'recallId'>): Promise<ApiResponse<RecallNotice>> {
    return this.post<RecallNotice>('/api/v1/recalls', recall);
  }

  async getActiveRecalls(): Promise<ApiResponse<RecallNotice[]>> {
    return this.get<RecallNotice[]>('/api/v1/recalls/active');
  }

  // ==========================================================================
  // HTTP Methods
  // ==========================================================================

  private async get<T>(path: string): Promise<ApiResponse<T>> {
    return this.request<T>('GET', path);
  }

  private async post<T>(path: string, data: unknown): Promise<ApiResponse<T>> {
    return this.request<T>('POST', path, data);
  }

  private async put<T>(path: string, data: unknown): Promise<ApiResponse<T>> {
    return this.request<T>('PUT', path, data);
  }

  private async request<T>(method: string, path: string, data?: unknown): Promise<ApiResponse<T>> {
    const url = `${this.config.endpoint}${path}`;
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

    try {
      const response = await fetch(url, {
        method,
        headers: this.headers,
        body: data ? JSON.stringify(data) : undefined,
        signal: controller.signal,
      });
      clearTimeout(timeoutId);
      return response.json();
    } catch (error) {
      clearTimeout(timeoutId);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error',
        timestamp: new Date().toISOString(),
      };
    }
  }

  private buildQueryParams(params?: Record<string, unknown>): string {
    if (!params) return '';
    const searchParams = new URLSearchParams();
    Object.entries(params).forEach(([key, value]) => {
      if (value !== undefined) searchParams.append(key, String(value));
    });
    return searchParams.toString() ? `?${searchParams.toString()}` : '';
  }
}

// ============================================================================
// Constants
// ============================================================================

export const VERSION = '1.0.0';

export const MAJOR_ALLERGENS = [
  'milk', 'eggs', 'fish', 'shellfish', 'tree_nuts',
  'peanuts', 'wheat', 'soybeans', 'sesame',
] as const;

export const HAZARD_CATEGORIES = {
  BIOLOGICAL: 'biological',
  CHEMICAL: 'chemical',
  PHYSICAL: 'physical',
  ALLERGEN: 'allergen',
} as const;

export default FoodSafetyClient;
