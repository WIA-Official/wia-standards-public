/**
 * WIA Aerospace Material Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @packageDocumentation
 * @module wia-aerospace-material
 * @license MIT
 * @author WIA / SmileStory Inc.
 * @version 1.0.0
 */

import EventEmitter from 'eventemitter3';
import {
  WIAConfig,
  APIResponse,
  Material,
  MaterialTest,
  Certification,
  Specification,
  Supplier,
  BatchRecord,
  PaginatedResponse,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface AerospaceMaterialSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class WIAAerospaceMaterialClient {
  private config: Required<AerospaceMaterialSDKConfig>;
  private headers: Record<string, string>;
  private eventEmitter = new EventEmitter();

  constructor(config: AerospaceMaterialSDKConfig) {
    this.config = {
      timeout: 60000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'AEROSPACE-MATERIAL',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Material APIs
  // ==========================================================================

  async listMaterials(): Promise<APIResponse<Material[]>> {
    return this.get<Material[]>('/api/v1/materials');
  }

  async getMaterial(id: string): Promise<APIResponse<Material>> {
    return this.get<Material>(`/api/v1/materials/${id}`);
  }

  async createMaterial(material: Omit<Material, 'materialId'>): Promise<APIResponse<Material>> {
    return this.post<Material>('/api/v1/materials', material);
  }

  async updateMaterial(id: string, updates: Partial<Material>): Promise<APIResponse<Material>> {
    return this.put<Material>(`/api/v1/materials/${id}`, updates);
  }

  // ==========================================================================
  // Testing APIs
  // ==========================================================================

  async submitTest(test: Omit<MaterialTest, 'testId'>): Promise<APIResponse<MaterialTest>> {
    return this.post<MaterialTest>('/api/v1/tests', test);
  }

  async getTest(testId: string): Promise<APIResponse<MaterialTest>> {
    return this.get<MaterialTest>(`/api/v1/tests/${testId}`);
  }

  async getMaterialTests(materialId: string): Promise<APIResponse<MaterialTest[]>> {
    return this.get<MaterialTest[]>(`/api/v1/materials/${materialId}/tests`);
  }

  // ==========================================================================
  // Specification APIs
  // ==========================================================================

  async getSpecification(specId: string): Promise<APIResponse<Specification>> {
    return this.get<Specification>(`/api/v1/specifications/${specId}`);
  }

  async listSpecifications(): Promise<APIResponse<Specification[]>> {
    return this.get<Specification[]>('/api/v1/specifications');
  }

  // ==========================================================================
  // Certification APIs
  // ==========================================================================

  async getCertification(certId: string): Promise<APIResponse<Certification>> {
    return this.get<Certification>(`/api/v1/certifications/${certId}`);
  }

  async getMaterialCertifications(materialId: string): Promise<APIResponse<Certification[]>> {
    return this.get<Certification[]>(`/api/v1/materials/${materialId}/certifications`);
  }

  // ==========================================================================
  // Supplier APIs
  // ==========================================================================

  async listSuppliers(): Promise<APIResponse<Supplier[]>> {
    return this.get<Supplier[]>('/api/v1/suppliers');
  }

  async getSupplier(supplierId: string): Promise<APIResponse<Supplier>> {
    return this.get<Supplier>(`/api/v1/suppliers/${supplierId}`);
  }

  // ==========================================================================
  // Batch APIs
  // ==========================================================================

  async getBatch(batchId: string): Promise<APIResponse<BatchRecord>> {
    return this.get<BatchRecord>(`/api/v1/batches/${batchId}`);
  }

  async getMaterialBatches(materialId: string): Promise<APIResponse<BatchRecord[]>> {
    return this.get<BatchRecord[]>(`/api/v1/materials/${materialId}/batches`);
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: string, callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  // ==========================================================================
  // HTTP Methods
  // ==========================================================================

  private async get<T>(path: string): Promise<APIResponse<T>> {
    return this.request<T>('GET', path);
  }

  private async post<T>(path: string, data: unknown): Promise<APIResponse<T>> {
    return this.request<T>('POST', path, data);
  }

  private async put<T>(path: string, data: unknown): Promise<APIResponse<T>> {
    return this.request<T>('PUT', path, data);
  }

  private async request<T>(method: string, path: string, data?: unknown): Promise<APIResponse<T>> {
    const url = `${this.config.endpoint}${path}`;
    if (this.config.debug) {
      console.log(`[WIA Aerospace Material] ${method} ${url}`);
    }
    const response = await fetch(url, {
      method,
      headers: this.headers,
      body: data ? JSON.stringify(data) : undefined,
    });
    return response.json();
  }
}

// ============================================================================
// Material Calculator
// ============================================================================

export class MaterialCalculator {
  static calculateStrengthToWeightRatio(strength: number, density: number): number {
    return strength / density;
  }

  static calculateSpecificStrength(strength: number, density: number): number {
    return (strength / density) * 1000;
  }

  static calculateFatigueLife(stressAmplitude: number, fatigueCoeff: number, exponent: number): number {
    return Math.pow(stressAmplitude / fatigueCoeff, -1 / exponent);
  }
}

// ============================================================================
// Constants
// ============================================================================

export const VERSION = '1.0.0';

export const MATERIAL_CLASSES = {
  ALUMINUM: 'aluminum',
  TITANIUM: 'titanium',
  STEEL: 'steel',
  COMPOSITE: 'composite',
  CERAMIC: 'ceramic',
} as const;

export default WIAAerospaceMaterialClient;
