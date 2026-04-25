/**
 * WIA-ENE-024: Upcycling Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Re-export all types
export * from './types';

import type {
  ClientConfig,
  UpcyclingProject,
  CreateProjectRequest,
  CreateProjectResponse,
  ListProjectsRequest,
  ListProjectsResponse,
  GetProjectResponse,
  UpdateProjectRequest,
  UpdateProjectResponse,
  CalculateImpactRequest,
  CalculateImpactResponse,
  GetCertificateResponse,
  GetStatisticsResponse,
  ErrorResponse,
  MaterialCategory,
  ProjectStatus
} from './types';

// ============================================================================
// Client Implementation
// ============================================================================

/**
 * WIA-ENE-024 Upcycling Client
 *
 * Main client for interacting with the WIA-ENE-024 Upcycling Standard API
 */
export class UpcyclingClient {
  private config: Required<ClientConfig>;

  constructor(config: ClientConfig) {
    this.config = {
      apiKey: config.apiKey || '',
      endpoint: config.endpoint,
      timeout: config.timeout || 30000,
      version: config.version || 'v1',
      locale: config.locale || 'ko'
    };
  }

  // ==========================================================================
  // Project Management
  // ==========================================================================

  /**
   * Create a new upcycling project
   *
   * @example
   * ```typescript
   * const project = await client.createProject({
   *   projectName: '청바지 토트백 제작',
   *   sourceMaterial: {
   *     materialType: MaterialCategory.TEXTILE,
   *     subType: 'denim',
   *     quantity: 3,
   *     weight: 1.5,
   *     weightUnit: 'kg',
   *     condition: MaterialGrade.B,
   *     sourceChannel: 'donation_center',
   *     acquisitionCost: 5000,
   *     currency: 'KRW'
   *   },
   *   outputProduct: {
   *     productType: ProductCategory.BAG,
   *     subType: 'tote_bag',
   *     productName: 'Vintage Denim Tote',
   *     quantity: 2
   *   }
   * });
   * ```
   */
  async createProject(request: CreateProjectRequest): Promise<CreateProjectResponse> {
    return this.request<CreateProjectResponse>('POST', '/upcycling/projects', request);
  }

  /**
   * List upcycling projects with optional filters
   *
   * @example
   * ```typescript
   * const response = await client.listProjects({
   *   status: ProjectStatus.COMPLETED,
   *   materialType: MaterialCategory.TEXTILE,
   *   limit: 10,
   *   offset: 0,
   *   sortBy: 'createdAt',
   *   sortOrder: 'desc'
   * });
   * ```
   */
  async listProjects(request?: ListProjectsRequest): Promise<ListProjectsResponse> {
    const params = this.buildQueryParams(request || {});
    return this.request<ListProjectsResponse>('GET', `/upcycling/projects${params}`);
  }

  /**
   * Get project details by ID
   *
   * @example
   * ```typescript
   * const response = await client.getProject('UP-2025-001234');
   * ```
   */
  async getProject(projectId: string): Promise<GetProjectResponse> {
    return this.request<GetProjectResponse>('GET', `/upcycling/projects/${projectId}`);
  }

  /**
   * Update an existing project
   *
   * @example
   * ```typescript
   * const response = await client.updateProject('UP-2025-001234', {
   *   status: ProjectStatus.COMPLETED,
   *   valueMetrics: {
   *     sellingPrice: 89000,
   *     valueMultiplier: 8.9
   *   }
   * });
   * ```
   */
  async updateProject(
    projectId: string,
    request: UpdateProjectRequest
  ): Promise<UpdateProjectResponse> {
    return this.request<UpdateProjectResponse>(
      'PATCH',
      `/upcycling/projects/${projectId}`,
      request
    );
  }

  /**
   * Delete a project
   *
   * @example
   * ```typescript
   * await client.deleteProject('UP-2025-001234');
   * ```
   */
  async deleteProject(projectId: string): Promise<{ success: boolean }> {
    return this.request<{ success: boolean }>('DELETE', `/upcycling/projects/${projectId}`);
  }

  // ==========================================================================
  // Material Management
  // ==========================================================================

  /**
   * Register source material
   */
  async registerMaterial(material: any): Promise<any> {
    return this.request('POST', '/materials', material);
  }

  /**
   * List available materials
   */
  async listMaterials(filters?: any): Promise<any> {
    const params = this.buildQueryParams(filters || {});
    return this.request('GET', `/materials${params}`);
  }

  /**
   * Get material details
   */
  async getMaterial(materialId: string): Promise<any> {
    return this.request('GET', `/materials/${materialId}`);
  }

  /**
   * Update material status
   */
  async updateMaterialStatus(materialId: string, status: string): Promise<any> {
    return this.request('PATCH', `/materials/${materialId}/status`, { status });
  }

  // ==========================================================================
  // Product Management
  // ==========================================================================

  /**
   * Register output product
   */
  async registerProduct(product: any): Promise<any> {
    return this.request('POST', '/products', product);
  }

  /**
   * List products
   */
  async listProducts(filters?: any): Promise<any> {
    const params = this.buildQueryParams(filters || {});
    return this.request('GET', `/products${params}`);
  }

  /**
   * Get product details
   */
  async getProduct(productId: string): Promise<any> {
    return this.request('GET', `/products/${productId}`);
  }

  /**
   * Get product certificate
   */
  async getProductCertificate(productId: string): Promise<GetCertificateResponse> {
    return this.request<GetCertificateResponse>('GET', `/products/${productId}/certificate`);
  }

  // ==========================================================================
  // Environmental Impact
  // ==========================================================================

  /**
   * Calculate environmental impact
   *
   * @example
   * ```typescript
   * const impact = await client.calculateImpact({
   *   sourceMaterial: {
   *     materialType: MaterialCategory.TEXTILE,
   *     weight: 1.5,
   *     weightUnit: 'kg'
   *   },
   *   processes: [
   *     { energyUsed: 0.5, waterUsed: 50 }
   *   ],
   *   productionMethod: 'manual'
   * });
   * ```
   */
  async calculateImpact(request: CalculateImpactRequest): Promise<CalculateImpactResponse> {
    return this.request<CalculateImpactResponse>('GET', '/impact/calculate', request);
  }

  /**
   * Get impact report for a project
   */
  async getImpactReport(projectId: string): Promise<any> {
    return this.request('GET', `/impact/report/${projectId}`);
  }

  // ==========================================================================
  // Statistics
  // ==========================================================================

  /**
   * Get overall statistics
   *
   * @example
   * ```typescript
   * const stats = await client.getStatistics();
   * console.log('Total projects:', stats.data.totalProjects);
   * console.log('Total CO2 avoided:', stats.data.totalCO2Avoided);
   * ```
   */
  async getStatistics(): Promise<GetStatisticsResponse> {
    return this.request<GetStatisticsResponse>('GET', '/statistics');
  }

  // ==========================================================================
  // Utility Methods
  // ==========================================================================

  /**
   * Build query parameters from object
   */
  private buildQueryParams(params: Record<string, any>): string {
    const query = Object.entries(params)
      .filter(([_, value]) => value !== undefined && value !== null)
      .map(([key, value]) => `${encodeURIComponent(key)}=${encodeURIComponent(String(value))}`)
      .join('&');
    return query ? `?${query}` : '';
  }

  /**
   * Make HTTP request to API
   */
  private async request<T>(
    method: string,
    path: string,
    body?: any
  ): Promise<T> {
    const url = `${this.config.endpoint}/api/${this.config.version}${path}`;

    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
      'Accept': 'application/json',
      'X-WIA-Version': this.config.version,
      'Accept-Language': this.config.locale
    };

    if (this.config.apiKey) {
      headers['Authorization'] = `Bearer ${this.config.apiKey}`;
    }

    const options: RequestInit = {
      method,
      headers,
      signal: AbortSignal.timeout(this.config.timeout)
    };

    if (body && (method === 'POST' || method === 'PATCH' || method === 'PUT')) {
      options.body = JSON.stringify(body);
    }

    try {
      const response = await fetch(url, options);
      const data = await response.json();

      if (!response.ok) {
        throw new UpcyclingAPIError(
          data.error?.message || `HTTP ${response.status}: ${response.statusText}`,
          response.status,
          data.error?.code,
          data.error?.details
        );
      }

      return data as T;
    } catch (error) {
      if (error instanceof UpcyclingAPIError) {
        throw error;
      }
      throw new UpcyclingAPIError(
        error instanceof Error ? error.message : 'Unknown error',
        0
      );
    }
  }
}

// ============================================================================
// Error Classes
// ============================================================================

/**
 * Custom error class for API errors
 */
export class UpcyclingAPIError extends Error {
  constructor(
    message: string,
    public statusCode: number,
    public code?: string,
    public details?: any
  ) {
    super(message);
    this.name = 'UpcyclingAPIError';
    Object.setPrototypeOf(this, UpcyclingAPIError.prototype);
  }

  toJSON(): ErrorResponse {
    return {
      code: this.code || `HTTP_${this.statusCode}`,
      message: this.message,
      details: this.details
    };
  }
}

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Calculate CO2 emissions avoided
 *
 * Based on WIA-ENE-024 standard emission coefficients
 */
export function calculateCO2Avoided(
  materialType: MaterialCategory,
  weight: number,
  productionMethod: 'manual' | 'semi_auto' | 'automated' = 'manual'
): number {
  // Emission coefficients (kg CO2 per kg material)
  const emissionCoefficients: Record<MaterialCategory, number> = {
    [MaterialCategory.TEXTILE]: 8.0,
    [MaterialCategory.PLASTIC]: 6.0,
    [MaterialCategory.METAL]: 2.0,
    [MaterialCategory.WOOD]: 0.5,
    [MaterialCategory.GLASS]: 0.85,
    [MaterialCategory.PAPER]: 1.5,
    [MaterialCategory.E_WASTE]: 5.0,
    [MaterialCategory.CONSTRUCTION]: 0.3,
    [MaterialCategory.RUBBER]: 3.5,
    [MaterialCategory.CERAMIC]: 0.7,
    [MaterialCategory.LEATHER]: 17.0,
    [MaterialCategory.MIXED]: 4.0
  };

  // Upcycling process emissions (kg CO2 per kg material)
  const processEmissions: Record<string, number> = {
    manual: 0.1,
    semi_auto: 0.2,
    automated: 0.3
  };

  const newProductEmissions = emissionCoefficients[materialType] * weight;
  const upcyclingEmissions = processEmissions[productionMethod] * weight;

  return newProductEmissions - upcyclingEmissions;
}

/**
 * Calculate water saved
 *
 * Based on WIA-ENE-024 standard water usage coefficients
 */
export function calculateWaterSaved(
  materialType: MaterialCategory,
  weight: number
): number {
  // Water usage coefficients (liters per kg material)
  const waterCoefficients: Record<MaterialCategory, number> = {
    [MaterialCategory.TEXTILE]: 10000,
    [MaterialCategory.PLASTIC]: 80,
    [MaterialCategory.METAL]: 50,
    [MaterialCategory.WOOD]: 100,
    [MaterialCategory.GLASS]: 30,
    [MaterialCategory.PAPER]: 300,
    [MaterialCategory.E_WASTE]: 100,
    [MaterialCategory.CONSTRUCTION]: 20,
    [MaterialCategory.RUBBER]: 150,
    [MaterialCategory.CERAMIC]: 40,
    [MaterialCategory.LEATHER]: 15000,
    [MaterialCategory.MIXED]: 1000
  };

  const newProductWater = waterCoefficients[materialType] * weight;
  const upcyclingWater = 10 * weight; // Assuming 10L/kg for upcycling cleaning

  return newProductWater - upcyclingWater;
}

/**
 * Calculate value multiplier
 */
export function calculateValueMultiplier(
  sellingPrice: number,
  sourceCost: number
): number {
  if (sourceCost === 0) {
    return 0;
  }
  return sellingPrice / sourceCost;
}

/**
 * Assess value multiplier quality
 */
export function assessValueMultiplier(multiplier: number): string {
  if (multiplier >= 10) return '우수 (Excellent)';
  if (multiplier >= 5) return '양호 (Good)';
  if (multiplier >= 3) return '보통 (Fair)';
  return '저조 (Poor)';
}

/**
 * Calculate profit margin
 */
export function calculateProfitMargin(
  sellingPrice: number,
  totalCost: number
): number {
  if (sellingPrice === 0) {
    return 0;
  }
  return ((sellingPrice - totalCost) / sellingPrice) * 100;
}

/**
 * Generate project ID
 */
export function generateProjectId(): string {
  const date = new Date();
  const year = date.getFullYear();
  const random = Math.floor(Math.random() * 1000000).toString().padStart(6, '0');
  return `UP-${year}-${random}`;
}

/**
 * Format currency (KRW)
 */
export function formatKRW(amount: number): string {
  return new Intl.NumberFormat('ko-KR', {
    style: 'currency',
    currency: 'KRW'
  }).format(amount);
}

/**
 * Format weight with unit
 */
export function formatWeight(weight: number, unit: string): string {
  return `${weight.toLocaleString()} ${unit}`;
}

/**
 * Validate project data
 */
export function validateProject(project: Partial<UpcyclingProject>): {
  valid: boolean;
  errors: string[];
} {
  const errors: string[] = [];

  if (!project.projectName || project.projectName.trim() === '') {
    errors.push('프로젝트 이름은 필수입니다');
  }

  if (!project.sourceMaterial) {
    errors.push('원료 정보는 필수입니다');
  } else {
    if (!project.sourceMaterial.materialType) {
      errors.push('재료 유형은 필수입니다');
    }
    if (project.sourceMaterial.quantity <= 0) {
      errors.push('수량은 0보다 커야 합니다');
    }
    if (project.sourceMaterial.weight <= 0) {
      errors.push('무게는 0보다 커야 합니다');
    }
  }

  if (!project.outputProduct) {
    errors.push('제품 정보는 필수입니다');
  } else {
    if (!project.outputProduct.productName) {
      errors.push('제품명은 필수입니다');
    }
    if (project.outputProduct.quantity <= 0) {
      errors.push('제품 수량은 0보다 커야 합니다');
    }
  }

  return {
    valid: errors.length === 0,
    errors
  };
}

// ============================================================================
// Export Default Client Constructor
// ============================================================================

export default UpcyclingClient;
