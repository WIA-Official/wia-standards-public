/**
 * WIA-AGRI-027 Lab-Grown Food Standard - TypeScript SDK
 * @version 1.0.0
 * @standard WIA-AGRI-027
 * @license MIT
 */

import axios, { AxiosInstance } from 'axios';
import * as Types from './types';

export * from './types';

/**
 * Main SDK client for WIA-AGRI-027 Lab-Grown Food Standard
 */
export class WIALabGrownFoodClient {
  private client: AxiosInstance;
  private config: Types.ClientConfig;

  constructor(config: Types.ClientConfig) {
    this.config = config;
    this.client = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'WIA-AGRI-027',
        'X-WIA-Version': '1.0.0',
        ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }),
      },
    });
  }

  // ========================================================================
  // Product Management
  // ========================================================================

  /**
   * Create a new lab-grown food product
   */
  async createProduct(product: Omit<Types.LabGrownProduct, 'productId' | 'createdAt' | 'updatedAt'>): Promise<Types.LabGrownProduct> {
    const response = await this.client.post('/api/v1/products', product);
    return response.data;
  }

  /**
   * Get product by ID
   */
  async getProduct(productId: string): Promise<Types.LabGrownProduct> {
    const response = await this.client.get(`/api/v1/products/${productId}`);
    return response.data;
  }

  /**
   * Update product information
   */
  async updateProduct(productId: string, updates: Partial<Types.LabGrownProduct>): Promise<Types.LabGrownProduct> {
    const response = await this.client.put(`/api/v1/products/${productId}`, updates);
    return response.data;
  }

  /**
   * Delete product
   */
  async deleteProduct(productId: string): Promise<void> {
    await this.client.delete(`/api/v1/products/${productId}`);
  }

  /**
   * List products with filters
   */
  async listProducts(filters?: {
    productType?: Types.ProductType;
    manufacturerId?: string;
    batchNumber?: string;
  }): Promise<Types.LabGrownProduct[]> {
    const response = await this.client.get('/api/v1/products', { params: filters });
    return response.data;
  }

  // ========================================================================
  // Production Records
  // ========================================================================

  /**
   * Create production record
   */
  async createProductionRecord(record: Omit<Types.ProductionRecord, 'recordId'>): Promise<Types.ProductionRecord> {
    const response = await this.client.post('/api/v1/production', record);
    return response.data;
  }

  /**
   * Get production records for a product
   */
  async getProductionRecords(productId: string): Promise<Types.ProductionRecord[]> {
    const response = await this.client.get(`/api/v1/products/${productId}/production`);
    return response.data;
  }

  /**
   * Get production record by ID
   */
  async getProductionRecord(recordId: string): Promise<Types.ProductionRecord> {
    const response = await this.client.get(`/api/v1/production/${recordId}`);
    return response.data;
  }

  /**
   * Update production record
   */
  async updateProductionRecord(recordId: string, updates: Partial<Types.ProductionRecord>): Promise<Types.ProductionRecord> {
    const response = await this.client.put(`/api/v1/production/${recordId}`, updates);
    return response.data;
  }

  // ========================================================================
  // Quality Management
  // ========================================================================

  /**
   * Get quality metrics for a product
   */
  async getQualityMetrics(productId: string): Promise<Types.QualityMetrics> {
    const response = await this.client.get(`/api/v1/products/${productId}/quality`);
    return response.data;
  }

  /**
   * Update quality metrics
   */
  async updateQualityMetrics(productId: string, metrics: Types.QualityMetrics): Promise<Types.QualityMetrics> {
    const response = await this.client.put(`/api/v1/products/${productId}/quality`, metrics);
    return response.data;
  }

  /**
   * Add quality check record
   */
  async addQualityCheck(productId: string, check: Omit<Types.QualityCheckRecord, 'checkId'>): Promise<Types.QualityCheckRecord> {
    const response = await this.client.post(`/api/v1/products/${productId}/quality-checks`, check);
    return response.data;
  }

  /**
   * Get quality check history
   */
  async getQualityCheckHistory(productId: string): Promise<Types.QualityCheckRecord[]> {
    const response = await this.client.get(`/api/v1/products/${productId}/quality-checks`);
    return response.data;
  }

  // ========================================================================
  // Cell Line Management
  // ========================================================================

  /**
   * Register cell line
   */
  async registerCellLine(cellLine: Types.CellLineInfo): Promise<Types.CellLineInfo> {
    const response = await this.client.post('/api/v1/cell-lines', cellLine);
    return response.data;
  }

  /**
   * Get cell line information
   */
  async getCellLine(cellLineId: string): Promise<Types.CellLineInfo> {
    const response = await this.client.get(`/api/v1/cell-lines/${cellLineId}`);
    return response.data;
  }

  /**
   * Update cell line information
   */
  async updateCellLine(cellLineId: string, updates: Partial<Types.CellLineInfo>): Promise<Types.CellLineInfo> {
    const response = await this.client.put(`/api/v1/cell-lines/${cellLineId}`, updates);
    return response.data;
  }

  /**
   * List cell lines
   */
  async listCellLines(filters?: {
    cellType?: Types.CellType;
    sourceOrganism?: string;
  }): Promise<Types.CellLineInfo[]> {
    const response = await this.client.get('/api/v1/cell-lines', { params: filters });
    return response.data;
  }

  // ========================================================================
  // Traceability
  // ========================================================================

  /**
   * Get traceability record
   */
  async getTraceability(productId: string): Promise<Types.TraceabilityRecord> {
    const response = await this.client.get(`/api/v1/products/${productId}/traceability`);
    return response.data;
  }

  /**
   * Add distribution record
   */
  async addDistributionRecord(
    productId: string,
    record: Omit<Types.DistributionRecord, 'recordId'>
  ): Promise<Types.DistributionRecord> {
    const response = await this.client.post(`/api/v1/products/${productId}/distribution`, record);
    return response.data;
  }

  /**
   * Get distribution history
   */
  async getDistributionHistory(productId: string): Promise<Types.DistributionRecord[]> {
    const response = await this.client.get(`/api/v1/products/${productId}/distribution`);
    return response.data;
  }

  /**
   * Verify blockchain record
   */
  async verifyBlockchain(productId: string): Promise<{
    verified: boolean;
    blockchainInfo: Types.BlockchainInfo;
  }> {
    const response = await this.client.get(`/api/v1/products/${productId}/blockchain/verify`);
    return response.data;
  }

  // ========================================================================
  // Certifications
  // ========================================================================

  /**
   * Add certification
   */
  async addCertification(
    productId: string,
    certification: Omit<Types.Certification, 'certificationId'>
  ): Promise<Types.Certification> {
    const response = await this.client.post(`/api/v1/products/${productId}/certifications`, certification);
    return response.data;
  }

  /**
   * Get certifications
   */
  async getCertifications(productId: string): Promise<Types.Certification[]> {
    const response = await this.client.get(`/api/v1/products/${productId}/certifications`);
    return response.data;
  }

  /**
   * Update certification status
   */
  async updateCertification(
    productId: string,
    certificationId: string,
    updates: Partial<Types.Certification>
  ): Promise<Types.Certification> {
    const response = await this.client.put(
      `/api/v1/products/${productId}/certifications/${certificationId}`,
      updates
    );
    return response.data;
  }

  // ========================================================================
  // Environmental Impact
  // ========================================================================

  /**
   * Get environmental impact assessment
   */
  async getEnvironmentalImpact(productId: string): Promise<Types.EnvironmentalImpact> {
    const response = await this.client.get(`/api/v1/products/${productId}/environmental-impact`);
    return response.data;
  }

  /**
   * Update environmental impact data
   */
  async updateEnvironmentalImpact(
    productId: string,
    impact: Types.EnvironmentalImpact
  ): Promise<Types.EnvironmentalImpact> {
    const response = await this.client.put(`/api/v1/products/${productId}/environmental-impact`, impact);
    return response.data;
  }

  /**
   * Compare with conventional production
   */
  async compareWithConventional(productId: string): Promise<Types.ComparisonMetrics> {
    const response = await this.client.get(`/api/v1/products/${productId}/comparison`);
    return response.data;
  }

  // ========================================================================
  // Safety Testing
  // ========================================================================

  /**
   * Submit microbiological test results
   */
  async submitMicrobiologicalTest(
    productId: string,
    results: Types.MicrobiologicalSafety
  ): Promise<void> {
    await this.client.post(`/api/v1/products/${productId}/tests/microbiological`, results);
  }

  /**
   * Submit chemical safety test results
   */
  async submitChemicalTest(
    productId: string,
    results: Types.ChemicalSafety
  ): Promise<void> {
    await this.client.post(`/api/v1/products/${productId}/tests/chemical`, results);
  }

  /**
   * Get all test results
   */
  async getTestResults(productId: string): Promise<{
    microbiological: Types.MicrobiologicalSafety;
    chemical: Types.ChemicalSafety;
  }> {
    const response = await this.client.get(`/api/v1/products/${productId}/tests`);
    return response.data;
  }

  // ========================================================================
  // Nutritional Information
  // ========================================================================

  /**
   * Update nutritional information
   */
  async updateNutritionalInfo(
    productId: string,
    nutritionalInfo: Types.NutritionalInfo
  ): Promise<Types.NutritionalInfo> {
    const response = await this.client.put(`/api/v1/products/${productId}/nutrition`, nutritionalInfo);
    return response.data;
  }

  /**
   * Get nutritional information
   */
  async getNutritionalInfo(productId: string): Promise<Types.NutritionalInfo> {
    const response = await this.client.get(`/api/v1/products/${productId}/nutrition`);
    return response.data;
  }
}

/**
 * Factory function to create WIA Lab-Grown Food client
 */
export function createClient(config: Types.ClientConfig): WIALabGrownFoodClient {
  return new WIALabGrownFoodClient(config);
}

/**
 * Default export
 */
export default {
  createClient,
  WIALabGrownFoodClient,
};
