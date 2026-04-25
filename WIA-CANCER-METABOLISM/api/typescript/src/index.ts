/**
 * WIA-CANCER-METABOLISM Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 * © 2025 WIA - World Certification Industry Association
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import { z } from 'zod';
import {
  CancerMetabolismProfile,
  SDKConfig,
  DEFAULT_CONFIG,
  APIResponse,
  PaginatedResponse,
  SearchQuery,
  ValidationResult,
  ProtocolMessage,
  Biomarker,
  MetaboliteProfile,
  PathwayAnalysis,
  WIA_CANCER_METABOLISM_VERSION,
  WIA_SCHEMA_CONTEXT,
  WIA_PHILOSOPHY,
  CancerType,
  MetabolicPathway,
  WarburgStatus
} from './types';

// ============================================================================
// Validation Schemas
// ============================================================================

const BiomarkerSchema = z.object({
  id: z.string(),
  name: z.string(),
  symbol: z.string(),
  type: z.enum(['gene_expression', 'protein', 'metabolite', 'enzyme_activity', 'mutation']),
  value: z.number(),
  unit: z.string(),
  expression: z.enum(['very_low', 'low', 'normal', 'elevated', 'very_elevated']),
  timestamp: z.string().datetime()
});

const WarburgEffectSchema = z.object({
  index: z.number().min(0).max(100),
  status: z.enum(['normal', 'mild', 'moderate', 'elevated', 'severe']),
  glycolysisRate: z.number().positive(),
  lactateLevel: z.number().positive()
});

const MetabolicProfileSchema = z.object({
  warburgEffect: WarburgEffectSchema,
  biomarkers: z.array(BiomarkerSchema),
  metabolites: z.array(z.object({
    id: z.string(),
    name: z.string(),
    concentration: z.number(),
    unit: z.string(),
    pathway: z.string()
  })),
  pathways: z.array(z.object({
    pathway: z.string(),
    activityScore: z.number().min(0).max(100),
    affectedGenes: z.array(z.string()),
    affectedMetabolites: z.array(z.string())
  }))
});

// ============================================================================
// Main SDK Class
// ============================================================================

export class WIACancerMetabolismSDK {
  private client: AxiosInstance;
  private config: SDKConfig;

  constructor(config: SDKConfig) {
    this.config = { ...DEFAULT_CONFIG, ...config };

    this.client = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'CANCER-METABOLISM',
        'X-WIA-Version': WIA_CANCER_METABOLISM_VERSION
      }
    });

    this.setupInterceptors();
  }

  private setupInterceptors(): void {
    this.client.interceptors.response.use(
      (response) => response,
      async (error: AxiosError) => {
        if (this.config.retryAttempts && this.config.retryAttempts > 0) {
          const config = error.config as any;
          config.retryCount = config.retryCount || 0;

          if (config.retryCount < this.config.retryAttempts!) {
            config.retryCount++;
            await this.delay(Math.pow(2, config.retryCount) * 1000);
            return this.client.request(config);
          }
        }
        return Promise.reject(error);
      }
    );
  }

  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  private log(message: string, data?: unknown): void {
    if (this.config.enableLogging) {
      console.log(`[WIA-CANCER-METABOLISM] ${message}`, data || '');
    }
  }

  // ==========================================================================
  // Profile Management
  // ==========================================================================

  /**
   * Create a new cancer metabolism profile
   */
  async createProfile(profile: Omit<CancerMetabolismProfile, '@context' | '@type' | 'version' | 'timestamp' | 'philosophy'>): Promise<APIResponse<CancerMetabolismProfile>> {
    this.log('Creating profile', { patientId: profile.patientId });

    const fullProfile: CancerMetabolismProfile = {
      '@context': WIA_SCHEMA_CONTEXT,
      '@type': 'CancerMetabolismProfile',
      version: `WIA-CANCER-METABOLISM-v${WIA_CANCER_METABOLISM_VERSION}`,
      timestamp: new Date().toISOString(),
      philosophy: WIA_PHILOSOPHY,
      ...profile
    };

    const response = await this.client.post<APIResponse<CancerMetabolismProfile>>('/profiles', fullProfile);
    return response.data;
  }

  /**
   * Get a profile by ID
   */
  async getProfile(profileId: string): Promise<APIResponse<CancerMetabolismProfile>> {
    this.log('Getting profile', { profileId });
    const response = await this.client.get<APIResponse<CancerMetabolismProfile>>(`/profiles/${profileId}`);
    return response.data;
  }

  /**
   * Update an existing profile
   */
  async updateProfile(profileId: string, updates: Partial<CancerMetabolismProfile>): Promise<APIResponse<CancerMetabolismProfile>> {
    this.log('Updating profile', { profileId });
    const response = await this.client.patch<APIResponse<CancerMetabolismProfile>>(`/profiles/${profileId}`, updates);
    return response.data;
  }

  /**
   * Search profiles with filters
   */
  async searchProfiles(query: SearchQuery, page = 1, limit = 20): Promise<PaginatedResponse<CancerMetabolismProfile>> {
    this.log('Searching profiles', { query, page, limit });
    const response = await this.client.get<PaginatedResponse<CancerMetabolismProfile>>('/profiles/search', {
      params: { ...query, page, limit }
    });
    return response.data;
  }

  // ==========================================================================
  // Biomarker Operations
  // ==========================================================================

  /**
   * Get biomarkers for a profile
   */
  async getBiomarkers(profileId: string): Promise<APIResponse<Biomarker[]>> {
    this.log('Getting biomarkers', { profileId });
    const response = await this.client.get<APIResponse<Biomarker[]>>(`/profiles/${profileId}/biomarkers`);
    return response.data;
  }

  /**
   * Add a biomarker to a profile
   */
  async addBiomarker(profileId: string, biomarker: Omit<Biomarker, 'id' | 'timestamp'>): Promise<APIResponse<Biomarker>> {
    this.log('Adding biomarker', { profileId, biomarkerName: biomarker.name });

    const fullBiomarker = {
      ...biomarker,
      id: `BIO-${Date.now()}`,
      timestamp: new Date().toISOString()
    };

    const response = await this.client.post<APIResponse<Biomarker>>(`/profiles/${profileId}/biomarkers`, fullBiomarker);
    return response.data;
  }

  /**
   * Analyze Warburg effect
   */
  async analyzeWarburgEffect(profileId: string): Promise<APIResponse<{ index: number; status: WarburgStatus; interpretation: string }>> {
    this.log('Analyzing Warburg effect', { profileId });
    const response = await this.client.get<APIResponse<{ index: number; status: WarburgStatus; interpretation: string }>>(`/profiles/${profileId}/warburg-analysis`);
    return response.data;
  }

  // ==========================================================================
  // Pathway Analysis
  // ==========================================================================

  /**
   * Analyze metabolic pathways
   */
  async analyzePathways(profileId: string, pathways?: MetabolicPathway[]): Promise<APIResponse<PathwayAnalysis[]>> {
    this.log('Analyzing pathways', { profileId, pathways });
    const response = await this.client.post<APIResponse<PathwayAnalysis[]>>(`/profiles/${profileId}/pathway-analysis`, { pathways });
    return response.data;
  }

  /**
   * Get pathway enrichment
   */
  async getPathwayEnrichment(metaboliteIds: string[]): Promise<APIResponse<{ pathway: MetabolicPathway; enrichmentScore: number; pValue: number }[]>> {
    this.log('Getting pathway enrichment', { metaboliteCount: metaboliteIds.length });
    const response = await this.client.post<APIResponse<{ pathway: MetabolicPathway; enrichmentScore: number; pValue: number }[]>>('/pathways/enrichment', { metaboliteIds });
    return response.data;
  }

  // ==========================================================================
  // Validation
  // ==========================================================================

  /**
   * Validate a profile against WIA schema
   */
  validateProfile(profile: unknown): ValidationResult {
    this.log('Validating profile');

    const errors: { path: string; code: string; message: string }[] = [];
    const warnings: { path: string; code: string; message: string }[] = [];

    try {
      // Basic structure validation
      if (typeof profile !== 'object' || profile === null) {
        errors.push({ path: '$', code: 'INVALID_TYPE', message: 'Profile must be an object' });
        return { valid: false, errors, warnings };
      }

      const p = profile as Record<string, unknown>;

      // Required fields
      if (!p.patientId) errors.push({ path: '$.patientId', code: 'REQUIRED', message: 'Patient ID is required' });
      if (!p.diagnosis) errors.push({ path: '$.diagnosis', code: 'REQUIRED', message: 'Diagnosis is required' });
      if (!p.metabolicProfile) errors.push({ path: '$.metabolicProfile', code: 'REQUIRED', message: 'Metabolic profile is required' });

      // Validate metabolic profile if present
      if (p.metabolicProfile) {
        const result = MetabolicProfileSchema.safeParse(p.metabolicProfile);
        if (!result.success) {
          result.error.issues.forEach(issue => {
            errors.push({
              path: `$.metabolicProfile.${issue.path.join('.')}`,
              code: 'VALIDATION_ERROR',
              message: issue.message
            });
          });
        }
      }

      // Compliance warnings
      if (!p.compliance) {
        warnings.push({ path: '$.compliance', code: 'MISSING_COMPLIANCE', message: 'Compliance information is recommended' });
      }

    } catch (error) {
      errors.push({ path: '$', code: 'VALIDATION_ERROR', message: String(error) });
    }

    return { valid: errors.length === 0, errors, warnings };
  }

  // ==========================================================================
  // Protocol Messaging
  // ==========================================================================

  /**
   * Build a protocol message for cross-institutional exchange
   */
  buildProtocolMessage(type: string, payload: unknown, target?: { id: string; name: string }): ProtocolMessage {
    return {
      header: {
        version: WIA_CANCER_METABOLISM_VERSION,
        messageId: `MSG-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
        type: type as any,
        timestamp: new Date().toISOString(),
        source: {
          id: 'self',
          name: 'WIA SDK Client',
          type: 'research'
        },
        target: target ? { ...target, type: 'research' } : undefined
      },
      payload
    };
  }

  /**
   * Send a protocol message
   */
  async sendMessage(message: ProtocolMessage): Promise<APIResponse<{ messageId: string; status: string }>> {
    this.log('Sending protocol message', { type: message.header.type });
    const response = await this.client.post<APIResponse<{ messageId: string; status: string }>>('/messages', message);
    return response.data;
  }

  // ==========================================================================
  // Utility Methods
  // ==========================================================================

  /**
   * Calculate Warburg index from metabolic data
   */
  static calculateWarburgIndex(glycolysisRate: number, lactateLevel: number, oxygenLevel: number): number {
    // Warburg index formula based on glycolytic flux and lactate accumulation
    const normalGlycolysis = 2.0; // mmol/L/h
    const normalLactate = 2.0; // mmol/L
    const normalOxygen = 21; // %

    const glycolysisRatio = glycolysisRate / normalGlycolysis;
    const lactateRatio = lactateLevel / normalLactate;
    const hypoxiaFactor = (normalOxygen - oxygenLevel) / normalOxygen;

    const rawIndex = (glycolysisRatio * 0.4 + lactateRatio * 0.4 + hypoxiaFactor * 0.2) * 50;
    return Math.min(100, Math.max(0, Math.round(rawIndex)));
  }

  /**
   * Get Warburg status from index
   */
  static getWarburgStatus(index: number): WarburgStatus {
    if (index < 20) return WarburgStatus.NORMAL;
    if (index < 40) return WarburgStatus.MILD;
    if (index < 60) return WarburgStatus.MODERATE;
    if (index < 80) return WarburgStatus.ELEVATED;
    return WarburgStatus.SEVERE;
  }

  /**
   * Get SDK version
   */
  static getVersion(): string {
    return WIA_CANCER_METABOLISM_VERSION;
  }
}

// ============================================================================
// Export all types and utilities
// ============================================================================

export * from './types';
export { WIA_CANCER_METABOLISM_VERSION, WIA_SCHEMA_CONTEXT, WIA_PHILOSOPHY };
