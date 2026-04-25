/**
 * WIA Data Quality Standard - TypeScript SDK
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig,
  WIADataQuality,
  ProfileResponse,
  QualityProfile,
  QualityDimension,
  QualityRule,
  QualityCheck,
  QualityReport,
  CheckRunRequest,
  CheckRunResponse,
  RuleResult,
  ValidationResult,
  PaginatedResponse,
  DimensionName,
  RuleType,
} from './types';

// ============================================================================
// WIA Data Quality Client
// ============================================================================

export class WIADataQualityClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }),
      },
    });
  }

  // ========================================================================
  // Profile Management
  // ========================================================================

  /**
   * Create a new quality profile
   */
  async createProfile(profile: WIADataQuality): Promise<ProfileResponse> {
    const response = await this.axios.post<ProfileResponse>('/profiles', profile);
    return response.data;
  }

  /**
   * Get profile by ID
   */
  async getProfile(id: string): Promise<WIADataQuality> {
    const response = await this.axios.get<WIADataQuality>(`/profiles/${id}`);
    return response.data;
  }

  /**
   * List all quality profiles
   */
  async listProfiles(params?: {
    status?: string;
    owner?: string;
    dataAsset?: string;
    limit?: number;
    offset?: number;
  }): Promise<PaginatedResponse<ProfileResponse>> {
    const response = await this.axios.get<PaginatedResponse<ProfileResponse>>('/profiles', {
      params,
    });
    return response.data;
  }

  /**
   * Update existing profile
   */
  async updateProfile(id: string, updates: Partial<WIADataQuality>): Promise<ProfileResponse> {
    const response = await this.axios.put<ProfileResponse>(`/profiles/${id}`, updates);
    return response.data;
  }

  /**
   * Delete profile
   */
  async deleteProfile(id: string): Promise<void> {
    await this.axios.delete(`/profiles/${id}`);
  }

  // ========================================================================
  // Rule Management
  // ========================================================================

  /**
   * Add rule to profile
   */
  async addRule(profileId: string, rule: QualityRule): Promise<QualityRule> {
    const response = await this.axios.post<QualityRule>(
      `/profiles/${profileId}/rules`,
      rule
    );
    return response.data;
  }

  /**
   * Get rule by ID
   */
  async getRule(profileId: string, ruleId: string): Promise<QualityRule> {
    const response = await this.axios.get<QualityRule>(
      `/profiles/${profileId}/rules/${ruleId}`
    );
    return response.data;
  }

  /**
   * List all rules in profile
   */
  async listRules(
    profileId: string,
    params?: {
      dimension?: string;
      type?: string;
      enabled?: boolean;
      limit?: number;
      offset?: number;
    }
  ): Promise<PaginatedResponse<QualityRule>> {
    const response = await this.axios.get<PaginatedResponse<QualityRule>>(
      `/profiles/${profileId}/rules`,
      { params }
    );
    return response.data;
  }

  /**
   * Update rule
   */
  async updateRule(
    profileId: string,
    ruleId: string,
    updates: Partial<QualityRule>
  ): Promise<QualityRule> {
    const response = await this.axios.put<QualityRule>(
      `/profiles/${profileId}/rules/${ruleId}`,
      updates
    );
    return response.data;
  }

  /**
   * Delete rule
   */
  async deleteRule(profileId: string, ruleId: string): Promise<void> {
    await this.axios.delete(`/profiles/${profileId}/rules/${ruleId}`);
  }

  /**
   * Enable/disable rule
   */
  async toggleRule(profileId: string, ruleId: string, enabled: boolean): Promise<void> {
    await this.axios.patch(`/profiles/${profileId}/rules/${ruleId}`, { enabled });
  }

  // ========================================================================
  // Check Management
  // ========================================================================

  /**
   * Create a quality check
   */
  async createCheck(profileId: string, check: QualityCheck): Promise<QualityCheck> {
    const response = await this.axios.post<QualityCheck>(
      `/profiles/${profileId}/checks`,
      check
    );
    return response.data;
  }

  /**
   * Get check by ID
   */
  async getCheck(profileId: string, checkId: string): Promise<QualityCheck> {
    const response = await this.axios.get<QualityCheck>(
      `/profiles/${profileId}/checks/${checkId}`
    );
    return response.data;
  }

  /**
   * List all checks for profile
   */
  async listChecks(
    profileId: string,
    params?: {
      type?: string;
      limit?: number;
      offset?: number;
    }
  ): Promise<PaginatedResponse<QualityCheck>> {
    const response = await this.axios.get<PaginatedResponse<QualityCheck>>(
      `/profiles/${profileId}/checks`,
      { params }
    );
    return response.data;
  }

  /**
   * Trigger check run
   */
  async runCheck(request: CheckRunRequest): Promise<CheckRunResponse> {
    const response = await this.axios.post<CheckRunResponse>('/runs', request);
    return response.data;
  }

  /**
   * Get check run status
   */
  async getRunStatus(runId: string): Promise<CheckRunResponse> {
    const response = await this.axios.get<CheckRunResponse>(`/runs/${runId}`);
    return response.data;
  }

  /**
   * Cancel running check
   */
  async cancelRun(runId: string): Promise<void> {
    await this.axios.post(`/runs/${runId}/cancel`);
  }

  // ========================================================================
  // Report Management
  // ========================================================================

  /**
   * Get quality report
   */
  async getReport(reportId: string): Promise<QualityReport> {
    const response = await this.axios.get<QualityReport>(`/reports/${reportId}`);
    return response.data;
  }

  /**
   * List reports for profile
   */
  async listReports(
    profileId: string,
    params?: {
      checkId?: string;
      startTime?: string;
      endTime?: string;
      limit?: number;
      offset?: number;
    }
  ): Promise<PaginatedResponse<QualityReport>> {
    const response = await this.axios.get<PaginatedResponse<QualityReport>>(
      `/profiles/${profileId}/reports`,
      { params }
    );
    return response.data;
  }

  /**
   * Get latest report for profile
   */
  async getLatestReport(profileId: string): Promise<QualityReport> {
    const response = await this.axios.get<QualityReport>(
      `/profiles/${profileId}/reports/latest`
    );
    return response.data;
  }

  /**
   * Export report
   */
  async exportReport(
    reportId: string,
    format: 'json' | 'csv' | 'pdf' | 'html'
  ): Promise<Blob> {
    const response = await this.axios.get(`/reports/${reportId}/export`, {
      params: { format },
      responseType: 'blob',
    });
    return response.data;
  }

  // ========================================================================
  // Analytics
  // ========================================================================

  /**
   * Get quality score trend
   */
  async getScoreTrend(
    profileId: string,
    params?: {
      startTime?: string;
      endTime?: string;
      granularity?: 'hour' | 'day' | 'week' | 'month';
    }
  ): Promise<{
    timestamps: string[];
    scores: number[];
    dimensions: Record<DimensionName, number[]>;
  }> {
    const response = await this.axios.get(`/profiles/${profileId}/analytics/trend`, {
      params,
    });
    return response.data;
  }

  /**
   * Get rule failure analysis
   */
  async getRuleFailureAnalysis(
    profileId: string,
    params?: {
      startTime?: string;
      endTime?: string;
    }
  ): Promise<{
    topFailingRules: { ruleId: string; ruleName: string; failureCount: number }[];
    failuresByDimension: Record<DimensionName, number>;
    failureTrend: { date: string; count: number }[];
  }> {
    const response = await this.axios.get(`/profiles/${profileId}/analytics/failures`, {
      params,
    });
    return response.data;
  }

  /**
   * Get data quality summary across profiles
   */
  async getQualitySummary(): Promise<{
    totalProfiles: number;
    avgScore: number;
    profilesByStatus: Record<string, number>;
    recentIssues: { profileId: string; profileName: string; issue: string }[];
  }> {
    const response = await this.axios.get('/analytics/summary');
    return response.data;
  }

  // ========================================================================
  // Validation
  // ========================================================================

  /**
   * Validate quality profile
   */
  validateProfile(profile: WIADataQuality): ValidationResult {
    const errors: any[] = [];

    if (!profile.standard || profile.standard !== 'WIA-DATA-QUALITY') {
      errors.push({
        path: 'standard',
        message: 'Standard must be "WIA-DATA-QUALITY"',
      });
    }

    if (!profile.version || !/^\d+\.\d+\.\d+$/.test(profile.version)) {
      errors.push({
        path: 'version',
        message: 'Version must follow semantic versioning (x.y.z)',
      });
    }

    if (!profile.profile || !profile.profile.id) {
      errors.push({
        path: 'profile.id',
        message: 'Profile ID is required',
      });
    }

    if (!profile.dimensions || profile.dimensions.length === 0) {
      errors.push({
        path: 'dimensions',
        message: 'At least one quality dimension is required',
      });
    }

    // Validate dimension weights sum to 100
    if (profile.dimensions) {
      const totalWeight = profile.dimensions.reduce((sum, d) => sum + d.weight, 0);
      if (Math.abs(totalWeight - 100) > 0.01) {
        errors.push({
          path: 'dimensions',
          message: `Dimension weights must sum to 100, got ${totalWeight}`,
        });
      }
    }

    // Validate rules reference valid dimensions
    if (profile.rules) {
      const validDimensions = new Set(profile.dimensions?.map((d) => d.name) || []);
      profile.rules.forEach((rule, index) => {
        if (!validDimensions.has(rule.dimension)) {
          errors.push({
            path: `rules[${index}].dimension`,
            message: `Rule dimension "${rule.dimension}" not found in dimensions`,
          });
        }
      });
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined,
    };
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Generate a UUID v4
 */
export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    const v = c === 'x' ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
}

/**
 * Create a minimal valid data quality profile
 */
export function createMinimalProfile(name: string, dataAssetName: string): WIADataQuality {
  return {
    standard: 'WIA-DATA-QUALITY',
    version: '1.0.0',
    profile: {
      id: generateUUID(),
      name,
      owner: 'default',
      dataAsset: {
        id: generateUUID(),
        name: dataAssetName,
        type: 'table',
        location: {
          system: 'default',
        },
      },
      createdAt: new Date().toISOString(),
      status: 'draft',
    },
    dimensions: [
      { id: generateUUID(), name: 'completeness', weight: 25, targetScore: 95, metrics: [], thresholds: { critical: 70, warning: 85, acceptable: 95 } },
      { id: generateUUID(), name: 'accuracy', weight: 25, targetScore: 95, metrics: [], thresholds: { critical: 70, warning: 85, acceptable: 95 } },
      { id: generateUUID(), name: 'consistency', weight: 25, targetScore: 95, metrics: [], thresholds: { critical: 70, warning: 85, acceptable: 95 } },
      { id: generateUUID(), name: 'validity', weight: 25, targetScore: 95, metrics: [], thresholds: { critical: 70, warning: 85, acceptable: 95 } },
    ],
    rules: [],
    checks: [],
  };
}

/**
 * Create a null check rule
 */
export function createNullCheckRule(
  name: string,
  columns: string[],
  maxNullPercentage: number = 0
): QualityRule {
  return {
    id: generateUUID(),
    name,
    dimension: 'completeness',
    type: 'null-check',
    scope: {
      type: 'column',
      columns,
    },
    definition: {
      type: 'null-check',
      maxNullPercentage,
      treatEmptyAsNull: true,
    },
    severity: 'major',
    enabled: true,
  };
}

/**
 * Create a format check rule
 */
export function createFormatCheckRule(
  name: string,
  column: string,
  predefinedFormat: 'email' | 'phone' | 'url' | 'date' | 'uuid' | 'ip'
): QualityRule {
  return {
    id: generateUUID(),
    name,
    dimension: 'validity',
    type: 'format-check',
    scope: {
      type: 'column',
      columns: [column],
    },
    definition: {
      type: 'format-check',
      pattern: '',
      patternType: 'predefined',
      predefinedFormat,
    },
    severity: 'major',
    enabled: true,
  };
}

/**
 * Create a range check rule
 */
export function createRangeCheckRule(
  name: string,
  column: string,
  min?: number,
  max?: number
): QualityRule {
  return {
    id: generateUUID(),
    name,
    dimension: 'validity',
    type: 'range-check',
    scope: {
      type: 'column',
      columns: [column],
    },
    definition: {
      type: 'range-check',
      min,
      max,
      inclusive: true,
    },
    severity: 'major',
    enabled: true,
  };
}

/**
 * Create a uniqueness check rule
 */
export function createUniquenessCheckRule(
  name: string,
  columns: string[]
): QualityRule {
  return {
    id: generateUUID(),
    name,
    dimension: 'uniqueness',
    type: 'uniqueness-check',
    scope: {
      type: 'table',
      columns,
    },
    definition: {
      type: 'uniqueness-check',
      columns,
      scope: 'table',
    },
    severity: 'critical',
    enabled: true,
  };
}

/**
 * Calculate overall quality score from dimension scores
 */
export function calculateOverallScore(
  dimensions: { name: DimensionName; score: number; weight: number }[]
): number {
  const totalWeight = dimensions.reduce((sum, d) => sum + d.weight, 0);
  if (totalWeight === 0) return 0;

  const weightedSum = dimensions.reduce((sum, d) => sum + d.score * d.weight, 0);
  return Math.round((weightedSum / totalWeight) * 100) / 100;
}

// ============================================================================
// Exports
// ============================================================================

export default {
  WIADataQualityClient,
  generateUUID,
  createMinimalProfile,
  createNullCheckRule,
  createFormatCheckRule,
  createRangeCheckRule,
  createUniquenessCheckRule,
  calculateOverallScore,
};
