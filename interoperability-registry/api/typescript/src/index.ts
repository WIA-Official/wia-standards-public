/**
 * WIA-CORE-004: Interoperability Registry SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Core Standards Working Group
 *
 * 弘익人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for interacting with the WIA
 * Interoperability Registry including:
 * - System registration and management
 * - Standard discovery and search
 * - Compliance verification
 * - Integration template generation
 * - Analytics and monitoring
 */

import {
  RegistryConfig,
  StandardDefinition,
  SystemRegistration,
  DiscoveryQuery,
  DiscoveryResponse,
  ComplianceVerificationRequest,
  ComplianceVerificationResult,
  CertificationRequest,
  CertificationResult,
  IntegrationTemplate,
  TemplateGenerationRequest,
  IntegrationRecommendation,
  StandardVersionHistory,
  MigrationPath,
  RegistryStatistics,
  TrendingStandard,
  SystemQualityScore,
  RegistryError,
  RegistryErrorCode,
  EntityId,
  SemanticVersion,
  REGISTRY_CONSTANTS,
  CompatibilityResult,
} from './types';

// ============================================================================
// HTTP Client
// ============================================================================

/**
 * HTTP client for registry API calls
 */
class RegistryHttpClient {
  private config: Required<RegistryConfig>;

  constructor(config: RegistryConfig) {
    this.config = {
      endpoint: config.endpoint || REGISTRY_CONSTANTS.DEFAULT_ENDPOINT,
      apiKey: config.apiKey,
      oauth: config.oauth,
      timeout: config.timeout || REGISTRY_CONSTANTS.TIMEOUTS.DEFAULT,
      retries: config.retries || 3,
      cache: config.cache || { enabled: false, ttl: 300 },
    };
  }

  /**
   * Make HTTP request to registry API
   */
  async request<T>(
    method: 'GET' | 'POST' | 'PUT' | 'PATCH' | 'DELETE',
    path: string,
    data?: any,
    customTimeout?: number
  ): Promise<T> {
    const url = `${this.config.endpoint}/api/${REGISTRY_CONSTANTS.API_VERSION}${path}`;
    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
      'User-Agent': 'WIA-Registry-SDK/1.0.0',
    };

    // Add authentication
    if (this.config.apiKey) {
      headers['X-API-Key'] = this.config.apiKey;
    } else if (this.config.oauth) {
      // Get OAuth token (simplified - real implementation would handle token refresh)
      const token = await this.getOAuthToken();
      headers['Authorization'] = `Bearer ${token}`;
    }

    const controller = new AbortController();
    const timeout = customTimeout || this.config.timeout;
    const timeoutId = setTimeout(() => controller.abort(), timeout);

    try {
      const response = await fetch(url, {
        method,
        headers,
        body: data ? JSON.stringify(data) : undefined,
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        await this.handleErrorResponse(response);
      }

      return await response.json();
    } catch (error: any) {
      clearTimeout(timeoutId);
      if (error.name === 'AbortError') {
        throw new RegistryError(
          RegistryErrorCode.INTERNAL_ERROR,
          `Request timeout after ${timeout}ms`,
          408
        );
      }
      throw error;
    }
  }

  /**
   * Get OAuth token
   */
  private async getOAuthToken(): Promise<string> {
    if (!this.config.oauth) {
      throw new RegistryError(
        RegistryErrorCode.UNAUTHORIZED,
        'OAuth configuration missing',
        401
      );
    }

    const response = await fetch(this.config.oauth.tokenUrl, {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: new URLSearchParams({
        grant_type: 'client_credentials',
        client_id: this.config.oauth.clientId,
        client_secret: this.config.oauth.clientSecret,
      }),
    });

    if (!response.ok) {
      throw new RegistryError(
        RegistryErrorCode.UNAUTHORIZED,
        'OAuth token request failed',
        401
      );
    }

    const data = await response.json();
    return data.access_token;
  }

  /**
   * Handle error responses
   */
  private async handleErrorResponse(response: Response): Promise<never> {
    let errorData: any;
    try {
      errorData = await response.json();
    } catch {
      errorData = { message: response.statusText };
    }

    const errorCode = this.mapStatusToErrorCode(response.status);
    throw new RegistryError(
      errorCode,
      errorData.message || 'Request failed',
      response.status,
      errorData
    );
  }

  /**
   * Map HTTP status to error code
   */
  private mapStatusToErrorCode(status: number): RegistryErrorCode {
    switch (status) {
      case 400:
        return RegistryErrorCode.INVALID_REQUEST;
      case 401:
        return RegistryErrorCode.UNAUTHORIZED;
      case 403:
        return RegistryErrorCode.FORBIDDEN;
      case 404:
        return RegistryErrorCode.NOT_FOUND;
      case 409:
        return RegistryErrorCode.CONFLICT;
      case 429:
        return RegistryErrorCode.RATE_LIMIT_EXCEEDED;
      case 422:
        return RegistryErrorCode.VALIDATION_ERROR;
      default:
        return RegistryErrorCode.INTERNAL_ERROR;
    }
  }
}

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-CORE-004 Interoperability Registry SDK
 */
export class InteroperabilityRegistry {
  private client: RegistryHttpClient;
  private version = '1.0.0';

  constructor(config: RegistryConfig) {
    this.client = new RegistryHttpClient(config);
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  // ==========================================================================
  // System Management
  // ==========================================================================

  /**
   * Register a new system in the registry
   *
   * @param registration - System registration data
   * @returns Registered system with assigned ID
   */
  async registerSystem(
    registration: Omit<SystemRegistration, 'systemId' | 'metadata'>
  ): Promise<SystemRegistration> {
    return this.client.request<SystemRegistration>(
      'POST',
      '/systems/register',
      registration,
      REGISTRY_CONSTANTS.TIMEOUTS.REGISTRATION
    );
  }

  /**
   * Get system details by ID
   *
   * @param systemId - System identifier
   * @returns System registration details
   */
  async getSystem(systemId: EntityId): Promise<SystemRegistration> {
    return this.client.request<SystemRegistration>('GET', `/systems/${systemId}`);
  }

  /**
   * Update system registration
   *
   * @param systemId - System identifier
   * @param updates - Partial system updates
   * @returns Updated system registration
   */
  async updateSystem(
    systemId: EntityId,
    updates: Partial<SystemRegistration>
  ): Promise<SystemRegistration> {
    return this.client.request<SystemRegistration>(
      'PUT',
      `/systems/${systemId}`,
      updates
    );
  }

  /**
   * Deactivate system registration
   *
   * @param systemId - System identifier
   * @returns Success status
   */
  async deactivateSystem(systemId: EntityId): Promise<{ success: boolean }> {
    return this.client.request<{ success: boolean }>(
      'DELETE',
      `/systems/${systemId}`
    );
  }

  /**
   * List all systems
   *
   * @param options - Pagination and filter options
   * @returns List of systems
   */
  async listSystems(options?: {
    limit?: number;
    offset?: number;
    category?: string;
  }): Promise<{ systems: SystemRegistration[]; total: number }> {
    const query = new URLSearchParams();
    if (options?.limit) query.set('limit', options.limit.toString());
    if (options?.offset) query.set('offset', options.offset.toString());
    if (options?.category) query.set('category', options.category);

    return this.client.request<{ systems: SystemRegistration[]; total: number }>(
      'GET',
      `/systems?${query.toString()}`
    );
  }

  // ==========================================================================
  // Discovery
  // ==========================================================================

  /**
   * Search for systems matching criteria
   *
   * @param query - Discovery query
   * @returns Matching systems with relevance scores
   */
  async discoverSystems(query: DiscoveryQuery): Promise<DiscoveryResponse> {
    return this.client.request<DiscoveryResponse>(
      'POST',
      '/discovery/search',
      query,
      REGISTRY_CONSTANTS.TIMEOUTS.SEARCH
    );
  }

  /**
   * Calculate compatibility between two systems
   *
   * @param sourceId - Source system ID
   * @param targetId - Target system ID
   * @returns Compatibility score and details
   */
  async calculateCompatibility(
    sourceId: EntityId,
    targetId: EntityId
  ): Promise<CompatibilityResult> {
    return this.client.request<CompatibilityResult>(
      'POST',
      '/discovery/compatibility',
      { sourceSystemId: sourceId, targetSystemIds: [targetId] }
    );
  }

  /**
   * Get integration recommendations
   *
   * @param sourceId - Source system ID
   * @param targetId - Target system ID
   * @returns Integration recommendations
   */
  async getIntegrationRecommendation(params: {
    source: EntityId;
    target: EntityId;
  }): Promise<IntegrationRecommendation> {
    return this.client.request<IntegrationRecommendation>(
      'POST',
      '/discovery/recommendations',
      {
        sourceSystemId: params.source,
        targetSystemId: params.target,
      }
    );
  }

  // ==========================================================================
  // Standards
  // ==========================================================================

  /**
   * Get standard definition
   *
   * @param standardId - Standard identifier
   * @returns Standard definition
   */
  async getStandard(standardId: EntityId): Promise<StandardDefinition> {
    return this.client.request<StandardDefinition>('GET', `/standards/${standardId}`);
  }

  /**
   * List all standards
   *
   * @param category - Optional category filter
   * @returns List of standards
   */
  async listStandards(
    category?: string
  ): Promise<{ standards: StandardDefinition[]; total: number }> {
    const query = category ? `?category=${category}` : '';
    return this.client.request<{ standards: StandardDefinition[]; total: number }>(
      'GET',
      `/standards${query}`
    );
  }

  /**
   * Get standard version history
   *
   * @param standardId - Standard identifier
   * @returns Version history
   */
  async getStandardVersions(standardId: EntityId): Promise<StandardVersionHistory> {
    return this.client.request<StandardVersionHistory>(
      'GET',
      `/standards/${standardId}/versions`
    );
  }

  /**
   * Get migration path between versions
   *
   * @param standardId - Standard identifier
   * @param fromVersion - Source version
   * @param toVersion - Target version
   * @returns Migration guide
   */
  async getMigrationPath(
    standardId: EntityId,
    fromVersion: SemanticVersion,
    toVersion: SemanticVersion
  ): Promise<MigrationPath> {
    return this.client.request<MigrationPath>(
      'GET',
      `/standards/${standardId}/migration?from=${fromVersion}&to=${toVersion}`
    );
  }

  // ==========================================================================
  // Compliance
  // ==========================================================================

  /**
   * Verify system compliance with a standard
   *
   * @param request - Compliance verification request
   * @returns Verification results
   */
  async verifyCompliance(
    request: ComplianceVerificationRequest
  ): Promise<ComplianceVerificationResult> {
    return this.client.request<ComplianceVerificationResult>(
      'POST',
      '/compliance/verify',
      request,
      REGISTRY_CONSTANTS.TIMEOUTS.COMPLIANCE
    );
  }

  /**
   * Request official certification
   *
   * @param request - Certification request
   * @returns Certification status
   */
  async requestCertification(
    request: CertificationRequest
  ): Promise<CertificationResult> {
    return this.client.request<CertificationResult>(
      'POST',
      '/compliance/certify',
      request
    );
  }

  /**
   * Get certification status
   *
   * @param requestId - Certification request ID
   * @returns Current certification status
   */
  async getCertificationStatus(requestId: EntityId): Promise<CertificationResult> {
    return this.client.request<CertificationResult>(
      'GET',
      `/compliance/certificates/${requestId}`
    );
  }

  /**
   * List all certifications for a system
   *
   * @param systemId - System identifier
   * @returns List of certifications
   */
  async listCertifications(
    systemId: EntityId
  ): Promise<{ certifications: CertificationResult[] }> {
    return this.client.request<{ certifications: CertificationResult[] }>(
      'GET',
      `/compliance/certificates?systemId=${systemId}`
    );
  }

  // ==========================================================================
  // Integration Templates
  // ==========================================================================

  /**
   * Generate integration template
   *
   * @param request - Template generation parameters
   * @returns Generated integration code
   */
  async generateTemplate(
    request: TemplateGenerationRequest
  ): Promise<IntegrationTemplate> {
    return this.client.request<IntegrationTemplate>(
      'POST',
      '/templates/generate',
      request
    );
  }

  /**
   * Get existing template
   *
   * @param templateId - Template identifier
   * @returns Integration template
   */
  async getTemplate(templateId: EntityId): Promise<IntegrationTemplate> {
    return this.client.request<IntegrationTemplate>(
      'GET',
      `/templates/${templateId}`
    );
  }

  /**
   * List available templates
   *
   * @param filters - Optional filters
   * @returns List of templates
   */
  async listTemplates(filters?: {
    language?: string;
    protocol?: string;
  }): Promise<{ templates: IntegrationTemplate[]; total: number }> {
    const query = new URLSearchParams();
    if (filters?.language) query.set('language', filters.language);
    if (filters?.protocol) query.set('protocol', filters.protocol);

    return this.client.request<{ templates: IntegrationTemplate[]; total: number }>(
      'GET',
      `/templates?${query.toString()}`
    );
  }

  // ==========================================================================
  // Analytics
  // ==========================================================================

  /**
   * Get registry statistics
   *
   * @param options - Time range and filters
   * @returns Registry statistics
   */
  async getStatistics(options?: {
    timeRange?: 'last-24h' | 'last-7d' | 'last-30d' | 'last-90d';
  }): Promise<RegistryStatistics> {
    const query = options?.timeRange ? `?timeRange=${options.timeRange}` : '';
    return this.client.request<RegistryStatistics>(
      'GET',
      `/analytics/statistics${query}`
    );
  }

  /**
   * Get trending standards
   *
   * @param options - Metric and period
   * @returns List of trending standards
   */
  async getTrendingStandards(options?: {
    metric?: 'adoption-rate' | 'growth-rate' | 'implementations';
    period?: 'last-week' | 'last-month' | 'last-quarter';
  }): Promise<{ trending: TrendingStandard[] }> {
    const query = new URLSearchParams();
    if (options?.metric) query.set('metric', options.metric);
    if (options?.period) query.set('period', options.period);

    return this.client.request<{ trending: TrendingStandard[] }>(
      'GET',
      `/analytics/trending?${query.toString()}`
    );
  }

  /**
   * Get system quality score
   *
   * @param systemId - System identifier
   * @returns Quality score breakdown
   */
  async getSystemQualityScore(systemId: EntityId): Promise<SystemQualityScore> {
    return this.client.request<SystemQualityScore>(
      'GET',
      `/analytics/quality/${systemId}`
    );
  }

  // ==========================================================================
  // Utility Methods
  // ==========================================================================

  /**
   * Validate system registration data
   *
   * @param registration - System registration to validate
   * @returns Validation errors (empty if valid)
   */
  validateRegistration(
    registration: Partial<SystemRegistration>
  ): { valid: boolean; errors: string[] } {
    const errors: string[] = [];

    if (!registration.name || registration.name.trim() === '') {
      errors.push('System name is required');
    }

    if (!registration.version) {
      errors.push('System version is required');
    } else if (!/^\d+\.\d+\.\d+$/.test(registration.version)) {
      errors.push('Version must follow semantic versioning (e.g., 1.0.0)');
    }

    if (!registration.organization?.name) {
      errors.push('Organization name is required');
    }

    if (!registration.organization?.contact?.email) {
      errors.push('Organization contact email is required');
    }

    if (!registration.standards || registration.standards.length === 0) {
      errors.push('At least one standard implementation is required');
    }

    if (!registration.endpoints || registration.endpoints.length === 0) {
      errors.push('At least one endpoint is required');
    }

    return {
      valid: errors.length === 0,
      errors,
    };
  }

  /**
   * Compare compliance levels
   *
   * @param level1 - First compliance level
   * @param level2 - Second compliance level
   * @returns -1 if level1 < level2, 0 if equal, 1 if level1 > level2
   */
  compareComplianceLevels(level1: string, level2: string): number {
    const levels = REGISTRY_CONSTANTS.COMPLIANCE_LEVELS;
    const index1 = levels.indexOf(level1 as any);
    const index2 = levels.indexOf(level2 as any);

    if (index1 < index2) return -1;
    if (index1 > index2) return 1;
    return 0;
  }
}

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * Create a registry client instance
 *
 * @param config - Registry configuration
 * @returns Registry client
 */
export function createRegistry(config: RegistryConfig): InteroperabilityRegistry {
  return new InteroperabilityRegistry(config);
}

/**
 * Quick search for systems by standard
 *
 * @param standardId - Standard ID to search for
 * @param config - Registry configuration
 * @returns Systems implementing the standard
 */
export async function discoverCompatibleSystems(
  standardId: EntityId,
  config: RegistryConfig
): Promise<DiscoveryResponse> {
  const registry = createRegistry(config);
  return registry.discoverSystems({
    standards: [{ id: standardId }],
  });
}

// ============================================================================
// Re-export Types
// ============================================================================

export * from './types';
