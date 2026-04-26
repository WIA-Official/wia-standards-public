import {
  REGISTRY_CONSTANTS,
  RegistryError,
  RegistryErrorCode
} from "./chunk-BEVXQ2HR.mjs";

// src/index.ts
var RegistryHttpClient = class {
  config;
  constructor(config) {
    this.config = {
      endpoint: config.endpoint || REGISTRY_CONSTANTS.DEFAULT_ENDPOINT,
      apiKey: config.apiKey,
      oauth: config.oauth,
      timeout: config.timeout || REGISTRY_CONSTANTS.TIMEOUTS.DEFAULT,
      retries: config.retries || 3,
      cache: config.cache || { enabled: false, ttl: 300 }
    };
  }
  /**
   * Make HTTP request to registry API
   */
  async request(method, path, data, customTimeout) {
    const url = `${this.config.endpoint}/api/${REGISTRY_CONSTANTS.API_VERSION}${path}`;
    const headers = {
      "Content-Type": "application/json",
      "User-Agent": "WIA-Registry-SDK/1.0.0"
    };
    if (this.config.apiKey) {
      headers["X-API-Key"] = this.config.apiKey;
    } else if (this.config.oauth) {
      const token = await this.getOAuthToken();
      headers["Authorization"] = `Bearer ${token}`;
    }
    const controller = new AbortController();
    const timeout = customTimeout || this.config.timeout;
    const timeoutId = setTimeout(() => controller.abort(), timeout);
    try {
      const response = await fetch(url, {
        method,
        headers,
        body: data ? JSON.stringify(data) : void 0,
        signal: controller.signal
      });
      clearTimeout(timeoutId);
      if (!response.ok) {
        await this.handleErrorResponse(response);
      }
      return await response.json();
    } catch (error) {
      clearTimeout(timeoutId);
      if (error.name === "AbortError") {
        throw new RegistryError(
          "INTERNAL_ERROR" /* INTERNAL_ERROR */,
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
  async getOAuthToken() {
    if (!this.config.oauth) {
      throw new RegistryError(
        "UNAUTHORIZED" /* UNAUTHORIZED */,
        "OAuth configuration missing",
        401
      );
    }
    const response = await fetch(this.config.oauth.tokenUrl, {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body: new URLSearchParams({
        grant_type: "client_credentials",
        client_id: this.config.oauth.clientId,
        client_secret: this.config.oauth.clientSecret
      })
    });
    if (!response.ok) {
      throw new RegistryError(
        "UNAUTHORIZED" /* UNAUTHORIZED */,
        "OAuth token request failed",
        401
      );
    }
    const data = await response.json();
    return data.access_token;
  }
  /**
   * Handle error responses
   */
  async handleErrorResponse(response) {
    let errorData;
    try {
      errorData = await response.json();
    } catch {
      errorData = { message: response.statusText };
    }
    const errorCode = this.mapStatusToErrorCode(response.status);
    throw new RegistryError(
      errorCode,
      errorData.message || "Request failed",
      response.status,
      errorData
    );
  }
  /**
   * Map HTTP status to error code
   */
  mapStatusToErrorCode(status) {
    switch (status) {
      case 400:
        return "INVALID_REQUEST" /* INVALID_REQUEST */;
      case 401:
        return "UNAUTHORIZED" /* UNAUTHORIZED */;
      case 403:
        return "FORBIDDEN" /* FORBIDDEN */;
      case 404:
        return "NOT_FOUND" /* NOT_FOUND */;
      case 409:
        return "CONFLICT" /* CONFLICT */;
      case 429:
        return "RATE_LIMIT_EXCEEDED" /* RATE_LIMIT_EXCEEDED */;
      case 422:
        return "VALIDATION_ERROR" /* VALIDATION_ERROR */;
      default:
        return "INTERNAL_ERROR" /* INTERNAL_ERROR */;
    }
  }
};
var InteroperabilityRegistry = class {
  client;
  version = "1.0.0";
  constructor(config) {
    this.client = new RegistryHttpClient(config);
  }
  /**
   * Get SDK version
   */
  getVersion() {
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
  async registerSystem(registration) {
    return this.client.request(
      "POST",
      "/systems/register",
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
  async getSystem(systemId) {
    return this.client.request("GET", `/systems/${systemId}`);
  }
  /**
   * Update system registration
   *
   * @param systemId - System identifier
   * @param updates - Partial system updates
   * @returns Updated system registration
   */
  async updateSystem(systemId, updates) {
    return this.client.request(
      "PUT",
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
  async deactivateSystem(systemId) {
    return this.client.request(
      "DELETE",
      `/systems/${systemId}`
    );
  }
  /**
   * List all systems
   *
   * @param options - Pagination and filter options
   * @returns List of systems
   */
  async listSystems(options) {
    const query = new URLSearchParams();
    if (options == null ? void 0 : options.limit) query.set("limit", options.limit.toString());
    if (options == null ? void 0 : options.offset) query.set("offset", options.offset.toString());
    if (options == null ? void 0 : options.category) query.set("category", options.category);
    return this.client.request(
      "GET",
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
  async discoverSystems(query) {
    return this.client.request(
      "POST",
      "/discovery/search",
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
  async calculateCompatibility(sourceId, targetId) {
    return this.client.request(
      "POST",
      "/discovery/compatibility",
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
  async getIntegrationRecommendation(params) {
    return this.client.request(
      "POST",
      "/discovery/recommendations",
      {
        sourceSystemId: params.source,
        targetSystemId: params.target
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
  async getStandard(standardId) {
    return this.client.request("GET", `/standards/${standardId}`);
  }
  /**
   * List all standards
   *
   * @param category - Optional category filter
   * @returns List of standards
   */
  async listStandards(category) {
    const query = category ? `?category=${category}` : "";
    return this.client.request(
      "GET",
      `/standards${query}`
    );
  }
  /**
   * Get standard version history
   *
   * @param standardId - Standard identifier
   * @returns Version history
   */
  async getStandardVersions(standardId) {
    return this.client.request(
      "GET",
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
  async getMigrationPath(standardId, fromVersion, toVersion) {
    return this.client.request(
      "GET",
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
  async verifyCompliance(request) {
    return this.client.request(
      "POST",
      "/compliance/verify",
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
  async requestCertification(request) {
    return this.client.request(
      "POST",
      "/compliance/certify",
      request
    );
  }
  /**
   * Get certification status
   *
   * @param requestId - Certification request ID
   * @returns Current certification status
   */
  async getCertificationStatus(requestId) {
    return this.client.request(
      "GET",
      `/compliance/certificates/${requestId}`
    );
  }
  /**
   * List all certifications for a system
   *
   * @param systemId - System identifier
   * @returns List of certifications
   */
  async listCertifications(systemId) {
    return this.client.request(
      "GET",
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
  async generateTemplate(request) {
    return this.client.request(
      "POST",
      "/templates/generate",
      request
    );
  }
  /**
   * Get existing template
   *
   * @param templateId - Template identifier
   * @returns Integration template
   */
  async getTemplate(templateId) {
    return this.client.request(
      "GET",
      `/templates/${templateId}`
    );
  }
  /**
   * List available templates
   *
   * @param filters - Optional filters
   * @returns List of templates
   */
  async listTemplates(filters) {
    const query = new URLSearchParams();
    if (filters == null ? void 0 : filters.language) query.set("language", filters.language);
    if (filters == null ? void 0 : filters.protocol) query.set("protocol", filters.protocol);
    return this.client.request(
      "GET",
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
  async getStatistics(options) {
    const query = (options == null ? void 0 : options.timeRange) ? `?timeRange=${options.timeRange}` : "";
    return this.client.request(
      "GET",
      `/analytics/statistics${query}`
    );
  }
  /**
   * Get trending standards
   *
   * @param options - Metric and period
   * @returns List of trending standards
   */
  async getTrendingStandards(options) {
    const query = new URLSearchParams();
    if (options == null ? void 0 : options.metric) query.set("metric", options.metric);
    if (options == null ? void 0 : options.period) query.set("period", options.period);
    return this.client.request(
      "GET",
      `/analytics/trending?${query.toString()}`
    );
  }
  /**
   * Get system quality score
   *
   * @param systemId - System identifier
   * @returns Quality score breakdown
   */
  async getSystemQualityScore(systemId) {
    return this.client.request(
      "GET",
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
  validateRegistration(registration) {
    var _a, _b, _c;
    const errors = [];
    if (!registration.name || registration.name.trim() === "") {
      errors.push("System name is required");
    }
    if (!registration.version) {
      errors.push("System version is required");
    } else if (!/^\d+\.\d+\.\d+$/.test(registration.version)) {
      errors.push("Version must follow semantic versioning (e.g., 1.0.0)");
    }
    if (!((_a = registration.organization) == null ? void 0 : _a.name)) {
      errors.push("Organization name is required");
    }
    if (!((_c = (_b = registration.organization) == null ? void 0 : _b.contact) == null ? void 0 : _c.email)) {
      errors.push("Organization contact email is required");
    }
    if (!registration.standards || registration.standards.length === 0) {
      errors.push("At least one standard implementation is required");
    }
    if (!registration.endpoints || registration.endpoints.length === 0) {
      errors.push("At least one endpoint is required");
    }
    return {
      valid: errors.length === 0,
      errors
    };
  }
  /**
   * Compare compliance levels
   *
   * @param level1 - First compliance level
   * @param level2 - Second compliance level
   * @returns -1 if level1 < level2, 0 if equal, 1 if level1 > level2
   */
  compareComplianceLevels(level1, level2) {
    const levels = REGISTRY_CONSTANTS.COMPLIANCE_LEVELS;
    const index1 = levels.indexOf(level1);
    const index2 = levels.indexOf(level2);
    if (index1 < index2) return -1;
    if (index1 > index2) return 1;
    return 0;
  }
};
function createRegistry(config) {
  return new InteroperabilityRegistry(config);
}
async function discoverCompatibleSystems(standardId, config) {
  const registry = createRegistry(config);
  return registry.discoverSystems({
    standards: [{ id: standardId }]
  });
}
export {
  InteroperabilityRegistry,
  REGISTRY_CONSTANTS,
  RegistryError,
  RegistryErrorCode,
  createRegistry,
  discoverCompatibleSystems
};
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
