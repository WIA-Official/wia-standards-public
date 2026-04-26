import { RegistryConfig, SystemRegistration, EntityId, DiscoveryQuery, DiscoveryResponse, CompatibilityResult, IntegrationRecommendation, StandardDefinition, StandardVersionHistory, SemanticVersion, MigrationPath, ComplianceVerificationRequest, ComplianceVerificationResult, CertificationRequest, CertificationResult, TemplateGenerationRequest, IntegrationTemplate, RegistryStatistics, TrendingStandard, SystemQualityScore } from './types.js';
export { ApiDocumentation, AuthenticationMethod, BreakingChange, CapabilityFilter, CertificationStatusType, CompatibilityDetails, ComplianceConfiguration, ComplianceLevel, DataResidency, DateTime, DiscoveryResultItem, Email, EndpointAuthentication, EndpointAvailability, EndpointType, GeographicFilter, IntegrationComplexity, IntegrationPattern, MatchedCapabilities, MigrationStep, OAuth2Config, OrganizationFilter, OrganizationInfo, PackageDependency, PerformanceFilter, ProgrammingLanguage, QualityDimensions, REGISTRY_CONSTANTS, RateLimit, RegistryError, RegistryErrorCode, SortBy, SortOrder, StandardCategory, StandardDependency, StandardFilter, StandardImplementation, StandardRequirements, StandardScope, StandardSpecification, StandardStatus, SystemCapabilities, SystemEndpoint, SystemMetadata, SystemVisibility, TestResult, TestSeverity, TestStatus, TestType, Url, VersionChanges, VersionInfo, VersionStatus, VersioningConfiguration } from './types.js';

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

/**
 * WIA-CORE-004 Interoperability Registry SDK
 */
declare class InteroperabilityRegistry {
    private client;
    private version;
    constructor(config: RegistryConfig);
    /**
     * Get SDK version
     */
    getVersion(): string;
    /**
     * Register a new system in the registry
     *
     * @param registration - System registration data
     * @returns Registered system with assigned ID
     */
    registerSystem(registration: Omit<SystemRegistration, 'systemId' | 'metadata'>): Promise<SystemRegistration>;
    /**
     * Get system details by ID
     *
     * @param systemId - System identifier
     * @returns System registration details
     */
    getSystem(systemId: EntityId): Promise<SystemRegistration>;
    /**
     * Update system registration
     *
     * @param systemId - System identifier
     * @param updates - Partial system updates
     * @returns Updated system registration
     */
    updateSystem(systemId: EntityId, updates: Partial<SystemRegistration>): Promise<SystemRegistration>;
    /**
     * Deactivate system registration
     *
     * @param systemId - System identifier
     * @returns Success status
     */
    deactivateSystem(systemId: EntityId): Promise<{
        success: boolean;
    }>;
    /**
     * List all systems
     *
     * @param options - Pagination and filter options
     * @returns List of systems
     */
    listSystems(options?: {
        limit?: number;
        offset?: number;
        category?: string;
    }): Promise<{
        systems: SystemRegistration[];
        total: number;
    }>;
    /**
     * Search for systems matching criteria
     *
     * @param query - Discovery query
     * @returns Matching systems with relevance scores
     */
    discoverSystems(query: DiscoveryQuery): Promise<DiscoveryResponse>;
    /**
     * Calculate compatibility between two systems
     *
     * @param sourceId - Source system ID
     * @param targetId - Target system ID
     * @returns Compatibility score and details
     */
    calculateCompatibility(sourceId: EntityId, targetId: EntityId): Promise<CompatibilityResult>;
    /**
     * Get integration recommendations
     *
     * @param sourceId - Source system ID
     * @param targetId - Target system ID
     * @returns Integration recommendations
     */
    getIntegrationRecommendation(params: {
        source: EntityId;
        target: EntityId;
    }): Promise<IntegrationRecommendation>;
    /**
     * Get standard definition
     *
     * @param standardId - Standard identifier
     * @returns Standard definition
     */
    getStandard(standardId: EntityId): Promise<StandardDefinition>;
    /**
     * List all standards
     *
     * @param category - Optional category filter
     * @returns List of standards
     */
    listStandards(category?: string): Promise<{
        standards: StandardDefinition[];
        total: number;
    }>;
    /**
     * Get standard version history
     *
     * @param standardId - Standard identifier
     * @returns Version history
     */
    getStandardVersions(standardId: EntityId): Promise<StandardVersionHistory>;
    /**
     * Get migration path between versions
     *
     * @param standardId - Standard identifier
     * @param fromVersion - Source version
     * @param toVersion - Target version
     * @returns Migration guide
     */
    getMigrationPath(standardId: EntityId, fromVersion: SemanticVersion, toVersion: SemanticVersion): Promise<MigrationPath>;
    /**
     * Verify system compliance with a standard
     *
     * @param request - Compliance verification request
     * @returns Verification results
     */
    verifyCompliance(request: ComplianceVerificationRequest): Promise<ComplianceVerificationResult>;
    /**
     * Request official certification
     *
     * @param request - Certification request
     * @returns Certification status
     */
    requestCertification(request: CertificationRequest): Promise<CertificationResult>;
    /**
     * Get certification status
     *
     * @param requestId - Certification request ID
     * @returns Current certification status
     */
    getCertificationStatus(requestId: EntityId): Promise<CertificationResult>;
    /**
     * List all certifications for a system
     *
     * @param systemId - System identifier
     * @returns List of certifications
     */
    listCertifications(systemId: EntityId): Promise<{
        certifications: CertificationResult[];
    }>;
    /**
     * Generate integration template
     *
     * @param request - Template generation parameters
     * @returns Generated integration code
     */
    generateTemplate(request: TemplateGenerationRequest): Promise<IntegrationTemplate>;
    /**
     * Get existing template
     *
     * @param templateId - Template identifier
     * @returns Integration template
     */
    getTemplate(templateId: EntityId): Promise<IntegrationTemplate>;
    /**
     * List available templates
     *
     * @param filters - Optional filters
     * @returns List of templates
     */
    listTemplates(filters?: {
        language?: string;
        protocol?: string;
    }): Promise<{
        templates: IntegrationTemplate[];
        total: number;
    }>;
    /**
     * Get registry statistics
     *
     * @param options - Time range and filters
     * @returns Registry statistics
     */
    getStatistics(options?: {
        timeRange?: 'last-24h' | 'last-7d' | 'last-30d' | 'last-90d';
    }): Promise<RegistryStatistics>;
    /**
     * Get trending standards
     *
     * @param options - Metric and period
     * @returns List of trending standards
     */
    getTrendingStandards(options?: {
        metric?: 'adoption-rate' | 'growth-rate' | 'implementations';
        period?: 'last-week' | 'last-month' | 'last-quarter';
    }): Promise<{
        trending: TrendingStandard[];
    }>;
    /**
     * Get system quality score
     *
     * @param systemId - System identifier
     * @returns Quality score breakdown
     */
    getSystemQualityScore(systemId: EntityId): Promise<SystemQualityScore>;
    /**
     * Validate system registration data
     *
     * @param registration - System registration to validate
     * @returns Validation errors (empty if valid)
     */
    validateRegistration(registration: Partial<SystemRegistration>): {
        valid: boolean;
        errors: string[];
    };
    /**
     * Compare compliance levels
     *
     * @param level1 - First compliance level
     * @param level2 - Second compliance level
     * @returns -1 if level1 < level2, 0 if equal, 1 if level1 > level2
     */
    compareComplianceLevels(level1: string, level2: string): number;
}
/**
 * Create a registry client instance
 *
 * @param config - Registry configuration
 * @returns Registry client
 */
declare function createRegistry(config: RegistryConfig): InteroperabilityRegistry;
/**
 * Quick search for systems by standard
 *
 * @param standardId - Standard ID to search for
 * @param config - Registry configuration
 * @returns Systems implementing the standard
 */
declare function discoverCompatibleSystems(standardId: EntityId, config: RegistryConfig): Promise<DiscoveryResponse>;

export { CertificationRequest, CertificationResult, CompatibilityResult, ComplianceVerificationRequest, ComplianceVerificationResult, DiscoveryQuery, DiscoveryResponse, EntityId, IntegrationRecommendation, IntegrationTemplate, InteroperabilityRegistry, MigrationPath, RegistryConfig, RegistryStatistics, SemanticVersion, StandardDefinition, StandardVersionHistory, SystemQualityScore, SystemRegistration, TemplateGenerationRequest, TrendingStandard, createRegistry, discoverCompatibleSystems };
