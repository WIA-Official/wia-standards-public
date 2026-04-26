/**
 * WIA-CORE-004: Interoperability Registry - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Core Standards Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 */
/**
 * Unique identifier for systems, standards, and other entities
 */
type EntityId = string;
/**
 * Semantic version string (MAJOR.MINOR.PATCH)
 */
type SemanticVersion = string;
/**
 * ISO 8601 date-time string
 */
type DateTime = string;
/**
 * URL string
 */
type Url = string;
/**
 * Email address string
 */
type Email = string;
/**
 * Standard categories
 */
type StandardCategory = 'CORE' | 'AAC' | 'AI' | 'MED' | 'BIO' | 'CRYO' | 'ENE' | 'SEMI' | 'ROB' | 'SPACE' | 'COMM' | 'SEC' | 'AUTO' | 'AGRI' | 'CITY' | 'FIN' | 'EDU' | 'MAT' | 'DEF' | 'SOC' | 'LEG' | 'PET' | 'QUA' | 'TIME' | 'UNI' | 'IND' | 'COMP' | 'DATA' | 'AUG';
/**
 * Standard status
 */
type StandardStatus = 'draft' | 'active' | 'deprecated' | 'retired';
/**
 * Standard specification
 */
interface StandardSpecification {
    url: Url;
    format: 'markdown' | 'pdf' | 'html';
    language: string;
}
/**
 * Standard scope
 */
interface StandardScope {
    domains: string[];
    applicability: 'Universal' | 'Domain-Specific' | 'Organization-Specific';
}
/**
 * Standard requirements
 */
interface StandardRequirements {
    mandatory: string[];
    optional: string[];
}
/**
 * Compliance levels
 */
type ComplianceLevel = 'documented' | 'testable' | 'compliant' | 'certified' | 'reference';
/**
 * Compliance configuration
 */
interface ComplianceConfiguration {
    levels: ComplianceLevel[];
    testSuiteUrl?: Url;
    certificationUrl?: Url;
}
/**
 * Versioning configuration
 */
interface VersioningConfiguration {
    scheme: 'semver' | 'calver' | 'custom';
    compatibility: 'backward' | 'forward' | 'none';
    deprecationPolicy: string;
}
/**
 * Standard dependency
 */
interface StandardDependency {
    standardId: EntityId;
    version: string;
    required: boolean;
}
/**
 * Complete standard definition
 */
interface StandardDefinition {
    id: EntityId;
    name: string;
    version: SemanticVersion;
    category: StandardCategory;
    status: StandardStatus;
    publishedDate: DateTime;
    authors: string[];
    description: string;
    specification: StandardSpecification;
    scope: StandardScope;
    requirements: StandardRequirements;
    compliance: ComplianceConfiguration;
    versioning: VersioningConfiguration;
    dependencies: StandardDependency[];
    related: EntityId[];
    tags: string[];
}
/**
 * Organization information
 */
interface OrganizationInfo {
    name: string;
    url?: Url;
    verified?: boolean;
    contact: {
        email: Email;
        support?: Url;
        slack?: Url;
        discord?: Url;
    };
}
/**
 * Standard implementation
 */
interface StandardImplementation {
    standardId: EntityId;
    version: SemanticVersion;
    complianceLevel: ComplianceLevel;
    certificationDate?: DateTime;
    certificationId?: string;
    expirationDate?: DateTime;
}
/**
 * Authentication methods
 */
type AuthenticationMethod = 'OAuth2' | 'OIDC' | 'API-Key' | 'JWT' | 'Basic' | 'mTLS' | 'SAML';
/**
 * OAuth2 configuration
 */
interface OAuth2Config {
    authorizationUrl: Url;
    tokenUrl: Url;
    scopes: string[];
    supportedGrants?: ('authorization_code' | 'client_credentials' | 'refresh_token')[];
}
/**
 * Endpoint authentication
 */
interface EndpointAuthentication {
    methods: AuthenticationMethod[];
    oauth2?: OAuth2Config;
    apiKeyHeader?: string;
    jwtIssuer?: string;
}
/**
 * API documentation
 */
interface ApiDocumentation {
    openapi?: Url;
    asyncapi?: Url;
    graphql?: Url;
    postman?: Url;
    swagger?: Url;
}
/**
 * Rate limiting configuration
 */
interface RateLimit {
    requests: number;
    period: 'second' | 'minute' | 'hour' | 'day' | 'month';
    burstAllowance?: number;
}
/**
 * Endpoint availability
 */
interface EndpointAvailability {
    sla: number;
    regions: string[];
    healthCheck?: Url;
    statusPage?: Url;
}
/**
 * Endpoint types
 */
type EndpointType = 'REST' | 'GraphQL' | 'gRPC' | 'WebSocket' | 'MQTT' | 'AMQP' | 'Kafka' | 'SSE';
/**
 * System endpoint
 */
interface SystemEndpoint {
    id: EntityId;
    type: EndpointType;
    url: Url;
    protocol: 'http' | 'https' | 'ws' | 'wss' | 'tcp' | 'udp';
    authentication: EndpointAuthentication;
    documentation: ApiDocumentation;
    rateLimit?: RateLimit;
    availability: EndpointAvailability;
}
/**
 * System capabilities
 */
interface SystemCapabilities {
    features: string[];
    protocols: string[];
    dataFormats: string[];
    authentication: AuthenticationMethod[];
    compliance: Record<string, boolean>;
    performance?: {
        maxThroughput?: number;
        avgLatency?: number;
        p99Latency?: number;
    };
}
/**
 * Data residency
 */
interface DataResidency {
    regions: string[];
    customersControlLocation: boolean;
    sovereignCloud?: boolean;
}
/**
 * System visibility
 */
type SystemVisibility = 'public' | 'private' | 'organization';
/**
 * System metadata
 */
interface SystemMetadata {
    registeredAt: DateTime;
    updatedAt: DateTime;
    visibility: SystemVisibility;
    tags: string[];
}
/**
 * Complete system registration
 */
interface SystemRegistration {
    systemId: EntityId;
    name: string;
    version: SemanticVersion;
    organization: OrganizationInfo;
    description: string;
    standards: StandardImplementation[];
    endpoints: SystemEndpoint[];
    capabilities: SystemCapabilities;
    dataResidency: DataResidency;
    metadata: SystemMetadata;
}
/**
 * Standard filter
 */
interface StandardFilter {
    id: EntityId;
    version?: SemanticVersion;
    minComplianceLevel?: ComplianceLevel;
}
/**
 * Capability filter
 */
interface CapabilityFilter {
    features?: string[];
    protocols?: string[];
    dataFormats?: string[];
    authMethods?: AuthenticationMethod[];
    compliance?: Record<string, boolean>;
}
/**
 * Organization filter
 */
interface OrganizationFilter {
    name?: string;
    verified?: boolean;
}
/**
 * Geographic filter
 */
interface GeographicFilter {
    regions?: string[];
    dataResidency?: string[];
}
/**
 * Performance filter
 */
interface PerformanceFilter {
    minAvailability?: number;
    maxLatency?: number;
    minThroughput?: number;
}
/**
 * Sort options
 */
type SortBy = 'relevance' | 'popularity' | 'compliance' | 'updated' | 'name';
type SortOrder = 'asc' | 'desc';
/**
 * Discovery query
 */
interface DiscoveryQuery {
    standards?: StandardFilter[];
    capabilities?: CapabilityFilter;
    organization?: OrganizationFilter;
    geography?: GeographicFilter;
    performance?: PerformanceFilter;
    limit?: number;
    offset?: number;
    sortBy?: SortBy;
    sortOrder?: SortOrder;
}
/**
 * Matched capabilities
 */
interface MatchedCapabilities {
    features: string[];
    protocols: string[];
    compliance: Record<string, boolean>;
}
/**
 * Compatibility details
 */
interface CompatibilityDetails {
    protocolMatch: boolean;
    dataFormatMatch: boolean;
    authenticationMatch: boolean;
    complianceMatch: boolean;
    versionCompatible: boolean;
}
/**
 * Compatibility result
 */
interface CompatibilityResult {
    score: number;
    details: CompatibilityDetails;
}
/**
 * Discovery result item
 */
interface DiscoveryResultItem {
    systemId: EntityId;
    name: string;
    organization: string;
    score: number;
    matchedStandards: EntityId[];
    matchedCapabilities: MatchedCapabilities;
    endpoints: SystemEndpoint[];
    compatibility: CompatibilityResult;
}
/**
 * Discovery response
 */
interface DiscoveryResponse {
    query: DiscoveryQuery;
    results: DiscoveryResultItem[];
    total: number;
    page: number;
    pageSize: number;
    executionTime: number;
}
/**
 * Test types
 */
type TestType = 'functional' | 'performance' | 'security' | 'integration';
/**
 * Test severity
 */
type TestSeverity = 'critical' | 'high' | 'medium' | 'low';
/**
 * Test status
 */
type TestStatus = 'passed' | 'failed' | 'skipped' | 'error';
/**
 * Test result
 */
interface TestResult {
    id: EntityId;
    name: string;
    type: TestType;
    severity: TestSeverity;
    status: TestStatus;
    message?: string;
    duration: number;
    evidence?: string;
}
/**
 * Compliance verification request
 */
interface ComplianceVerificationRequest {
    systemId: EntityId;
    standardId: EntityId;
    version: SemanticVersion;
    runTests?: boolean;
}
/**
 * Compliance verification result
 */
interface ComplianceVerificationResult {
    systemId: EntityId;
    standardId: EntityId;
    version: SemanticVersion;
    complianceLevel: ComplianceLevel;
    testResults?: TestResult[];
    overallStatus: 'pass' | 'fail' | 'partial';
    score: number;
    verifiedAt: DateTime;
    expiresAt?: DateTime;
    recommendations?: string[];
}
/**
 * Certification request
 */
interface CertificationRequest {
    systemId: EntityId;
    standardId: EntityId;
    version: SemanticVersion;
    contactEmail: Email;
    notes?: string;
}
/**
 * Certification status
 */
type CertificationStatusType = 'pending' | 'under-review' | 'approved' | 'rejected' | 'expired';
/**
 * Certification result
 */
interface CertificationResult {
    requestId: EntityId;
    status: CertificationStatusType;
    systemId: EntityId;
    standardId: EntityId;
    certificationId?: string;
    issuedAt?: DateTime;
    expiresAt?: DateTime;
    reviewer?: string;
    notes?: string;
}
/**
 * Supported programming languages
 */
type ProgrammingLanguage = 'typescript' | 'javascript' | 'python' | 'java' | 'go' | 'csharp' | 'rust' | 'php' | 'ruby';
/**
 * Integration pattern
 */
type IntegrationPattern = 'sync' | 'async' | 'batch' | 'stream' | 'event-driven';
/**
 * Package dependency
 */
interface PackageDependency {
    name: string;
    version: string;
    optional?: boolean;
}
/**
 * Integration template
 */
interface IntegrationTemplate {
    id: EntityId;
    name: string;
    sourceSystem: EntityId;
    targetSystem: EntityId;
    language: ProgrammingLanguage;
    framework?: string;
    protocol: string;
    pattern: IntegrationPattern;
    code: string;
    dependencies: PackageDependency[];
    configuration: Record<string, any>;
    documentation: string;
    examples: string[];
    generatedAt: DateTime;
}
/**
 * Template generation request
 */
interface TemplateGenerationRequest {
    sourceSystem: EntityId;
    targetSystem: EntityId;
    language: ProgrammingLanguage;
    framework?: string;
    protocol?: string;
    includeErrorHandling?: boolean;
    includeRetryLogic?: boolean;
    includeRateLimiting?: boolean;
    includeLogging?: boolean;
}
/**
 * Integration complexity
 */
type IntegrationComplexity = 'trivial' | 'low' | 'medium' | 'high' | 'critical';
/**
 * Integration recommendation
 */
interface IntegrationRecommendation {
    sourceSystemId: EntityId;
    targetSystemId: EntityId;
    compatibilityScore: number;
    recommendedProtocol: string;
    recommendedDataFormat: string;
    integrationComplexity: IntegrationComplexity;
    estimatedEffortDays: number;
    requiredAdapters: string[];
    potentialIssues: string[];
    migrationPath?: string;
    bestPractices: string[];
}
/**
 * Version status
 */
type VersionStatus = 'active' | 'deprecated' | 'retired';
/**
 * Version changes
 */
interface VersionChanges {
    added: string[];
    changed: string[];
    deprecated: string[];
    removed: string[];
    fixed?: string[];
    security?: string[];
}
/**
 * Version information
 */
interface VersionInfo {
    version: SemanticVersion;
    status: VersionStatus;
    releaseDate: DateTime;
    endOfLifeDate?: DateTime;
    breaking: boolean;
    changes: VersionChanges;
    migrationGuide?: Url;
    backwardCompatible: boolean;
    forwardCompatible: boolean;
}
/**
 * Standard version history
 */
interface StandardVersionHistory {
    standardId: EntityId;
    versions: VersionInfo[];
}
/**
 * Migration step
 */
interface MigrationStep {
    order: number;
    title: string;
    description: string;
    automated: boolean;
    script?: string;
    estimatedTime?: number;
}
/**
 * Breaking change
 */
interface BreakingChange {
    component: string;
    change: string;
    impact: string;
    resolution: string;
}
/**
 * Migration path
 */
interface MigrationPath {
    fromVersion: SemanticVersion;
    toVersion: SemanticVersion;
    complexity: IntegrationComplexity;
    steps: MigrationStep[];
    breakingChanges: BreakingChange[];
    estimatedEffort: {
        hours: number;
        difficulty: IntegrationComplexity;
    };
    automationAvailable: boolean;
}
/**
 * Registry statistics
 */
interface RegistryStatistics {
    systems: {
        total: number;
        active: number;
        certified: number;
        byCategory: Record<StandardCategory, number>;
        byRegion: Record<string, number>;
    };
    standards: {
        total: number;
        active: number;
        deprecated: number;
        byCategory: Record<StandardCategory, number>;
    };
    integrations: {
        totalTemplatesGenerated: number;
        successfulIntegrations: number;
        averageIntegrationTime: number;
    };
    compliance: {
        certifiedSystems: number;
        complianceRate: number;
        averageTestScore: number;
    };
    performance: {
        avgSearchLatency: number;
        avgRegistrationTime: number;
        uptime: number;
    };
    timeRange?: {
        start: DateTime;
        end: DateTime;
    };
}
/**
 * Trending standard
 */
interface TrendingStandard {
    standardId: EntityId;
    name: string;
    category: StandardCategory;
    adoptionRate: number;
    growthRate: number;
    totalImplementations: number;
    newImplementations: number;
    trend: 'rising' | 'stable' | 'declining';
}
/**
 * System quality dimensions
 */
interface QualityDimensions {
    documentation: number;
    compliance: number;
    availability: number;
    performance: number;
    security: number;
    support: number;
}
/**
 * System quality score
 */
interface SystemQualityScore {
    systemId: EntityId;
    overallScore: number;
    dimensions: QualityDimensions;
    trend: 'improving' | 'stable' | 'declining';
    lastUpdated: DateTime;
}
/**
 * Registry error codes
 */
declare enum RegistryErrorCode {
    INVALID_REQUEST = "INVALID_REQUEST",
    UNAUTHORIZED = "UNAUTHORIZED",
    FORBIDDEN = "FORBIDDEN",
    NOT_FOUND = "NOT_FOUND",
    CONFLICT = "CONFLICT",
    RATE_LIMIT_EXCEEDED = "RATE_LIMIT_EXCEEDED",
    VALIDATION_ERROR = "VALIDATION_ERROR",
    SYSTEM_ALREADY_REGISTERED = "SYSTEM_ALREADY_REGISTERED",
    STANDARD_NOT_FOUND = "STANDARD_NOT_FOUND",
    COMPLIANCE_CHECK_FAILED = "COMPLIANCE_CHECK_FAILED",
    TEMPLATE_GENERATION_FAILED = "TEMPLATE_GENERATION_FAILED",
    INTERNAL_ERROR = "INTERNAL_ERROR"
}
/**
 * Registry error
 */
declare class RegistryError extends Error {
    code: RegistryErrorCode;
    statusCode: number;
    details?: Record<string, unknown>;
    constructor(code: RegistryErrorCode, message: string, statusCode?: number, details?: Record<string, unknown>);
}
/**
 * Registry client configuration
 */
interface RegistryConfig {
    endpoint: Url;
    apiKey?: string;
    oauth?: {
        clientId: string;
        clientSecret: string;
        tokenUrl: Url;
    };
    timeout?: number;
    retries?: number;
    cache?: {
        enabled: boolean;
        ttl: number;
    };
}
/**
 * Registry constants
 */
declare const REGISTRY_CONSTANTS: {
    readonly DEFAULT_ENDPOINT: "https://registry.wiastandards.com";
    readonly API_VERSION: "v1";
    readonly DEFAULT_PAGE_SIZE: 10;
    readonly MAX_PAGE_SIZE: 100;
    readonly COMPLIANCE_LEVELS: ComplianceLevel[];
    readonly CATEGORIES: {
        readonly CORE: {
            readonly name: "Core";
            readonly color: "#6366F1";
            readonly emoji: "🔷";
        };
        readonly AAC: {
            readonly name: "Accessibility";
            readonly color: "#3B82F6";
            readonly emoji: "♿";
        };
        readonly AI: {
            readonly name: "Artificial Intelligence";
            readonly color: "#10B981";
            readonly emoji: "🤖";
        };
        readonly MED: {
            readonly name: "Medical";
            readonly color: "#14B8A6";
            readonly emoji: "🏥";
        };
        readonly BIO: {
            readonly name: "Biotechnology";
            readonly color: "#14B8A6";
            readonly emoji: "🧬";
        };
        readonly DEF: {
            readonly name: "Defense";
            readonly color: "#64748B";
            readonly emoji: "🛡️";
        };
    };
    readonly TIMEOUTS: {
        readonly DEFAULT: 30000;
        readonly SEARCH: 10000;
        readonly REGISTRATION: 60000;
        readonly COMPLIANCE: 120000;
    };
};

export { type ApiDocumentation, type AuthenticationMethod, type BreakingChange, type CapabilityFilter, type CertificationRequest, type CertificationResult, type CertificationStatusType, type CompatibilityDetails, type CompatibilityResult, type ComplianceConfiguration, type ComplianceLevel, type ComplianceVerificationRequest, type ComplianceVerificationResult, type DataResidency, type DateTime, type DiscoveryQuery, type DiscoveryResponse, type DiscoveryResultItem, type Email, type EndpointAuthentication, type EndpointAvailability, type EndpointType, type EntityId, type GeographicFilter, type IntegrationComplexity, type IntegrationPattern, type IntegrationRecommendation, type IntegrationTemplate, type MatchedCapabilities, type MigrationPath, type MigrationStep, type OAuth2Config, type OrganizationFilter, type OrganizationInfo, type PackageDependency, type PerformanceFilter, type ProgrammingLanguage, type QualityDimensions, REGISTRY_CONSTANTS, type RateLimit, type RegistryConfig, RegistryError, RegistryErrorCode, type RegistryStatistics, type SemanticVersion, type SortBy, type SortOrder, type StandardCategory, type StandardDefinition, type StandardDependency, type StandardFilter, type StandardImplementation, type StandardRequirements, type StandardScope, type StandardSpecification, type StandardStatus, type StandardVersionHistory, type SystemCapabilities, type SystemEndpoint, type SystemMetadata, type SystemQualityScore, type SystemRegistration, type SystemVisibility, type TemplateGenerationRequest, type TestResult, type TestSeverity, type TestStatus, type TestType, type TrendingStandard, type Url, type VersionChanges, type VersionInfo, type VersionStatus, type VersioningConfiguration };
