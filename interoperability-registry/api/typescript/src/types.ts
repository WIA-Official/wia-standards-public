/**
 * WIA-CORE-004: Interoperability Registry - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Core Standards Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Unique identifier for systems, standards, and other entities
 */
export type EntityId = string;

/**
 * Semantic version string (MAJOR.MINOR.PATCH)
 */
export type SemanticVersion = string;

/**
 * ISO 8601 date-time string
 */
export type DateTime = string;

/**
 * URL string
 */
export type Url = string;

/**
 * Email address string
 */
export type Email = string;

// ============================================================================
// Standard Definitions
// ============================================================================

/**
 * Standard categories
 */
export type StandardCategory =
  | 'CORE'
  | 'AAC'
  | 'AI'
  | 'MED'
  | 'BIO'
  | 'CRYO'
  | 'ENE'
  | 'SEMI'
  | 'ROB'
  | 'SPACE'
  | 'COMM'
  | 'SEC'
  | 'AUTO'
  | 'AGRI'
  | 'CITY'
  | 'FIN'
  | 'EDU'
  | 'MAT'
  | 'DEF'
  | 'SOC'
  | 'LEG'
  | 'PET'
  | 'QUA'
  | 'TIME'
  | 'UNI'
  | 'IND'
  | 'COMP'
  | 'DATA'
  | 'AUG';

/**
 * Standard status
 */
export type StandardStatus = 'draft' | 'active' | 'deprecated' | 'retired';

/**
 * Standard specification
 */
export interface StandardSpecification {
  url: Url;
  format: 'markdown' | 'pdf' | 'html';
  language: string;
}

/**
 * Standard scope
 */
export interface StandardScope {
  domains: string[];
  applicability: 'Universal' | 'Domain-Specific' | 'Organization-Specific';
}

/**
 * Standard requirements
 */
export interface StandardRequirements {
  mandatory: string[];
  optional: string[];
}

/**
 * Compliance levels
 */
export type ComplianceLevel =
  | 'documented'
  | 'testable'
  | 'compliant'
  | 'certified'
  | 'reference';

/**
 * Compliance configuration
 */
export interface ComplianceConfiguration {
  levels: ComplianceLevel[];
  testSuiteUrl?: Url;
  certificationUrl?: Url;
}

/**
 * Versioning configuration
 */
export interface VersioningConfiguration {
  scheme: 'semver' | 'calver' | 'custom';
  compatibility: 'backward' | 'forward' | 'none';
  deprecationPolicy: string;
}

/**
 * Standard dependency
 */
export interface StandardDependency {
  standardId: EntityId;
  version: string; // Can include operators: >=1.0.0, ~2.1.0, etc.
  required: boolean;
}

/**
 * Complete standard definition
 */
export interface StandardDefinition {
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

// ============================================================================
// System Registration
// ============================================================================

/**
 * Organization information
 */
export interface OrganizationInfo {
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
export interface StandardImplementation {
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
export type AuthenticationMethod =
  | 'OAuth2'
  | 'OIDC'
  | 'API-Key'
  | 'JWT'
  | 'Basic'
  | 'mTLS'
  | 'SAML';

/**
 * OAuth2 configuration
 */
export interface OAuth2Config {
  authorizationUrl: Url;
  tokenUrl: Url;
  scopes: string[];
  supportedGrants?: ('authorization_code' | 'client_credentials' | 'refresh_token')[];
}

/**
 * Endpoint authentication
 */
export interface EndpointAuthentication {
  methods: AuthenticationMethod[];
  oauth2?: OAuth2Config;
  apiKeyHeader?: string;
  jwtIssuer?: string;
}

/**
 * API documentation
 */
export interface ApiDocumentation {
  openapi?: Url;
  asyncapi?: Url;
  graphql?: Url;
  postman?: Url;
  swagger?: Url;
}

/**
 * Rate limiting configuration
 */
export interface RateLimit {
  requests: number;
  period: 'second' | 'minute' | 'hour' | 'day' | 'month';
  burstAllowance?: number;
}

/**
 * Endpoint availability
 */
export interface EndpointAvailability {
  sla: number; // Percentage (99.9 = 99.9%)
  regions: string[];
  healthCheck?: Url;
  statusPage?: Url;
}

/**
 * Endpoint types
 */
export type EndpointType =
  | 'REST'
  | 'GraphQL'
  | 'gRPC'
  | 'WebSocket'
  | 'MQTT'
  | 'AMQP'
  | 'Kafka'
  | 'SSE'; // Server-Sent Events

/**
 * System endpoint
 */
export interface SystemEndpoint {
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
export interface SystemCapabilities {
  features: string[];
  protocols: string[];
  dataFormats: string[];
  authentication: AuthenticationMethod[];
  compliance: Record<string, boolean>;
  performance?: {
    maxThroughput?: number; // requests per second
    avgLatency?: number; // milliseconds
    p99Latency?: number; // milliseconds
  };
}

/**
 * Data residency
 */
export interface DataResidency {
  regions: string[];
  customersControlLocation: boolean;
  sovereignCloud?: boolean;
}

/**
 * System visibility
 */
export type SystemVisibility = 'public' | 'private' | 'organization';

/**
 * System metadata
 */
export interface SystemMetadata {
  registeredAt: DateTime;
  updatedAt: DateTime;
  visibility: SystemVisibility;
  tags: string[];
}

/**
 * Complete system registration
 */
export interface SystemRegistration {
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

// ============================================================================
// Discovery
// ============================================================================

/**
 * Standard filter
 */
export interface StandardFilter {
  id: EntityId;
  version?: SemanticVersion;
  minComplianceLevel?: ComplianceLevel;
}

/**
 * Capability filter
 */
export interface CapabilityFilter {
  features?: string[];
  protocols?: string[];
  dataFormats?: string[];
  authMethods?: AuthenticationMethod[];
  compliance?: Record<string, boolean>;
}

/**
 * Organization filter
 */
export interface OrganizationFilter {
  name?: string;
  verified?: boolean;
}

/**
 * Geographic filter
 */
export interface GeographicFilter {
  regions?: string[];
  dataResidency?: string[];
}

/**
 * Performance filter
 */
export interface PerformanceFilter {
  minAvailability?: number;
  maxLatency?: number;
  minThroughput?: number;
}

/**
 * Sort options
 */
export type SortBy = 'relevance' | 'popularity' | 'compliance' | 'updated' | 'name';
export type SortOrder = 'asc' | 'desc';

/**
 * Discovery query
 */
export interface DiscoveryQuery {
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
export interface MatchedCapabilities {
  features: string[];
  protocols: string[];
  compliance: Record<string, boolean>;
}

/**
 * Compatibility details
 */
export interface CompatibilityDetails {
  protocolMatch: boolean;
  dataFormatMatch: boolean;
  authenticationMatch: boolean;
  complianceMatch: boolean;
  versionCompatible: boolean;
}

/**
 * Compatibility result
 */
export interface CompatibilityResult {
  score: number; // 0-1
  details: CompatibilityDetails;
}

/**
 * Discovery result item
 */
export interface DiscoveryResultItem {
  systemId: EntityId;
  name: string;
  organization: string;
  score: number; // Relevance score 0-1
  matchedStandards: EntityId[];
  matchedCapabilities: MatchedCapabilities;
  endpoints: SystemEndpoint[];
  compatibility: CompatibilityResult;
}

/**
 * Discovery response
 */
export interface DiscoveryResponse {
  query: DiscoveryQuery;
  results: DiscoveryResultItem[];
  total: number;
  page: number;
  pageSize: number;
  executionTime: number; // milliseconds
}

// ============================================================================
// Compliance Verification
// ============================================================================

/**
 * Test types
 */
export type TestType = 'functional' | 'performance' | 'security' | 'integration';

/**
 * Test severity
 */
export type TestSeverity = 'critical' | 'high' | 'medium' | 'low';

/**
 * Test status
 */
export type TestStatus = 'passed' | 'failed' | 'skipped' | 'error';

/**
 * Test result
 */
export interface TestResult {
  id: EntityId;
  name: string;
  type: TestType;
  severity: TestSeverity;
  status: TestStatus;
  message?: string;
  duration: number; // milliseconds
  evidence?: string;
}

/**
 * Compliance verification request
 */
export interface ComplianceVerificationRequest {
  systemId: EntityId;
  standardId: EntityId;
  version: SemanticVersion;
  runTests?: boolean;
}

/**
 * Compliance verification result
 */
export interface ComplianceVerificationResult {
  systemId: EntityId;
  standardId: EntityId;
  version: SemanticVersion;
  complianceLevel: ComplianceLevel;
  testResults?: TestResult[];
  overallStatus: 'pass' | 'fail' | 'partial';
  score: number; // 0-100
  verifiedAt: DateTime;
  expiresAt?: DateTime;
  recommendations?: string[];
}

/**
 * Certification request
 */
export interface CertificationRequest {
  systemId: EntityId;
  standardId: EntityId;
  version: SemanticVersion;
  contactEmail: Email;
  notes?: string;
}

/**
 * Certification status
 */
export type CertificationStatusType =
  | 'pending'
  | 'under-review'
  | 'approved'
  | 'rejected'
  | 'expired';

/**
 * Certification result
 */
export interface CertificationResult {
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

// ============================================================================
// Integration Templates
// ============================================================================

/**
 * Supported programming languages
 */
export type ProgrammingLanguage =
  | 'typescript'
  | 'javascript'
  | 'python'
  | 'java'
  | 'go'
  | 'csharp'
  | 'rust'
  | 'php'
  | 'ruby';

/**
 * Integration pattern
 */
export type IntegrationPattern = 'sync' | 'async' | 'batch' | 'stream' | 'event-driven';

/**
 * Package dependency
 */
export interface PackageDependency {
  name: string;
  version: string;
  optional?: boolean;
}

/**
 * Integration template
 */
export interface IntegrationTemplate {
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
export interface TemplateGenerationRequest {
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
export type IntegrationComplexity = 'trivial' | 'low' | 'medium' | 'high' | 'critical';

/**
 * Integration recommendation
 */
export interface IntegrationRecommendation {
  sourceSystemId: EntityId;
  targetSystemId: EntityId;
  compatibilityScore: number; // 0-100
  recommendedProtocol: string;
  recommendedDataFormat: string;
  integrationComplexity: IntegrationComplexity;
  estimatedEffortDays: number;
  requiredAdapters: string[];
  potentialIssues: string[];
  migrationPath?: string;
  bestPractices: string[];
}

// ============================================================================
// Version Management
// ============================================================================

/**
 * Version status
 */
export type VersionStatus = 'active' | 'deprecated' | 'retired';

/**
 * Version changes
 */
export interface VersionChanges {
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
export interface VersionInfo {
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
export interface StandardVersionHistory {
  standardId: EntityId;
  versions: VersionInfo[];
}

/**
 * Migration step
 */
export interface MigrationStep {
  order: number;
  title: string;
  description: string;
  automated: boolean;
  script?: string;
  estimatedTime?: number; // minutes
}

/**
 * Breaking change
 */
export interface BreakingChange {
  component: string;
  change: string;
  impact: string;
  resolution: string;
}

/**
 * Migration path
 */
export interface MigrationPath {
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

// ============================================================================
// Analytics
// ============================================================================

/**
 * Registry statistics
 */
export interface RegistryStatistics {
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
    averageIntegrationTime: number; // days
  };
  compliance: {
    certifiedSystems: number;
    complianceRate: number; // percentage
    averageTestScore: number;
  };
  performance: {
    avgSearchLatency: number; // ms
    avgRegistrationTime: number; // ms
    uptime: number; // percentage
  };
  timeRange?: {
    start: DateTime;
    end: DateTime;
  };
}

/**
 * Trending standard
 */
export interface TrendingStandard {
  standardId: EntityId;
  name: string;
  category: StandardCategory;
  adoptionRate: number; // percentage
  growthRate: number; // percentage change
  totalImplementations: number;
  newImplementations: number;
  trend: 'rising' | 'stable' | 'declining';
}

/**
 * System quality dimensions
 */
export interface QualityDimensions {
  documentation: number; // 0-100
  compliance: number; // 0-100
  availability: number; // 0-100
  performance: number; // 0-100
  security: number; // 0-100
  support: number; // 0-100
}

/**
 * System quality score
 */
export interface SystemQualityScore {
  systemId: EntityId;
  overallScore: number; // 0-100
  dimensions: QualityDimensions;
  trend: 'improving' | 'stable' | 'declining';
  lastUpdated: DateTime;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * Registry error codes
 */
export enum RegistryErrorCode {
  INVALID_REQUEST = 'INVALID_REQUEST',
  UNAUTHORIZED = 'UNAUTHORIZED',
  FORBIDDEN = 'FORBIDDEN',
  NOT_FOUND = 'NOT_FOUND',
  CONFLICT = 'CONFLICT',
  RATE_LIMIT_EXCEEDED = 'RATE_LIMIT_EXCEEDED',
  VALIDATION_ERROR = 'VALIDATION_ERROR',
  SYSTEM_ALREADY_REGISTERED = 'SYSTEM_ALREADY_REGISTERED',
  STANDARD_NOT_FOUND = 'STANDARD_NOT_FOUND',
  COMPLIANCE_CHECK_FAILED = 'COMPLIANCE_CHECK_FAILED',
  TEMPLATE_GENERATION_FAILED = 'TEMPLATE_GENERATION_FAILED',
  INTERNAL_ERROR = 'INTERNAL_ERROR',
}

/**
 * Registry error
 */
export class RegistryError extends Error {
  code: RegistryErrorCode;
  statusCode: number;
  details?: Record<string, unknown>;

  constructor(
    code: RegistryErrorCode,
    message: string,
    statusCode: number = 500,
    details?: Record<string, unknown>
  ) {
    super(message);
    this.code = code;
    this.statusCode = statusCode;
    this.details = details;
    this.name = 'RegistryError';
  }
}

// ============================================================================
// Configuration
// ============================================================================

/**
 * Registry client configuration
 */
export interface RegistryConfig {
  endpoint: Url;
  apiKey?: string;
  oauth?: {
    clientId: string;
    clientSecret: string;
    tokenUrl: Url;
  };
  timeout?: number; // milliseconds
  retries?: number;
  cache?: {
    enabled: boolean;
    ttl: number; // seconds
  };
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Registry constants
 */
export const REGISTRY_CONSTANTS = {
  // Default registry endpoint
  DEFAULT_ENDPOINT: 'https://registry.wiastandards.com',

  // API version
  API_VERSION: 'v1',

  // Default pagination
  DEFAULT_PAGE_SIZE: 10,
  MAX_PAGE_SIZE: 100,

  // Compliance levels in order
  COMPLIANCE_LEVELS: [
    'documented',
    'testable',
    'compliant',
    'certified',
    'reference',
  ] as ComplianceLevel[],

  // Standard categories
  CATEGORIES: {
    CORE: { name: 'Core', color: '#6366F1', emoji: '🔷' },
    AAC: { name: 'Accessibility', color: '#3B82F6', emoji: '♿' },
    AI: { name: 'Artificial Intelligence', color: '#10B981', emoji: '🤖' },
    MED: { name: 'Medical', color: '#14B8A6', emoji: '🏥' },
    BIO: { name: 'Biotechnology', color: '#14B8A6', emoji: '🧬' },
    DEF: { name: 'Defense', color: '#64748B', emoji: '🛡️' },
    // ... add more as needed
  },

  // Timeout values (milliseconds)
  TIMEOUTS: {
    DEFAULT: 30000,
    SEARCH: 10000,
    REGISTRATION: 60000,
    COMPLIANCE: 120000,
  },
} as const;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core exports are defined inline above
};
