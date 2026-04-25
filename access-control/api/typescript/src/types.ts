/**
 * WIA Access Control Standard - Type Definitions
 *
 * @packageDocumentation
 * @module wia-access-control
 */

/**
 * Access control model type
 */
export enum AccessControlModel {
  /** Role-Based Access Control */
  RBAC = 'rbac',
  /** Attribute-Based Access Control */
  ABAC = 'abac',
  /** Discretionary Access Control */
  DAC = 'dac',
  /** Mandatory Access Control */
  MAC = 'mac',
  /** Rule-Based Access Control */
  RuleBAC = 'rule_bac',
  /** Hybrid model */
  Hybrid = 'hybrid'
}

/**
 * Permission action types
 */
export enum PermissionAction {
  Create = 'create',
  Read = 'read',
  Update = 'update',
  Delete = 'delete',
  Execute = 'execute',
  List = 'list',
  Manage = 'manage',
  Admin = 'admin'
}

/**
 * Resource type
 */
export enum ResourceType {
  File = 'file',
  Directory = 'directory',
  API = 'api',
  Database = 'database',
  Service = 'service',
  User = 'user',
  System = 'system',
  Custom = 'custom'
}

/**
 * Authentication method
 */
export enum AuthMethod {
  Password = 'password',
  OAuth2 = 'oauth2',
  SAML = 'saml',
  OIDC = 'oidc',
  MFA = 'mfa',
  Biometric = 'biometric',
  Certificate = 'certificate',
  APIKey = 'api_key',
  JWT = 'jwt'
}

/**
 * User identity
 */
export interface Identity {
  /** Unique user identifier */
  id: string;
  /** Username */
  username: string;
  /** Email address */
  email?: string;
  /** Display name */
  displayName?: string;
  /** Authentication method used */
  authMethod: AuthMethod;
  /** Identity provider */
  provider?: string;
  /** Identity attributes */
  attributes: Record<string, unknown>;
  /** Last authentication time */
  lastAuthAt?: Date;
  /** Identity creation time */
  createdAt: Date;
}

/**
 * Role definition
 */
export interface Role {
  /** Role identifier */
  id: string;
  /** Role name */
  name: string;
  /** Role description */
  description?: string;
  /** Permissions assigned to role */
  permissions: Permission[];
  /** Parent role for inheritance */
  parentRole?: string;
  /** Role priority (for conflict resolution) */
  priority: number;
  /** Role metadata */
  metadata?: Record<string, unknown>;
  /** Is system role */
  isSystem: boolean;
}

/**
 * Permission definition
 */
export interface Permission {
  /** Permission identifier */
  id: string;
  /** Permission name */
  name: string;
  /** Allowed actions */
  actions: PermissionAction[];
  /** Resource pattern (glob-style) */
  resource: string;
  /** Resource type */
  resourceType: ResourceType;
  /** Conditions for permission */
  conditions?: Condition[];
  /** Permission effect */
  effect: 'allow' | 'deny';
}

/**
 * Condition for ABAC
 */
export interface Condition {
  /** Condition field */
  field: string;
  /** Comparison operator */
  operator: ConditionOperator;
  /** Expected value */
  value: unknown;
}

/**
 * Condition operators
 */
export enum ConditionOperator {
  Equals = 'eq',
  NotEquals = 'neq',
  GreaterThan = 'gt',
  LessThan = 'lt',
  GreaterThanOrEqual = 'gte',
  LessThanOrEqual = 'lte',
  Contains = 'contains',
  StartsWith = 'starts_with',
  EndsWith = 'ends_with',
  In = 'in',
  NotIn = 'not_in',
  Exists = 'exists',
  NotExists = 'not_exists'
}

/**
 * Access request
 */
export interface AccessRequest {
  /** Request identifier */
  requestId: string;
  /** Subject (user or service) */
  subject: Subject;
  /** Action being performed */
  action: PermissionAction;
  /** Resource being accessed */
  resource: Resource;
  /** Request context */
  context: RequestContext;
  /** Request timestamp */
  timestamp: Date;
}

/**
 * Subject (who is requesting access)
 */
export interface Subject {
  /** Subject identifier */
  id: string;
  /** Subject type */
  type: 'user' | 'service' | 'application';
  /** Assigned roles */
  roles: string[];
  /** Subject attributes */
  attributes: Record<string, unknown>;
}

/**
 * Resource being accessed
 */
export interface Resource {
  /** Resource identifier */
  id: string;
  /** Resource type */
  type: ResourceType;
  /** Resource path or URI */
  path: string;
  /** Resource owner */
  owner?: string;
  /** Resource attributes */
  attributes: Record<string, unknown>;
}

/**
 * Request context
 */
export interface RequestContext {
  /** Client IP address */
  ipAddress?: string;
  /** User agent */
  userAgent?: string;
  /** Request location */
  location?: GeoLocation;
  /** Request time */
  time: Date;
  /** Device information */
  device?: DeviceInfo;
  /** Session information */
  session?: SessionInfo;
  /** Additional context */
  extra?: Record<string, unknown>;
}

/**
 * Geographic location
 */
export interface GeoLocation {
  /** Country code */
  country?: string;
  /** Region/State */
  region?: string;
  /** City */
  city?: string;
  /** Latitude */
  latitude?: number;
  /** Longitude */
  longitude?: number;
}

/**
 * Device information
 */
export interface DeviceInfo {
  /** Device identifier */
  deviceId: string;
  /** Device type */
  type: 'desktop' | 'mobile' | 'tablet' | 'iot' | 'unknown';
  /** Operating system */
  os?: string;
  /** Browser */
  browser?: string;
  /** Is trusted device */
  trusted: boolean;
}

/**
 * Session information
 */
export interface SessionInfo {
  /** Session identifier */
  sessionId: string;
  /** Session start time */
  startedAt: Date;
  /** Last activity time */
  lastActivityAt: Date;
  /** Session expiry */
  expiresAt: Date;
  /** MFA verified */
  mfaVerified: boolean;
}

/**
 * Access decision
 */
export interface AccessDecision {
  /** Request ID */
  requestId: string;
  /** Decision result */
  decision: 'allow' | 'deny' | 'indeterminate' | 'not_applicable';
  /** Reason for decision */
  reason?: string;
  /** Matching policies */
  matchingPolicies: string[];
  /** Obligations to fulfill */
  obligations?: Obligation[];
  /** Decision timestamp */
  timestamp: Date;
}

/**
 * Obligation (actions to perform)
 */
export interface Obligation {
  /** Obligation identifier */
  id: string;
  /** Obligation type */
  type: 'log' | 'notify' | 'encrypt' | 'mask' | 'custom';
  /** Obligation parameters */
  parameters: Record<string, unknown>;
}

/**
 * Policy definition
 */
export interface Policy {
  /** Policy identifier */
  id: string;
  /** Policy name */
  name: string;
  /** Policy description */
  description?: string;
  /** Policy version */
  version: string;
  /** Target subjects */
  subjects: PolicySubject[];
  /** Target resources */
  resources: PolicyResource[];
  /** Actions covered */
  actions: PermissionAction[];
  /** Policy effect */
  effect: 'allow' | 'deny';
  /** Policy conditions */
  conditions?: Condition[];
  /** Policy priority */
  priority: number;
  /** Is policy enabled */
  enabled: boolean;
}

/**
 * Policy subject target
 */
export interface PolicySubject {
  /** Subject type */
  type: 'user' | 'role' | 'group' | 'any';
  /** Subject identifier pattern */
  pattern: string;
}

/**
 * Policy resource target
 */
export interface PolicyResource {
  /** Resource type */
  type: ResourceType;
  /** Resource pattern */
  pattern: string;
}

/**
 * Audit log entry
 */
export interface AuditLogEntry {
  /** Log entry identifier */
  id: string;
  /** Access request */
  request: AccessRequest;
  /** Access decision */
  decision: AccessDecision;
  /** Log timestamp */
  timestamp: Date;
  /** Duration in milliseconds */
  duration: number;
}

/**
 * SDK configuration
 */
export interface AccessControlConfig {
  /** Access control model */
  model: AccessControlModel;
  /** Policy evaluation strategy */
  evaluationStrategy: 'first_match' | 'all_applicable' | 'priority';
  /** Default decision when no policy matches */
  defaultDecision: 'allow' | 'deny';
  /** Enable audit logging */
  auditEnabled: boolean;
  /** Cache TTL in seconds */
  cacheTTL: number;
  /** Policy storage backend */
  storageBackend?: 'memory' | 'redis' | 'database';
}

/**
 * Certification level
 */
export enum CertificationLevel {
  Bronze = 'bronze',
  Silver = 'silver',
  Gold = 'gold'
}

/**
 * Compliance report
 */
export interface ComplianceReport {
  /** Standard identifier */
  standard: 'WIA-ACCESS-CONTROL';
  /** Test date */
  testDate: string;
  /** Configuration */
  config: AccessControlConfig;
  /** Target certification level */
  targetLevel: CertificationLevel;
  /** Test results */
  tests: TestResult[];
  /** Overall pass/fail */
  passed: boolean;
  /** Achieved level */
  achievedLevel?: CertificationLevel;
}

/**
 * Test result
 */
export interface TestResult {
  /** Test name */
  testName: string;
  /** Pass/fail */
  passed: boolean;
  /** Notes */
  notes?: string;
}
