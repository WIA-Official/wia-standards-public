/**
 * WIA-IOT_SECURITY: TypeScript Type Definitions
 * Version: 1.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 *
 * Comprehensive type definitions for IoT security management,
 * device authentication, firmware updates, and security monitoring.
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Security severity levels for events and incidents
 */
export type SecuritySeverity = 'critical' | 'high' | 'medium' | 'low' | 'info';

/**
 * Security event categories
 */
export type SecurityCategory =
  | 'authentication'
  | 'authorization'
  | 'encryption'
  | 'network'
  | 'firmware'
  | 'physical'
  | 'configuration'
  | 'compliance';

/**
 * Device status enumeration
 */
export type DeviceStatus =
  | 'active'
  | 'inactive'
  | 'suspended'
  | 'revoked'
  | 'pending'
  | 'compromised'
  | 'quarantined';

/**
 * Credential types for device authentication
 */
export type CredentialType = 'certificate' | 'token' | 'key' | 'psk';

/**
 * Cryptographic algorithms
 */
export type CryptoAlgorithm =
  | 'RSA-2048'
  | 'RSA-4096'
  | 'ED25519'
  | 'ECDSA-P256'
  | 'ECDSA-P384'
  | 'AES-256-GCM'
  | 'CHACHA20-POLY1305';

// ============================================================================
// Device Identity and Registration
// ============================================================================

/**
 * Device identity information
 */
export interface DeviceIdentity {
  /** Unique device identifier (UUID v4) */
  deviceId: string;
  /** Device manufacturer name */
  manufacturer: string;
  /** Device model number */
  model: string;
  /** Unique serial number */
  serialNumber: string;
  /** Current firmware version */
  firmwareVersion: string;
  /** Hardware revision */
  hardwareVersion: string;
  /** Network MAC address */
  macAddress: string;
  /** X.509 certificate ID */
  certificateId?: string;
  /** Device metadata */
  metadata?: Record<string, any>;
}

/**
 * Device registration request
 */
export interface DeviceRegistration {
  /** Device manufacturer */
  manufacturer: string;
  /** Device model */
  model: string;
  /** Device serial number */
  serialNumber: string;
  /** Device MAC address */
  macAddress: string;
  /** Public key in PEM format */
  publicKey: string;
  /** Hardware attestation data (optional) */
  attestationData?: string;
  /** Additional metadata */
  metadata?: Record<string, any>;
}

/**
 * Device registration response
 */
export interface DeviceRegistrationResponse {
  /** Assigned device ID */
  deviceId: string;
  /** Registration status */
  status: 'registered' | 'pending' | 'rejected';
  /** Certificate ID */
  certificateId: string;
  /** X.509 certificate in PEM format */
  certificate: string;
  /** Certificate validity end date */
  validUntil: string;
  /** Registration timestamp */
  registeredAt: string;
}

// ============================================================================
// Authentication and Authorization
// ============================================================================

/**
 * Device authentication request
 */
export interface AuthenticationRequest {
  /** Device identifier */
  deviceId: string;
  /** Authentication challenge */
  challenge: string;
  /** Signature of challenge */
  signature: string;
  /** Request timestamp */
  timestamp: string;
  /** Authentication method */
  method?: 'certificate' | 'jwt' | 'hmac';
}

/**
 * Authentication response with tokens
 */
export interface AuthenticationResponse {
  /** JWT access token */
  accessToken: string;
  /** Refresh token */
  refreshToken: string;
  /** Token type (Bearer) */
  tokenType: 'Bearer';
  /** Token expiration in seconds */
  expiresIn: number;
  /** Granted scopes */
  scope: string[];
  /** Current device status */
  deviceStatus: DeviceStatus;
}

/**
 * Token refresh request
 */
export interface TokenRefreshRequest {
  /** Refresh token */
  refreshToken: string;
}

/**
 * Token refresh response
 */
export interface TokenRefreshResponse {
  /** New access token */
  accessToken: string;
  /** Token expiration in seconds */
  expiresIn: number;
}

/**
 * Access control permissions
 */
export interface AccessControl {
  /** Resource identifier */
  resourceId: string;
  /** Allowed operations */
  permissions: Array<'read' | 'write' | 'execute' | 'admin'>;
  /** Conditions for access */
  conditions?: {
    /** IP address whitelist */
    ipWhitelist?: string[];
    /** Time-based access */
    timeWindow?: {
      start: string;
      end: string;
    };
    /** Required security level */
    minSecurityLevel?: number;
  };
}

// ============================================================================
// Security Credentials
// ============================================================================

/**
 * Security credential information
 */
export interface SecurityCredential {
  /** Credential unique identifier */
  credentialId: string;
  /** Associated device ID */
  deviceId: string;
  /** Credential type */
  type: CredentialType;
  /** Cryptographic algorithm */
  algorithm: CryptoAlgorithm;
  /** Public key (PEM format) */
  publicKey: string;
  /** Certificate chain */
  certificateChain?: string[];
  /** Issuing authority */
  issuedBy: string;
  /** Validity start date */
  validFrom: string;
  /** Validity end date */
  validUntil: string;
  /** Credential status */
  status: 'active' | 'revoked' | 'expired' | 'pending';
  /** Creation timestamp */
  createdAt: string;
  /** Last updated timestamp */
  updatedAt?: string;
}

/**
 * Key rotation policy
 */
export interface KeyRotationPolicy {
  /** Enable automatic rotation */
  enabled: boolean;
  /** Rotation interval in days */
  intervalDays: number;
  /** Warning period before expiry (days) */
  warningDays: number;
  /** Automatic rotation flag */
  autoRotate: boolean;
  /** Overlap period for dual-key operation (seconds) */
  overlapPeriod?: number;
}

/**
 * Key storage configuration
 */
export interface KeyStorage {
  /** Storage type */
  type: 'TPM' | 'TEE' | 'HSM' | 'Software';
  /** Storage location/path */
  location: string;
  /** Encryption enabled */
  encrypted: boolean;
  /** Backup enabled */
  backupEnabled: boolean;
  /** Access control */
  accessControl: {
    /** Authorized applications */
    authorizedApps: string[];
    /** Require user authentication */
    requireAuth: boolean;
  };
}

// ============================================================================
// Firmware Management
// ============================================================================

/**
 * Firmware information
 */
export interface FirmwareInfo {
  /** Firmware version */
  version: string;
  /** Release date */
  releaseDate: string;
  /** Critical update flag */
  isCritical: boolean;
  /** Security fixes included */
  securityFixes: string[];
  /** Download URL */
  downloadUrl: string;
  /** SHA-256 checksum */
  checksum: string;
  /** Firmware size in bytes */
  size: number;
  /** Digital signature */
  signature: string;
  /** Release notes */
  releaseNotes?: string;
}

/**
 * Firmware update check request
 */
export interface FirmwareUpdateCheck {
  /** Device ID */
  deviceId: string;
  /** Current firmware version */
  currentVersion: string;
  /** Device model */
  model?: string;
  /** Update channel */
  channel?: 'stable' | 'beta' | 'dev';
}

/**
 * Firmware update check response
 */
export interface FirmwareUpdateResponse {
  /** Update available flag */
  updateAvailable: boolean;
  /** Latest firmware version */
  latestVersion: string;
  /** Current version */
  currentVersion: string;
  /** Firmware information */
  firmware?: FirmwareInfo;
}

/**
 * Firmware installation report
 */
export interface FirmwareInstallation {
  /** Device ID */
  deviceId: string;
  /** Previous firmware version */
  fromVersion: string;
  /** Installed firmware version */
  toVersion: string;
  /** Installation status */
  installStatus: 'success' | 'failed' | 'partial';
  /** Installation duration (seconds) */
  installDuration: number;
  /** Installation timestamp */
  timestamp: string;
  /** Verification hash */
  verificationHash: string;
  /** Error message (if failed) */
  errorMessage?: string;
}

/**
 * Firmware validation result
 */
export interface FirmwareValidation {
  /** Validation passed */
  valid: boolean;
  /** Signature verified */
  signatureVerified: boolean;
  /** Checksum verified */
  checksumVerified: boolean;
  /** Compatibility check */
  compatible: boolean;
  /** Validation errors */
  errors?: string[];
}

// ============================================================================
// Security Events and Monitoring
// ============================================================================

/**
 * Security event
 */
export interface SecurityEvent {
  /** Event unique identifier */
  eventId: string;
  /** Device identifier */
  deviceId: string;
  /** Event timestamp */
  timestamp: string;
  /** Event severity */
  severity: SecuritySeverity;
  /** Event category */
  category: SecurityCategory;
  /** Event type */
  eventType: string;
  /** Event description */
  description: string;
  /** Event source */
  source: string;
  /** Event destination (optional) */
  destination?: string;
  /** Additional metadata */
  metadata?: Record<string, any>;
  /** Event signature */
  signature?: string;
}

/**
 * Security alert configuration
 */
export interface SecurityAlert {
  /** Alert unique identifier */
  alertId: string;
  /** Alert name */
  name: string;
  /** Alert condition */
  condition: string;
  /** Alert severity */
  severity: SecuritySeverity;
  /** Alert enabled */
  enabled: boolean;
  /** Alert actions */
  actions: AlertAction[];
  /** Threshold configuration */
  threshold?: {
    /** Threshold value */
    value: number;
    /** Time window (seconds) */
    window: number;
  };
}

/**
 * Alert action configuration
 */
export interface AlertAction {
  /** Action type */
  type: 'email' | 'slack' | 'webhook' | 'sms' | 'auto-block' | 'ticket';
  /** Action configuration */
  config: Record<string, any>;
  /** Action priority */
  priority?: number;
}

/**
 * Monitoring metrics
 */
export interface MonitoringMetrics {
  /** Device ID */
  deviceId: string;
  /** Metrics timestamp */
  timestamp: string;
  /** CPU usage percentage */
  cpuUsage?: number;
  /** Memory usage percentage */
  memoryUsage?: number;
  /** Network bytes received */
  networkBytesIn?: number;
  /** Network bytes sent */
  networkBytesOut?: number;
  /** Failed authentication count */
  failedAuthCount?: number;
  /** Active connections */
  activeConnections?: number;
  /** Certificate expiry days */
  certificateExpiryDays?: number;
  /** Custom metrics */
  customMetrics?: Record<string, number>;
}

// ============================================================================
// Security Policies
// ============================================================================

/**
 * Security policy definition
 */
export interface SecurityPolicy {
  /** Policy unique identifier */
  policyId: string;
  /** Policy name */
  name: string;
  /** Policy version */
  version: string;
  /** Policy description */
  description?: string;
  /** Policy rules */
  rules: PolicyRules;
  /** Policy enforcement level */
  enforcement: 'strict' | 'moderate' | 'lenient';
  /** Policy applied timestamp */
  appliedAt: string;
  /** Policy expiry (optional) */
  expiresAt?: string;
}

/**
 * Policy rules configuration
 */
export interface PolicyRules {
  /** Authentication rules */
  authentication?: {
    /** Require mutual TLS */
    requireMutualTLS?: boolean;
    /** Minimum TLS version */
    minimumTLSVersion?: '1.2' | '1.3';
    /** Allowed cipher suites */
    allowedCipherSuites?: string[];
    /** Certificate validation */
    certificateValidation?: 'strict' | 'standard' | 'lenient';
    /** Maximum failed attempts */
    maxFailedAttempts?: number;
    /** Lockout duration (seconds) */
    lockoutDuration?: number;
  };
  /** Encryption rules */
  encryption?: {
    /** Require encryption at rest */
    encryptionAtRest?: boolean;
    /** Encryption algorithm */
    algorithm?: CryptoAlgorithm;
    /** Key rotation interval (days) */
    keyRotationDays?: number;
    /** Minimum key length */
    minimumKeyLength?: number;
  };
  /** Network rules */
  network?: {
    /** Allowed IP ranges */
    allowedIPRanges?: string[];
    /** Blocked IP addresses */
    blockedIPs?: string[];
    /** Rate limiting */
    rateLimit?: {
      /** Requests per minute */
      requestsPerMinute: number;
      /** Burst size */
      burstSize: number;
    };
  };
  /** Firmware rules */
  firmware?: {
    /** Require signed firmware */
    requireSigned?: boolean;
    /** Allow downgrade */
    allowDowngrade?: boolean;
    /** Auto-update enabled */
    autoUpdate?: boolean;
    /** Update window */
    updateWindow?: {
      start: string;
      end: string;
    };
  };
}

/**
 * Policy application request
 */
export interface PolicyApplication {
  /** Policy ID to apply */
  policyId: string;
  /** Target device IDs */
  targetDevices: string[];
  /** Enforce immediately */
  enforceImmediately: boolean;
  /** Rollback on failure */
  rollbackOnFailure?: boolean;
}

/**
 * Policy application result
 */
export interface PolicyApplicationResult {
  /** Application successful */
  applied: boolean;
  /** Number of affected devices */
  affectedDevices: number;
  /** Application timestamp */
  timestamp: string;
  /** Failed devices */
  failures?: Array<{
    deviceId: string;
    reason: string;
  }>;
}

// ============================================================================
// Security Audits
// ============================================================================

/**
 * Security audit configuration
 */
export interface SecurityAudit {
  /** Audit unique identifier */
  auditId: string;
  /** Target device IDs */
  deviceIds: string[];
  /** Audit type */
  auditType: 'comprehensive' | 'quick' | 'targeted';
  /** Audit checks to perform */
  checks: Array<'certificates' | 'encryption' | 'authentication' | 'vulnerabilities' | 'configuration' | 'compliance'>;
  /** Audit status */
  status: 'pending' | 'running' | 'completed' | 'failed';
  /** Audit start time */
  startedAt: string;
  /** Estimated completion time */
  estimatedCompletion?: string;
  /** Audit completion time */
  completedAt?: string;
}

/**
 * Security audit results
 */
export interface AuditResults {
  /** Audit ID */
  auditId: string;
  /** Audit status */
  status: 'completed' | 'failed';
  /** Start timestamp */
  startedAt: string;
  /** Completion timestamp */
  completedAt: string;
  /** Overall security score (0-100) */
  overallScore: number;
  /** Audit findings */
  findings: AuditFinding[];
  /** Number of passed checks */
  passed: number;
  /** Number of failed checks */
  failed: number;
  /** Number of warnings */
  warnings: number;
  /** Compliance status */
  compliance?: {
    framework: string;
    compliant: boolean;
    score: number;
  }[];
}

/**
 * Audit finding
 */
export interface AuditFinding {
  /** Finding ID */
  findingId: string;
  /** Finding severity */
  severity: SecuritySeverity;
  /** Finding category */
  category: SecurityCategory;
  /** Issue description */
  issue: string;
  /** Recommendation */
  recommendation: string;
  /** Affected devices count */
  affectedDevices: number;
  /** CVE references (if applicable) */
  cveReferences?: string[];
  /** Remediation steps */
  remediation?: string[];
}

// ============================================================================
// Incident Response
// ============================================================================

/**
 * Security incident report
 */
export interface SecurityIncident {
  /** Incident unique identifier */
  incidentId: string;
  /** Device ID */
  deviceId: string;
  /** Incident severity */
  severity: SecuritySeverity;
  /** Incident category */
  category: SecurityCategory;
  /** Incident description */
  description: string;
  /** Detection timestamp */
  detectedAt: string;
  /** Incident status */
  status: 'open' | 'investigating' | 'resolved' | 'closed';
  /** Assigned team/person */
  assignedTo?: string;
  /** Priority level */
  priority: 'critical' | 'high' | 'medium' | 'low';
  /** Evidence data */
  evidence?: Record<string, any>;
  /** Incident timeline */
  timeline?: IncidentTimelineEntry[];
  /** Resolution details */
  resolution?: {
    /** Resolution timestamp */
    resolvedAt: string;
    /** Resolution action */
    action: string;
    /** Root cause */
    rootCause?: string;
    /** Lessons learned */
    lessonsLearned?: string;
  };
}

/**
 * Incident timeline entry
 */
export interface IncidentTimelineEntry {
  /** Entry timestamp */
  timestamp: string;
  /** Action performed */
  action: string;
  /** Actor (user/system) */
  actor: string;
  /** Additional details */
  details?: string;
}

/**
 * Incident response playbook
 */
export interface IncidentPlaybook {
  /** Playbook ID */
  playbookId: string;
  /** Playbook name */
  name: string;
  /** Trigger condition */
  trigger: string;
  /** Playbook steps */
  steps: PlaybookStep[];
  /** Auto-execute flag */
  autoExecute: boolean;
}

/**
 * Playbook step
 */
export interface PlaybookStep {
  /** Step number */
  stepNumber: number;
  /** Action to perform */
  action: string;
  /** Action parameters */
  parameters: Record<string, any>;
  /** Required approval */
  requiresApproval?: boolean;
  /** Timeout (seconds) */
  timeout?: number;
}

// ============================================================================
// Device Management
// ============================================================================

/**
 * Complete device information
 */
export interface Device {
  /** Device identity */
  identity: DeviceIdentity;
  /** Security information */
  security: DeviceSecurity;
  /** Network information */
  network?: DeviceNetwork;
  /** Last seen timestamp */
  lastSeen: string;
  /** Device tags */
  tags?: string[];
}

/**
 * Device security information
 */
export interface DeviceSecurity {
  /** Security status */
  status: 'secure' | 'warning' | 'compromised';
  /** Last security audit */
  lastAudit: string;
  /** Certificate expiry */
  certificateExpiry: string;
  /** Compliance score (0-100) */
  complianceScore: number;
  /** Known vulnerabilities */
  vulnerabilities: Vulnerability[];
  /** Applied security policies */
  appliedPolicies?: string[];
}

/**
 * Device network information
 */
export interface DeviceNetwork {
  /** IP address */
  ipAddress?: string;
  /** Connection type */
  connectionType: 'wifi' | 'ethernet' | 'cellular' | 'lora' | 'bluetooth';
  /** Signal strength */
  signalStrength?: number;
  /** Last connected */
  lastConnected: string;
  /** Data usage */
  dataUsage?: {
    uploaded: number;
    downloaded: number;
  };
}

/**
 * Vulnerability information
 */
export interface Vulnerability {
  /** CVE identifier */
  cveId: string;
  /** Vulnerability description */
  description: string;
  /** Severity score (CVSS) */
  cvssScore: number;
  /** Severity level */
  severity: SecuritySeverity;
  /** Affected components */
  affectedComponents: string[];
  /** Patch available */
  patchAvailable: boolean;
  /** Patch version */
  patchVersion?: string;
}

// ============================================================================
// Configuration
// ============================================================================

/**
 * SDK configuration
 */
export interface WIAIoTSecurityConfig {
  /** API base URL */
  apiBaseUrl: string;
  /** API key for authentication */
  apiKey?: string;
  /** OAuth token */
  token?: string;
  /** Request timeout (ms) */
  timeout?: number;
  /** Retry configuration */
  retry?: {
    /** Maximum retry attempts */
    maxRetries: number;
    /** Retry delay (ms) */
    retryDelay: number;
  };
  /** TLS configuration */
  tls?: {
    /** Client certificate */
    clientCert?: string;
    /** Client key */
    clientKey?: string;
    /** CA certificate */
    caCert?: string;
    /** Verify server certificate */
    rejectUnauthorized?: boolean;
  };
}

// ============================================================================
// API Response Types
// ============================================================================

/**
 * Standard API response wrapper
 */
export interface APIResponse<T> {
  /** Response data */
  data: T;
  /** Response metadata */
  metadata?: {
    /** Request ID */
    requestId: string;
    /** Response timestamp */
    timestamp: string;
    /** API version */
    version: string;
  };
}

/**
 * API error response
 */
export interface APIError {
  /** Error code */
  code: string;
  /** Error message */
  message: string;
  /** Error timestamp */
  timestamp: string;
  /** Request ID */
  requestId: string;
  /** Additional error details */
  details?: Record<string, any>;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  /** Response items */
  items: T[];
  /** Total items count */
  total: number;
  /** Current page */
  page: number;
  /** Page size */
  pageSize: number;
  /** Has more pages */
  hasMore: boolean;
}
