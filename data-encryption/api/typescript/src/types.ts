/**
 * WIA Data Encryption Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Data Encryption Types
// ============================================================================

export interface WIADataEncryptionProject {
  standard: 'WIA-DATA-ENCRYPTION';
  version: string;
  metadata: ProjectMetadata;
  keyManagement: KeyManagementConfig;
  encryption: EncryptionConfig;
  policies: PolicyConfig;
  operations: OperationsConfig;
  compliance: ComplianceConfig;
  monitoring: MonitoringConfig;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  organization: Organization;
  classification: SecurityClassification;
  createdAt: string;
  updatedAt?: string;
  status: ProjectStatus;
}

export interface Organization {
  name: string;
  department?: string;
  securityContact: ContactInfo;
}

export interface ContactInfo {
  name: string;
  email: string;
  phone?: string;
  role: string;
}

export type SecurityClassification = 'public' | 'internal' | 'confidential' | 'secret' | 'top-secret';
export type ProjectStatus = 'active' | 'provisioning' | 'rotating' | 'suspended';

// ============================================================================
// Key Management Types
// ============================================================================

export interface KeyManagementConfig {
  provider: KMSProvider;
  hierarchy: KeyHierarchy;
  lifecycle: KeyLifecycle;
  storage: KeyStorage;
  backup: KeyBackupConfig;
}

export interface KMSProvider {
  type: 'aws-kms' | 'azure-keyvault' | 'gcp-kms' | 'hashicorp-vault' | 'hsm' | 'custom';
  endpoint?: string;
  region?: string;
  authentication: AuthConfig;
  options?: Record<string, unknown>;
}

export interface AuthConfig {
  method: 'iam' | 'service-account' | 'token' | 'certificate' | 'msi';
  credentials: CredentialRef;
}

export interface CredentialRef {
  type: 'secret-manager' | 'env' | 'file';
  reference: string;
}

export interface KeyHierarchy {
  levels: KeyLevel[];
  wrapping: KeyWrapping;
  derivation: KeyDerivation;
}

export interface KeyLevel {
  level: number;
  name: string;
  purpose: 'master' | 'key-encrypting' | 'data-encrypting' | 'signing';
  algorithm: string;
  keyLength: number;
  protection: 'hsm' | 'software';
}

export interface KeyWrapping {
  algorithm: 'aes-wrap' | 'rsa-oaep' | 'envelope';
  padding?: string;
}

export interface KeyDerivation {
  function: 'hkdf' | 'pbkdf2' | 'argon2' | 'scrypt';
  iterations?: number;
  saltLength: number;
}

export interface KeyLifecycle {
  generation: KeyGenerationPolicy;
  rotation: RotationPolicy;
  archival: ArchivalPolicy;
  destruction: DestructionPolicy;
}

export interface KeyGenerationPolicy {
  source: 'hardware' | 'software' | 'external';
  entropy: 'hardware' | 'os' | 'hybrid';
  validation: boolean;
}

export interface RotationPolicy {
  automatic: boolean;
  interval: string;
  triggerConditions: string[];
  gracePeriod: string;
  notification: string[];
}

export interface ArchivalPolicy {
  enabled: boolean;
  retentionPeriod: string;
  format: 'encrypted' | 'split' | 'escrowed';
  location: string;
}

export interface DestructionPolicy {
  method: 'crypto-erase' | 'overwrite' | 'physical';
  verification: boolean;
  audit: boolean;
  holdPeriod?: string;
}

export interface KeyStorage {
  primary: StorageConfig;
  secondary?: StorageConfig;
  caching: CachingConfig;
}

export interface StorageConfig {
  type: 'hsm' | 'kms' | 'vault' | 'encrypted-db';
  location: string;
  redundancy: 'none' | 'geo' | 'multi-region';
  encryption: boolean;
}

export interface CachingConfig {
  enabled: boolean;
  ttl: string;
  maxEntries: number;
  encryption: boolean;
}

export interface KeyBackupConfig {
  enabled: boolean;
  frequency: string;
  destinations: BackupDestination[];
  encryption: BackupEncryption;
  testing: string;
}

export interface BackupDestination {
  type: 'cloud' | 'offline' | 'escrow';
  location: string;
  copies: number;
}

export interface BackupEncryption {
  algorithm: string;
  keySource: 'separate' | 'derived';
}

// ============================================================================
// Encryption Configuration Types
// ============================================================================

export interface EncryptionConfig {
  algorithms: AlgorithmConfig;
  modes: EncryptionModes;
  dataTypes: DataTypeEncryption[];
  fieldLevel: FieldLevelEncryption;
}

export interface AlgorithmConfig {
  symmetric: SymmetricAlgorithm[];
  asymmetric: AsymmetricAlgorithm[];
  hashing: HashAlgorithm[];
  signing: SigningAlgorithm[];
}

export interface SymmetricAlgorithm {
  id: string;
  name: string;
  algorithm: 'aes' | 'chacha20' | '3des';
  keyLength: 128 | 192 | 256;
  mode: 'gcm' | 'cbc' | 'ctr' | 'ccm';
  recommended: boolean;
}

export interface AsymmetricAlgorithm {
  id: string;
  name: string;
  algorithm: 'rsa' | 'ec' | 'ed25519' | 'x25519';
  keyLength?: number;
  curve?: string;
  purpose: 'encryption' | 'signing' | 'key-agreement';
}

export interface HashAlgorithm {
  id: string;
  name: string;
  algorithm: 'sha256' | 'sha384' | 'sha512' | 'sha3-256' | 'blake2b';
  outputLength: number;
  recommended: boolean;
}

export interface SigningAlgorithm {
  id: string;
  name: string;
  algorithm: 'rsa-pss' | 'ecdsa' | 'ed25519';
  hashAlgorithm: string;
}

export interface EncryptionModes {
  atRest: AtRestConfig;
  inTransit: InTransitConfig;
  inUse: InUseConfig;
}

export interface AtRestConfig {
  enabled: boolean;
  defaultAlgorithm: string;
  keyScope: 'database' | 'table' | 'column' | 'row';
  transparentEncryption: boolean;
}

export interface InTransitConfig {
  required: boolean;
  protocols: TLSConfig;
  mtls: boolean;
  certificateManagement: string;
}

export interface TLSConfig {
  minimumVersion: '1.2' | '1.3';
  cipherSuites: string[];
  certificateValidation: boolean;
}

export interface InUseConfig {
  enabled: boolean;
  technology: 'enclaves' | 'homomorphic' | 'mpc' | 'none';
  scope: string[];
}

export interface DataTypeEncryption {
  dataType: string;
  classification: SecurityClassification;
  algorithm: string;
  preserveFormat: boolean;
  searchable: boolean;
}

export interface FieldLevelEncryption {
  enabled: boolean;
  defaultAction: 'encrypt' | 'hash' | 'mask' | 'none';
  rules: FieldEncryptionRule[];
}

export interface FieldEncryptionRule {
  id: string;
  pattern: string;
  classifications: string[];
  action: 'encrypt' | 'hash' | 'mask' | 'tokenize';
  algorithm?: string;
  preserveFormat?: boolean;
}

// ============================================================================
// Key & Encryption Operations Types
// ============================================================================

export interface Key {
  id: string;
  name: string;
  type: 'symmetric' | 'asymmetric' | 'derived';
  algorithm: string;
  keyLength: number;
  purpose: string;
  status: KeyStatus;
  createdAt: string;
  expiresAt?: string;
  rotatedAt?: string;
  metadata: Record<string, unknown>;
}

export type KeyStatus = 'active' | 'pending-rotation' | 'rotating' | 'deactivated' | 'archived' | 'destroyed';

export interface KeyVersion {
  version: number;
  status: 'primary' | 'secondary' | 'archived';
  createdAt: string;
  deactivatedAt?: string;
}

export interface EncryptionRequest {
  plaintext: string;
  keyId: string;
  context?: Record<string, string>;
  algorithm?: string;
  outputFormat?: 'base64' | 'hex' | 'binary';
}

export interface EncryptionResult {
  ciphertext: string;
  keyId: string;
  keyVersion: number;
  algorithm: string;
  iv?: string;
  tag?: string;
}

export interface DecryptionRequest {
  ciphertext: string;
  keyId: string;
  context?: Record<string, string>;
  iv?: string;
  tag?: string;
}

export interface DecryptionResult {
  plaintext: string;
  keyId: string;
  keyVersion: number;
}

export interface SigningRequest {
  message: string;
  keyId: string;
  algorithm?: string;
  hashAlgorithm?: string;
}

export interface SignatureResult {
  signature: string;
  keyId: string;
  algorithm: string;
}

export interface VerificationRequest {
  message: string;
  signature: string;
  keyId: string;
}

export interface VerificationResult {
  valid: boolean;
  keyId: string;
  timestamp: string;
}

// ============================================================================
// Policy & Compliance Types
// ============================================================================

export interface PolicyConfig {
  accessPolicies: AccessPolicy[];
  usagePolicies: UsagePolicy[];
  rotationPolicies: RotationPolicyDef[];
  exceptionHandling: ExceptionConfig;
}

export interface AccessPolicy {
  id: string;
  name: string;
  principals: string[];
  resources: string[];
  operations: ('encrypt' | 'decrypt' | 'sign' | 'verify' | 'generate' | 'rotate' | 'destroy')[];
  conditions?: PolicyCondition[];
}

export interface PolicyCondition {
  type: 'time' | 'ip' | 'mfa' | 'context';
  operator: string;
  value: unknown;
}

export interface UsagePolicy {
  id: string;
  name: string;
  keyTypes: string[];
  maxOperations?: number;
  rateLimit?: { requests: number; period: string };
  allowedContexts?: string[];
}

export interface RotationPolicyDef {
  id: string;
  name: string;
  keyTypes: string[];
  rotationInterval: string;
  gracePeriod: string;
  autoRotate: boolean;
}

export interface ExceptionConfig {
  emergencyAccess: EmergencyAccessConfig;
  breakGlass: BreakGlassConfig;
}

export interface EmergencyAccessConfig {
  enabled: boolean;
  approvers: string[];
  maxDuration: string;
  auditRequired: boolean;
}

export interface BreakGlassConfig {
  enabled: boolean;
  procedure: string;
  notification: string[];
  postIncidentReview: boolean;
}

export interface ComplianceConfig {
  standards: ComplianceStandard[];
  certifications: Certification[];
  controls: EncryptionControl[];
  auditing: AuditConfig;
}

export interface ComplianceStandard {
  id: string;
  name: string;
  requirements: string[];
  status: 'compliant' | 'partial' | 'non-compliant';
}

export interface Certification {
  name: string;
  level: string;
  validUntil: string;
  scope: string[];
}

export interface EncryptionControl {
  id: string;
  standard: string;
  requirement: string;
  implementation: string;
  evidence: string[];
  status: 'implemented' | 'planned' | 'not-applicable';
}

export interface AuditConfig {
  enabled: boolean;
  events: AuditEvent[];
  retention: string;
  integrity: 'hash' | 'signature' | 'blockchain';
  destination: string;
}

export interface AuditEvent {
  type: string;
  severity: 'info' | 'warning' | 'critical';
  capture: string[];
}

// ============================================================================
// Operations & Monitoring Types
// ============================================================================

export interface OperationsConfig {
  automation: AutomationConfig;
  integration: IntegrationConfig;
  performance: PerformanceConfig;
}

export interface AutomationConfig {
  keyRotation: boolean;
  certificateRenewal: boolean;
  policyEnforcement: boolean;
  incidentResponse: boolean;
}

export interface IntegrationConfig {
  apis: APIIntegration[];
  sdks: string[];
  plugins: PluginConfig[];
}

export interface APIIntegration {
  name: string;
  type: 'rest' | 'grpc' | 'sdk';
  authentication: string;
  rateLimit: number;
}

export interface PluginConfig {
  name: string;
  type: 'database' | 'application' | 'storage';
  version: string;
  config: Record<string, unknown>;
}

export interface PerformanceConfig {
  caching: boolean;
  batching: BatchingConfig;
  threading: ThreadingConfig;
}

export interface BatchingConfig {
  enabled: boolean;
  maxSize: number;
  maxLatency: string;
}

export interface ThreadingConfig {
  poolSize: number;
  async: boolean;
}

export interface MonitoringConfig {
  metrics: MetricsConfig;
  alerting: AlertConfig;
  logging: LoggingConfig;
  reporting: ReportingConfig;
}

export interface MetricsConfig {
  enabled: boolean;
  provider: string;
  metrics: string[];
  interval: string;
}

export interface AlertConfig {
  enabled: boolean;
  rules: AlertRule[];
  channels: string[];
}

export interface AlertRule {
  id: string;
  name: string;
  condition: string;
  severity: 'low' | 'medium' | 'high' | 'critical';
  actions: string[];
}

export interface LoggingConfig {
  level: 'debug' | 'info' | 'warn' | 'error';
  format: 'json' | 'text';
  destination: string;
  sensitiveDataMasking: boolean;
}

export interface ReportingConfig {
  scheduled: ScheduledReport[];
  onDemand: boolean;
  formats: ('pdf' | 'json' | 'csv')[];
}

export interface ScheduledReport {
  name: string;
  type: string;
  frequency: string;
  recipients: string[];
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface ProjectResponse {
  id: string;
  name: string;
  status: ProjectStatus;
  createdAt: string;
  updatedAt?: string;
}

export interface ValidationResult {
  valid: boolean;
  errors?: ValidationError[];
}

export interface ValidationError {
  path: string;
  message: string;
  value?: unknown;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    total: number;
    limit: number;
    offset: number;
    hasMore: boolean;
  };
}
