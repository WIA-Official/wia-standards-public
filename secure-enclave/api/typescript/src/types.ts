/**
 * WIA Secure Enclave - TypeScript SDK
 * Type Definitions
 *
 * @version 1.0.0
 * @license Apache-2.0
 * @author WIA Technical Committee
 *
 * Philosophy: 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

// ============================================================================
// Hardware Security Module (HSM) Types
// ============================================================================

export type HSMType = 'hardware' | 'software' | 'cloud' | 'hybrid';
export type HSMStatus = 'operational' | 'degraded' | 'offline' | 'maintenance';
export type HSMCompliance = 'FIPS_140_2_L1' | 'FIPS_140_2_L2' | 'FIPS_140_2_L3' | 'FIPS_140_2_L4' | 'FIPS_140_3_L1' | 'FIPS_140_3_L2' | 'FIPS_140_3_L3' | 'FIPS_140_3_L4' | 'CC_EAL4' | 'CC_EAL5' | 'CC_EAL6' | 'CC_EAL7';

export interface HSMCapabilities {
  max_keys: number;
  max_sessions: number;
  supports_key_backup: boolean;
  supports_key_replication: boolean;
  supports_tamper_detection: boolean;
  supports_secure_boot: boolean;
  cryptographic_algorithms: string[];
  key_sizes: number[];
}

export interface HSMInfo {
  hsm_id: string;
  name: string;
  type: HSMType;
  status: HSMStatus;
  compliance_levels: HSMCompliance[];
  capabilities: HSMCapabilities;
  firmware_version: string;
  manufacturer: string;
  serial_number: string;
  installation_date: string;
  last_maintenance?: string;
  metadata?: Record<string, any>;
}

// ============================================================================
// Trusted Execution Environment (TEE) Types
// ============================================================================

export type TEEType = 'SGX' | 'TrustZone' | 'SEV' | 'TDX' | 'Keystone' | 'OP-TEE' | 'custom';
export type TEEStatus = 'initialized' | 'running' | 'suspended' | 'terminated';
export type IsolationLevel = 'process' | 'container' | 'vm' | 'hardware';

export interface TEECapabilities {
  secure_storage: boolean;
  remote_attestation: boolean;
  sealed_storage: boolean;
  memory_encryption: boolean;
  secure_io: boolean;
  trusted_time: boolean;
  max_memory_mb: number;
}

export interface TEEInfo {
  tee_id: string;
  type: TEEType;
  status: TEEStatus;
  isolation_level: IsolationLevel;
  capabilities: TEECapabilities;
  version: string;
  measurement?: string;
  created_at: string;
  metadata?: Record<string, any>;
}

export interface CreateTEERequest {
  type: TEEType;
  name: string;
  isolation_level: IsolationLevel;
  memory_mb: number;
  code_hash: string;
  initial_data?: ArrayBuffer;
  metadata?: Record<string, any>;
}

// ============================================================================
// Secure Key Storage Types
// ============================================================================

export type KeyType = 'symmetric' | 'asymmetric' | 'hmac' | 'certificate';
export type KeyAlgorithm = 'AES' | 'RSA' | 'ECC' | 'Ed25519' | 'X25519' | 'HMAC_SHA256' | 'HMAC_SHA512';
export type KeyUsage = 'encrypt' | 'decrypt' | 'sign' | 'verify' | 'wrap' | 'unwrap' | 'derive';
export type KeyStatus = 'active' | 'inactive' | 'compromised' | 'destroyed' | 'suspended';

export interface KeyAttributes {
  key_id: string;
  label: string;
  key_type: KeyType;
  algorithm: KeyAlgorithm;
  key_size: number;
  usages: KeyUsage[];
  status: KeyStatus;
  extractable: boolean;
  expires_at?: string;
  created_at: string;
  last_used?: string;
  owner_id?: string;
  tags?: Record<string, string>;
}

export interface CreateKeyRequest {
  label: string;
  key_type: KeyType;
  algorithm: KeyAlgorithm;
  key_size: number;
  usages: KeyUsage[];
  extractable?: boolean;
  expires_at?: string;
  metadata?: Record<string, any>;
  backup_enabled?: boolean;
}

export interface ImportKeyRequest {
  label: string;
  key_material: ArrayBuffer;
  key_type: KeyType;
  algorithm: KeyAlgorithm;
  usages: KeyUsage[];
  extractable?: boolean;
  wrapped?: boolean;
  wrapping_key_id?: string;
}

export interface ExportKeyRequest {
  key_id: string;
  format?: 'raw' | 'pkcs8' | 'spki' | 'jwk';
  wrapping_key_id?: string;
}

export interface KeyRotationPolicy {
  enabled: boolean;
  rotation_interval_days: number;
  auto_rotate: boolean;
  retain_old_versions: number;
}

// ============================================================================
// Cryptographic Operation Types
// ============================================================================

export type CryptoOperation = 'encrypt' | 'decrypt' | 'sign' | 'verify' | 'derive' | 'wrap' | 'unwrap';
export type PaddingScheme = 'PKCS1' | 'OAEP' | 'PSS' | 'PKCS7' | 'ISO9797';
export type HashAlgorithm = 'SHA256' | 'SHA384' | 'SHA512' | 'SHA3_256' | 'SHA3_512';

export interface EncryptRequest {
  key_id: string;
  plaintext: ArrayBuffer;
  algorithm: KeyAlgorithm;
  padding?: PaddingScheme;
  iv?: ArrayBuffer;
  aad?: ArrayBuffer;
  tag_length?: number;
}

export interface EncryptResponse {
  ciphertext: ArrayBuffer;
  iv?: ArrayBuffer;
  tag?: ArrayBuffer;
  metadata?: Record<string, any>;
}

export interface DecryptRequest {
  key_id: string;
  ciphertext: ArrayBuffer;
  algorithm: KeyAlgorithm;
  padding?: PaddingScheme;
  iv?: ArrayBuffer;
  aad?: ArrayBuffer;
  tag?: ArrayBuffer;
}

export interface DecryptResponse {
  plaintext: ArrayBuffer;
  metadata?: Record<string, any>;
}

export interface SignRequest {
  key_id: string;
  data: ArrayBuffer;
  algorithm: KeyAlgorithm;
  hash_algorithm: HashAlgorithm;
  padding?: PaddingScheme;
}

export interface SignResponse {
  signature: ArrayBuffer;
  metadata?: Record<string, any>;
}

export interface VerifyRequest {
  key_id: string;
  data: ArrayBuffer;
  signature: ArrayBuffer;
  algorithm: KeyAlgorithm;
  hash_algorithm: HashAlgorithm;
  padding?: PaddingScheme;
}

export interface VerifyResponse {
  valid: boolean;
  metadata?: Record<string, any>;
}

// ============================================================================
// Attestation and Verification Types
// ============================================================================

export type AttestationType = 'local' | 'remote' | 'mutual';
export type AttestationProtocol = 'SGX_EPID' | 'SGX_DCAP' | 'TPM_2_0' | 'DICE' | 'custom';
export type AttestationStatus = 'valid' | 'invalid' | 'expired' | 'revoked' | 'unknown';

export interface AttestationEvidence {
  evidence_type: string;
  evidence_data: ArrayBuffer;
  timestamp: string;
  nonce?: ArrayBuffer;
  protocol: AttestationProtocol;
}

export interface AttestationRequest {
  tee_id: string;
  attestation_type: AttestationType;
  nonce?: ArrayBuffer;
  user_data?: ArrayBuffer;
  protocol: AttestationProtocol;
}

export interface AttestationResponse {
  quote: ArrayBuffer;
  evidence: AttestationEvidence;
  certificate_chain?: ArrayBuffer[];
  tcb_info?: Record<string, any>;
  status: AttestationStatus;
  verified_at: string;
  verifier_id?: string;
  metadata?: Record<string, any>;
}

export interface VerifyAttestationRequest {
  quote: ArrayBuffer;
  evidence: AttestationEvidence;
  certificate_chain?: ArrayBuffer[];
  policy?: AttestationPolicy;
}

export interface AttestationPolicy {
  minimum_tcb_level?: number;
  allowed_signers?: string[];
  required_measurements?: string[];
  max_age_seconds?: number;
  custom_rules?: Record<string, any>;
}

// ============================================================================
// Secure Boot Types
// ============================================================================

export type BootPhase = 'firmware' | 'bootloader' | 'os' | 'application';
export type MeasurementType = 'sha256' | 'sha384' | 'sha512';

export interface BootMeasurement {
  phase: BootPhase;
  component: string;
  measurement: string;
  measurement_type: MeasurementType;
  timestamp: string;
  pcr_index?: number;
}

export interface SecureBootStatus {
  enabled: boolean;
  verified: boolean;
  measurements: BootMeasurement[];
  boot_time: string;
  last_verified: string;
  violations?: string[];
}

export interface UpdateSecureBootPolicyRequest {
  allowed_measurements: Array<{
    component: string;
    measurement: string;
  }>;
  enforce: boolean;
  alert_on_violation: boolean;
}

// ============================================================================
// Access Control Types
// ============================================================================

export type AccessLevel = 'read' | 'write' | 'execute' | 'admin' | 'owner';
export type AuthenticationMethod = 'password' | 'certificate' | 'biometric' | 'hardware_token' | 'multi_factor';

export interface AccessPolicy {
  policy_id: string;
  name: string;
  resource_type: string;
  resource_id?: string;
  principals: string[];
  allowed_operations: CryptoOperation[];
  conditions?: Record<string, any>;
  priority: number;
  enabled: boolean;
  created_at: string;
  updated_at: string;
}

export interface CreateAccessPolicyRequest {
  name: string;
  resource_type: string;
  resource_id?: string;
  principals: string[];
  allowed_operations: CryptoOperation[];
  conditions?: Record<string, any>;
  priority?: number;
}

export interface AccessRequest {
  principal_id: string;
  resource_type: string;
  resource_id: string;
  operation: CryptoOperation;
  context?: Record<string, any>;
}

export interface AccessDecision {
  allowed: boolean;
  reason: string;
  matched_policies?: string[];
  enforced_conditions?: Record<string, any>;
}

// ============================================================================
// Audit Logging Types
// ============================================================================

export type AuditEventType =
  | 'key_created'
  | 'key_deleted'
  | 'key_rotated'
  | 'key_exported'
  | 'key_imported'
  | 'encrypt_operation'
  | 'decrypt_operation'
  | 'sign_operation'
  | 'verify_operation'
  | 'attestation_requested'
  | 'attestation_verified'
  | 'access_granted'
  | 'access_denied'
  | 'policy_created'
  | 'policy_updated'
  | 'policy_deleted'
  | 'tee_created'
  | 'tee_terminated'
  | 'boot_verified'
  | 'boot_failed';

export type AuditSeverity = 'info' | 'warning' | 'error' | 'critical';

export interface AuditEvent {
  event_id: string;
  event_type: AuditEventType;
  severity: AuditSeverity;
  timestamp: string;
  principal_id?: string;
  resource_type?: string;
  resource_id?: string;
  operation?: string;
  result: 'success' | 'failure';
  details?: Record<string, any>;
  ip_address?: string;
  user_agent?: string;
  correlation_id?: string;
  integrity_hash?: string;
}

export interface AuditQuery {
  start_date?: string;
  end_date?: string;
  event_type?: AuditEventType;
  principal_id?: string;
  resource_id?: string;
  severity?: AuditSeverity;
  result?: 'success' | 'failure';
  page?: number;
  per_page?: number;
}

export interface AuditLogConfig {
  enabled: boolean;
  log_level: AuditSeverity;
  retention_days: number;
  tamper_proof: boolean;
  real_time_alerts: boolean;
  export_format?: 'json' | 'syslog' | 'cef';
}

// ============================================================================
// Client Configuration Types
// ============================================================================

export interface WiaSecureEnclaveConfig {
  apiUrl: string;
  apiKey?: string;
  accessToken?: string;
  hsmId?: string;
  timeout?: number;
  retries?: number;
  enableAuditLogging?: boolean;
  headers?: Record<string, string>;
}

// ============================================================================
// Error Types
// ============================================================================

export interface ApiError {
  code: string;
  message: string;
  details?: string;
  field?: string;
  request_id?: string;
  documentation_url?: string;
}

export interface ApiErrorResponse {
  error: ApiError;
}

// ============================================================================
// Pagination Types
// ============================================================================

export interface Pagination {
  current_page: number;
  per_page: number;
  total_pages: number;
  total_items: number;
  has_next: boolean;
  has_previous: boolean;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: Pagination;
}
