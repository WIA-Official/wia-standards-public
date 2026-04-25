/**
 * WIA Digital Executor Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export interface WIADigitalExecutor {
  standard: 'WIA-DIGITAL-EXECUTOR';
  version: string;
  executor: ExecutorProfile;
  principal: PrincipalInfo;
  digitalAssets: DigitalAsset[];
  instructions: ExecutionInstruction[];
  triggers: ExecutionTrigger[];
  verification: VerificationRequirements;
  audit: AuditConfiguration;
  extensions?: Record<string, unknown>;
}

export interface ExecutorProfile {
  id: string;
  name: string;
  type: ExecutorType;
  status: ExecutorStatus;
  jurisdiction: string;
  createdAt: string;
  updatedAt?: string;
  contact: ContactInfo;
  credentials?: Credential[];
}

export type ExecutorType = 'individual' | 'attorney' | 'trust-company' | 'platform' | 'ai-agent';
export type ExecutorStatus = 'pending' | 'active' | 'suspended' | 'revoked' | 'deceased';

export interface ContactInfo {
  email: string;
  phone?: string;
  address?: Address;
  notificationPreferences: NotificationPreference[];
}

export interface Address {
  street: string;
  city: string;
  state?: string;
  country: string;
  postalCode: string;
}

export type NotificationPreference = 'email' | 'sms' | 'push' | 'mail';

export interface Credential {
  type: 'bar-license' | 'certification' | 'bonding' | 'insurance';
  issuer: string;
  number: string;
  validUntil: string;
  verified: boolean;
}

export interface PrincipalInfo {
  id: string;
  name: string;
  dateOfBirth?: string;
  identifiers: Identifier[];
  lastKnownStatus: 'alive' | 'incapacitated' | 'deceased';
  statusUpdatedAt: string;
  deathCertificate?: DeathCertificate;
}

export interface Identifier {
  type: 'ssn' | 'passport' | 'national-id' | 'driver-license';
  value: string;
  country: string;
}

export interface DeathCertificate {
  number: string;
  issuedBy: string;
  issuedDate: string;
  dateOfDeath: string;
  verified: boolean;
}

// ============================================================================
// Digital Asset Types
// ============================================================================

export interface DigitalAsset {
  id: string;
  category: AssetCategory;
  name: string;
  platform: string;
  accountId?: string;
  estimatedValue?: MonetaryValue;
  accessCredentials?: AccessCredential;
  disposition: AssetDisposition;
  beneficiary?: Beneficiary;
  lastVerified?: string;
}

export type AssetCategory =
  | 'social-media'
  | 'email'
  | 'financial'
  | 'cryptocurrency'
  | 'cloud-storage'
  | 'digital-media'
  | 'gaming'
  | 'subscription'
  | 'domain'
  | 'business'
  | 'other';

export interface MonetaryValue {
  amount: number;
  currency: string;
  asOf: string;
}

export interface AccessCredential {
  type: 'password' | 'recovery-key' | 'hardware-token' | 'biometric';
  encryptedValue?: string;
  storageLocation: string;
  lastUpdated: string;
}

export interface AssetDisposition {
  action: DispositionAction;
  timing: DispositionTiming;
  conditions?: string[];
}

export type DispositionAction = 'transfer' | 'memorialize' | 'delete' | 'archive' | 'liquidate' | 'maintain';
export type DispositionTiming = 'immediate' | 'after-30-days' | 'after-90-days' | 'after-1-year' | 'upon-probate' | 'manual';

export interface Beneficiary {
  id: string;
  name: string;
  relationship: string;
  contact: ContactInfo;
  share?: number;
}

// ============================================================================
// Instruction Types
// ============================================================================

export interface ExecutionInstruction {
  id: string;
  priority: number;
  category: InstructionCategory;
  action: string;
  details: Record<string, unknown>;
  conditions?: InstructionCondition[];
  status: InstructionStatus;
  executedAt?: string;
  result?: ExecutionResult;
}

export type InstructionCategory = 'asset-transfer' | 'account-closure' | 'data-backup' | 'notification' | 'legal' | 'memorial' | 'custom';
export type InstructionStatus = 'pending' | 'in-progress' | 'completed' | 'failed' | 'skipped';

export interface InstructionCondition {
  type: 'time-elapsed' | 'event-occurred' | 'approval-received' | 'asset-threshold';
  parameters: Record<string, unknown>;
  met: boolean;
}

export interface ExecutionResult {
  success: boolean;
  message: string;
  artifacts?: string[];
  timestamp: string;
}

// ============================================================================
// Trigger Types
// ============================================================================

export interface ExecutionTrigger {
  id: string;
  type: TriggerType;
  configuration: TriggerConfiguration;
  status: TriggerStatus;
  activatedAt?: string;
}

export type TriggerType = 'death-certificate' | 'inactivity' | 'manual' | 'scheduled' | 'incapacity' | 'third-party-notification';
export type TriggerStatus = 'armed' | 'activated' | 'cancelled' | 'expired';

export interface TriggerConfiguration {
  inactivityDays?: number;
  scheduledDate?: string;
  verificationRequired: boolean;
  notifyBefore?: number;
  witnesses?: Witness[];
}

export interface Witness {
  id: string;
  name: string;
  email: string;
  role: 'primary' | 'backup';
  confirmed: boolean;
}

// ============================================================================
// Verification Types
// ============================================================================

export interface VerificationRequirements {
  deathVerification: DeathVerificationConfig;
  executorVerification: ExecutorVerificationConfig;
  beneficiaryVerification: BeneficiaryVerificationConfig;
}

export interface DeathVerificationConfig {
  methods: ('death-certificate' | 'obituary' | 'witness' | 'government-api')[];
  requiredConfirmations: number;
  autoVerifyAfter?: number;
}

export interface ExecutorVerificationConfig {
  identityVerification: boolean;
  courtAppointment?: boolean;
  periodicReVerification: boolean;
  reVerificationInterval?: number;
}

export interface BeneficiaryVerificationConfig {
  identityVerification: boolean;
  relationshipVerification: boolean;
  minimumAge?: number;
}

// ============================================================================
// Audit Types
// ============================================================================

export interface AuditConfiguration {
  enabled: boolean;
  retentionYears: number;
  immutable: boolean;
  events: AuditEvent[];
}

export interface AuditEvent {
  id: string;
  timestamp: string;
  eventType: AuditEventType;
  actor: string;
  action: string;
  details: Record<string, unknown>;
  ipAddress?: string;
  signature?: string;
}

export type AuditEventType = 'access' | 'modification' | 'execution' | 'verification' | 'notification' | 'dispute';

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface ExecutorResponse {
  id: string;
  name: string;
  type: ExecutorType;
  status: ExecutorStatus;
  principalName: string;
  assetCount: number;
  createdAt: string;
  links: { self: string };
}

export interface ValidationResult {
  valid: boolean;
  errors?: { path: string; message: string }[];
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: { total: number; limit: number; offset: number; hasMore: boolean };
}
