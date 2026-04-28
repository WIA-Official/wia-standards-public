/**
 * WIA Zero Trust Security Types
 *
 * Zero Trust Architecture: Never Trust, Always Verify
 * Philosophy: 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @package @wia/zero-trust
 * @version 1.0.0
 */

// ============================================================================
// Identity Verification Types
// ============================================================================

export interface WIAIdentity {
  id: string;
  type: IdentityType;
  attributes: IdentityAttributes;
  credentials: CredentialSet[];
  trustScore: number; // 0-100
  verificationLevel: VerificationLevel;
  metadata: Record<string, unknown>;
}

export enum IdentityType {
  HUMAN = 'HUMAN',
  MACHINE = 'MACHINE',
  SERVICE = 'SERVICE',
  APPLICATION = 'APPLICATION',
  IOT_DEVICE = 'IOT_DEVICE'
}

export interface IdentityAttributes {
  email?: string;
  username?: string;
  organizationId?: string;
  department?: string;
  role?: string;
  clearanceLevel?: string;
  lastVerified: Date;
  attributes: Record<string, string>;
}

export interface CredentialSet {
  type: CredentialType;
  value: string;
  issuedAt: Date;
  expiresAt?: Date;
  issuer: string;
  verified: boolean;
}

export enum CredentialType {
  PASSWORD = 'PASSWORD',
  MFA_TOKEN = 'MFA_TOKEN',
  BIOMETRIC = 'BIOMETRIC',
  CERTIFICATE = 'CERTIFICATE',
  API_KEY = 'API_KEY',
  HARDWARE_TOKEN = 'HARDWARE_TOKEN'
}

export enum VerificationLevel {
  NONE = 'NONE',
  LOW = 'LOW',
  MEDIUM = 'MEDIUM',
  HIGH = 'HIGH',
  CRITICAL = 'CRITICAL'
}

// ============================================================================
// Device Trust Types
// ============================================================================

export interface WIADevice {
  id: string;
  type: DeviceType;
  posture: DevicePosture;
  trustScore: number; // 0-100
  location: DeviceLocation;
  compliance: ComplianceStatus;
  lastAssessed: Date;
  metadata: Record<string, unknown>;
}

export enum DeviceType {
  LAPTOP = 'LAPTOP',
  DESKTOP = 'DESKTOP',
  MOBILE = 'MOBILE',
  TABLET = 'TABLET',
  IOT = 'IOT',
  SERVER = 'SERVER',
  VIRTUAL = 'VIRTUAL'
}

export interface DevicePosture {
  osVersion: string;
  osPatched: boolean;
  antivirusEnabled: boolean;
  antivirusUpdated: boolean;
  encryptionEnabled: boolean;
  firewallEnabled: boolean;
  unauthorizedSoftware: string[];
  securityAgentRunning: boolean;
  lastBootTime: Date;
}

export interface DeviceLocation {
  ipAddress: string;
  geoLocation?: GeoLocation;
  networkType: NetworkType;
  isKnownLocation: boolean;
  isCorporateNetwork: boolean;
}

export interface GeoLocation {
  country: string;
  region?: string;
  city?: string;
  latitude?: number;
  longitude?: number;
}

export enum NetworkType {
  CORPORATE = 'CORPORATE',
  VPN = 'VPN',
  PUBLIC = 'PUBLIC',
  HOME = 'HOME',
  UNKNOWN = 'UNKNOWN'
}

export interface ComplianceStatus {
  isCompliant: boolean;
  violations: string[];
  lastChecked: Date;
  policies: string[];
}

// ============================================================================
// Network Segmentation Types
// ============================================================================

export interface WIANetworkSegment {
  id: string;
  name: string;
  type: SegmentType;
  trustLevel: TrustLevel;
  allowedIdentities: string[];
  allowedDevices: string[];
  policies: SegmentPolicy[];
  microsegments: MicroSegment[];
}

export enum SegmentType {
  PERIMETER = 'PERIMETER',
  DMZ = 'DMZ',
  INTERNAL = 'INTERNAL',
  SENSITIVE = 'SENSITIVE',
  CRITICAL = 'CRITICAL',
  ISOLATED = 'ISOLATED'
}

export enum TrustLevel {
  UNTRUSTED = 'UNTRUSTED',
  LOW = 'LOW',
  MEDIUM = 'MEDIUM',
  HIGH = 'HIGH',
  CRITICAL = 'CRITICAL'
}

export interface SegmentPolicy {
  id: string;
  name: string;
  rules: NetworkRule[];
  enforcement: EnforcementMode;
}

export interface NetworkRule {
  id: string;
  source: NetworkEntity;
  destination: NetworkEntity;
  protocol: string;
  port?: number;
  action: RuleAction;
  conditions: RuleCondition[];
}

export interface NetworkEntity {
  type: 'IDENTITY' | 'DEVICE' | 'SEGMENT' | 'IP' | 'SUBNET';
  value: string;
}

export enum RuleAction {
  ALLOW = 'ALLOW',
  DENY = 'DENY',
  INSPECT = 'INSPECT',
  CHALLENGE = 'CHALLENGE'
}

export interface RuleCondition {
  attribute: string;
  operator: 'EQUALS' | 'NOT_EQUALS' | 'CONTAINS' | 'GREATER_THAN' | 'LESS_THAN';
  value: string | number | boolean;
}

export enum EnforcementMode {
  PERMISSIVE = 'PERMISSIVE',
  MONITOR = 'MONITOR',
  ENFORCE = 'ENFORCE'
}

// ============================================================================
// Access Policy Types
// ============================================================================

export interface WIAAccessPolicy {
  id: string;
  name: string;
  description: string;
  resources: Resource[];
  subjects: Subject[];
  conditions: AccessCondition[];
  permissions: Permission[];
  leastPrivilege: LeastPrivilegeConfig;
  ttl?: number; // Time to live in seconds
}

export interface Resource {
  type: ResourceType;
  id: string;
  path?: string;
  sensitivity: SensitivityLevel;
}

export enum ResourceType {
  API = 'API',
  DATABASE = 'DATABASE',
  FILE = 'FILE',
  APPLICATION = 'APPLICATION',
  SERVICE = 'SERVICE',
  NETWORK = 'NETWORK'
}

export enum SensitivityLevel {
  PUBLIC = 'PUBLIC',
  INTERNAL = 'INTERNAL',
  CONFIDENTIAL = 'CONFIDENTIAL',
  RESTRICTED = 'RESTRICTED',
  TOP_SECRET = 'TOP_SECRET'
}

export interface Subject {
  type: 'IDENTITY' | 'ROLE' | 'GROUP';
  id: string;
}

export interface AccessCondition {
  type: ConditionType;
  parameters: Record<string, unknown>;
  required: boolean;
}

export enum ConditionType {
  TIME_OF_DAY = 'TIME_OF_DAY',
  LOCATION = 'LOCATION',
  DEVICE_TRUST = 'DEVICE_TRUST',
  IDENTITY_TRUST = 'IDENTITY_TRUST',
  MFA_REQUIRED = 'MFA_REQUIRED',
  NETWORK_TYPE = 'NETWORK_TYPE',
  RISK_SCORE = 'RISK_SCORE'
}

export interface Permission {
  action: string;
  effect: 'ALLOW' | 'DENY';
  constraints?: PermissionConstraint[];
}

export interface PermissionConstraint {
  attribute: string;
  value: unknown;
}

export interface LeastPrivilegeConfig {
  enabled: boolean;
  justInTimeAccess: boolean;
  autoRevoke: boolean;
  reviewPeriod?: number; // in days
}

// ============================================================================
// Continuous Authentication Types
// ============================================================================

export interface WIAContinuousAuth {
  sessionId: string;
  identity: WIAIdentity;
  device: WIADevice;
  initialAuthAt: Date;
  lastVerifiedAt: Date;
  reAuthRequired: boolean;
  riskScore: number; // 0-100
  anomalies: AuthAnomaly[];
  challenges: AuthChallenge[];
}

export interface AuthAnomaly {
  type: AnomalyType;
  severity: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  detectedAt: Date;
  description: string;
  indicators: Record<string, unknown>;
}

export enum AnomalyType {
  LOCATION_CHANGE = 'LOCATION_CHANGE',
  DEVICE_CHANGE = 'DEVICE_CHANGE',
  BEHAVIOR_CHANGE = 'BEHAVIOR_CHANGE',
  UNUSUAL_ACCESS = 'UNUSUAL_ACCESS',
  VELOCITY_ANOMALY = 'VELOCITY_ANOMALY',
  IMPOSSIBLE_TRAVEL = 'IMPOSSIBLE_TRAVEL'
}

export interface AuthChallenge {
  id: string;
  type: ChallengeType;
  issuedAt: Date;
  expiresAt: Date;
  status: ChallengeStatus;
  attempts: number;
}

export enum ChallengeType {
  MFA = 'MFA',
  BIOMETRIC = 'BIOMETRIC',
  SECURITY_QUESTION = 'SECURITY_QUESTION',
  DEVICE_VERIFICATION = 'DEVICE_VERIFICATION',
  CAPTCHA = 'CAPTCHA'
}

export enum ChallengeStatus {
  PENDING = 'PENDING',
  COMPLETED = 'COMPLETED',
  FAILED = 'FAILED',
  EXPIRED = 'EXPIRED'
}

// ============================================================================
// Micro-Segmentation Types
// ============================================================================

export interface MicroSegment {
  id: string;
  name: string;
  workloads: Workload[];
  isolationLevel: IsolationLevel;
  policies: MicroSegmentPolicy[];
}

export interface Workload {
  id: string;
  type: WorkloadType;
  labels: Record<string, string>;
  trustedCommunications: string[];
}

export enum WorkloadType {
  CONTAINER = 'CONTAINER',
  VM = 'VM',
  PROCESS = 'PROCESS',
  SERVICE = 'SERVICE'
}

export enum IsolationLevel {
  NONE = 'NONE',
  LOGICAL = 'LOGICAL',
  PHYSICAL = 'PHYSICAL',
  CRYPTOGRAPHIC = 'CRYPTOGRAPHIC'
}

export interface MicroSegmentPolicy {
  id: string;
  fromWorkload: string;
  toWorkload: string;
  allowedProtocols: string[];
  allowedPorts: number[];
  encryption: EncryptionRequirement;
}

export interface EncryptionRequirement {
  required: boolean;
  algorithm?: string;
  minimumKeyLength?: number;
}

// ============================================================================
// Access Decision Types
// ============================================================================

export interface WIAAccessDecision {
  requestId: string;
  decision: Decision;
  reason: string;
  identity: WIAIdentity;
  device: WIADevice;
  resource: Resource;
  evaluatedPolicies: string[];
  timestamp: Date;
  ttl?: number;
}

export enum Decision {
  ALLOW = 'ALLOW',
  DENY = 'DENY',
  CHALLENGE = 'CHALLENGE',
  STEP_UP = 'STEP_UP'
}

// ============================================================================
// Event Types
// ============================================================================

export interface WIAZeroTrustEvent {
  id: string;
  type: EventType;
  timestamp: Date;
  severity: EventSeverity;
  identity?: WIAIdentity;
  device?: WIADevice;
  resource?: Resource;
  decision?: WIAAccessDecision;
  details: Record<string, unknown>;
}

export enum EventType {
  ACCESS_REQUEST = 'ACCESS_REQUEST',
  ACCESS_GRANTED = 'ACCESS_GRANTED',
  ACCESS_DENIED = 'ACCESS_DENIED',
  AUTHENTICATION = 'AUTHENTICATION',
  ANOMALY_DETECTED = 'ANOMALY_DETECTED',
  POLICY_VIOLATION = 'POLICY_VIOLATION',
  DEVICE_COMPLIANCE = 'DEVICE_COMPLIANCE',
  THREAT_DETECTED = 'THREAT_DETECTED'
}

export enum EventSeverity {
  INFO = 'INFO',
  WARNING = 'WARNING',
  ERROR = 'ERROR',
  CRITICAL = 'CRITICAL'
}

// ============================================================================
// Configuration Types
// ============================================================================

export interface WIAZeroTrustConfig {
  strictMode: boolean;
  defaultDeny: boolean;
  continuousVerification: boolean;
  devicePostureChecks: boolean;
  microsegmentation: boolean;
  encryptionInTransit: boolean;
  encryptionAtRest: boolean;
  auditLogging: boolean;
  threatDetection: boolean;
  minimumTrustScore: number;
  sessionTimeout: number;
  reAuthInterval: number;
}
