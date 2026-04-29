/**
 * WIA-TIME-032: Time Access Control - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Geographic Types
// ============================================================================

/**
 * Geographic coordinate (latitude/longitude)
 */
export interface GeoCoordinate {
  lat: number;  // Latitude (-90 to 90)
  lon: number;  // Longitude (-180 to 180)
  alt?: number; // Altitude in meters (optional)
}

/**
 * Geographic boundary types
 */
export type GeoBoundary = GeoCircle | GeoBox | GeoPolygon;

export interface GeoCircle {
  type: 'circle';
  center: GeoCoordinate;
  radius: number; // Meters
}

export interface GeoBox {
  type: 'box';
  southwest: GeoCoordinate;
  northeast: GeoCoordinate;
}

export interface GeoPolygon {
  type: 'polygon';
  vertices: GeoCoordinate[];
}

// ============================================================================
// Access Control Enums
// ============================================================================

/**
 * Access permission types
 */
export enum Permission {
  READ = 'READ',                    // Observe timeline
  INTERACT = 'INTERACT',            // Limited interaction
  MODIFY = 'MODIFY',                // Timeline alteration
  ADMIN = 'ADMIN',                  // Manage access control
  EMERGENCY = 'EMERGENCY',          // Override restrictions
  GRANT = 'GRANT',                  // Grant access to others
  REVOKE = 'REVOKE',                // Revoke access
  AUDIT = 'AUDIT',                  // View audit logs
  CONFIGURE = 'CONFIGURE'           // System configuration
}

/**
 * Temporal clearance levels
 */
export enum ClearanceLevel {
  OBSERVER = 1,        // View-only access
  RESEARCHER = 2,      // Academic investigation
  OPERATOR = 3,        // Standard time travel
  ADMINISTRATOR = 4,   // System management
  GUARDIAN = 5         // Emergency authority
}

/**
 * Era classification levels
 */
export enum EraClass {
  PUBLIC = 'PUBLIC',           // Open access
  RESTRICTED = 'RESTRICTED',   // Requires oversight
  PROTECTED = 'PROTECTED',     // High sensitivity
  CLASSIFIED = 'CLASSIFIED',   // Extreme sensitivity
  FORBIDDEN = 'FORBIDDEN'      // Guardian only
}

/**
 * Protection levels for events
 */
export enum ProtectionLevel {
  NONE = 0,           // No special protection
  LOW = 1,            // Basic monitoring
  MEDIUM = 2,         // Enhanced restrictions
  HIGH = 3,           // Strict access control
  CRITICAL = 4,       // Multi-party authorization
  ABSOLUTE = 5        // Guardian only
}

/**
 * Access violation types
 */
export enum ViolationType {
  UNAUTHORIZED_ACCESS = 'UNAUTHORIZED_ACCESS',
  CLEARANCE_VIOLATION = 'CLEARANCE_VIOLATION',
  TIME_EXCEEDED = 'TIME_EXCEEDED',
  AREA_VIOLATION = 'AREA_VIOLATION',
  PERMISSION_VIOLATION = 'PERMISSION_VIOLATION',
  INTERACTION_VIOLATION = 'INTERACTION_VIOLATION',
  MODIFICATION_VIOLATION = 'MODIFICATION_VIOLATION',
  EQUIPMENT_TAMPERING = 'EQUIPMENT_TAMPERING',
  COMMUNICATION_VIOLATION = 'COMMUNICATION_VIOLATION',
  PARADOX_ATTEMPT = 'PARADOX_ATTEMPT'
}

/**
 * Risk levels
 */
export type RiskLevel = 'low' | 'medium' | 'high' | 'critical';

/**
 * Impact levels
 */
export type ImpactLevel = 'minimal' | 'moderate' | 'significant' | 'severe';

// ============================================================================
// Authentication
// ============================================================================

/**
 * Biometric authentication data
 */
export interface BiometricData {
  fingerprint?: string;      // Fingerprint hash
  retinalScan?: string;      // Retinal pattern hash
  dnaSample?: string;        // DNA sequence hash
  voiceprint?: string;       // Voice pattern hash
  brainwave?: string;        // EEG signature hash
  heartRhythm?: string;      // Cardiac pattern hash
}

/**
 * Authentication credentials
 */
export interface AuthCredentials {
  password?: string;
  biometric?: BiometricData;
  temporalToken?: string;
  physicalKey?: string;
  mfaCode?: string;
  quantumKey?: string;
}

/**
 * Authentication request
 */
export interface AuthenticationRequest {
  userId: string;
  credentials: AuthCredentials;
  deviceId: string;
  location: GeoCoordinate;
  timestamp?: Date;
}

/**
 * Authentication response
 */
export interface AuthenticationResponse {
  authenticated: boolean;
  sessionToken?: string;
  expiresAt?: Date;
  mfaRequired?: string[];
  clearanceLevel?: ClearanceLevel;
  error?: string;
}

/**
 * Authentication session
 */
export interface AuthSession {
  sessionId: string;
  userId: string;
  authenticated: Date;
  expiresAt: Date;
  mfaFactors: string[];
  clearanceLevel: ClearanceLevel;
  activeAccess?: TemporalAccess[];
  renewalCount: number;
  lastActivity: Date;
}

/**
 * Authentication method
 */
export type AuthenticationMethod =
  | 'password'
  | 'biometric'
  | 'temporal-token'
  | 'physical-key'
  | 'mfa'
  | 'quantum-key';

// ============================================================================
// Authorization
// ============================================================================

/**
 * User role in RBAC system
 */
export interface UserRole {
  id: string;
  name: string;
  clearanceLevel: ClearanceLevel;
  permissions: Permission[];
  description?: string;
}

/**
 * Access attributes for ABAC
 */
export interface AccessAttributes {
  user: {
    id: string;
    clearanceLevel: ClearanceLevel;
    department: string;
    citizenship: string[];
    securityClearance: string[];
    trainingCompleted: string[];
  };

  resource: {
    time: Date;
    location: GeoCoordinate;
    eraClassification: EraClass;
    protectionLevel: ProtectionLevel;
    sensitivityScore: number;
  };

  environment: {
    currentTime: Date;
    requestOrigin: string;
    riskLevel: RiskLevel;
    activeAlerts: Alert[];
    timelineStability: number;
  };

  action: {
    type: Permission;
    duration: number;
    purpose: string;
    impact: ImpactLevel;
  };
}

/**
 * Authorization token
 */
export interface AuthorizationToken {
  token: string;
  userId: string;
  clearanceLevel: ClearanceLevel;
  permissions: Permission[];
  issuedAt: Date;
  expiresAt: Date;
  scope: {
    eraRange?: { start: Date; end: Date };
    geoRestrictions?: GeoBoundary[];
    maxDuration?: number;
  };
}

// ============================================================================
// Access Control
// ============================================================================

/**
 * Temporal access request
 */
export interface AccessRequest {
  userId: string;
  sessionToken: string;

  // Target
  targetTime: Date;
  targetLocation: GeoCoordinate;

  // Details
  purpose: string;
  duration: number;  // Seconds
  permissions: Permission[];

  // Supporting documentation
  researchProposal?: string;
  supervisorApproval?: string;
  emergencyJustification?: string;
}

/**
 * Access grant
 */
export interface AccessGrant {
  grantId: string;
  userId: string;
  granted: Date;
  expiresAt: Date;

  // Authorized access
  targetTime: Date;
  targetLocation: GeoCoordinate;
  duration: number;
  permissions: Permission[];

  // Token
  accessToken: string;

  // Restrictions
  restrictions: Restriction[];
  warnings: string[];

  // Monitoring
  monitoringRequired: boolean;
  realTimeTracking: boolean;
}

/**
 * Access denial
 */
export interface AccessDenial {
  denialId: string;
  userId: string;
  deniedAt: Date;
  reason: string;
  denialCode: string;
  appealProcess?: string;
  requiredActions?: string[];
}

/**
 * Access response (grant or denial)
 */
export interface AccessResponse {
  granted: boolean;
  grant?: AccessGrant;
  denial?: AccessDenial;
}

/**
 * Temporal access (active)
 */
export interface TemporalAccess {
  accessId: string;
  userId: string;
  grantId: string;

  // Access details
  startTime: Date;
  targetTime: Date;
  targetLocation: GeoCoordinate;
  expiresAt: Date;

  // Status
  status: 'active' | 'expired' | 'revoked' | 'completed';
  permissions: Permission[];

  // Monitoring
  currentLocation?: GeoCoordinate;
  lastUpdate?: Date;
  violations: AccessViolation[];
}

/**
 * Access restriction
 */
export interface Restriction {
  type: 'time' | 'location' | 'permission' | 'interaction' | 'custom';
  description: string;
  enforcement: 'advisory' | 'warning' | 'blocking';
  details: Record<string, unknown>;
}

// ============================================================================
// Protected Eras and Events
// ============================================================================

/**
 * Restricted era definition
 */
export interface RestrictedEra {
  id: string;
  name: string;
  description: string;

  // Time bounds
  startTime: Date;
  endTime: Date;

  // Access control
  classification: EraClass;
  minClearance: ClearanceLevel;
  allowedPermissions: Permission[];

  // Justification
  reason: string;
  sensitivityScore: number;  // 0-100
}

/**
 * Protected event
 */
export interface ProtectedEvent {
  id: string;
  name: string;
  description: string;

  // Temporal bounds
  startTime: Date;
  endTime: Date;
  bufferBefore: number;     // Seconds
  bufferAfter: number;      // Seconds

  // Geographic bounds
  location: GeoCoordinate;
  radius: number;           // Meters

  // Protection details
  protectionLevel: ProtectionLevel;
  minClearance: ClearanceLevel;
  allowedPermissions: Permission[];

  // Justification
  reason: string;
  historicalSignificance: number;  // 0-100
  paradoxRisk: number;             // 0-100
  timelineSensitivity: number;     // 0-100

  // Oversight
  guardians: string[];
  approvalRequired: boolean;
  realTimeMonitoring: boolean;
}

/**
 * Event category for automatic protection
 */
export type EventCategory =
  | 'assassination'
  | 'natural-disaster'
  | 'scientific-breakthrough'
  | 'personal-moment'
  | 'timeline-branch-point'
  | 'war-crime'
  | 'terrorist-attack';

// ============================================================================
// Geographic Restrictions
// ============================================================================

/**
 * Reason for geographic restriction
 */
export type RestrictionReason =
  | 'national-security'
  | 'privacy'
  | 'cultural-sensitivity'
  | 'safety'
  | 'legal'
  | 'environmental';

/**
 * Geographic restriction
 */
export interface GeographicRestriction {
  id: string;
  name: string;

  // Geographic definition
  boundary: GeoBoundary;
  altitude?: { min: number; max: number };  // Meters

  // Temporal scope
  timeRange?: { start: Date; end: Date };

  // Access control
  minClearance: ClearanceLevel;
  allowedNationalities?: string[];
  requiredPermissions: Permission[];

  // Reason
  restrictionReason: RestrictionReason;
  sensitivity: number;  // 0-100
}

// ============================================================================
// Access Revocation
// ============================================================================

/**
 * Revocation severity
 */
export type RevocationSeverity = 'immediate' | 'scheduled' | 'gradual';

/**
 * Revocation scope
 */
export type RevocationScope = 'all' | 'specific-clearance' | 'specific-era';

/**
 * Revocation duration
 */
export type RevocationDuration = 'permanent' | 'temporary';

/**
 * Revocation request
 */
export interface RevocationRequest {
  targetUserId: string;
  initiatorUserId: string;
  reason: string;
  severity: RevocationSeverity;
  scope: RevocationScope;
  duration: RevocationDuration;
  temporaryDuration?: number;  // Seconds

  // For active travelers
  recallRequired: boolean;
  recallGracePeriod?: number;  // Seconds

  // Notifications
  notifyUser: boolean;
  notifyAuthorities: boolean;
  notifyGuardians: boolean;
}

/**
 * Revocation response
 */
export interface RevocationResponse {
  success: boolean;
  revocationId: string;
  sessionsTerminated: number;
  tokensInvalidated: number;
  recallInitiated: boolean;
  error?: string;
}

/**
 * Active revocation
 */
export interface AccessRevocation {
  revocationId: string;
  userId: string;
  initiatedBy: string;
  timestamp: Date;
  reason: string;
  severity: RevocationSeverity;
  scope: RevocationScope;
  duration: RevocationDuration;
  expiresAt?: Date;
  status: 'active' | 'expired' | 'appealed' | 'overturned';
}

// ============================================================================
// Audit and Monitoring
// ============================================================================

/**
 * Action types for audit logging
 */
export type AuditActionType =
  | 'ACCESS_REQUEST'
  | 'ACCESS_GRANT'
  | 'ACCESS_DENY'
  | 'TIME_TRAVEL'
  | 'RETURN'
  | 'VIOLATION'
  | 'REVOCATION'
  | 'AUTHENTICATION'
  | 'AUTHORIZATION'
  | 'CONFIGURATION_CHANGE';

/**
 * Audit log entry
 */
export interface AuditLog {
  // Record identity
  id: string;
  timestamp: Date;

  // Subject
  userId: string;
  sessionId: string;
  clearanceLevel: ClearanceLevel;

  // Action
  action: string;
  actionType: AuditActionType;

  // Target
  targetTime?: Date;
  targetLocation?: GeoCoordinate;
  targetEra?: string;

  // Decision
  granted: boolean;
  denialReason?: string;

  // Details
  duration?: number;
  permissions?: Permission[];
  purpose?: string;

  // Monitoring
  violations: AccessViolation[];
  alerts: Alert[];

  // Context
  ipAddress: string;
  deviceId: string;
  geolocation: GeoCoordinate;

  // Verification
  signature: string;  // Cryptographic signature
  integrity: string;  // Hash for tamper detection
}

/**
 * Access violation
 */
export interface AccessViolation {
  id: string;
  timestamp: Date;
  userId: string;

  // Violation details
  type: ViolationType;
  severity: 'minor' | 'moderate' | 'severe' | 'critical';
  description: string;

  // Impact
  timelineImpact: number;      // 0-100
  paradoxProbability: number;  // 0-1
  historicalSignificance: number;  // 0-100

  // Response
  automaticAction?: string;
  guardianNotified: boolean;
  investigationOpened: boolean;
}

/**
 * Alert for monitoring system
 */
export interface Alert {
  id: string;
  timestamp: Date;
  severity: 'info' | 'warning' | 'error' | 'critical';
  type: string;
  message: string;
  userId?: string;
  requiresAction: boolean;
  actionTaken?: string;
}

/**
 * Real-time monitoring data
 */
export interface MonitoringData {
  userId: string;
  currentTime: Date;
  currentLocation: GeoCoordinate;
  accessGranted: Date;
  expiresAt: Date;

  // Activity tracking
  interactions: number;
  observations: number;
  modifications: number;

  // Compliance
  withinAuthorizedArea: boolean;
  withinAuthorizedTime: boolean;
  permissionsRespected: boolean;

  // Alerts
  violations: AccessViolation[];
  warnings: string[];

  // Vital signs (for safety)
  heartRate?: number;
  locationAccuracy: number;
  deviceBattery: number;

  // Timeline integrity
  novikovCompliance: number;  // 0-1
  timelineStability: number;  // 0-1
  paradoxRisk: number;        // 0-1
}

// ============================================================================
// Anomaly Detection
// ============================================================================

/**
 * Anomaly type
 */
export type AnomalyType =
  | 'UNUSUAL_FREQUENCY'
  | 'UNUSUAL_ERA'
  | 'UNUSUAL_LOCATION'
  | 'UNUSUAL_DURATION'
  | 'UNUSUAL_PERMISSIONS'
  | 'PATTERN_DEVIATION';

/**
 * Detected anomaly
 */
export interface Anomaly {
  type: AnomalyType;
  severity: 'low' | 'moderate' | 'high' | 'critical';
  description: string;
  confidence: number;  // 0-1
  detectedAt: Date;
  userId: string;
  recommendation?: string;
}

/**
 * User behavior baseline
 */
export interface BehaviorBaseline {
  userId: string;
  frequency: number;              // Average access per week
  commonEras: string[];
  commonLocations: GeoCoordinate[];
  averageDuration: number;        // Seconds
  typicalPermissions: Permission[];
  lastUpdated: Date;
}

// ============================================================================
// Helper Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

/**
 * Pagination parameters
 */
export interface PaginationParams {
  page: number;
  pageSize: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  data: T[];
  total: number;
  page: number;
  pageSize: number;
  totalPages: number;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-TIME-032 error codes
 */
export enum AccessErrorCode {
  AUTHENTICATION_FAILED = 'AC001',
  AUTHORIZATION_DENIED = 'AC002',
  CLEARANCE_INSUFFICIENT = 'AC003',
  ERA_RESTRICTED = 'AC004',
  LOCATION_RESTRICTED = 'AC005',
  ACCESS_REVOKED = 'AC006',
  TOKEN_EXPIRED = 'AC007',
  VIOLATION_DETECTED = 'AC008',
  GUARDIAN_APPROVAL_REQUIRED = 'AC009',
  SYSTEM_EMERGENCY = 'AC010',
  INVALID_REQUEST = 'AC011',
  GEOFENCE_VIOLATION = 'AC012'
}

/**
 * Access control error
 */
export class AccessControlError extends Error {
  constructor(
    public code: AccessErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'AccessControlError';
  }
}

// ============================================================================
// Configuration
// ============================================================================

/**
 * Access control configuration
 */
export interface AccessControlConfig {
  // Authentication
  mfaRequired: boolean;
  mfaFactorCount: number;
  biometricThreshold: number;  // 0-1
  tokenValiditySeconds: number;
  sessionTimeoutSeconds: number;

  // Authorization
  defaultClearance: ClearanceLevel;
  clearanceRenewalDays: number;
  guardianOverrideEnabled: boolean;

  // Monitoring
  realTimeMonitoringEnabled: boolean;
  anomalyDetectionEnabled: boolean;
  violationAlertThreshold: number;

  // Audit
  auditRetentionDays: number;
  auditEncryptionEnabled: boolean;
  auditReplicationEnabled: boolean;

  // Security
  encryptionAlgorithm: string;
  quantumResistantEnabled: boolean;
  penetrationTestingFrequency: number;  // Days
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Default configuration values
 */
export const DEFAULT_CONFIG: AccessControlConfig = {
  mfaRequired: true,
  mfaFactorCount: 3,
  biometricThreshold: 0.9999,
  tokenValiditySeconds: 30,
  sessionTimeoutSeconds: 28800,  // 8 hours
  defaultClearance: ClearanceLevel.OBSERVER,
  clearanceRenewalDays: 365,
  guardianOverrideEnabled: true,
  realTimeMonitoringEnabled: true,
  anomalyDetectionEnabled: true,
  violationAlertThreshold: 3,
  auditRetentionDays: 3650,  // 10 years
  auditEncryptionEnabled: true,
  auditReplicationEnabled: true,
  encryptionAlgorithm: 'AES-256-GCM',
  quantumResistantEnabled: true,
  penetrationTestingFrequency: 90,
};

/**
 * Clearance level requirements
 */
export const CLEARANCE_REQUIREMENTS = {
  [ClearanceLevel.OBSERVER]: {
    trainingHours: 40,
    backgroundCheck: 'basic',
    eraAccess: [EraClass.PUBLIC],
  },
  [ClearanceLevel.RESEARCHER]: {
    trainingHours: 160,
    backgroundCheck: 'enhanced',
    eraAccess: [EraClass.PUBLIC, EraClass.RESTRICTED],
  },
  [ClearanceLevel.OPERATOR]: {
    trainingHours: 400,
    backgroundCheck: 'top-secret',
    eraAccess: [EraClass.PUBLIC, EraClass.RESTRICTED, EraClass.PROTECTED],
  },
  [ClearanceLevel.ADMINISTRATOR]: {
    trainingHours: 1000,
    backgroundCheck: 'top-secret-sci',
    eraAccess: [EraClass.PUBLIC, EraClass.RESTRICTED, EraClass.PROTECTED, EraClass.CLASSIFIED],
  },
  [ClearanceLevel.GUARDIAN]: {
    trainingHours: 2000,
    backgroundCheck: 'exceptional',
    eraAccess: [EraClass.PUBLIC, EraClass.RESTRICTED, EraClass.PROTECTED, EraClass.CLASSIFIED, EraClass.FORBIDDEN],
  },
} as const;

// ============================================================================
// Export All
// ============================================================================

export type {
  GeoCoordinate,
  GeoBoundary,
  GeoCircle,
  GeoBox,
  GeoPolygon,
  BiometricData,
  AuthCredentials,
  AuthenticationRequest,
  AuthenticationResponse,
  AuthSession,
  AuthenticationMethod,
  UserRole,
  AccessAttributes,
  AuthorizationToken,
  AccessRequest,
  AccessGrant,
  AccessDenial,
  AccessResponse,
  TemporalAccess,
  Restriction,
  RestrictedEra,
  ProtectedEvent,
  EventCategory,
  RestrictionReason,
  GeographicRestriction,
  RevocationSeverity,
  RevocationScope,
  RevocationDuration,
  RevocationRequest,
  RevocationResponse,
  AccessRevocation,
  AuditActionType,
  AuditLog,
  AccessViolation,
  Alert,
  MonitoringData,
  AnomalyType,
  Anomaly,
  BehaviorBaseline,
  PaginationParams,
  PaginatedResponse,
  AccessControlConfig,
};

export {
  Permission,
  ClearanceLevel,
  EraClass,
  ProtectionLevel,
  ViolationType,
  AccessErrorCode,
  AccessControlError,
  DEFAULT_CONFIG,
  CLEARANCE_REQUIREMENTS,
};
