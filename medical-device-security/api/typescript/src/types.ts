/**
 * WIA-MED-016: Medical Device Security Standard - TypeScript Types
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Enums
// ============================================================================

export enum DeviceStatus {
  ACTIVE = 'active',
  INACTIVE = 'inactive',
  COMPROMISED = 'compromised',
  QUARANTINED = 'quarantined',
  PATCHING = 'patching',
}

export enum ThreatLevel {
  CRITICAL = 'critical',
  HIGH = 'high',
  MEDIUM = 'medium',
  LOW = 'low',
  INFO = 'info',
}

export enum VulnerabilityStatus {
  OPEN = 'open',
  MITIGATED = 'mitigated',
  PATCHED = 'patched',
  ACCEPTED = 'accepted',
}

export enum DeviceClass {
  CLASS_I = 'class_i',
  CLASS_II = 'class_ii',
  CLASS_III = 'class_iii',
}

// ============================================================================
// Device Types
// ============================================================================

export interface MedicalDevice {
  deviceId: string;
  manufacturer: string;
  model: string;
  serialNumber: string;
  deviceClass: DeviceClass;
  fdaClearance?: string;
  firmwareVersion: string;
  status: DeviceStatus;
  lastScan: string;
  location: DeviceLocation;
  networkInfo: NetworkInfo;
}

export interface DeviceLocation {
  facility: string;
  department: string;
  room?: string;
  floor?: number;
}

export interface NetworkInfo {
  ipAddress: string;
  macAddress: string;
  vlan?: string;
  segmented: boolean;
  encrypted: boolean;
}

// ============================================================================
// Security Types
// ============================================================================

export interface Vulnerability {
  vulnerabilityId: string;
  deviceId: string;
  cveId?: string;
  description: string;
  threatLevel: ThreatLevel;
  status: VulnerabilityStatus;
  discoveredAt: string;
  mitigatedAt?: string;
  cvssScore?: number;
}

export interface SecurityIncident {
  incidentId: string;
  deviceId: string;
  type: 'intrusion' | 'malware' | 'unauthorized_access' | 'data_breach';
  threatLevel: ThreatLevel;
  description: string;
  detectedAt: string;
  resolvedAt?: string;
  response: IncidentResponse[];
}

export interface IncidentResponse {
  action: string;
  performedBy: string;
  timestamp: string;
  notes?: string;
}

export interface SecurityPatch {
  patchId: string;
  deviceId: string;
  version: string;
  releaseDate: string;
  appliedAt?: string;
  status: 'available' | 'scheduled' | 'applied' | 'failed';
  criticalLevel: ThreatLevel;
}

// ============================================================================
// Compliance Types
// ============================================================================

export interface ComplianceCheck {
  checkId: string;
  deviceId: string;
  framework: 'HIPAA' | 'FDA' | 'ISO27001' | 'NIST';
  status: 'compliant' | 'non_compliant' | 'partial';
  checkedAt: string;
  findings: ComplianceFinding[];
}

export interface ComplianceFinding {
  findingId: string;
  requirement: string;
  status: 'pass' | 'fail' | 'na';
  notes?: string;
}

// ============================================================================
// API Types
// ============================================================================

export interface WIAConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  debug?: boolean;
}

export interface APIResponse<T = unknown> {
  success: boolean;
  data?: T;
  error?: APIError;
  timestamp: string;
}

export interface APIError {
  code: string;
  message: string;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    page: number;
    pageSize: number;
    totalPages: number;
    totalCount: number;
  };
}
