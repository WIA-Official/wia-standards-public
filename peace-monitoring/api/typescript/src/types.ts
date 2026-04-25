/**
 * WIA-UNI-015: Peace Monitoring Standard
 * TypeScript Type Definitions
 * Version: 1.0.0
 */

// ============================================================================
// Enumerations
// ============================================================================

export enum MonitoringType {
  ARMS_INVENTORY = "ARMS_INVENTORY",
  TROOP_MOVEMENT = "TROOP_MOVEMENT",
  DMZ_SENSOR = "DMZ_SENSOR",
  CONFIDENCE_BUILDING = "CONFIDENCE_BUILDING",
  VERIFICATION_REQUEST = "VERIFICATION_REQUEST",
  INCIDENT_REPORT = "INCIDENT_REPORT"
}

export enum GeographicZone {
  DMZ = "DMZ",
  NLL = "NLL",
  NORTH_KOREA = "NORTH_KOREA",
  SOUTH_KOREA = "SOUTH_KOREA",
  JSA = "JSA",
  INTERNATIONAL = "INTERNATIONAL"
}

export enum EventStatus {
  ACTIVE = "ACTIVE",
  RESOLVED = "RESOLVED",
  UNDER_INVESTIGATION = "UNDER_INVESTIGATION",
  VERIFIED = "VERIFIED",
  ESCALATED = "ESCALATED"
}

export enum VerificationStatus {
  DECLARED = "DECLARED",
  INSPECTION_SCHEDULED = "INSPECTION_SCHEDULED",
  VERIFIED = "VERIFIED",
  DISCREPANCY_FOUND = "DISCREPANCY_FOUND",
  PENDING_CLARIFICATION = "PENDING_CLARIFICATION"
}

export enum SensorType {
  SEISMIC = "SEISMIC",
  ACOUSTIC = "ACOUSTIC",
  RADAR = "RADAR",
  INFRARED = "INFRARED",
  RADIATION = "RADIATION",
  CHEMICAL = "CHEMICAL",
  OPTICAL = "OPTICAL"
}

export enum AlertLevel {
  NONE = "NONE",
  LOW = "LOW",
  MEDIUM = "MEDIUM",
  HIGH = "HIGH",
  CRITICAL = "CRITICAL"
}

export enum ComplianceStatus {
  COMPLIANT = "COMPLIANT",
  NON_COMPLIANT = "NON_COMPLIANT",
  EXEMPTED = "EXEMPTED",
  UNDER_REVIEW = "UNDER_REVIEW"
}

export enum CBMType {
  MILITARY_HOTLINE_TEST = "MILITARY_HOTLINE_TEST",
  JOINT_INSPECTION = "JOINT_INSPECTION",
  INFORMATION_EXCHANGE = "INFORMATION_EXCHANGE",
  NOTIFICATION_OF_EXERCISES = "NOTIFICATION_OF_EXERCISES",
  JOINT_SEARCH_AND_RESCUE = "JOINT_SEARCH_AND_RESCUE",
  DMZ_DEMINING = "DMZ_DEMINING",
  CULTURAL_EXCHANGE = "CULTURAL_EXCHANGE"
}

export type Party = "NORTH_KOREA" | "SOUTH_KOREA";
export type MovementType = "DEPLOYMENT" | "WITHDRAWAL" | "ROTATION" | "EXERCISE";
export type UnitType = "INFANTRY" | "ARMOR" | "ARTILLERY" | "AIR_DEFENSE" | "SUPPORT";
export type CommandLevel = "BATTALION" | "REGIMENT" | "BRIGADE" | "DIVISION" | "CORPS";

// ============================================================================
// Core Data Types
// ============================================================================

export interface GeoCoordinates {
  latitude: number;
  longitude: number;
  altitude?: number;
  accuracy?: number;
  datum?: "WGS84" | "Korea2000";
}

export interface Location {
  name: string;
  coordinates: GeoCoordinates;
  zone: GeographicZone;
  administrativeArea?: string;
  militaryGrid?: string;
}

export interface Attachment {
  attachmentId: string;
  type: "PHOTO" | "VIDEO" | "DOCUMENT" | "SENSOR_DATA" | "REPORT";
  filename: string;
  mimeType: string;
  sizeBytes: number;
  hash: string;
  uploadedBy: string;
  uploadTimestamp: string;
  classification: "PUBLIC" | "RESTRICTED" | "CONFIDENTIAL" | "SECRET";
  encryptionUsed: boolean;
}

// ============================================================================
// Monitoring Events
// ============================================================================

export interface MonitoringEvent {
  monitoringId: string;
  timestamp: string;
  type: MonitoringType;
  zone: GeographicZone;
  coordinates: GeoCoordinates;
  description: string;
  status: EventStatus;
  verificationRequired: boolean;
  reportedBy: string;
  confidenceLevel: number;
  attachments?: Attachment[];
}

// ============================================================================
// Arms Inventory
// ============================================================================

export interface WeaponSpecifications {
  model: string;
  caliber?: string;
  range?: number;
  operational: boolean;
}

export interface WeaponCategory {
  category: string;
  subcategory?: string;
  quantity: number;
  serialNumbers?: string[];
  specifications?: WeaponSpecifications;
}

export interface ArmsInventory {
  inventoryId: string;
  declaringParty: Party;
  timestamp: string;
  location: Location;
  categories: WeaponCategory[];
  totalItems: number;
  verificationStatus: VerificationStatus;
  verifiedBy?: string[];
  nextInspectionDue?: string;
}

// ============================================================================
// DMZ Sensors
// ============================================================================

export interface SensorReading {
  timestamp: string;
  value: number;
  unit: string;
  threshold: number;
  thresholdExceeded: boolean;
  confidence: number;
}

export interface SensorMetadata {
  installationDate: string;
  lastMaintenance: string;
  operationalStatus: "ACTIVE" | "DEGRADED" | "OFFLINE";
  calibrationStatus: "CALIBRATED" | "NEEDS_CALIBRATION";
}

export interface DMZSensorData {
  sensorId: string;
  sensorType: SensorType;
  location: GeoCoordinates;
  timestamp: string;
  readings: SensorReading[];
  alertLevel: AlertLevel;
  processingStatus: "RAW" | "PROCESSED" | "ANALYZED";
  metadata: SensorMetadata;
}

// ============================================================================
// Troop Movements
// ============================================================================

export interface MilitaryUnit {
  unitId: string;
  unitType: UnitType;
  personnelCount: number;
  equipmentCount: number;
  commandLevel: CommandLevel;
}

export interface TroopMovement {
  movementId: string;
  timestamp: string;
  party: Party;
  movementType: MovementType;
  origin: Location;
  destination: Location;
  units: MilitaryUnit[];
  estimatedDuration: number;
  purpose?: string;
  notificationGiven: boolean;
  observersInvited: string[];
  complianceStatus: ComplianceStatus;
}

// ============================================================================
// Confidence Building Measures
// ============================================================================

export interface Participant {
  party: string;
  role: "LEAD" | "PARTICIPANT" | "OBSERVER";
  representatives: string[];
}

export interface Outcome {
  timestamp: string;
  description: string;
  agreementsReached: string[];
  followUpRequired: boolean;
}

export interface ConfidenceBuildingMeasure {
  cbmId: string;
  type: CBMType;
  initiatingParty: Party | "JOINT";
  timestamp: string;
  status: "PROPOSED" | "SCHEDULED" | "IN_PROGRESS" | "COMPLETED" | "CANCELLED";
  participants: Participant[];
  objectives: string[];
  outcomes?: Outcome[];
}

// ============================================================================
// Verification
// ============================================================================

export interface Finding {
  category: string;
  observation: string;
  evidenceType: "VISUAL" | "SENSOR_DATA" | "DOCUMENTATION" | "TESTIMONY";
  evidenceId?: string;
  conformity: "CONFORMING" | "NON_CONFORMING" | "UNCLEAR";
}

export interface VerificationRecord {
  verificationId: string;
  targetEventId: string;
  verifierOrganization: string;
  verificationDate: string;
  methodology: string[];
  findings: Finding[];
  conclusion: "VERIFIED" | "PARTIALLY_VERIFIED" | "NOT_VERIFIED" | "INCONCLUSIVE";
  digitalSignature: string;
}

export interface VerificationRequest {
  requestId: string;
  requestType: "ARMS_INSPECTION" | "SITE_VISIT" | "TROOP_COUNT_VERIFICATION" | "DEMILITARIZATION_CHECK";
  targetLocation: Location;
  proposedDate: string;
  requestedBy: string;
  respondent: Party;
  observersRequired: string[];
  justification: string;
  priority: "CRITICAL" | "HIGH" | "MEDIUM" | "LOW";
  status?: "SUBMITTED" | "UNDER_REVIEW" | "ACCEPTED" | "CONDITIONAL" | "REJECTED";
}

// ============================================================================
// API Request/Response Types
// ============================================================================

export interface PaginationParams {
  page?: number;
  limit?: number;
}

export interface PaginationInfo {
  page: number;
  limit: number;
  total: number;
  totalPages: number;
}

export interface MonitoringEventsQuery extends PaginationParams {
  type?: MonitoringType;
  zone?: GeographicZone;
  startDate?: string;
  endDate?: string;
  status?: EventStatus;
}

export interface MonitoringEventsResponse {
  data: MonitoringEvent[];
  pagination: PaginationInfo;
}

export interface ArmsInventoryQuery extends PaginationParams {
  party?: Party;
  category?: string;
  verificationStatus?: VerificationStatus;
  asOfDate?: string;
}

export interface ArmsInventoryResponse {
  data: ArmsInventory[];
  pagination?: PaginationInfo;
}

export interface SensorDataQuery extends PaginationParams {
  sensorId?: string;
  sensorType?: SensorType;
  zone?: GeographicZone;
  alertLevel?: AlertLevel;
  startTime?: string;
  endTime?: string;
}

export interface SensorDataResponse {
  data: DMZSensorData[];
  pagination?: PaginationInfo;
}

export interface ThreatAssessment {
  threatLevel: AlertLevel;
  score: number;
  maxScore: number;
  factors: {
    troopConcentration: number;
    weaponDeployment: number;
    proximityToDMZ: number;
    recentIncidents: number;
  };
  recommendation: string;
  timestamp: string;
}

// ============================================================================
// WebSocket Event Types
// ============================================================================

export interface WebSocketMessage {
  channel: string;
  eventType: string;
  timestamp: string;
  data: any;
}

export interface WebSocketSubscription {
  type: 'subscribe' | 'unsubscribe';
  channels: string[];
}

export interface WebSocketAuth {
  type: 'authenticate';
  token: string;
}

// ============================================================================
// Client Configuration
// ============================================================================

export interface PeaceMonitoringConfig {
  apiKey: string;
  baseURL?: string;
  environment?: 'production' | 'staging' | 'development';
  timeout?: number;
  region?: 'global' | 'asia-pacific';
}

export interface APIResponse<T = any> {
  data?: T;
  error?: {
    code: string;
    message: string;
    timestamp: string;
    requestId: string;
    details?: any;
  };
}

// ============================================================================
// Verifiable Credentials
// ============================================================================

export interface VerifiableCredential {
  '@context': string[];
  id: string;
  type: string[];
  issuer: string;
  issuanceDate: string;
  expirationDate: string;
  credentialSubject: {
    id: string;
    type: string;
    name: string;
    organization: string;
    clearanceLevel: string;
    zones: GeographicZone[];
    permissions: string[];
  };
  proof: {
    type: string;
    created: string;
    proofPurpose: string;
    verificationMethod: string;
  };
}
