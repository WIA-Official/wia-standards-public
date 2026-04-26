/**
 * WIA-CITY-015: Access Control System Standard - TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Basic Types
// ============================================================================

/**
 * ISO 8601 timestamp string
 */
export type Timestamp = string;

/**
 * Geographic coordinates
 */
export interface Coordinates {
  latitude: number;
  longitude: number;
}

/**
 * Address information
 */
export interface Address {
  street: string;
  city: string;
  state: string;
  country: string;
  postalCode: string;
  coordinates?: Coordinates;
}

// ============================================================================
// User and Identity
// ============================================================================

/**
 * User types
 */
export enum UserType {
  EMPLOYEE = 'employee',
  CONTRACTOR = 'contractor',
  VISITOR = 'visitor',
  VIP = 'vip',
  ADMIN = 'admin',
  EMERGENCY = 'emergency',
}

/**
 * User status
 */
export enum UserStatus {
  ACTIVE = 'active',
  SUSPENDED = 'suspended',
  EXPIRED = 'expired',
  REVOKED = 'revoked',
}

/**
 * User information
 */
export interface UserInfo {
  userId: string;
  employeeId?: string;
  name: string;
  department?: string;
  position?: string;
  email: string;
  phone: string;
  userType: UserType;
  status: UserStatus;
  photoUrl?: string;
  credentials: Credential[];
  accessLevel: string;
  accessZones: string[];
  schedule?: AccessSchedule;
  validFrom: Timestamp;
  validUntil?: Timestamp;
}

/**
 * Access schedule
 */
export interface AccessSchedule {
  weekdays?: TimeSlot;
  saturday?: TimeSlot;
  sunday?: TimeSlot;
  holidays?: boolean;
  exceptions?: {
    date: string;
    allowed: boolean;
    timeSlot?: TimeSlot;
  }[];
}

/**
 * Time slot
 */
export interface TimeSlot {
  start: string;  // HH:MM format
  end: string;    // HH:MM format
}

// ============================================================================
// Credentials
// ============================================================================

/**
 * Credential types
 */
export enum CredentialType {
  RFID_CARD = 'rfid-card',
  NFC_CARD = 'nfc-card',
  MOBILE_CREDENTIAL = 'mobile-credential',
  FINGERPRINT = 'fingerprint',
  FACE = 'face',
  IRIS = 'iris',
  VEIN = 'vein',
  PIN = 'pin',
  QR_CODE = 'qr-code',
}

/**
 * Credential status
 */
export enum CredentialStatus {
  ACTIVE = 'active',
  SUSPENDED = 'suspended',
  LOST = 'lost',
  STOLEN = 'stolen',
  EXPIRED = 'expired',
  REVOKED = 'revoked',
}

/**
 * Base credential interface
 */
export interface Credential {
  credentialId: string;
  userId: string;
  type: CredentialType;
  status: CredentialStatus;
  issuedDate: Timestamp;
  validFrom: Timestamp;
  validUntil?: Timestamp;
}

/**
 * Access card (RFID/NFC)
 */
export interface AccessCard extends Credential {
  type: CredentialType.RFID_CARD | CredentialType.NFC_CARD;
  cardNumber: string;
  facilityCode?: string;
  wiegandFormat?: 'W26' | 'W34' | 'W37';
  technology: 'EM4100' | 'HID-Prox' | 'MIFARE-Classic' | 'MIFARE-DESFire' | 'FeliCa';
  printedName?: string;
  printedPhoto?: string;
}

/**
 * Mobile credential
 */
export interface MobileCredential extends Credential {
  type: CredentialType.MOBILE_CREDENTIAL;
  deviceId: string;
  platform: 'iOS' | 'Android';
  appVersion: string;
  publicKey: string;
  lastSyncDate: Timestamp;
}

/**
 * Biometric credential
 */
export interface BiometricCredential extends Credential {
  type: CredentialType.FINGERPRINT | CredentialType.FACE | CredentialType.IRIS | CredentialType.VEIN;
  templateHash: string;
  templateVersion: string;
  enrolledDate: Timestamp;
  quality: number;  // 0-100
  algorithm: string;
}

/**
 * PIN credential
 */
export interface PINCredential extends Credential {
  type: CredentialType.PIN;
  pinHash: string;
  pinLength: number;
  lastChanged: Timestamp;
  expirationDate?: Timestamp;
  failedAttempts: number;
}

/**
 * QR code credential (typically for visitors)
 */
export interface QRCodeCredential extends Credential {
  type: CredentialType.QR_CODE;
  qrCode: string;
  singleUse: boolean;
  usedAt?: Timestamp;
}

// ============================================================================
// Biometric Data
// ============================================================================

/**
 * Fingerprint data
 */
export interface FingerprintData {
  finger: 'left-thumb' | 'left-index' | 'left-middle' | 'left-ring' | 'left-little' |
          'right-thumb' | 'right-index' | 'right-middle' | 'right-ring' | 'right-little';
  template: string;  // Encrypted template
  minutiae: number;
  quality: number;   // 0-100
  captureMethod: 'optical' | 'capacitive' | 'ultrasonic';
}

/**
 * Face recognition data
 */
export interface FaceData {
  template: string;  // Encrypted template
  features: number;
  quality: number;   // 0-100
  captureMethod: '2d' | '3d-structured-light' | '3d-tof' | 'thermal';
  livenessScore?: number;  // 0-100
  photoUrl?: string;
}

/**
 * Iris recognition data
 */
export interface IrisData {
  eye: 'left' | 'right' | 'both';
  template: string;  // Encrypted template
  quality: number;   // 0-100
  captureDistance: number;  // cm
}

/**
 * Vein recognition data
 */
export interface VeinData {
  type: 'finger' | 'palm';
  template: string;  // Encrypted template
  quality: number;   // 0-100
  captureMethod: 'near-infrared';
}

/**
 * Biometric performance metrics
 */
export interface BiometricMetrics {
  far: number;  // False Acceptance Rate (%)
  frr: number;  // False Rejection Rate (%)
  eer: number;  // Equal Error Rate (%)
  authTime: number;  // Authentication time (ms)
  livenessDetection: boolean;
}

// ============================================================================
// Access Points
// ============================================================================

/**
 * Access point types
 */
export enum AccessPointType {
  MAIN_ENTRANCE = 'main-entrance',
  SIDE_ENTRANCE = 'side-entrance',
  EMPLOYEE_ENTRANCE = 'employee-entrance',
  EMERGENCY_EXIT = 'emergency-exit',
  PARKING_GATE = 'parking-gate',
  TURNSTILE = 'turnstile',
  ELEVATOR = 'elevator',
  INTERNAL_DOOR = 'internal-door',
  MANTRAP = 'mantrap',
}

/**
 * Lock types
 */
export enum LockType {
  ELECTROMAGNETIC = 'electromagnetic',
  ELECTRIC_STRIKE = 'electric-strike',
  ELECTRIC_BOLT = 'electric-bolt',
  MOTORIZED_LOCK = 'motorized-lock',
  SMART_LOCK = 'smart-lock',
}

/**
 * Reader information
 */
export interface ReaderInfo {
  readerId: string;
  type: 'rfid' | 'nfc' | 'biometric' | 'pin-pad' | 'qr-scanner' | 'combo';
  model: string;
  manufacturer: string;
  firmwareVersion: string;
  ipAddress?: string;
  status: 'online' | 'offline' | 'error';
  lastHeartbeat: Timestamp;
}

/**
 * Lock information
 */
export interface LockInfo {
  lockId: string;
  type: LockType;
  holdingForce?: number;  // lbs or kg
  status: 'locked' | 'unlocked' | 'error';
  tamperStatus: 'normal' | 'tampered';
}

/**
 * Door sensor
 */
export interface DoorSensor {
  sensorId: string;
  type: 'magnetic-contact' | 'mechanical-switch';
  status: 'closed' | 'open' | 'error';
  forcedOpen: boolean;
  heldOpen: boolean;
}

/**
 * Access point devices
 */
export interface AccessPointDevices {
  reader?: ReaderInfo;
  lock: LockInfo;
  doorSensor: DoorSensor;
  camera?: {
    cameraId: string;
    resolution: string;
    recordingEnabled: boolean;
    streamUrl?: string;
  };
  rex?: {  // Request to Exit
    rexId: string;
    type: 'motion-sensor' | 'push-button';
    status: 'inactive' | 'active';
  };
}

/**
 * Access control settings
 */
export interface AccessControlSettings {
  normallyOpen: boolean;
  unlockDuration: number;  // seconds
  reLockDelay: number;     // seconds
  antiPassback: boolean;
  twoPersonRule: boolean;
  mantrapInterlock: boolean;
  forcedOpenAlarm: boolean;
  heldOpenAlarm: boolean;
  heldOpenTimeout: number; // seconds
}

/**
 * Access point information
 */
export interface AccessPoint {
  accessPointId: string;
  name: string;
  location: {
    building: string;
    floor: string;
    zone: string;
    coordinates?: Coordinates;
  };
  type: AccessPointType;
  devices: AccessPointDevices;
  accessControl: AccessControlSettings;
  allowedCredentialTypes: CredentialType[];
  requireMultiFactor: boolean;
  status: 'operational' | 'maintenance' | 'disabled';
}

// ============================================================================
// Access Zones
// ============================================================================

/**
 * Zone security level
 */
export enum SecurityLevel {
  PUBLIC = 'public',
  LOW = 'low',
  MEDIUM = 'medium',
  HIGH = 'high',
  CRITICAL = 'critical',
}

/**
 * Access zone
 */
export interface AccessZone {
  zoneId: string;
  name: string;
  description: string;
  securityLevel: SecurityLevel;
  accessPoints: string[];  // accessPointIds
  parentZone?: string;     // zoneId
  requireEscort: boolean;
  requireApproval: boolean;
  maxOccupancy?: number;
  currentOccupancy?: number;
}

// ============================================================================
// Access Logs
// ============================================================================

/**
 * Access actions
 */
export enum AccessAction {
  ENTRY = 'entry',
  EXIT = 'exit',
  DENIED = 'denied',
  FORCED_OPEN = 'forced-open',
  HELD_OPEN = 'held-open',
  UNLOCKED_REMOTELY = 'unlocked-remotely',
}

/**
 * Access result
 */
export enum AccessResult {
  GRANTED = 'granted',
  DENIED_NO_CREDENTIAL = 'denied-no-credential',
  DENIED_INVALID_CREDENTIAL = 'denied-invalid-credential',
  DENIED_EXPIRED = 'denied-expired',
  DENIED_TIME_RESTRICTION = 'denied-time-restriction',
  DENIED_ZONE_RESTRICTION = 'denied-zone-restriction',
  DENIED_ANTI_PASSBACK = 'denied-anti-passback',
  DENIED_TWO_PERSON_RULE = 'denied-two-person-rule',
  DENIED_BIOMETRIC_MISMATCH = 'denied-biometric-mismatch',
  ERROR = 'error',
}

/**
 * Access log entry
 */
export interface AccessLog {
  logId: string;
  timestamp: Timestamp;
  accessPointId: string;
  accessPointName: string;
  userId?: string;
  userName?: string;
  credentialType?: CredentialType;
  credentialId?: string;
  action: AccessAction;
  result: AccessResult;
  reason?: string;
  doorOpenDuration?: number;  // seconds
  photoCapture?: string;      // URL or base64
  videoClip?: string;         // URL
  accompanyingPersons?: string[];  // userIds
  metadata?: Record<string, any>;
}

// ============================================================================
// Visitors
// ============================================================================

/**
 * Visitor status
 */
export enum VisitorStatus {
  PRE_REGISTERED = 'pre-registered',
  CHECKED_IN = 'checked-in',
  CHECKED_OUT = 'checked-out',
  EXPIRED = 'expired',
  CANCELLED = 'cancelled',
}

/**
 * Visitor type
 */
export enum VisitorType {
  BUSINESS = 'business',
  INTERVIEW = 'interview',
  DELIVERY = 'delivery',
  MAINTENANCE = 'maintenance',
  VIP = 'vip',
  OTHER = 'other',
}

/**
 * Host information
 */
export interface HostInfo {
  employeeId: string;
  name: string;
  department: string;
  email: string;
  phone: string;
}

/**
 * Visitor information
 */
export interface Visitor {
  visitorId: string;
  name: string;
  company?: string;
  phone: string;
  email?: string;
  idCardType?: string;
  idCardNumber?: string;
  purpose: string;
  visitorType: VisitorType;
  host: HostInfo;
  visitDate: string;  // YYYY-MM-DD
  timeSlot: TimeSlot;
  accessZones: string[];
  credential?: QRCodeCredential;
  status: VisitorStatus;
  checkInTime?: Timestamp;
  checkOutTime?: Timestamp;
  photoUrl?: string;
  notes?: string;
}

// ============================================================================
// Parking Access Control
// ============================================================================

/**
 * Vehicle information
 */
export interface VehicleInfo {
  vehicleId: string;
  licensePlate: string;
  make?: string;
  model?: string;
  color?: string;
  ownerId: string;
  ownerName: string;
  vehicleType: 'car' | 'motorcycle' | 'truck' | 'van' | 'bus';
  rfidTag?: string;
}

/**
 * Parking pass types
 */
export enum ParkingPassType {
  MONTHLY = 'monthly',
  DAILY = 'daily',
  VISITOR = 'visitor',
  RESERVED = 'reserved',
  VIP = 'vip',
}

/**
 * Parking pass
 */
export interface ParkingPass {
  passId: string;
  vehicleId: string;
  passType: ParkingPassType;
  validFrom: Timestamp;
  validUntil: Timestamp;
  assignedSlot?: string;
  allowedZones: string[];  // parking zone IDs
  status: 'active' | 'suspended' | 'expired';
}

/**
 * Parking entry/exit record
 */
export interface ParkingRecord {
  recordId: string;
  vehicleId: string;
  licensePlate: string;
  action: 'entry' | 'exit';
  timestamp: Timestamp;
  gateId: string;
  gateName: string;
  parkingZone?: string;
  parkingSlot?: string;
  duration?: number;  // minutes
  fee?: number;
  paymentStatus?: 'unpaid' | 'paid';
  photoUrl?: string;
}

/**
 * Parking lot status
 */
export interface ParkingLotStatus {
  parkingLotId: string;
  name: string;
  totalSlots: number;
  occupiedSlots: number;
  availableSlots: number;
  occupancyRate: number;  // percentage
  floors?: {
    floor: string;
    totalSlots: number;
    occupiedSlots: number;
    availableSlots: number;
  }[];
  lastUpdated: Timestamp;
}

// ============================================================================
// Elevator Access Control
// ============================================================================

/**
 * Elevator call request
 */
export interface ElevatorCallRequest {
  userId: string;
  credentialId: string;
  currentFloor: string;
  destinationFloor: string;
  timestamp: Timestamp;
  allowed: boolean;
  reason?: string;
}

/**
 * Elevator access rules
 */
export interface ElevatorAccessRules {
  userId: string;
  allowedFloors: string[];
  deniedFloors: string[];
  timeRestrictions?: AccessSchedule;
  requireEscort?: string[];  // floors requiring escort
}

// ============================================================================
// Emergency Modes
// ============================================================================

/**
 * Emergency types
 */
export enum EmergencyType {
  FIRE = 'fire',
  INTRUSION = 'intrusion',
  LOCKDOWN = 'lockdown',
  EVACUATION = 'evacuation',
  NATURAL_DISASTER = 'natural-disaster',
  MEDICAL = 'medical',
  OTHER = 'other',
}

/**
 * Emergency action
 */
export enum EmergencyAction {
  UNLOCK_ALL = 'unlock-all',
  LOCK_ALL = 'lock-all',
  UNLOCK_EXITS = 'unlock-exits',
  LOCK_PERIMETER = 'lock-perimeter',
  DISABLE_ELEVATORS = 'disable-elevators',
  SOUND_ALARM = 'sound-alarm',
  NOTIFY_AUTHORITIES = 'notify-authorities',
}

/**
 * Emergency mode status
 */
export interface EmergencyMode {
  emergencyId: string;
  type: EmergencyType;
  activatedAt: Timestamp;
  activatedBy: string;  // userId
  deactivatedAt?: Timestamp;
  deactivatedBy?: string;  // userId
  status: 'active' | 'resolved';
  actions: EmergencyAction[];
  affectedZones: string[];
  affectedAccessPoints: string[];
  notes?: string;
}

// ============================================================================
// Monitoring and Analytics
// ============================================================================

/**
 * Real-time dashboard
 */
export interface RealtimeDashboard {
  buildingId: string;
  timestamp: Timestamp;
  occupancy: {
    current: number;
    max: number;
    rate: number;  // percentage
    byZone: {
      zoneId: string;
      zoneName: string;
      occupancy: number;
    }[];
  };
  accessActivity: {
    totalToday: number;
    entries: number;
    exits: number;
    denied: number;
    lastHourActivity: number;
  };
  visitors: {
    current: number;
    checkedIn: number;
    checkedOut: number;
    pending: number;
  };
  parking: {
    totalSlots: number;
    occupied: number;
    available: number;
    occupancyRate: number;
  };
  alerts: AlertInfo[];
  systemHealth: {
    onlineReaders: number;
    offlineReaders: number;
    onlineControllers: number;
    offlineControllers: number;
    criticalIssues: number;
  };
}

/**
 * Alert types
 */
export enum AlertType {
  UNAUTHORIZED_ACCESS = 'unauthorized-access',
  FORCED_DOOR = 'forced-door',
  DOOR_HELD_OPEN = 'door-held-open',
  TAILGATING = 'tailgating',
  DEVICE_OFFLINE = 'device-offline',
  DEVICE_TAMPER = 'device-tamper',
  CREDENTIAL_CLONED = 'credential-cloned',
  MULTIPLE_FAILURES = 'multiple-failures',
  ZONE_BREACH = 'zone-breach',
  EMERGENCY_ACTIVATED = 'emergency-activated',
}

/**
 * Alert severity
 */
export enum AlertSeverity {
  INFO = 'info',
  WARNING = 'warning',
  CRITICAL = 'critical',
  EMERGENCY = 'emergency',
}

/**
 * Alert information
 */
export interface AlertInfo {
  alertId: string;
  timestamp: Timestamp;
  type: AlertType;
  severity: AlertSeverity;
  accessPointId?: string;
  accessPointName?: string;
  zoneId?: string;
  zoneName?: string;
  userId?: string;
  userName?: string;
  message: string;
  details?: any;
  acknowledged: boolean;
  acknowledgedBy?: string;
  acknowledgedAt?: Timestamp;
  resolvedAt?: Timestamp;
  actionTaken?: string;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

/**
 * Generic API response
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    timestamp: Timestamp;
    version: string;
  };
}

/**
 * Pagination parameters
 */
export interface PaginationParams {
  page: number;
  limit: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  limit: number;
  hasMore: boolean;
}

/**
 * Date range filter
 */
export interface DateRangeFilter {
  startDate: Timestamp;
  endDate: Timestamp;
}

// ============================================================================
// Control Commands
// ============================================================================

/**
 * Grant access command
 */
export interface GrantAccessCommand {
  userId: string;
  accessZones: string[];
  validFrom: Timestamp;
  validUntil?: Timestamp;
  timeRestrictions?: AccessSchedule;
  requireEscort?: boolean;
  reason?: string;
}

/**
 * Revoke access command
 */
export interface RevokeAccessCommand {
  userId: string;
  accessZones?: string[];  // if empty, revoke all
  reason: string;
  immediate: boolean;
}

/**
 * Unlock door command
 */
export interface UnlockDoorCommand {
  accessPointId: string;
  duration?: number;  // seconds, 0 = permanent until re-locked
  reason: string;
  authorizedBy: string;  // userId
}

/**
 * Activate emergency mode command
 */
export interface ActivateEmergencyCommand {
  emergencyType: EmergencyType;
  actions: EmergencyAction[];
  affectedZones?: string[];
  affectedAccessPoints?: string[];
  reason: string;
  activatedBy: string;  // userId
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  Address,
};
