/**
 * WIA-ENE-003 Carbon Capture & Storage Standard - Type Definitions
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Basic Types
// ============================================================================

export type Timestamp = string;
export type UUID = string;

// ============================================================================
// System Status Types
// ============================================================================

export enum SystemStatus {
  OPERATING = "OPERATING",
  STANDBY = "STANDBY",
  MAINTENANCE = "MAINTENANCE",
  OFFLINE = "OFFLINE",
  STARTUP = "STARTUP",
  SHUTDOWN = "SHUTDOWN",
  EMERGENCY = "EMERGENCY",
}

export enum CaptureMethod {
  POST_COMBUSTION = "POST_COMBUSTION",
  PRE_COMBUSTION = "PRE_COMBUSTION",
  OXY_FUEL = "OXY_FUEL",
  DIRECT_AIR = "DIRECT_AIR",
  BIOENERGY = "BIOENERGY",
}

export enum SolventType {
  MEA = "MEA",
  PZ = "PZ",
  MDEA = "MDEA",
  AMP = "AMP",
  IONIC_LIQUID = "IONIC_LIQUID",
  SOLID_SORBENT = "SOLID_SORBENT",
}

// ============================================================================
// Capture System Types
// ============================================================================

export interface CaptureSystemData {
  facilityId: string;
  timestamp: Timestamp;
  captureRate: number;
  flowRate: number;
  inletConcentration: number;
  outletConcentration: number;
  solventType: SolventType | string;
  temperature: number;
  pressure: number;
  energyConsumption: number;
  status: SystemStatus;
  method: CaptureMethod;
  operatingHours: number;
}

export interface CaptureUnit {
  unitId: string;
  facilityId: string;
  name: string;
  method: CaptureMethod;
  capacity: number;
  designEfficiency: number;
  installDate: Timestamp;
  lastMaintenance: Timestamp;
  status: SystemStatus;
  specifications: UnitSpecifications;
}

export interface UnitSpecifications {
  absorberHeight: number;
  absorberDiameter: number;
  stripperHeight: number;
  stripperDiameter: number;
  heatExchangerArea: number;
  solventInventory: number;
  maxFlowRate: number;
  designPressure: number;
  designTemperature: number;
}

// ============================================================================
// Storage Site Types
// ============================================================================

export interface StorageSiteData {
  siteId: string;
  timestamp: Timestamp;
  injectionRate: number;
  cumulativeInjected: number;
  reservoirPressure: number;
  temperature: number;
  seismicData?: SeismicData;
  monitoringStatus: MonitoringStatus;
  integrityScore: number;
}

export interface SeismicData {
  timestamp: Timestamp;
  magnitude: number;
  depth: number;
  epicenterLatitude: number;
  epicenterLongitude: number;
  isInduced: boolean;
  analysisStatus: string;
}

export interface MonitoringStatus {
  wellheadPressure: number;
  annulusPressure: number;
  injectionTemperature: number;
  co2Saturation: number;
  plumeMigration: PlumeMigration;
  leakageDetected: boolean;
  lastInspection: Timestamp;
}

export interface PlumeMigration {
  extentNorth: number;
  extentSouth: number;
  extentEast: number;
  extentWest: number;
  verticalMigration: number;
  modeledVsActual: number;
}

export interface StorageSite {
  siteId: string;
  name: string;
  location: GeoLocation;
  formationType: FormationType;
  capacity: number;
  currentVolume: number;
  wells: InjectionWell[];
  permits: Permit[];
  status: SiteStatus;
}

export interface GeoLocation {
  latitude: number;
  longitude: number;
  depth: number;
  country: string;
  region: string;
}

export enum FormationType {
  SALINE_AQUIFER = "SALINE_AQUIFER",
  DEPLETED_OIL = "DEPLETED_OIL",
  DEPLETED_GAS = "DEPLETED_GAS",
  COAL_SEAM = "COAL_SEAM",
  BASALT = "BASALT",
}

export enum SiteStatus {
  EXPLORATION = "EXPLORATION",
  DEVELOPMENT = "DEVELOPMENT",
  OPERATIONAL = "OPERATIONAL",
  CLOSED = "CLOSED",
  MONITORING = "MONITORING",
}

export interface InjectionWell {
  wellId: string;
  name: string;
  depth: number;
  casingDiameter: number;
  maxInjectionRate: number;
  currentRate: number;
  status: WellStatus;
  completionDate: Timestamp;
}

export enum WellStatus {
  ACTIVE = "ACTIVE",
  IDLE = "IDLE",
  WORKOVER = "WORKOVER",
  PLUGGED = "PLUGGED",
}

export interface Permit {
  permitId: string;
  type: string;
  issuingAuthority: string;
  issueDate: Timestamp;
  expiryDate: Timestamp;
  conditions: string[];
  status: "ACTIVE" | "EXPIRED" | "PENDING" | "REVOKED";
}

// ============================================================================
// Carbon Credit Types
// ============================================================================

export interface CarbonCredit {
  creditId: string;
  facilityId: string;
  tonnes: number;
  verificationDate: Timestamp;
  expiryDate: Timestamp;
  status: CreditStatus;
  registryId: string;
  vintage: number;
  projectType: string;
  methodology: string;
}

export enum CreditStatus {
  PENDING = "PENDING",
  VERIFIED = "VERIFIED",
  RETIRED = "RETIRED",
  TRANSFERRED = "TRANSFERRED",
  CANCELLED = "CANCELLED",
}

export interface CreditTransaction {
  transactionId: string;
  creditId: string;
  type: TransactionType;
  fromAccount: string;
  toAccount: string;
  tonnes: number;
  pricePerTonne?: number;
  timestamp: Timestamp;
  blockchainHash?: string;
}

export enum TransactionType {
  ISSUANCE = "ISSUANCE",
  TRANSFER = "TRANSFER",
  RETIREMENT = "RETIREMENT",
  CANCELLATION = "CANCELLATION",
}

// ============================================================================
// Compliance Types
// ============================================================================

export interface ComplianceReport {
  reportId: string;
  facilityId: string;
  period: ReportingPeriod;
  totalCaptured: number;
  emissions: EmissionsData;
  complianceStatus: ComplianceStatus;
  generatedDate: Timestamp;
  submittedDate?: Timestamp;
  approvedDate?: Timestamp;
}

export interface ReportingPeriod {
  start: Timestamp;
  end: Timestamp;
  type: "MONTHLY" | "QUARTERLY" | "ANNUAL";
}

export enum ComplianceStatus {
  COMPLIANT = "COMPLIANT",
  NON_COMPLIANT = "NON_COMPLIANT",
  UNDER_REVIEW = "UNDER_REVIEW",
  PENDING_SUBMISSION = "PENDING_SUBMISSION",
}

export interface EmissionsData {
  co2Captured: number;
  co2Emitted: number;
  netReduction: number;
  fugitiveEmissions: number;
  energyEmissions: number;
  transportEmissions: number;
  processingEmissions: number;
}

// ============================================================================
// Performance Types
// ============================================================================

export interface PerformanceMetrics {
  captureEfficiency: number;
  energyIntensity: number;
  availability: number;
  solventMakeup: number;
  operatingCost: number;
  maintenanceCost: number;
  totalCost: number;
}

export interface OperationalData {
  timestamp: Timestamp;
  facilityId: string;
  metrics: PerformanceMetrics;
  alerts: Alert[];
  events: OperationalEvent[];
}

export interface Alert {
  alertId: string;
  severity: AlertSeverity;
  message: string;
  timestamp: Timestamp;
  acknowledged: boolean;
  resolvedAt?: Timestamp;
}

export enum AlertSeverity {
  INFO = "INFO",
  WARNING = "WARNING",
  CRITICAL = "CRITICAL",
  EMERGENCY = "EMERGENCY",
}

export interface OperationalEvent {
  eventId: string;
  type: EventType;
  description: string;
  timestamp: Timestamp;
  duration?: number;
  impact?: string;
}

export enum EventType {
  STARTUP = "STARTUP",
  SHUTDOWN = "SHUTDOWN",
  TRIP = "TRIP",
  MAINTENANCE = "MAINTENANCE",
  INSPECTION = "INSPECTION",
  INCIDENT = "INCIDENT",
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey: string;
  timeout?: number;
  retryAttempts?: number;
}

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: ApiError;
  timestamp: Timestamp;
  requestId: string;
}

export interface ApiError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
}

export interface PaginationParams {
  page: number;
  pageSize: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  pageSize: number;
  totalPages: number;
}
