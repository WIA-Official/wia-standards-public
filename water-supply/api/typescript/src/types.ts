/**
 * WIA-SOC-008: Water Supply Standard - TypeScript Type Definitions
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
 * UUID v4 string
 */
export type UUID = string;

/**
 * Geographic coordinates (WGS84)
 */
export interface Coordinates {
  lat: number;
  lon: number;
}

// ============================================================================
// System Types
// ============================================================================

/**
 * Certification levels
 */
export enum CertificationLevel {
  BASIC = 'BASIC',
  STANDARD = 'STANDARD',
  ADVANCED = 'ADVANCED'
}

/**
 * System identity and configuration
 */
export interface SystemIdentity {
  systemId: UUID;
  utility: string;
  jurisdiction: string;
  operator: string;
  serviceArea: string;
  population: number;
  networkLength: number; // km
  dailyCapacity: number; // m³/day
  certificationLevel: CertificationLevel;
}

// ============================================================================
// Water Quality Types
// ============================================================================

/**
 * Parameter status
 */
export enum ParameterStatus {
  NORMAL = 'normal',
  WARNING = 'warning',
  CRITICAL = 'critical'
}

/**
 * Quality parameter
 */
export interface QualityParameter {
  value: number;
  unit: string;
  status: ParameterStatus;
}

/**
 * Monitoring station location type
 */
export enum LocationType {
  SOURCE = 'source',
  TREATMENT = 'treatment',
  DISTRIBUTION = 'distribution',
  TAP = 'tap'
}

/**
 * Water quality reading location
 */
export interface MonitoringLocation {
  id: UUID;
  type: LocationType;
  coordinates: Coordinates;
}

/**
 * Complete water quality reading
 */
export interface WaterQuality {
  timestamp: Timestamp;
  location: MonitoringLocation;
  parameters: {
    pH?: QualityParameter;
    turbidity?: QualityParameter;
    chlorine?: QualityParameter;
    temperature?: QualityParameter;
    conductivity?: QualityParameter;
    dissolvedOxygen?: QualityParameter;
    TDS?: QualityParameter;
    ORP?: QualityParameter;
    hardness?: QualityParameter;
    alkalinity?: QualityParameter;
  };
  compliance: 'compliant' | 'non-compliant';
  alerts?: string[];
}

// ============================================================================
// Hydraulic Types
// ============================================================================

/**
 * Flow data
 */
export interface FlowData {
  rate: number; // L/min
  velocity: number; // m/s
  direction: 'inbound' | 'outbound';
}

/**
 * Pressure data
 */
export interface PressureData {
  value: number; // bar
  min: number; // bar
  max: number; // bar
  average: number; // bar
}

/**
 * Hydraulic sensor reading
 */
export interface HydraulicData {
  timestamp: Timestamp;
  sensorId: UUID;
  location: {
    coordinates: Coordinates;
    zone?: string;
    pipeId?: string;
  };
  flow?: FlowData;
  pressure?: PressureData;
  anomalyDetected: boolean;
  confidence: number; // 0-1
}

// ============================================================================
// Leak Detection Types
// ============================================================================

/**
 * Leak severity levels
 */
export enum LeakSeverity {
  LOW = 'low',
  MEDIUM = 'medium',
  HIGH = 'high',
  CRITICAL = 'critical'
}

/**
 * Leak detection methods
 */
export enum DetectionMethod {
  ACOUSTIC = 'acoustic',
  PRESSURE = 'pressure',
  FLOW = 'flow',
  VISUAL = 'visual',
  AI = 'AI'
}

/**
 * Leak event status
 */
export enum LeakStatus {
  DETECTED = 'detected',
  VERIFIED = 'verified',
  REPAIRING = 'repairing',
  RESOLVED = 'resolved'
}

/**
 * Leak location data
 */
export interface LeakLocation {
  coordinates: Coordinates;
  address?: string;
  accuracy: number; // meters
}

/**
 * Estimated water loss
 */
export interface WaterLoss {
  rate: number; // L/day
  volume: number; // L
}

/**
 * Leak event
 */
export interface LeakEvent {
  eventId: UUID;
  detectionTime: Timestamp;
  location: LeakLocation;
  severity: LeakSeverity;
  estimatedLoss: WaterLoss;
  detectionMethod: DetectionMethod;
  confidence: number; // 0-100 %
  status: LeakStatus;
  assignedTeam?: string;
  responseTime?: number; // minutes
  repairTime?: number; // minutes
}

// ============================================================================
// Network Types
// ============================================================================

/**
 * Node types in water network
 */
export enum NodeType {
  SOURCE = 'source',
  PUMP = 'pump',
  TANK = 'tank',
  VALVE = 'valve',
  JUNCTION = 'junction'
}

/**
 * Pipe materials
 */
export enum PipeMaterial {
  CAST_IRON = 'cast_iron',
  PVC = 'PVC',
  STEEL = 'steel',
  COPPER = 'copper',
  HDPE = 'HDPE'
}

/**
 * Asset condition
 */
export enum AssetCondition {
  EXCELLENT = 'excellent',
  GOOD = 'good',
  FAIR = 'fair',
  POOR = 'poor',
  CRITICAL = 'critical'
}

/**
 * Network node
 */
export interface NetworkNode {
  id: UUID;
  type: NodeType;
  coordinates: Coordinates;
  elevation: number; // meters
  capacity?: number; // m³
  status: 'active' | 'inactive' | 'maintenance';
}

/**
 * Network pipe
 */
export interface NetworkPipe {
  id: UUID;
  startNode: UUID;
  endNode: UUID;
  length: number; // meters
  diameter: number; // mm
  material: PipeMaterial;
  installDate: string; // ISO 8601 date
  condition: AssetCondition;
}

/**
 * Pressure zone
 */
export interface PressureZone {
  id: UUID;
  name: string;
  polygon: Coordinates[];
  population: number;
  consumption: number; // m³/day
}

/**
 * Complete network topology
 */
export interface NetworkTopology {
  version: string;
  lastUpdated: Timestamp;
  nodes: NetworkNode[];
  pipes: NetworkPipe[];
  zones: PressureZone[];
}

// ============================================================================
// Consumption Types
// ============================================================================

/**
 * Customer tier
 */
export enum CustomerTier {
  RESIDENTIAL = 'residential',
  COMMERCIAL = 'commercial',
  INDUSTRIAL = 'industrial'
}

/**
 * Consumption anomaly types
 */
export enum AnomalyType {
  LEAK = 'leak',
  BURST = 'burst',
  UNUSUAL_PATTERN = 'unusual_pattern',
  METER_ERROR = 'meter_error'
}

/**
 * Meter reading
 */
export interface MeterReading {
  value: number; // m³
  cumulative: number; // m³
  interval: 'hourly' | 'daily' | 'monthly';
}

/**
 * Flow rate statistics
 */
export interface FlowRateStats {
  average: number; // L/min
  peak: number; // L/min
  minimum: number; // L/min
}

/**
 * Consumption anomaly
 */
export interface ConsumptionAnomaly {
  detected: boolean;
  type?: AnomalyType;
  confidence?: number; // 0-100 %
}

/**
 * Billing information
 */
export interface BillingInfo {
  tier: CustomerTier;
  rate: number; // $/m³
  amount: number; // $
}

/**
 * Complete consumption record
 */
export interface ConsumptionRecord {
  meterId: UUID;
  customerId: string;
  timestamp: Timestamp;
  reading: MeterReading;
  flowRate: FlowRateStats;
  anomaly: ConsumptionAnomaly;
  billing: BillingInfo;
}

// ============================================================================
// Alert Types
// ============================================================================

/**
 * Alert severity
 */
export enum AlertSeverity {
  INFO = 'info',
  WARNING = 'warning',
  ERROR = 'error',
  CRITICAL = 'critical'
}

/**
 * Alert type
 */
export enum AlertType {
  WATER_QUALITY = 'water-quality',
  LEAK = 'leak',
  PRESSURE = 'pressure',
  SYSTEM = 'system',
  SECURITY = 'security'
}

/**
 * Alert status
 */
export enum AlertStatus {
  ACTIVE = 'active',
  ACKNOWLEDGED = 'acknowledged',
  RESOLVED = 'resolved'
}

/**
 * System alert
 */
export interface Alert {
  alertId: UUID;
  timestamp: Timestamp;
  severity: AlertSeverity;
  type: AlertType;
  message: string;
  location?: {
    zoneId?: string;
    stationId?: string;
    coordinates?: Coordinates;
  };
  status: AlertStatus;
  actionRequired?: string;
  affectedPopulation?: number;
  acknowledgedBy?: string;
  acknowledgedAt?: Timestamp;
  resolvedAt?: Timestamp;
}

// ============================================================================
// API Response Types
// ============================================================================

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  data: T[];
  count: number;
  page?: number;
  pageSize?: number;
  totalPages?: number;
}

/**
 * Error response
 */
export interface ErrorResponse {
  error: {
    code: string;
    message: string;
    details?: any;
    timestamp: Timestamp;
    requestId: string;
  };
}

/**
 * API configuration
 */
export interface ApiConfig {
  host: string;
  port?: number;
  protocol?: 'http' | 'https';
  apiKey?: string;
  bearerToken?: string;
  timeout?: number; // milliseconds
}
