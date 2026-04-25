/**
 * WIA-SOC-009: Sewage System Standard - TypeScript Type Definitions
 *
 * 弘익人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Basic Types
// ============================================================================

export type Timestamp = string;
export type UUID = string;

export interface Coordinates {
  latitude: number;
  longitude: number;
  elevation?: number;
}

// ============================================================================
// System Types
// ============================================================================

export enum SystemStatus {
  NORMAL = 'normal',
  WARNING = 'warning',
  CRITICAL = 'critical',
  MAINTENANCE = 'maintenance',
  OFFLINE = 'offline'
}

export enum AlertSeverity {
  INFO = 'info',
  WARNING = 'warning',
  ERROR = 'error',
  CRITICAL = 'critical'
}

export enum AlertCategory {
  WATER_QUALITY = 'water_quality',
  FLOW = 'flow',
  EQUIPMENT = 'equipment',
  ENVIRONMENTAL = 'environmental',
  SAFETY = 'safety'
}

// ============================================================================
// Water Quality
// ============================================================================

export interface WaterQualityReading {
  timestamp: Timestamp;
  location: string;
  pH?: number;
  dissolvedOxygen?: number;      // mg/L
  temperature?: number;           // °C
  turbidity?: number;             // NTU
  conductivity?: number;          // µS/cm
  BOD?: number;                   // mg/L
  COD?: number;                   // mg/L
  TSS?: number;                   // mg/L
  TDS?: number;                   // mg/L
  ammoniaNitrogen?: number;       // mg/L
  totalPhosphorus?: number;       // mg/L
  fecalColiform?: number;         // CFU/100ml
  EColi?: number;                 // CFU/100ml
}

export interface WaterQualityCompliance {
  parameter: string;
  value: number;
  limit: number;
  unit: string;
  status: 'compliant' | 'warning' | 'violation';
}

// ============================================================================
// Flow Management
// ============================================================================

export interface FlowReading {
  timestamp: Timestamp;
  locationId: string;
  locationName: string;
  flowRate: number;               // m³/s
  velocity?: number;              // m/s
  depth?: number;                 // meters
  pressure?: number;              // bar
  trend: 'increasing' | 'stable' | 'decreasing';
}

export interface FlowEvent {
  eventId: UUID;
  timestamp: Timestamp;
  eventType: 'overflow' | 'bypass' | 'surge' | 'blockage' | 'leak';
  location: Coordinates & { description: string };
  severity: AlertSeverity;
  flowData: {
    peakFlow: number;
    duration: number;             // seconds
    volume: number;               // m³
    estimatedContaminantLoad?: number;
  };
  response?: {
    alertSent: Timestamp;
    responseTime: number;         // minutes
    actionTaken: string;
    resolvedAt?: Timestamp;
  };
}

// ============================================================================
// Sensor Types
// ============================================================================

export enum SensorType {
  FLOW = 'flow',
  QUALITY = 'quality',
  LEVEL = 'level',
  PRESSURE = 'pressure',
  GAS = 'gas'
}

export interface SensorReading {
  sensorId: UUID;
  sensorType: SensorType;
  timestamp: Timestamp;
  location: Coordinates & { zone?: string };
  reading: {
    value: number;
    unit: string;
    quality: 'good' | 'fair' | 'poor' | 'invalid';
    confidence: number;           // 0-1
  };
  calibration?: {
    lastCalibrated: Timestamp;
    nextDue: Timestamp;
  };
}

// ============================================================================
// Equipment
// ============================================================================

export enum EquipmentStatus {
  RUNNING = 'running',
  STOPPED = 'stopped',
  MAINTENANCE = 'maintenance',
  FAULT = 'fault'
}

export interface PumpStatus {
  pumpId: UUID;
  location: string;
  status: EquipmentStatus;
  flowRate: number;               // m³/s
  power: number;                  // kW
  current: number;                // A
  voltage: number;                // V
  vibration?: number;             // mm/s
  temperature: number;            // °C
  runtime: number;                // hours
  cycleCount: number;
  maintenance: {
    lastService: Timestamp;
    nextDue: Timestamp;
    predictedFailure?: Timestamp;
  };
}

export interface TreatmentProcess {
  processId: UUID;
  stage: 'screening' | 'grit_removal' | 'primary' | 'secondary' | 'tertiary' | 'disinfection';
  timestamp: Timestamp;
  influent: {
    flow: number;
    quality: WaterQualityReading;
  };
  effluent: {
    flow: number;
    quality: WaterQualityReading;
  };
  efficiency: {
    BODremoval: number;           // %
    SSSremoval: number;           // %
    pathogenReduction?: number;   // log reduction
  };
  chemicals: ChemicalUsage[];
  energy: {
    consumption: number;          // kWh
    cost: number;                 // $
  };
}

export interface ChemicalUsage {
  type: 'coagulant' | 'flocculant' | 'disinfectant' | 'pH_adjuster';
  name: string;
  dosage: number;                 // mg/L
  costPerUnit: number;            // $/kg
}

// ============================================================================
// System State
// ============================================================================

export interface SystemState {
  timestamp: Timestamp;
  systemId: UUID;
  status: SystemStatus;
  flowRate: number;
  activePumps: number;
  treatmentEfficiency: number;    // %
  alerts: SystemAlert[];
  lastUpdate: Timestamp;
}

export interface SystemInfo {
  systemId: UUID;
  municipality: string;
  operator: string;
  location: Coordinates;
  capacity: number;               // m³/day
  servingPopulation: number;
  installDate: Timestamp;
  capabilities: string[];
}

// ============================================================================
// Alerts
// ============================================================================

export interface SystemAlert {
  alertId: UUID;
  timestamp: Timestamp;
  severity: AlertSeverity;
  category: AlertCategory;
  message: string;
  location: string;
  parameters?: {
    threshold: number;
    actual: number;
    deviation: number;            // %
  };
  actionRequired: boolean;
  acknowledgedBy?: string;
  acknowledgedAt?: Timestamp;
  resolvedAt?: Timestamp;
}

// ============================================================================
// Reports
// ============================================================================

export interface ComplianceReport {
  reportId: UUID;
  period: 'daily' | 'weekly' | 'monthly' | 'quarterly' | 'annual';
  startDate: Timestamp;
  endDate: Timestamp;
  overallCompliance: number;      // %
  violations: WaterQualityCompliance[];
  summary: string;
  generatedAt: Timestamp;
}

export interface PerformanceMetrics {
  period: 'daily' | 'weekly' | 'monthly' | 'yearly';
  startDate: Timestamp;
  endDate: Timestamp;
  metrics: {
    averageInflowRate: number;
    totalVolumeTreated: number;
    overallEfficiency: number;
    complianceRate: number;
    overflowEvents: number;
    energyConsumption: number;
    chemicalCost: number;
    maintenanceCost: number;
  };
}

// ============================================================================
// Predictions
// ============================================================================

export interface OverflowPrediction {
  location: string;
  riskLevel: 'high' | 'medium' | 'low';
  probability: number;            // 0-1
  expectedTime: Timestamp;
  estimatedVolume: number;
  confidence: number;             // 0-1
}

export interface EquipmentFailurePrediction {
  equipmentId: UUID;
  equipmentType: string;
  failureProbability: number;     // 0-1
  expectedFailureDate: Timestamp;
  confidence: number;             // 0-1
  recommendedAction: string;
  estimatedCost: number;
}

// ============================================================================
// API Client Options
// ============================================================================

export interface ApiClientOptions {
  host: string;
  port?: number;
  protocol?: 'http' | 'https';
  token?: string;
  timeout?: number;               // milliseconds
  retries?: number;
}

export interface WebSocketOptions {
  reconnect?: boolean;
  reconnectInterval?: number;     // milliseconds
  maxReconnectAttempts?: number;
}

// ============================================================================
// Events
// ============================================================================

export interface SystemEvent {
  eventId: UUID;
  timestamp: Timestamp;
  type: string;
  data: Record<string, any>;
}

export type EventCallback = (event: SystemEvent) => void;

// ============================================================================
// Command and Response
// ============================================================================

export interface CommandResponse {
  commandId: UUID;
  status: 'accepted' | 'rejected' | 'completed' | 'failed';
  message?: string;
  data?: Record<string, any>;
}
