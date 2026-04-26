/**
 * WIA-SPACE-015: Aerospace Engine Standard - TypeScript Types
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

export type Timestamp = string;

export enum EngineType {
  TURBOFAN = 'TURBOFAN',
  TURBOPROP = 'TURBOPROP',
  TURBOSHAFT = 'TURBOSHAFT',
  TURBOJET = 'TURBOJET',
  ROCKET_LIQUID = 'ROCKET_LIQUID',
  ROCKET_SOLID = 'ROCKET_SOLID',
  RAMJET = 'RAMJET',
  SCRAMJET = 'SCRAMJET',
  ION = 'ION',
  ELECTRIC = 'ELECTRIC',
}

export enum EngineStatus {
  NOMINAL = 'NOMINAL',
  WARNING = 'WARNING',
  CRITICAL = 'CRITICAL',
  SHUTDOWN = 'SHUTDOWN',
  STARTING = 'STARTING',
  IDLE = 'IDLE',
  MAINTENANCE = 'MAINTENANCE',
}

export interface Specifications {
  thrust_lbf?: number;
  thrust_kn?: number;
  power_kw?: number;
  bypass_ratio?: number;
  sfc_lb_lbf_hr?: number;
  opr?: number;
  fanDiameter?: number;
  length?: number;
  dryWeight?: number;
  stages?: StageConfiguration;
}

export interface StageConfiguration {
  compressorStages: number;
  turbineStages: number;
  fanStages?: number;
}

export interface Performance {
  max_temperature_C: number;
  fuel_efficiency_percent: number;
  noise_reduction_dB?: number;
  emissions: EmissionsData;
  reliability: ReliabilityMetrics;
}

export interface EmissionsData {
  nox_g_kn?: number;
  co_g_kn?: number;
  hc_g_kn?: number;
  smoke_number?: number;
  co2_kg_hour?: number;
}

export interface ReliabilityMetrics {
  mtbf_hours: number;
  mtbr_hours: number;
  dispatchReliability: number;
  ifsd_rate: number;
}

export interface Certification {
  faa: boolean;
  easa: boolean;
  caac?: boolean;
  tcca?: boolean;
  etops?: number;
  typeDesignation?: string;
  certificationDate?: Timestamp;
}

export interface Engine {
  engineId: string;
  model: string;
  type: EngineType;
  manufacturer: string;
  serialNumber: string;
  specifications: Specifications;
  performance: Performance;
  certification: Certification;
  installationDate?: Timestamp;
  totalCycles: number;
  totalHours: number;
  status: EngineStatus;
}

export interface EngineTelemetry {
  timestamp: Timestamp;
  engineId: string;
  thrust_lbf: number;
  n1_percent: number;
  n2_percent: number;
  egt_celsius: number;
  fuel_flow_kg_hr: number;
  oil_pressure_psi: number;
  oil_temperature_C: number;
  vibration: VibrationData;
  status: EngineStatus;
}

export interface VibrationData {
  n1_amplitude: number;
  n2_amplitude: number;
  broadband: number;
  trend: 'stable' | 'increasing' | 'decreasing';
}

export interface MaintenanceRecord {
  recordId: string;
  engineId: string;
  type: MaintenanceType;
  description: string;
  performedDate: Timestamp;
  technician: string;
  workOrder: string;
  partsReplaced: Part[];
  nextDue?: MaintenanceDue;
}

export enum MaintenanceType {
  SCHEDULED = 'SCHEDULED',
  UNSCHEDULED = 'UNSCHEDULED',
  OVERHAUL = 'OVERHAUL',
  INSPECTION = 'INSPECTION',
  REPAIR = 'REPAIR',
  MODIFICATION = 'MODIFICATION',
}

export interface Part {
  partNumber: string;
  description: string;
  serialNumber?: string;
  quantity: number;
}

export interface MaintenanceDue {
  type: string;
  dueHours?: number;
  dueCycles?: number;
  dueDate?: Timestamp;
}

export interface HealthMonitoring {
  engineId: string;
  timestamp: Timestamp;
  healthScore: number;
  alerts: EngineAlert[];
  trends: TrendData[];
  recommendations: string[];
}

export interface EngineAlert {
  alertId: string;
  severity: 'INFO' | 'WARNING' | 'CRITICAL';
  parameter: string;
  message: string;
  timestamp: Timestamp;
  acknowledged: boolean;
}

export interface TrendData {
  parameter: string;
  baseline: number;
  current: number;
  deviation: number;
  trend: 'stable' | 'increasing' | 'decreasing';
}

export interface FuelAnalysis {
  analysisId: string;
  sampleDate: Timestamp;
  fuelType: string;
  density: number;
  viscosity: number;
  contaminants: Contaminant[];
  passedSpec: boolean;
}

export interface Contaminant {
  type: string;
  level: number;
  unit: string;
  limit: number;
}

export interface APIResponse<T> {
  status: 'success' | 'error';
  data?: T;
  error?: APIError;
  timestamp: Timestamp;
  requestId: string;
}

export interface APIError {
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

export interface SDKConfig {
  apiKey: string;
  baseURL?: string;
  timeout?: number;
  debug?: boolean;
}
