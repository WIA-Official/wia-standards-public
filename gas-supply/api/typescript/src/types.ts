/**
 * WIA-SOC-011: Gas Supply Standard - TypeScript Type Definitions
 * Version: 1.0.0
 *
 * © 2025 World Certification Industry Association
 * License: MIT
 */

// ============================================================================
// Core Types
// ============================================================================

export type PipelineType = 'transmission' | 'distribution' | 'gathering' | 'service';
export type OperationalStatus = 'active' | 'inactive' | 'abandoned' | 'planned' | 'under_construction';
export type MaterialType = 'steel' | 'plastic' | 'cast_iron' | 'composite';
export type MeasurementQuality = 'good' | 'uncertain' | 'bad';
export type EventSeverity = 'info' | 'warning' | 'high' | 'critical';
export type EventCategory = 'operational' | 'maintenance' | 'safety' | 'security' | 'environmental';

// ============================================================================
// Geometry Types (GeoJSON)
// ============================================================================

export interface Point {
  type: 'Point';
  coordinates: [number, number]; // [longitude, latitude]
}

export interface LineString {
  type: 'LineString';
  coordinates: Array<[number, number]>;
}

export interface GeoLocation {
  latitude: number;
  longitude: number;
  altitude?: number;
}

// ============================================================================
// Pipeline Types
// ============================================================================

export interface PipelineMaterial {
  type: MaterialType;
  grade?: string;
  diameter_mm: number;
  wallThickness_mm?: number;
  coating?: string;
}

export interface PressureRating {
  maop_bar: number; // Maximum Allowable Operating Pressure
  designPressure_bar?: number;
  testPressure_bar?: number;
}

export interface InstallationInfo {
  date: string; // ISO 8601 date
  installer?: string;
  manufacturer?: string;
  certificationNumber?: string;
}

export interface Pipeline {
  pipelineId: string;
  pipelineType: PipelineType;
  geometry: LineString;
  material: PipelineMaterial;
  pressureRating: PressureRating;
  installation: InstallationInfo;
  operationalStatus: OperationalStatus;
  length_km?: number;
  owner?: string;
  operator?: string;
}

// ============================================================================
// Measurement Types
// ============================================================================

export interface PipelineLocation {
  pipelineId: string;
  kilometer: number;
}

export interface Measurement {
  measurementId: string;
  timestamp: string; // ISO 8601 timestamp
  location: PipelineLocation | GeoLocation;
  measurementType: 'pressure' | 'flow' | 'temperature' | 'composition' | 'density';
  value: number;
  unit: string;
  quality: MeasurementQuality;
  sensorId?: string;
}

export interface TimeSeriesData {
  seriesId: string;
  startTime: string;
  interval_seconds: number;
  unit: string;
  values: number[];
  quality: MeasurementQuality[];
}

// ============================================================================
// Gas Composition Types
// ============================================================================

export interface GasComponent {
  molecule: string; // e.g., 'CH4', 'C2H6', 'CO2'
  moleFraction: number; // 0.0 to 1.0
}

export interface CalculatedProperties {
  heatingValue_MJ_m3: number;
  wobbeIndex: number;
  density_kg_m3: number;
  compressibility: number;
}

export interface Contaminants {
  H2S_ppm?: number;
  moisture_mg_m3?: number;
  totalSulfur_mg_m3?: number;
  oxygen_ppm?: number;
}

export interface GasComposition {
  compositionId: string;
  timestamp: string;
  location: PipelineLocation | GeoLocation;
  components: GasComponent[];
  calculatedProperties: CalculatedProperties;
  contaminants?: Contaminants;
}

// ============================================================================
// Equipment Types
// ============================================================================

export interface CompressorCapacity {
  flow_m3_h: number;
  pressureRatio: number;
  powerRating_MW: number;
}

export interface CompressorEfficiency {
  designPoint: number;
  currentOperating: number;
}

export interface Compressor {
  compressorId: string;
  type: 'centrifugal' | 'reciprocating' | 'screw';
  driver: 'gas_turbine' | 'electric_motor' | 'gas_engine';
  capacity: CompressorCapacity;
  efficiency: CompressorEfficiency;
  status: 'running' | 'stopped' | 'standby' | 'maintenance';
  runningHours: number;
}

export interface CompressorStation {
  stationId: string;
  location: GeoLocation & { address?: string };
  compressors: Compressor[];
  controlSystem?: {
    type: string;
    manufacturer: string;
    softwareVersion: string;
  };
}

export interface CalibrationInfo {
  lastDate: string;
  nextDue: string;
  certificateNumber: string;
  laboratory?: string;
}

export interface FlowMeter {
  meterId: string;
  manufacturer: string;
  model: string;
  diameter_mm: number;
  meteringTechnology: 'ultrasonic' | 'turbine' | 'coriolis' | 'orifice';
  accuracyClass: number;
  calibration: CalibrationInfo;
  measurement?: {
    flow_m3_h: number;
    totalVolume_m3: number;
    pressure_bar: number;
    temperature_C: number;
  };
}

export interface MeterStation {
  stationId: string;
  location: GeoLocation;
  flowMeters: FlowMeter[];
}

// ============================================================================
// Event and Alarm Types
// ============================================================================

export interface AffectedAsset {
  type: 'pipeline' | 'compressor' | 'meter' | 'valve' | 'station';
  id: string;
}

export interface Event {
  eventId: string;
  timestamp: string;
  eventType: 'alarm' | 'event' | 'notification';
  severity: EventSeverity;
  category: EventCategory;
  description: string;
  affectedAssets: AffectedAsset[];
  measurements?: Record<string, number>;
  status: 'active' | 'acknowledged' | 'resolved';
  acknowledgedBy?: string;
  acknowledgedAt?: string;
  resolvedTimestamp?: string;
  actionsTaken?: string[];
}

// ============================================================================
// Hydrogen Blending Types
// ============================================================================

export interface BlendingComposition {
  H2_moleFraction: number;
  CH4_moleFraction: number;
  other_moleFraction: number;
}

export interface BlendingSource {
  H2_source: 'green_hydrogen_electrolysis' | 'blue_hydrogen' | 'grey_hydrogen';
  H2_injectionRate_m3_h: number;
  NG_flowRate_m3_h: number;
}

export interface CompatibilityAssessment {
  pipelineMaterial: 'compatible' | 'requires_assessment' | 'not_compatible';
  compressorSeals: 'compatible' | 'requires_monitoring' | 'requires_upgrade';
  endUseEquipment: 'within_tolerance' | 'requires_adjustment' | 'not_compatible';
}

export interface HydrogenBlend {
  blendId: string;
  timestamp: string;
  location: PipelineLocation;
  composition: BlendingComposition;
  blendingSource: BlendingSource;
  calculatedProperties: Partial<CalculatedProperties>;
  compatibility: CompatibilityAssessment;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

export interface PaginationParams {
  limit?: number;
  offset?: number;
}

export interface PaginationInfo {
  total: number;
  limit: number;
  offset: number;
  hasMore: boolean;
}

export interface ListPipelinesParams extends PaginationParams {
  type?: PipelineType;
  region?: string;
  status?: OperationalStatus;
}

export interface ListPipelinesResponse {
  data: Pipeline[];
  pagination: PaginationInfo;
}

export interface MeasurementQueryParams {
  assetIds?: string[];
  types?: string[];
  since?: string;
}

export interface HistoricalDataParams {
  assetId: string;
  type: string;
  start: string;
  end: string;
  interval?: '1min' | '5min' | '1hour' | '1day';
  aggregation?: 'avg' | 'min' | 'max' | 'sum';
}

export interface ControlCommand {
  command: string;
  parameters?: Record<string, any>;
  authorization: {
    approvedBy: string;
    reason: string;
  };
}

export interface CommandResponse {
  commandId: string;
  status: 'pending' | 'executing' | 'completed' | 'failed';
  estimatedCompletion?: string;
  statusUrl: string;
}

// ============================================================================
// Client Configuration
// ============================================================================

export interface GasSupplyClientConfig {
  baseUrl: string;
  apiKey?: string;
  accessToken?: string;
  timeout?: number;
  retryAttempts?: number;
}

export interface WebSocketSubscription {
  channels: Array<{
    type: 'measurements' | 'alarms' | 'events';
    assetIds?: string[];
    severity?: EventSeverity[];
  }>;
}

// ============================================================================
// Error Types
// ============================================================================

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, any>;
  correlationId?: string;
}

export class GasSupplyError extends Error {
  constructor(
    message: string,
    public code: string,
    public details?: Record<string, any>,
    public correlationId?: string
  ) {
    super(message);
    this.name = 'GasSupplyError';
  }
}
