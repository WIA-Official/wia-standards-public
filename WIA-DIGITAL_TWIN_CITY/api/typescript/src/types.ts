/**
 * WIA Digital Twin City Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export interface WIADigitalTwinCity {
  standard: 'WIA-DIGITAL-TWIN-CITY';
  version: string;
  twin: TwinMetadata;
  city: CityInfo;
  layers: DataLayer[];
  sensors: SensorNetwork;
  simulations: SimulationConfig[];
  integrations: SystemIntegration[];
  analytics?: AnalyticsConfig;
  extensions?: Record<string, unknown>;
}

export interface TwinMetadata {
  id: string;
  name: string;
  description?: string;
  status: TwinStatus;
  fidelity: FidelityLevel;
  createdAt: string;
  updatedAt?: string;
  lastSyncAt?: string;
  owner: string;
}

export type TwinStatus = 'building' | 'calibrating' | 'operational' | 'maintenance' | 'archived';
export type FidelityLevel = 'low' | 'medium' | 'high' | 'ultra';

export interface CityInfo {
  name: string;
  country: string;
  region?: string;
  population: number;
  area: number;
  timezone: string;
  coordinates: { latitude: number; longitude: number };
  boundary: GeoPolygon;
}

export interface GeoPolygon {
  type: 'Polygon';
  coordinates: number[][][];
}

// ============================================================================
// Data Layer Types
// ============================================================================

export interface DataLayer {
  id: string;
  name: string;
  type: LayerType;
  source: DataSource;
  schema: LayerSchema;
  updateFrequency: UpdateFrequency;
  coverage: number;
  quality: DataQuality;
}

export type LayerType =
  | 'terrain'
  | 'buildings'
  | 'roads'
  | 'utilities'
  | 'vegetation'
  | 'traffic'
  | 'environment'
  | 'demographics'
  | 'energy'
  | 'water'
  | 'waste'
  | 'emergency'
  | 'custom';

export interface DataSource {
  type: 'gis' | 'iot' | 'api' | 'satellite' | 'lidar' | 'survey' | 'opendata';
  url?: string;
  format: string;
  credentials?: string;
  lastUpdate: string;
}

export interface LayerSchema {
  fields: SchemaField[];
  geometryType: 'point' | 'line' | 'polygon' | '3d-mesh' | 'point-cloud';
  crs: string;
}

export interface SchemaField {
  name: string;
  type: string;
  unit?: string;
  description?: string;
}

export type UpdateFrequency = 'realtime' | 'hourly' | 'daily' | 'weekly' | 'monthly' | 'on-demand';

export interface DataQuality {
  accuracy: number;
  completeness: number;
  timeliness: number;
  consistency: number;
}

// ============================================================================
// Sensor Types
// ============================================================================

export interface SensorNetwork {
  totalSensors: number;
  categories: SensorCategory[];
  protocols: string[];
  centralHub?: string;
}

export interface SensorCategory {
  type: SensorType;
  count: number;
  coverage: number;
  avgLatency: number;
  dataPoints: string[];
}

export type SensorType =
  | 'traffic'
  | 'air-quality'
  | 'noise'
  | 'weather'
  | 'energy-meter'
  | 'water-meter'
  | 'waste-bin'
  | 'parking'
  | 'pedestrian'
  | 'structural'
  | 'cctv'
  | 'environmental';

// ============================================================================
// Simulation Types
// ============================================================================

export interface SimulationConfig {
  id: string;
  name: string;
  type: SimulationType;
  status: SimulationStatus;
  parameters: SimulationParameters;
  schedule?: SimulationSchedule;
  outputs: SimulationOutput[];
}

export type SimulationType =
  | 'traffic-flow'
  | 'pedestrian-movement'
  | 'flood-risk'
  | 'air-dispersion'
  | 'energy-demand'
  | 'urban-heat'
  | 'emergency-evacuation'
  | 'construction-impact'
  | 'noise-propagation'
  | 'growth-scenario';

export type SimulationStatus = 'idle' | 'running' | 'completed' | 'failed';

export interface SimulationParameters {
  timespan: { start: string; end: string };
  resolution: { spatial: number; temporal: number };
  scenarios: Scenario[];
  constraints?: Record<string, unknown>;
}

export interface Scenario {
  name: string;
  description?: string;
  variables: Record<string, unknown>;
}

export interface SimulationSchedule {
  frequency: string;
  nextRun?: string;
  lastRun?: string;
}

export interface SimulationOutput {
  type: 'visualization' | 'report' | 'dataset' | 'alert';
  format: string;
  destination: string;
}

// ============================================================================
// Integration Types
// ============================================================================

export interface SystemIntegration {
  id: string;
  name: string;
  type: IntegrationType;
  status: IntegrationStatus;
  endpoint?: string;
  dataFlow: DataFlowDirection;
  lastSync?: string;
}

export type IntegrationType =
  | 'scada'
  | 'bms'
  | 'traffic-management'
  | 'emergency-services'
  | 'public-transit'
  | 'utility-grid'
  | 'waste-management'
  | 'water-system'
  | 'parking-system'
  | 'lighting-control'
  | 'erp'
  | 'gis';

export type IntegrationStatus = 'connected' | 'disconnected' | 'error' | 'syncing';
export type DataFlowDirection = 'inbound' | 'outbound' | 'bidirectional';

// ============================================================================
// Analytics Types
// ============================================================================

export interface AnalyticsConfig {
  dashboards: Dashboard[];
  alerts: AlertRule[];
  reports: ReportConfig[];
  mlModels?: MLModel[];
}

export interface Dashboard {
  id: string;
  name: string;
  widgets: Widget[];
  refreshInterval: number;
}

export interface Widget {
  type: 'map' | 'chart' | 'kpi' | 'table' | '3d-view';
  config: Record<string, unknown>;
}

export interface AlertRule {
  id: string;
  name: string;
  condition: string;
  severity: 'info' | 'warning' | 'critical';
  channels: string[];
}

export interface ReportConfig {
  id: string;
  name: string;
  schedule: string;
  format: 'pdf' | 'html' | 'excel';
  recipients: string[];
}

export interface MLModel {
  id: string;
  name: string;
  type: 'prediction' | 'anomaly-detection' | 'optimization' | 'classification';
  status: 'training' | 'deployed' | 'retired';
  accuracy?: number;
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface TwinResponse {
  id: string;
  name: string;
  cityName: string;
  status: TwinStatus;
  layerCount: number;
  sensorCount: number;
  lastSync: string;
  links: { self: string };
}

export interface ValidationResult {
  valid: boolean;
  errors?: { path: string; message: string }[];
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: { total: number; limit: number; offset: number; hasMore: boolean };
}
