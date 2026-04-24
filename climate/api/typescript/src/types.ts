/**
 * WIA Climate Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Climate Types
// ============================================================================

export interface WIAClimateProject {
  standard: 'WIA-CLIMATE';
  version: string;
  metadata: ProjectMetadata;
  climateData: ClimateData;
  monitoring: MonitoringConfiguration;
  analysis: AnalysisConfiguration;
  reporting: ReportingConfiguration;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  type: ClimateProjectType;
  region: GeographicRegion;
  timeframe: TimeFrame;
  stakeholders?: Stakeholder[];
  tags?: string[];
}

export type ClimateProjectType =
  | 'monitoring'
  | 'prediction'
  | 'mitigation'
  | 'adaptation'
  | 'research'
  | 'policy-analysis';

export interface GeographicRegion {
  name: string;
  type: 'global' | 'continental' | 'national' | 'regional' | 'local';
  bounds?: GeoBounds;
  countries?: string[];
  coordinates?: Coordinates;
}

export interface GeoBounds {
  north: number;
  south: number;
  east: number;
  west: number;
}

export interface Coordinates {
  latitude: number;
  longitude: number;
  elevation?: number;
}

export interface TimeFrame {
  start: string;
  end: string;
  resolution: TemporalResolution;
  timezone?: string;
}

export type TemporalResolution =
  | 'hourly'
  | 'daily'
  | 'weekly'
  | 'monthly'
  | 'quarterly'
  | 'yearly'
  | 'decadal';

export interface Stakeholder {
  name: string;
  role: string;
  organization?: string;
  contact?: string;
}

// ============================================================================
// Climate Data Types
// ============================================================================

export interface ClimateData {
  variables: ClimateVariable[];
  sources: DataSource[];
  quality: DataQuality;
}

export interface ClimateVariable {
  id: string;
  name: string;
  type: ClimateVariableType;
  unit: string;
  description?: string;
  range?: ValueRange;
  uncertainty?: number;
}

export type ClimateVariableType =
  | 'temperature'
  | 'precipitation'
  | 'humidity'
  | 'pressure'
  | 'wind-speed'
  | 'wind-direction'
  | 'solar-radiation'
  | 'cloud-cover'
  | 'sea-level'
  | 'ice-extent'
  | 'carbon-dioxide'
  | 'methane'
  | 'nitrous-oxide'
  | 'aerosol'
  | 'ozone';

export interface ValueRange {
  min: number;
  max: number;
  typical?: number;
}

export interface DataSource {
  id: string;
  name: string;
  type: DataSourceType;
  provider: string;
  url?: string;
  format: DataFormat;
  coverage: TemporalCoverage;
  resolution: SpatialResolution;
  license?: string;
}

export type DataSourceType =
  | 'satellite'
  | 'ground-station'
  | 'buoy'
  | 'aircraft'
  | 'model-output'
  | 'reanalysis'
  | 'proxy';

export type DataFormat =
  | 'netcdf'
  | 'grib'
  | 'csv'
  | 'json'
  | 'geotiff'
  | 'hdf5'
  | 'zarr';

export interface TemporalCoverage {
  start: string;
  end: string;
  completeness?: number;
}

export interface SpatialResolution {
  horizontal: number;
  unit: 'km' | 'm' | 'degrees';
  vertical?: number;
  levels?: number;
}

export interface DataQuality {
  level: 'raw' | 'quality-controlled' | 'homogenized' | 'validated';
  checks: QualityCheck[];
  flags?: QualityFlag[];
}

export interface QualityCheck {
  type: string;
  status: 'passed' | 'warning' | 'failed';
  details?: string;
}

export interface QualityFlag {
  code: string;
  description: string;
  severity: 'info' | 'warning' | 'error';
}

// ============================================================================
// Monitoring Configuration
// ============================================================================

export interface MonitoringConfiguration {
  stations?: MonitoringStation[];
  sensors?: SensorConfiguration[];
  schedule: MonitoringSchedule;
  alerts?: AlertConfiguration[];
}

export interface MonitoringStation {
  id: string;
  name: string;
  type: StationType;
  location: Coordinates;
  elevation: number;
  operationalSince?: string;
  sensors: string[];
  status: 'active' | 'maintenance' | 'inactive';
}

export type StationType =
  | 'surface'
  | 'upper-air'
  | 'marine'
  | 'satellite-ground'
  | 'reference';

export interface SensorConfiguration {
  id: string;
  type: string;
  manufacturer?: string;
  model?: string;
  accuracy: number;
  precision: number;
  calibrationDate?: string;
  maintenanceSchedule?: string;
}

export interface MonitoringSchedule {
  frequency: string;
  startTime?: string;
  endTime?: string;
  timezone: string;
}

export interface AlertConfiguration {
  id: string;
  variable: string;
  condition: AlertCondition;
  threshold: number;
  severity: 'low' | 'medium' | 'high' | 'critical';
  recipients: string[];
  enabled: boolean;
}

export type AlertCondition =
  | 'above'
  | 'below'
  | 'equals'
  | 'rate-of-change'
  | 'anomaly';

// ============================================================================
// Analysis Configuration
// ============================================================================

export interface AnalysisConfiguration {
  methods: AnalysisMethod[];
  models?: ClimateModel[];
  scenarios?: ClimateScenario[];
  outputs: AnalysisOutput[];
}

export interface AnalysisMethod {
  id: string;
  name: string;
  type: AnalysisType;
  parameters: Record<string, unknown>;
  validation?: ValidationMethod;
}

export type AnalysisType =
  | 'trend-analysis'
  | 'anomaly-detection'
  | 'statistical-downscaling'
  | 'dynamic-downscaling'
  | 'extreme-value-analysis'
  | 'attribution'
  | 'projection';

export interface ValidationMethod {
  type: 'cross-validation' | 'holdout' | 'bootstrap';
  metrics: string[];
  threshold?: number;
}

export interface ClimateModel {
  id: string;
  name: string;
  type: 'gcm' | 'rcm' | 'esm' | 'emulator';
  institution: string;
  resolution: SpatialResolution;
  forcing?: string[];
  experiments?: string[];
}

export interface ClimateScenario {
  id: string;
  name: string;
  type: ScenarioType;
  description: string;
  parameters: ScenarioParameters;
}

export type ScenarioType =
  | 'rcp'
  | 'ssp'
  | 'historical'
  | 'custom';

export interface ScenarioParameters {
  emissions?: string;
  landUse?: string;
  population?: string;
  economy?: string;
  forcingLevel?: number;
}

export interface AnalysisOutput {
  id: string;
  type: string;
  format: DataFormat;
  variables: string[];
  temporal: TemporalCoverage;
  spatial: GeographicRegion;
}

// ============================================================================
// Reporting Configuration
// ============================================================================

export interface ReportingConfiguration {
  reports: ReportTemplate[];
  schedule?: ReportSchedule;
  distribution?: DistributionConfig;
}

export interface ReportTemplate {
  id: string;
  name: string;
  type: ReportType;
  sections: ReportSection[];
  format: 'pdf' | 'html' | 'docx' | 'json';
  language?: string;
}

export type ReportType =
  | 'summary'
  | 'detailed'
  | 'executive'
  | 'technical'
  | 'public';

export interface ReportSection {
  id: string;
  title: string;
  type: 'text' | 'chart' | 'map' | 'table' | 'statistics';
  content?: string;
  data?: string;
}

export interface ReportSchedule {
  frequency: 'daily' | 'weekly' | 'monthly' | 'quarterly' | 'annually';
  dayOfWeek?: number;
  dayOfMonth?: number;
  time?: string;
}

export interface DistributionConfig {
  channels: DistributionChannel[];
  recipients: Recipient[];
}

export interface DistributionChannel {
  type: 'email' | 'api' | 'ftp' | 'web';
  config: Record<string, unknown>;
}

export interface Recipient {
  id: string;
  name: string;
  email?: string;
  endpoint?: string;
  reports: string[];
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface ClimateDataRequest {
  variables: string[];
  region: GeographicRegion;
  timeframe: TimeFrame;
  format?: DataFormat;
  aggregation?: AggregationType;
}

export type AggregationType =
  | 'none'
  | 'mean'
  | 'sum'
  | 'min'
  | 'max'
  | 'std';

export interface ClimateDataResponse {
  requestId: string;
  data: DataPoint[];
  metadata: ResponseMetadata;
  pagination?: Pagination;
}

export interface DataPoint {
  timestamp: string;
  location?: Coordinates;
  values: Record<string, number>;
  quality?: Record<string, string>;
}

export interface ResponseMetadata {
  source: string;
  generated: string;
  units: Record<string, string>;
  coverage: number;
}

export interface Pagination {
  total: number;
  limit: number;
  offset: number;
  hasMore: boolean;
}

export interface ProjectResponse {
  id: string;
  name: string;
  status: ProjectStatus;
  createdAt: string;
  updatedAt?: string;
}

export type ProjectStatus =
  | 'created'
  | 'active'
  | 'paused'
  | 'completed'
  | 'archived';

// ============================================================================
// Utility Types
// ============================================================================

export interface ValidationResult {
  valid: boolean;
  errors?: ValidationError[];
}

export interface ValidationError {
  path: string;
  message: string;
  value?: unknown;
}

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
  timestamp: string;
  requestId?: string;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: Pagination;
  links: {
    first?: string;
    prev?: string;
    self: string;
    next?: string;
    last?: string;
  };
}
