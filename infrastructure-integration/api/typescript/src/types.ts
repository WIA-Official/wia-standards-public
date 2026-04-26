/**
 * WIA-UNI-005 Infrastructure Integration Standard
 * TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @description Type definitions for Phase 1 data formats
 */

/**
 * Base WIA entity with JSON-LD context
 */
export interface WIAEntity {
  '@context': string;
  '@type': string;
  id: string;
  metadata?: Metadata;
}

/**
 * Metadata for tracking data provenance
 */
export interface Metadata {
  createdBy: string;
  createdAt: string;
  modifiedBy?: string;
  modifiedAt?: string;
  version?: string;
}

/**
 * Geographic location with coordinates
 */
export interface Location {
  latitude: number;
  longitude: number;
  elevation?: number;
  name?: string;
}

/**
 * Project location with start and end points
 */
export interface ProjectLocation {
  startPoint: Location;
  endPoint: Location;
  corridor?: string;
  jurisdiction?: string[];
}

/**
 * Budget breakdown
 */
export interface Budget {
  total: number;
  currency: string;
  breakdown?: {
    construction?: number;
    equipment?: number;
    environmental?: number;
    contingency?: number;
  };
}

/**
 * Project timeline
 */
export interface Timeline {
  planningStart: string;
  constructionStart: string;
  targetCompletion: string;
  actualCompletion?: string;
  phaseApproach?: boolean;
}

/**
 * Project stakeholder
 */
export interface Stakeholder {
  organization: string;
  role: 'primary' | 'implementing' | 'funding' | 'regulatory';
  contactInfo?: Record<string, any>;
}

/**
 * Infrastructure project types
 */
export type ProjectType =
  | 'railway'
  | 'highway'
  | 'bridge'
  | 'tunnel'
  | 'pipeline'
  | 'power-grid'
  | 'telecom';

/**
 * Certification status
 */
export type CertificationStatus =
  | 'pending'
  | 'in-progress'
  | 'certified'
  | 'expired';

/**
 * Complete infrastructure project
 */
export interface InfrastructureProject extends WIAEntity {
  '@type': 'InfrastructureProject';
  projectType: ProjectType;
  name: string;
  description: string;
  location: ProjectLocation;
  budget: Budget;
  timeline: Timeline;
  stakeholders: Stakeholder[];
  standards: string[];
  certificationStatus: CertificationStatus;
}

/**
 * Asset types
 */
export type AssetType =
  | 'bridge'
  | 'tunnel'
  | 'track-section'
  | 'power-station'
  | 'pipeline-segment'
  | 'telecom-tower';

/**
 * Operational status
 */
export type OperationalStatus =
  | 'planned'
  | 'under-construction'
  | 'operational'
  | 'maintenance'
  | 'decommissioned';

/**
 * GeoJSON geometry
 */
export interface Geometry {
  type: 'Point' | 'LineString' | 'Polygon' | 'MultiPoint' | 'MultiLineString';
  coordinates: number[] | number[][] | number[][][];
  crs?: string;
}

/**
 * Asset location with geometry
 */
export interface AssetLocation {
  geometry: Geometry;
  address?: string;
  jurisdiction?: string;
}

/**
 * Certification record
 */
export interface Certification {
  type: 'safety' | 'structural' | 'environmental';
  issuer: string;
  issueDate: string;
  expiryDate: string;
  certificateId: string;
}

/**
 * Infrastructure asset
 */
export interface InfrastructureAsset extends WIAEntity {
  '@type': 'InfrastructureAsset';
  assetType: AssetType;
  name: string;
  description: string;
  projectId: string;
  location: AssetLocation;
  technicalSpecs: Record<string, any>;
  operationalStatus: OperationalStatus;
  commissionDate?: string;
  expectedLifespan?: number;
  certifications?: Certification[];
}

/**
 * Sensor measurement types
 */
export type MeasurementType =
  | 'temperature'
  | 'pressure'
  | 'vibration'
  | 'flow'
  | 'voltage'
  | 'current'
  | 'humidity'
  | 'wind-speed';

/**
 * Data quality indicator
 */
export type DataQuality = 'good' | 'suspect' | 'bad';

/**
 * Sensor reading
 */
export interface SensorReading extends WIAEntity {
  '@type': 'SensorReading';
  sensorId: string;
  assetId: string;
  timestamp: string;
  measurementType: MeasurementType;
  value: number;
  unit: string;
  quality: DataQuality;
  calibrationDate: string;
}

/**
 * Status severity levels
 */
export type StatusLevel = 'operational' | 'degraded' | 'offline' | 'emergency';

/**
 * Alert severity
 */
export type AlertSeverity = 'info' | 'warning' | 'critical';

/**
 * Status alert
 */
export interface StatusAlert {
  severity: AlertSeverity;
  message: string;
  code: string;
}

/**
 * Status metrics
 */
export interface StatusMetrics {
  utilization: number;
  efficiency: number;
  errorCount: number;
}

/**
 * Real-time status update
 */
export interface StatusUpdate extends WIAEntity {
  '@type': 'StatusUpdate';
  assetId: string;
  timestamp: string;
  status: StatusLevel;
  metrics: StatusMetrics;
  alerts: StatusAlert[];
}

/**
 * Maintenance record types
 */
export type MaintenanceType =
  | 'inspection'
  | 'repair'
  | 'upgrade'
  | 'certification';

/**
 * Component condition
 */
export type ComponentCondition =
  | 'excellent'
  | 'good'
  | 'fair'
  | 'poor'
  | 'critical';

/**
 * Maintenance finding
 */
export interface MaintenanceFinding {
  component: string;
  condition: ComponentCondition;
  notes: string;
  actionRequired: boolean;
}

/**
 * Replaced part
 */
export interface ReplacedPart {
  partId: string;
  description: string;
  quantity: number;
  serialNumbers?: string[];
}

/**
 * Maintenance cost breakdown
 */
export interface MaintenanceCost {
  labor: number;
  materials: number;
  total: number;
  currency: string;
}

/**
 * Personnel info
 */
export interface MaintenancePersonnel {
  organization: string;
  technician: string;
  certification: string;
}

/**
 * Maintenance record
 */
export interface MaintenanceRecord extends WIAEntity {
  '@type': 'MaintenanceRecord';
  assetId: string;
  recordType: MaintenanceType;
  scheduledDate: string;
  completionDate: string;
  performedBy: MaintenancePersonnel;
  findings: MaintenanceFinding[];
  workPerformed: string;
  partsReplaced?: ReplacedPart[];
  cost: MaintenanceCost;
  nextScheduledMaintenance?: string;
}

/**
 * Air quality measurements
 */
export interface AirQuality {
  pm25: number;
  pm10: number;
  no2: number;
  so2: number;
  co: number;
  o3: number;
  aqi: number;
}

/**
 * Water quality measurements
 */
export interface WaterQuality {
  ph: number;
  dissolvedOxygen: number;
  turbidity: number;
  temperature: number;
  pollutants?: Record<string, number>;
}

/**
 * Noise measurement
 */
export interface NoiseMeasurement {
  level: number;
  measurementPeriod: string;
}

/**
 * Carbon emissions
 */
export interface CarbonEmissions {
  co2: number;
  calculationMethod: string;
}

/**
 * Environmental monitoring reading
 */
export interface EnvironmentalReading extends WIAEntity {
  '@type': 'EnvironmentalReading';
  monitoringStationId: string;
  location: Location;
  timestamp: string;
  airQuality?: AirQuality;
  waterQuality?: WaterQuality;
  noise?: NoiseMeasurement;
  carbonEmissions?: CarbonEmissions;
}

/**
 * API Error response
 */
export interface APIError {
  error: {
    code: string;
    message: string;
    details?: Array<{
      field: string;
      issue: string;
    }>;
    timestamp: string;
    requestId: string;
  };
}

/**
 * API Response with links (HATEOAS)
 */
export interface APIResponse<T> {
  data: T;
  _links?: Record<string, string>;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    total: number;
    limit: number;
    offset: number;
    hasMore: boolean;
  };
  _links?: {
    self: string;
    next?: string;
    prev?: string;
  };
}

/**
 * Query parameters for filtering
 */
export interface QueryParams {
  type?: ProjectType | AssetType;
  status?: OperationalStatus | CertificationStatus;
  region?: string;
  limit?: number;
  offset?: number;
  startDate?: string;
  endDate?: string;
}
