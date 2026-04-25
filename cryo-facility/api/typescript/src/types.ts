/**
 * WIA Cryo Facility Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Cryo Facility Types
// ============================================================================

export interface WIACryoFacilityProject {
  standard: 'WIA-CRYO-FACILITY';
  version: string;
  metadata: ProjectMetadata;
  facility: FacilityConfiguration;
  equipment: EquipmentInventory;
  operations: OperationalProcedures;
  safety: SafetyManagement;
  quality: QualityManagement;
  environmental: EnvironmentalControl;
  monitoring: MonitoringSystem;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  type: FacilityType;
  location: FacilityLocation;
  organization: Organization;
  licenses: License[];
  certifications: Certification[];
  createdAt: string;
  status: FacilityStatus;
}

export type FacilityType =
  | 'biobank'
  | 'tissue-bank'
  | 'fertility-center'
  | 'research-facility'
  | 'hospital-unit'
  | 'commercial-storage';

export interface FacilityLocation {
  address: string;
  city: string;
  state?: string;
  country: string;
  postalCode: string;
  coordinates: Coordinates;
  timezone: string;
}

export interface Coordinates {
  latitude: number;
  longitude: number;
}

export interface Organization {
  name: string;
  type: string;
  registrationNumber?: string;
  contact: ContactInfo;
}

export interface ContactInfo {
  name: string;
  email: string;
  phone: string;
  emergencyPhone?: string;
}

export interface License {
  type: string;
  number: string;
  issuer: string;
  validFrom: string;
  validTo: string;
  scope: string[];
  status: 'active' | 'expired' | 'suspended' | 'pending';
}

export interface Certification {
  name: string;
  body: string;
  number: string;
  scope: string[];
  validFrom: string;
  validTo: string;
  status: 'active' | 'expired' | 'pending';
}

export type FacilityStatus =
  | 'operational'
  | 'limited-operations'
  | 'maintenance'
  | 'emergency'
  | 'offline';

// ============================================================================
// Facility Configuration
// ============================================================================

export interface FacilityConfiguration {
  layout: FacilityLayout;
  zones: FacilityZone[];
  capacity: CapacityMetrics;
  infrastructure: InfrastructureConfig;
  access: AccessControlConfig;
}

export interface FacilityLayout {
  totalArea: number;
  storageArea: number;
  processingArea: number;
  officeArea: number;
  unit: 'sqft' | 'sqm';
  floors: Floor[];
  floorPlan?: string;
}

export interface Floor {
  level: number;
  name: string;
  area: number;
  zones: string[];
}

export interface FacilityZone {
  id: string;
  name: string;
  type: ZoneType;
  classification: CleanroomClass;
  area: number;
  equipment: string[];
  accessLevel: AccessLevel;
  environmentalRequirements: EnvironmentalRequirements;
}

export type ZoneType =
  | 'storage'
  | 'processing'
  | 'quality-control'
  | 'receiving'
  | 'shipping'
  | 'preparation'
  | 'office'
  | 'utility';

export type CleanroomClass = 'ISO-5' | 'ISO-6' | 'ISO-7' | 'ISO-8' | 'non-classified';

export type AccessLevel = 'public' | 'restricted' | 'controlled' | 'high-security';

export interface EnvironmentalRequirements {
  temperature: TemperatureRange;
  humidity: HumidityRange;
  particulate?: ParticulateLimit;
  pressure?: PressureDifferential;
}

export interface TemperatureRange {
  min: number;
  max: number;
  unit: 'celsius' | 'fahrenheit';
}

export interface HumidityRange {
  min: number;
  max: number;
}

export interface ParticulateLimit {
  size: number;
  count: number;
}

export interface PressureDifferential {
  value: number;
  unit: 'pascals' | 'inches-h2o';
}

export interface CapacityMetrics {
  storage: StorageCapacity;
  processing: ProcessingCapacity;
  personnel: PersonnelCapacity;
}

export interface StorageCapacity {
  tanks: number;
  totalSpecimens: number;
  currentSpecimens: number;
  utilizationPercent: number;
}

export interface ProcessingCapacity {
  daily: number;
  weekly: number;
  unit: string;
}

export interface PersonnelCapacity {
  maximum: number;
  current: number;
  shifts: ShiftConfig[];
}

export interface ShiftConfig {
  name: string;
  start: string;
  end: string;
  days: string[];
  minimumStaff: number;
}

export interface InfrastructureConfig {
  power: PowerSystem;
  hvac: HVACSystem;
  gasSupply: GasSupply;
  backup: BackupSystems;
  networking: NetworkingConfig;
}

export interface PowerSystem {
  mainSupply: string;
  capacity: number;
  unit: 'kW';
  redundancy: boolean;
  ups: UPSConfig;
}

export interface UPSConfig {
  capacity: number;
  runtime: number;
  units: number;
}

export interface HVACSystem {
  type: string;
  zones: number;
  redundancy: boolean;
  filtration: string;
}

export interface GasSupply {
  liquidNitrogen: LN2Supply;
  co2?: GasConfig;
  medicalGases?: GasConfig[];
}

export interface LN2Supply {
  bulkTank: { capacity: number; unit: 'liters' };
  deliverySchedule: string;
  backupSupply: boolean;
  monitoring: boolean;
}

export interface GasConfig {
  type: string;
  supply: string;
  backup: boolean;
}

export interface BackupSystems {
  generator: GeneratorConfig;
  alternateStorage: boolean;
  disasterRecovery: string;
}

export interface GeneratorConfig {
  type: string;
  capacity: number;
  fuelType: string;
  fuelCapacity: number;
  autoStart: boolean;
  testingSchedule: string;
}

export interface NetworkingConfig {
  type: string;
  redundancy: boolean;
  bandwidth: number;
  security: string[];
}

export interface AccessControlConfig {
  system: string;
  methods: AccessMethod[];
  logging: boolean;
  retention: string;
}

export type AccessMethod = 'keycard' | 'biometric' | 'pin' | 'key' | 'multi-factor';

// ============================================================================
// Equipment Inventory
// ============================================================================

export interface EquipmentInventory {
  cryoStorage: CryoStorageEquipment[];
  processing: ProcessingEquipment[];
  monitoring: MonitoringEquipment[];
  safety: SafetyEquipment[];
  maintenance: MaintenanceSchedule;
}

export interface CryoStorageEquipment {
  id: string;
  type: CryoEquipmentType;
  model: string;
  manufacturer: string;
  serialNumber: string;
  location: string;
  capacity: number;
  temperature: number;
  status: EquipmentStatus;
  installation: InstallationInfo;
  maintenance: MaintenanceInfo;
  monitoring: MonitoringConfig;
}

export type CryoEquipmentType =
  | 'ln2-tank'
  | 'ln2-freezer'
  | 'mechanical-freezer'
  | 'controlled-rate-freezer'
  | 'incubator';

export type EquipmentStatus =
  | 'operational'
  | 'warning'
  | 'alarm'
  | 'maintenance'
  | 'offline'
  | 'decommissioned';

export interface InstallationInfo {
  date: string;
  installer: string;
  warranty: { expires: string; provider: string };
}

export interface MaintenanceInfo {
  lastService: string;
  nextService: string;
  serviceProvider: string;
  history: ServiceRecord[];
}

export interface ServiceRecord {
  date: string;
  type: string;
  technician: string;
  description: string;
  parts?: string[];
}

export interface MonitoringConfig {
  sensors: Sensor[];
  interval: number;
  alerts: AlertConfig[];
}

export interface Sensor {
  id: string;
  type: 'temperature' | 'level' | 'pressure' | 'humidity';
  location: string;
  accuracy: number;
  calibration: CalibrationInfo;
}

export interface CalibrationInfo {
  lastCalibration: string;
  nextCalibration: string;
  certificate?: string;
}

export interface AlertConfig {
  parameter: string;
  threshold: number;
  condition: 'above' | 'below' | 'equals';
  severity: 'info' | 'warning' | 'critical';
  recipients: string[];
  escalation?: EscalationConfig;
}

export interface EscalationConfig {
  delay: number;
  recipients: string[];
}

export interface ProcessingEquipment {
  id: string;
  name: string;
  type: string;
  manufacturer: string;
  model: string;
  location: string;
  status: EquipmentStatus;
  capabilities: string[];
}

export interface MonitoringEquipment {
  id: string;
  type: string;
  coverage: string[];
  status: EquipmentStatus;
}

export interface SafetyEquipment {
  id: string;
  type: SafetyEquipmentType;
  location: string;
  quantity: number;
  lastInspection: string;
  nextInspection: string;
  status: 'ready' | 'needs-attention' | 'expired';
}

export type SafetyEquipmentType =
  | 'oxygen-monitor'
  | 'fire-extinguisher'
  | 'emergency-shower'
  | 'eyewash'
  | 'first-aid-kit'
  | 'ppe-station'
  | 'spill-kit';

export interface MaintenanceSchedule {
  preventive: MaintenanceTask[];
  predictive: boolean;
  contracts: MaintenanceContract[];
}

export interface MaintenanceTask {
  equipment: string;
  task: string;
  frequency: string;
  responsible: string;
  documentation: string;
}

export interface MaintenanceContract {
  provider: string;
  scope: string[];
  start: string;
  end: string;
  sla: string;
}

// ============================================================================
// Operations & Safety
// ============================================================================

export interface OperationalProcedures {
  sops: StandardProcedure[];
  workflows: Workflow[];
  training: TrainingProgram;
  documentation: DocumentationSystem;
}

export interface StandardProcedure {
  id: string;
  title: string;
  version: string;
  category: string;
  scope: string;
  steps: ProcedureStep[];
  approvedBy: string;
  approvedAt: string;
  reviewDate: string;
}

export interface ProcedureStep {
  number: number;
  action: string;
  responsibility: string;
  documentation?: string;
  criticalStep: boolean;
}

export interface Workflow {
  id: string;
  name: string;
  type: string;
  steps: string[];
  automation: number;
}

export interface TrainingProgram {
  requirements: TrainingRequirement[];
  records: boolean;
  refreshInterval: string;
  competencyAssessment: boolean;
}

export interface TrainingRequirement {
  topic: string;
  frequency: string;
  mandatory: boolean;
  roles: string[];
}

export interface DocumentationSystem {
  format: string;
  storage: string;
  retention: string;
  version: boolean;
  audit: boolean;
}

export interface SafetyManagement {
  policies: SafetyPolicy[];
  hazards: HazardAssessment[];
  emergency: EmergencyProcedures;
  incidents: IncidentManagement;
  ppe: PPERequirements;
}

export interface SafetyPolicy {
  id: string;
  title: string;
  scope: string;
  requirements: string[];
  review: string;
}

export interface HazardAssessment {
  hazard: string;
  location: string[];
  risk: 'low' | 'medium' | 'high';
  controls: string[];
  monitoring: string;
}

export interface EmergencyProcedures {
  contacts: EmergencyContact[];
  procedures: EmergencyProcedure[];
  drills: DrillSchedule;
  equipment: string[];
}

export interface EmergencyContact {
  role: string;
  name: string;
  phone: string;
  available: string;
}

export interface EmergencyProcedure {
  scenario: string;
  steps: string[];
  responsible: string;
  notification: string[];
}

export interface DrillSchedule {
  type: string;
  frequency: string;
  lastDrill: string;
  nextDrill: string;
}

export interface IncidentManagement {
  reporting: string;
  investigation: string;
  corrective: string;
  tracking: boolean;
}

export interface PPERequirements {
  zones: { zone: string; required: string[] }[];
  training: boolean;
  inspection: string;
}

export interface QualityManagement {
  system: string;
  audits: AuditProgram;
  deviations: DeviationManagement;
  capa: CAPASystem;
}

export interface AuditProgram {
  internal: { frequency: string; scope: string[] };
  external: { frequency: string; bodies: string[] };
  tracking: boolean;
}

export interface DeviationManagement {
  categories: string[];
  investigation: string;
  timeline: string;
}

export interface CAPASystem {
  enabled: boolean;
  workflow: string;
  tracking: boolean;
  effectiveness: string;
}

export interface EnvironmentalControl {
  monitoring: EnvironmentalMonitoring;
  controls: EnvironmentalControls;
  alerts: EnvironmentalAlerts;
}

export interface EnvironmentalMonitoring {
  parameters: MonitoredParameter[];
  frequency: string;
  logging: boolean;
}

export interface MonitoredParameter {
  name: string;
  unit: string;
  range: { min: number; max: number };
  locations: string[];
}

export interface EnvironmentalControls {
  hvac: boolean;
  humidification: boolean;
  filtration: string;
  pressurization: boolean;
}

export interface EnvironmentalAlerts {
  enabled: boolean;
  thresholds: { parameter: string; warning: number; critical: number }[];
  notifications: string[];
}

export interface MonitoringSystem {
  realtime: RealtimeConfig;
  alerts: AlertSystem;
  reporting: ReportingConfig;
  integration: IntegrationConfig;
}

export interface RealtimeConfig {
  enabled: boolean;
  dashboard: string;
  refresh: number;
}

export interface AlertSystem {
  channels: string[];
  escalation: { levels: number; timeout: number };
  acknowledgement: string;
}

export interface ReportingConfig {
  automated: { type: string; frequency: string; recipients: string[] }[];
  onDemand: string[];
}

export interface IntegrationConfig {
  lims: boolean;
  buildingManagement: boolean;
  external: string[];
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface ProjectResponse {
  id: string;
  name: string;
  status: FacilityStatus;
  createdAt: string;
  updatedAt?: string;
}

export interface ValidationResult {
  valid: boolean;
  errors?: ValidationError[];
}

export interface ValidationError {
  path: string;
  message: string;
  value?: unknown;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    total: number;
    limit: number;
    offset: number;
    hasMore: boolean;
  };
}
