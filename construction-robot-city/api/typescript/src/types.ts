/**
 * WIA Construction Robot City Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Robot City Types
// ============================================================================

export interface WIARobotCityProject {
  standard: 'WIA-CONSTRUCTION-ROBOT-CITY';
  version: string;
  metadata: ProjectMetadata;
  cityDesign: CityDesign;
  robotFleet: RobotFleet;
  construction: ConstructionPlan;
  infrastructure: InfrastructureSystem;
  automation: AutomationFramework;
  safety: SafetyProtocol;
  monitoring: MonitoringSystem;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  type: CityType;
  location: ProjectLocation;
  scale: CityScale;
  timeline: ProjectTimeline;
  developer: Organization;
  stakeholders?: Stakeholder[];
  status: ProjectStatus;
}

export type CityType =
  | 'smart-city'
  | 'industrial-zone'
  | 'residential-district'
  | 'mixed-use'
  | 'research-hub'
  | 'logistics-center';

export interface ProjectLocation {
  country: string;
  region?: string;
  city?: string;
  coordinates: Coordinates;
  area: AreaSpec;
  terrain: TerrainType;
}

export interface Coordinates {
  latitude: number;
  longitude: number;
  elevation?: number;
}

export interface AreaSpec {
  value: number;
  unit: 'km2' | 'hectares' | 'acres';
  boundaries?: GeoJSON;
}

export interface GeoJSON {
  type: 'Polygon' | 'MultiPolygon';
  coordinates: number[][][];
}

export type TerrainType =
  | 'flat'
  | 'hilly'
  | 'coastal'
  | 'desert'
  | 'mixed';

export interface CityScale {
  population: number;
  buildings: number;
  roads: number;
  greenSpace: number;
}

export interface ProjectTimeline {
  startDate: string;
  endDate?: string;
  phases: ConstructionPhase[];
  milestones: Milestone[];
}

export interface ConstructionPhase {
  id: string;
  name: string;
  start: string;
  end: string;
  scope: string[];
  robotsRequired: number;
  status: PhaseStatus;
}

export type PhaseStatus =
  | 'planning'
  | 'preparation'
  | 'active'
  | 'completed'
  | 'paused';

export interface Milestone {
  id: string;
  name: string;
  date: string;
  criteria: string;
  status: 'pending' | 'achieved' | 'delayed';
}

export interface Organization {
  name: string;
  type: 'developer' | 'contractor' | 'technology' | 'government';
  country: string;
  contact?: ContactInfo;
}

export interface ContactInfo {
  name: string;
  email: string;
  phone?: string;
}

export interface Stakeholder {
  name: string;
  role: string;
  organization?: string;
}

export type ProjectStatus =
  | 'concept'
  | 'design'
  | 'approved'
  | 'construction'
  | 'testing'
  | 'operational'
  | 'maintenance';

// ============================================================================
// City Design
// ============================================================================

export interface CityDesign {
  masterPlan: MasterPlan;
  zones: CityZone[];
  buildings: BuildingPlan[];
  transportation: TransportationNetwork;
  utilities: UtilityNetwork;
  greenSpaces: GreenSpacePlan;
}

export interface MasterPlan {
  designPhilosophy: string;
  densityTarget: number;
  sustainabilityGoals: string[];
  smartFeatures: string[];
  blueprints: Blueprint[];
}

export interface Blueprint {
  id: string;
  name: string;
  type: string;
  format: 'BIM' | 'CAD' | 'IFC' | 'CityGML';
  url?: string;
  version: string;
}

export interface CityZone {
  id: string;
  name: string;
  type: ZoneType;
  area: AreaSpec;
  coordinates: Coordinates;
  regulations: ZoneRegulation[];
  buildingLimit: number;
}

export type ZoneType =
  | 'residential'
  | 'commercial'
  | 'industrial'
  | 'mixed'
  | 'green'
  | 'infrastructure';

export interface ZoneRegulation {
  type: string;
  value: string;
  mandatory: boolean;
}

export interface BuildingPlan {
  id: string;
  name: string;
  type: BuildingType;
  zone: string;
  floors: number;
  height: number;
  footprint: number;
  materials: MaterialSpec[];
  robotConstructed: boolean;
  timeline: { start: string; end: string };
}

export type BuildingType =
  | 'residential'
  | 'office'
  | 'retail'
  | 'industrial'
  | 'civic'
  | 'educational'
  | 'healthcare';

export interface MaterialSpec {
  material: string;
  quantity: number;
  unit: string;
  sustainable: boolean;
  robotCompatible: boolean;
}

export interface TransportationNetwork {
  roads: RoadNetwork;
  publicTransit: TransitSystem;
  pedestrian: PedestrianNetwork;
  autonomous: AutonomousVehicleNetwork;
}

export interface RoadNetwork {
  totalLength: number;
  lanes: number;
  surfaceType: string;
  smartFeatures: string[];
}

export interface TransitSystem {
  type: string[];
  stations: number;
  coverage: number;
  capacity: number;
}

export interface PedestrianNetwork {
  walkways: number;
  bikeways: number;
  accessibility: string;
}

export interface AutonomousVehicleNetwork {
  coverage: number;
  chargingStations: number;
  controlCenter: string;
}

export interface UtilityNetwork {
  electricity: ElectricityGrid;
  water: WaterSystem;
  waste: WasteSystem;
  telecommunications: TelecomSystem;
}

export interface ElectricityGrid {
  capacity: number;
  renewable: number;
  smartGrid: boolean;
  storage: number;
}

export interface WaterSystem {
  source: string;
  treatment: string;
  recycling: number;
  smartMetering: boolean;
}

export interface WasteSystem {
  collection: string;
  recycling: number;
  processing: string;
  automation: boolean;
}

export interface TelecomSystem {
  coverage: string;
  technology: string[];
  bandwidth: number;
  redundancy: boolean;
}

export interface GreenSpacePlan {
  percentage: number;
  parks: Park[];
  vegetation: VegetationPlan;
  ecosystem: EcosystemGoals;
}

export interface Park {
  id: string;
  name: string;
  area: number;
  type: 'urban' | 'nature' | 'recreational';
  features: string[];
}

export interface VegetationPlan {
  trees: number;
  coverage: number;
  species: string[];
  maintenance: string;
}

export interface EcosystemGoals {
  biodiversity: string;
  wildlife: string[];
  sustainability: string[];
}

// ============================================================================
// Robot Fleet
// ============================================================================

export interface RobotFleet {
  totalRobots: number;
  categories: RobotCategory[];
  robots: Robot[];
  controlSystem: ControlSystem;
  maintenance: MaintenancePlan;
}

export interface RobotCategory {
  type: RobotType;
  count: number;
  capabilities: string[];
  manufacturer?: string;
}

export type RobotType =
  | 'excavation'
  | 'foundation'
  | 'structural'
  | 'finishing'
  | 'logistics'
  | 'inspection'
  | 'maintenance'
  | 'demolition'
  | 'landscaping'
  | 'utility';

export interface Robot {
  id: string;
  name: string;
  type: RobotType;
  model: string;
  manufacturer: string;
  capabilities: RobotCapability[];
  specifications: RobotSpecs;
  status: RobotStatus;
  location?: Coordinates;
  currentTask?: string;
}

export interface RobotCapability {
  name: string;
  description: string;
  parameters?: Record<string, unknown>;
}

export interface RobotSpecs {
  weight: number;
  dimensions: { length: number; width: number; height: number };
  speed: number;
  payload: number;
  batteryLife: number;
  operatingTemp: { min: number; max: number };
  sensors: string[];
  communication: string[];
}

export type RobotStatus =
  | 'idle'
  | 'operating'
  | 'charging'
  | 'maintenance'
  | 'error'
  | 'offline';

export interface ControlSystem {
  type: 'centralized' | 'distributed' | 'hybrid';
  aiEngine: AIEngine;
  coordination: CoordinationProtocol;
  communication: CommunicationSystem;
}

export interface AIEngine {
  model: string;
  capabilities: string[];
  learningEnabled: boolean;
  decisionLatency: number;
}

export interface CoordinationProtocol {
  algorithm: string;
  conflictResolution: string;
  taskAllocation: string;
  swarmBehavior?: boolean;
}

export interface CommunicationSystem {
  protocol: string;
  frequency: string;
  range: number;
  redundancy: boolean;
  encryption: boolean;
}

export interface MaintenancePlan {
  schedule: MaintenanceSchedule;
  facilities: MaintenanceFacility[];
  spares: SpareInventory;
}

export interface MaintenanceSchedule {
  preventive: string;
  predictive: boolean;
  emergency: string;
}

export interface MaintenanceFacility {
  id: string;
  name: string;
  location: Coordinates;
  capacity: number;
  capabilities: string[];
}

export interface SpareInventory {
  categories: { type: string; quantity: number }[];
  reorderPoint: number;
  supplier: string;
}

// ============================================================================
// Construction & Automation
// ============================================================================

export interface ConstructionPlan {
  methodology: ConstructionMethodology;
  tasks: ConstructionTask[];
  materials: MaterialPlan;
  logistics: LogisticsPlan;
  quality: QualityPlan;
}

export interface ConstructionMethodology {
  approach: 'modular' | 'traditional' | 'hybrid' | '3d-printing';
  automation: number;
  humanRoles: string[];
  robotRoles: string[];
}

export interface ConstructionTask {
  id: string;
  name: string;
  type: TaskType;
  zone: string;
  building?: string;
  robotType: RobotType[];
  duration: number;
  dependencies: string[];
  status: TaskStatus;
  progress?: number;
}

export type TaskType =
  | 'site-preparation'
  | 'excavation'
  | 'foundation'
  | 'structure'
  | 'envelope'
  | 'interior'
  | 'mechanical'
  | 'electrical'
  | 'finishing'
  | 'landscaping';

export type TaskStatus =
  | 'pending'
  | 'scheduled'
  | 'in-progress'
  | 'completed'
  | 'blocked'
  | 'cancelled';

export interface MaterialPlan {
  requirements: MaterialRequirement[];
  suppliers: Supplier[];
  storage: StorageFacility[];
  tracking: TrackingSystem;
}

export interface MaterialRequirement {
  material: string;
  quantity: number;
  unit: string;
  deliverySchedule: string;
  quality: string;
}

export interface Supplier {
  name: string;
  materials: string[];
  leadTime: number;
  reliability: number;
  location: string;
}

export interface StorageFacility {
  id: string;
  location: Coordinates;
  capacity: number;
  materials: string[];
  robotAccess: boolean;
}

export interface TrackingSystem {
  technology: 'RFID' | 'barcode' | 'GPS' | 'blockchain';
  realTime: boolean;
  integration: string[];
}

export interface LogisticsPlan {
  routes: LogisticsRoute[];
  vehicles: LogisticsVehicle[];
  scheduling: SchedulingSystem;
}

export interface LogisticsRoute {
  id: string;
  from: string;
  to: string;
  distance: number;
  capacity: number;
  autonomous: boolean;
}

export interface LogisticsVehicle {
  id: string;
  type: 'truck' | 'drone' | 'robot-carrier' | 'conveyor';
  capacity: number;
  autonomous: boolean;
  status: string;
}

export interface SchedulingSystem {
  algorithm: string;
  optimization: string[];
  constraints: string[];
}

export interface QualityPlan {
  standards: string[];
  inspections: InspectionPlan;
  testing: TestingProtocol;
  documentation: string;
}

export interface InspectionPlan {
  automated: boolean;
  frequency: string;
  checkpoints: string[];
  robotInspectors: boolean;
}

export interface TestingProtocol {
  types: string[];
  equipment: string[];
  acceptance: string;
}

export interface AutomationFramework {
  level: AutomationLevel;
  systems: AutomatedSystem[];
  integration: SystemIntegration;
  humanInterface: HumanInterface;
}

export type AutomationLevel = 1 | 2 | 3 | 4 | 5;

export interface AutomatedSystem {
  name: string;
  function: string;
  automation: number;
  reliability: number;
  failsafe: string;
}

export interface SystemIntegration {
  platform: string;
  protocols: string[];
  dataExchange: string;
  interoperability: string[];
}

export interface HumanInterface {
  controlCenter: ControlCenterSpec;
  monitoring: MonitoringInterface;
  intervention: InterventionProtocol;
}

export interface ControlCenterSpec {
  location: string;
  staffing: number;
  capabilities: string[];
  redundancy: boolean;
}

export interface MonitoringInterface {
  dashboards: string[];
  alerts: string[];
  visualization: string[];
}

export interface InterventionProtocol {
  triggers: string[];
  procedures: string[];
  authority: string[];
}

// ============================================================================
// Infrastructure & Safety
// ============================================================================

export interface InfrastructureSystem {
  digital: DigitalInfrastructure;
  physical: PhysicalInfrastructure;
  energy: EnergyInfrastructure;
  resilience: ResiliencePlan;
}

export interface DigitalInfrastructure {
  network: NetworkSpec;
  dataCenter: DataCenterSpec;
  iotPlatform: IoTPlatform;
  digitalTwin: DigitalTwinSpec;
}

export interface NetworkSpec {
  type: string;
  coverage: number;
  bandwidth: number;
  latency: number;
  redundancy: boolean;
}

export interface DataCenterSpec {
  location: string;
  capacity: number;
  tier: number;
  backup: string;
}

export interface IoTPlatform {
  sensors: number;
  devices: number;
  protocol: string;
  analytics: string;
}

export interface DigitalTwinSpec {
  scope: string;
  realTime: boolean;
  simulation: boolean;
  predictive: boolean;
}

export interface PhysicalInfrastructure {
  foundations: string;
  structures: string;
  utilities: string;
  accessibility: string;
}

export interface EnergyInfrastructure {
  generation: EnergyGeneration;
  storage: EnergyStorage;
  distribution: EnergyDistribution;
  efficiency: number;
}

export interface EnergyGeneration {
  sources: { type: string; capacity: number }[];
  renewable: number;
  backup: string;
}

export interface EnergyStorage {
  type: string;
  capacity: number;
  duration: number;
}

export interface EnergyDistribution {
  grid: string;
  smartMetering: boolean;
  loadBalancing: boolean;
}

export interface ResiliencePlan {
  hazards: string[];
  mitigation: string[];
  recovery: string;
  redundancy: string[];
}

export interface SafetyProtocol {
  standards: string[];
  zones: SafetyZone[];
  procedures: SafetyProcedure[];
  emergency: EmergencyPlan;
  humanSafety: HumanSafetyMeasures;
}

export interface SafetyZone {
  id: string;
  type: 'exclusion' | 'caution' | 'mixed' | 'public';
  boundaries: AreaSpec;
  restrictions: string[];
  monitoring: string[];
}

export interface SafetyProcedure {
  id: string;
  name: string;
  trigger: string;
  actions: string[];
  responsible: string;
}

export interface EmergencyPlan {
  scenarios: EmergencyScenario[];
  resources: EmergencyResource[];
  communication: string;
  evacuation: string;
}

export interface EmergencyScenario {
  type: string;
  response: string[];
  robotBehavior: string;
  humanIntervention: boolean;
}

export interface EmergencyResource {
  type: string;
  location: string;
  quantity: number;
  status: string;
}

export interface HumanSafetyMeasures {
  training: string[];
  ppe: string[];
  protocols: string[];
  monitoring: string;
}

// ============================================================================
// Monitoring & API Types
// ============================================================================

export interface MonitoringSystem {
  realTime: RealTimeMonitoring;
  analytics: AnalyticsPlatform;
  reporting: ReportingSystem;
  alerts: AlertSystem;
}

export interface RealTimeMonitoring {
  dashboards: Dashboard[];
  metrics: Metric[];
  refresh: number;
}

export interface Dashboard {
  id: string;
  name: string;
  type: string;
  widgets: string[];
}

export interface Metric {
  id: string;
  name: string;
  type: string;
  unit: string;
  threshold?: { warning: number; critical: number };
}

export interface AnalyticsPlatform {
  capabilities: string[];
  predictive: boolean;
  optimization: string[];
  reporting: string[];
}

export interface ReportingSystem {
  automated: boolean;
  templates: string[];
  schedule: string;
  formats: string[];
}

export interface AlertSystem {
  channels: string[];
  escalation: string[];
  response: string;
}

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface ProjectResponse {
  id: string;
  name: string;
  status: ProjectStatus;
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
