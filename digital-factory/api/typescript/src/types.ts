/**
 * WIA-IND-028: Digital Factory - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * 3D position vector
 */
export interface Vector3D {
  x: number;
  y: number;
  z: number;
}

/**
 * 2D position
 */
export interface Vector2D {
  x: number;
  y: number;
}

/**
 * GPS coordinates
 */
export interface GPSCoordinates {
  latitude: number;
  longitude: number;
  altitude?: number;
}

/**
 * Timestamp with timezone
 */
export type Timestamp = Date | string;

/**
 * Factory identifier
 */
export type FactoryId = string;

/**
 * Digital twin identifier
 */
export type TwinId = string;

/**
 * Component identifier
 */
export type ComponentId = string;

/**
 * Simulation identifier
 */
export type SimulationId = string;

// ============================================================================
// Digital Factory SDK Configuration
// ============================================================================

/**
 * Digital Factory SDK configuration
 */
export interface DigitalFactorySDKConfig {
  factoryId: FactoryId;
  apiEndpoint?: string;
  apiKey?: string;
  enableRealTimeSync?: boolean;
  syncInterval?: number; // milliseconds
  security?: SecurityConfig;
  timeout?: number; // milliseconds
}

/**
 * Security configuration
 */
export interface SecurityConfig {
  encryption?: 'AES-256-GCM' | 'AES-128-GCM';
  authentication?: 'OAuth2' | 'API-Key' | 'JWT';
  authorization?: 'RBAC' | 'ABAC';
  auditLogging?: boolean;
  dataResidency?: 'US' | 'EU' | 'APAC';
}

// ============================================================================
// Factory Types
// ============================================================================

/**
 * Factory definition
 */
export interface Factory {
  factoryId: FactoryId;
  name: string;
  location: Location;
  size: {
    area: number; // square meters
    unit: 'sqm' | 'sqft';
  };
  layout?: FactoryLayout;
  infrastructure: Infrastructure;
  productionLines?: ProductionLine[];
  metadata: Metadata;
}

/**
 * Location information
 */
export interface Location {
  address: string;
  gps: GPSCoordinates;
  timezone?: string;
  country?: string;
}

/**
 * Factory infrastructure
 */
export interface Infrastructure {
  power: PowerInfrastructure;
  compressedAir?: CompressedAirInfrastructure;
  hvac?: HVACInfrastructure;
  network?: NetworkInfrastructure;
}

/**
 * Power infrastructure
 */
export interface PowerInfrastructure {
  mainCapacity: number; // kW
  voltage: number;
  phases: 1 | 3;
  frequency: 50 | 60; // Hz
  renewableCapacity?: number; // kW
}

/**
 * Compressed air infrastructure
 */
export interface CompressedAirInfrastructure {
  pressure: number; // psi or bar
  unit: 'psi' | 'bar';
  capacity: number; // CFM or m³/h
}

/**
 * HVAC infrastructure
 */
export interface HVACInfrastructure {
  coolingCapacity: number; // tons or kW
  heatingCapacity: number; // kW or BTU/h
  ventilationRate: number; // CFM or m³/h
}

/**
 * Network infrastructure
 */
export interface NetworkInfrastructure {
  bandwidth: number; // Mbps
  type: '5G' | 'WiFi-6' | 'Ethernet' | 'TSN';
  coverage: 'full' | 'partial';
}

/**
 * Factory layout
 */
export interface FactoryLayout {
  buildingGeometry: string; // 3D model URL or base64
  format: 'gltf' | 'step' | 'iges' | 'stl';
  zones: Zone[];
  equipment: EquipmentPlacement[];
}

/**
 * Factory zone
 */
export interface Zone {
  zoneId: string;
  name: string;
  type: 'production' | 'storage' | 'office' | 'maintenance' | 'hazardous';
  area: number; // square meters
  boundaries: Vector2D[];
}

/**
 * Equipment placement in factory
 */
export interface EquipmentPlacement {
  equipmentId: ComponentId;
  position: Vector3D;
  orientation: {
    rx: number; // degrees
    ry: number;
    rz: number;
  };
  zoneId?: string;
}

/**
 * Production line
 */
export interface ProductionLine {
  lineId: string;
  name: string;
  type: 'assembly' | 'machining' | 'welding' | 'packaging';
  status: 'running' | 'idle' | 'maintenance' | 'error';
  workstations: number;
  targetOutput: number; // units per hour
  currentOutput?: number;
  efficiency?: number; // percentage
}

/**
 * Metadata for versioning and tracking
 */
export interface Metadata {
  created: Timestamp;
  updated: Timestamp;
  createdBy?: string;
  updatedBy?: string;
  version?: string;
  tags?: string[];
}

// ============================================================================
// Digital Twin Types
// ============================================================================

/**
 * Digital twin
 */
export interface DigitalTwin {
  twinId: TwinId;
  factoryId: FactoryId;
  name: string;
  fidelityLevel: 0 | 1 | 2 | 3 | 4 | 5;
  synchronization: SynchronizationConfig;
  components: TwinComponent[];
  predictiveModels?: PredictiveModel[];
  metadata: Metadata;
}

/**
 * Digital twin synchronization configuration
 */
export interface SynchronizationConfig {
  enabled: boolean;
  method: 'real-time' | 'near-real-time' | 'batch';
  frequency: number; // milliseconds or seconds
  latency?: number; // milliseconds
  dataSources?: string[];
  enablePredictiveAnalytics?: boolean;
}

/**
 * Digital twin component
 */
export interface TwinComponent {
  componentId: ComponentId;
  type: ComponentType;
  physicalId?: string;
  model?: string; // 3D model URL
  position: Vector3D;
  orientation: { rx: number; ry: number; rz: number };
  sensors?: Sensor[];
  status?: ComponentStatus;
  properties?: Record<string, any>;
}

/**
 * Component type
 */
export type ComponentType =
  | 'robot-arm'
  | 'cnc-machine'
  | 'conveyor'
  | 'agv'
  | 'plc'
  | 'sensor'
  | 'camera'
  | 'hvac'
  | 'lighting'
  | 'other';

/**
 * Component status
 */
export interface ComponentStatus {
  operational: 'running' | 'idle' | 'maintenance' | 'error' | 'offline';
  health: number; // 0-100 percentage
  alarms?: Alarm[];
  runtime?: number; // hours
  cycleCount?: number;
}

/**
 * Sensor data
 */
export interface Sensor {
  sensorId: string;
  type: SensorType;
  value: number | boolean | string;
  unit?: string;
  timestamp: Timestamp;
  quality?: 'good' | 'uncertain' | 'bad';
}

/**
 * Sensor type
 */
export type SensorType =
  | 'temperature'
  | 'vibration'
  | 'pressure'
  | 'position'
  | 'velocity'
  | 'current'
  | 'voltage'
  | 'flow'
  | 'level'
  | 'proximity'
  | 'other';

/**
 * Alarm
 */
export interface Alarm {
  alarmId: string;
  severity: 'critical' | 'warning' | 'info';
  message: string;
  timestamp: Timestamp;
  acknowledged: boolean;
  acknowledgedBy?: string;
}

/**
 * Predictive model
 */
export interface PredictiveModel {
  modelId: string;
  type: 'predictive-maintenance' | 'quality-prediction' | 'energy-forecasting' | 'throughput-prediction';
  algorithm: string;
  accuracy: number; // 0-1
  lastTrained: Timestamp;
  parameters?: Record<string, any>;
}

// ============================================================================
// Virtual Commissioning Types
// ============================================================================

/**
 * Virtual commissioning configuration
 */
export interface VirtualCommissioning {
  commissioningId: string;
  productionLineId: string;
  equipment: VirtualEquipment[];
  productionScenarios: ProductionScenario[];
  testCases?: TestCase[];
  status: 'planned' | 'in-progress' | 'completed' | 'failed';
}

/**
 * Virtual equipment
 */
export interface VirtualEquipment {
  type: string;
  model: string;
  quantity: number;
  specifications?: Record<string, any>;
}

/**
 * Production scenario
 */
export interface ProductionScenario {
  productId: string;
  cycleTime: number; // seconds
  batch: number;
  setupTime?: number; // seconds
}

/**
 * Test case
 */
export interface TestCase {
  name: string;
  type: 'startup' | 'normal-operation' | 'emergency-stop' | 'changeover' | 'fault-injection';
  iterations: number;
  expectedResult?: string;
}

/**
 * Virtual commissioning test results
 */
export interface VirtualCommissioningResults {
  commissioningId: string;
  successRate: number; // percentage
  avgCycleTime: number; // seconds
  issues: Issue[];
  report?: string; // URL or content
}

/**
 * Issue found during virtual commissioning
 */
export interface Issue {
  issueId: string;
  severity: 'critical' | 'major' | 'minor';
  description: string;
  component?: string;
  recommendation?: string;
}

// ============================================================================
// Production Simulation Types
// ============================================================================

/**
 * Production simulation
 */
export interface ProductionSimulation {
  simulationId: SimulationId;
  name: string;
  type: 'discrete-event' | 'agent-based' | 'system-dynamics' | 'physics';
  scenario: string;
  duration: number; // seconds
  parameters: SimulationParameters;
  status: 'queued' | 'running' | 'completed' | 'failed';
  results?: SimulationResults;
}

/**
 * Simulation parameters
 */
export interface SimulationParameters {
  productMix: ProductMix[];
  shifts?: number;
  overtime?: number; // hours
  constraints?: SimulationConstraints;
  randomSeed?: number;
}

/**
 * Product mix
 */
export interface ProductMix {
  productId: string;
  demandRate: number; // units per hour
}

/**
 * Simulation constraints
 */
export interface SimulationConstraints {
  maxOvertimeHours?: number;
  minBufferStock?: number;
  maxEnergyConsumption?: number; // kWh
  maxCost?: number;
}

/**
 * Simulation results
 */
export interface SimulationResults {
  throughput: number; // units
  utilization: number; // percentage
  bottlenecks: string[];
  energyCost: number; // currency
  cycleTime: number; // seconds
  wip: number; // work-in-process units
  metrics?: Record<string, number>;
}

// ============================================================================
// Layout Optimization Types
// ============================================================================

/**
 * Layout optimization request
 */
export interface LayoutOptimization {
  optimizationId: string;
  objectives: LayoutObjectives;
  constraints: LayoutConstraints;
  currentLayout?: FactoryLayout;
  status: 'queued' | 'running' | 'completed' | 'failed';
}

/**
 * Layout objectives (weights must sum to 1)
 */
export interface LayoutObjectives {
  minimizeMaterialHandling: number; // 0-1
  maximizeWorkflow: number; // 0-1
  minimizeFootprint: number; // 0-1
  improveSafety: number; // 0-1
}

/**
 * Layout constraints
 */
export interface LayoutConstraints {
  minAisleWidth: number; // meters
  safetyZones: boolean;
  emergencyExits: number;
  maxBuildingArea?: number; // square meters
  fixedEquipment?: string[]; // IDs of equipment that cannot be moved
}

/**
 * Layout optimization results
 */
export interface LayoutOptimizationResults {
  optimizationId: string;
  optimizedLayout: FactoryLayout;
  improvements: LayoutImprovements;
  score: number; // overall score
}

/**
 * Layout improvements
 */
export interface LayoutImprovements {
  materialHandling: number; // percentage reduction
  workflow: number; // percentage increase
  footprint: number; // square meters saved
  safetyScore: number; // 0-100
}

// ============================================================================
// Energy Management Types
// ============================================================================

/**
 * Energy management system
 */
export interface EnergyManagement {
  factoryId: FactoryId;
  currentConsumption?: EnergyConsumption;
  forecast?: EnergyForecast;
  optimization?: EnergyOptimization;
}

/**
 * Current energy consumption
 */
export interface EnergyConsumption {
  timestamp: Timestamp;
  total: number; // kW
  breakdown: Record<string, number>; // by category
  powerFactor: number;
  peakDemand: number; // kW
  cost: number; // currency per hour
}

/**
 * Energy forecast
 */
export interface EnergyForecast {
  period: string; // e.g., "next-24-hours"
  predictions: EnergyPrediction[];
  accuracy?: number; // MAPE
}

/**
 * Energy prediction
 */
export interface EnergyPrediction {
  timestamp: Timestamp;
  consumption: number; // kWh
  cost: number; // currency
  confidence: number; // 0-1
}

/**
 * Energy optimization
 */
export interface EnergyOptimization {
  targetReduction: number; // percentage
  constraints: EnergyOptimizationConstraints;
  strategies: EnergyStrategy[];
  recommendations?: EnergyRecommendation[];
  estimatedSavings?: number; // currency per year
}

/**
 * Energy optimization constraints
 */
export interface EnergyOptimizationConstraints {
  maintainProduction: boolean;
  peakDemandLimit?: number; // kW
  minComfortLevel?: number; // for HVAC
}

/**
 * Energy strategy
 */
export type EnergyStrategy =
  | 'load-shifting'
  | 'demand-response'
  | 'renewable-integration'
  | 'equipment-scheduling'
  | 'waste-heat-recovery';

/**
 * Energy recommendation
 */
export interface EnergyRecommendation {
  action: string;
  estimatedSavings: number; // currency per year
  paybackPeriod?: number; // years
  priority: 'high' | 'medium' | 'low';
}

// ============================================================================
// Worker Safety Types
// ============================================================================

/**
 * Worker safety management
 */
export interface WorkerSafety {
  factoryId: FactoryId;
  safetyZones: SafetyZone[];
  incidents?: SafetyIncident[];
  metrics?: SafetyMetrics;
}

/**
 * Safety zone
 */
export interface SafetyZone {
  zoneId: string;
  type: 'restricted' | 'hazardous' | 'collaborative' | 'safe';
  area: { x: number; y: number; width: number; height: number };
  maxOccupancy?: number;
  requiredPPE?: PPEType[];
  hazards?: string[];
  sensors?: string[];
}

/**
 * PPE (Personal Protective Equipment) type
 */
export type PPEType =
  | 'hard-hat'
  | 'safety-glasses'
  | 'high-visibility-vest'
  | 'gloves'
  | 'steel-toed-boots'
  | 'ear-protection'
  | 'respirator';

/**
 * Safety incident
 */
export interface SafetyIncident {
  incidentId: string;
  type: 'injury' | 'near-miss' | 'ppe-violation' | 'unsafe-behavior';
  severity: 'critical' | 'major' | 'minor';
  timestamp: Timestamp;
  location: Vector3D;
  workerId?: string;
  description: string;
  actions: string[];
}

/**
 * Safety metrics
 */
export interface SafetyMetrics {
  nearMissFrequency: number; // per month
  ppeComplianceRate: number; // percentage
  injuryRate: number; // OSHA TRIR
  lostTimeInjuryFrequency: number; // LTIF
  safetyTrainingCompletion: number; // percentage
}

// ============================================================================
// AR/VR Training Types
// ============================================================================

/**
 * AR/VR training
 */
export interface ARVRTraining {
  trainingId: string;
  title: string;
  type: 'vr' | 'ar' | 'mr';
  equipment?: string[];
  duration: number; // seconds
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  includesHazards: boolean;
  content?: TrainingContent;
}

/**
 * Training content
 */
export interface TrainingContent {
  modules: TrainingModule[];
  assessments?: Assessment[];
}

/**
 * Training module
 */
export interface TrainingModule {
  moduleId: string;
  name: string;
  type: 'operation' | 'maintenance' | 'safety' | 'quality';
  duration: number; // seconds
  scenarios: Scenario[];
}

/**
 * Training scenario
 */
export interface Scenario {
  scenarioId: string;
  name: string;
  description: string;
  steps: ScenarioStep[];
  passCriteria?: PassCriteria;
}

/**
 * Scenario step
 */
export interface ScenarioStep {
  stepId: string;
  instruction: string;
  expectedAction: string;
  hints?: string[];
  timeLimit?: number; // seconds
}

/**
 * Pass criteria
 */
export interface PassCriteria {
  minScore: number; // percentage
  maxErrors: number;
  maxTime?: number; // seconds
}

/**
 * Assessment
 */
export interface Assessment {
  assessmentId: string;
  questions: Question[];
  passingScore: number; // percentage
}

/**
 * Question
 */
export interface Question {
  questionId: string;
  text: string;
  type: 'multiple-choice' | 'true-false' | 'fill-in-blank';
  options?: string[];
  correctAnswer: string | string[];
  points: number;
}

/**
 * Training session
 */
export interface TrainingSession {
  sessionId: string;
  trainingId: string;
  traineeId: string;
  supervisorId?: string;
  startTime: Timestamp;
  endTime?: Timestamp;
  status: 'in-progress' | 'completed' | 'failed' | 'cancelled';
  score?: number; // percentage
  errors?: number;
  completionTime?: number; // seconds
}

// ============================================================================
// KPI Dashboard Types
// ============================================================================

/**
 * KPI dashboard
 */
export interface KPIDashboard {
  dashboardId: string;
  factoryId: FactoryId;
  type: 'executive' | 'operations' | 'maintenance' | 'quality' | 'energy';
  kpis: KPI[];
  refreshInterval: number; // milliseconds
}

/**
 * KPI (Key Performance Indicator)
 */
export interface KPI {
  kpiId: string;
  name: string;
  value: number;
  unit?: string;
  target?: number;
  trend?: 'up' | 'down' | 'stable';
  status?: 'good' | 'warning' | 'critical';
  timestamp: Timestamp;
}

/**
 * Dashboard widget
 */
export interface DashboardWidget {
  widgetId: string;
  type: 'chart' | 'gauge' | 'table' | 'map' | '3d-view';
  title: string;
  dataSource: string;
  configuration: Record<string, any>;
}

// ============================================================================
// Connected Worker Types
// ============================================================================

/**
 * Connected worker
 */
export interface ConnectedWorker {
  workerId: string;
  name: string;
  role: string;
  devices: WorkerDevice[];
  location?: Vector3D;
  status: 'active' | 'break' | 'offline';
  tasks?: WorkerTask[];
}

/**
 * Worker device
 */
export interface WorkerDevice {
  deviceId: string;
  type: 'smartphone' | 'tablet' | 'smartwatch' | 'smart-glasses' | 'smart-helmet';
  model: string;
  batteryLevel?: number; // percentage
  connected: boolean;
}

/**
 * Worker task
 */
export interface WorkerTask {
  taskId: string;
  title: string;
  type: 'production' | 'maintenance' | 'quality' | 'safety';
  priority: 'high' | 'medium' | 'low';
  status: 'assigned' | 'in-progress' | 'completed' | 'cancelled';
  assignedAt: Timestamp;
  dueBy?: Timestamp;
  completedAt?: Timestamp;
  instructions?: string;
}

// ============================================================================
// Factory-as-a-Service Types
// ============================================================================

/**
 * Factory-as-a-Service configuration
 */
export interface FactoryAsAService {
  serviceId: string;
  type: 'DTaaS' | 'SimaaS' | 'AIaaS' | 'AaaS' | 'TaaS';
  pricing: ServicePricing;
  sla: ServiceLevelAgreement;
}

/**
 * Service pricing
 */
export interface ServicePricing {
  model: 'consumption' | 'subscription' | 'per-unit' | 'tiered';
  basePrice: number; // currency
  billingPeriod: 'hourly' | 'daily' | 'monthly' | 'annual';
  overageRate?: number; // currency per unit over limit
}

/**
 * Service Level Agreement
 */
export interface ServiceLevelAgreement {
  availability: number; // percentage (e.g., 99.9)
  maxLatency: number; // milliseconds
  support: 'business-hours' | '24x7';
  responseTime: {
    critical: number; // minutes
    high: number;
    medium: number;
    low: number;
  };
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * Factory event
 */
export interface FactoryEvent {
  eventId: string;
  eventType: EventType;
  factoryId: FactoryId;
  timestamp: Timestamp;
  data: any;
  severity?: 'info' | 'warning' | 'critical';
}

/**
 * Event type
 */
export type EventType =
  | 'twin-sync'
  | 'component-status'
  | 'alarm'
  | 'production'
  | 'energy'
  | 'safety-violation'
  | 'quality-issue'
  | 'maintenance'
  | 'simulation-complete'
  | 'optimization-complete';

/**
 * Event listener
 */
export type EventListener = (event: FactoryEvent) => void;

// ============================================================================
// API Response Types
// ============================================================================

/**
 * API response wrapper
 */
export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  timestamp: Timestamp;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  pageSize: number;
  totalPages: number;
}

/**
 * Pagination parameters
 */
export interface PaginationParams {
  page: number;
  pageSize: number;
}

// ============================================================================
// Connection Types
// ============================================================================

/**
 * Connection options
 */
export interface ConnectionOptions {
  timeout?: number; // milliseconds
  retryAttempts?: number;
  retryDelay?: number; // milliseconds
  keepAlive?: boolean;
}

/**
 * WebSocket connection
 */
export interface WebSocketConnection {
  connected: boolean;
  url: string;
  reconnecting: boolean;
  lastError?: string;
}

// **弘益人間 (홍익인간) · Benefit All Humanity**
