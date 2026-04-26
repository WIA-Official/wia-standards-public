/**
 * WIA-IND-024: Manufacturing Automation - TypeScript Type Definitions
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
 * Timestamp with timezone
 */
export type Timestamp = Date | string;

/**
 * Manufacturing facility identifier
 */
export type FacilityId = string;

/**
 * Production line identifier
 */
export type LineId = string;

/**
 * Equipment identifier
 */
export type EquipmentId = string;

// ============================================================================
// Production Line Types
// ============================================================================

/**
 * Production line configuration
 */
export interface ProductionLine {
  lineId: LineId;
  facilityId: FacilityId;
  name: string;
  type: 'single-model' | 'mixed-model' | 'batch' | 'continuous';
  status: 'running' | 'idle' | 'maintenance' | 'error' | 'offline';
  workstations: Workstation[];
  conveyors: Conveyor[];
  metrics: LineMetrics;
}

/**
 * Workstation definition
 */
export interface Workstation {
  stationId: string;
  type: 'assembly' | 'machining' | 'welding' | 'inspection' | 'packaging';
  position: number;
  equipment: Equipment[];
  cycleTime: number;
  targetOutput: number;
  currentOutput: number;
  efficiency: number;
}

/**
 * Production line metrics
 */
export interface LineMetrics {
  oee: number;
  availability: number;
  performance: number;
  quality: number;
  targetCycleTime: number;
  actualCycleTime: number;
  unitsProduced: number;
  defectiveUnits: number;
}

/**
 * Line control configuration
 */
export interface LineConfig {
  targetSpeed: number;
  cycleTime: number;
  productModel: string;
  batchSize?: number;
  qualityThreshold?: number;
}

// ============================================================================
// PLC/SCADA Types
// ============================================================================

/**
 * PLC type enumeration
 */
export type PLCType =
  | 'siemens-s7-300'
  | 'siemens-s7-400'
  | 'siemens-s7-1200'
  | 'siemens-s7-1500'
  | 'allen-bradley-controllogix'
  | 'allen-bradley-compactlogix'
  | 'schneider-modicon'
  | 'mitsubishi-melsec'
  | 'omron-cj2m'
  | 'omron-nx';

/**
 * PLC connection configuration
 */
export interface PLCConnection {
  type: PLCType;
  host: string;
  port?: number;
  rack?: number;
  slot?: number;
  timeout?: number;
}

/**
 * PLC address specification
 */
export interface PLCAddress {
  area: 'I' | 'Q' | 'M' | 'DB';
  dbNumber?: number;
  byteOffset: number;
  bitOffset?: number;
  dataType: 'BOOL' | 'BYTE' | 'WORD' | 'DWORD' | 'REAL' | 'STRING';
}

/**
 * OPC UA connection configuration
 */
export interface OPCUAConnection {
  endpoint: string;
  securityMode: 'None' | 'Sign' | 'SignAndEncrypt';
  securityPolicy: 'None' | 'Basic128Rsa15' | 'Basic256' | 'Basic256Sha256';
  authentication: {
    type: 'anonymous' | 'username' | 'certificate';
    username?: string;
    password?: string;
    certificatePath?: string;
    privateKeyPath?: string;
  };
}

/**
 * SCADA tag configuration
 */
export interface SCADATag {
  tagName: string;
  description: string;
  dataSource: string;
  dataType: 'BOOL' | 'INT' | 'REAL' | 'STRING';
  unit?: string;
  scanRate: number;
  deadband?: number;
  alarmLimits?: {
    highHigh?: number;
    high?: number;
    low?: number;
    lowLow?: number;
  };
  trending?: {
    enabled: boolean;
    retentionDays: number;
  };
}

// ============================================================================
// Robot Types
// ============================================================================

/**
 * Robot arm type
 */
export type RobotType =
  | 'articulated-6axis'
  | 'scara-4axis'
  | 'delta-parallel'
  | 'cartesian-gantry'
  | 'collaborative-6axis';

/**
 * Robot arm configuration
 */
export interface RobotArm {
  robotId: EquipmentId;
  manufacturer: string;
  model: string;
  type: RobotType;
  specifications: RobotSpecifications;
  status: EquipmentStatus;
}

/**
 * Robot specifications
 */
export interface RobotSpecifications {
  reach: number; // mm
  payload: number; // kg
  repeatability: number; // mm
  axes: number;
  maxSpeed: {
    joint: number; // deg/s
    tcp: number; // m/s
  };
  weight: number; // kg
}

/**
 * Joint position (joint space)
 */
export interface JointPosition {
  j1: number; // degrees
  j2: number;
  j3: number;
  j4: number;
  j5: number;
  j6: number;
}

/**
 * Cartesian pose (task space)
 */
export interface CartesianPose {
  x: number; // mm
  y: number;
  z: number;
  rx: number; // degrees (roll)
  ry: number; // degrees (pitch)
  rz: number; // degrees (yaw)
}

/**
 * Gripper control interface
 */
export interface GripperControl {
  position: number; // 0-100%
  force: number; // N
  objectDetected: boolean;
}

/**
 * Tool Center Point configuration
 */
export interface TCPConfiguration {
  offset: CartesianPose;
  mass: number; // kg
  cog: Vector3D; // center of gravity
}

/**
 * Vision-guided pick parameters
 */
export interface VisionGuidedPick {
  cameraId: string;
  template: string;
  confidence: number;
  pickPose: CartesianPose;
  placePose: CartesianPose;
  gripForce: number;
}

// ============================================================================
// Conveyor Types
// ============================================================================

/**
 * Conveyor type
 */
export type ConveyorType = 'belt' | 'roller' | 'chain' | 'overhead' | 'slat';

/**
 * Conveyor configuration
 */
export interface Conveyor {
  conveyorId: EquipmentId;
  type: ConveyorType;
  specifications: ConveyorSpecifications;
  status: ConveyorStatus;
}

/**
 * Conveyor specifications
 */
export interface ConveyorSpecifications {
  length: number; // mm
  width: number; // mm
  speed: {
    min: number; // m/s
    max: number; // m/s
    current: number;
  };
  loadCapacity: number; // kg/m
  beltMaterial?: string;
  motorPower?: number; // kW
}

/**
 * Conveyor status
 */
export interface ConveyorStatus {
  running: boolean;
  speed: number; // m/s
  direction: 'forward' | 'reverse' | 'stopped';
  loadStatus: number; // 0-100%
  alarms: Alarm[];
}

/**
 * Part tracking on conveyor
 */
export interface PartPosition {
  partId: string;
  conveyorId: EquipmentId;
  position: number; // mm from origin
  velocity: number; // m/s
  entryTime: Date;
  estimatedExitTime: Date;
}

// ============================================================================
// Assembly Types
// ============================================================================

/**
 * Assembly operation
 */
export interface AssemblyOperation {
  operationId: string;
  type: 'screw-driving' | 'riveting' | 'welding' | 'gluing' | 'pressing';
  parameters: AssemblyParameters;
  result?: AssemblyResult;
}

/**
 * Screw driving parameters
 */
export interface ScrewDrivingParams {
  targetTorque: number; // Nm
  targetAngle?: number; // degrees
  speed: number; // RPM
  angleControl: boolean;
}

/**
 * Screw driving result
 */
export interface ScrewDrivingResult {
  success: boolean;
  finalTorque: number; // Nm
  finalAngle: number; // degrees
  time: number; // seconds
  curve?: TorqueAngleCurve;
}

/**
 * Torque-angle curve
 */
export interface TorqueAngleCurve {
  timestamps: number[];
  torques: number[];
  angles: number[];
}

/**
 * Welding parameters
 */
export interface WeldingParams {
  weldType: 'MIG' | 'TIG' | 'Laser' | 'Spot';
  voltage: number; // V
  current: number; // A
  wireSpeed?: number; // mm/s
  travelSpeed: number; // mm/s
  shieldingGas?: string;
  gasFlowRate?: number; // L/min
}

/**
 * Assembly parameters union
 */
export type AssemblyParameters = ScrewDrivingParams | WeldingParams;

/**
 * Assembly result union
 */
export type AssemblyResult = ScrewDrivingResult;

// ============================================================================
// Quality Inspection Types
// ============================================================================

/**
 * Inspection type
 */
export type InspectionType =
  | 'vision-2d'
  | 'vision-3d'
  | 'dimensional'
  | 'xray'
  | 'ultrasonic'
  | 'electrical'
  | 'leak-test';

/**
 * Quality inspection request
 */
export interface QualityInspection {
  inspectionId: string;
  workstationId: string;
  partId: string;
  inspectionType: InspectionType;
  criteria: InspectionCriteria;
}

/**
 * Inspection criteria
 */
export interface InspectionCriteria {
  dimensions?: {
    tolerance: number; // mm
  };
  surfaceDefects?: {
    maxCount: number;
  };
  colorMatch?: {
    tolerance: number; // Delta E
  };
  electrical?: {
    resistance?: { min: number; max: number };
    voltage?: { min: number; max: number };
  };
}

/**
 * Inspection result
 */
export interface InspectionResult {
  inspectionId: string;
  timestamp: Date;
  passed: boolean;
  measurements: Measurement[];
  defects: Defect[];
  images?: string[]; // URLs or base64
}

/**
 * Measurement data
 */
export interface Measurement {
  featureName: string;
  type: 'length' | 'width' | 'height' | 'diameter' | 'angle' | 'flatness';
  nominal: number;
  measured: number;
  deviation: number;
  tolerance: { upper: number; lower: number };
  passed: boolean;
}

/**
 * Defect detection
 */
export interface Defect {
  defectType: string;
  location: { x: number; y: number };
  severity: 'critical' | 'major' | 'minor';
  confidence: number; // 0-1
}

// ============================================================================
// Material Handling Types
// ============================================================================

/**
 * AGV/AMR type
 */
export type VehicleType = 'wire-guided-agv' | 'laser-guided-agv' | 'vision-amr' | 'lidar-amr';

/**
 * Automated vehicle
 */
export interface AutomatedVehicle {
  vehicleId: EquipmentId;
  type: VehicleType;
  specifications: VehicleSpecifications;
  status: VehicleStatus;
  currentPosition: Vector3D;
  currentTask?: TransportTask;
}

/**
 * Vehicle specifications
 */
export interface VehicleSpecifications {
  payload: number; // kg
  speed: number; // m/s
  batteryCapacity: number; // Ah
  batteryVoltage: number; // V
  chargingTime: number; // hours
}

/**
 * Vehicle status
 */
export interface VehicleStatus {
  state: 'idle' | 'traveling' | 'loading' | 'unloading' | 'charging' | 'error';
  batteryLevel: number; // 0-100%
  position: Vector3D;
  heading: number; // degrees
  velocity: number; // m/s
}

/**
 * Transport task
 */
export interface TransportTask {
  taskId: string;
  vehicleId: EquipmentId;
  pickupLocation: Vector3D;
  dropoffLocation: Vector3D;
  payload: {
    itemId: string;
    weight: number;
  };
  priority: number;
  status: 'pending' | 'in-progress' | 'completed' | 'failed';
}

/**
 * Warehouse location
 */
export interface WarehouseLocation {
  aisle: number;
  level: number;
  column: number;
  occupied: boolean;
  itemId?: string;
}

// ============================================================================
// Production Scheduling Types
// ============================================================================

/**
 * Production order
 */
export interface ProductionOrder {
  orderId: string;
  product: {
    productId: string;
    name: string;
    version: string;
  };
  quantity: number;
  dueDate: Date;
  priority: number;
  status: 'planned' | 'released' | 'in-progress' | 'completed' | 'cancelled';
}

/**
 * Scheduling problem
 */
export interface SchedulingProblem {
  orders: ProductionOrder[];
  resources: Resource[];
  constraints: SchedulingConstraints;
  objectives: SchedulingObjectives;
}

/**
 * Resource definition
 */
export interface Resource {
  resourceId: string;
  type: 'machine' | 'labor' | 'material';
  capacity: number;
  availability: TimeWindow[];
}

/**
 * Time window
 */
export interface TimeWindow {
  start: Date;
  end: Date;
}

/**
 * Scheduling constraints
 */
export interface SchedulingConstraints {
  precedence?: { jobA: string; jobB: string }[];
  capacityLimits?: { resourceId: string; limit: number }[];
  timeWindows?: { jobId: string; window: TimeWindow }[];
}

/**
 * Scheduling objectives (weights must sum to 1)
 */
export interface SchedulingObjectives {
  minimizeMakespan: number; // 0-1
  minimizeLateness: number; // 0-1
  minimizeSetupTime: number; // 0-1
  maximizeUtilization: number; // 0-1
}

/**
 * Scheduling result
 */
export interface SchedulingResult {
  schedule: ScheduledJob[];
  metrics: SchedulingMetrics;
}

/**
 * Scheduled job
 */
export interface ScheduledJob {
  jobId: string;
  resourceId: string;
  startTime: Date;
  endTime: Date;
  setupStart: Date;
  processingStart: Date;
}

/**
 * Scheduling metrics
 */
export interface SchedulingMetrics {
  makespan: number; // seconds
  averageLateness: number; // seconds
  totalSetupTime: number; // seconds
  resourceUtilization: { [resourceId: string]: number }; // %
}

// ============================================================================
// OEE Types
// ============================================================================

/**
 * OEE calculation parameters
 */
export interface OEEParameters {
  plannedProductionTime: number; // seconds
  downtime: number; // seconds
  targetOutput: number; // units
  actualOutput: number;
  goodUnits: number;
}

/**
 * OEE metrics
 */
export interface OEEMetrics {
  overall: number; // %
  availability: number; // %
  performance: number; // %
  quality: number; // %
}

/**
 * Six big losses
 */
export interface SixBigLosses {
  equipmentFailure: LossDetail;
  setupAdjustment: LossDetail;
  minorStops: LossDetail;
  reducedSpeed: LossDetail;
  processDefects: LossDetail;
  reducedYield: LossDetail;
}

/**
 * Loss detail
 */
export interface LossDetail {
  incidents: number;
  totalTime: number; // seconds
  impactOnOEE: number; // %
}

// ============================================================================
// Safety Types
// ============================================================================

/**
 * Safety zone
 */
export interface SafetyZone {
  zoneId: string;
  type: 'restricted' | 'limited' | 'collaborative';
  boundaries: Vector3D[];
  devices: string[];
  action: 'immediate-stop' | 'controlled-stop' | 'speed-reduction';
}

/**
 * Safety device
 */
export interface SafetyDevice {
  deviceId: string;
  type: 'e-stop' | 'light-curtain' | 'safety-mat' | 'safety-gate';
  status: 'active' | 'inactive' | 'triggered' | 'error';
  category?: 0 | 1 | 2;
  performanceLevel?: 'PLa' | 'PLb' | 'PLc' | 'PLd' | 'PLe';
}

/**
 * Emergency stop request
 */
export interface EmergencyStopRequest {
  lineId: LineId;
  reason: string;
  requestedBy: string;
  timestamp: Date;
}

// ============================================================================
// Equipment Types
// ============================================================================

/**
 * Equipment definition
 */
export interface Equipment {
  equipmentId: EquipmentId;
  name: string;
  type: string;
  manufacturer: string;
  model: string;
  serialNumber: string;
  status: EquipmentStatus;
  maintenance?: MaintenanceInfo;
}

/**
 * Equipment status
 */
export interface EquipmentStatus {
  operational: 'running' | 'idle' | 'maintenance' | 'error' | 'offline';
  alarms: Alarm[];
  runtime: number; // seconds
  cycleCount: number;
}

/**
 * Alarm
 */
export interface Alarm {
  alarmId: string;
  severity: 'critical' | 'warning' | 'info';
  message: string;
  timestamp: Date;
  acknowledged: boolean;
}

/**
 * Maintenance information
 */
export interface MaintenanceInfo {
  lastPM: Date; // preventive maintenance
  nextPM: Date;
  pmInterval: number; // hours
  mtbf: number; // Mean Time Between Failures (hours)
  mttr: number; // Mean Time To Repair (hours)
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * Production event
 */
export interface ProductionEvent {
  eventId: string;
  eventType: 'start' | 'stop' | 'alarm' | 'quality-issue' | 'maintenance';
  lineId: LineId;
  timestamp: Date;
  data: any;
}

/**
 * Event listener
 */
export type EventListener = (event: ProductionEvent) => void;

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
  };
  timestamp: Date;
}

/**
 * Pagination parameters
 */
export interface PaginationParams {
  page: number;
  pageSize: number;
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

// ============================================================================
// SDK Configuration Types
// ============================================================================

/**
 * Manufacturing SDK configuration
 */
export interface ManufacturingSDKConfig {
  facilityId: FacilityId;
  plcType?: PLCType;
  plcHost?: string;
  plcPort?: number;
  scadaEndpoint?: string;
  opcuaEndpoint?: string;
  mqttBroker?: string;
  apiEndpoint?: string;
  apiKey?: string;
}

/**
 * Connection options
 */
export interface ConnectionOptions {
  timeout?: number; // ms
  retryAttempts?: number;
  retryDelay?: number; // ms
  keepAlive?: boolean;
}

// **弘益人間 (홍익인간) · Benefit All Humanity**
