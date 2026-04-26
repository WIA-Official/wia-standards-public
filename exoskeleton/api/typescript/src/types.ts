/**
 * WIA Exoskeleton Standard
 * TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Basic Types
// ============================================================================

export type Timestamp = string;
export type UUID = string;

// ============================================================================
// Exoskeleton Device Types
// ============================================================================

export enum ExoskeletonType {
  FULL_BODY = 'full_body',
  UPPER_BODY = 'upper_body',
  LOWER_BODY = 'lower_body',
  ARM = 'arm',
  LEG = 'leg',
  HAND = 'hand',
  SPINE = 'spine',
  HYBRID = 'hybrid',
}

export enum ExoskeletonPurpose {
  REHABILITATION = 'rehabilitation',
  MOBILITY_ASSIST = 'mobility_assist',
  INDUSTRIAL = 'industrial',
  MILITARY = 'military',
  MEDICAL = 'medical',
  SPORTS = 'sports',
  RESEARCH = 'research',
}

export interface ExoskeletonDevice {
  deviceId: string;
  type: ExoskeletonType;
  purpose: ExoskeletonPurpose;
  manufacturer: string;
  model: string;
  serialNumber: string;
  firmwareVersion: string;
  status: DeviceStatus;
  capabilities: DeviceCapabilities;
  joints: JointInfo[];
  sensors: SensorInfo[];
  actuators: ActuatorInfo[];
  battery: BatteryInfo;
  safety: SafetyConfig;
}

export enum DeviceStatus {
  OFFLINE = 'offline',
  INITIALIZING = 'initializing',
  READY = 'ready',
  ACTIVE = 'active',
  PAUSED = 'paused',
  ERROR = 'error',
  MAINTENANCE = 'maintenance',
  EMERGENCY_STOP = 'emergency_stop',
}

export interface DeviceCapabilities {
  maxPayload: number;
  maxSpeed: number;
  maxTorque: number;
  degreesOfFreedom: number;
  operatingModes: OperatingMode[];
  supportedGaits: GaitType[];
  waterResistance: WaterResistanceLevel;
  operatingTemperature: TemperatureRange;
  certifications: string[];
}

export interface TemperatureRange {
  min: number;
  max: number;
  unit: 'celsius' | 'fahrenheit';
}

export enum WaterResistanceLevel {
  NONE = 'none',
  SPLASH_PROOF = 'splash_proof',
  WATER_RESISTANT = 'water_resistant',
  WATERPROOF = 'waterproof',
}

export enum OperatingMode {
  PASSIVE = 'passive',
  ACTIVE_ASSIST = 'active_assist',
  ACTIVE_RESIST = 'active_resist',
  TRANSPARENT = 'transparent',
  THERAPY = 'therapy',
  TRAINING = 'training',
  MANUAL = 'manual',
}

export enum GaitType {
  WALKING = 'walking',
  RUNNING = 'running',
  STAIR_CLIMBING = 'stair_climbing',
  SITTING = 'sitting',
  STANDING = 'standing',
  SQUATTING = 'squatting',
  CUSTOM = 'custom',
}

// ============================================================================
// Joint Types
// ============================================================================

export interface JointInfo {
  jointId: string;
  name: string;
  type: JointType;
  location: JointLocation;
  rangeOfMotion: RangeOfMotion;
  currentPosition: number;
  currentVelocity: number;
  currentTorque: number;
  status: JointStatus;
  limits: JointLimits;
}

export enum JointType {
  REVOLUTE = 'revolute',
  PRISMATIC = 'prismatic',
  SPHERICAL = 'spherical',
  FIXED = 'fixed',
}

export interface JointLocation {
  bodyPart: BodyPart;
  side: Side;
  segment: string;
}

export enum BodyPart {
  SHOULDER = 'shoulder',
  ELBOW = 'elbow',
  WRIST = 'wrist',
  HAND = 'hand',
  FINGER = 'finger',
  HIP = 'hip',
  KNEE = 'knee',
  ANKLE = 'ankle',
  FOOT = 'foot',
  SPINE = 'spine',
  NECK = 'neck',
}

export enum Side {
  LEFT = 'left',
  RIGHT = 'right',
  CENTER = 'center',
  BILATERAL = 'bilateral',
}

export interface RangeOfMotion {
  min: number;
  max: number;
  unit: AngleUnit;
  flexion?: number;
  extension?: number;
  abduction?: number;
  adduction?: number;
  rotation?: number;
}

export enum AngleUnit {
  DEGREES = 'degrees',
  RADIANS = 'radians',
}

export enum JointStatus {
  NORMAL = 'normal',
  LIMITED = 'limited',
  LOCKED = 'locked',
  OVERLOAD = 'overload',
  CALIBRATING = 'calibrating',
  ERROR = 'error',
}

export interface JointLimits {
  positionMin: number;
  positionMax: number;
  velocityMax: number;
  torqueMax: number;
  accelerationMax: number;
  softStopMargin: number;
}

// ============================================================================
// Sensor Types
// ============================================================================

export interface SensorInfo {
  sensorId: string;
  name: string;
  type: SensorType;
  location: SensorLocation;
  samplingRate: number;
  resolution: number;
  range: SensorRange;
  status: SensorStatus;
  calibration: CalibrationInfo;
}

export enum SensorType {
  ENCODER = 'encoder',
  IMU = 'imu',
  FORCE_TORQUE = 'force_torque',
  PRESSURE = 'pressure',
  EMG = 'emg',
  FSR = 'fsr',
  TEMPERATURE = 'temperature',
  PROXIMITY = 'proximity',
  STRAIN_GAUGE = 'strain_gauge',
  ACCELEROMETER = 'accelerometer',
  GYROSCOPE = 'gyroscope',
}

export interface SensorLocation {
  jointId?: string;
  segmentId?: string;
  position: Position3D;
  orientation: Orientation3D;
}

export interface Position3D {
  x: number;
  y: number;
  z: number;
}

export interface Orientation3D {
  roll: number;
  pitch: number;
  yaw: number;
}

export interface SensorRange {
  min: number;
  max: number;
  unit: string;
}

export enum SensorStatus {
  ACTIVE = 'active',
  INACTIVE = 'inactive',
  CALIBRATING = 'calibrating',
  ERROR = 'error',
  DISCONNECTED = 'disconnected',
}

export interface CalibrationInfo {
  lastCalibrated: Timestamp;
  nextCalibration: Timestamp;
  calibrationQuality: number;
  parameters: Record<string, number>;
}

// ============================================================================
// Actuator Types
// ============================================================================

export interface ActuatorInfo {
  actuatorId: string;
  name: string;
  type: ActuatorType;
  jointId: string;
  specifications: ActuatorSpecs;
  status: ActuatorStatus;
  currentOutput: ActuatorOutput;
}

export enum ActuatorType {
  ELECTRIC_MOTOR = 'electric_motor',
  HYDRAULIC = 'hydraulic',
  PNEUMATIC = 'pneumatic',
  SERIES_ELASTIC = 'series_elastic',
  CABLE_DRIVEN = 'cable_driven',
  ARTIFICIAL_MUSCLE = 'artificial_muscle',
}

export interface ActuatorSpecs {
  maxTorque: number;
  maxSpeed: number;
  maxPower: number;
  efficiency: number;
  gearRatio?: number;
  backdrivable: boolean;
}

export enum ActuatorStatus {
  READY = 'ready',
  ACTIVE = 'active',
  OVERHEATING = 'overheating',
  OVERLOADED = 'overloaded',
  FAULT = 'fault',
  DISABLED = 'disabled',
}

export interface ActuatorOutput {
  torque: number;
  velocity: number;
  position: number;
  power: number;
  temperature: number;
}

// ============================================================================
// Battery Types
// ============================================================================

export interface BatteryInfo {
  batteryId: string;
  type: BatteryType;
  capacity: number;
  currentLevel: number;
  voltage: number;
  current: number;
  temperature: number;
  status: BatteryStatus;
  health: number;
  cycleCount: number;
  estimatedRuntime: number;
}

export enum BatteryType {
  LITHIUM_ION = 'lithium_ion',
  LITHIUM_POLYMER = 'lithium_polymer',
  SOLID_STATE = 'solid_state',
  FUEL_CELL = 'fuel_cell',
}

export enum BatteryStatus {
  CHARGING = 'charging',
  DISCHARGING = 'discharging',
  FULL = 'full',
  LOW = 'low',
  CRITICAL = 'critical',
  ERROR = 'error',
}

// ============================================================================
// Safety Types
// ============================================================================

export interface SafetyConfig {
  emergencyStopEnabled: boolean;
  collisionDetection: boolean;
  forceLimit: number;
  speedLimit: number;
  safetyZones: SafetyZone[];
  alerts: SafetyAlert[];
}

export interface SafetyZone {
  zoneId: string;
  name: string;
  type: ZoneType;
  boundaries: Boundary[];
  restrictions: ZoneRestrictions;
}

export enum ZoneType {
  RESTRICTED = 'restricted',
  WARNING = 'warning',
  NORMAL = 'normal',
  SAFE = 'safe',
}

export interface Boundary {
  type: BoundaryType;
  parameters: Record<string, number>;
}

export enum BoundaryType {
  SPHERE = 'sphere',
  CYLINDER = 'cylinder',
  BOX = 'box',
  PLANE = 'plane',
  CUSTOM = 'custom',
}

export interface ZoneRestrictions {
  maxSpeed: number;
  maxForce: number;
  allowedModes: OperatingMode[];
}

export interface SafetyAlert {
  alertId: string;
  type: AlertType;
  severity: AlertSeverity;
  condition: AlertCondition;
  actions: AlertAction[];
}

export enum AlertType {
  COLLISION = 'collision',
  OVERLOAD = 'overload',
  OVERHEAT = 'overheat',
  LOW_BATTERY = 'low_battery',
  COMMUNICATION_LOSS = 'communication_loss',
  SENSOR_FAILURE = 'sensor_failure',
  LIMIT_EXCEEDED = 'limit_exceeded',
}

export enum AlertSeverity {
  INFO = 'info',
  WARNING = 'warning',
  CRITICAL = 'critical',
  EMERGENCY = 'emergency',
}

export interface AlertCondition {
  parameter: string;
  operator: ComparisonOperator;
  threshold: number;
  duration?: number;
}

export enum ComparisonOperator {
  GREATER = 'gt',
  LESS = 'lt',
  EQUAL = 'eq',
  GREATER_EQUAL = 'gte',
  LESS_EQUAL = 'lte',
}

export interface AlertAction {
  type: ActionType;
  parameters: Record<string, unknown>;
}

export enum ActionType {
  NOTIFY = 'notify',
  SLOW_DOWN = 'slow_down',
  STOP = 'stop',
  EMERGENCY_STOP = 'emergency_stop',
  LOG = 'log',
  SEND_ALERT = 'send_alert',
}

// ============================================================================
// User Profile Types
// ============================================================================

export interface UserProfile {
  userId: string;
  name: string;
  physicalProfile: PhysicalProfile;
  medicalConditions: MedicalCondition[];
  therapyGoals: TherapyGoal[];
  preferences: UserPreferences;
  fittingData: FittingData;
  sessionHistory: string[];
}

export interface PhysicalProfile {
  height: number;
  weight: number;
  legLength: number;
  armLength: number;
  shoulderWidth: number;
  hipWidth: number;
  measurements: Record<string, number>;
}

export interface MedicalCondition {
  condition: string;
  severity: string;
  affectedAreas: BodyPart[];
  precautions: string[];
  contraindications: string[];
}

export interface TherapyGoal {
  goalId: string;
  description: string;
  targetMetric: string;
  currentValue: number;
  targetValue: number;
  deadline?: Timestamp;
  status: GoalStatus;
}

export enum GoalStatus {
  NOT_STARTED = 'not_started',
  IN_PROGRESS = 'in_progress',
  ACHIEVED = 'achieved',
  PAUSED = 'paused',
}

export interface UserPreferences {
  assistanceLevel: number;
  speedPreference: number;
  feedbackType: FeedbackType[];
  language: string;
  audioEnabled: boolean;
  hapticEnabled: boolean;
}

export enum FeedbackType {
  VISUAL = 'visual',
  AUDIO = 'audio',
  HAPTIC = 'haptic',
  VERBAL = 'verbal',
}

export interface FittingData {
  fittingId: string;
  date: Timestamp;
  adjustments: Adjustment[];
  comfort: number;
  notes: string;
}

export interface Adjustment {
  component: string;
  parameter: string;
  value: number;
  unit: string;
}

// ============================================================================
// Session Types
// ============================================================================

export interface ExoskeletonSession {
  sessionId: string;
  userId: string;
  deviceId: string;
  startTime: Timestamp;
  endTime?: Timestamp;
  mode: OperatingMode;
  activities: ActivityLog[];
  metrics: SessionMetrics;
  events: SessionEvent[];
}

export interface ActivityLog {
  activityId: string;
  type: ActivityType;
  startTime: Timestamp;
  endTime: Timestamp;
  parameters: Record<string, unknown>;
  performance: PerformanceData;
}

export enum ActivityType {
  WALKING = 'walking',
  STANDING = 'standing',
  SITTING = 'sitting',
  EXERCISE = 'exercise',
  THERAPY = 'therapy',
  TRAINING = 'training',
}

export interface PerformanceData {
  stepsCount?: number;
  distance?: number;
  duration: number;
  averageSpeed?: number;
  energyConsumption: number;
  assistanceProvided: number;
}

export interface SessionMetrics {
  totalDuration: number;
  activeTime: number;
  stepsTotal: number;
  distanceTotal: number;
  caloriesBurned: number;
  averageAssistance: number;
  peakForce: number;
  batteryUsed: number;
}

export interface SessionEvent {
  timestamp: Timestamp;
  type: EventType;
  data: Record<string, unknown>;
}

export enum EventType {
  MODE_CHANGE = 'mode_change',
  ALERT = 'alert',
  USER_INPUT = 'user_input',
  MILESTONE = 'milestone',
  ERROR = 'error',
}

// ============================================================================
// API Types
// ============================================================================

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: ApiError;
  timestamp: Timestamp;
  requestId: string;
}

export interface ApiError {
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

export type EventHandler<T> = (event: T) => void | Promise<void>;
