/**
 * WIA Bionic Eye - Core Type Definitions
 */

// ============================================================================
// Image and Frame Types
// ============================================================================

export interface CapturedFrame {
  frameId: string;
  timestamp: number;
  sequenceNumber: number;
  deviceId?: string;
  image: ImageData;
  camera: CameraParams;
  sensors?: SensorData;
}

export interface ImageData {
  width: number;
  height: number;
  format: ImageFormat;
  data: Uint8Array;
  encoding: 'raw' | 'jpeg' | 'h264' | 'h265';
  compression?: number;
}

export enum ImageFormat {
  GRAY8 = 'gray8',
  GRAY16 = 'gray16',
  RGB24 = 'rgb24',
  RGBD32 = 'rgbd32',
  DEPTH16 = 'depth16',
  IR8 = 'ir8',
}

export interface CameraParams {
  fov: { horizontal: number; vertical: number };
  exposure: number;
  gain: number;
  whiteBalance: number;
  focusDistance?: number;
}

export interface SensorData {
  ambientLight: number;
  proximity: number;
  imu: IMUData;
  timestamp: number;
}

export interface IMUData {
  accelerometer: [number, number, number];
  gyroscope: [number, number, number];
  magnetometer?: [number, number, number];
  orientation: {
    quaternion?: [number, number, number, number];
    euler: { roll: number; pitch: number; yaw: number };
  };
}

// ============================================================================
// Processed Visual Types
// ============================================================================

export interface ProcessedVisual {
  frameId: string;
  processingTime: number;
  edges?: EdgeDetection;
  objects: DetectedObject[];
  faces: DetectedFace[];
  depth?: DepthData;
  text?: TextRegion[];
  motion?: MotionData;
}

export interface EdgeDetection {
  data: Uint8Array;
  algorithm: 'canny' | 'sobel' | 'laplacian';
  threshold: number;
}

export interface DetectedObject {
  objectId: string;
  label: string;
  labelKorean: string;
  confidence: number;
  boundingBox: BoundingBox;
  center: { x: number; y: number };
  distance: number;
  priority: ObjectPriority;
  threat: boolean;
  moving: boolean;
  velocity?: { x: number; y: number };
}

export interface BoundingBox {
  x: number;
  y: number;
  width: number;
  height: number;
}

export enum ObjectPriority {
  LOW = 1,
  MEDIUM = 2,
  HIGH = 3,
  CRITICAL = 4,
}

export interface DetectedFace {
  faceId: string;
  boundingBox: BoundingBox;
  landmarks: FaceLandmarks;
  identity?: { name: string; confidence: number };
  emotion?: { type: string; confidence: number };
  distance: number;
}

export interface FaceLandmarks {
  leftEye: [number, number];
  rightEye: [number, number];
  nose: [number, number];
  leftMouth: [number, number];
  rightMouth: [number, number];
}

export interface DepthData {
  map: Float32Array;
  width: number;
  height: number;
  minDistance: number;
  maxDistance: number;
}

export interface TextRegion {
  text: string;
  boundingBox: BoundingBox;
  confidence: number;
  language: string;
}

export interface MotionData {
  movingRegions: { boundingBox: BoundingBox; velocity: { x: number; y: number } }[];
  sceneChange: boolean;
}

// ============================================================================
// Electrode Array Types
// ============================================================================

export interface ElectrodeArray {
  arrayId: string;
  type: ImplantType;
  manufacturer: string;
  model: string;
  serialNumber: string;
  physical: PhysicalSpec;
  electrodes: ElectrodeInfo[];
  placement: PlacementInfo;
  safetyLimits: SafetyLimits;
}

export enum ImplantType {
  EPIRETINAL = 'epiretinal',
  SUBRETINAL = 'subretinal',
  SUPRACHOROIDAL = 'suprachoroidal',
  OPTIC_NERVE = 'optic_nerve',
  CORTICAL = 'cortical',
}

export interface PhysicalSpec {
  totalElectrodes: number;
  activeElectrodes: number;
  rows: number;
  columns: number;
  electrodeDiameter: number;
  electrodeSpacing: number;
  material: ElectrodeMaterial;
}

export enum ElectrodeMaterial {
  PLATINUM = 'platinum',
  PLATINUM_IRIDIUM = 'platinum_iridium',
  IRIDIUM_OXIDE = 'iridium_oxide',
  TITANIUM_NITRIDE = 'titanium_nitride',
  PEDOT = 'pedot',
}

export interface ElectrodeInfo {
  index: number;
  gridPosition: { row: number; col: number };
  functional: boolean;
  enabled: boolean;
  impedance: number;
  thresholdCurrent: number;
  maxSafeCurrent: number;
}

export interface PlacementInfo {
  eye: 'left' | 'right';
  implantDate: Date;
  surgeon: string;
}

export interface SafetyLimits {
  maxChargeDensity: number;
  maxChargePerPhase: number;
  maxCurrentPerElectrode: number;
  maxTotalCurrent: number;
  maxFrequency: number;
  maxDutyCycle: number;
}

// ============================================================================
// Stimulation Types
// ============================================================================

export interface StimulationParams {
  waveform: WaveformType;
  amplitude: number;
  pulseWidth: number;
  frequency: number;
  interphaseGap: number;
  pulsesPerBurst: number;
}

export enum WaveformType {
  BIPHASIC_SYMMETRIC = 'biphasic_symmetric',
  BIPHASIC_ASYMMETRIC = 'biphasic_asymmetric',
  TRIPHASIC = 'triphasic',
}

export interface StimulationMap {
  frameId: string;
  timestamp: number;
  electrodeGrid: {
    rows: number;
    columns: number;
    activeElectrodes: number[];
  };
  intensityMap: Uint8Array;
  strategy: MappingStrategy;
}

export enum MappingStrategy {
  DIRECT = 'direct',
  SCOREBOARD = 'scoreboard',
  AXON_MAP = 'axon_map',
  PHOSPHENE = 'phosphene',
  SALIENCY = 'saliency',
}

// ============================================================================
// Phosphene Types
// ============================================================================

export interface PhospheneMap {
  patientId: string;
  arrayId: string;
  mappingDate: Date;
  phosphenes: PhospheneDefinition[];
  visualField: VisualFieldCoverage;
}

export interface PhospheneDefinition {
  electrodeIndex: number;
  functional: boolean;
  perception: {
    location: { x: number; y: number; uncertainty: number };
    size: number;
    shape: 'round' | 'oval' | 'elongated' | 'irregular';
  };
  response: {
    thresholdCurrent: number;
    maxCurrent: number;
    brightnessLevels: number;
  };
}

export interface VisualFieldCoverage {
  horizontalExtent: number;
  verticalExtent: number;
  centerOffset: [number, number];
}

// ============================================================================
// Safety Types
// ============================================================================

export interface EmergencyStopConfig {
  autoTriggers: AutoTriggers;
  manualTriggers: ManualTriggers;
}

export interface AutoTriggers {
  overCurrent: { enabled: boolean; threshold: number; responseTime: number };
  overVoltage: { enabled: boolean; threshold: number; responseTime: number };
  impedanceAnomaly: { enabled: boolean; changeThreshold: number };
  thermalOverload: { enabled: boolean; maxTemperatureRise: number };
}

export interface ManualTriggers {
  patientButton: { enabled: boolean; location: string };
  voiceCommand: { enabled: boolean; keywords: string[] };
  appControl: { enabled: boolean };
}

export enum EStopState {
  NORMAL = 'normal',
  TRIGGERED = 'triggered',
  RECOVERING = 'recovering',
}
