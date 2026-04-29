/**
 * WIA-OCEAN-006: Submarine Technology Standard
 * TypeScript Type Definitions
 *
 * Philosophy: 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ==================== Core Types ====================

export interface Position {
  lat: number;
  lon: number;
  depth?: number;
  altitude?: number;
}

export interface GPSCoordinate {
  latitude: number;
  longitude: number;
  accuracy?: number;
  timestamp: Date;
}

export interface VehicleConfig {
  vehicleId: string;
  maxDepth: number;
  sensors: SensorType[];
  capabilities?: VehicleCapabilities;
}

export interface VehicleCapabilities {
  imaging: boolean;
  sampling: boolean;
  sonarMapping: boolean;
  autonomousNavigation: boolean;
  manipulatorArm: boolean;
}

export type SensorType =
  | 'ctd'
  | 'camera'
  | 'sonar'
  | 'sampler'
  | 'multibeam'
  | 'sidescan'
  | 'magnetometer'
  | 'subbottom'
  | 'fluorometer';

// ==================== Mission Types ====================

export interface MissionConfig {
  targetDepth: number;
  duration: number; // hours
  route: Waypoint[];
  objectives?: MissionObjective[];
  emergencyProtocols?: EmergencyProtocol[];
}

export interface Waypoint {
  lat: number;
  lon: number;
  depth: number;
  action?: WaypointAction;
  arrivalRadius?: number; // meters
}

export type WaypointAction =
  | 'survey'
  | 'sample'
  | 'hover'
  | 'image'
  | 'transit';

export interface MissionObjective {
  id: string;
  type: ObjectiveType;
  priority: 'low' | 'medium' | 'high' | 'critical';
  location?: Position;
  parameters?: Record<string, any>;
}

export type ObjectiveType =
  | 'mapping'
  | 'sampling'
  | 'photography'
  | 'video_transect'
  | 'inspection'
  | 'deployment'
  | 'recovery';

export interface EmergencyProtocol {
  trigger: EmergencyType;
  action: 'surface' | 'hold_position' | 'return_home' | 'shutdown';
  priority: number;
}

export type EmergencyType =
  | 'low_battery'
  | 'communication_loss'
  | 'system_fault'
  | 'leak_detected'
  | 'obstacle_detected'
  | 'weather_deterioration';

// ==================== Sensor Types ====================

export interface SensorReading {
  timestamp: Date;
  sensorId: string;
  type: SensorType;
  data: any;
  quality: QualityFlag;
}

export type QualityFlag =
  | 0 // unknown
  | 1 // good
  | 2 // probably good
  | 3 // probably bad
  | 4 // bad
  | 9; // missing

export interface CTDData {
  temperature: number; // Celsius
  salinity: number; // PSU (Practical Salinity Units)
  pressure: number; // dbar (decibars)
  depth: number; // meters
  conductivity: number; // mS/cm
  soundVelocity?: number; // m/s
}

export interface CameraConfig {
  resolution: '720p' | '1080p' | '4K' | '8K';
  frameRate: number; // fps
  duration?: number; // seconds
  lights: 'off' | 'low' | 'medium' | 'high' | 'auto';
  whiteBalance?: 'auto' | 'manual';
  exposure?: 'auto' | 'manual';
}

export interface SonarData {
  type: 'multibeam' | 'sidescan' | 'synthetic_aperture';
  range: number; // meters
  resolution: number; // cm
  swathWidth?: number; // degrees
  frequency: number; // kHz
  pointCloud?: Point3D[];
}

export interface Point3D {
  x: number;
  y: number;
  z: number;
  intensity?: number;
  classification?: string;
}

// ==================== Sample Collection ====================

export interface SampleConfig {
  type: SampleType;
  volume?: number; // ml
  location: Position;
  preservationMethod?: PreservationMethod;
  metadata?: SampleMetadata;
}

export type SampleType =
  | 'water'
  | 'sediment'
  | 'rock'
  | 'biological'
  | 'core'
  | 'push_core';

export type PreservationMethod =
  | 'none'
  | 'frozen'
  | 'cooled'
  | 'formaldehyde'
  | 'ethanol'
  | 'in_situ_pressure';

export interface SampleMetadata {
  containerId: string;
  collectionTime: Date;
  depth: number;
  temperature: number;
  photos?: string[];
  description?: string;
  collector?: string;
}

// ==================== Navigation & Control ====================

export interface NavigationData {
  position: Position;
  heading: number; // degrees true
  pitch: number; // degrees
  roll: number; // degrees
  velocity: Velocity;
  accuracy: PositionAccuracy;
}

export interface Velocity {
  north: number; // m/s
  east: number; // m/s
  down: number; // m/s
  speed: number; // m/s
  course: number; // degrees
}

export interface PositionAccuracy {
  horizontal: number; // meters
  vertical: number; // meters
  heading: number; // degrees
}

export interface ControlCommand {
  thrust: ThrustVector;
  lights?: number; // 0-100%
  sampleTrigger?: boolean;
  cameraTrigger?: boolean;
}

export interface ThrustVector {
  forward: number; // -100 to 100%
  strafe: number; // -100 to 100%
  vertical: number; // -100 to 100%
  yaw: number; // -100 to 100%
}

// ==================== Vehicle Status ====================

export interface VehicleStatus {
  timestamp: Date;
  state: VehicleState;
  depth: number;
  position: Position;
  battery: BatteryStatus;
  systems: SystemHealth;
  warnings: Warning[];
  errors: Error[];
}

export type VehicleState =
  | 'surface'
  | 'descending'
  | 'at_depth'
  | 'ascending'
  | 'emergency'
  | 'docked'
  | 'maintenance';

export interface BatteryStatus {
  percentage: number; // 0-100
  voltage: number; // Volts
  current: number; // Amps
  temperature: number; // Celsius
  remainingTime: number; // hours
  cycleCount: number;
}

export interface SystemHealth {
  propulsion: HealthStatus;
  navigation: HealthStatus;
  communication: HealthStatus;
  sensors: HealthStatus;
  power: HealthStatus;
  cameras: HealthStatus;
}

export type HealthStatus =
  | 'normal'
  | 'degraded'
  | 'warning'
  | 'critical'
  | 'offline';

export interface Warning {
  id: string;
  severity: 'low' | 'medium' | 'high';
  system: string;
  message: string;
  timestamp: Date;
  acknowledged: boolean;
}

// ==================== Communication ====================

export interface CommunicationPacket {
  header: PacketHeader;
  payload: any;
  checksum: string;
}

export interface PacketHeader {
  version: string;
  timestamp: number;
  vehicleId: string;
  packetType: PacketType;
  sequenceNumber: number;
}

export type PacketType =
  | 'telemetry'
  | 'command'
  | 'data'
  | 'heartbeat'
  | 'emergency';

export interface TelemetryPacket {
  depth: number;
  position: [number, number];
  status: VehicleStatus;
  sensorData?: SensorReading[];
}

// ==================== Data Management ====================

export interface DataStream {
  id: string;
  type: DataType;
  format: string;
  sampleRate: number; // Hz
  storage: StorageLocation;
}

export type DataType =
  | 'video'
  | 'audio'
  | 'telemetry'
  | 'navigation'
  | 'sensors'
  | 'sonar';

export type StorageLocation =
  | 'local'
  | 'cloud'
  | 'both';

export interface MissionData {
  missionId: string;
  vehicle: string;
  startTime: Date;
  endTime: Date;
  maxDepth: number;
  distance: number; // meters
  dataStreams: DataStream[];
  samples: SampleMetadata[];
  summary: MissionSummary;
}

export interface MissionSummary {
  objectives: MissionObjective[];
  objectivesCompleted: number;
  totalDistance: number;
  bottomTime: number; // hours
  samplesCollected: number;
  videoRecorded: number; // hours
  dataVolume: number; // GB
  incidents: Incident[];
}

export interface Incident {
  timestamp: Date;
  type: IncidentType;
  severity: 'minor' | 'moderate' | 'major' | 'critical';
  description: string;
  response: string;
  resolved: boolean;
}

export type IncidentType =
  | 'technical_fault'
  | 'navigation_error'
  | 'communication_loss'
  | 'environmental'
  | 'near_miss'
  | 'safety_event';

// ==================== Configuration ====================

export interface SystemConfig {
  vehicle: VehicleConfig;
  sensors: SensorConfig[];
  communication: CommunicationConfig;
  navigation: NavigationConfig;
  safety: SafetyConfig;
}

export interface SensorConfig {
  id: string;
  type: SensorType;
  enabled: boolean;
  sampleRate: number;
  calibration?: CalibrationData;
}

export interface CalibrationData {
  date: Date;
  coefficients: number[];
  accuracy: number;
  nextDue: Date;
}

export interface CommunicationConfig {
  acoustic: {
    frequency: number; // kHz
    dataRate: number; // bps
    range: number; // meters
  };
  optical?: {
    wavelength: number; // nm
    dataRate: number; // Mbps
    range: number; // meters
  };
}

export interface NavigationConfig {
  ins: {
    updateRate: number; // Hz
    accuracy: number; // meters
  };
  dvl: {
    frequency: number; // kHz
    maxAltitude: number; // meters
  };
  usbl: {
    range: number; // meters
    accuracy: number; // % of slant range
  };
}

export interface SafetyConfig {
  maxDepth: number;
  minBattery: number; // percentage
  maxMissionDuration: number; // hours
  emergencyAscentRate: number; // m/min
  ballastWeight: number; // kg
  autoSurfaceEnabled: boolean;
}

// ==================== Events ====================

export interface Event {
  id: string;
  timestamp: Date;
  type: EventType;
  source: string;
  data: any;
}

export type EventType =
  | 'mission_start'
  | 'mission_end'
  | 'dive_start'
  | 'dive_end'
  | 'sample_collected'
  | 'waypoint_reached'
  | 'anomaly_detected'
  | 'system_alert'
  | 'user_action';

// ==================== Export All ====================

export default {
  // Types are exported individually above
};
