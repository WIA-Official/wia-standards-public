/**
 * WIA-SPACE: Space Technology Standard - TypeScript Type Definitions
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Core Types
// ============================================================================

export enum OrbitType {
  LEO = 'LEO',                    // Low Earth Orbit
  MEO = 'MEO',                    // Medium Earth Orbit
  GEO = 'GEO',                    // Geostationary Orbit
  HEO = 'HEO',                    // Highly Elliptical Orbit
  SSO = 'SSO',                    // Sun-Synchronous Orbit
  POLAR = 'POLAR',                // Polar Orbit
  TRANSFER = 'TRANSFER',          // Transfer Orbit
  INTERPLANETARY = 'INTERPLANETARY' // Interplanetary Trajectory
}

export enum SpacecraftType {
  SATELLITE = 'SATELLITE',
  SPACE_STATION = 'SPACE_STATION',
  PROBE = 'PROBE',
  LANDER = 'LANDER',
  ROVER = 'ROVER',
  CAPSULE = 'CAPSULE',
  TELESCOPE = 'TELESCOPE',
  COMMUNICATION = 'COMMUNICATION',
  NAVIGATION = 'NAVIGATION',
  EARTH_OBSERVATION = 'EARTH_OBSERVATION'
}

export enum MissionStatus {
  PLANNING = 'PLANNING',
  DEVELOPMENT = 'DEVELOPMENT',
  TESTING = 'TESTING',
  LAUNCH_READY = 'LAUNCH_READY',
  IN_TRANSIT = 'IN_TRANSIT',
  OPERATIONAL = 'OPERATIONAL',
  DEORBITING = 'DEORBITING',
  COMPLETED = 'COMPLETED',
  FAILED = 'FAILED'
}

export enum LaunchVehicleClass {
  SMALL = 'SMALL',              // <2,000 kg to LEO
  MEDIUM = 'MEDIUM',            // 2,000-20,000 kg to LEO
  HEAVY = 'HEAVY',              // 20,000-50,000 kg to LEO
  SUPER_HEAVY = 'SUPER_HEAVY'   // >50,000 kg to LEO
}

// ============================================================================
// Orbital Mechanics Types
// ============================================================================

export interface OrbitalElements {
  semiMajorAxis: number;        // km
  eccentricity: number;         // 0-1
  inclination: number;          // degrees
  longitudeOfAscendingNode: number; // degrees
  argumentOfPeriapsis: number;  // degrees
  trueAnomaly: number;          // degrees
  epoch: Date;                  // reference time
}

export interface StateVector {
  position: Vector3D;           // km
  velocity: Vector3D;           // km/s
  epoch: Date;
  referenceFrame: 'ECI' | 'ECEF' | 'J2000';
}

export interface Vector3D {
  x: number;
  y: number;
  z: number;
}

export interface OrbitPropagation {
  stateVectors: StateVector[];
  duration: number;             // seconds
  stepSize: number;             // seconds
  perturbations: Perturbation[];
}

export interface Perturbation {
  type: 'J2' | 'DRAG' | 'SRP' | 'THIRD_BODY' | 'THRUST';
  enabled: boolean;
  parameters?: Record<string, number>;
}

export interface ManeuverPlan {
  id: string;
  type: 'ORBITAL_INSERTION' | 'PLANE_CHANGE' | 'CIRCULARIZATION' | 'DEORBIT' | 'STATION_KEEPING';
  deltaV: Vector3D;             // km/s
  executionTime: Date;
  duration: number;             // seconds
  fuelMass: number;             // kg
  status: 'PLANNED' | 'EXECUTING' | 'COMPLETED' | 'ABORTED';
}

// ============================================================================
// Spacecraft Types
// ============================================================================

export interface Spacecraft {
  id: string;
  name: string;
  type: SpacecraftType;
  orbitType: OrbitType;
  mass: number;                 // kg
  dimensions: Dimensions;
  power: PowerSystem;
  propulsion: PropulsionSystem;
  communication: CommunicationSystem;
  thermalControl: ThermalControlSystem;
  attitude: AttitudeControlSystem;
  payload: Payload[];
  launchDate?: Date;
  missionStatus: MissionStatus;
  orbitalElements?: OrbitalElements;
  stateVector?: StateVector;
  metadata?: Record<string, any>;
}

export interface Dimensions {
  length: number;               // meters
  width: number;                // meters
  height: number;               // meters
  volume: number;               // cubic meters
}

export interface PowerSystem {
  type: 'SOLAR' | 'NUCLEAR' | 'BATTERY' | 'FUEL_CELL';
  generationCapacity: number;   // watts
  storageCapacity: number;      // watt-hours
  currentPower: number;         // watts
  efficiency: number;           // 0-1
  batteryCharge: number;        // 0-1
}

export interface PropulsionSystem {
  type: 'CHEMICAL' | 'ELECTRIC' | 'HYBRID' | 'COLD_GAS' | 'NONE';
  totalImpulse: number;         // N·s
  specificImpulse: number;      // seconds
  thrust: number;               // newtons
  fuelMass: number;             // kg
  fuelRemaining: number;        // kg
}

export interface CommunicationSystem {
  uplink: CommLink[];
  downlink: CommLink[];
  dataRate: number;             // bits/second
  frequency: number;            // MHz
  antennas: Antenna[];
}

export interface CommLink {
  groundStation: string;
  frequency: number;            // MHz
  bandwidth: number;            // MHz
  signalStrength: number;       // dBm
  linkQuality: number;          // 0-1
  lastContact?: Date;
}

export interface Antenna {
  name: string;
  type: 'OMNIDIRECTIONAL' | 'DIRECTIONAL' | 'PHASED_ARRAY';
  gain: number;                 // dBi
  beamwidth: number;            // degrees
  pointing?: Vector3D;
}

export interface ThermalControlSystem {
  currentTemperature: number;   // Celsius
  minTemperature: number;       // Celsius
  maxTemperature: number;       // Celsius
  heaters: Heater[];
  radiators: Radiator[];
}

export interface Heater {
  id: string;
  power: number;                // watts
  enabled: boolean;
  temperature: number;          // Celsius
}

export interface Radiator {
  id: string;
  area: number;                 // square meters
  emissivity: number;           // 0-1
  temperature: number;          // Celsius
}

export interface AttitudeControlSystem {
  mode: 'DETUMBLE' | 'SUN_POINTING' | 'EARTH_POINTING' | 'INERTIAL' | 'TARGET_POINTING';
  orientation: Quaternion;
  angularVelocity: Vector3D;    // rad/s
  actuators: Actuator[];
  sensors: Sensor[];
}

export interface Quaternion {
  w: number;
  x: number;
  y: number;
  z: number;
}

export interface Actuator {
  type: 'REACTION_WHEEL' | 'MAGNETORQUER' | 'THRUSTER';
  id: string;
  status: 'OPERATIONAL' | 'DEGRADED' | 'FAILED';
  momentum?: number;            // N·m·s (for reaction wheels)
  torque?: number;              // N·m
}

export interface Sensor {
  type: 'STAR_TRACKER' | 'SUN_SENSOR' | 'MAGNETOMETER' | 'GYROSCOPE' | 'GPS';
  id: string;
  accuracy: number;
  status: 'OPERATIONAL' | 'DEGRADED' | 'FAILED';
  lastReading?: any;
}

export interface Payload {
  id: string;
  name: string;
  type: string;
  mass: number;                 // kg
  powerConsumption: number;     // watts
  dataRate: number;             // bits/second
  status: 'ACTIVE' | 'STANDBY' | 'OFF' | 'CALIBRATING';
  pointingRequirement?: 'NADIR' | 'ZENITH' | 'CUSTOM';
}

// ============================================================================
// Telemetry & Command Types
// ============================================================================

export interface Telemetry {
  spacecraftId: string;
  timestamp: Date;
  sequenceNumber: number;
  health: HealthStatus;
  subsystems: SubsystemTelemetry;
  science?: ScienceData[];
}

export interface HealthStatus {
  overall: 'NOMINAL' | 'CAUTION' | 'WARNING' | 'CRITICAL';
  flags: HealthFlag[];
}

export interface HealthFlag {
  subsystem: string;
  level: 'INFO' | 'CAUTION' | 'WARNING' | 'CRITICAL';
  message: string;
  timestamp: Date;
}

export interface SubsystemTelemetry {
  power: PowerTelemetry;
  thermal: ThermalTelemetry;
  attitude: AttitudeTelemetry;
  communication: CommTelemetry;
  propulsion?: PropulsionTelemetry;
}

export interface PowerTelemetry {
  voltage: number;              // volts
  current: number;              // amperes
  power: number;                // watts
  batteryCharge: number;        // 0-1
  solarPanelCurrent: number[];  // amperes
}

export interface ThermalTelemetry {
  temperatures: Record<string, number>; // Celsius
  heaterStatus: Record<string, boolean>;
}

export interface AttitudeTelemetry {
  orientation: Quaternion;
  angularVelocity: Vector3D;
  controlMode: string;
  pointingError?: number;       // degrees
}

export interface CommTelemetry {
  signalStrength: number;       // dBm
  dataRate: number;             // bits/second
  packetsSent: number;
  packetsReceived: number;
  errorRate: number;            // 0-1
}

export interface PropulsionTelemetry {
  fuelRemaining: number;        // kg
  tankPressure: number;         // bar
  thrustLevel: number;          // newtons
}

export interface ScienceData {
  instrumentId: string;
  dataType: string;
  timestamp: Date;
  data: any;
  quality: number;              // 0-1
  processingLevel: 0 | 1 | 2 | 3 | 4;
}

export interface Command {
  id: string;
  spacecraftId: string;
  type: CommandType;
  parameters: Record<string, any>;
  priority: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  scheduledTime?: Date;
  executionTime?: Date;
  status: 'QUEUED' | 'UPLINKED' | 'EXECUTING' | 'COMPLETED' | 'FAILED';
  result?: CommandResult;
}

export enum CommandType {
  ATTITUDE_CONTROL = 'ATTITUDE_CONTROL',
  MANEUVER = 'MANEUVER',
  PAYLOAD_OPERATION = 'PAYLOAD_OPERATION',
  POWER_MANAGEMENT = 'POWER_MANAGEMENT',
  COMMUNICATION = 'COMMUNICATION',
  SOFTWARE_UPDATE = 'SOFTWARE_UPDATE',
  SAFING = 'SAFING',
  CALIBRATION = 'CALIBRATION'
}

export interface CommandResult {
  success: boolean;
  executionTime: Date;
  message: string;
  telemetrySnapshot?: Partial<Telemetry>;
}

// ============================================================================
// Mission Planning Types
// ============================================================================

export interface Mission {
  id: string;
  name: string;
  description: string;
  objectives: MissionObjective[];
  spacecraft: Spacecraft[];
  launchWindow: LaunchWindow;
  duration: number;             // days
  budget: number;               // USD
  status: MissionStatus;
  timeline: MissionEvent[];
  groundSegment: GroundStation[];
  risks: RiskAssessment[];
}

export interface MissionObjective {
  id: string;
  description: string;
  priority: 'PRIMARY' | 'SECONDARY' | 'TERTIARY';
  successCriteria: string[];
  status: 'NOT_STARTED' | 'IN_PROGRESS' | 'COMPLETED' | 'FAILED';
}

export interface LaunchWindow {
  openDate: Date;
  closeDate: Date;
  optimalDate: Date;
  constraints: LaunchConstraint[];
}

export interface LaunchConstraint {
  type: 'ORBITAL' | 'WEATHER' | 'RANGE_AVAILABILITY' | 'PLANETARY_ALIGNMENT';
  description: string;
  mandatory: boolean;
}

export interface MissionEvent {
  id: string;
  name: string;
  type: 'LAUNCH' | 'DEPLOYMENT' | 'MANEUVER' | 'SCIENCE_OPERATION' | 'COMMUNICATION' | 'ANOMALY';
  scheduledTime: Date;
  actualTime?: Date;
  duration: number;             // seconds
  status: 'SCHEDULED' | 'IN_PROGRESS' | 'COMPLETED' | 'CANCELLED';
}

export interface GroundStation {
  id: string;
  name: string;
  location: GeographicCoordinate;
  antennaSize: number;          // meters
  frequency: number[];          // MHz
  capabilities: string[];
  contactSchedule: ContactWindow[];
}

export interface GeographicCoordinate {
  latitude: number;             // degrees
  longitude: number;            // degrees
  altitude: number;             // meters
}

export interface ContactWindow {
  startTime: Date;
  endTime: Date;
  elevation: number;            // degrees
  azimuth: number;              // degrees
  quality: 'EXCELLENT' | 'GOOD' | 'FAIR' | 'POOR';
}

export interface RiskAssessment {
  id: string;
  category: 'TECHNICAL' | 'PROGRAMMATIC' | 'SAFETY' | 'ENVIRONMENTAL';
  description: string;
  probability: number;          // 0-1
  impact: number;               // 0-1
  mitigation: string[];
  owner: string;
}

// ============================================================================
// Launch Vehicle Types
// ============================================================================

export interface LaunchVehicle {
  id: string;
  name: string;
  manufacturer: string;
  class: LaunchVehicleClass;
  stages: LaunchStage[];
  payload: PayloadCapacity[];
  reliability: number;          // 0-1
  launchSites: string[];
  cost: number;                 // USD
  status: 'ACTIVE' | 'RETIRED' | 'DEVELOPMENT';
}

export interface LaunchStage {
  stageNumber: number;
  propellantType: string;
  propellantMass: number;       // kg
  dryMass: number;              // kg
  thrust: number;               // newtons
  specificImpulse: number;      // seconds
  burnTime: number;             // seconds
  reusable: boolean;
}

export interface PayloadCapacity {
  orbit: OrbitType;
  capacity: number;             // kg
}

// ============================================================================
// Space Debris Tracking Types
// ============================================================================

export interface SpaceDebris {
  id: string;
  catalogNumber: string;
  name?: string;
  type: 'PAYLOAD' | 'ROCKET_BODY' | 'DEBRIS' | 'UNKNOWN';
  size: 'SMALL' | 'MEDIUM' | 'LARGE';
  mass?: number;                // kg
  orbitalElements: OrbitalElements;
  trackingAccuracy: number;     // km
  collisionProbability?: number; // 0-1
  source?: string;
  launchDate?: Date;
}

export interface ConjunctionEvent {
  id: string;
  primaryObject: string;
  secondaryObject: string;
  timeOfClosestApproach: Date;
  missDistance: number;         // km
  collisionProbability: number; // 0-1
  relativeVelocity: number;     // km/s
  status: 'MONITORING' | 'ACTIONABLE' | 'RESOLVED';
  avoidanceManeuver?: ManeuverPlan;
}

// ============================================================================
// Space Station Operations Types
// ============================================================================

export interface SpaceStation {
  id: string;
  name: string;
  orbitalElements: OrbitalElements;
  modules: StationModule[];
  crew: CrewMember[];
  dockingPorts: DockingPort[];
  lifeSupportSystem: LifeSupportSystem;
  experiments: Experiment[];
}

export interface StationModule {
  id: string;
  name: string;
  type: 'HABITATION' | 'LABORATORY' | 'LOGISTICS' | 'AIRLOCK' | 'DOCKING';
  volume: number;               // cubic meters
  mass: number;                 // kg
  pressurized: boolean;
  power: number;                // watts
}

export interface CrewMember {
  id: string;
  name: string;
  role: 'COMMANDER' | 'PILOT' | 'SCIENTIST' | 'ENGINEER' | 'SPECIALIST';
  arrivalDate: Date;
  plannedDeparture: Date;
  extravehicularActivity: number; // hours
  health: 'NOMINAL' | 'MONITORING' | 'CONCERN';
}

export interface DockingPort {
  id: string;
  type: 'ACTIVE' | 'PASSIVE';
  status: 'AVAILABLE' | 'OCCUPIED' | 'RESERVED' | 'MAINTENANCE';
  dockedVehicle?: string;
  dockingTime?: Date;
}

export interface LifeSupportSystem {
  oxygenLevel: number;          // percentage
  co2Level: number;             // ppm
  pressure: number;             // kPa
  temperature: number;          // Celsius
  humidity: number;             // percentage
  waterSupply: number;          // liters
  foodSupply: number;           // days
}

export interface Experiment {
  id: string;
  name: string;
  principalInvestigator: string;
  field: string;
  startDate: Date;
  duration: number;             // days
  status: 'PLANNED' | 'ACTIVE' | 'PAUSED' | 'COMPLETED';
  dataGenerated: number;        // MB
}

// ============================================================================
// Utility Types
// ============================================================================

export interface ValidationResult {
  valid: boolean;
  errors: ValidationError[];
  warnings: ValidationWarning[];
}

export interface ValidationError {
  field: string;
  message: string;
  code: string;
}

export interface ValidationWarning {
  field: string;
  message: string;
  recommendation: string;
}

/**
 * 弘益人間 (Benefit All Humanity)
 *
 * All types and interfaces in this module support the development of
 * space technology systems that advance humanity's exploration and
 * understanding of the cosmos.
 */
