/**
 * WIA eVTOL Standard - TypeScript Types
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 * © 2025 SmileStory Inc. / WIA
 */

export type Timestamp = string;

export type EVTOLConfiguration =
  | 'multicopter'
  | 'lift-cruise'
  | 'tilt-wing'
  | 'tilt-rotor'
  | 'vectored-thrust'
  | 'powered-lift';

export type FlightPhase =
  | 'preflight'
  | 'takeoff'
  | 'transition-to-cruise'
  | 'cruise'
  | 'transition-to-hover'
  | 'hover'
  | 'approach'
  | 'landing'
  | 'emergency';

export enum OperationalStatus {
  AVAILABLE = 'available',
  IN_FLIGHT = 'in_flight',
  CHARGING = 'charging',
  MAINTENANCE = 'maintenance',
  GROUNDED = 'grounded',
  RESERVED = 'reserved',
}

export interface BatterySystem {
  batteryId: string;
  voltage: number;
  capacity: number;
  chemistry: BatteryChemistry;
  cells: number;
  stateOfCharge: number;
  stateOfHealth: number;
  temperature: number;
  cycleCount: number;
  health: number;
  chargingStatus: ChargingStatus;
  estimatedRange: number;
}

export type BatteryChemistry = 'Li-ion NMC' | 'Li-ion LFP' | 'Solid-state' | 'Li-polymer';

export enum ChargingStatus {
  NOT_CHARGING = 'not_charging',
  SLOW_CHARGING = 'slow_charging',
  FAST_CHARGING = 'fast_charging',
  COMPLETE = 'complete',
  ERROR = 'error',
}

export interface MotorController {
  id: string;
  position: MotorPosition;
  power: number;
  rpm: number;
  current: number;
  temperature: number;
  efficiency: number;
  status: ComponentStatus;
}

export type MotorPosition = 'front-left' | 'front-right' | 'rear-left' | 'rear-right' | 'pusher';

export enum ComponentStatus {
  NORMAL = 'normal',
  WARNING = 'warning',
  FAULT = 'fault',
  OFFLINE = 'offline',
}

export interface PropulsionSystem {
  type: 'electric' | 'hybrid';
  motors: MotorController[];
  rotorDiameter: number;
  bladeCount: number;
  totalPower: number;
  maxThrust: number;
}

export interface Performance {
  maxSpeed: number;
  cruiseSpeed: number;
  range: number;
  endurance: number;
  serviceCeiling: number;
  rateOfClimb: number;
  maxPayload: number;
  noiseLevel: number;
}

export interface EVTOLAircraft {
  id: string;
  configuration: EVTOLConfiguration;
  manufacturer: string;
  model: string;
  registrationNumber: string;
  mtow: number;
  payload: number;
  passengers: number;
  batterySystem: BatterySystem;
  propulsionSystem: PropulsionSystem;
  performance: Performance;
  currentPhase?: FlightPhase;
  operationalStatus: OperationalStatus;
  certification: Certification;
  avionics: AvionicsSystem;
  metadata?: Record<string, unknown>;
}

export interface Certification {
  standard: string;
  authority: string;
  category: string;
  validUntil: Timestamp;
  approvals: string[];
}

export interface AvionicsSystem {
  flightComputers: number;
  redundancyLevel: 'dual' | 'triple' | 'quad';
  autopilotCapable: boolean;
  navigationSystems: string[];
  communicationSystems: string[];
}

export interface Vertiport {
  vertiportId: string;
  name: string;
  icaoCode?: string;
  location: GeoLocation;
  elevation: number;
  pads: number;
  chargingCapacity: number;
  status: 'open' | 'closed' | 'limited';
}

export interface GeoLocation {
  latitude: number;
  longitude: number;
  altitude?: number;
}

export interface FlightPlan {
  flightId: string;
  aircraftId: string;
  departure: Vertiport;
  destination: Vertiport;
  scheduledDeparture: Timestamp;
  scheduledArrival: Timestamp;
  route: Waypoint[];
  passengers: number;
  status: FlightStatus;
}

export interface Waypoint {
  waypointId: string;
  location: GeoLocation;
  altitude: number;
  speed?: number;
  type: 'departure' | 'cruise' | 'approach' | 'arrival';
}

export enum FlightStatus {
  SCHEDULED = 'scheduled',
  BOARDING = 'boarding',
  IN_FLIGHT = 'in_flight',
  LANDED = 'landed',
  CANCELLED = 'cancelled',
  DELAYED = 'delayed',
}

export interface FlightTelemetry {
  timestamp: Timestamp;
  aircraftId: string;
  position: GeoLocation;
  groundSpeed: number;
  airSpeed: number;
  heading: number;
  verticalSpeed: number;
  batteryState: number;
  motorStatus: MotorController[];
  warnings: Warning[];
}

export interface Warning {
  code: string;
  severity: 'info' | 'caution' | 'warning' | 'critical';
  message: string;
  timestamp: Timestamp;
}

export interface WIAConfig {
  apiKey: string;
  baseURL?: string;
  timeout?: number;
  debug?: boolean;
}

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  timestamp: Timestamp;
  requestId: string;
}

export interface APIError {
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
