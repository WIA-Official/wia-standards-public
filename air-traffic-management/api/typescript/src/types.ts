/**
 * WIA Air Traffic Management Standard - TypeScript Types
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 * © 2025 SmileStory Inc. / WIA
 */

export type Timestamp = string;
export type SurveillanceType = 'psr' | 'ssr' | 'ads-b' | 'mlat' | 'asde-x' | 'wam';
export type CommunicationType = 'vhf' | 'hf' | 'cpdlc' | 'satcom' | 'vdl';
export type NavigationType = 'vor' | 'dme' | 'gnss' | 'ils' | 'pbn' | 'rnav' | 'rnp';

// ============================================================================
// Position Types
// ============================================================================

export interface Position {
  latitude: number;
  longitude: number;
  altitude: number;
  altitudeReference: AltitudeReference;
  timestamp: Timestamp;
  accuracy?: PositionAccuracy;
}

export enum AltitudeReference {
  MSL = 'msl',
  AGL = 'agl',
  FL = 'flight_level',
}

export interface PositionAccuracy {
  horizontal: number;
  vertical: number;
  velocity: number;
}

// ============================================================================
// Aircraft Types
// ============================================================================

export interface Aircraft {
  id: string;
  callsign: string;
  registration: string;
  squawk: string;
  icao24: string;
  aircraftType: string;
  category: AircraftCategory;
  position: Position;
  speed: number;
  groundSpeed: number;
  heading: number;
  track: number;
  verticalRate: number;
  status: FlightStatus;
  surveillance: SurveillanceData;
}

export enum AircraftCategory {
  LIGHT = 'light',
  MEDIUM = 'medium',
  HEAVY = 'heavy',
  SUPER = 'super',
  ROTORCRAFT = 'rotorcraft',
  UAV = 'uav',
}

export enum FlightStatus {
  PARKED = 'parked',
  TAXIING = 'taxiing',
  TAKING_OFF = 'taking_off',
  CLIMBING = 'climbing',
  EN_ROUTE = 'en_route',
  DESCENDING = 'descending',
  APPROACHING = 'approaching',
  LANDING = 'landing',
  HOLDING = 'holding',
  EMERGENCY = 'emergency',
}

export interface SurveillanceData {
  source: SurveillanceType;
  quality: number;
  lastUpdate: Timestamp;
  nic: number;
  nac: number;
  sil: number;
}

// ============================================================================
// Flight Plan Types
// ============================================================================

export interface FlightPlan {
  flightId: string;
  callsign: string;
  aircraftType: string;
  departure: Airport;
  destination: Airport;
  alternate?: Airport;
  departureTime: Timestamp;
  arrivalTime: Timestamp;
  route: RouteSegment[];
  cruiseAltitude: number;
  cruiseSpeed: number;
  flightRules: FlightRules;
  equipment: string;
  status: FlightPlanStatus;
  remarks?: string;
}

export interface Airport {
  icao: string;
  iata?: string;
  name: string;
  position: Position;
  elevation: number;
  runways?: Runway[];
}

export interface Runway {
  designator: string;
  length: number;
  width: number;
  heading: number;
  surface: string;
  status: 'open' | 'closed' | 'maintenance';
}

export interface RouteSegment {
  waypoint: Waypoint;
  altitude: number;
  speed?: number;
  estimatedTime: Timestamp;
  airway?: string;
}

export interface Waypoint {
  identifier: string;
  name?: string;
  type: WaypointType;
  position: Position;
}

export enum WaypointType {
  VOR = 'vor',
  NDB = 'ndb',
  FIX = 'fix',
  AIRPORT = 'airport',
  RNAV = 'rnav',
}

export enum FlightRules {
  IFR = 'ifr',
  VFR = 'vfr',
  SVFR = 'svfr',
  MIXED = 'mixed',
}

export enum FlightPlanStatus {
  FILED = 'filed',
  APPROVED = 'approved',
  ACTIVE = 'active',
  COMPLETED = 'completed',
  CANCELLED = 'cancelled',
  AMENDED = 'amended',
}

// ============================================================================
// Conflict Detection Types
// ============================================================================

export interface ConflictAlert {
  alertId: string;
  timestamp: Timestamp;
  type: ConflictType;
  severity: AlertSeverity;
  aircraft1: string;
  aircraft2?: string;
  position: Position;
  timeToConflict: number;
  horizontalSeparation: number;
  verticalSeparation: number;
  resolution?: Resolution;
  status: AlertStatus;
}

export enum ConflictType {
  LOSS_OF_SEPARATION = 'los',
  PREDICTED_LOS = 'plos',
  TERRAIN = 'terrain',
  RESTRICTED_AIRSPACE = 'restricted',
  WAKE_TURBULENCE = 'wake',
  RUNWAY_INCURSION = 'runway',
}

export enum AlertSeverity {
  CAUTION = 'caution',
  WARNING = 'warning',
  ALERT = 'alert',
}

export enum AlertStatus {
  ACTIVE = 'active',
  RESOLVED = 'resolved',
  ACKNOWLEDGED = 'acknowledged',
}

export interface Resolution {
  type: ResolutionType;
  instruction: string;
  heading?: number;
  altitude?: number;
  speed?: number;
}

export enum ResolutionType {
  HEADING_CHANGE = 'heading',
  ALTITUDE_CHANGE = 'altitude',
  SPEED_CHANGE = 'speed',
  HOLD = 'hold',
  VECTOR = 'vector',
}

// ============================================================================
// Weather Types
// ============================================================================

export interface WeatherData {
  stationId: string;
  timestamp: Timestamp;
  type: WeatherType;
  wind: Wind;
  visibility: number;
  ceiling?: number;
  temperature: number;
  dewpoint: number;
  pressure: number;
  conditions: WeatherCondition[];
  remarks?: string;
}

export enum WeatherType {
  METAR = 'metar',
  SPECI = 'speci',
  TAF = 'taf',
  SIGMET = 'sigmet',
  AIRMET = 'airmet',
}

export interface Wind {
  direction: number;
  speed: number;
  gust?: number;
  variable?: boolean;
}

export enum WeatherCondition {
  CLEAR = 'clr',
  FEW = 'few',
  SCATTERED = 'sct',
  BROKEN = 'bkn',
  OVERCAST = 'ovc',
  RAIN = 'ra',
  SNOW = 'sn',
  FOG = 'fg',
  THUNDERSTORM = 'ts',
  ICING = 'ic',
}

// ============================================================================
// API Types
// ============================================================================

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
