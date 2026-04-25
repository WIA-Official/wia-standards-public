/**
 * WIA-SPACE-014: Launch Vehicle Standard - TypeScript Types
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Enums
// ============================================================================

export enum PropellantType {
  LOX_RP1 = 'LOX/RP-1',
  LOX_LH2 = 'LOX/LH2',
  LOX_CH4 = 'LOX/CH4',
  SOLID = 'Solid',
  HYPERGOLIC = 'Hypergolic',
}

export enum VehicleClass {
  MICRO = 'Micro',
  SMALL = 'Small',
  MEDIUM = 'Medium',
  HEAVY = 'Heavy',
  SUPER_HEAVY = 'Super-Heavy',
}

export enum LaunchStatus {
  INTEGRATION = 'integration',
  FUELING = 'fueling',
  COUNTDOWN = 'countdown',
  LIFTOFF = 'liftoff',
  MAX_Q = 'max-q',
  MECO = 'meco',
  STAGE_SEPARATION = 'stage-separation',
  ORBITAL = 'orbital',
  COMPLETED = 'completed',
  ABORT = 'abort',
}

export enum RecoveryMethod {
  EXPENDABLE = 'expendable',
  PROPULSIVE = 'propulsive',
  PARACHUTE = 'parachute',
}

// ============================================================================
// Vehicle Types
// ============================================================================

export interface LaunchVehicle {
  vehicleId: string;
  name: string;
  manufacturer: string;
  vehicleClass: VehicleClass;
  stages: RocketStage[];
  fairing: Fairing;
  payloadCapacityLeoKg: number;
  payloadCapacityGtoKg?: number;
  heightMeters: number;
  diameterMeters: number;
  reusable: boolean;
  flightCount: number;
}

export interface RocketStage {
  stageNumber: number;
  engineType: string;
  engineCount: number;
  propellantType: PropellantType;
  propellantMassKg: number;
  dryMassKg: number;
  thrustKn: number;
  specificImpulseVac: number;
  burnTimeS: number;
  recoveryMethod?: RecoveryMethod;
}

export interface Fairing {
  heightMeters: number;
  diameterMeters: number;
  volumeCubicMeters: number;
  recoverable: boolean;
}

// ============================================================================
// Telemetry Types
// ============================================================================

export interface TelemetryData {
  timestamp: string;
  missionElapsedTimeS: number;
  altitudeKm: number;
  velocityKmS: number;
  accelerationG: number;
  downrangeDistanceKm: number;
  propellantRemainingPercent: number;
  stageActive: number;
}

export interface EngineStatus {
  engineId: string;
  active: boolean;
  throttlePercent: number;
  chamberPressureBar?: number;
}

// ============================================================================
// Mission Types
// ============================================================================

export interface LaunchMission {
  missionId: string;
  missionName: string;
  vehicleId: string;
  customer: string;
  payload: Payload[];
  launchSite: LaunchSite;
  targetOrbit: OrbitParameters;
  scheduledLaunchTime: string;
  status: LaunchStatus;
}

export interface Payload {
  payloadId: string;
  name: string;
  customer: string;
  type: 'satellite' | 'cargo' | 'crewed';
  massKg: number;
  deployed: boolean;
}

export interface LaunchSite {
  siteId: string;
  name: string;
  latitude: number;
  longitude: number;
  padDesignation: string;
}

export interface OrbitParameters {
  type: 'LEO' | 'MEO' | 'GEO' | 'GTO' | 'SSO';
  altitudeKm?: number;
  perigeeKm?: number;
  apogeeKm?: number;
  inclinationDeg: number;
}

// ============================================================================
// API Types
// ============================================================================

export interface WIAConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  debug?: boolean;
}

export interface APIResponse<T = unknown> {
  success: boolean;
  data?: T;
  error?: APIError;
  timestamp: string;
}

export interface APIError {
  code: string;
  message: string;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    page: number;
    pageSize: number;
    totalPages: number;
    totalCount: number;
  };
}
