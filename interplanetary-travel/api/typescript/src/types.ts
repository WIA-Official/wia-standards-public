/**
 * WIA-SPACE-013: Interplanetary Travel Standard - TypeScript Types
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Enums
// ============================================================================

export enum Planet {
  MERCURY = 'MERCURY',
  VENUS = 'VENUS',
  EARTH = 'EARTH',
  MARS = 'MARS',
  JUPITER = 'JUPITER',
  SATURN = 'SATURN',
  URANUS = 'URANUS',
  NEPTUNE = 'NEPTUNE',
}

export enum TrajectoryType {
  DIRECT = 'DIRECT',
  GRAVITY_ASSIST = 'GRAVITY_ASSIST',
  ELLIPTICAL = 'ELLIPTICAL',
  HOHMANN_TRANSFER = 'HOHMANN_TRANSFER',
  LOW_THRUST = 'LOW_THRUST',
}

export enum PropulsionType {
  CHEMICAL = 'CHEMICAL',
  ION_DRIVE = 'ION_DRIVE',
  NUCLEAR_THERMAL = 'NUCLEAR_THERMAL',
  NUCLEAR_ELECTRIC = 'NUCLEAR_ELECTRIC',
  SOLAR_SAIL = 'SOLAR_SAIL',
}

export enum MissionPhase {
  PLANNING = 'PLANNING',
  LAUNCH = 'LAUNCH',
  CRUISE = 'CRUISE',
  APPROACH = 'APPROACH',
  ORBIT_INSERTION = 'ORBIT_INSERTION',
  LANDING = 'LANDING',
  SURFACE_OPS = 'SURFACE_OPS',
}

export enum MissionType {
  FLYBY = 'FLYBY',
  ORBITAL = 'ORBITAL',
  LANDER = 'LANDER',
  CREWED = 'CREWED',
  CARGO = 'CARGO',
}

// ============================================================================
// Trajectory Types
// ============================================================================

export interface Trajectory {
  trajectoryId: string;
  type: TrajectoryType;
  origin: string;
  destination: string;
  waypoints: Waypoint[];
  totalDurationDays: number;
  deltaVRequired: number;
}

export interface Waypoint {
  body: string;
  arrivalDate: string;
  departureDate?: string;
  gravityAssist?: GravityAssist;
}

export interface GravityAssist {
  body: string;
  periapsisKm: number;
  deltaVGained: number;
}

// ============================================================================
// Propulsion Types
// ============================================================================

export interface Propulsion {
  propulsionId: string;
  type: PropulsionType;
  specificImpulse: number;
  thrust: number;
  propellantMassKg: number;
  deltaVCapable: number;
}

// ============================================================================
// Spacecraft Types
// ============================================================================

export interface Spacecraft {
  spacecraftId: string;
  name: string;
  type: 'crewed' | 'cargo' | 'probe';
  dryMassKg: number;
  propulsion: Propulsion;
  crewCapacity?: number;
  payloadCapacityKg: number;
}

export interface CrewModule {
  capacity: number;
  lifeSupportDays: number;
  radiationShielding: number;
}

// ============================================================================
// Mission Types
// ============================================================================

export interface FlightPlan {
  planId: string;
  missionName: string;
  missionType: MissionType;
  origin: string;
  destination: string;
  spacecraft: Spacecraft;
  trajectory: Trajectory;
  crew?: CrewManifest;
  launchDate: string;
  currentPhase: MissionPhase;
}

export interface CrewManifest {
  totalCrew: number;
  members: CrewMember[];
  radiationLimitMsv: number;
}

export interface CrewMember {
  crewId: string;
  name: string;
  role: string;
  specialization: string;
}

// ============================================================================
// Navigation Types
// ============================================================================

export interface NavigationState {
  timestamp: string;
  position: Vector3D;
  velocity: Vector3D;
}

export interface Vector3D {
  x: number;
  y: number;
  z: number;
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
  status: 'success' | 'error';
  data?: T;
  error?: string;
  timestamp: string;
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
