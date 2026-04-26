/**
 * WIA-SPACE-012: Mars Mission Standard - TypeScript Types
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Enums
// ============================================================================

export enum MissionType {
  FLYBY = 'FLYBY',
  ORBITAL = 'ORBITAL',
  LANDER = 'LANDER',
  ROVER = 'ROVER',
  SAMPLE_RETURN = 'SAMPLE_RETURN',
  CREWED = 'CREWED',
  CARGO = 'CARGO',
}

export enum TrajectoryType {
  HOHMANN = 'HOHMANN',
  CONJUNCTION = 'CONJUNCTION',
  OPPOSITION = 'OPPOSITION',
  BALLISTIC = 'BALLISTIC',
  LOW_THRUST = 'LOW_THRUST',
}

export enum MissionPhase {
  PLANNING = 'PLANNING',
  EARTH_ORBIT = 'EARTH_ORBIT',
  TRANS_MARS = 'TRANS_MARS',
  MARS_APPROACH = 'MARS_APPROACH',
  MARS_ORBIT = 'MARS_ORBIT',
  ENTRY_DESCENT = 'ENTRY_DESCENT',
  SURFACE_OPS = 'SURFACE_OPS',
}

export enum SurfaceFeature {
  CRATER = 'CRATER',
  VALLEY = 'VALLEY',
  VOLCANO = 'VOLCANO',
  PLAIN = 'PLAIN',
  CANYON = 'CANYON',
}

// ============================================================================
// Trajectory Types
// ============================================================================

export interface Trajectory {
  trajectoryId: string;
  type: TrajectoryType;
  launchDate: string;
  arrivalDate: string;
  durationDays: number;
  c3Energy: number;
}

export interface LaunchWindow {
  openDate: string;
  closeDate: string;
  optimalDate: string;
}

// ============================================================================
// Crew Types
// ============================================================================

export interface Crew {
  crewId: string;
  missionId: string;
  totalCount: number;
  members: CrewMember[];
  radiationLimitMsv: number;
}

export interface CrewMember {
  memberId: string;
  name: string;
  role: 'commander' | 'pilot' | 'engineer' | 'scientist';
  specialization: string;
  radiationExposure: number;
  healthStatus: HealthStatus;
}

export interface HealthStatus {
  overall: 'nominal' | 'minor_concern' | 'attention_required';
  boneDensity: number;
  muscleAtrophy: number;
  radiationDose: number;
}

// ============================================================================
// Landing Types
// ============================================================================

export interface LandingSite {
  siteId: string;
  name: string;
  latitude: number;
  longitude: number;
  elevationM: number;
  feature: SurfaceFeature;
  suitabilityScore: number;
}

export interface EDLSequence {
  sequenceId: string;
  phases: EDLPhase[];
  landingAccuracyKm: number;
}

export interface EDLPhase {
  name: string;
  startAltitudeKm: number;
  endAltitudeKm: number;
  durationS: number;
}

// ============================================================================
// Surface Operations Types
// ============================================================================

export interface SurfaceHabitat {
  habitatId: string;
  type: 'inflatable' | 'rigid' | 'buried';
  volumeCubicM: number;
  crewCapacity: number;
  powerRequirementKw: number;
}

export interface Rover {
  roverId: string;
  name: string;
  type: 'science' | 'transport';
  massKg: number;
  rangeKm: number;
  speedKmH: number;
  status: 'active' | 'charging' | 'maintenance';
}

// ============================================================================
// Mission Types
// ============================================================================

export interface MarsMission {
  missionId: string;
  name: string;
  agency: string;
  type: MissionType;
  spacecraft: Spacecraft;
  trajectory: Trajectory;
  crew?: Crew;
  landingSite?: LandingSite;
  phase: MissionPhase;
  launchDate: string;
  status: 'planning' | 'active' | 'completed' | 'failed';
}

export interface Spacecraft {
  spacecraftId: string;
  name: string;
  dryMassKg: number;
  totalMassKg: number;
  deltaVCapableKmS: number;
  crewCapacity?: number;
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
