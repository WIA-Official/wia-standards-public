/**
 * WIA-SPACE-011: Moon Base Standard - TypeScript Types
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Enums
// ============================================================================

export enum ModuleStatus {
  OPERATIONAL = 'OPERATIONAL',
  MAINTENANCE = 'MAINTENANCE',
  OFFLINE = 'OFFLINE',
  DEPLOYING = 'DEPLOYING',
  EMERGENCY = 'EMERGENCY',
}

export enum ModuleType {
  HABITAT = 'HABITAT',
  LAB = 'LAB',
  STORAGE = 'STORAGE',
  POWER = 'POWER',
  ISRU = 'ISRU',
  COMMUNICATIONS = 'COMMUNICATIONS',
  MEDICAL = 'MEDICAL',
  GREENHOUSE = 'GREENHOUSE',
}

export enum CrewRole {
  COMMANDER = 'COMMANDER',
  PILOT = 'PILOT',
  ENGINEER = 'ENGINEER',
  SCIENTIST = 'SCIENTIST',
  PHYSICIAN = 'PHYSICIAN',
}

// ============================================================================
// Location Types
// ============================================================================

export interface Location {
  crater?: string;
  region: string;
  latitude: number;
  longitude: number;
  elevationM: number;
  solarExposure: number;
}

// ============================================================================
// Module Types
// ============================================================================

export interface BaseModule {
  moduleId: string;
  name: string;
  type: ModuleType;
  status: ModuleStatus;
  capacity?: number;
  powerConsumptionKw: number;
  volumeCubicM: number;
  installedDate: string;
  lastMaintenance?: string;
}

export interface HabitatModule extends BaseModule {
  crewCapacity: number;
  lifeSupportStatus: LifeSupportStatus;
  radiationShieldingMm: number;
}

export interface LifeSupportStatus {
  oxygenLevel: number;
  co2Level: number;
  humidity: number;
  temperature: number;
  pressureKpa: number;
}

// ============================================================================
// Resource Types
// ============================================================================

export interface Resources {
  powerKw: number;
  powerCapacityKw: number;
  oxygenKgDay: number;
  waterRecyclingPercent: number;
  waterIceKg: number;
  foodSupplyDays: number;
}

export interface ISRUProduction {
  oxygenKgDay: number;
  waterKgDay: number;
  helium3MgDay?: number;
  metalsKgDay?: number;
  regolithProcessedKgDay: number;
}

// ============================================================================
// Crew Types
// ============================================================================

export interface CrewMember {
  crewId: string;
  name: string;
  role: CrewRole;
  nationality: string;
  missionDays: number;
  radiationExposure: number;
  healthStatus: 'nominal' | 'minor_issue' | 'medical_attention';
}

// ============================================================================
// Base Types
// ============================================================================

export interface LunarBase {
  baseId: string;
  name: string;
  location: Location;
  modules: BaseModule[];
  resources: Resources;
  isruProduction?: ISRUProduction;
  crew: CrewMember[];
  establishedDate: string;
  status: 'operational' | 'construction' | 'emergency';
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
