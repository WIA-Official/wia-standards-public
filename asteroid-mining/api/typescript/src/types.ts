/**
 * WIA-SPACE-027: Asteroid Mining Standard - TypeScript Types
 * Version: 1.0
 * Philosophy: 弘익人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

export type AsteroidType = 'C-Type' | 'S-Type' | 'M-Type' | 'X-Type' | 'Other';
export type MissionStatus = 'Surveyed' | 'Planned' | 'Active' | 'Completed' | 'Depleted';
export type ExtractionMethod = 'Surface' | 'Subsurface' | 'Capture' | 'InSitu';
export type ResourceType = 'Water' | 'Metals' | 'Silicates' | 'Organics' | 'PGM';

export interface OrbitalParameters {
  semiMajorAxis: number; // AU
  eccentricity: number; // 0-1
  inclination: number; // degrees
  perihelion: number; // AU
  aphelion: number; // AU
  period?: number; // years
}

export interface Asteroid {
  designation: string; // e.g., "101955 Bennu"
  alternateNames?: string[];
  type: AsteroidType;
  diameter: number; // meters
  mass: number; // kg
  rotation Period?: number; // hours
  orbit: OrbitalParameters;
  deltaV: number; // km/s (Earth-asteroid-Earth)
  discoveryDate?: string; // ISO8601
}

export interface Composition {
  water?: number; // mass %
  silicates?: number; // mass %
  metals?: {
    iron?: number; // mass %
    nickel?: number; // mass %
    platinum?: number; // ppm
    palladium?: number; // ppm
    rhodium?: number; // ppm
    gold?: number; // ppm
    cobalt?: number; // mass %
  };
  organics?: number; // mass %
  volatiles?: number; // mass %
}

export interface ResourceEstimate {
  totalMass: number; // tonnes
  waterMass?: number; // tonnes
  metalMass?: number; // tonnes
  pgmMass?: number; // kg
  silicateMass?: number; // tonnes
  economicValue: number; // USD
  valueBreakdown?: {
    water?: number;
    metals?: number;
    pgm?: number;
    other?: number;
  };
}

export interface MiningMission {
  id: string;
  name: string;
  operator: {
    entity: string;
    country: string; // ISO 3166-1 alpha-3
    license?: string;
  };
  target: Asteroid;
  method: ExtractionMethod;
  status: MissionStatus;
  timeline: {
    launchDate?: string; // ISO8601
    arrivalDate?: string; // ISO8601
    operationStart?: string; // ISO8601
    operationEnd?: string; // ISO8601
    returnDate?: string; // ISO8601
  };
  extractionPlan: {
    targetYield: number; // tonnes
    duration: number; // days
    equipment: string[];
    processingMethod: string;
  };
  economics: {
    estimatedCost: number; // USD
    projectedRevenue: number; // USD
    roi?: number; // %
  };
}

export interface DeltaVCalculation {
  departure: number; // km/s
  arrival: number; // km/s
  total: number; // km/s
  adjustedForEccentricity: number; // km/s
  transferTime?: number; // days
  feasibility: 'Highly Feasible' | 'Feasible' | 'Challenging' | 'Not Feasible';
}

export interface ExtractionRate {
  method: ExtractionMethod;
  ratePerHour: number; // kg/hour
  powerRequirement: number; // kW
  efficiency: number; // %
  equipmentMass: number; // kg
}

export interface ProcessingCapability {
  type: 'Water Extraction' | 'Metal Refining' | 'Electrolysis' | '3D Printing';
  inputMaterial: string;
  outputMaterial: string;
  conversionRate: number; // %
  energyRequired: number; // kWh/kg
  temperature?: number; // °C
}

export interface TransportVehicle {
  id: string;
  type: 'Ion Drive' | 'Chemical' | 'Solar Sail' | 'Hybrid';
  payloadCapacity: number; // tonnes
  specificImpulse: number; // seconds
  thrust: number; // Newtons
  powerSource: string;
  fuelMass?: number; // tonnes
}

export interface LegalCompliance {
  resourceClaimFiled: boolean;
  filingDate?: string; // ISO8601
  nationalLaw?: string;
  internationalApproval: boolean;
  environmentalAssessment: boolean;
  liabilityInsurance: {
    provider: string;
    coverage: number; // USD
    policyNumber: string;
  };
}

export interface AsteroidMiningConfig {
  apiKey: string;
  baseURL?: string;
  timeout?: number;
  debug?: boolean;
}

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
  };
  timestamp: string;
}

export interface NEACatalog {
  asteroids: Asteroid[];
  totalCount: number;
  filterCriteria?: {
    maxDeltaV?: number;
    minDiameter?: number;
    types?: AsteroidType[];
  };
}

export interface MiningFeasibility {
  asteroid: Asteroid;
  deltaV: DeltaVCalculation;
  resources: ResourceEstimate;
  technical Feasibility: 'High' | 'Medium' | 'Low';
  economicFeasibility: 'Profitable' | 'Marginal' | 'Not Viable';
  timeline: {
    missionDuration: number; // days
    operationsPeriod: number; // days
  };
  risks: string[];
  recommendations: string[];
}
