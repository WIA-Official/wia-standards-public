/**
 * WIA-SPACE-024: Space Law Standard - TypeScript Types
 * Version: 1.0
 * Philosophy: 弘익人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

export type EntityType = 'Government' | 'Commercial' | 'International' | 'NonProfit';
export type ActivityType = 'Launch' | 'Landing' | 'Orbit' | 'Mining' | 'Research' | 'Settlement';
export type CelestialBody = 'Earth' | 'Moon' | 'Mars' | 'Asteroid' | 'Other';
export type TreatyStatus = 'Ratified' | 'Signed' | 'NotParty';
export type ComplianceStatus = 'Compliant' | 'UnderReview' | 'NonCompliant';

export interface SpaceOperator {
  entity: string;
  country: string; // ISO 3166-1 alpha-3
  type: EntityType;
  establishedDate?: string;
  licenses?: string[];
}

export interface MissionRegistration {
  id: string;
  name: string;
  operator: SpaceOperator;
  registryState: string; // ISO 3166-1 alpha-3
  activity: ActivityType;
  location: {
    celestialBody: CelestialBody;
    coordinates?: string;
    orbitParameters?: OrbitalParameters;
  };
  timeline: {
    launchDate: string; // ISO8601
    missionDuration: number; // days
    deorbitPlan?: string;
  };
  registrationDate: string; // ISO8601
}

export interface OrbitalParameters {
  semiMajorAxis?: number; // km
  eccentricity?: number;
  inclination?: number; // degrees
  perigee?: number; // km
  apogee?: number; // km
}

export interface ResourceClaim {
  id: string;
  missionId: string;
  celestialBody: CelestialBody;
  resourceTypes: string[];
  estimatedVolume?: number; // tonnes
  location: {
    coordinates?: string;
    area?: number; // km²
  };
  filingDate: string; // ISO8601
  status: 'Filed' | 'Approved' | 'Disputed' | 'Rejected';
  nationalLaw?: string;
}

export interface TreatyCompliance {
  outerSpaceTreaty: boolean;
  rescueAgreement?: boolean;
  liabilityConvention?: boolean;
  registrationConvention?: boolean;
  moonAgreement?: boolean;
  artemisAccords?: boolean;
  nationalLicense?: string;
  certifications?: string[];
}

export interface LiabilityInsurance {
  provider: string;
  policyNumber: string;
  coverageAmount: number; // USD
  effectiveDate: string; // ISO8601
  expirationDate: string; // ISO8601
  coverageType: 'Surface' | 'Space' | 'Comprehensive';
}

export interface SpaceActivity {
  mission: MissionRegistration;
  resourceExtraction?: {
    planned: boolean;
    claims?: ResourceClaim[];
  };
  compliance: TreatyCompliance;
  liability: {
    primaryContact: string;
    insurance: LiabilityInsurance;
  };
  safetyZone?: {
    radius: number; // meters
    justification: string;
    duration: number; // days
  };
}

export interface Dispute {
  id: string;
  parties: string[]; // entity names
  subject: string;
  celestialBody?: CelestialBody;
  filingDate: string; // ISO8601
  status: 'Open' | 'Mediation' | 'Arbitration' | 'Resolved' | 'Rejected';
  resolution?: string;
}

export interface Treaty {
  name: string;
  year: number;
  ratifyingStates: number;
  status: 'Active' | 'Failed' | 'Evolving';
  articles?: string[];
}

export interface NationalSpaceLaw {
  country: string; // ISO 3166-1 alpha-3
  lawName: string;
  enactedYear: number;
  resourceRights: boolean;
  licensingRequired: boolean;
  keyProvisions: string[];
}

export interface SpaceLawConfig {
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

export interface ComplianceReport {
  missionId: string;
  overallStatus: ComplianceStatus;
  treaties: {
    name: string;
    compliant: boolean;
    notes?: string;
  }[];
  recommendations: string[];
  generatedAt: string; // ISO8601
}

export interface SafetyZoneRequest {
  missionId: string;
  location: {
    celestialBody: CelestialBody;
    coordinates: string;
  };
  radius: number; // meters
  duration: number; // days
  justification: string;
  operationalRequirements: string;
}
