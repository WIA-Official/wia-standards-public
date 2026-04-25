/**
 * WIA-SPACE-026: Space Debris Tracking Standard - TypeScript Types
 * Version: 1.0
 * Philosophy: 弘익人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

export type ObjectClassification = 'Payload' | 'RocketBody' | 'Debris' | 'Unknown';
export type ObservationQuality = 'High' | 'Medium' | 'Low';
export type ConjunctionStatus = 'Safe' | 'Monitoring' | 'Warning' | 'Alert';

export interface SpaceObject {
  noradId: number;
  name: string;
  intlDesignator?: string;
  classification: ObjectClassification;
  mass?: number; // kg
  size?: number; // cm (estimated)
  rcs?: number; // radar cross-section, m²
}

export interface OrbitalElements {
  epoch: string; // ISO8601
  meanMotion: number; // revolutions per day
  eccentricity: number; // 0-1
  inclination: number; // degrees
  raan: number; // Right Ascension of Ascending Node, degrees
  argOfPerigee: number; // degrees
  meanAnomaly: number; // degrees
  bstar?: number; // drag term
  revolutionNumber?: number;
}

export interface TwoLineElement {
  line1: string; // 69 characters
  line2: string; // 69 characters
  epoch: string; // ISO8601
  elementSetNumber?: number;
}

export interface TrackingData {
  object: SpaceObject;
  orbit: OrbitalElements;
  tle?: TwoLineElement;
  tracking: {
    lastObservation: string; // ISO8601
    observationQuality: ObservationQuality;
    positionError: number; // meters
    velocityError?: number; // m/s
    nextPredictedPass?: string; // ISO8601
  };
}

export interface StateVector {
  position: {
    x: number; // km
    y: number; // km
    z: number; // km
  };
  velocity: {
    vx: number; // km/s
    vy: number; // km/s
    vz: number; // km/s
  };
  epoch: string; // ISO8601
}

export interface Conjunction {
  id: string;
  object1: {
    noradId: number;
    name: string;
  };
  object2: {
    noradId: number;
    name: string;
  };
  tca: string; // Time of Closest Approach, ISO8601
  missDistance: number; // meters
  relativeVelocity: number; // m/s
  collisionProbability: number; // 0-1
  status: ConjunctionStatus;
  screeningTime: string; // ISO8601
}

export interface ConjunctionDataMessage {
  messageId: string;
  creationDate: string; // ISO8601
  originator: string;
  conjunction: Conjunction;
  covariance?: {
    object1: number[][]; // 3x3 covariance matrix
    object2: number[][]; // 3x3 covariance matrix
  };
  recommendations?: string[];
}

export interface CollisionRisk {
  hardBodyRadius: number; // meters
  positionUncertainty: number; // meters
  probability: number; // 0-1
  riskLevel: 'Low' | 'Medium' | 'High' | 'Critical';
  maneuverRecommended: boolean;
}

export interface OrbitDecayEstimate {
  object: SpaceObject;
  currentAltitude: number; // km
  decayRate: number; // km/day
  estimatedLifetime: number; // days
  reentryDate?: string; // ISO8601
  complianceWithIADC: boolean; // 25-year rule
}

export interface DebrisTrackingConfig {
  apiKey: string;
  baseURL?: string;
  timeout?: number;
  debug?: boolean;
  alertThresholds?: {
    collisionProbability: number; // default: 1e-4
    missDistance: number; // meters, default: 1000
  };
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

export interface CatalogStatistics {
  totalObjects: number;
  payloads: number;
  rocketBodies: number;
  debris: number;
  activeManeuverableObjects: number;
  objectsByAltitude: {
    leo: number; // <2000 km
    meo: number; // 2000-35786 km
    geo: number; // ~35786 km
    heo: number; // elliptical
  };
}

export interface DebrisMitigationCompliance {
  objectId: number;
  postMissionDisposal: boolean;
  disposalMethod?: 'Deorbit' | 'Graveyard' | 'Retrieval';
  estimatedDisposalTime?: number; // days
  iadc25YearRule: boolean;
  passivationComplete: boolean;
}
