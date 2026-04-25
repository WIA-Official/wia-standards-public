/**
 * WIA Security Robot Standard - Type Definitions
 * @module wia-security-robot
 */

export enum PatrolMode {
  Scheduled = 'scheduled', Random = 'random', Manual = 'manual',
  AlertResponse = 'alert_response', Perimeter = 'perimeter'
}

export enum AlertSeverity {
  Info = 'info', Low = 'low', Medium = 'medium', High = 'high', Critical = 'critical'
}

export enum ThreatType {
  Intrusion = 'intrusion', Fire = 'fire', Smoke = 'smoke',
  WaterLeak = 'water_leak', Vandalism = 'vandalism', SuspiciousBehavior = 'suspicious_behavior',
  UnauthorizedAccess = 'unauthorized_access', Loitering = 'loitering'
}

export interface SecurityRobotSpec {
  standard: 'WIA-SECURITY-ROBOT';
  version: string;
  robotId: string;
  model: string;
  manufacturer: string;
  cameras: CameraSpec[];
  sensors: string[];
  communication: string[];
  batteryCapacity: number;
  maxSpeed: number;
  weatherResistance: string;
}

export interface CameraSpec {
  id: string;
  type: 'rgb' | 'thermal' | 'night_vision' | 'ptz' | '360';
  resolution: { width: number; height: number };
  frameRate: number;
  fieldOfView: number;
  nightVision: boolean;
  zoom?: { min: number; max: number };
}

export interface PatrolRoute {
  id: string;
  name: string;
  waypoints: Waypoint[];
  duration: number;
  schedule?: { startTime: string; endTime: string; days: string[] };
  priority: number;
}

export interface Waypoint {
  id: string;
  position: { x: number; y: number; z?: number };
  dwellTime: number;
  actions: string[];
  checkpoint?: boolean;
}

export interface PatrolSession {
  id: string;
  routeId: string;
  robotId: string;
  startTime: number;
  endTime?: number;
  status: 'active' | 'completed' | 'interrupted' | 'paused';
  waypointsCompleted: number;
  incidents: Incident[];
  distanceTraveled: number;
}

export interface Incident {
  id: string;
  timestamp: number;
  type: ThreatType;
  severity: AlertSeverity;
  location: { x: number; y: number };
  description: string;
  evidence: { type: 'image' | 'video' | 'audio'; url: string }[];
  responded: boolean;
  resolvedAt?: number;
}

export interface Alert {
  id: string;
  robotId: string;
  timestamp: number;
  type: ThreatType;
  severity: AlertSeverity;
  location: { x: number; y: number };
  description: string;
  acknowledged: boolean;
  acknowledgedBy?: string;
  resolvedAt?: number;
  actions: string[];
}

export interface DetectedPerson {
  id: string;
  timestamp: number;
  boundingBox: { x: number; y: number; width: number; height: number };
  confidence: number;
  attributes: { age?: string; gender?: string; clothing?: string[] };
  authorized?: boolean;
  faceMatch?: { personId: string; confidence: number };
}

export interface ZoneConfig {
  id: string;
  name: string;
  type: 'restricted' | 'monitored' | 'public' | 'sensitive';
  boundaries: { x: number; y: number }[];
  accessLevel: number;
  alertOnEntry: boolean;
  schedule?: { active: boolean; startTime: string; endTime: string }[];
}

export interface EnvironmentReading {
  timestamp: number;
  temperature: number;
  humidity: number;
  smoke: number;
  co2: number;
  noise: number;
  light: number;
  motion: boolean;
}

export enum CertificationLevel { Bronze = 'bronze', Silver = 'silver', Gold = 'gold' }

export interface ComplianceReport {
  standard: 'WIA-SECURITY-ROBOT';
  testDate: string;
  robotId: string;
  certificationLevel: CertificationLevel;
  tests: { name: string; passed: boolean }[];
  compliant: boolean;
}

export type SecurityEventType = 'alert' | 'patrol-complete' | 'person-detected' | 'zone-breach' | 'environment-alert';
export type EventCallback<T = unknown> = (data: T) => void;
