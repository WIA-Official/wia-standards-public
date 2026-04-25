/**
 * WIA-IND-011: Sports Analytics Standard - Type Definitions
 * @module wia-ind-011
 */

export enum SportType {
  Soccer = 'soccer', Basketball = 'basketball', Baseball = 'baseball',
  AmericanFootball = 'american_football', Tennis = 'tennis', Golf = 'golf',
  Swimming = 'swimming', Athletics = 'athletics', Cycling = 'cycling',
  MMA = 'mma', Boxing = 'boxing', Esports = 'esports'
}

export enum MetricCategory {
  Physical = 'physical', Technical = 'technical', Tactical = 'tactical',
  Mental = 'mental', Recovery = 'recovery'
}

export interface Athlete {
  id: string;
  name: string;
  sport: SportType;
  position?: string;
  team?: string;
  age: number;
  height: number;
  weight: number;
  nationality: string;
  metrics: PerformanceMetric[];
}

export interface PerformanceMetric {
  id: string;
  name: string;
  category: MetricCategory;
  value: number;
  unit: string;
  timestamp: number;
  source: 'wearable' | 'video' | 'manual' | 'ai';
  confidence?: number;
}

export interface TrainingSession {
  id: string;
  athleteId: string;
  type: string;
  startTime: number;
  endTime: number;
  duration: number;
  intensity: 'low' | 'medium' | 'high' | 'max';
  metrics: PerformanceMetric[];
  notes?: string;
  recoveryNeeded: number;
}

export interface MatchEvent {
  id: string;
  matchId: string;
  timestamp: number;
  eventType: string;
  athleteId: string;
  position?: { x: number; y: number };
  value?: number;
  details?: Record<string, unknown>;
}

export interface Match {
  id: string;
  sport: SportType;
  homeTeam: string;
  awayTeam: string;
  date: string;
  venue: string;
  events: MatchEvent[];
  statistics: Record<string, number>;
  result?: { home: number; away: number };
}

export interface BiomechanicsData {
  timestamp: number;
  joints: { name: string; angle: number; velocity: number; acceleration: number }[];
  centerOfMass: { x: number; y: number; z: number };
  groundReactionForce?: { x: number; y: number; z: number };
  balanceScore: number;
}

export interface InjuryRisk {
  athleteId: string;
  bodyPart: string;
  riskLevel: 'low' | 'medium' | 'high' | 'critical';
  probability: number;
  factors: string[];
  recommendations: string[];
  assessmentDate: string;
}

export interface PerformancePrediction {
  athleteId: string;
  metric: string;
  currentValue: number;
  predictedValue: number;
  timeframe: string;
  confidence: number;
  factors: { factor: string; impact: number }[];
}

export interface TeamAnalytics {
  teamId: string;
  sport: SportType;
  players: Athlete[];
  formations: { name: string; positions: { playerId: string; x: number; y: number }[] }[];
  teamStats: Record<string, number>;
  strengths: string[];
  weaknesses: string[];
}

export interface VideoAnalysis {
  videoId: string;
  duration: number;
  frames: { timestamp: number; detections: { type: string; bbox: number[]; confidence: number }[] }[];
  events: MatchEvent[];
  highlights: { timestamp: number; type: string; duration: number }[];
}

export enum CertificationLevel { Bronze = 'bronze', Silver = 'silver', Gold = 'gold' }

export interface ComplianceReport {
  standard: 'WIA-IND-011';
  testDate: string;
  platformId: string;
  certificationLevel: CertificationLevel;
  tests: { name: string; passed: boolean }[];
  compliant: boolean;
}

export type SportsEventType = 'metric-recorded' | 'injury-alert' | 'session-complete' | 'match-event';
export type EventCallback<T = unknown> = (data: T) => void;
