/**
 * WIA-IND-014: Virtual Fitness Standard - Type Definitions
 * @module wia-ind-014
 */

export enum PlatformType {
  VR = 'vr', AR = 'ar', MR = 'mr', Mobile = 'mobile', Desktop = 'desktop'
}

export enum WorkoutCategory {
  Cardio = 'cardio', Strength = 'strength', HIIT = 'hiit',
  Boxing = 'boxing', Dance = 'dance', Yoga = 'yoga',
  Meditation = 'meditation', Sports = 'sports', Gaming = 'gaming'
}

export enum Difficulty {
  Beginner = 'beginner', Intermediate = 'intermediate',
  Advanced = 'advanced', Expert = 'expert'
}

export interface VRHeadset {
  id: string;
  model: string;
  manufacturer: string;
  resolution: { width: number; height: number };
  refreshRate: number;
  fieldOfView: number;
  tracking: string;
  controllers: boolean;
  handTracking: boolean;
}

export interface MotionData {
  timestamp: number;
  headPosition: { x: number; y: number; z: number };
  headRotation: { x: number; y: number; z: number; w: number };
  leftHand?: { position: { x: number; y: number; z: number }; rotation: { x: number; y: number; z: number; w: number } };
  rightHand?: { position: { x: number; y: number; z: number }; rotation: { x: number; y: number; z: number; w: number } };
  velocity?: number;
  acceleration?: number;
}

export interface VirtualWorkout {
  id: string;
  name: string;
  category: WorkoutCategory;
  difficulty: Difficulty;
  duration: number;
  calories: number;
  environment: string;
  musicTrack?: string;
  instructor?: string;
  equipment?: string[];
  movements: Movement[];
}

export interface Movement {
  id: string;
  name: string;
  type: 'punch' | 'kick' | 'squat' | 'lunge' | 'jump' | 'block' | 'pose' | 'step';
  targetPosition: { x: number; y: number; z: number };
  timing: number;
  duration: number;
  points: number;
  accuracy?: number;
}

export interface WorkoutSession {
  id: string;
  workoutId: string;
  userId: string;
  startTime: number;
  endTime?: number;
  duration: number;
  caloriesBurned: number;
  score: number;
  accuracy: number;
  movementsCompleted: number;
  totalMovements: number;
  heartRateData?: { timestamp: number; bpm: number }[];
  motionData?: MotionData[];
}

export interface Leaderboard {
  workoutId: string;
  entries: { userId: string; username: string; score: number; accuracy: number; date: string }[];
  period: 'daily' | 'weekly' | 'monthly' | 'alltime';
}

export interface Achievement {
  id: string;
  name: string;
  description: string;
  icon: string;
  category: string;
  requirement: { type: string; value: number };
  reward?: { type: string; value: number };
  unlockedAt?: number;
}

export interface VirtualEnvironment {
  id: string;
  name: string;
  theme: string;
  skybox: string;
  lighting: { ambient: number; directional: number };
  props: { id: string; position: { x: number; y: number; z: number }; scale: number }[];
  audio?: { ambient: string; music?: string };
}

export interface SafetyBoundary {
  type: 'rectangle' | 'circle' | 'custom';
  dimensions: { width: number; depth: number; height: number };
  warningDistance: number;
  visualIndicator: boolean;
}

export enum CertificationLevel { Bronze = 'bronze', Silver = 'silver', Gold = 'gold' }

export interface ComplianceReport {
  standard: 'WIA-IND-014';
  testDate: string;
  platformId: string;
  certificationLevel: CertificationLevel;
  tests: { name: string; passed: boolean }[];
  compliant: boolean;
}

export type VirtualFitnessEventType = 'session-start' | 'session-end' | 'movement-hit' | 'achievement-unlocked' | 'boundary-warning';
export type EventCallback<T = unknown> = (data: T) => void;
