/**
 * WIA-IND-012: Fitness Wearable Standard - Type Definitions
 * @module wia-ind-012
 */

export enum DeviceType {
  SmartWatch = 'smart_watch', FitnessBand = 'fitness_band',
  SmartRing = 'smart_ring', ChestStrap = 'chest_strap',
  SmartClothing = 'smart_clothing', SmartShoes = 'smart_shoes'
}

export enum WorkoutType {
  Running = 'running', Walking = 'walking', Cycling = 'cycling',
  Swimming = 'swimming', Strength = 'strength', HIIT = 'hiit',
  Yoga = 'yoga', Pilates = 'pilates', Rowing = 'rowing', Elliptical = 'elliptical'
}

export interface DeviceSpec {
  standard: 'WIA-IND-012';
  version: string;
  deviceId: string;
  type: DeviceType;
  sensors: string[];
  batteryLife: number;
  waterResistance: string;
  connectivity: string[];
  displayType?: string;
  gps: boolean;
}

export interface HeartRateData {
  timestamp: number;
  bpm: number;
  zone: 'rest' | 'fat_burn' | 'cardio' | 'peak';
  variability?: number;
}

export interface SleepData {
  date: string;
  totalSleep: number;
  stages: { stage: 'awake' | 'light' | 'deep' | 'rem'; duration: number; percentage: number }[];
  efficiency: number;
  interruptions: number;
  sleepScore: number;
  bedTime: string;
  wakeTime: string;
}

export interface ActivityData {
  date: string;
  steps: number;
  distance: number;
  distanceUnit: 'km' | 'miles';
  calories: number;
  activeMinutes: number;
  floors?: number;
  sedentaryMinutes: number;
}

export interface Workout {
  id: string;
  type: WorkoutType;
  startTime: number;
  endTime: number;
  duration: number;
  calories: number;
  heartRateData: HeartRateData[];
  avgHeartRate: number;
  maxHeartRate: number;
  distance?: number;
  pace?: number;
  elevation?: number;
  laps?: { duration: number; distance: number; pace: number }[];
  gpsRoute?: { lat: number; lon: number; elevation?: number }[];
  performanceScore?: number;
}

export interface StressData {
  timestamp: number;
  level: number;
  category: 'low' | 'medium' | 'high';
  recoveryTime?: number;
}

export interface SpO2Data {
  timestamp: number;
  percentage: number;
  altitude?: number;
}

export interface BodyComposition {
  date: string;
  weight: number;
  bmi: number;
  bodyFat?: number;
  muscleMass?: number;
  boneMass?: number;
  waterPercentage?: number;
  metabolicAge?: number;
}

export interface Goal {
  id: string;
  type: 'steps' | 'calories' | 'distance' | 'active_minutes' | 'sleep' | 'weight';
  target: number;
  current: number;
  unit: string;
  period: 'daily' | 'weekly' | 'monthly';
  startDate: string;
  endDate?: string;
}

export interface HealthInsight {
  type: string;
  message: string;
  severity: 'info' | 'warning' | 'alert';
  recommendation?: string;
  timestamp: number;
}

export enum CertificationLevel { Bronze = 'bronze', Silver = 'silver', Gold = 'gold' }

export interface ComplianceReport {
  standard: 'WIA-IND-012';
  testDate: string;
  deviceId: string;
  certificationLevel: CertificationLevel;
  tests: { name: string; passed: boolean }[];
  compliant: boolean;
}

export type FitnessEventType = 'workout-complete' | 'goal-achieved' | 'health-alert' | 'sync-complete';
export type EventCallback<T = unknown> = (data: T) => void;
