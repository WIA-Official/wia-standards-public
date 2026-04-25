/**
 * WIA-SPACE-023: Space Medicine Standard - TypeScript Types
 * Version: 1.0
 * Philosophy: 弘益人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

export type CrewRole = 'Commander' | 'Pilot' | 'Specialist' | 'CMO' | 'Scientist' | 'Engineer';
export type HealthStatus = 'Normal' | 'Warning' | 'Alert' | 'Critical';
export type ExerciseType = 'Resistive' | 'Aerobic' | 'Mixed';
export type MissionLocation = 'LEO' | 'Lunar' | 'Mars' | 'DeepSpace';

export interface CrewMember {
  id: string;
  name: string;
  mission: string;
  role: CrewRole;
  age?: number;
  mass?: number; // kg
}

export interface VitalSigns {
  heartRate: number; // bpm
  bloodPressure: {
    systolic: number; // mmHg
    diastolic: number; // mmHg
  };
  temperature: number; // °C
  respiratoryRate: number; // breaths/min
  oxygenSaturation: number; // %
}

export interface RadiationExposure {
  cumulative: number; // mSv
  daily: number; // mSv/day
  eyeLens: number; // mSv
  skin: number; // mSv
  bloodForming: number; // mSv
  organSpecific?: Record<string, number>;
}

export interface ExerciseLog {
  duration: number; // minutes
  type: ExerciseType;
  compliance: number; // %
  loadResistive?: number; // kg
  distanceAerobic?: number; // km
  notes?: string;
}

export interface NutritionData {
  calories: number; // kcal
  protein: number; // g
  calcium: number; // mg
  vitaminD: number; // IU
  hydration: number; // L
  supplements?: string[];
}

export interface MedicalRecord {
  medications: string[];
  symptoms: string[];
  diagnoses?: string[];
  treatments?: string[];
  nextExam: string; // ISO8601 timestamp
  examNotes?: string;
}

export interface HealthData {
  crewMember: CrewMember;
  vitals: VitalSigns;
  radiation: RadiationExposure;
  exercise: ExerciseLog;
  nutrition?: NutritionData;
  medical?: MedicalRecord;
  timestamp: string; // ISO8601
  missionDay: number;
  status: HealthStatus;
}

export interface RadiationLimits {
  careerLimit: number; // mSv
  annualLimit: number; // mSv
  eyeLensCareer: number; // mSv
  eyeLensAnnual: number; // mSv
  skinAnnual: number; // mSv
}

export interface ExercisePrescription {
  aerobicMinutes: number;
  resistiveMinutes: number;
  frequency: number; // days per week
  intensity: 'Low' | 'Moderate' | 'High';
  equipment: string[];
}

export interface MissionProfile {
  id: string;
  name: string;
  location: MissionLocation;
  duration: number; // days
  crewSize: number;
  launchDate: string; // ISO8601
  radiationEnvironment: {
    dailyDose: number; // mSv/day
    solarActivity: 'Low' | 'Moderate' | 'High';
  };
}

export interface HealthMonitoringConfig {
  apiKey: string;
  baseURL?: string;
  timeout?: number;
  debug?: boolean;
  alertThresholds?: {
    heartRate?: { min: number; max: number };
    bloodPressure?: { systolic: number; diastolic: number };
    radiationDaily?: number;
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

export interface HealthAlert {
  id: string;
  crewMemberId: string;
  severity: HealthStatus;
  category: 'Vital' | 'Radiation' | 'Exercise' | 'Medical' | 'Psychological';
  message: string;
  timestamp: string;
  acknowledged: boolean;
  resolution?: string;
}

export interface CountermeasureProtocol {
  type: 'Exercise' | 'Nutrition' | 'Radiation' | 'Psychological';
  description: string;
  frequency: string;
  monitoring: string[];
  effectiveness?: number; // %
}
