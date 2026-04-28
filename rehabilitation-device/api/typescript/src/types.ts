/**
 * WIA-MED-028: Rehabilitation Device Standard - TypeScript Types
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Type Aliases
// ============================================================================

export type Timestamp = string;
export type DeviceID = string;
export type PatientID = string;
export type SessionID = string;

// ============================================================================
// Enums
// ============================================================================

export enum ExerciseType {
  STRENGTH = 'strength',
  FLEXIBILITY = 'flexibility',
  BALANCE = 'balance',
  ENDURANCE = 'endurance',
  COORDINATION = 'coordination',
  GAIT = 'gait',
}

export enum DeviceType {
  EXOSKELETON = 'exoskeleton',
  TREADMILL = 'treadmill',
  STATIONARY_BIKE = 'stationary_bike',
  ARM_TRAINER = 'arm_trainer',
  LEG_TRAINER = 'leg_trainer',
  BALANCE_PLATFORM = 'balance_platform',
  HAND_GRIPPER = 'hand_gripper',
}

export enum DeviceStatus {
  ACTIVE = 'active',
  IDLE = 'idle',
  MAINTENANCE = 'maintenance',
  CALIBRATING = 'calibrating',
  ERROR = 'error',
}

export enum ProgramPhase {
  WARMUP = 'warmup',
  ACTIVE = 'active',
  COOLDOWN = 'cooldown',
  REST = 'rest',
}

export enum RecoveryStage {
  ACUTE = 'acute',
  SUBACUTE = 'subacute',
  CHRONIC = 'chronic',
  MAINTENANCE = 'maintenance',
}

// ============================================================================
// Device Types
// ============================================================================

export interface RehabDevice {
  deviceId: DeviceID;
  type: DeviceType;
  manufacturer: string;
  model: string;
  serialNumber: string;
  status: DeviceStatus;
  firmwareVersion: string;
  lastCalibration: Timestamp;
  settings: DeviceSettings;
  capabilities: DeviceCapabilities;
}

export interface DeviceSettings {
  resistance: number;
  speed?: number;
  assistanceLevel: number;
  maxDuration: number;
  safetyLimits: SafetyLimits;
}

export interface SafetyLimits {
  maxHeartRate: number;
  maxResistance: number;
  maxSpeed: number;
  emergencyStopEnabled: boolean;
}

export interface DeviceCapabilities {
  biofeedback: boolean;
  gamification: boolean;
  virtualReality: boolean;
  forceControl: boolean;
  rangeOfMotionTracking: boolean;
}

// ============================================================================
// Session Types
// ============================================================================

export interface ExerciseSession {
  sessionId: SessionID;
  patientId: PatientID;
  deviceId: DeviceID;
  type: ExerciseType;
  phase: ProgramPhase;
  startTime: Timestamp;
  endTime?: Timestamp;
  metrics: SessionMetrics;
  notes?: string;
}

export interface SessionMetrics {
  durationSeconds: number;
  repetitions?: number;
  distanceMeters?: number;
  caloriesBurned: number;
  heartRate: HeartRateData;
  rangeOfMotion?: RangeOfMotion;
  performanceScore: number;
}

export interface HeartRateData {
  average: number;
  max: number;
  min: number;
  restingHR?: number;
}

export interface RangeOfMotion {
  joint: string;
  startAngle: number;
  endAngle: number;
  improvement: number;
}

// ============================================================================
// Program Types
// ============================================================================

export interface RehabProgram {
  programId: string;
  patientId: PatientID;
  therapistId: string;
  diagnosis: string;
  recoveryStage: RecoveryStage;
  goals: RehabGoal[];
  exercises: ProgramExercise[];
  startDate: Timestamp;
  endDate?: Timestamp;
  progress: number;
}

export interface RehabGoal {
  goalId: string;
  description: string;
  metric: string;
  targetValue: number;
  currentValue: number;
  achieved: boolean;
  achievedAt?: Timestamp;
}

export interface ProgramExercise {
  exerciseId: string;
  type: ExerciseType;
  deviceType: DeviceType;
  frequency: string;
  durationMinutes: number;
  completedSessions: number;
  targetSessions: number;
  instructions: string;
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
  success: boolean;
  data?: T;
  error?: APIError;
  timestamp: Timestamp;
}

export interface APIError {
  code: string;
  message: string;
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
