/**
 * WIA-PET-007 Pet Wearable Standard - TypeScript Type Definitions
 * @version 1.0.0
 * @standard WIA-PET-007
 */

// ============================================================================
// Core Types
// ============================================================================

export type Species = 'dog' | 'cat' | 'rabbit' | 'ferret' | 'bird' | 'other';
export type Gender = 'male' | 'female' | 'unknown';
export type ActivityType = 'walking' | 'running' | 'playing' | 'resting' | 'sleeping';
export type VitalSignStatus = 'normal' | 'elevated' | 'low';
export type QualityScore = 'good' | 'fair' | 'poor';
export type AlertSeverity = 'critical' | 'high' | 'medium' | 'low';
export type Unit = 'kg' | 'lb' | 'km' | 'mi' | 'celsius' | 'fahrenheit';

// ============================================================================
// Pet Profile
// ============================================================================

export interface PetProfile {
  petId: string;
  name: string;
  species: Species;
  breed: string;
  birthDate: string; // ISO 8601 date
  gender: Gender;
  neutered: boolean;
  weight: Measurement;
  microchipId?: string;
  owner: OwnerInfo;
  veterinarian?: VeterinarianInfo;
  medicalHistory?: MedicalCondition[];
  allergies?: string[];
  emergencyContacts?: EmergencyContact[];
  insurancePolicy?: InsuranceInfo;
  createdAt: string; // ISO 8601 timestamp
  updatedAt: string; // ISO 8601 timestamp
  version: string;
  standard: string;
}

export interface Measurement {
  value: number;
  unit: Unit;
  measuredAt: string; // ISO 8601 timestamp
}

export interface OwnerInfo {
  ownerId: string;
  name: string;
  contactEmail: string;
  contactPhone: string;
}

export interface VeterinarianInfo {
  clinicId: string;
  clinicName: string;
  primaryVetName: string;
  contactPhone: string;
}

export interface MedicalCondition {
  condition: string;
  diagnosedDate: string;
  status: string;
  medications?: string[];
}

export interface EmergencyContact {
  name: string;
  relationship: string;
  phone: string;
}

export interface InsuranceInfo {
  provider: string;
  policyNumber: string;
  coverageLevel: string;
}

// ============================================================================
// Activity Data
// ============================================================================

export interface ActivityData {
  deviceId: string;
  petId: string;
  timestamp: string; // ISO 8601 timestamp
  activityType: ActivityType;
  intensity: number; // 1-10
  duration: number; // minutes
  metrics: ActivityMetrics;
  heartRate?: HeartRateData;
  location?: LocationData;
  temperature?: TemperatureData;
}

export interface ActivityMetrics {
  steps: number;
  distance: Measurement;
  caloriesBurned: number;
  averageSpeed: Measurement;
  elevation?: ElevationData;
}

export interface ElevationData {
  gain: number;
  loss: number;
  unit: string;
}

export interface DailyActivitySummary {
  petId: string;
  date: string; // ISO 8601 date
  activities: ActivitySummary[];
  dailyTotals: DailyTotals;
  goals?: GoalStatus;
  comparison?: ComparisonData;
}

export interface ActivitySummary {
  type: ActivityType;
  totalDuration: number;
  totalDistance?: number;
  totalCalories?: number;
  sessions: number;
}

export interface DailyTotals {
  steps: number;
  distance: number;
  calories: number;
  activeMinutes: number;
  restMinutes: number;
}

export interface GoalStatus {
  stepsGoal: number;
  stepsAchieved: boolean;
  activeMinutesGoal: number;
  activeMinutesAchieved: boolean;
}

export interface ComparisonData {
  vs7DayAverage?: Record<string, string>;
  vsBreedAverage?: Record<string, string>;
}

// ============================================================================
// Health Metrics
// ============================================================================

export interface HealthMetrics {
  petId: string;
  timestamp: string; // ISO 8601 timestamp
  vitalSigns: VitalSigns;
  bodyMetrics?: BodyMetrics;
  context?: HealthContext;
  alerts?: HealthAlert[];
  deviceId: string;
}

export interface VitalSigns {
  heartRate?: VitalSign;
  respiratoryRate?: VitalSign;
  temperature?: VitalSign;
  oxygenSaturation?: VitalSign;
}

export interface VitalSign {
  value: number;
  unit: string;
  measurement?: string;
  quality?: QualityScore;
  normalRange?: { min: number; max: number };
  status: VitalSignStatus;
}

export interface BodyMetrics {
  weight?: WeightData;
  bodyConditionScore?: BodyConditionData;
}

export interface WeightData {
  value: number;
  unit: Unit;
  trend: string;
  changeFromLastWeek?: number;
}

export interface BodyConditionData {
  value: number;
  scale: string;
  assessment: string;
}

export interface HealthContext {
  activity: string;
  location: string;
  timeOfDay: string;
}

export interface HealthAlert {
  type: string;
  severity: AlertSeverity;
  message: string;
  timestamp: string;
}

// ============================================================================
// Heart Rate Data
// ============================================================================

export interface HeartRateData {
  average: number;
  minimum: number;
  maximum: number;
  variability?: number;
}

export interface TemperatureData {
  ambient?: number;
  body?: number;
  unit: 'celsius' | 'fahrenheit';
}

// ============================================================================
// Location Data
// ============================================================================

export interface LocationData {
  latitude: number;
  longitude: number;
  accuracy: number;
  altitude?: number;
  heading?: number;
  speed?: number;
}

export interface LocationUpdate {
  petId: string;
  deviceId: string;
  timestamp: string;
  location: LocationData;
  safeZones?: SafeZoneStatus[];
  alerts?: LocationAlert[];
  battery?: BatteryStatus;
}

export interface SafeZoneStatus {
  zoneId: string;
  name: string;
  center: { latitude: number; longitude: number };
  radius: number;
  status: 'inside' | 'outside';
}

export interface LocationAlert {
  type: string;
  severity: AlertSeverity;
  message: string;
}

export interface BatteryStatus {
  level: number; // percentage
  estimatedHoursRemaining: number;
}

// ============================================================================
// Geofencing
// ============================================================================

export interface Geofence {
  geofenceId: string;
  name: string;
  type: 'circle' | 'polygon';
  center?: { latitude: number; longitude: number };
  radius?: number;
  polygon?: { latitude: number; longitude: number }[];
  unit: string;
  enabled: boolean;
  alertOnExit: boolean;
  alertOnEntry: boolean;
  schedule?: GeofenceSchedule;
  notifications?: NotificationSettings;
}

export interface GeofenceSchedule {
  allDay: boolean;
  startTime?: string;
  endTime?: string;
  days?: string[];
}

export interface NotificationSettings {
  push: boolean;
  sms: boolean;
  email: boolean;
}

// ============================================================================
// Sleep Data
// ============================================================================

export interface SleepData {
  petId: string;
  date: string;
  sleepSessions: SleepSession[];
  dailySummary: SleepSummary;
  comparison?: ComparisonData;
}

export interface SleepSession {
  sessionId: string;
  startTime: string;
  endTime: string;
  totalDuration: number; // minutes
  stages: SleepStage[];
  quality: SleepQuality;
  averageHeartRate: number;
  averageRespiratoryRate: number;
  movements: number;
  environment?: SleepEnvironment;
}

export interface SleepStage {
  stage: 'light' | 'deep' | 'REM';
  duration: number;
  percentage: number;
}

export interface SleepQuality {
  score: number; // 0-100
  assessment: string;
  disruptions: number;
  restlessness: string;
}

export interface SleepEnvironment {
  temperature: number;
  noise: string;
  location: string;
}

export interface SleepSummary {
  totalSleepTime: number;
  sleepEfficiency: number;
  deepSleepPercentage: number;
  REMPercentage: number;
  wakings: number;
  timeToFallAsleep: number;
}

// ============================================================================
// Behavioral Data
// ============================================================================

export interface BehaviorData {
  petId: string;
  date: string;
  behaviors: BehaviorPattern[];
  emotionalState: EmotionalState;
  anomalies: string[];
  recommendations: string[];
}

export interface BehaviorPattern {
  type: string;
  events: number;
  totalDuration?: number;
  triggers?: string[];
  locations?: string[];
  intensity: 'low' | 'moderate' | 'high';
}

export interface EmotionalState {
  anxiety: 'low' | 'moderate' | 'high';
  excitement?: 'low' | 'moderate' | 'high';
  stress: 'low' | 'moderate' | 'high';
  overall: string;
}

// ============================================================================
// API Client Configuration
// ============================================================================

export interface ClientConfig {
  apiKey?: string;
  baseURL: string;
  timeout?: number;
  retryAttempts?: number;
}

export interface AuthTokens {
  accessToken: string;
  refreshToken?: string;
  expiresAt: number;
}

// ============================================================================
// Error Types
// ============================================================================

export class WIAPetWearableError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number
  ) {
    super(message);
    this.name = 'WIAPetWearableError';
  }
}
