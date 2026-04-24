/**
 * WIA Carebot Standard - Type Definitions
 * @packageDocumentation
 * @module wia-carebot
 */

export enum CarebotType {
  Companion = 'companion',
  Medical = 'medical',
  Mobility = 'mobility',
  Therapy = 'therapy',
  Monitoring = 'monitoring',
  Rehabilitation = 'rehabilitation',
  Telepresence = 'telepresence'
}

export enum CareLevel {
  Independent = 'independent',
  LightAssist = 'light_assist',
  ModerateAssist = 'moderate_assist',
  FullAssist = 'full_assist',
  Critical = 'critical'
}

export enum TaskCategory {
  Medication = 'medication',
  VitalSigns = 'vital_signs',
  Mobility = 'mobility',
  Companionship = 'companionship',
  Emergency = 'emergency',
  DailyLiving = 'daily_living',
  Exercise = 'exercise',
  Entertainment = 'entertainment'
}

export enum AlertPriority {
  Low = 'low',
  Medium = 'medium',
  High = 'high',
  Critical = 'critical'
}

export interface CarebotInfo {
  id: string;
  name: string;
  type: CarebotType;
  model: string;
  manufacturer: string;
  serialNumber: string;
  capabilities: Capability[];
  sensors: Sensor[];
  firmwareVersion: string;
  location?: RoomLocation;
}

export interface Capability {
  name: string;
  type: TaskCategory;
  description: string;
  requiresHumanSupervision: boolean;
}

export interface Sensor {
  id: string;
  type: 'camera' | 'microphone' | 'temperature' | 'proximity' | 'pressure' | 'heart_rate' | 'spo2' | 'motion';
  name: string;
  active: boolean;
  lastReading?: SensorReading;
}

export interface SensorReading {
  sensorId: string;
  value: number | string | boolean;
  unit?: string;
  timestamp: Date;
  quality: number;
}

export interface RoomLocation {
  building?: string;
  floor?: string;
  room: string;
  zone?: string;
  coordinates?: { x: number; y: number };
}

export interface Patient {
  id: string;
  name: string;
  dateOfBirth: Date;
  careLevel: CareLevel;
  conditions: string[];
  allergies: string[];
  medications: Medication[];
  emergencyContacts: EmergencyContact[];
  carePreferences: CarePreferences;
  assignedCarebot?: string;
}

export interface Medication {
  id: string;
  name: string;
  dosage: string;
  frequency: string;
  times: string[];
  instructions?: string;
  startDate: Date;
  endDate?: Date;
  reminders: boolean;
}

export interface EmergencyContact {
  name: string;
  relationship: string;
  phone: string;
  email?: string;
  isPrimary: boolean;
}

export interface CarePreferences {
  language: string;
  voiceSpeed: number;
  volumeLevel: number;
  wakeWord?: string;
  privacyMode: boolean;
  nightMode: boolean;
  companionshipLevel: 'minimal' | 'moderate' | 'high';
}

export interface VitalSigns {
  patientId: string;
  heartRate?: number;
  bloodPressure?: { systolic: number; diastolic: number };
  temperature?: number;
  spo2?: number;
  respiratoryRate?: number;
  weight?: number;
  bloodGlucose?: number;
  timestamp: Date;
  status: 'normal' | 'warning' | 'critical';
}

export interface CareTask {
  id: string;
  patientId: string;
  carebotId: string;
  category: TaskCategory;
  name: string;
  description: string;
  scheduledTime?: Date;
  recurring?: RecurrenceRule;
  status: 'pending' | 'in_progress' | 'completed' | 'failed' | 'skipped';
  priority: AlertPriority;
  requiredCapabilities: string[];
  result?: TaskResult;
}

export interface RecurrenceRule {
  frequency: 'daily' | 'weekly' | 'monthly';
  interval: number;
  days?: number[];
  times: string[];
  endDate?: Date;
}

export interface TaskResult {
  success: boolean;
  completedAt: Date;
  notes?: string;
  measurements?: Record<string, unknown>;
}

export interface CareAlert {
  id: string;
  patientId: string;
  carebotId?: string;
  type: 'fall' | 'vital_sign' | 'medication' | 'emergency' | 'behavioral' | 'system';
  priority: AlertPriority;
  message: string;
  data?: Record<string, unknown>;
  timestamp: Date;
  acknowledgedAt?: Date;
  acknowledgedBy?: string;
  resolvedAt?: Date;
  actions: AlertAction[];
}

export interface AlertAction {
  type: 'notify' | 'call' | 'dispatch' | 'escalate';
  target: string;
  status: 'pending' | 'sent' | 'acknowledged' | 'failed';
  timestamp?: Date;
}

export interface Interaction {
  id: string;
  patientId: string;
  carebotId: string;
  type: 'voice' | 'touch' | 'gesture' | 'button';
  intent?: string;
  transcript?: string;
  response?: string;
  duration: number;
  satisfaction?: number;
  timestamp: Date;
}

export interface DailyReport {
  patientId: string;
  date: Date;
  vitalsSummary: VitalsSummary;
  medicationCompliance: number;
  activityLevel: number;
  moodAssessment?: string;
  sleepQuality?: number;
  tasks: CareTask[];
  interactions: Interaction[];
  alerts: CareAlert[];
}

export interface VitalsSummary {
  readings: number;
  avgHeartRate?: number;
  avgBloodPressure?: { systolic: number; diastolic: number };
  avgSpo2?: number;
  anomalies: number;
}

export interface CarebotConfig {
  apiEndpoint: string;
  apiKey?: string;
  enableVoice: boolean;
  enableFallDetection: boolean;
  enableVitalsMonitoring: boolean;
  alertThresholds: AlertThresholds;
}

export interface AlertThresholds {
  heartRateLow: number;
  heartRateHigh: number;
  spo2Low: number;
  temperatureHigh: number;
  bloodPressureHigh: { systolic: number; diastolic: number };
}

export enum CertificationLevel {
  Bronze = 'bronze',
  Silver = 'silver',
  Gold = 'gold'
}

export interface ComplianceReport {
  standard: 'WIA-CAREBOT';
  testDate: string;
  config: CarebotConfig;
  targetLevel: CertificationLevel;
  tests: TestResult[];
  passed: boolean;
  achievedLevel?: CertificationLevel;
}

export interface TestResult {
  testName: string;
  passed: boolean;
  notes?: string;
}
