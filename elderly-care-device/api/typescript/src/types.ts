/**
 * WIA-MED-013 Elderly Care Device Standard - Type Definitions
 * @version 1.0.0
 */

// ============================================================================
// Core Types
// ============================================================================

export type DeviceType = 'wearable' | 'home_sensor' | 'smart_medication' | 'emergency_button' | 'voice_assistant';
export type AlertSeverity = 'low' | 'medium' | 'high' | 'critical';
export type ActivityType = 'walking' | 'sitting' | 'lying' | 'standing' | 'falling' | 'unknown';
export type CognitiveLevel = 'normal' | 'mild_impairment' | 'moderate_impairment' | 'severe_impairment';
export type EmergencyType = 'fall' | 'no_activity' | 'vital_abnormal' | 'sos_button' | 'medication_missed';

// ============================================================================
// Device Data
// ============================================================================

export interface ElderlyDevice {
  id: string;
  userId: string;
  type: DeviceType;
  model: string;
  firmwareVersion: string;

  // Sensors
  sensors: Sensor[];

  // Capabilities
  capabilities: DeviceCapabilities;

  // Status
  batteryLevel: number;
  isOnline: boolean;
  lastSeen: string;

  // Configuration
  config: DeviceConfig;
}

export interface Sensor {
  type: 'accelerometer' | 'gyroscope' | 'heart_rate' | 'temperature' | 'gps' | 'microphone' | 'pir';
  status: 'active' | 'inactive' | 'error';
  lastReading?: SensorReading;
}

export interface SensorReading {
  timestamp: string;
  value: number | number[];
  unit: string;
}

export interface DeviceCapabilities {
  fallDetection: boolean;
  activityMonitoring: boolean;
  vitalSigns: boolean;
  voiceControl: boolean;
  gpsTracking: boolean;
  medicationReminder: boolean;
  socialConnectivity: boolean;
}

export interface DeviceConfig {
  fallDetectionSensitivity: number; // 0-100
  activityCheckInterval: number; // minutes
  emergencyContacts: EmergencyContact[];
  medicationSchedule?: MedicationSchedule[];
  quietHours?: { start: string; end: string };
}

export interface EmergencyContact {
  id: string;
  name: string;
  relationship: string;
  phone: string;
  email?: string;
  priority: number;
}

// ============================================================================
// Fall Detection
// ============================================================================

export interface FallEvent {
  id: string;
  userId: string;
  deviceId: string;

  // Detection
  detectedAt: string;
  confidence: number; // 0-100%
  location?: GeoLocation;

  // Sensor Data
  accelerometerData: number[];
  gyroscopeData: number[];
  impactForce?: number;

  // Response
  alertSent: boolean;
  userResponded: boolean;
  responseTime?: number;
  falseAlarm: boolean;

  // Emergency
  emergencyDispatch?: EmergencyDispatch;
}

export interface GeoLocation {
  latitude: number;
  longitude: number;
  accuracy: number;
  address?: string;
}

export interface EmergencyDispatch {
  id: string;
  dispatchedAt: string;
  arrivedAt?: string;
  type: 'ambulance' | 'family' | 'caregiver';
  status: 'dispatched' | 'enroute' | 'arrived' | 'resolved';
}

// ============================================================================
// Activity Monitoring
// ============================================================================

export interface ActivityData {
  userId: string;
  date: string;

  // Daily Summary
  totalSteps: number;
  activeMinutes: number;
  sedentaryMinutes: number;
  sleepHours: number;

  // Activity Breakdown
  activities: ActivityPeriod[];

  // Vital Signs
  vitals: VitalSigns;

  // AI Insights
  insights?: ActivityInsights;
}

export interface ActivityPeriod {
  startTime: string;
  endTime: string;
  type: ActivityType;
  duration: number; // minutes
  steps?: number;
  calories?: number;
}

export interface VitalSigns {
  heartRate?: {
    average: number;
    min: number;
    max: number;
    readings: HeartRateReading[];
  };
  bloodPressure?: {
    systolic: number;
    diastolic: number;
    timestamp: string;
  };
  oxygenSaturation?: {
    value: number;
    timestamp: string;
  };
  temperature?: {
    value: number;
    timestamp: string;
  };
}

export interface HeartRateReading {
  timestamp: string;
  bpm: number;
}

export interface ActivityInsights {
  normalForUser: boolean;
  anomalies: string[];
  recommendations: string[];
  trendComparison: 'improving' | 'stable' | 'declining';
}

// ============================================================================
// Cognitive Assistance
// ============================================================================

export interface CognitiveAssessment {
  id: string;
  userId: string;
  assessmentDate: string;

  // Scores
  memoryScore: number; // 0-100
  attentionScore: number;
  languageScore: number;
  executiveFunctionScore: number;
  overallScore: number;

  // Classification
  cognitiveLevel: CognitiveLevel;

  // Recommendations
  interventions: string[];
  monitoringFrequency: string;
}

export interface CognitiveReminder {
  id: string;
  userId: string;
  type: 'medication' | 'appointment' | 'task' | 'event';
  title: string;
  description: string;
  scheduledTime: string;
  repeatPattern?: string;

  // Delivery
  deliveryMethod: 'voice' | 'visual' | 'both';
  acknowledged: boolean;
  acknowledgedAt?: string;

  // Escalation
  escalateIfNotAcknowledged: boolean;
  escalationDelay: number; // minutes
}

// ============================================================================
// Medication Management
// ============================================================================

export interface MedicationSchedule {
  id: string;
  medicationName: string;
  dosage: string;
  frequency: string;
  times: string[];

  // Tracking
  adherenceRate: number; // 0-100%
  missedDoses: MissedDose[];

  // Reminders
  reminderEnabled: boolean;
  reminderAdvance: number; // minutes before
}

export interface MissedDose {
  scheduledTime: string;
  missedAt: string;
  notificationSent: boolean;
  caregiverAlerted: boolean;
}

export interface MedicationDispenser {
  id: string;
  deviceId: string;
  compartments: MedicationCompartment[];

  // Status
  isLocked: boolean;
  lastRefilled: string;
  nextRefillDue: string;
}

export interface MedicationCompartment {
  number: number;
  medicationId: string;
  pillCount: number;
  scheduledTime: string;
  dispensed: boolean;
  dispensedAt?: string;
}

// ============================================================================
// Social Connectivity
// ============================================================================

export interface SocialInteraction {
  id: string;
  userId: string;
  type: 'video_call' | 'phone_call' | 'message' | 'visit';
  withPerson: string;
  duration?: number; // minutes
  timestamp: string;

  // Quality
  sentimentScore?: number; // -100 to 100
  engagementLevel?: 'low' | 'medium' | 'high';
}

export interface SocialMetrics {
  userId: string;
  period: string;

  // Interaction Stats
  totalInteractions: number;
  averagePerWeek: number;
  uniqueContacts: number;

  // Loneliness Assessment
  lonelinessScore: number; // 0-100
  isolationRisk: 'low' | 'medium' | 'high';

  // Recommendations
  suggestedActivities: string[];
}

// ============================================================================
// Emergency Alert
// ============================================================================

export interface EmergencyAlert {
  id: string;
  userId: string;
  type: EmergencyType;
  severity: AlertSeverity;

  // Details
  triggeredAt: string;
  location?: GeoLocation;
  vitalSigns?: VitalSigns;

  // Response
  acknowledged: boolean;
  acknowledgedBy?: string;
  acknowledgedAt?: string;
  responseActions: ResponseAction[];

  // Resolution
  resolved: boolean;
  resolvedAt?: string;
  outcome: string;
}

export interface ResponseAction {
  timestamp: string;
  action: string;
  performedBy: string;
  result: string;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

export interface RegisterDeviceRequest {
  userId: string;
  type: DeviceType;
  model: string;
  serialNumber: string;
}

export interface RegisterDeviceResponse {
  deviceId: string;
  activationCode: string;
}

export interface ReportFallRequest {
  deviceId: string;
  userId: string;
  confidence: number;
  accelerometerData: number[];
  gyroscopeData: number[];
  location?: GeoLocation;
}

export interface ReportFallResponse {
  fallEventId: string;
  alertSent: boolean;
  emergencyDispatched: boolean;
}

export interface GetActivityRequest {
  userId: string;
  startDate: string;
  endDate: string;
}

export interface GetActivityResponse {
  activities: ActivityData[];
  summary: {
    avgSteps: number;
    avgActiveMinutes: number;
    avgSleepHours: number;
  };
}

export interface UpdateMedicationRequest {
  userId: string;
  medicationId: string;
  taken: boolean;
  timestamp: string;
}

export interface UpdateMedicationResponse {
  success: boolean;
  adherenceRate: number;
}

// ============================================================================
// SDK Configuration
// ============================================================================

export interface SDKConfig {
  apiKey: string;
  environment: 'production' | 'staging' | 'development';
  baseUrl?: string;
  wsUrl?: string;
  timeout?: number;
}

// ============================================================================
// Error Types
// ============================================================================

export class ElderlyDeviceError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number,
    public details?: any
  ) {
    super(message);
    this.name = 'ElderlyDeviceError';
  }
}

export interface APIErrorResponse {
  error: {
    code: string;
    message: string;
    details?: any;
  };
}
