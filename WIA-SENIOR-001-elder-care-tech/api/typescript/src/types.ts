/**
 * WIA-SENIOR-001: Elder Care Technology Standard
 * TypeScript Type Definitions
 *
 * @philosophy 弘益人間 (Benefit All Humanity)
 * @version 1.0.0
 * @license Apache-2.0
 */

/**
 * Core Elder Care Interfaces
 */

export interface ElderProfile {
  id: string;
  personalInfo: PersonalInfo;
  medicalInfo: MedicalInfo;
  carePreferences: CarePreferences;
  emergencyContacts: EmergencyContact[];
  createdAt: Date;
  updatedAt: Date;
}

export interface PersonalInfo {
  firstName: string;
  lastName: string;
  dateOfBirth: Date;
  gender: 'male' | 'female' | 'other' | 'prefer-not-to-say';
  language: string;
  photoUrl?: string;
  address?: Address;
}

export interface Address {
  street: string;
  city: string;
  state: string;
  zipCode: string;
  country: string;
}

export interface MedicalInfo {
  bloodType?: string;
  allergies: string[];
  chronicConditions: string[];
  medications: Medication[];
  primaryPhysician?: HealthcareProvider;
  insuranceInfo?: InsuranceInfo;
}

export interface Medication {
  id: string;
  name: string;
  dosage: string;
  frequency: MedicationFrequency;
  schedule: string[];
  startDate: Date;
  endDate?: Date;
  instructions?: string;
  prescribedBy?: string;
}

export type MedicationFrequency =
  | 'once-daily'
  | 'twice-daily'
  | 'three-times-daily'
  | 'four-times-daily'
  | 'as-needed'
  | 'custom';

export interface HealthcareProvider {
  id: string;
  name: string;
  specialty: string;
  phone: string;
  email?: string;
  address?: Address;
}

export interface InsuranceInfo {
  provider: string;
  policyNumber: string;
  groupNumber?: string;
  expirationDate?: Date;
}

export interface CarePreferences {
  mobilityLevel: MobilityLevel;
  cognitiveLevel: CognitiveLevel;
  communicationPreferences: CommunicationPreference[];
  activityPreferences: string[];
  dietaryRestrictions: string[];
  religiousCulturalPreferences?: string[];
}

export type MobilityLevel =
  | 'independent'
  | 'needs-assistance'
  | 'walker'
  | 'wheelchair'
  | 'bedridden';

export type CognitiveLevel =
  | 'normal'
  | 'mild-impairment'
  | 'moderate-impairment'
  | 'severe-impairment'
  | 'dementia';

export interface CommunicationPreference {
  type: 'voice' | 'text' | 'video' | 'in-person';
  preferred: boolean;
}

export interface EmergencyContact {
  id: string;
  name: string;
  relationship: string;
  phone: string;
  email?: string;
  isPrimary: boolean;
  canMakeMedicalDecisions: boolean;
}

/**
 * Vital Signs Monitoring
 */

export interface VitalSigns {
  timestamp: Date;
  elderId: string;
  heartRate?: HeartRate;
  bloodPressure?: BloodPressure;
  temperature?: Temperature;
  oxygenSaturation?: OxygenSaturation;
  respiratoryRate?: RespiratoryRate;
  weight?: Weight;
  bloodGlucose?: BloodGlucose;
}

export interface HeartRate {
  bpm: number;
  rhythm: 'regular' | 'irregular';
  isAbnormal: boolean;
}

export interface BloodPressure {
  systolic: number;
  diastolic: number;
  unit: 'mmHg';
  isAbnormal: boolean;
}

export interface Temperature {
  value: number;
  unit: 'celsius' | 'fahrenheit';
  location: 'oral' | 'temporal' | 'axillary' | 'tympanic';
  isAbnormal: boolean;
}

export interface OxygenSaturation {
  percentage: number;
  isAbnormal: boolean;
}

export interface RespiratoryRate {
  breathsPerMinute: number;
  isAbnormal: boolean;
}

export interface Weight {
  value: number;
  unit: 'kg' | 'lbs';
}

export interface BloodGlucose {
  value: number;
  unit: 'mg/dL' | 'mmol/L';
  measurementContext: 'fasting' | 'post-meal' | 'random';
  isAbnormal: boolean;
}

/**
 * Alert System
 */

export interface Alert {
  id: string;
  type: AlertType;
  severity: AlertSeverity;
  elderId: string;
  title: string;
  message: string;
  timestamp: Date;
  data?: any;
  resolved: boolean;
  resolvedAt?: Date;
  resolvedBy?: string;
  actions?: AlertAction[];
}

export type AlertType =
  | 'VITAL_SIGN_ABNORMAL'
  | 'FALL_DETECTED'
  | 'MEDICATION_MISSED'
  | 'EMERGENCY_BUTTON'
  | 'UNUSUAL_ACTIVITY'
  | 'DEVICE_OFFLINE'
  | 'LOW_BATTERY'
  | 'GEOFENCE_BREACH'
  | 'TEMPERATURE_EXTREME'
  | 'CUSTOM';

export type AlertSeverity = 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';

export interface AlertAction {
  id: string;
  label: string;
  actionType: string;
  data?: any;
}

/**
 * Activity Monitoring
 */

export interface Activity {
  id: string;
  elderId: string;
  type: ActivityType;
  startTime: Date;
  endTime?: Date;
  duration?: number;
  location?: string;
  notes?: string;
  completedBy?: string;
}

export type ActivityType =
  | 'SLEEP'
  | 'MEAL'
  | 'MEDICATION'
  | 'EXERCISE'
  | 'SOCIAL'
  | 'BATHROOM'
  | 'SHOWER'
  | 'ENTERTAINMENT'
  | 'MEDICAL_APPOINTMENT'
  | 'THERAPY'
  | 'OTHER';

/**
 * Caregiver Management
 */

export interface Caregiver {
  id: string;
  personalInfo: PersonalInfo;
  role: CaregiverRole;
  certifications: Certification[];
  availability: Availability[];
  assignedElders: string[];
  rating?: number;
  reviewCount?: number;
}

export type CaregiverRole =
  | 'FAMILY'
  | 'PROFESSIONAL'
  | 'NURSE'
  | 'DOCTOR'
  | 'THERAPIST'
  | 'VOLUNTEER';

export interface Certification {
  name: string;
  issuedBy: string;
  issuedDate: Date;
  expirationDate?: Date;
  certificateUrl?: string;
}

export interface Availability {
  dayOfWeek: number;
  startTime: string;
  endTime: string;
}

/**
 * Care Plan
 */

export interface CarePlan {
  id: string;
  elderId: string;
  createdBy: string;
  createdAt: Date;
  updatedAt: Date;
  goals: CareGoal[];
  tasks: CareTask[];
  medications: Medication[];
  appointments: Appointment[];
  notes?: string;
}

export interface CareGoal {
  id: string;
  description: string;
  targetDate?: Date;
  completed: boolean;
  completedDate?: Date;
}

export interface CareTask {
  id: string;
  title: string;
  description?: string;
  frequency: TaskFrequency;
  assignedTo?: string;
  dueDate?: Date;
  completed: boolean;
  completedDate?: Date;
  completedBy?: string;
}

export type TaskFrequency =
  | 'hourly'
  | 'daily'
  | 'twice-daily'
  | 'three-times-daily'
  | 'weekly'
  | 'monthly'
  | 'as-needed';

export interface Appointment {
  id: string;
  type: AppointmentType;
  provider: HealthcareProvider;
  scheduledTime: Date;
  duration: number;
  location: string;
  notes?: string;
  reminder?: boolean;
  reminderTime?: number;
}

export type AppointmentType =
  | 'CHECKUP'
  | 'SPECIALIST'
  | 'THERAPY'
  | 'DENTAL'
  | 'VISION'
  | 'LABORATORY'
  | 'IMAGING'
  | 'PROCEDURE'
  | 'OTHER';

/**
 * SDK Configuration
 */

export interface ElderCareConfig {
  apiKey: string;
  apiUrl?: string;
  patientId?: string;
  caregiverId?: string;
  enableRealTimeMonitoring?: boolean;
  alertThresholds?: AlertThresholds;
  locale?: string;
  timezone?: string;
}

export interface AlertThresholds {
  heartRate?: { min: number; max: number };
  bloodPressure?: { systolicMax: number; diastolicMax: number };
  temperature?: { min: number; max: number };
  oxygenSaturation?: { min: number };
  respiratoryRate?: { min: number; max: number };
}

/**
 * API Response Types
 */

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: ApiError;
  timestamp: Date;
}

export interface ApiError {
  code: string;
  message: string;
  details?: any;
}

/**
 * Event Types
 */

export interface VitalSignsEvent {
  type: 'vitals';
  data: VitalSigns;
}

export interface AlertEvent {
  type: 'alert';
  data: Alert;
}

export interface ActivityEvent {
  type: 'activity';
  data: Activity;
}

export type ElderCareEvent = VitalSignsEvent | AlertEvent | ActivityEvent;
