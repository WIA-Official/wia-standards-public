/**
 * WIA-MED-022: Medication Adherence Standard - TypeScript Types
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
export type MedicationID = string;

// ============================================================================
// Enums
// ============================================================================

export enum AdherenceStatus {
  TAKEN = 'taken',
  MISSED = 'missed',
  LATE = 'late',
  SKIPPED = 'skipped',
  PENDING = 'pending',
}

export enum ReminderType {
  PUSH = 'push',
  SMS = 'sms',
  EMAIL = 'email',
  ALARM = 'alarm',
}

export enum DeviceType {
  SMART_PILL_BOTTLE = 'smart_pill_bottle',
  PILL_DISPENSER = 'pill_dispenser',
  SMART_BLISTER = 'smart_blister',
  WEARABLE = 'wearable',
  MOBILE_APP = 'mobile_app',
}

// ============================================================================
// Medication Types
// ============================================================================

export interface Medication {
  medicationId: MedicationID;
  name: string;
  dosage: string;
  dosageUnit: string;
  frequency: string;
  instructions?: string;
  startDate: Timestamp;
  endDate?: Timestamp;
  prescribedBy: string;
  refillsRemaining?: number;
}

export interface MedicationSchedule {
  scheduleId: string;
  patientId: PatientID;
  medicationId: MedicationID;
  times: ScheduledTime[];
  reminderEnabled: boolean;
  reminderTypes: ReminderType[];
}

export interface ScheduledTime {
  hour: number;
  minute: number;
  daysOfWeek: number[];
}

// ============================================================================
// Adherence Types
// ============================================================================

export interface AdherenceRecord {
  recordId: string;
  patientId: PatientID;
  medicationId: MedicationID;
  scheduledTime: Timestamp;
  actualTime?: Timestamp;
  status: AdherenceStatus;
  source: DeviceType;
  notes?: string;
}

export interface AdherenceMetrics {
  patientId: PatientID;
  medicationId?: MedicationID;
  period: string;
  adherenceRate: number;
  totalDoses: number;
  takenOnTime: number;
  takenLate: number;
  missed: number;
  streak: number;
}

// ============================================================================
// Device Types
// ============================================================================

export interface MonitoringDevice {
  deviceId: DeviceID;
  patientId: PatientID;
  type: DeviceType;
  manufacturer: string;
  model: string;
  firmwareVersion?: string;
  batteryLevel?: number;
  lastSync: Timestamp;
  status: 'active' | 'inactive' | 'error';
}

// ============================================================================
// Alert Types
// ============================================================================

export interface AdherenceAlert {
  alertId: string;
  patientId: PatientID;
  medicationId: MedicationID;
  type: 'missed_dose' | 'low_adherence' | 'refill_needed';
  severity: 'low' | 'medium' | 'high';
  message: string;
  createdAt: Timestamp;
  acknowledged: boolean;
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
  status: number;
  success: boolean;
  data?: T;
  timestamp: Timestamp;
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
