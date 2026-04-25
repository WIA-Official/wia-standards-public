/**
 * WIA-MED-027: Remote Patient Monitoring Standard - TypeScript Types
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
export type PatientID = string;
export type DeviceID = string;
export type AlertID = string;

// ============================================================================
// Enums
// ============================================================================

export enum MonitoringType {
  CONTINUOUS = 'continuous',
  PERIODIC = 'periodic',
  EVENT_BASED = 'event_based',
  ON_DEMAND = 'on_demand',
}

export enum AlertPriority {
  LOW = 'low',
  MEDIUM = 'medium',
  HIGH = 'high',
  CRITICAL = 'critical',
}

export enum AlertStatus {
  ACTIVE = 'active',
  ACKNOWLEDGED = 'acknowledged',
  RESOLVED = 'resolved',
  ESCALATED = 'escalated',
}

export enum VitalType {
  HEART_RATE = 'heart_rate',
  BLOOD_PRESSURE = 'blood_pressure',
  OXYGEN_SATURATION = 'oxygen_saturation',
  TEMPERATURE = 'temperature',
  RESPIRATORY_RATE = 'respiratory_rate',
  BLOOD_GLUCOSE = 'blood_glucose',
  WEIGHT = 'weight',
}

export enum DeviceStatus {
  ONLINE = 'online',
  OFFLINE = 'offline',
  LOW_BATTERY = 'low_battery',
  ERROR = 'error',
}

// ============================================================================
// Vital Signs Types
// ============================================================================

export interface VitalSign {
  type: VitalType;
  value: number;
  unit: string;
  timestamp: Timestamp;
  deviceId: DeviceID;
  quality?: number;
}

export interface BloodPressure {
  systolic: number;
  diastolic: number;
  pulse?: number;
  unit: 'mmHg';
  timestamp: Timestamp;
}

export interface VitalSignsSnapshot {
  patientId: PatientID;
  timestamp: Timestamp;
  heartRate?: VitalSign;
  bloodPressure?: BloodPressure;
  oxygenSaturation?: VitalSign;
  temperature?: VitalSign;
  respiratoryRate?: VitalSign;
  bloodGlucose?: VitalSign;
}

// ============================================================================
// Monitoring Types
// ============================================================================

export interface PatientMonitoringData {
  format: 'WIA-REMOTE-PATIENT-MONITORING-v1.0';
  timestamp: Timestamp;
  patientId: PatientID;
  monitoringId: string;
  monitoringType: MonitoringType;
  vitalSigns: VitalSignsSnapshot;
  alerts: MonitoringAlert[];
  devices: MonitoringDevice[];
}

export interface MonitoringPlan {
  planId: string;
  patientId: PatientID;
  careProviderId: string;
  monitoringType: MonitoringType;
  frequencyHours: number;
  startDate: Timestamp;
  endDate?: Timestamp;
  parametersToMonitor: VitalType[];
  alertThresholds: AlertThreshold[];
  status: 'active' | 'paused' | 'completed';
}

export interface AlertThreshold {
  vitalType: VitalType;
  minValue?: number;
  maxValue?: number;
  priority: AlertPriority;
}

// ============================================================================
// Alert Types
// ============================================================================

export interface MonitoringAlert {
  alertId: AlertID;
  patientId: PatientID;
  priority: AlertPriority;
  status: AlertStatus;
  vitalType: VitalType;
  value: number;
  threshold: AlertThreshold;
  message: string;
  createdAt: Timestamp;
  acknowledgedAt?: Timestamp;
  acknowledgedBy?: string;
  resolvedAt?: Timestamp;
}

// ============================================================================
// Device Types
// ============================================================================

export interface MonitoringDevice {
  deviceId: DeviceID;
  patientId: PatientID;
  type: string;
  manufacturer: string;
  model: string;
  status: DeviceStatus;
  batteryLevel?: number;
  lastSync: Timestamp;
  firmwareVersion?: string;
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
