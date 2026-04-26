/**
 * WIA-MED-025: Emergency Medical Data Monitoring Standard - TypeScript Type Definitions
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Basic Types
// ============================================================================

export type Timestamp = string;
export type DeviceID = string;
export type PatientID = string;

// ============================================================================
// Device Types
// ============================================================================

export type DeviceType = 'smartwatch' | 'fitness_tracker' | 'smart_ring' | 'patch' | 'ecg_monitor' | 'pulse_oximeter' | 'blood_pressure_monitor' | 'cgm' | 'other';

export interface WearableDevice {
  device_id: DeviceID;
  device_type: DeviceType;
  manufacturer: string;
  model: string;
  firmware_version?: string;
  battery_level?: number;
  last_sync: Timestamp;
  status: DeviceStatus;
  capabilities: DeviceCapabilities;
}

export enum DeviceStatus {
  ACTIVE = 'active',
  INACTIVE = 'inactive',
  LOW_BATTERY = 'low_battery',
  DISCONNECTED = 'disconnected',
  ERROR = 'error',
  UPDATING = 'updating',
}

export interface DeviceCapabilities {
  heartRate: boolean;
  ecg: boolean;
  spo2: boolean;
  bloodPressure: boolean;
  temperature: boolean;
  glucose: boolean;
  activity: boolean;
  sleep: boolean;
  stress: boolean;
  fallDetection: boolean;
}

// ============================================================================
// Health Metrics Types
// ============================================================================

export interface HealthMetrics {
  timestamp: Timestamp;
  device_id: DeviceID;
  patient_id?: PatientID;
  heart_rate_bpm?: number;
  steps?: number;
  distance_meters?: number;
  calories_burned?: number;
  spo2_percent?: number;
  skin_temperature_celsius?: number;
  blood_pressure?: BloodPressure;
  sleep_data?: SleepData;
  stress_level?: number;
  hrv_ms?: number;
  respiratory_rate?: number;
  ecg_data?: ECGData;
  glucose_data?: GlucoseData;
}

export interface BloodPressure {
  systolic: number;
  diastolic: number;
  pulse?: number;
  timestamp: Timestamp;
  position?: 'sitting' | 'standing' | 'lying';
}

export interface SleepData {
  duration_minutes: number;
  deep_sleep_minutes: number;
  light_sleep_minutes: number;
  rem_sleep_minutes: number;
  awake_minutes: number;
  sleep_score?: number;
  bedtime: Timestamp;
  wake_time: Timestamp;
  interruptions: number;
}

export interface ECGData {
  samples: number[];
  sampling_rate: number;
  duration_seconds: number;
  lead_type: string;
  classification?: ECGClassification;
  heart_rate: number;
}

export enum ECGClassification {
  NORMAL = 'normal',
  AFIB = 'atrial_fibrillation',
  BRADYCARDIA = 'bradycardia',
  TACHYCARDIA = 'tachycardia',
  IRREGULAR = 'irregular',
  INCONCLUSIVE = 'inconclusive',
}

export interface GlucoseData {
  value: number;
  unit: 'mg/dL' | 'mmol/L';
  trend: GlucoseTrend;
  timestamp: Timestamp;
  meal_tag?: 'fasting' | 'before_meal' | 'after_meal' | 'bedtime';
}

export enum GlucoseTrend {
  RISING_RAPIDLY = 'rising_rapidly',
  RISING = 'rising',
  STABLE = 'stable',
  FALLING = 'falling',
  FALLING_RAPIDLY = 'falling_rapidly',
}

// ============================================================================
// Emergency Types
// ============================================================================

export interface EmergencyAlert {
  alert_id: string;
  patient_id: PatientID;
  device_id: DeviceID;
  alert_type: EmergencyAlertType;
  severity: AlertSeverity;
  timestamp: Timestamp;
  location?: GeoLocation;
  vital_signs: VitalSigns;
  status: AlertStatus;
  responders?: Responder[];
  resolution?: AlertResolution;
}

export enum EmergencyAlertType {
  CARDIAC_ARREST = 'cardiac_arrest',
  FALL_DETECTED = 'fall_detected',
  ABNORMAL_HEART_RATE = 'abnormal_heart_rate',
  LOW_SPO2 = 'low_spo2',
  HIGH_BLOOD_PRESSURE = 'high_blood_pressure',
  LOW_BLOOD_PRESSURE = 'low_blood_pressure',
  HYPOGLYCEMIA = 'hypoglycemia',
  HYPERGLYCEMIA = 'hyperglycemia',
  AFIB_DETECTED = 'afib_detected',
  MANUAL_SOS = 'manual_sos',
}

export enum AlertSeverity {
  LOW = 'low',
  MEDIUM = 'medium',
  HIGH = 'high',
  CRITICAL = 'critical',
}

export enum AlertStatus {
  TRIGGERED = 'triggered',
  ACKNOWLEDGED = 'acknowledged',
  DISPATCHED = 'dispatched',
  RESOLVED = 'resolved',
  FALSE_ALARM = 'false_alarm',
  CANCELLED = 'cancelled',
}

export interface VitalSigns {
  heart_rate?: number;
  spo2?: number;
  blood_pressure?: BloodPressure;
  temperature?: number;
  respiratory_rate?: number;
  consciousness_level?: string;
}

export interface GeoLocation {
  latitude: number;
  longitude: number;
  accuracy_meters?: number;
  altitude?: number;
  address?: string;
  timestamp: Timestamp;
}

export interface Responder {
  responder_id: string;
  type: ResponderType;
  name: string;
  eta_minutes?: number;
  status: ResponderStatus;
  contact?: string;
}

export enum ResponderType {
  AMBULANCE = 'ambulance',
  PARAMEDIC = 'paramedic',
  FIRE = 'fire',
  POLICE = 'police',
  FAMILY = 'family',
  NEIGHBOR = 'neighbor',
}

export enum ResponderStatus {
  NOTIFIED = 'notified',
  EN_ROUTE = 'en_route',
  ON_SCENE = 'on_scene',
  TRANSPORTING = 'transporting',
  COMPLETED = 'completed',
}

export interface AlertResolution {
  resolved_at: Timestamp;
  resolved_by: string;
  outcome: string;
  notes?: string;
  follow_up_required: boolean;
}

// ============================================================================
// Patient Types
// ============================================================================

export interface PatientProfile {
  patient_id: PatientID;
  name: string;
  date_of_birth: string;
  gender: 'male' | 'female' | 'other';
  blood_type?: string;
  allergies: string[];
  medical_conditions: MedicalCondition[];
  medications: Medication[];
  emergency_contacts: EmergencyContact[];
  devices: DeviceID[];
  insurance?: InsuranceInfo;
}

export interface MedicalCondition {
  condition: string;
  diagnosed_date?: string;
  severity?: string;
  notes?: string;
}

export interface Medication {
  name: string;
  dosage: string;
  frequency: string;
  start_date?: string;
  end_date?: string;
  prescriber?: string;
}

export interface EmergencyContact {
  name: string;
  relationship: string;
  phone: string;
  email?: string;
  is_primary: boolean;
  notify_on_alert: boolean;
}

export interface InsuranceInfo {
  provider: string;
  policy_number: string;
  group_number?: string;
  valid_until: string;
}

// ============================================================================
// Analytics Types
// ============================================================================

export interface HealthTrend {
  patient_id: PatientID;
  metric: string;
  period: TrendPeriod;
  data_points: DataPoint[];
  statistics: TrendStatistics;
}

export type TrendPeriod = 'day' | 'week' | 'month' | 'quarter' | 'year';

export interface DataPoint {
  timestamp: Timestamp;
  value: number;
  quality?: number;
}

export interface TrendStatistics {
  min: number;
  max: number;
  average: number;
  median: number;
  std_deviation: number;
  trend_direction: 'improving' | 'stable' | 'declining';
}

// ============================================================================
// API Types
// ============================================================================

export interface APIResponse<T = unknown> {
  status: number;
  success: boolean;
  data?: T;
  error?: APIError;
  timestamp: Timestamp;
  request_id: string;
}

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
}

export interface PaginationParams {
  page: number;
  page_size: number;
  sort_by?: string;
  sort_order?: 'asc' | 'desc';
}

export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  page_size: number;
  total_pages: number;
}

export interface SDKConfig {
  api_key: string;
  base_url?: string;
  timeout_ms?: number;
  retry_attempts?: number;
  debug?: boolean;
}
