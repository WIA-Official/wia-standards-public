/**
 * WIA Medical IoT Standard - TypeScript Type Definitions
 * Version: 1.0.0
 *
 * Standard for connected medical devices and IoT healthcare sensors
 */

export type DeviceType =
  | 'pulse_oximeter'
  | 'blood_pressure'
  | 'glucose_meter'
  | 'thermometer'
  | 'ecg'
  | 'weight_scale'
  | 'respiratory_monitor'
  | 'infusion_pump'
  | 'ventilator';

export type MeasurementUnit =
  | '%'           // SpO2, percentage
  | 'mmHg'        // Blood pressure
  | 'mg/dL'       // Glucose
  | 'mmol/L'      // Glucose (SI)
  | '°C'          // Temperature (Celsius)
  | '°F'          // Temperature (Fahrenheit)
  | 'bpm'         // Heart rate
  | 'kg'          // Weight
  | 'lbs'         // Weight (imperial)
  | 'mV'          // ECG voltage
  | 'L/min';      // Flow rate

export type DeviceStatus = 'online' | 'offline' | 'maintenance' | 'error';

export type AlertLevel = 'normal' | 'warning' | 'critical';

export interface Measurement {
  value: number;
  unit: MeasurementUnit;
  timestamp: string; // ISO 8601
  confidence?: number; // 0-1
}

export interface DeviceMetadata {
  firmware_version: string;
  hardware_version?: string;
  battery_level?: number; // 0-100
  signal_strength?: number; // dBm
  last_calibration?: string; // ISO 8601
  next_maintenance?: string; // ISO 8601
}

export interface Alert {
  alert_id: string;
  level: AlertLevel;
  message: string;
  timestamp: string; // ISO 8601
  acknowledged: boolean;
  resolved: boolean;
}

export interface MedicalIoTDevice {
  format: 'WIA-MEDICAL-IOT-v1.0';
  timestamp: string; // ISO 8601
  device_id: string;
  device_type: DeviceType;
  patient_id: string;
  measurement: Measurement;
  location?: string;
  metadata: DeviceMetadata;
  status?: DeviceStatus;
  alerts?: Alert[];
}

export interface VitalSigns {
  heart_rate?: Measurement;
  blood_pressure?: {
    systolic: Measurement;
    diastolic: Measurement;
  };
  spo2?: Measurement;
  temperature?: Measurement;
  respiratory_rate?: Measurement;
  weight?: Measurement;
  glucose?: Measurement;
}

export interface PatientReading {
  patient_id: string;
  timestamp: string; // ISO 8601
  vital_signs: VitalSigns;
  devices: string[]; // Array of device IDs
  location: string;
  notes?: string;
}

export interface DeviceRegistration {
  serial_number: string;
  device_type: DeviceType;
  manufacturer: string;
  model: string;
  registration_date: string; // ISO 8601
  certification?: {
    wia_certified: boolean;
    certification_id?: string;
    expiry_date?: string; // ISO 8601
  };
}

export interface DeviceConfiguration {
  device_id: string;
  sampling_rate?: number; // Hz
  alert_thresholds?: {
    [key: string]: {
      min?: number;
      max?: number;
    };
  };
  transmission_interval?: number; // seconds
  encryption_enabled: boolean;
}

export interface DeviceCommand {
  command_id: string;
  device_id: string;
  command_type: 'start' | 'stop' | 'calibrate' | 'configure' | 'reset';
  parameters?: Record<string, any>;
  timestamp: string; // ISO 8601
  executed: boolean;
  result?: {
    success: boolean;
    message: string;
    timestamp: string; // ISO 8601
  };
}

export interface DataStream {
  stream_id: string;
  device_id: string;
  start_time: string; // ISO 8601
  end_time?: string; // ISO 8601
  sample_rate: number; // Hz
  data_points: Array<{
    timestamp: string; // ISO 8601
    value: number;
    unit: MeasurementUnit;
  }>;
}

export interface IntegrationConfig {
  integration_id: string;
  source_device: string;
  destination_system: 'ehr' | 'hospital' | 'cloud' | 'mobile_app' | 'analytics';
  endpoint: string;
  protocol: 'mqtt' | 'coap' | 'http' | 'websocket';
  authentication: {
    method: 'api_key' | 'oauth2' | 'certificate';
    credentials: Record<string, string>;
  };
  data_mapping?: Record<string, string>;
}
