/**
 * WIA-MED-020: Wearable Health Monitoring Standard - TypeScript Type Definitions
 * 弘益人間 - Benefit All Humanity
 * @version 1.0.0
 * @license MIT
 */

export type Timestamp = string;
export type DeviceID = string;

export interface WearableDevice {
  device_id: DeviceID;
  device_type: 'smartwatch' | 'fitness_tracker' | 'smart_ring' | 'patch' | 'other';
  manufacturer: string;
  model: string;
  firmware_version?: string;
  battery_level?: number;
  last_sync: Timestamp;
}

export interface HealthMetrics {
  timestamp: Timestamp;
  heart_rate_bpm?: number;
  steps?: number;
  distance_meters?: number;
  calories_burned?: number;
  spo2_percent?: number;
  skin_temperature_celsius?: number;
  blood_pressure?: { systolic: number; diastolic: number; };
  sleep_data?: {
    duration_minutes: number;
    deep_sleep_minutes: number;
    light_sleep_minutes: number;
    rem_sleep_minutes: number;
    awake_minutes: number;
  };
  stress_level?: number;
  hrv_ms?: number;
}

export interface APIResponse<T = any> {
  status: number;
  success: boolean;
  data?: T;
  timestamp: Timestamp;
}
