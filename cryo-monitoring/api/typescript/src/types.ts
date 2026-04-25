/**
 * WIA CRYO-008 Cryo Monitoring Standard - TypeScript Type Definitions
 *
 * @module @wia/cryo-monitoring
 * @version 1.0.0
 * @license MIT
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

export enum SensorType {
  TEMPERATURE = 'temperature',
  PRESSURE = 'pressure',
  OXYGEN = 'oxygen',
  NITROGEN = 'nitrogen',
  EEG = 'eeg',
  ECG = 'ecg',
  VIBRATION = 'vibration',
  HUMIDITY = 'humidity'
}

export enum SensorStatus {
  OK = 'ok',
  WARNING = 'warning',
  CRITICAL = 'critical',
  ERROR = 'error',
  OFFLINE = 'offline',
  CALIBRATING = 'calibrating'
}

export enum AlertSeverity {
  INFO = 'info',
  WARNING = 'warning',
  URGENT = 'urgent',
  CRITICAL = 'critical'
}

export interface SensorReading {
  sensorId: string;
  type: SensorType;
  value: number;
  unit: string;
  location: string;
  status: SensorStatus;
  confidence: number;
  calibrationDate: string;
  nextCalibration: string;
}

export interface Alert {
  alertId: string;
  severity: AlertSeverity;
  type: string;
  message: string;
  triggeredAt: string;
  acknowledgedAt: string | null;
  acknowledgedBy: string | null;
  relatedSensors: string[];
}

export interface Metadata {
  version: string;
  checksum: string;
  createdBy: string;
  systemId: string;
  softwareVersion: string;
}

export interface MonitoringRecord {
  monitoringId: string;
  subjectId: string;
  timestamp: string;
  facilityId: string;
  sensors: SensorReading[];
  alerts: Alert[];
  metadata: Metadata;
  signature: string;
}

export interface MonitoringQuery {
  start?: string;
  end?: string;
  limit?: number;
  offset?: number;
  status?: SensorStatus;
  severity?: AlertSeverity;
}

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: string;
  errorCode?: string;
  total?: number;
}

export interface MonitoringConfig {
  apiUrl: string;
  token: string;
  facilityId: string;
  enableStreaming?: boolean;
  mqttUrl?: string;
  wsUrl?: string;
  timeout?: number;
  debug?: boolean;
}

export interface StreamingEvent {
  type: 'monitoring_update' | 'alert' | 'sensor_status' | 'connection_status';
  payload: any;
  timestamp: string;
}

export interface ValidationResult {
  valid: boolean;
  errors: string[];
  warnings: string[];
}
