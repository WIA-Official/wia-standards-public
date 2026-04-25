/**
 * WIA-SEMI-015 Smart Sensor TypeScript Type Definitions
 *
 * @module @wia/smart-sensor/types
 */

/**
 * Sensor types supported by WIA-SEMI-015
 */
export enum SensorType {
  IMU = 'imu',
  ACCELEROMETER = 'accelerometer',
  GYROSCOPE = 'gyroscope',
  MAGNETOMETER = 'magnetometer',
  TEMPERATURE = 'temperature',
  HUMIDITY = 'humidity',
  PRESSURE = 'pressure',
  LIGHT = 'light',
  PROXIMITY = 'proximity',
  GAS = 'gas',
  MICROPHONE = 'microphone',
  CAMERA = 'camera',
}

/**
 * Sensor data reading
 */
export interface SensorReading {
  timestamp: number;
  sensorId: string;
  sensorType: SensorType;
  value: number | number[] | SensorVector3D;
  unit?: string;
  accuracy?: number;
}

/**
 * 3D vector for motion sensors
 */
export interface SensorVector3D {
  x: number;
  y: number;
  z: number;
}

/**
 * Sensor configuration
 */
export interface SensorConfig {
  sensorId: string;
  sensorType: SensorType;
  samplingRate: number; // Hz
  resolution?: number; // bits
  range?: number | [number, number];
  powerMode?: 'low' | 'normal' | 'high';
  enableInterrupt?: boolean;
}

/**
 * ML model metadata
 */
export interface MLModel {
  modelId: string;
  name: string;
  version: string;
  inputShape: number[];
  outputShape: number[];
  quantization: 'float32' | 'float16' | 'int8' | 'int16';
  framework: 'tflite' | 'onnx' | 'pytorch' | 'custom';
  modelSize: number; // bytes
  inferenceTime?: number; // ms
}

/**
 * ML inference result
 */
export interface InferenceResult {
  modelId: string;
  timestamp: number;
  predictions: Prediction[];
  inferenceTime: number; // ms
  confidence: number;
}

/**
 * Single prediction from ML model
 */
export interface Prediction {
  label: string;
  score: number;
  bbox?: BoundingBox;
}

/**
 * Bounding box for object detection
 */
export interface BoundingBox {
  x: number;
  y: number;
  width: number;
  height: number;
}

/**
 * Wireless protocol types
 */
export enum WirelessProtocol {
  BLE = 'ble',
  LORAWAN = 'lorawan',
  NBIOT = 'nbiot',
  WIFI = 'wifi',
  ZIGBEE = 'zigbee',
  THREAD = 'thread',
}

/**
 * Communication configuration
 */
export interface CommConfig {
  protocol: WirelessProtocol;
  deviceId: string;
  encryption: boolean;
  qos?: 0 | 1 | 2;
  keepAlive?: number; // seconds
}

/**
 * Device power state
 */
export enum PowerState {
  ACTIVE = 'active',
  IDLE = 'idle',
  SLEEP = 'sleep',
  DEEP_SLEEP = 'deep_sleep',
  SHUTDOWN = 'shutdown',
}

/**
 * Power metrics
 */
export interface PowerMetrics {
  state: PowerState;
  voltage: number; // V
  current: number; // A
  power: number; // W
  batteryLevel: number; // percentage
  estimatedRuntime?: number; // seconds
}

/**
 * Device status
 */
export interface DeviceStatus {
  deviceId: string;
  online: boolean;
  lastSeen: number; // timestamp
  firmwareVersion: string;
  powerMetrics: PowerMetrics;
  sensorStatus: Map<string, SensorHealth>;
}

/**
 * Sensor health status
 */
export enum SensorHealth {
  HEALTHY = 'healthy',
  DEGRADED = 'degraded',
  FAILED = 'failed',
  UNKNOWN = 'unknown',
}

/**
 * OTA update status
 */
export enum OTAStatus {
  IDLE = 'idle',
  DOWNLOADING = 'downloading',
  VERIFYING = 'verifying',
  INSTALLING = 'installing',
  COMPLETE = 'complete',
  FAILED = 'failed',
  ROLLED_BACK = 'rolled_back',
}

/**
 * OTA update information
 */
export interface OTAUpdate {
  version: string;
  size: number; // bytes
  checksum: string;
  signature?: string;
  releaseNotes?: string;
  mandatory: boolean;
}

/**
 * Security configuration
 */
export interface SecurityConfig {
  secureBootEnabled: boolean;
  encryptionAlgorithm: 'aes128' | 'aes256' | 'chacha20';
  authenticationMethod: 'psk' | 'certificate' | 'jwt';
  keyStorage: 'software' | 'hardware' | 'trustzone';
}

/**
 * WIA-SEMI-015 compliance level
 */
export enum ComplianceLevel {
  LEVEL1_BASIC = 1,
  LEVEL2_STANDARD = 2,
  LEVEL3_ADVANCED = 3,
}

/**
 * Device capabilities
 */
export interface DeviceCapabilities {
  complianceLevel: ComplianceLevel;
  supportedSensors: SensorType[];
  supportedProtocols: WirelessProtocol[];
  mlSupport: boolean;
  maxModelSize: number; // bytes
  securityFeatures: string[];
  otaSupport: boolean;
}
