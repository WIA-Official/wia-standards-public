/**
 * WIA-SEMI-012 Sensor Technology Standard
 * TypeScript Type Definitions
 */

// Base Sensor Interface
export interface ISensor {
  id: string;
  name: string;
  type: SensorType;
  calibrate(): Promise<void>;
  read(): Promise<SensorReading>;
  getStatus(): SensorStatus;
}

export enum SensorType {
  ACCELEROMETER = 'accelerometer',
  GYROSCOPE = 'gyroscope',
  MAGNETOMETER = 'magnetometer',
  PRESSURE = 'pressure',
  TEMPERATURE = 'temperature',
  HUMIDITY = 'humidity',
  GAS = 'gas',
  IMAGE = 'image',
  LIGHT = 'light',
  PROXIMITY = 'proximity',
}

export interface SensorReading {
  timestamp: number;
  value: number | number[] | ImageData;
  unit: string;
  quality: DataQuality;
}

export enum DataQuality {
  GOOD = 'good',
  POOR = 'poor',
  INVALID = 'invalid',
}

export interface SensorStatus {
  isCalibrated: boolean;
  lastCalibration?: Date;
  temperature: number;
  powerMode: PowerMode;
  errors: string[];
}

export enum PowerMode {
  ACTIVE = 'active',
  LOW_POWER = 'low_power',
  SLEEP = 'sleep',
  SHUTDOWN = 'shutdown',
}

// MEMS Sensors
export interface IAccelerometer extends ISensor {
  type: SensorType.ACCELEROMETER;
  fullScaleRange: number; // ±g
  resolution: number; // bits
  read(): Promise<AccelerometerReading>;
}

export interface AccelerometerReading extends SensorReading {
  value: [number, number, number]; // [x, y, z] in g
  unit: 'g';
}

export interface IGyroscope extends ISensor {
  type: SensorType.GYROSCOPE;
  fullScaleRange: number; // ±dps
  resolution: number; // bits
  read(): Promise<GyroscopeReading>;
}

export interface GyroscopeReading extends SensorReading {
  value: [number, number, number]; // [x, y, z] in dps
  unit: 'dps';
}

export interface IMagnetometer extends ISensor {
  type: SensorType.MAGNETOMETER;
  fullScaleRange: number; // ±μT
  read(): Promise<MagnetometerReading>;
}

export interface MagnetometerReading extends SensorReading {
  value: [number, number, number]; // [x, y, z] in μT
  unit: 'μT';
}

// IMU (Inertial Measurement Unit)
export interface IIMU {
  accelerometer: IAccelerometer;
  gyroscope: IGyroscope;
  magnetometer?: IMagnetometer;

  getOrientation(): Promise<Quaternion>;
  getEulerAngles(): Promise<EulerAngles>;
  fuseSensors(): Promise<void>;
}

export interface Quaternion {
  w: number;
  x: number;
  y: number;
  z: number;
}

export interface EulerAngles {
  roll: number;  // degrees
  pitch: number; // degrees
  yaw: number;   // degrees (heading)
}

// Environmental Sensors
export interface ITemperatureSensor extends ISensor {
  type: SensorType.TEMPERATURE;
  read(): Promise<TemperatureReading>;
}

export interface TemperatureReading extends SensorReading {
  value: number; // °C
  unit: '°C' | '°F' | 'K';
}

export interface IHumiditySensor extends ISensor {
  type: SensorType.HUMIDITY;
  read(): Promise<HumidityReading>;
}

export interface HumidityReading extends SensorReading {
  value: number; // % RH
  unit: '%RH';
}

export interface IPressureSensor extends ISensor {
  type: SensorType.PRESSURE;
  read(): Promise<PressureReading>;
  getAltitude(seaLevelPressure: number): Promise<number>;
}

export interface PressureReading extends SensorReading {
  value: number; // hPa
  unit: 'hPa' | 'Pa' | 'mbar';
}

// Gas Sensors
export interface IGasSensor extends ISensor {
  type: SensorType.GAS;
  gasType: GasType;
  read(): Promise<GasReading>;
}

export enum GasType {
  CO = 'CO',
  CO2 = 'CO2',
  NO2 = 'NO2',
  VOC = 'VOC',
  O3 = 'O3',
  PM25 = 'PM2.5',
}

export interface GasReading extends SensorReading {
  value: number;
  unit: 'ppm' | 'ppb' | 'μg/m³';
  gasType: GasType;
}

// Image Sensor
export interface IImageSensor extends ISensor {
  type: SensorType.IMAGE;
  resolution: [number, number]; // [width, height]
  pixelSize: number; // μm
  read(): Promise<ImageReading>;
  capture(settings: CaptureSettings): Promise<ImageReading>;
}

export interface ImageReading extends SensorReading {
  value: ImageData;
  unit: 'pixels';
  metadata: ImageMetadata;
}

export interface CaptureSettings {
  exposureTime?: number; // ms
  iso?: number;
  whiteBalance?: WhiteBalanceMode;
  hdr?: boolean;
}

export enum WhiteBalanceMode {
  AUTO = 'auto',
  DAYLIGHT = 'daylight',
  CLOUDY = 'cloudy',
  TUNGSTEN = 'tungsten',
  FLUORESCENT = 'fluorescent',
}

export interface ImageMetadata {
  width: number;
  height: number;
  format: string;
  exposureTime: number;
  iso: number;
  timestamp: Date;
}

// Calibration
export interface CalibrationData {
  sensorId: string;
  date: Date;
  method: CalibrationMethod;
  coefficients: Record<string, number>;
  temperatureMap?: TemperatureCalibrationMap;
}

export enum CalibrationMethod {
  SIX_POSITION = 'six_position',
  MULTI_POINT = 'multi_point',
  FACTORY = 'factory',
  FIELD = 'field',
}

export interface TemperatureCalibrationMap {
  temperatures: number[]; // °C
  offsets: number[][];    // [temp_index][axis]
  scales: number[][];     // [temp_index][axis]
}

// Communication
export interface CommunicationConfig {
  interface: CommunicationType;
  address?: number; // I2C address
  speed?: number;   // Hz
}

export enum CommunicationType {
  I2C = 'i2c',
  SPI = 'spi',
  UART = 'uart',
  USB = 'usb',
}

// Error Handling
export class SensorError extends Error {
  constructor(
    message: string,
    public code: SensorErrorCode,
    public sensorId?: string
  ) {
    super(message);
    this.name = 'SensorError';
  }
}

export enum SensorErrorCode {
  COMMUNICATION_FAILED = 'COMMUNICATION_FAILED',
  CALIBRATION_REQUIRED = 'CALIBRATION_REQUIRED',
  OUT_OF_RANGE = 'OUT_OF_RANGE',
  SENSOR_NOT_FOUND = 'SENSOR_NOT_FOUND',
  INVALID_CONFIGURATION = 'INVALID_CONFIGURATION',
}
