/**
 * WIA-IND-003: Wearable Fashion Standard - Type Definitions
 *
 * @packageDocumentation
 * @module wia-ind-003
 */

/**
 * Wearable fashion category
 */
export enum WearableFashionCategory {
  SmartWatch = 'smart_watch',
  SmartRing = 'smart_ring',
  SmartEyewear = 'smart_eyewear',
  SmartJewelry = 'smart_jewelry',
  SmartBag = 'smart_bag',
  SmartShoes = 'smart_shoes',
  SmartHeadwear = 'smart_headwear',
  ConnectedClothing = 'connected_clothing'
}

/**
 * Device form factor
 */
export enum FormFactor {
  Ring = 'ring',
  Bracelet = 'bracelet',
  Necklace = 'necklace',
  Earring = 'earring',
  Brooch = 'brooch',
  Watch = 'watch',
  Glasses = 'glasses',
  Hairpin = 'hairpin'
}

/**
 * Material type for wearable
 */
export enum MaterialType {
  PreciousMetal = 'precious_metal',
  BaseMetal = 'base_metal',
  Ceramic = 'ceramic',
  Glass = 'glass',
  Polymer = 'polymer',
  Fabric = 'fabric',
  Leather = 'leather',
  Silicone = 'silicone'
}

/**
 * Design style
 */
export enum DesignStyle {
  Minimalist = 'minimalist',
  Classic = 'classic',
  Sporty = 'sporty',
  Luxury = 'luxury',
  Avant_garde = 'avant_garde',
  Casual = 'casual',
  Formal = 'formal'
}

/**
 * Physical dimensions
 */
export interface Dimensions {
  width: number;
  height: number;
  depth: number;
  unit: 'mm' | 'cm' | 'inch';
  weight: number;
  weightUnit: 'g' | 'oz';
}

/**
 * Display specifications
 */
export interface DisplaySpec {
  hasDisplay: boolean;
  type?: 'oled' | 'lcd' | 'e-ink' | 'led' | 'microled';
  resolution?: { width: number; height: number };
  size?: number;
  sizeUnit?: 'mm' | 'inch';
  touchEnabled?: boolean;
  alwaysOn?: boolean;
}

/**
 * Battery and power specs
 */
export interface PowerSpec {
  batteryCapacity: number;
  capacityUnit: 'mAh';
  batteryLife: number;
  batteryLifeUnit: 'hours' | 'days';
  chargingMethod: 'wireless' | 'magnetic' | 'usb-c' | 'proprietary';
  wirelessChargingStandard?: 'qi' | 'proprietary';
  fastCharging: boolean;
}

/**
 * Connectivity options
 */
export interface ConnectivitySpec {
  bluetooth: boolean;
  bluetoothVersion?: string;
  wifi: boolean;
  nfc: boolean;
  cellular: boolean;
  cellularGeneration?: '4g' | '5g';
  gps: boolean;
}

/**
 * Sensor array configuration
 */
export interface SensorArray {
  heartRate: boolean;
  spo2: boolean;
  temperature: boolean;
  accelerometer: boolean;
  gyroscope: boolean;
  barometer: boolean;
  ambient_light: boolean;
  proximity: boolean;
  uv: boolean;
  ecg: boolean;
}

/**
 * Customization options
 */
export interface CustomizationOptions {
  interchangeableBands: boolean;
  bandSizes?: string[];
  colorOptions: string[];
  engravingSupported: boolean;
  modularComponents: boolean;
  componentTypes?: string[];
}

/**
 * Health tracking features
 */
export interface HealthFeatures {
  stepCounting: boolean;
  sleepTracking: boolean;
  stressMonitoring: boolean;
  menstrualTracking: boolean;
  bloodOxygen: boolean;
  bloodPressure: boolean;
  calorieTracking: boolean;
  hydrationReminder: boolean;
}

/**
 * Smart features
 */
export interface SmartFeatures {
  notifications: boolean;
  callHandling: boolean;
  musicControl: boolean;
  voiceAssistant: boolean;
  contactlessPayment: boolean;
  emergencySOS: boolean;
  findMyDevice: boolean;
  hapticFeedback: boolean;
}

/**
 * Wearable fashion device specification
 */
export interface WearableFashionSpec {
  standard: 'WIA-IND-003';
  version: string;
  deviceId: string;
  name: string;
  category: WearableFashionCategory;
  formFactor: FormFactor;
  materials: MaterialType[];
  designStyle: DesignStyle;
  dimensions: Dimensions;
  display?: DisplaySpec;
  power: PowerSpec;
  connectivity: ConnectivitySpec;
  sensors: SensorArray;
  customization: CustomizationOptions;
  healthFeatures: HealthFeatures;
  smartFeatures: SmartFeatures;
  waterResistance?: { rating: string; depth: number };
  dustResistance?: string;
}

/**
 * Device configuration
 */
export interface DeviceConfig {
  deviceId: string;
  pairingCode?: string;
  autoSync: boolean;
  syncInterval: number;
  notifications: boolean;
  displaySettings?: {
    brightness: number;
    timeout: number;
    alwaysOn: boolean;
  };
}

/**
 * Activity data
 */
export interface ActivityData {
  timestamp: number;
  steps: number;
  distance: number;
  distanceUnit: 'km' | 'miles';
  caloriesBurned: number;
  activeMinutes: number;
  heartRateAvg?: number;
  heartRateMax?: number;
  heartRateMin?: number;
}

/**
 * Certification level
 */
export enum CertificationLevel {
  Bronze = 'bronze',
  Silver = 'silver',
  Gold = 'gold',
  Platinum = 'platinum'
}

/**
 * Compliance report
 */
export interface ComplianceReport {
  standard: 'WIA-IND-003';
  testDate: string;
  deviceId: string;
  certificationLevel: CertificationLevel;
  tests: { name: string; passed: boolean; details?: string }[];
  compliant: boolean;
}

/**
 * Event types
 */
export type WearableFashionEventType =
  | 'connected'
  | 'disconnected'
  | 'activity-update'
  | 'notification'
  | 'health-alert'
  | 'battery-low'
  | 'error';

/**
 * Event callback
 */
export type EventCallback<T = unknown> = (data: T) => void;
