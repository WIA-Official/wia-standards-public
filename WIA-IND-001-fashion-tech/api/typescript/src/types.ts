/**
 * WIA-IND-001: Fashion Technology Standard - Type Definitions
 *
 * @packageDocumentation
 * @module wia-ind-001
 */

/**
 * Fashion technology category
 */
export enum FashionTechCategory {
  /** Smart wearable fashion */
  SmartWearable = 'smart_wearable',
  /** Digital fashion and virtual clothing */
  DigitalFashion = 'digital_fashion',
  /** AI-powered styling */
  AIStyling = 'ai_styling',
  /** Virtual fitting rooms */
  VirtualFitting = 'virtual_fitting',
  /** Sustainable fashion tech */
  SustainableTech = 'sustainable_tech'
}

/**
 * Garment type classification
 */
export enum GarmentType {
  Top = 'top',
  Bottom = 'bottom',
  Dress = 'dress',
  Outerwear = 'outerwear',
  Footwear = 'footwear',
  Accessory = 'accessory',
  Jewelry = 'jewelry'
}

/**
 * Size standard system
 */
export enum SizeSystem {
  US = 'us',
  EU = 'eu',
  UK = 'uk',
  Asia = 'asia',
  Universal = 'universal'
}

/**
 * Body measurement data
 */
export interface BodyMeasurements {
  /** Height in centimeters */
  height: number;
  /** Chest circumference in cm */
  chest: number;
  /** Waist circumference in cm */
  waist: number;
  /** Hip circumference in cm */
  hips: number;
  /** Shoulder width in cm */
  shoulderWidth?: number;
  /** Arm length in cm */
  armLength?: number;
  /** Inseam length in cm */
  inseam?: number;
  /** Neck circumference in cm */
  neck?: number;
}

/**
 * Size recommendation result
 */
export interface SizeRecommendation {
  /** Recommended size */
  size: string;
  /** Size system used */
  system: SizeSystem;
  /** Confidence score (0-100) */
  confidence: number;
  /** Fit description */
  fitType: 'tight' | 'regular' | 'relaxed' | 'oversized';
  /** Alternative sizes to consider */
  alternatives?: string[];
}

/**
 * Fabric composition
 */
export interface FabricComposition {
  /** Material name */
  material: string;
  /** Percentage in composition */
  percentage: number;
  /** Recycled content indicator */
  recycled?: boolean;
  /** Organic certification */
  organic?: boolean;
}

/**
 * Smart textile sensor data
 */
export interface TextileSensorData {
  /** Sensor type */
  type: 'temperature' | 'humidity' | 'pressure' | 'motion' | 'biometric';
  /** Sensor value */
  value: number;
  /** Unit of measurement */
  unit: string;
  /** Timestamp */
  timestamp: number;
  /** Sensor location on garment */
  location?: string;
}

/**
 * Virtual garment representation
 */
export interface VirtualGarment {
  /** Unique garment ID */
  id: string;
  /** Garment name */
  name: string;
  /** Garment type */
  type: GarmentType;
  /** Brand name */
  brand: string;
  /** 3D model URL */
  model3dUrl: string;
  /** Texture URLs */
  textures: {
    diffuse: string;
    normal?: string;
    roughness?: string;
  };
  /** Available sizes */
  sizes: string[];
  /** Available colors */
  colors: string[];
  /** Price in USD */
  price?: number;
}

/**
 * Virtual fitting session configuration
 */
export interface FittingSessionConfig {
  /** User ID */
  userId: string;
  /** User body measurements */
  measurements: BodyMeasurements;
  /** Lighting preset */
  lighting?: 'studio' | 'outdoor' | 'evening' | 'custom';
  /** Background type */
  background?: 'neutral' | 'scene' | 'transparent';
  /** Camera angle */
  cameraAngle?: 'front' | 'side' | 'back' | '360';
}

/**
 * Style profile for AI recommendations
 */
export interface StyleProfile {
  /** Preferred styles */
  preferredStyles: string[];
  /** Disliked styles */
  dislikedStyles: string[];
  /** Color preferences */
  colorPreferences: string[];
  /** Occasion preferences */
  occasions: ('casual' | 'formal' | 'business' | 'sport' | 'evening')[];
  /** Budget range */
  budgetRange?: {
    min: number;
    max: number;
    currency: string;
  };
  /** Sustainability preference */
  sustainabilityFocus?: boolean;
}

/**
 * Outfit recommendation
 */
export interface OutfitRecommendation {
  /** Outfit ID */
  id: string;
  /** Outfit name */
  name: string;
  /** Items in outfit */
  items: VirtualGarment[];
  /** Occasion match */
  occasion: string;
  /** Style match score (0-100) */
  styleMatchScore: number;
  /** Total price */
  totalPrice?: number;
  /** Styling tips */
  stylingTips?: string[];
}

/**
 * Sustainability metrics
 */
export interface SustainabilityMetrics {
  /** Carbon footprint in kg CO2 */
  carbonFootprint: number;
  /** Water usage in liters */
  waterUsage: number;
  /** Recyclability score (0-100) */
  recyclability: number;
  /** Certifications */
  certifications: string[];
  /** Supply chain transparency score */
  transparencyScore: number;
}

/**
 * Fashion tech device configuration
 */
export interface DeviceConfig {
  /** Device ID */
  deviceId: string;
  /** Device type */
  deviceType: 'smart_mirror' | 'body_scanner' | 'smart_garment' | 'ar_display';
  /** Firmware version */
  firmwareVersion: string;
  /** Connectivity type */
  connectivity: 'bluetooth' | 'wifi' | 'nfc';
  /** Power mode */
  powerMode?: 'low' | 'normal' | 'high';
}

/**
 * Certification level for fashion tech compliance
 */
export enum CertificationLevel {
  Bronze = 'bronze',
  Silver = 'silver',
  Gold = 'gold',
  Platinum = 'platinum'
}

/**
 * Compliance report for fashion tech standards
 */
export interface ComplianceReport {
  /** Standard identifier */
  standard: 'WIA-IND-001';
  /** Version */
  version: string;
  /** Test date */
  testDate: string;
  /** Device or product tested */
  productId: string;
  /** Certification level achieved */
  certificationLevel: CertificationLevel;
  /** Individual test results */
  testResults: {
    testName: string;
    passed: boolean;
    details?: string;
  }[];
  /** Overall compliance status */
  compliant: boolean;
}

/**
 * Event types for fashion tech SDK
 */
export type FashionTechEventType =
  | 'sensor-data'
  | 'fitting-complete'
  | 'recommendation-ready'
  | 'device-connected'
  | 'device-disconnected'
  | 'error';

/**
 * Event callback type
 */
export type EventCallback<T = unknown> = (data: T) => void;
