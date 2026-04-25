/**
 * WIA-IND-004: Beauty Technology Standard - Type Definitions
 * @module wia-ind-004
 */

export enum BeautyTechCategory {
  SkinAnalysis = 'skin_analysis',
  HairCare = 'hair_care',
  NailTech = 'nail_tech',
  Makeup = 'makeup',
  AntiAging = 'anti_aging',
  Fragrance = 'fragrance',
  PersonalCare = 'personal_care'
}

export enum SkinType {
  Dry = 'dry', Oily = 'oily', Combination = 'combination',
  Sensitive = 'sensitive', Normal = 'normal'
}

export enum SkinConcern {
  Acne = 'acne', Wrinkles = 'wrinkles', DarkSpots = 'dark_spots',
  Pores = 'pores', Redness = 'redness', Dryness = 'dryness',
  Dullness = 'dullness', UnderEyeCircles = 'under_eye_circles'
}

export interface SkinAnalysisResult {
  timestamp: number;
  overallScore: number;
  skinType: SkinType;
  concerns: { concern: SkinConcern; severity: number }[];
  hydration: number;
  oiliness: number;
  elasticity: number;
  poreSize: number;
  wrinkleDepth: number;
  pigmentation: number;
  uvDamage: number;
  skinAge: number;
  recommendations: ProductRecommendation[];
}

export interface ProductRecommendation {
  productType: string;
  ingredients: string[];
  priority: 'high' | 'medium' | 'low';
  reason: string;
  usage: { frequency: string; timing: string };
}

export interface BeautyRoutine {
  id: string;
  name: string;
  steps: RoutineStep[];
  frequency: 'daily' | 'weekly' | 'monthly';
  time: 'morning' | 'evening' | 'both';
  duration: number;
}

export interface RoutineStep {
  order: number;
  productType: string;
  productName?: string;
  duration: number;
  technique: string;
  notes?: string;
}

export interface DeviceSpec {
  standard: 'WIA-IND-004';
  version: string;
  deviceId: string;
  category: BeautyTechCategory;
  sensors: string[];
  treatments: string[];
  connectivity: 'bluetooth' | 'wifi' | 'usb';
  batteryLife: number;
  waterproof: boolean;
}

export interface TreatmentSession {
  sessionId: string;
  deviceId: string;
  treatmentType: string;
  startTime: number;
  endTime?: number;
  intensity: number;
  area: string;
  notes?: string;
  effectiveness?: number;
}

export interface IngredientInfo {
  name: string;
  inci: string;
  category: string;
  benefits: string[];
  concerns: string[];
  safetyRating: number;
  comedogenic: number;
  irritancy: number;
}

export interface ProductAnalysis {
  productName: string;
  brand?: string;
  ingredients: IngredientInfo[];
  overallRating: number;
  suitability: { skinType: SkinType; score: number }[];
  concerns: string[];
  benefits: string[];
}

export enum CertificationLevel {
  Bronze = 'bronze', Silver = 'silver', Gold = 'gold', Platinum = 'platinum'
}

export interface ComplianceReport {
  standard: 'WIA-IND-004';
  testDate: string;
  deviceId: string;
  certificationLevel: CertificationLevel;
  tests: { name: string; passed: boolean; details?: string }[];
  compliant: boolean;
}

export type BeautyTechEventType = 'analysis-complete' | 'treatment-start' | 'treatment-end' | 'device-connected' | 'error';
export type EventCallback<T = unknown> = (data: T) => void;
