/**
 * WIA Aircraft Component Standard - TypeScript Types
 * Version: 1.0
 * Philosophy: 弘益人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

export type ComponentType =
  | 'fuselage'
  | 'wing'
  | 'tail'
  | 'landing-gear'
  | 'flight-control'
  | 'hydraulic-system'
  | 'fuel-system'
  | 'avionics'
  | 'engine-mount';

export type ComponentStatus =
  | 'design'
  | 'testing'
  | 'certified'
  | 'production'
  | 'maintenance'
  | 'retired';

export type MaterialType =
  | 'aluminum-2024'
  | 'aluminum-7075'
  | 'titanium-ti6al4v'
  | 'carbon-fiber'
  | 'fiberglass'
  | 'aramid'
  | 'composite';

export interface Dimensions {
  length: number;  // mm
  width: number;   // mm
  height: number;  // mm
  weight: number;  // kg
}

export interface MaterialSpecification {
  type: MaterialType;
  grade: string;
  strength: number;        // MPa
  fatigueLimit: number;    // MPa
  density: number;         // kg/m³
  corrosionResistance: 'low' | 'medium' | 'high';
}

export interface StructuralAnalysis {
  appliedLoad: number;       // kN
  calculatedStress: number;  // MPa
  safetyFactor: number;
  fatigueLife: number;       // cycles
  analysisDate: string;
  analyst: string;
}

export interface Certification {
  standard: string;           // e.g., "WIA-SPACE-016"
  authority: string;
  certificateNumber: string;
  issueDate: string;
  expiryDate: string;
  compliance: boolean;
}

export interface AircraftComponent {
  id: string;
  type: ComponentType;
  status: ComponentStatus;
  partNumber: string;
  serialNumber: string;
  manufacturer: string;
  manufacturingDate: string;
  dimensions: Dimensions;
  material: MaterialSpecification;
  structuralAnalysis?: StructuralAnalysis;
  certification?: Certification;
  inspectionHistory: Inspection[];
  metadata?: Record<string, any>;
}

export interface Inspection {
  id: string;
  inspectionDate: string;
  inspector: string;
  method: 'visual' | 'ultrasonic' | 'radiographic' | 'magnetic-particle' | 'eddy-current';
  result: 'pass' | 'fail' | 'conditional';
  findings?: string;
  nextInspectionDate?: string;
}

export interface WIAConfig {
  apiKey: string;
  baseURL?: string;
  timeout?: number;
  debug?: boolean;
}

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
  };
}

export interface ComponentSearchQuery {
  type?: ComponentType;
  status?: ComponentStatus;
  manufacturer?: string;
  certificationRequired?: boolean;
  materialType?: MaterialType;
}
