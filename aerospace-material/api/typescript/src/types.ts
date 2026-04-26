/**
 * WIA Aerospace Material Standard - TypeScript Types
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 * © 2025 SmileStory Inc. / WIA
 */

export type Timestamp = string;

export type MaterialCategory = 'aluminum' | 'titanium' | 'composite' | 'ceramic' | 'superalloy' | 'steel' | 'magnesium';

export enum MaterialClass {
  STRUCTURAL = 'structural',
  THERMAL_PROTECTION = 'thermal_protection',
  ABLATIVE = 'ablative',
  COMPOSITE_MATRIX = 'composite_matrix',
  COATING = 'coating',
}

export interface MaterialProperties {
  density: number;
  tensileStrength: number;
  yieldStrength: number;
  elongation: number;
  hardness?: number;
  thermalConductivity?: number;
  meltingPoint?: number;
  specificHeat?: number;
  thermalExpansion?: number;
}

export interface MechanicalProperties {
  tensileStrength: number;
  yieldStrength: number;
  ultimateStrength: number;
  elongation: number;
  modulusOfElasticity: number;
  shearModulus: number;
  poissonsRatio: number;
  fatigueStrength?: number;
  hardness: HardnessValue;
}

export interface HardnessValue {
  value: number;
  scale: 'HB' | 'HRC' | 'HV' | 'HRB';
}

export interface ThermalProperties {
  meltingPoint: number;
  thermalConductivity: number;
  specificHeat: number;
  thermalExpansion: number;
  maxServiceTemperature: number;
}

export interface ChemicalProperties {
  composition: ElementComposition[];
  corrosionResistance: 'excellent' | 'good' | 'moderate' | 'poor';
  oxidationResistance: 'excellent' | 'good' | 'moderate' | 'poor';
}

export interface ElementComposition {
  element: string;
  minPercent: number;
  maxPercent: number;
}

export interface Material {
  id: string;
  name: string;
  category: MaterialCategory;
  class: MaterialClass;
  grade: string;
  specification: string;
  properties: MaterialProperties;
  mechanicalProperties?: MechanicalProperties;
  thermalProperties?: ThermalProperties;
  chemicalProperties?: ChemicalProperties;
  applications: Application[];
  certification?: MaterialCertification;
  supplier?: SupplierInfo;
  metadata?: Record<string, unknown>;
}

export interface Application {
  component: string;
  aircraftType?: string;
  requirements: string[];
}

export interface MaterialCertification {
  standard: string;
  authority: string;
  approved: boolean;
  validUntil?: Timestamp;
  certificationNumber: string;
  specifications: string[];
}

export interface SupplierInfo {
  supplierId: string;
  name: string;
  country: string;
  qualityRating: number;
}

export interface CompositeMaterial extends Material {
  fiberType: FiberType;
  matrixType: MatrixType;
  fiberVolumeFraction: number;
  layup: LayupConfiguration;
}

export type FiberType = 'carbon' | 'glass' | 'aramid' | 'basalt' | 'boron';
export type MatrixType = 'epoxy' | 'polyester' | 'vinylester' | 'phenolic' | 'bismaleimide';

export interface LayupConfiguration {
  sequence: string[];
  plyCount: number;
  plyThickness: number;
  totalThickness: number;
}

export interface MaterialTest {
  testId: string;
  materialId: string;
  testType: TestType;
  testMethod: string;
  testDate: Timestamp;
  laboratory: string;
  results: TestResult[];
  status: TestStatus;
}

export type TestType = 'tensile' | 'compression' | 'shear' | 'fatigue' | 'impact' | 'hardness' | 'chemical';

export interface TestResult {
  parameter: string;
  value: number;
  unit: string;
  specification: number;
  pass: boolean;
}

export enum TestStatus {
  SCHEDULED = 'scheduled',
  IN_PROGRESS = 'in_progress',
  COMPLETED = 'completed',
  FAILED = 'failed',
}

export interface QualityRecord {
  recordId: string;
  materialId: string;
  batchNumber: string;
  inspectionDate: Timestamp;
  inspector: string;
  findings: Finding[];
  disposition: Disposition;
}

export interface Finding {
  type: string;
  description: string;
  severity: 'minor' | 'major' | 'critical';
}

export enum Disposition {
  ACCEPT = 'accept',
  REJECT = 'reject',
  REWORK = 'rework',
  PENDING = 'pending',
}

export interface MaterialBatch {
  batchId: string;
  materialId: string;
  lotNumber: string;
  quantity: number;
  unit: string;
  manufactureDate: Timestamp;
  expirationDate?: Timestamp;
  certificates: Certificate[];
}

export interface Certificate {
  type: 'mill_test' | 'conformance' | 'quality';
  number: string;
  issuer: string;
  issueDate: Timestamp;
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
  error?: APIError;
  timestamp: Timestamp;
  requestId: string;
}

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
}

export interface PaginationParams {
  page: number;
  pageSize: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  pageSize: number;
  totalPages: number;
}
