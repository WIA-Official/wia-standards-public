/**
 * WIA Food Safety Standard - TypeScript Types
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

export type Timestamp = string;

export type HazardType = 'BIOLOGICAL' | 'CHEMICAL' | 'PHYSICAL' | 'ALLERGEN' | 'RADIOLOGICAL';
export type TestStatus = 'PASS' | 'FAIL' | 'PENDING' | 'IN_PROGRESS';
export type RiskLevel = 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';

export interface FoodProduct {
  productId: string;
  productName: string;
  category: FoodCategory;
  batchNumber: string;
  productionDate: Timestamp;
  expiryDate: Timestamp;
  manufacturer: string;
  ingredients: Ingredient[];
  allergens: AllergenInfo[];
  nutritionalInfo?: NutritionalInfo;
  storageConditions: StorageConditions;
}

export enum FoodCategory {
  DAIRY = 'dairy',
  MEAT = 'meat',
  SEAFOOD = 'seafood',
  PRODUCE = 'produce',
  BAKERY = 'bakery',
  BEVERAGES = 'beverages',
  PROCESSED = 'processed',
  FROZEN = 'frozen',
}

export interface Ingredient {
  name: string;
  percentage?: number;
  origin?: string;
  organic: boolean;
  gmo: boolean;
}

export interface AllergenInfo {
  allergen: Allergen;
  present: boolean;
  mayContain: boolean;
}

export enum Allergen {
  MILK = 'milk',
  EGGS = 'eggs',
  FISH = 'fish',
  SHELLFISH = 'shellfish',
  TREE_NUTS = 'tree_nuts',
  PEANUTS = 'peanuts',
  WHEAT = 'wheat',
  SOY = 'soy',
  SESAME = 'sesame',
}

export interface NutritionalInfo {
  servingSize: string;
  calories: number;
  totalFat: number;
  saturatedFat: number;
  cholesterol: number;
  sodium: number;
  carbohydrates: number;
  fiber: number;
  sugars: number;
  protein: number;
}

export interface StorageConditions {
  temperatureMin: number;
  temperatureMax: number;
  humidity?: number;
  shelfLife: number;
  specialInstructions?: string[];
}

export interface Hazard {
  hazardId: string;
  hazardType: HazardType;
  description: string;
  riskLevel: RiskLevel;
  controlMeasures: ControlMeasure[];
  criticalLimits: CriticalLimit[];
}

export interface ControlMeasure {
  measureId: string;
  description: string;
  frequency: string;
  responsible: string;
}

export interface CriticalLimit {
  parameter: string;
  minValue?: number;
  maxValue?: number;
  unit: string;
}

export interface HACCPPlan {
  planId: string;
  facilityId: string;
  productLines: string[];
  criticalControlPoints: CriticalControlPoint[];
  version: string;
  effectiveDate: Timestamp;
  reviewDate: Timestamp;
}

export interface CriticalControlPoint {
  ccpId: string;
  name: string;
  hazards: string[];
  criticalLimits: CriticalLimit[];
  monitoringProcedure: string;
  correctiveAction: string;
  verification: string;
  records: string[];
}

export interface TemperatureLog {
  logId: string;
  equipmentId: string;
  timestamp: Timestamp;
  temperature: number;
  humidity?: number;
  status: 'normal' | 'warning' | 'critical';
  corrective_action?: string;
}

export interface TestResult {
  testId: string;
  productId: string;
  testType: TestType;
  testDate: Timestamp;
  laboratory: string;
  parameter: string;
  result: number;
  unit: string;
  limit: number;
  status: TestStatus;
  analyst?: string;
  method?: string;
}

export enum TestType {
  MICROBIOLOGICAL = 'microbiological',
  CHEMICAL = 'chemical',
  PHYSICAL = 'physical',
  SENSORY = 'sensory',
  NUTRITIONAL = 'nutritional',
}

export interface Inspection {
  inspectionId: string;
  facilityId: string;
  inspectionDate: Timestamp;
  inspectionType: InspectionType;
  inspector: string;
  findings: Finding[];
  corrective_actions: CorrectiveAction[];
  complianceScore: number;
  status: 'passed' | 'failed' | 'conditional';
}

export enum InspectionType {
  ROUTINE = 'routine',
  FOLLOW_UP = 'follow_up',
  COMPLAINT = 'complaint',
  CERTIFICATION = 'certification',
}

export interface Finding {
  findingId: string;
  category: string;
  severity: RiskLevel;
  description: string;
  evidence?: string;
}

export interface CorrectiveAction {
  actionId: string;
  findingId: string;
  description: string;
  deadline: Timestamp;
  status: 'pending' | 'in_progress' | 'completed' | 'verified';
}

export interface Certification {
  certificateId: string;
  certificateType: CertificationType;
  issuingBody: string;
  issueDate: Timestamp;
  expiryDate: Timestamp;
  scope: string;
  status: 'active' | 'suspended' | 'expired' | 'revoked';
}

export enum CertificationType {
  ISO_22000 = 'iso_22000',
  FSSC_22000 = 'fssc_22000',
  BRC = 'brc',
  SQF = 'sqf',
  IFS = 'ifs',
  HALAL = 'halal',
  KOSHER = 'kosher',
  ORGANIC = 'organic',
}

export interface IncidentReport {
  incidentId: string;
  timestamp: Timestamp;
  productId: string;
  description: string;
  severity: RiskLevel;
  affectedBatches: string[];
  rootCause?: string;
  corrective_actions: CorrectiveAction[];
  resolved: boolean;
  reportedTo?: string[];
}

export interface RecallNotice {
  recallId: string;
  productId: string;
  batchNumbers: string[];
  reason: string;
  riskLevel: RiskLevel;
  issueDate: Timestamp;
  distributionArea: string[];
  instructions: string;
  status: 'active' | 'completed' | 'cancelled';
}

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: ApiError;
  timestamp: Timestamp;
  requestId: string;
}

export interface ApiError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
}

export interface FoodSafetyQuery {
  productId?: string;
  batchNumber?: string;
  testStatus?: TestStatus;
  riskLevel?: RiskLevel;
  startDate?: Timestamp;
  endDate?: Timestamp;
  limit?: number;
  offset?: number;
}

export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  pageSize: number;
  totalPages: number;
}
